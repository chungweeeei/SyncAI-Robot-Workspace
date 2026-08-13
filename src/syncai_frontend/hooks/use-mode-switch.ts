"use client";

import * as React from "react";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { switchRobotMode, type SwitchableMode } from "@/lib/api/mapping";
import type { RobotMode } from "@/lib/types/robot";

export interface ModeSwitch {
  /**
   * The mode the robot last reported, or null before the first state frame.
   * During a switch this is the *old* mode held by the query cache — the
   * backend is down and the poll is erroring — until the rebuilt stack's
   * robot_state answers with the new one.
   */
  reported: RobotMode | null;
  /** The state poll's health. "error" during a switch is the expected shape. */
  stateStatus: "loading" | "ok" | "error";
  /**
   * The mode a switch is in flight towards, or null. Derived, not stored: the
   * moment a state frame reports the requested mode this is null again, so it
   * resolves itself with no effect to clear it — the same shape as
   * useLocomotion's pendingPolicy.
   */
  pending: SwitchableMode | null;
  /** True while the POST itself is in flight (well before `pending` clears). */
  busy: boolean;
  error: string | null;
  switchTo: (mode: SwitchableMode) => Promise<void>;
}

/**
 * Command the operating mode and watch it land.
 *
 * **The confirmation channel is the robot state poll, not the POST.** A real
 * switch tears down the byobu session the backend runs in, so the POST's
 * connection usually just drops — and that is the switch *working*. The two
 * halves of this hook reflect that: `switchTo` records the request and
 * forgives the network error, and `pending` compares the request against what
 * `RobotState.mode` (fed by sys_manager's get_mode) actually reports, across
 * the 10–30 s hole while the stack rebuilds.
 *
 * Reads the console's shared 1 Hz poll via useConsoleRobotState rather than
 * running its own — the cache holding the last good frame through the outage
 * is also what keeps `reported` meaningful while the API is down.
 */
export function useModeSwitch(): ModeSwitch {
  const { state, status } = useConsoleRobotState();
  const [requested, setRequested] = React.useState<SwitchableMode | null>(null);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const reported = state?.mode ?? null;

  // Derived, not stored — see the interface doc. Also self-corrects the case
  // where the switch was requested from another console: whatever we asked for
  // stops pending the moment the robot reports it, however it got there.
  const pending = requested && requested !== reported ? requested : null;

  const switchTo = React.useCallback(
    async (next: SwitchableMode) => {
      setBusy(true);
      setError(null);
      // Recorded before the call, not after: the request usually kills its own
      // responder, so "the POST resolved" is not the moment the switch began.
      setRequested(next);
      try {
        await switchRobotMode(next);
        // A resolved POST is either the no-op (already in `next`, in which
        // case `pending` is already null) or a dispatch that answered inside
        // the backend's ack window. Nothing to do for either.
      } catch (cause) {
        if (cause instanceof TypeError) {
          // fetch's network-level failure: the connection dropped because the
          // switch is tearing the backend down. Expected; the state poll will
          // report the landing.
          return;
        }
        // An HTTP-level refusal (502 with sys_manager's reason, a 422): the
        // switch did not start, so it must not be shown as pending.
        setRequested(null);
        setError(cause instanceof Error ? cause.message : String(cause));
      } finally {
        setBusy(false);
      }
    },
    [],
  );

  return { reported, stateStatus: status, pending, busy, error, switchTo };
}
