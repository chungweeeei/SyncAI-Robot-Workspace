"use client";

import * as React from "react";

import {
  setMotionKey,
  setPolicyMode,
  type MotionKey,
  type PolicyMode,
} from "@/lib/api/robot";
import type { RobotLowLevelMode } from "@/lib/types/robot";

/** Which locomotion controller the gait controller should run. */
export type Controller = "RL" | "MPC";

/** Which learned policy the RL controller should load. */
export type Policy = "PPO" | "HIMLOCO";

/**
 * The motion keys that select a controller. Stand / lie down / damping are the
 * other three and are not this control's business — they are postures, and
 * PostureControl owns them.
 */
const CONTROLLER_KEY: Record<Controller, MotionKey> = {
  RL: "1",
  MPC: "5",
};

const POLICY_MODE: Record<Policy, PolicyMode> = {
  PPO: 0,
  HIMLOCO: 1,
};

/**
 * The reported policy labels this control can light a segment for.
 *
 * The backend can also report `CHAMP`, `ISSAC` or `UNKNOWN` — policies the
 * command surface deliberately does not expose. Those map to no segment, which is
 * honest: the robot is running something this control cannot ask for.
 */
const REPORTED_POLICY: Record<string, Policy> = {
  PPO: "PPO",
  HIMLOCO: "HIMLOCO",
};

export interface LocomotionControl {
  /** Commanded, defaulting to RL. Not readable back — see the note below. */
  controller: Controller;
  /** The policy to light: the reported one, or a pending request. */
  policy: Policy | null;
  /**
   * Set while a policy request has gone out and the robot has not yet reported
   * it. `policy` shows this value optimistically; the label says it is unconfirmed.
   */
  pendingPolicy: Policy | null;
  /** True while either request is in flight. */
  busy: boolean;
  error: string | null;
  selectController: (controller: Controller) => Promise<void>;
  selectPolicy: (policy: Policy) => Promise<void>;
}

/**
 * Controller (RL / MPC) and, under RL, the learned policy (PPO / HIMLOCO).
 *
 * **The two halves have different epistemics, and the UI has to reflect that.**
 *
 * `policy` is MEASURED: `RobotState.low_level_mode.policy` is the controller's own
 * answer, so the lit segment is what the robot says it is running rather than what
 * we asked for. A request that never takes effect stays visibly unconfirmed
 * instead of silently looking applied.
 *
 * `controller` is COMMANDED and defaults to `RL`, because it cannot be read back
 * at all: `low_level_mode.motion` reports STAND / LOCOMOTION / LIE_DOWN /
 * DAMPING / ESTOP, and **MPC has no known motion code** — this workspace added the
 * `MODE M` command without knowing what the controller reports for it. RL is the
 * default because it is the normal operating mode and the one a policy switch is
 * meaningful under; it is a starting assumption, not a reading.
 *
 * The pending window matters because the two sides run at different rates: the
 * POST returns as soon as a UDP datagram is written, while `robot_state` is polled
 * at 1 Hz. Without it, clicking HIMLOCO would leave PPO lit for up to a second and
 * then jump. `pendingPolicy` is DERIVED (request !== report) rather than cleared
 * by an effect, so it resolves itself the moment a frame confirms — and if no
 * frame ever does, it stays pending, which is the truth.
 */
export function useLocomotion(
  reported: RobotLowLevelMode | null,
): LocomotionControl {
  const [controller, setController] = React.useState<Controller>("RL");
  const [requestedPolicy, setRequestedPolicy] = React.useState<Policy | null>(null);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const reportedPolicy = reported
    ? REPORTED_POLICY[reported.policy] ?? null
    : null;

  // Derived, not stored: as soon as a robot_state frame agrees with what we asked
  // for, this is null again and `policy` falls through to the reported value.
  const pendingPolicy =
    requestedPolicy && requestedPolicy !== reportedPolicy ? requestedPolicy : null;

  const selectController = React.useCallback(async (next: Controller) => {
    setBusy(true);
    setError(null);
    try {
      const result = await setMotionKey(CONTROLLER_KEY[next]);
      // `sent` is always true for these two keys -- only "4" is refused -- but
      // honouring it means a future backend that declines something else cannot
      // leave this control claiming a switch that never left the process.
      if (!result.sent) {
        setError(result.message);
        return;
      }
      setController(next);
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, []);

  const selectPolicy = React.useCallback(async (next: Policy) => {
    setBusy(true);
    setError(null);
    try {
      await setPolicyMode(POLICY_MODE[next]);
      // Records the request only. What gets lit is still driven by the report;
      // this just keeps the click visible across the 1 Hz poll gap.
      setRequestedPolicy(next);
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, []);

  return {
    controller,
    policy: pendingPolicy ?? reportedPolicy,
    pendingPolicy,
    busy,
    error,
    selectController,
    selectPolicy,
  };
}
