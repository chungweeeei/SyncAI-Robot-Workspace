"use client";

import * as React from "react";

import {
  setMotionKey,
  setPolicyMode,
  type MotionKey,
  type PolicyMode,
} from "@/lib/api/robot";

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

export interface LocomotionControl {
  /** The controller commanded *in this session*, or null if none has been. */
  controller: Controller | null;
  /** The policy commanded in this session under the current controller. */
  policy: Policy | null;
  /** True while either request is in flight. */
  busy: boolean;
  error: string | null;
  selectController: (controller: Controller) => Promise<void>;
  selectPolicy: (policy: Policy) => Promise<void>;
}

/**
 * Controller (RL / MPC) and, under RL, the learned policy (PPO / HIMLOCO).
 *
 * **This state is commanded, never measured, and that is not fixable here.** The
 * driver does publish the controller's `MODE_STATE` telemetry on the `mode`
 * topic, but nothing in the stack subscribes to it and the backend exposes no
 * endpoint for it, so the console has no way to ask what is live. The same is
 * true of PostureControl next door, whose comment says so too. Consequences the
 * UI has to be honest about: nothing is selected until the operator selects it,
 * a reload forgets everything, and a segment lighting up means "we asked",
 * because the command is one-way UDP that nothing acknowledges.
 *
 * State is set **after** the request succeeds rather than optimistically (the
 * opposite of usePosture, whose lit button is a progress indicator for a task
 * that takes seconds). These calls either go out or fail immediately, and a
 * failed switch that left the wrong segment lit would be worse than a moment of
 * nothing.
 */
export function useLocomotion(): LocomotionControl {
  const [controller, setController] = React.useState<Controller | null>(null);
  const [policy, setPolicy] = React.useState<Policy | null>(null);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

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
      // Cleared rather than kept: a policy only means something under RL, so
      // showing one lit under MPC would be a claim about a controller that is
      // not running it. And coming back to RL, we do not know what policy the
      // gait controller still has loaded -- only what we asked for since.
      if (next === "MPC") setPolicy(null);
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
      setPolicy(next);
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, []);

  return { controller, policy, busy, error, selectController, selectPolicy };
}
