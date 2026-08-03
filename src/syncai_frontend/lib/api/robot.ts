// Client for the backend robot API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/robot.py).

import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import { normalizeTheta } from "@/lib/api/task";
import type { PlanarPose } from "@/lib/types/robot";

/**
 * Seed localization with an operator-supplied pose (RViz's "2D Pose Estimate").
 *
 * The backend publishes it on `initialpose`, which the FAST-LIO2 localizer takes
 * as an ICP initial guess. That is fire-and-forget on the ROS side: a 200 here
 * means the sample went out, not that localization converged on it — the answer
 * to "did it work" is the pose feed moving to where the operator put the marker.
 */
export async function setInitialPose(pose: PlanarPose): Promise<void> {
  const res = await fetch(apiUrl("/api/v1/robot/set_initial_pose"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      x: pose.x,
      y: pose.y,
      theta: normalizeTheta(pose.theta),
    }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));
}

/**
 * A gait-controller motion key, as the driver's numeric string.
 *
 * `"0"` stand, `"1"` locomotion (RL), `"2"` lie down, `"3"` damping,
 * `"4"` emergency stop, `"5"` MPC. Stand and lie down are not sent from here —
 * they go through the posture task (`sendPostureTask`), which is what owns
 * sequencing; this client exists for the two keys that select a *controller*.
 */
export type MotionKey = "0" | "1" | "2" | "3" | "4" | "5";

export interface SetMotionKeyResult {
  key: MotionKey;
  /**
   * Whether the key was forwarded to the driver.
   *
   * False only for `"4"`, which the backend accepts and deliberately does not
   * forward — no emergency stop was sent. True means the driver reported writing
   * the datagram, **not** that the robot moved: the command is one-way UDP with
   * no acknowledgement.
   */
  sent: boolean;
  message: string;
}

export async function setMotionKey(key: MotionKey): Promise<SetMotionKeyResult> {
  const res = await fetch(apiUrl("/api/v1/robot/set_motion_key"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ key }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));

  return (await res.json()) as SetMotionKeyResult;
}

/**
 * The gait controller's policy index: 0 PPO, 1 HIMLOCO.
 *
 * The backend accepts only these two of the four the controller defines (2 CHAMP
 * and 3 ISSAC are not exposed), so this union is the whole REST vocabulary —
 * anything else is a 422 whose `detail` is a validation *array*, not a sentence.
 * Keep the UI a fixed set of choices rather than free input and that cannot
 * happen.
 *
 * Not the robot mode in `GET /api/v1/robot/state` (MAINTENANCE / MANUAL / AUTO),
 * which is an unrelated thing also called "mode".
 */
export type PolicyMode = 0 | 1;

export interface SetPolicyModeResult {
  mode: PolicyMode;
  message: string;
}

/**
 * Switch the learned locomotion policy. Only meaningful under the RL controller.
 *
 * Same fire-and-forget caveat as the motion key, and the backend cannot check the
 * precondition for you: nothing in the stack reports which controller is live, and
 * the driver forwards the policy index without consulting the current mode. A 200
 * under MPC means the datagram went out and the controller did whatever it does
 * with it.
 */
export async function setPolicyMode(
  mode: PolicyMode,
): Promise<SetPolicyModeResult> {
  const res = await fetch(apiUrl("/api/v1/robot/set_policy_mode"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));

  return (await res.json()) as SetPolicyModeResult;
}
