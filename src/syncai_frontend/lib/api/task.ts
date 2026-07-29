// Client for the backend task API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/task.py).
//
// Only the single-step MOVE case is wired here: the operator drags a goal on
// the point-cloud viewport and we submit it as a one-step Temporal workflow.
// Multi-step task authoring stays a backend/API concern for now.

import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import type { PlanarPose } from "@/lib/types/robot";

export type TaskStatus =
  | "PENDING"
  | "IN_PROGRESS"
  | "COMPLETED"
  | "FAILED"
  | "CANCELED";

export const TERMINAL_TASK_STATUSES: readonly TaskStatus[] = [
  "COMPLETED",
  "FAILED",
  "CANCELED",
];

/**
 * A navigation goal in map coordinates. Structurally the same planar pose the
 * viewport produces for any drag (an initial pose is the other one), so it is an
 * alias rather than a second declaration — the name is what carries the meaning.
 */
export type GoalPose = PlanarPose;

export interface TaskStepState {
  id: string;
  status: TaskStatus;
  error_msg: string;
}

export interface TaskStateResponse {
  id: string;
  status: TaskStatus;
  steps: TaskStepState[];
}

interface TaskAckResponse {
  id: string;
  status: TaskStatus;
  message: string;
}

/**
 * MoveParams validates `gt=-180, le=180`, so exactly -180 is rejected: fold the
 * angle into (-180, 180] rather than the usual [-180, 180).
 */
export function normalizeTheta(deg: number): number {
  const wrapped = ((deg % 360) + 360) % 360; // [0, 360)
  return wrapped > 180 ? wrapped - 360 : wrapped;
}

/**
 * Submit a one-step task and return its id. The id doubles as the Temporal
 * workflow id, so it must be unique per submission and is scoped by robot to
 * match the `robotNN-task-NNN` convention used elsewhere. `kind` only shapes
 * that id — the step's `type` is what the workflow dispatches on.
 *
 * Second granularity is enough for uniqueness here because every submission
 * path in the console is a deliberate operator action, not a loop.
 */
async function submitOneStepTask(
  robotId: string,
  kind: string,
  step: { type: string; params?: unknown },
): Promise<string> {
  const timestamp = Math.floor(Date.now() / 1000);
  const id = `${robotId}-${kind}-${timestamp}`;

  const res = await fetch(apiUrl("/api/v1/tasks"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      id,
      timestamp,
      steps: [{ id: kind, ...step }],
    }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));

  const ack = (await res.json()) as TaskAckResponse;
  return ack.id;
}

/** Submit a one-step MOVE task for a dragged nav goal. */
export function sendMoveTask(robotId: string, goal: GoalPose): Promise<string> {
  return submitOneStepTask(robotId, "goal", {
    type: "MOVE",
    params: { x: goal.x, y: goal.y, theta: normalizeTheta(goal.theta) },
  });
}

/**
 * The two posture commands the backend exposes as step types. Each is a single
 * gait-controller motion key with nothing to parameterise, so the step carries
 * no params at all — sending one would be rejected at the request boundary.
 */
export type Posture = "STANDUP" | "LIEDOWN";

/** Submit a one-step posture task (STANDUP / LIEDOWN). */
export function sendPostureTask(
  robotId: string,
  posture: Posture,
): Promise<string> {
  return submitOneStepTask(robotId, posture.toLowerCase(), { type: posture });
}

export async function fetchTaskState(
  id: string,
  signal?: AbortSignal,
): Promise<TaskStateResponse> {
  const res = await fetch(apiUrl(`/api/v1/tasks/${encodeURIComponent(id)}`), {
    signal,
  });
  if (!res.ok) throw new Error(await errorDetail(res));
  return (await res.json()) as TaskStateResponse;
}

export async function cancelTask(id: string): Promise<void> {
  const res = await fetch(apiUrl(`/api/v1/tasks/${encodeURIComponent(id)}`), {
    method: "DELETE",
  });
  if (!res.ok) throw new Error(await errorDetail(res));
}
