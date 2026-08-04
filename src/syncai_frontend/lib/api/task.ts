// Client for the backend task API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/task.py).
//
// Two shapes of submission share one endpoint. The single-gesture ones — the
// operator drags a goal on the point-cloud viewport, or presses a posture
// button — go through `sendMoveTask` / `sendPostureTask` and are one step by
// construction. `submitTask` takes a list an operator authored on /tasks. Both
// end up in `postTask`, because a task *is* its step list and the difference is
// entirely in who assembled it.
//
// Creation is dispatch: the endpoint awaits Temporal's start_workflow before
// answering, so a 200 means the workflow is queued and the robot is about to
// act. There is no separate run/execute call, and no GET collection either —
// a caller that wants to follow a task has to keep the id this returns.

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";
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
 * The two posture commands the backend exposes as step types. Each is a single
 * gait-controller motion key with nothing to parameterise, so the step carries
 * no params at all — sending one would be rejected at the request boundary.
 */
export type Posture = "STANDUP" | "LIEDOWN";

/**
 * The step types the console can author.
 *
 * A deliberate subset of the backend's `StepType`: ARTIFACT is absent because
 * `ArtifactParams` is a discriminated command union (pickup / drop, robot / zone
 * / box indices, a `wait_for` conveyor phase and a timeout) aimed at hardware the
 * console has no other surface for. An ARTIFACT row would be a step the operator
 * cannot fill in.
 *
 * `Posture` is reused rather than restating "STANDUP" | "LIEDOWN", so the two
 * cannot drift.
 */
export type StepType = "MOVE" | Posture;

/** `MoveParams`, verbatim. `theta` in degrees, folded into (-180, 180] on the way out. */
export interface MoveStepParams {
  x: number;
  y: number;
  theta: number;
}

/**
 * One step as the endpoint takes it.
 *
 * A discriminated union, deliberately: this is the only thing that guarantees a
 * posture step never carries a `params` key. `StepRequest`'s model_validator
 * answers `params` on a STANDUP with a 422 whose `detail` is a validation
 * *array* rather than a sentence, so making it unrepresentable in TypeScript is
 * what keeps that off the operator's screen. (`params: undefined` would also be
 * dropped by JSON.stringify, but it is one careless refactor away from
 * `params: null`, which the MOVE branch rejects in turn.)
 */
export type TaskStepRequest =
  | { id: string; type: "MOVE"; params: MoveStepParams }
  | { id: string; type: Posture };

/**
 * Per-page-load submission sequence, appended to every task id.
 *
 * A task id *is* the Temporal workflow id, and a duplicate does not come back as
 * a 409: `client.start_workflow` raises, the gateway wraps it in
 * `InternalServerError`, and the operator is shown a 502 reading "Start workflow
 * failed" — the same sentence Temporal being unreachable produces. Second
 * granularity alone made that reachable, because nothing in the console rate
 * limits a second press: two dispatches inside one wall-clock second collide and
 * the operator is told the orchestrator is broken when it is not.
 *
 * Not a random suffix. This id is what an engineer greps for in the Temporal UI
 * and in the worker log, and `robot01-task-1782786519-3` is legible where a
 * base36 blob is not. Two page loads inside the same second would need the same
 * operator to reload and re-dispatch between two clock ticks.
 */
let submitSeq = 0;

/**
 * A fresh task identity. `kind` only shapes the id — the step's `type` is what
 * the workflow dispatches on. The `timestamp` field is required by `TaskRequest`
 * and read by nothing, so it reuses the value the id was built from and the two
 * can never disagree.
 */
function newTaskIdentity(
  robotId: string,
  kind: string,
): { id: string; timestamp: number } {
  const timestamp = Math.floor(Date.now() / 1000);
  submitSeq += 1;
  return { id: `${robotId}-${kind}-${timestamp}-${submitSeq}`, timestamp };
}

async function postTask(
  robotId: string,
  kind: string,
  steps: readonly TaskStepRequest[],
): Promise<string> {
  const { id, timestamp } = newTaskIdentity(robotId, kind);
  const ack = await requestJson<TaskAckResponse>(apiUrl("/api/v1/tasks"), {
    method: "POST",
    body: JSON.stringify({ id, timestamp, steps }),
  });
  return ack.id;
}

/** Submit a one-step MOVE task for a dragged nav goal. */
export function sendMoveTask(robotId: string, goal: GoalPose): Promise<string> {
  return postTask(robotId, "goal", [
    {
      id: "goal",
      type: "MOVE",
      params: { x: goal.x, y: goal.y, theta: normalizeTheta(goal.theta) },
    },
  ]);
}

/** Submit a one-step posture task (STANDUP / LIEDOWN). */
export function sendPostureTask(
  robotId: string,
  posture: Posture,
): Promise<string> {
  const kind = posture.toLowerCase();
  return postTask(robotId, kind, [{ id: kind, type: posture }]);
}

/**
 * Dispatch a task the operator authored step by step, and return its id for the
 * tracker. The steps arrive already in wire shape — see lib/task/step.ts, which
 * owns the draft model and the conversion, so this client never has to know what
 * a half-typed coordinate field looks like.
 */
export function submitTask(
  robotId: string,
  steps: readonly TaskStepRequest[],
): Promise<string> {
  return postTask(robotId, "task", steps);
}

export function fetchTaskState(
  id: string,
  signal?: AbortSignal,
): Promise<TaskStateResponse> {
  return requestJson<TaskStateResponse>(
    apiUrl(`/api/v1/tasks/${encodeURIComponent(id)}`),
    { signal },
  );
}

/**
 * Ask Temporal to cancel a task. The `{id, status, message}` envelope is
 * dropped: its `status` is a flat "CANCELED" the moment the request lands, which
 * is a claim about the *request* rather than the workflow. Whether the robot
 * actually stopped is what the next poll of `fetchTaskState` answers.
 */
export function cancelTask(id: string): Promise<void> {
  return requestJson<void>(apiUrl(`/api/v1/tasks/${encodeURIComponent(id)}`), {
    method: "DELETE",
    parse: false,
  });
}
