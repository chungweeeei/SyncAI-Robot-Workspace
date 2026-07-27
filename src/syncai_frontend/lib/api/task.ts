// Client for the backend task API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/task.py).
//
// Only the single-step MOVE case is wired here: the operator drags a goal on
// the 2D map and we submit it as a one-step Temporal workflow. Multi-step task
// authoring stays a backend/API concern for now.

import { apiUrl } from "@/lib/api/config";

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

/** A navigation goal in map coordinates; theta in degrees, CCW from +x. */
export interface GoalPose {
  x: number;
  y: number;
  theta: number;
}

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

/** FastAPI reports errors as {detail: string} (or a 422 validation array). */
async function errorDetail(res: Response): Promise<string> {
  try {
    const body = await res.json();
    if (typeof body?.detail === "string") return body.detail;
    if (body?.detail) return JSON.stringify(body.detail);
  } catch {
    /* non-JSON body (proxy error page); fall back to the status line */
  }
  return `${res.status} ${res.statusText}`;
}

/**
 * Submit a one-step MOVE task. The task id doubles as the Temporal workflow id,
 * so it must be unique per submission and is scoped by robot to match the
 * `robotNN-task-NNN` convention used elsewhere.
 */
export async function sendMoveTask(
  robotId: string,
  goal: GoalPose,
): Promise<string> {
  const timestamp = Math.floor(Date.now() / 1000);
  const id = `${robotId}-goal-${timestamp}`;

  const res = await fetch(apiUrl("/api/v1/tasks"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      id,
      timestamp,
      steps: [
        {
          id: "move",
          type: "MOVE",
          params: {
            x: goal.x,
            y: goal.y,
            theta: normalizeTheta(goal.theta),
          },
        },
      ],
    }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));

  const ack = (await res.json()) as TaskAckResponse;
  return ack.id;
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
