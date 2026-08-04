"use client";

import * as React from "react";

import {
  TERMINAL_TASK_STATUSES,
  fetchTaskState,
  type TaskStatus,
  type TaskStepState,
} from "@/lib/api/task";

// A workflow reports through a Temporal query, which is only as fresh as the
// poll; 1 Hz matches the robot-state poll and is plenty for either a nav goal
// (minutes) or a posture command (seconds).
const TASK_POLL_MS = 1000;

export interface TaskTracker {
  taskStatus: TaskStatus | null;
  /** A submitted task that has not reached a terminal state yet. */
  running: boolean;
  /** Only true while the task id is still known — see the poll effect. */
  cancelable: boolean;
  /** The error message of the first failed step, if any. */
  error: string | null;
  /**
   * Per-step state of the tracked task, as the workflow query last reported it.
   * Empty until the first poll answers with a populated list — a multi-step flow
   * joins it to its own rows by step id.
   */
  steps: TaskStepState[];
  /** Id of the task being polled, for callers that need to cancel it. */
  taskId: string | null;
  /** Start polling a freshly submitted task. */
  track: (id: string) => void;
  /** Surface a submit / cancel failure as this flow's error. */
  setError: (message: string | null) => void;
  /** Forget the tracked task and its outcome. */
  reset: () => void;
}

/**
 * Follows one submitted task until it is terminal.
 *
 * Shared by every one-step-task flow in the console (nav goal, posture) rather
 * than reimplemented per flow: the subtleties below — dropping the id at a
 * terminal status so the interval tears down while the *status* stays on
 * screen, and swallowing query errors instead of showing them — are the kind
 * that drift apart once there are two copies.
 */
export function useTaskTracker(): TaskTracker {
  const [taskId, setTaskId] = React.useState<string | null>(null);
  const [taskStatus, setTaskStatus] = React.useState<TaskStatus | null>(null);
  const [error, setError] = React.useState<string | null>(null);
  const [steps, setSteps] = React.useState<TaskStepState[]>([]);

  React.useEffect(() => {
    if (!taskId) return;
    const abort = new AbortController();
    let active = true;

    const tick = async () => {
      try {
        const state = await fetchTaskState(taskId, abort.signal);
        if (!active) return;
        setTaskStatus(state.status);
        // Never overwrite a known step list with an empty one. get_task_state
        // degrades `steps` to [] whenever the workflow query fails — in the
        // window before the workflow's first task executes, when no worker is
        // polling this robot's queue, and again once the execution has closed —
        // and each of those would otherwise blank the per-step readback the
        // operator is watching. An empty list is never meaningful here: the
        // composer refuses to dispatch a zero-step task.
        if (state.steps.length) setSteps(state.steps);
        const failed = state.steps.find((step) => step.error_msg);
        setError(failed?.error_msg || null);
        // Terminal: drop the id so this effect tears the interval down. The
        // status stays on screen until the caller clears it, so the operator
        // can see how the task ended.
        if (TERMINAL_TASK_STATUSES.includes(state.status)) setTaskId(null);
      } catch {
        // The workflow query fails while Temporal is starting the workflow, and
        // once the task ages out of Temporal's retention. Neither is worth
        // surfacing: keep the last known status.
      }
    };

    tick();
    const id = setInterval(tick, TASK_POLL_MS);
    return () => {
      active = false;
      abort.abort();
      clearInterval(id);
    };
  }, [taskId]);

  const track = React.useCallback((id: string) => {
    setTaskId(id);
    setTaskStatus("PENDING");
  }, []);

  // `track` deliberately does not clear `steps`: it is only ever called with a
  // fresh id, and the caller that knows a new task is starting calls reset()
  // first. Clearing in both places would be two owners of one responsibility.
  const reset = React.useCallback(() => {
    setTaskStatus(null);
    setError(null);
    setSteps([]);
  }, []);

  return {
    taskStatus,
    running:
      taskStatus !== null && !TERMINAL_TASK_STATUSES.includes(taskStatus),
    cancelable: taskId !== null,
    error,
    steps,
    taskId,
    track,
    setError,
    reset,
  };
}
