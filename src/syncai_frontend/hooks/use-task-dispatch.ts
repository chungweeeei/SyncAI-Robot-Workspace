"use client";

import * as React from "react";

import { useTaskTracker } from "@/hooks/use-task-tracker";
import {
  cancelTask,
  submitTask,
  type TaskStatus,
  type TaskStepRequest,
  type TaskStepState,
} from "@/lib/api/task";

export interface TaskDispatch {
  taskStatus: TaskStatus | null;
  /** A submitted task that has not reached a terminal state yet. */
  running: boolean;
  /** True while a submit / cancel request is in flight. */
  busy: boolean;
  /** Only true while the task id is still known — see the tracker's poll effect. */
  cancelable: boolean;
  error: string | null;
  /** Per-step state of the tracked task, keyed by step id. Empty before a dispatch. */
  stepStates: ReadonlyMap<string, TaskStepState>;
  /**
   * The run this hook is following, or null.
   *
   * Exposed so the console can tell a run it already has a readback and a Cancel
   * for from one it has only heard about through GET /api/v1/active_tasks —
   * without it, a task dispatched from this very tab would also be announced as
   * an unattended run in the banner above the library.
   */
  taskId: string | null;
  send: (steps: readonly TaskStepRequest[]) => Promise<void>;
  cancel: () => Promise<void>;
  clear: () => void;
}

/**
 * Dispatch an authored step list and follow it until it is terminal — the
 * multi-step sibling of useGoalTask / usePosture, composed the same way: this
 * hook owns `busy`, useTaskTracker owns everything about the submitted task.
 *
 * Two differences from those two are deliberate:
 *
 * `send` takes the steps as an argument rather than reading staged state. The
 * list lives in useStepDrafts and the conversion to wire shape is
 * lib/task/step.ts's job; threading the draft array through here would make this
 * hook a second owner of the list.
 *
 * `robotId` is nullable. GET /api/v1/robot/state 404s until localization is
 * valid, so `robot_id` — the prefix of every task id — is genuinely unavailable
 * on a freshly booted robot. Authoring has to keep working then, and the schedule
 * path has to keep working *entirely*, because a schedule id is operator-authored
 * and needs no robot id at all. That asymmetry is why only this hook is gated.
 */
export function useTaskDispatch(robotId: string | null): TaskDispatch {
  const [busy, setBusy] = React.useState(false);
  const task = useTaskTracker();

  const { track, setError, reset, taskId, steps } = task;

  const send = React.useCallback(
    async (requests: readonly TaskStepRequest[]) => {
      if (!robotId || !requests.length) return;
      setBusy(true);
      // reset() here rather than in track(): this is the one place that knows a
      // new task is starting, so the previous task's per-step readback goes now
      // instead of lingering under the new one's rows.
      reset();
      try {
        track(await submitTask(robotId, requests));
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      } finally {
        setBusy(false);
      }
    },
    [robotId, track, setError, reset],
  );

  const cancel = React.useCallback(async () => {
    if (!taskId) return;
    setBusy(true);
    try {
      // No optimistic CANCELED: this is a *request*, and whether the workflow
      // actually stopped is what the next poll answers.
      await cancelTask(taskId);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }, [taskId, setError]);

  /**
   * A Map rather than the array the tracker hands over, because a row looks
   * itself up by its derived step id. An index-based join would mis-attribute a
   * status the moment the array is shorter than the list — which it legitimately
   * is right after creation, before the workflow query has any state to report.
   */
  const stepStates = React.useMemo(
    () => new Map(steps.map((step) => [step.id, step])),
    [steps],
  );

  return {
    taskStatus: task.taskStatus,
    running: task.running,
    busy,
    cancelable: task.cancelable,
    error: task.error,
    stepStates,
    taskId,
    send,
    cancel,
    clear: reset,
  };
}
