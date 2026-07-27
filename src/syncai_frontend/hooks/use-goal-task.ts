"use client";

import * as React from "react";

import {
  TERMINAL_TASK_STATUSES,
  cancelTask,
  fetchTaskState,
  normalizeTheta,
  sendMoveTask,
  type GoalPose,
  type TaskStatus,
} from "@/lib/api/task";

// The MOVE workflow reports through a Temporal query, which is only as fresh as
// the poll; 1 Hz matches the robot-state poll and is plenty for a nav goal.
const TASK_POLL_MS = 1000;

export interface GoalTask {
  /** True while the view should turn a drag into a goal (RViz "2D Nav Goal"). */
  goalMode: boolean;
  toggleGoalMode: () => void;
  /** Staged goal, not yet submitted. */
  goal: GoalPose | null;
  /** Called by the view when a drag produces a goal. */
  commitGoal: (goal: GoalPose) => void;
  taskStatus: TaskStatus | null;
  /** A submitted task that has not reached a terminal state yet. */
  running: boolean;
  /** True while a submit / cancel request is in flight. */
  busy: boolean;
  error: string | null;
  /** Only true while the task id is still known (see the poll effect). */
  cancelable: boolean;
  send: () => Promise<void>;
  cancel: () => Promise<void>;
  clear: () => void;
}

/**
 * The drag-a-goal state machine, shared by the 2D map and the 3D point-cloud
 * view: stage a goal, submit it as a one-step MOVE task, then track that task
 * until it is terminal.
 *
 * Staging (rather than firing on pointer-up) is deliberate -- the goal moves a
 * real robot, so the operator gets to read the coordinates and confirm.
 *
 * This lives in a hook rather than in either view because both views need the
 * identical flow: only the *picking* differs (grid pixels vs a ground-plane
 * raycast), and that stays in the respective canvas.
 */
export function useGoalTask(robotId: string): GoalTask {
  const [goalMode, setGoalMode] = React.useState(false);
  const [goal, setGoal] = React.useState<GoalPose | null>(null);
  const [taskId, setTaskId] = React.useState<string | null>(null);
  const [taskStatus, setTaskStatus] = React.useState<TaskStatus | null>(null);
  const [error, setError] = React.useState<string | null>(null);
  const [busy, setBusy] = React.useState(false);

  // Track the submitted task until it reaches a terminal state.
  React.useEffect(() => {
    if (!taskId) return;
    const abort = new AbortController();
    let active = true;

    const tick = async () => {
      try {
        const state = await fetchTaskState(taskId, abort.signal);
        if (!active) return;
        setTaskStatus(state.status);
        const failed = state.steps.find((step) => step.error_msg);
        setError(failed?.error_msg || null);
        // Terminal: drop the id so this effect tears the interval down. The
        // status (and the goal marker) stay on screen until the operator clears
        // them, so they can see where the robot was sent and how it ended.
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

  const running =
    taskStatus !== null && !TERMINAL_TASK_STATUSES.includes(taskStatus);

  const commitGoal = React.useCallback((next: GoalPose) => {
    setGoal({ ...next, theta: normalizeTheta(next.theta) });
    // Single-shot, like RViz's "2D Nav Goal": one drag, one goal.
    setGoalMode(false);
    setError(null);
  }, []);

  const toggleGoalMode = React.useCallback(() => setGoalMode((on) => !on), []);

  const send = React.useCallback(async () => {
    if (!goal) return;
    setBusy(true);
    setError(null);
    try {
      const id = await sendMoveTask(robotId, goal);
      setTaskId(id);
      setTaskStatus("PENDING");
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }, [goal, robotId]);

  const cancel = React.useCallback(async () => {
    if (!taskId) return;
    setBusy(true);
    try {
      await cancelTask(taskId);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }, [taskId]);

  const clear = React.useCallback(() => {
    setGoal(null);
    setTaskStatus(null);
    setError(null);
  }, []);

  return {
    goalMode,
    toggleGoalMode,
    goal,
    commitGoal,
    taskStatus,
    running,
    busy,
    error,
    cancelable: taskId !== null,
    send,
    cancel,
    clear,
  };
}
