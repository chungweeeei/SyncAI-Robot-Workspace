"use client";

import * as React from "react";

import { useTaskTracker } from "@/hooks/use-task-tracker";
import {
  cancelTask,
  normalizeTheta,
  sendMoveTask,
  type GoalPose,
  type TaskStatus,
} from "@/lib/api/task";

export interface GoalTask {
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
  /** Submit the staged goal. */
  send: () => Promise<void>;
  /**
   * Stage a goal and submit it in one call, for flows that confirmed the pose
   * before handing it over — double-clicking a stored vertex asks the same
   * question a `commitGoal` + read-back + Send does, so making the caller round
   * -trip through state to answer it twice would be theatre. It still stages,
   * so the marker and the read-back describe the task that is now running.
   */
  sendGoal: (goal: GoalPose) => Promise<void>;
  cancel: () => Promise<void>;
  clear: () => void;
}

/**
 * The drag-a-goal state machine: stage a goal, submit it as a one-step MOVE
 * task, then track that task until it is terminal.
 *
 * Staging (rather than firing on pointer-up) is deliberate -- the goal moves a
 * real robot, so the operator gets to read the coordinates and confirm.
 *
 * Which drag the viewport is currently collecting is NOT owned here: a goal and
 * an initial-pose estimate are two things one drag gesture can produce, and only
 * one of them can be armed at a time. The view owns that single pick mode (see
 * PointCloudView) and hands the finished pose to whichever flow asked for it.
 */
export function useGoalTask(robotId: string): GoalTask {
  const [goal, setGoal] = React.useState<GoalPose | null>(null);
  const [busy, setBusy] = React.useState(false);
  const task = useTaskTracker();

  const { track, setError, reset, taskId } = task;

  const commitGoal = React.useCallback(
    (next: GoalPose) => {
      setGoal({ ...next, theta: normalizeTheta(next.theta) });
      setError(null);
    },
    [setError],
  );

  const sendGoal = React.useCallback(
    async (next: GoalPose) => {
      // Normalised here as well as in commitGoal: this is the other door into
      // the same state, and the marker's heading has to mean the same thing
      // whichever one the pose came through.
      const staged = { ...next, theta: normalizeTheta(next.theta) };
      setGoal(staged);
      setBusy(true);
      setError(null);
      try {
        // `staged`, not the `goal` state — setState is not visible until the
        // next render, so reading it back here would submit the previous goal.
        track(await sendMoveTask(robotId, staged));
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      } finally {
        setBusy(false);
      }
    },
    [robotId, track, setError],
  );

  const send = React.useCallback(async () => {
    if (!goal) return;
    await sendGoal(goal);
  }, [goal, sendGoal]);

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
  }, [taskId, setError]);

  const clear = React.useCallback(() => {
    // The goal marker goes with the status: both describe the same finished
    // task, so leaving one on the map without the other is a lie.
    setGoal(null);
    reset();
  }, [reset]);

  return {
    goal,
    commitGoal,
    taskStatus: task.taskStatus,
    running: task.running,
    busy,
    error: task.error,
    cancelable: task.cancelable,
    send,
    sendGoal,
    cancel,
    clear,
  };
}
