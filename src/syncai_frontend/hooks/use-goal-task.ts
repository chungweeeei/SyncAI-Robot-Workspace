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
  /** The goal that went out — what the marker and the read-back describe. */
  goal: GoalPose | null;
  taskStatus: TaskStatus | null;
  /** A submitted task that has not reached a terminal state yet. */
  running: boolean;
  /** True while a submit / cancel request is in flight. */
  busy: boolean;
  error: string | null;
  /** Only true while the task id is still known (see the poll effect). */
  cancelable: boolean;
  /**
   * Re-send the pose already on screen. Not a step in the ordinary flow — the
   * drag dispatches — but the only way back from a submit that failed, since by
   * then the gesture that produced the pose is over and there is nothing left to
   * release.
   */
  send: () => Promise<void>;
  /**
   * Stage a pose and dispatch it as a one-step MOVE task. The only door in: both
   * a finished drag and a confirmed double-click on a stored vertex arrive here,
   * so however the pose was chosen there is one running task, one read-back and
   * one Cancel.
   */
  sendGoal: (goal: GoalPose) => Promise<void>;
  cancel: () => Promise<void>;
  clear: () => void;
}

/**
 * The drag-a-goal state machine: dispatch a pose as a one-step MOVE task, then
 * track that task until it is terminal.
 *
 * Firing on pointer-up rather than staging for a Send press is deliberate, and
 * it is the argument the initial-pose flow already makes (see `useInitialPose`):
 * the arrow drawn under the drag *is* the preview, so a confirm step only asks
 * the operator to re-read as numbers what they just aimed by eye — and a button
 * that has to be pressed on every goal stops being read by the tenth one.
 *
 * What keeps a real robot movement deliberate lives upstream instead: the tool
 * has to be armed by an explicit press, one drag disarms it again, and a press
 * outside the map extent never becomes a pose at all. And a goal, unlike a pose
 * estimate, is recoverable after the fact — Cancel stops the robot, which is a
 * better answer to a misplaced goal than a pre-flight read-back would have been.
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

  const sendGoal = React.useCallback(
    async (next: GoalPose) => {
      // Normalised on the way in, not just inside sendMoveTask: the marker and
      // the read-back are drawn from this state, and a heading they disagree
      // with the dispatched task about would be a lie about a moving robot.
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
