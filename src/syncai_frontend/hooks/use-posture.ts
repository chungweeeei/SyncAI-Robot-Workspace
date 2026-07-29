"use client";

import * as React from "react";

import { useTaskTracker } from "@/hooks/use-task-tracker";
import { sendPostureTask, type Posture, type TaskStatus } from "@/lib/api/task";

export interface PostureControl {
  /** Submit a posture command. */
  send: (posture: Posture) => Promise<void>;
  /** The command whose task is being tracked, or null. */
  sent: Posture | null;
  taskStatus: TaskStatus | null;
  running: boolean;
  /** True while the submit request itself is in flight. */
  busy: boolean;
  error: string | null;
  clear: () => void;
}

/**
 * Stand up / lie down, each submitted as a one-step posture task.
 *
 * One click, no confirmation step: unlike a nav goal there is nothing staged to
 * read back — the command is fully described by the button that was pressed,
 * and the operator standing next to the robot is the one pressing it.
 *
 * There is no cancel either: the backend turns each of these into a single
 * motion key that the gait controller has already acted on by the time the
 * workflow reports, so a cancel button would suggest a reversal the stack
 * cannot do. Sending the opposite posture is the way back.
 */
export function usePosture(robotId: string): PostureControl {
  const [sent, setSent] = React.useState<Posture | null>(null);
  const [busy, setBusy] = React.useState(false);
  const task = useTaskTracker();

  const { track, setError, reset } = task;

  const send = React.useCallback(
    async (posture: Posture) => {
      setBusy(true);
      setError(null);
      setSent(posture);
      try {
        track(await sendPostureTask(robotId, posture));
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      } finally {
        setBusy(false);
      }
    },
    [robotId, track, setError],
  );

  const clear = React.useCallback(() => {
    setSent(null);
    reset();
  }, [reset]);

  return {
    send,
    sent,
    taskStatus: task.taskStatus,
    running: task.running,
    busy,
    error: task.error,
    clear,
  };
}
