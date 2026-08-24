"use client";

import { CrosshairIcon, SendIcon, XIcon } from "lucide-react";

import { Chip, Readout, overlayPanel } from "@/components/console/instrument";
import { TaskStatusChip } from "@/components/console/task-chip";
import { cn } from "@/lib/utils";
import type { GoalTask } from "@/hooks/use-goal-task";

/**
 * Operator controls for the drag-a-goal flow: arm goal mode, read back the goal
 * that went out, cancel it or clear it.
 *
 * It floats on the viewport rather than living in the instrument rail because
 * the gesture that produces a goal happens on the map: moving the readback away
 * from the drag would put the number and the thing it describes on opposite
 * sides of the screen. The pose is set in the commanded hue — the same cyan the
 * arrow is drawn in on the canvas.
 *
 * There is no Send: the drag dispatches on release (see `useGoalTask`), so this
 * panel is a readback of what already went out, not a form waiting to be
 * submitted. A button standing there afterwards would read as a step still owed
 * — which is how the old Send button read once the arrow on the map had already
 * said where the robot was going. What is left is the way back out: Cancel while
 * the task runs, Clear once it is finished, and a Retry that appears only when
 * the submit itself failed, because by then the drag is over and re-aiming is
 * the only other option.
 *
 * Task state lives in `useGoalTask` and the armed flag in the view (one pick
 * mode is shared with the initial-pose control); this component is presentation
 * only.
 */
export function GoalControl({
  task,
  armed,
  onArm,
  className,
}: {
  task: GoalTask;
  /** True while a drag on the viewport will produce a goal. */
  armed: boolean;
  onArm: () => void;
  className?: string;
}) {
  const { goal, taskStatus, error, running, busy } = task;

  return (
    <div className={cn("flex flex-col items-start gap-2", className)}>
      {/* `busy` as well as `running`: the tracker only reports a task once the
        * POST has come back, and a drag is a dispatch now, so re-arming inside
        * that window would put a second goal on the wire before the first one
        * has a status to disarm on. */}
      <button
        type="button"
        disabled={running || busy}
        onClick={onArm}
        className={cn(
          overlayPanel,
          "instrument-label flex h-7 items-center gap-1.5 px-2 transition-colors disabled:opacity-50",
          armed
            ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
            : "hover:bg-elevated",
        )}
      >
        <CrosshairIcon className="size-3.5" />
        {/* Names the release, not the drag: letting go is what commands the
          * robot, and the operator should know that before the press. */}
        {armed ? "Aim and release to send" : "Set goal"}
      </button>

      {(goal || taskStatus || error) && (
        <div className={cn(overlayPanel, "w-full p-2.5")}>
          {goal && (
            <div className="space-y-1">
              <Readout label="Goal X" value={goal.x.toFixed(2)} unit="m" tone="cmd" />
              <Readout label="Goal Y" value={goal.y.toFixed(2)} unit="m" tone="cmd" />
              <Readout
                label="Heading"
                value={goal.theta.toFixed(1)}
                unit="°"
                tone="cmd"
              />
            </div>
          )}

          {/* `busy` earns a row of its own: the POST is in flight before the
            * tracker has a status to show, and coordinates sitting there with no
            * state beside them read as a drag that did nothing. */}
          {(taskStatus || busy) && (
            <div className="mt-2.5 flex items-center justify-between gap-2">
              <span className="instrument-label text-muted-foreground">
                Task
              </span>
              {busy && !taskStatus ? (
                <Chip tone="cmd">SENDING</Chip>
              ) : (
                taskStatus && <TaskStatusChip status={taskStatus} />
              )}
            </div>
          )}

          {error && (
            <p className="mt-2.5 text-[11px] leading-snug break-words text-signal-warn">
              {error}
            </p>
          )}

          <div className="mt-2.5 flex gap-1.5">
            {running ? (
              <button
                type="button"
                disabled={busy || !task.cancelable}
                onClick={task.cancel}
                className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm border border-signal-warn/50 bg-signal-warn/12 text-signal-warn transition-colors hover:bg-signal-warn/20 disabled:opacity-50"
              >
                <XIcon className="size-3.5" />
                Cancel
              </button>
            ) : (
              <>
                {/* Only ever a retry — see the header. To re-drive a goal that
                  * went out fine, drag again. */}
                {error && (
                  <button
                    type="button"
                    disabled={busy || !goal}
                    onClick={task.send}
                    className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm border border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd transition-colors hover:bg-signal-cmd/20 disabled:opacity-50"
                  >
                    <SendIcon className="size-3.5" />
                    Retry
                  </button>
                )}
                {/* Disabled while busy so a Clear landing between release and
                  * the POST resolving cannot drop a goal that is on its way. */}
                <button
                  type="button"
                  disabled={busy}
                  onClick={task.clear}
                  className="instrument-label h-7 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50"
                >
                  Clear
                </button>
              </>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
