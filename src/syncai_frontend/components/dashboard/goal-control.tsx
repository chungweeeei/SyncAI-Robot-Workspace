"use client";

import { CrosshairIcon, SendIcon, XIcon } from "lucide-react";

import { Readout, overlayPanel } from "@/components/console/instrument";
import { TaskStatusChip } from "@/components/console/task-chip";
import { cn } from "@/lib/utils";
import type { GoalTask } from "@/hooks/use-goal-task";

/**
 * Operator controls for the drag-a-goal flow: arm goal mode, read back the
 * staged pose, submit / cancel / clear.
 *
 * It floats on the viewport rather than living in the instrument rail because
 * the gesture that produces a goal happens on the map: moving the readback away
 * from the drag would put the number and the thing it describes on opposite
 * sides of the screen. The staged pose is set in the commanded hue — the same
 * cyan the arrow is drawn in on the canvas.
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
      <button
        type="button"
        disabled={running}
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
        {armed ? "Drag on the map" : "Set goal"}
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

          {taskStatus && (
            <div className="mt-2.5 flex items-center justify-between gap-2">
              <span className="instrument-label text-muted-foreground">
                Task
              </span>
              <TaskStatusChip status={taskStatus} />
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
                <button
                  type="button"
                  disabled={busy || !goal}
                  onClick={task.send}
                  className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
                >
                  <SendIcon className="size-3.5" />
                  Send
                </button>
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
