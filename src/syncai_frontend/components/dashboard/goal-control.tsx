"use client";

import { CrosshairIcon, SendIcon, XIcon } from "lucide-react";

import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import type { GoalTask } from "@/hooks/use-goal-task";
import type { TaskStatus } from "@/lib/api/task";

const STATUS_VARIANT: Record<
  TaskStatus,
  "default" | "secondary" | "destructive" | "outline"
> = {
  PENDING: "secondary",
  IN_PROGRESS: "default",
  COMPLETED: "outline",
  FAILED: "destructive",
  CANCELED: "outline",
};

/**
 * Operator controls for the drag-a-goal flow: arm goal mode, read back the
 * staged pose, submit / cancel / clear. Shared by the 2D map and the 3D
 * point-cloud view so a nav goal is dispatched the same way (and reads the
 * same) in both, with only the positioning left to the caller.
 *
 * All state lives in `useGoalTask`; this component is presentation only.
 */
export function GoalControl({
  task,
  className,
}: {
  task: GoalTask;
  className?: string;
}) {
  const { goal, taskStatus, error, running, busy } = task;

  return (
    <div className={cn("flex flex-col items-start gap-2", className)}>
      <Button
        size="sm"
        variant={task.goalMode ? "default" : "outline"}
        className="h-7 gap-1 px-2 text-xs"
        disabled={running}
        onClick={task.toggleGoalMode}
      >
        <CrosshairIcon className="size-3.5" />
        {task.goalMode ? "Drag on map…" : "Set goal"}
      </Button>

      {(goal || taskStatus || error) && (
        <div className="w-56 rounded-md border bg-background/90 p-2 text-xs shadow-sm backdrop-blur">
          {goal && (
            <dl className="space-y-0.5 tabular-nums">
              <div className="flex justify-between">
                <dt className="text-muted-foreground">Goal X</dt>
                <dd>{goal.x.toFixed(2)} m</dd>
              </div>
              <div className="flex justify-between">
                <dt className="text-muted-foreground">Goal Y</dt>
                <dd>{goal.y.toFixed(2)} m</dd>
              </div>
              <div className="flex justify-between">
                <dt className="text-muted-foreground">Heading</dt>
                <dd>{goal.theta.toFixed(1)}°</dd>
              </div>
            </dl>
          )}

          {taskStatus && (
            <div className="mt-2 flex items-center justify-between">
              <span className="text-muted-foreground">Task</span>
              <Badge variant={STATUS_VARIANT[taskStatus]}>{taskStatus}</Badge>
            </div>
          )}

          {error && (
            <p className="mt-2 break-words text-destructive">{error}</p>
          )}

          <div className="mt-2 flex gap-1">
            {running ? (
              <Button
                size="sm"
                variant="destructive"
                className="h-7 flex-1 gap-1 px-2 text-xs"
                disabled={busy || !task.cancelable}
                onClick={task.cancel}
              >
                <XIcon className="size-3.5" />
                Cancel
              </Button>
            ) : (
              <>
                <Button
                  size="sm"
                  className="h-7 flex-1 gap-1 px-2 text-xs"
                  disabled={busy || !goal}
                  onClick={task.send}
                >
                  <SendIcon className="size-3.5" />
                  Send
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  className="h-7 px-2 text-xs"
                  disabled={busy}
                  onClick={task.clear}
                >
                  Clear
                </Button>
              </>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
