"use client";

import { SendIcon, XIcon } from "lucide-react";

import { TaskStatusChip } from "@/components/console/task-chip";
import type { TaskDispatch } from "@/hooks/use-task-dispatch";

export interface DispatchPanelProps {
  dispatch: TaskDispatch;
  /** False when the step list or the robot id is not ready. */
  ready: boolean;
  /** Why not, as a muted line under the button. Null when ready. */
  reason: string | null;
  onDispatch: () => void;
}

/**
 * Send the authored list now, then watch it.
 *
 * Takes the whole hook value rather than a spread of its fields, following
 * GoalControl: status / error / busy / cancelable are always read together, and
 * flattening one object into eight props is noise.
 *
 * Not a <form>, unlike ScheduleForm. Enter-to-submit is right for typing a
 * schedule name and wrong for a control that moves a robot, since the operator's
 * cursor is usually still in a coordinate field when they finish.
 */
export function DispatchPanel({
  dispatch,
  ready,
  reason,
  onDispatch,
}: DispatchPanelProps) {
  const { taskStatus, running, busy, error, cancelable } = dispatch;

  return (
    <div className="space-y-2">
      {taskStatus && (
        <div className="flex items-center justify-between gap-2">
          <span className="instrument-label text-muted-foreground">Task</span>
          <TaskStatusChip status={taskStatus} />
        </div>
      )}

      {error && (
        <p
          role="alert"
          className="text-[11px] leading-snug break-words text-signal-warn"
        >
          {error}
        </p>
      )}

      <div className="flex gap-1.5">
        {running ? (
          <button
            type="button"
            disabled={busy || !cancelable}
            onClick={dispatch.cancel}
            className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm border border-signal-warn/50 bg-signal-warn/12 text-signal-warn transition-colors hover:bg-signal-warn/20 disabled:opacity-50"
          >
            <XIcon className="size-3.5" aria-hidden />
            Cancel
          </button>
        ) : (
          <>
            <button
              type="button"
              disabled={busy || !ready}
              onClick={onDispatch}
              className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
            >
              <SendIcon className="size-3.5" aria-hidden />
              Dispatch
            </button>
            {taskStatus && (
              <button
                type="button"
                disabled={busy}
                onClick={dispatch.clear}
                className="instrument-label h-7 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50"
              >
                Clear
              </button>
            )}
          </>
        )}
      </div>

      {!running && reason && (
        <p className="text-[11px] leading-tight text-muted-foreground">{reason}</p>
      )}
    </div>
  );
}
