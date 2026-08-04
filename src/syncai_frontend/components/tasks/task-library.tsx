"use client";

import * as React from "react";

import { SavedTaskRow } from "@/components/tasks/saved-task-row";
import type { SavedTask } from "@/lib/api/saved-task";
import type { TaskStatus, TaskStepState } from "@/lib/api/task";
import type { SavedTasksStatus } from "@/hooks/use-saved-tasks";

export interface TaskLibraryProps {
  /** Rows to show — already scoped by the caller (see TaskConsole). */
  tasks: SavedTask[];
  /** How many rows were withheld because they belong to another map. */
  hiddenCount: number;
  status: SavedTasksStatus;
  busy: boolean;
  activeMapName: string | null;
  /** True while a task is running, or before the robot id is known. */
  dispatchDisabled: boolean;
  /** The saved task the in-flight run came from, or null if it came from the editor. */
  dispatchedFromId: string | null;
  taskStatus: TaskStatus | null;
  stepStates: ReadonlyMap<string, TaskStepState>;
  onDispatch: (task: SavedTask) => void;
  onLoad: (task: SavedTask) => void;
  onSchedule: (task: SavedTask) => void;
  onDelete: (task: SavedTask) => void;
}

/**
 * The saved-task library. Presentation only.
 *
 * Height-capped with its own scroll, which is what makes putting it *above* the
 * composer affordable without any collapse state — no chevron on the frame,
 * nothing to remember, nothing to persist. Same pattern and rationale as
 * VertexList: a robot with thirty saved routes would otherwise push the composer
 * off the screen.
 */
export function TaskLibrary({
  tasks,
  hiddenCount,
  status,
  busy,
  activeMapName,
  dispatchDisabled,
  dispatchedFromId,
  taskStatus,
  stepStates,
  onDispatch,
  onLoad,
  onSchedule,
  onDelete,
}: TaskLibraryProps) {
  if (status === "loading") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        Reading the saved tasks…
      </p>
    );
  }

  if (status === "error") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        The saved tasks could not be read. The message is above.
      </p>
    );
  }

  return (
    <div className="space-y-1.5">
      {tasks.length === 0 ? (
        <p className="text-[11px] leading-tight text-muted-foreground">
          Nothing saved for this map yet. Build a list below, name it, and press
          Save as new.
        </p>
      ) : (
        <ul className="max-h-56 space-y-1.5 overflow-y-auto">
          {tasks.map((task) => (
            <SavedTaskRow
              key={task.id}
              task={task}
              activeMapName={activeMapName}
              dispatchDisabled={dispatchDisabled}
              busy={busy}
              // Only the row that started the run gets the readback. A row that
              // did not is left blank rather than showing another row's status.
              taskStatus={task.id === dispatchedFromId ? taskStatus : null}
              stepStates={task.id === dispatchedFromId ? stepStates : null}
              onDispatch={() => onDispatch(task)}
              onLoad={() => onLoad(task)}
              onSchedule={() => onSchedule(task)}
              onDelete={() => onDelete(task)}
            />
          ))}
        </ul>
      )}

      {/*
       * The honesty footnote. The library is scoped to the loaded map, which is
       * what was asked for — but silently dropping the rest is exactly how an
       * operator concludes their work was lost, which is the complaint this whole
       * feature answers. Saying how many are hidden costs one line and closes it.
       */}
      {hiddenCount > 0 && (
        <p className="text-[11px] leading-tight text-muted-foreground">
          {hiddenCount} {hiddenCount === 1 ? "task is" : "tasks are"} saved for
          other maps and not shown here.
        </p>
      )}
    </div>
  );
}
