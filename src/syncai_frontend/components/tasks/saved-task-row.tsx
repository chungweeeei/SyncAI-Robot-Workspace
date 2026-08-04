"use client";

import * as React from "react";
import {
  CalendarPlusIcon,
  ChevronDownIcon,
  ChevronRightIcon,
  PencilIcon,
  SendIcon,
  Trash2Icon,
} from "lucide-react";

import { Chip } from "@/components/console/instrument";
import { TaskStatusChip } from "@/components/console/task-chip";
import { IconButton } from "@/components/tasks/icon-button";
import type { SavedStep, SavedTask } from "@/lib/api/saved-task";
import type { TaskStepState } from "@/lib/api/task";
import { stepGlyph } from "@/lib/task/step";

export interface SavedTaskRowProps {
  task: SavedTask;
  /** The map the robot is on, for the mismatch sentence. Null when none loaded. */
  activeMapName: string | null;
  /** True while any task is in flight, or the robot id is not known yet. */
  dispatchDisabled: boolean;
  /** Per-step state, but only when *this* row is the one that was dispatched. */
  stepStates: ReadonlyMap<string, TaskStepState> | null;
  /** Task-level status of the run this row started, or null. */
  taskStatus: React.ComponentProps<typeof TaskStatusChip>["status"] | null;
  busy: boolean;
  onDispatch: () => void;
  onLoad: () => void;
  onSchedule: () => void;
  onDelete: () => void;
}

/**
 * One library row: what it does, and the four things you can do with it.
 *
 * Expansion is a single boolean and a chevron, not an accordion component —
 * there is no such primitive in this project and one row's open state is not
 * worth adding it. Actions are four bare icon buttons rather than a row menu,
 * matching every other list in the console (ScheduleList, VertexList) and keeping
 * the tap targets large on a touch console.
 */
export function SavedTaskRow({
  task,
  activeMapName,
  dispatchDisabled,
  stepStates,
  taskStatus,
  busy,
  onDispatch,
  onLoad,
  onSchedule,
  onDelete,
}: SavedTaskRowProps) {
  const [open, setOpen] = React.useState(false);

  // The one hard gate in this feature. A task whose coordinates are in another
  // map's frame points somewhere else entirely in the loaded map, so it cannot be
  // dispatched or scheduled — unlike a missing vertex, which merely falls back to
  // the snapshot and is reported.
  const wrongMap = task.map_name !== null && !task.map_matches_active;
  const blocked = wrongMap || dispatchDisabled;

  return (
    <li className="rounded-sm border border-hairline bg-elevated/40 px-2 py-2">
      <div className="flex flex-wrap items-center gap-2">
        <button
          type="button"
          aria-label={open ? "Hide steps" : "Show steps"}
          aria-expanded={open}
          onClick={() => setOpen((v) => !v)}
          className="flex size-5 shrink-0 items-center justify-center rounded-sm text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
        >
          {open ? (
            <ChevronDownIcon className="size-3.5" aria-hidden />
          ) : (
            <ChevronRightIcon className="size-3.5" aria-hidden />
          )}
        </button>

        <span className="readout min-w-0 flex-1 truncate text-[13px]">
          {task.name}
        </span>

        {/* The step shape at a glance — M M S reads as a route without expanding. */}
        <span
          className="readout shrink-0 text-[11px] tracking-wider text-muted-foreground"
          aria-label={`${task.steps.length} steps`}
        >
          {task.steps.map((step) => stepGlyph(step.type)).join(" ")}
        </span>

        {task.map_name && (
          <Chip tone={wrongMap ? "caution" : "neutral"}>{task.map_name}</Chip>
        )}
        {task.missing_vertex_count > 0 && (
          <Chip tone="caution">{task.missing_vertex_count} missing</Chip>
        )}
        {taskStatus && <TaskStatusChip status={taskStatus} />}

        <div className="ml-auto flex shrink-0 items-center gap-0.5">
          <IconButton
            label={`Dispatch "${task.name}" now`}
            disabled={blocked || busy}
            onClick={onDispatch}
            className="text-signal-cmd hover:bg-signal-cmd/12"
          >
            <SendIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Load "${task.name}" into the editor`}
            // Loading is always allowed, even for another map's task: reading and
            // fixing one is exactly what you would want to do next.
            disabled={busy}
            onClick={onLoad}
          >
            <PencilIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Schedule "${task.name}"`}
            disabled={wrongMap || busy}
            onClick={onSchedule}
          >
            <CalendarPlusIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Delete "${task.name}"`}
            disabled={busy}
            className="text-signal-warn hover:bg-signal-warn/12"
            onClick={onDelete}
          >
            <Trash2Icon className="size-3.5" aria-hidden />
          </IconButton>
        </div>
      </div>

      {wrongMap && (
        <p className="mt-1 pl-7 text-[11px] leading-tight text-signal-caution">
          Saved for <span className="readout">{task.map_name}</span>; the robot has{" "}
          <span className="readout">{activeMapName ?? "no map"}</span> loaded.
        </p>
      )}

      {open && (
        <ol className="mt-1.5 space-y-0.5 pl-7">
          {task.steps.map((step, index) => (
            <SavedStepLine
              key={step.id}
              step={step}
              index={index}
              state={stepStates?.get(step.id) ?? null}
            />
          ))}
        </ol>
      )}
    </li>
  );
}

function SavedStepLine({
  step,
  index,
  state,
}: {
  step: SavedStep;
  index: number;
  state: TaskStepState | null;
}) {
  // resolved_params, not params: this is what a dispatch would actually send, so
  // showing the snapshot would be showing a number the robot will not drive to.
  const pose = step.resolved_params;

  return (
    <li className="flex items-baseline gap-2 text-[11px]">
      <span className="readout w-3 shrink-0 text-muted-foreground">{index + 1}</span>
      <span className="instrument-label w-3 shrink-0 text-muted-foreground">
        {stepGlyph(step.type)}
      </span>
      {step.vertex_name && (
        <span
          className={
            step.vertex_status === "MISSING"
              ? "readout shrink-0 text-signal-caution line-through"
              : "readout shrink-0 text-muted-foreground"
          }
          title={
            step.vertex_status === "MISSING"
              ? "This vertex was deleted; the coordinates are the snapshot taken when the task was saved."
              : undefined
          }
        >
          {step.vertex_name}
        </span>
      )}
      {pose && (
        <span className="readout min-w-0 truncate">
          {pose.x.toFixed(3)}, {pose.y.toFixed(3)} · {pose.theta.toFixed(1)}°
        </span>
      )}
      {state && <TaskStatusChip status={state.status} />}
    </li>
  );
}
