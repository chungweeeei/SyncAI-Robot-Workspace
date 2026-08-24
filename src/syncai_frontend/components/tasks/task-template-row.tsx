"use client";

import * as React from "react";
import {
  CalendarPlusIcon,
  ChevronDownIcon,
  ChevronRightIcon,
  ClockIcon,
  PencilIcon,
  SendIcon,
  Trash2Icon,
} from "lucide-react";

import { Chip } from "@/components/console/instrument";
import { TaskStatusChip } from "@/components/console/task-chip";
import { IconButton } from "@/components/tasks/icon-button";
import type { ScheduleState } from "@/lib/api/schedule";
import type { TemplateStep, TaskTemplate } from "@/lib/api/task-template";
import type { TaskStepState } from "@/lib/api/task";
import { describeTrigger } from "@/lib/task/schedule";
import { stepGlyph } from "@/lib/task/step";

export interface TaskTemplateRowProps {
  template: TaskTemplate;
  /** The map the robot is on, for the mismatch sentence. Null when none loaded. */
  activeMapName: string | null;
  /** True while any task is in flight, or the robot id is not known yet. */
  dispatchDisabled: boolean;
  /** Per-step state, but only when *this* row is the one that was dispatched. */
  stepStates: ReadonlyMap<string, TaskStepState> | null;
  /** Task-level status of the run this row started, or null. */
  taskStatus: React.ComponentProps<typeof TaskStatusChip>["status"] | null;
  /**
   * The registered schedules frozen from this template. Empty means the row is a
   * one-time task — which, until this existed, was indistinguishable from a
   * scheduled one: the schedules live in their own list, on the other pane of
   * the composer, and nothing on the row said the robot would run it unattended.
   */
  schedules: readonly ScheduleState[];
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
export function TaskTemplateRow({
  template,
  activeMapName,
  dispatchDisabled,
  stepStates,
  taskStatus,
  schedules,
  busy,
  onDispatch,
  onLoad,
  onSchedule,
  onDelete,
}: TaskTemplateRowProps) {
  const [open, setOpen] = React.useState(false);

  /**
   * One chip for however many schedules point at this template.
   *
   * A single schedule spells its trigger out, because that is the fact the
   * operator is actually after — "every 30 min" answers "will this run on its
   * own, and when" in one read. Two or more collapse to a count: a row is not
   * the place to list them, and the Registered schedules pane is one click away
   * on the composer's Schedule pane.
   *
   * Paused counts as scheduled but not as armed, so an all-paused row goes
   * caution — the same tone the wrong-map chip uses for "registered, but it will
   * not do what the label implies".
   */
  const schedule =
    schedules.length === 0
      ? null
      : {
          label:
            schedules.length === 1
              ? describeTrigger(schedules[0].trigger)
              : `${schedules.length} schedules`,
          paused: schedules.every((entry) => entry.paused),
          title: schedules
            .map(
              (entry) =>
                `${entry.id}: ${describeTrigger(entry.trigger)}${entry.paused ? " (paused)" : ""}`,
            )
            .join("\n"),
        };

  // The one hard gate in this feature. A task whose coordinates are in another
  // map's frame points somewhere else entirely in the loaded map, so it cannot be
  // dispatched or scheduled — unlike a missing vertex, which merely falls back to
  // the snapshot and is reported.
  const wrongMap = template.map_name !== null && !template.map_matches_active;
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
          {template.name}
        </span>

        {/* The step shape at a glance — M M S reads as a route without expanding. */}
        <span
          className="readout shrink-0 text-[11px] tracking-wider text-muted-foreground"
          aria-label={`${template.steps.length} steps`}
        >
          {template.steps.map((step) => stepGlyph(step.type)).join(" ")}
        </span>

        {/* Before the map chip: whether the robot runs this by itself outranks
          * which map it is in. */}
        {schedule && (
          <Chip
            tone={schedule.paused ? "caution" : "active"}
            title={schedule.title}
            className="gap-1"
          >
            <ClockIcon className="size-3" aria-hidden />
            {schedule.paused ? "paused" : schedule.label}
          </Chip>
        )}
        {template.map_name && (
          <Chip tone={wrongMap ? "caution" : "neutral"}>{template.map_name}</Chip>
        )}
        {template.missing_vertex_count > 0 && (
          <Chip tone="caution">{template.missing_vertex_count} missing</Chip>
        )}
        {taskStatus && <TaskStatusChip status={taskStatus} />}

        <div className="ml-auto flex shrink-0 items-center gap-0.5">
          <IconButton
            label={`Dispatch "${template.name}" now`}
            disabled={blocked || busy}
            onClick={onDispatch}
            className="text-signal-cmd hover:bg-signal-cmd/12"
          >
            <SendIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Load "${template.name}" into the editor`}
            // Loading is always allowed, even for another map's task: reading and
            // fixing one is exactly what you would want to do next.
            disabled={busy}
            onClick={onLoad}
          >
            <PencilIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Schedule "${template.name}"`}
            disabled={wrongMap || busy}
            onClick={onSchedule}
          >
            <CalendarPlusIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label={`Delete "${template.name}"`}
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
          Saved for <span className="readout">{template.map_name}</span>; the robot has{" "}
          <span className="readout">{activeMapName ?? "no map"}</span> loaded.
        </p>
      )}

      {open && (
        <ol className="mt-1.5 space-y-0.5 pl-7">
          {template.steps.map((step, index) => (
            <TemplateStepLine
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

function TemplateStepLine({
  step,
  index,
  state,
}: {
  step: TemplateStep;
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
