"use client";

import * as React from "react";

import { TaskTemplateRow } from "@/components/tasks/task-template-row";
import type { ScheduleState } from "@/lib/api/schedule";
import type { TaskTemplate } from "@/lib/api/task-template";
import type { TaskStatus, TaskStepState } from "@/lib/api/task";
import type { TaskTemplatesStatus } from "@/hooks/use-task-templates";
import { schedulesByTemplate } from "@/lib/task/schedule";

/** Shared empty list, so an unscheduled row is not a fresh prop every render. */
const EMPTY_SCHEDULES: readonly ScheduleState[] = [];

export interface TaskLibraryProps {
  /** Rows to show — already scoped by the caller (see TaskConsole). */
  templates: TaskTemplate[];
  /** How many rows were withheld because they belong to another map. */
  hiddenCount: number;
  /**
   * Every registered schedule, not the ones for a given row: the rows are
   * matched here so the caller does not have to build the same index once per
   * row. Passed in rather than fetched, like every other prop on this component.
   */
  schedules: readonly ScheduleState[];
  /**
   * How many of those name no template. They cannot be shown on a row — there
   * is no row they belong to — but they are unattended runs on this robot, so
   * counting them is what keeps "no clock chip anywhere" from meaning "nothing
   * runs on its own here".
   */
  unlinkedScheduleCount: number;
  status: TaskTemplatesStatus;
  busy: boolean;
  activeMapName: string | null;
  /** True while a task is running, or before the robot id is known. */
  dispatchDisabled: boolean;
  /** The template the in-flight run came from, or null if it came from the editor. */
  dispatchedFromId: string | null;
  taskStatus: TaskStatus | null;
  stepStates: ReadonlyMap<string, TaskStepState>;
  onDispatch: (template: TaskTemplate) => void;
  onLoad: (template: TaskTemplate) => void;
  onSchedule: (template: TaskTemplate) => void;
  onDelete: (template: TaskTemplate) => void;
}

/**
 * The template library. Presentation only.
 *
 * Height-capped with its own scroll, which is what makes putting it *above* the
 * composer affordable without any collapse state — no chevron on the frame,
 * nothing to remember, nothing to persist. Same pattern and rationale as
 * VertexList: a robot with thirty saved routes would otherwise push the composer
 * off the screen.
 */
export function TaskLibrary({
  templates,
  hiddenCount,
  schedules,
  unlinkedScheduleCount,
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
  const scheduledBy = React.useMemo(
    () => schedulesByTemplate(schedules),
    [schedules],
  );

  if (status === "loading") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        Reading the task templates…
      </p>
    );
  }

  if (status === "error") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        The task templates could not be read. The message is above.
      </p>
    );
  }

  return (
    <div className="space-y-1.5">
      {templates.length === 0 ? (
        <p className="text-[11px] leading-tight text-muted-foreground">
          No templates for this map yet. Build a list below, name it, and press
          Save as new.
        </p>
      ) : (
        <ul className="max-h-56 space-y-1.5 overflow-y-auto">
          {templates.map((template) => (
            <TaskTemplateRow
              key={template.id}
              template={template}
              activeMapName={activeMapName}
              dispatchDisabled={dispatchDisabled}
              busy={busy}
              // Only the row that started the run gets the readback. A row that
              // did not is left blank rather than showing another row's status.
              taskStatus={template.id === dispatchedFromId ? taskStatus : null}
              stepStates={template.id === dispatchedFromId ? stepStates : null}
              schedules={scheduledBy.get(template.id) ?? EMPTY_SCHEDULES}
              onDispatch={() => onDispatch(template)}
              onLoad={() => onLoad(template)}
              onSchedule={() => onSchedule(template)}
              onDelete={() => onDelete(template)}
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
          {hiddenCount} {hiddenCount === 1 ? "template is" : "templates are"} saved
          for other maps and not shown here.
        </p>
      )}

      {/*
       * The same kind of footnote, for the other direction. A schedule
       * registered from loose steps — the composer's Schedule pane with nothing
       * loaded — records no source, so no row above can carry its clock chip. It
       * still runs the robot unattended, and an operator reading a library with
       * no clock chips on it would otherwise conclude nothing does.
       */}
      {unlinkedScheduleCount > 0 && (
        <p className="text-[11px] leading-tight text-signal-caution">
          {unlinkedScheduleCount}{" "}
          {unlinkedScheduleCount === 1 ? "schedule runs" : "schedules run"} steps
          that were never saved as a template, so {unlinkedScheduleCount === 1 ? "it is" : "they are"}{" "}
          not shown on any row here — see Registered schedules below.
        </p>
      )}
    </div>
  );
}
