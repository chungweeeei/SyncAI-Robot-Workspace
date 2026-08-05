"use client";

import * as React from "react";

import { Chip } from "@/components/console/instrument";
import { getSchedule } from "@/lib/api/schedule";
import { toDispatchSteps, type SavedTask } from "@/lib/api/saved-task";
import type { TaskStepRequest } from "@/lib/api/task";
import { stepGlyph } from "@/lib/task/step";

type Status = "loading" | "ok" | "error";

export interface ScheduleStepsProps {
  scheduleId: string;
  /** The saved task this schedule was frozen from, if it is still in the library. */
  source: SavedTask | null;
}

/**
 * A registered schedule's frozen step list, fetched when the row is expanded.
 *
 * Its own fetch rather than data threaded down from `useSchedules`, because the
 * collection endpoint cannot carry steps at all: a Temporal schedule *list*
 * element holds only the workflow type name, so the args have to come from a
 * per-schedule `describe`. Paying that on expansion means one RPC for the one
 * schedule the operator opened, instead of N on every first paint.
 *
 * When the schedule came from a saved task that is still in the library, the
 * frozen steps are diffed against that task's *current* resolution. That is the
 * only way the operator can see that a schedule is about to drive somewhere the
 * map no longer says — the steps in Temporal are concrete and nothing re-reads
 * them, so a vertex moved after registration never reaches a scheduled run.
 */
export function ScheduleSteps({ scheduleId, source }: ScheduleStepsProps) {
  const [steps, setSteps] = React.useState<TaskStepRequest[] | null>(null);
  const [status, setStatus] = React.useState<Status>("loading");

  React.useEffect(() => {
    let active = true;
    const abort = new AbortController();

    getSchedule(scheduleId, abort.signal)
      .then((state) => {
        if (!active) return;
        setSteps(state.steps ?? []);
        setStatus("ok");
      })
      .catch(() => {
        if (active && !abort.signal.aborted) setStatus("error");
      });

    return () => {
      active = false;
      abort.abort();
    };
  }, [scheduleId]);

  if (status === "loading") {
    return <Line>Reading the frozen steps…</Line>;
  }
  if (status === "error") {
    return <Line>The frozen steps could not be read.</Line>;
  }
  if (!steps?.length) {
    // Legitimate, not an error: a schedule registered before the backend exposed
    // its args, or one created outside this API, has nothing to show.
    return <Line>This schedule does not report its steps.</Line>;
  }

  const drift = source ? findDrift(steps, source) : null;

  return (
    <div className="mt-1.5 space-y-1">
      {drift !== null && (
        <p className="text-[11px] leading-tight text-signal-caution">
          {drift === 0
            ? "Frozen coordinates match the saved task."
            : `${drift} ${drift === 1 ? "step no longer matches" : "steps no longer match"} ` +
              `“${source?.name}”. A scheduled run uses what is frozen below — ` +
              "delete and re-create the schedule to pick up the current positions."}
        </p>
      )}

      <ol className="space-y-0.5">
        {steps.map((step, index) => (
          <li key={step.id} className="flex items-baseline gap-2 text-[11px]">
            <span className="readout w-3 shrink-0 text-muted-foreground">
              {index + 1}
            </span>
            <span className="instrument-label w-3 shrink-0 text-muted-foreground">
              {stepGlyph(step.type)}
            </span>
            {step.type === "MOVE" && (
              <span className="readout min-w-0 truncate">
                {step.params.x.toFixed(3)}, {step.params.y.toFixed(3)} ·{" "}
                {step.params.theta.toFixed(1)}°
              </span>
            )}
          </li>
        ))}
      </ol>
    </div>
  );
}

/**
 * How many steps the schedule froze that the saved task would now dispatch
 * differently, or null when the two are not comparable.
 *
 * A changed step *count* makes a per-step diff meaningless — the saved task was
 * edited after the schedule was registered — so that reports as "not comparable"
 * rather than as a misleading number.
 */
function findDrift(frozen: readonly TaskStepRequest[], source: SavedTask): number | null {
  let current: TaskStepRequest[];
  try {
    current = toDispatchSteps(source.steps);
  } catch {
    // A saved step with no resolvable coordinates at all; nothing to compare to.
    return null;
  }
  if (current.length !== frozen.length) return null;

  return frozen.reduce((count, step, index) => {
    const now = current[index];
    if (!now || now.type !== step.type) return count + 1;
    if (step.type !== "MOVE" || now.type !== "MOVE") return count;
    // Compared at the precision the console displays and dispatches at, so a
    // float-representation difference is not reported as drift.
    const same =
      step.params.x.toFixed(3) === now.params.x.toFixed(3) &&
      step.params.y.toFixed(3) === now.params.y.toFixed(3) &&
      step.params.theta.toFixed(1) === now.params.theta.toFixed(1);
    return same ? count : count + 1;
  }, 0);
}

function Line({ children }: { children: React.ReactNode }) {
  return (
    <p className="mt-1.5 text-[11px] leading-tight text-muted-foreground">{children}</p>
  );
}

/** The `stale` badge for the collapsed row, so drift is visible without expanding. */
export function ScheduleSourceChip({
  schedule,
  savedTasks,
}: {
  schedule: { saved_task_id?: string | null; saved_task_name?: string | null };
  savedTasks: readonly SavedTask[];
}) {
  // No source at all: registered from loose steps in the composer (or before the
  // memo carried a source). Saying so beats saying nothing — the alternative was
  // a row that looked exactly like a linked one, on a page whose library then
  // showed no sign that this schedule existed.
  if (!schedule.saved_task_id) {
    return <Chip tone="caution">unsaved steps</Chip>;
  }

  const source = savedTasks.find((task) => task.id === schedule.saved_task_id);
  // The source was deleted after registration. The schedule still runs — it holds
  // its own copy of the steps — so this is information, not a fault.
  if (!source) {
    return <Chip tone="neutral">source deleted</Chip>;
  }
  return <Chip tone="neutral">{schedule.saved_task_name ?? source.name}</Chip>;
}
