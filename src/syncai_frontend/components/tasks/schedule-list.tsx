"use client";

import * as React from "react";
import {
  ChevronDownIcon,
  ChevronRightIcon,
  PauseIcon,
  PlayIcon,
  Trash2Icon,
} from "lucide-react";

import { Chip, Readout } from "@/components/console/instrument";
import { IconButton } from "@/components/tasks/icon-button";
import {
  ScheduleSourceChip,
  ScheduleSteps,
} from "@/components/tasks/schedule-steps";
import type { SchedulesStatus } from "@/hooks/use-schedules";
import type { ScheduleState, ScheduleTrigger } from "@/lib/api/schedule";
import type { SavedTask } from "@/lib/api/saved-task";

export interface ScheduleListProps {
  schedules: ScheduleState[];
  /** The library, so a row can be diffed against the task it was frozen from. */
  savedTasks: SavedTask[];
  status: SchedulesStatus;
  busy: boolean;
  onPause: (id: string) => void;
  onResume: (id: string) => void;
  onDelete: (id: string) => void;
}

/**
 * `every 30 min` / `0 9 * * 1-5 · Asia/Taipei`.
 *
 * Local to this file, like map-card's formatSize: one consumer, and a
 * lib/task/schedule.ts holding two formatters would be a file whose header has
 * nothing to say.
 */
function describeTrigger(trigger: ScheduleTrigger): string {
  if (trigger.cron) {
    return trigger.timezone ? `${trigger.cron} · ${trigger.timezone}` : trigger.cron;
  }
  const seconds = trigger.interval_seconds;
  if (!seconds) return "—";
  if (seconds % 3600 === 0) return `every ${seconds / 3600} h`;
  if (seconds % 60 === 0) return `every ${seconds / 60} min`;
  return `every ${seconds} s`;
}

/**
 * `2026-08-04 09:00`, sliced out of the ISO string rather than run through
 * toLocaleString — this is a client component Next still prerenders, and a
 * server/browser timezone difference would be a hydration mismatch. UTC for
 * everyone is the honest trade, which is why the readout is labelled with it.
 * Same reasoning as map-card's formatTimestamp.
 */
function formatRunTime(iso: string): string {
  return iso.slice(0, 16).replace("T", " ");
}

export function ScheduleList({
  schedules,
  savedTasks,
  status,
  busy,
  onPause,
  onResume,
  onDelete,
}: ScheduleListProps) {
  if (status === "loading") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        Reading the schedules from Temporal…
      </p>
    );
  }

  if (status === "error") {
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        The schedules could not be read. The message is above.
      </p>
    );
  }

  if (!schedules.length) {
    // A muted line rather than the bordered Notice panel /maps uses: inside an
    // InstrumentGroup that would be a box in a box.
    return (
      <p className="text-[11px] leading-tight text-muted-foreground">
        No schedules are registered on this robot.
      </p>
    );
  }

  return (
    <ul className="space-y-1.5">
      {schedules.map((schedule) => (
        <ScheduleRow
          key={schedule.id}
          schedule={schedule}
          savedTasks={savedTasks}
          busy={busy}
          onPause={onPause}
          onResume={onResume}
          onDelete={onDelete}
        />
      ))}
    </ul>
  );
}

/**
 * One schedule row, expandable to show the steps it froze at registration.
 *
 * Its own component rather than inline in the list above, because the expanded
 * flag is per-row state and a hook cannot live inside a `.map` callback.
 */
function ScheduleRow({
  schedule,
  savedTasks,
  busy,
  onPause,
  onResume,
  onDelete,
}: {
  schedule: ScheduleState;
  savedTasks: SavedTask[];
  busy: boolean;
  onPause: (id: string) => void;
  onResume: (id: string) => void;
  onDelete: (id: string) => void;
}) {
  const [open, setOpen] = React.useState(false);

  /**
   * A paused schedule keeps reporting future run times — Temporal computes them
   * from the spec and pausing does not clear them, so the list answers with five
   * dates that will not happen. Showing them would be the one genuinely
   * misleading thing on this screen, so paused rows read "—". The array is also
   * indexed only after a length check, since a spec that can no longer fire
   * returns an empty one.
   */
  const next = schedule.paused ? undefined : schedule.next_run_times[0];

  const source =
    savedTasks.find((task) => task.id === schedule.saved_task_id) ?? null;

  return (
    <li className="rounded-sm border border-hairline bg-elevated/40 px-2 py-2">
      <div className="flex flex-wrap items-center gap-2">
        <button
          type="button"
          aria-label={open ? "Hide frozen steps" : "Show frozen steps"}
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
          {schedule.id}
        </span>

        <ScheduleSourceChip schedule={schedule} savedTasks={savedTasks} />
        {schedule.paused && <Chip tone="caution">Paused</Chip>}

        <div className="flex shrink-0 items-center gap-0.5">
          <IconButton
            label={schedule.paused ? "Resume schedule" : "Pause schedule"}
            disabled={busy}
            onClick={() =>
              schedule.paused ? onResume(schedule.id) : onPause(schedule.id)
            }
          >
            {schedule.paused ? (
              <PlayIcon className="size-3.5" aria-hidden />
            ) : (
              <PauseIcon className="size-3.5" aria-hidden />
            )}
          </IconButton>
          <IconButton
            label="Delete schedule"
            disabled={busy}
            className="text-signal-warn hover:bg-signal-warn/12"
            onClick={() => {
              // A confirm rather than an undo: deleting a Temporal schedule is not
              // reversible from here, and there is no local history to step back
              // over. Same stance as the vertex panel.
              if (
                window.confirm(
                  `Delete schedule "${schedule.id}"? This cannot be undone.`,
                )
              ) {
                onDelete(schedule.id);
              }
            }}
          >
            <Trash2Icon className="size-3.5" aria-hidden />
          </IconButton>
        </div>
      </div>

      <div className="mt-1.5 space-y-1 pl-7">
        <Readout
          label="Trigger"
          value={describeTrigger(schedule.trigger)}
          tone="cmd"
        />
        <Readout
          label="Next run"
          value={next ? formatRunTime(next) : "—"}
          unit={next ? "UTC" : undefined}
          tone={next ? "live" : "neutral"}
        />
      </div>

      {/* Mounted only while expanded, which is what defers the per-schedule
       * describe RPC to a deliberate gesture. Unmounting on collapse also means
       * re-expanding re-reads, so a schedule deleted and re-created elsewhere does
       * not show its old steps. */}
      {open && (
        <div className="pl-7">
          <ScheduleSteps scheduleId={schedule.id} source={source} />
        </div>
      )}
    </li>
  );
}
