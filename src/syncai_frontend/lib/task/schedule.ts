// How a registered trigger is written for an operator.
//
// This lived inside schedule-list.tsx, with a note saying a lib module holding
// one formatter would be a file whose header has nothing to say. It has
// something to say now: the saved-task library also has to render a trigger, so
// that a row can be told apart from a one-time task at a glance, and two copies
// of "every 30 min" would drift the first time the wording changed.

import type { ScheduleState, ScheduleTrigger } from "@/lib/api/schedule";

/** `every 30 min` / `0 9 * * 1-5 · Asia/Taipei`. */
export function describeTrigger(trigger: ScheduleTrigger): string {
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
 * The registered schedules that were frozen from each saved task, keyed by task
 * id.
 *
 * Only schedules that carry a `saved_task_id` land here — one registered
 * straight from the composer references no row and belongs to no task. That is
 * also why this is a lookup built from the schedule list rather than a field on
 * the saved task: the backend's saved-task response says nothing about
 * schedules, and the provenance only exists in the Temporal memo.
 */
export function schedulesBySavedTask(
  schedules: readonly ScheduleState[],
): Map<string, ScheduleState[]> {
  const byTask = new Map<string, ScheduleState[]>();
  for (const schedule of schedules) {
    const id = schedule.saved_task_id;
    if (!id) continue;
    const existing = byTask.get(id);
    if (existing) existing.push(schedule);
    else byTask.set(id, [schedule]);
  }
  return byTask;
}
