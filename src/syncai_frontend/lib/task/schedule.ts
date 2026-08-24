// How a registered trigger is written for an operator.
//
// This lived inside schedule-list.tsx, with a note saying a lib module holding
// one formatter would be a file whose header has nothing to say. It has
// something to say now: the template library also has to render a trigger, so
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
 * The registered schedules that were frozen from each template, keyed by
 * template id.
 *
 * Only schedules that carry a `task_template_id` land here — one registered
 * straight from the composer references no row and belongs to no template. That
 * is also why this is a lookup built from the schedule list rather than a field
 * on the template: the backend's template response says nothing about
 * schedules, and the provenance only exists in the Temporal memo.
 */
export function schedulesByTemplate(
  schedules: readonly ScheduleState[],
): Map<string, ScheduleState[]> {
  const byTemplate = new Map<string, ScheduleState[]>();
  for (const schedule of schedules) {
    const id = schedule.task_template_id;
    if (!id) continue;
    const existing = byTemplate.get(id);
    if (existing) existing.push(schedule);
    else byTemplate.set(id, [schedule]);
  }
  return byTemplate;
}
