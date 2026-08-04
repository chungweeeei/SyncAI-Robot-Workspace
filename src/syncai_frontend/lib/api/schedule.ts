// Client for the backend schedule API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/schedule.py).
//
// Separate from lib/api/task.ts even though a schedule's `steps` are literally
// the same `StepRequest`, because the two have opposite lifecycles. A task is
// created and immediately dispatched — creation *is* dispatch — and then polled
// once until it is terminal, after which it is gone. A schedule is a durable
// registration that is listed, paused, resumed and deleted, and never polled.
// Merging them would leave one file whose header has to explain both.
//
// snake_case throughout, and `interval_seconds` in particular. `ScheduleRequest`
// and `ScheduleTriggerRequest` are plain pydantic BaseModels with no alias
// generator, unlike the `BaseSchema` in gateways/workflow/schema.py that the step
// params inherit from. So `intervalSeconds` is not an alias here: it is silently
// ignored, `interval_seconds` stays None, and the request then fails the
// `_exactly_one` validator with a 422 naming a field the client never sent.

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";
import type { TaskStepRequest } from "@/lib/api/task";

/**
 * `ScheduleTriggerRequest` / `ScheduleTriggerResponse`.
 *
 * Exactly one of `cron` / `interval_seconds` — the backend's validator rejects
 * both and neither with equal prejudice, so the two are modelled as optional
 * rather than as a union: the form that builds one only ever sets a single field.
 *
 * Every field is `| null` because the *response* is not the request shape read
 * back. `ScheduleTriggerResponse` declares all three as `Optional[...] = None`
 * and FastAPI serialises them, so an interval schedule answers with an explicit
 * `{"cron": null, "interval_seconds": 86400, "timezone": null}` rather than
 * omitting the two that do not apply. Consumers must test truthiness, not
 * `=== undefined`.
 */
export interface ScheduleTrigger {
  cron?: string | null;
  /** Must be > 0. Cannot be combined with `cron`. */
  interval_seconds?: number | null;
  /**
   * IANA zone name. Only meaningful for `cron` — the interval path ignores it,
   * so it is omitted rather than sent-and-ignored (see schedule-form.tsx).
   */
  timezone?: string | null;
}

/** What the operator supplies. `steps` is the same wire shape a task takes. */
export interface ScheduleDraft {
  id: string;
  trigger: ScheduleTrigger;
  steps: readonly TaskStepRequest[];
}

/** `ScheduleStateResponse`, verbatim. */
export interface ScheduleState {
  id: string;
  /**
   * The trigger as it was registered, read back from the Temporal schedule's
   * memo rather than from the schedule spec — Temporal normalises a cron string
   * into calendar fields, so the spec can no longer say what was asked for.
   */
  trigger: ScheduleTrigger;
  paused: boolean;
  /** ISO-8601 **UTC**, soonest first. Empty while the schedule is paused. */
  next_run_times: string[];
  /** Map the frozen coordinates are in, from the memo. Null for older schedules. */
  map_name?: string | null;
  /** The saved task this was frozen from, if it came from one. */
  saved_task_id?: string | null;
  saved_task_name?: string | null;
  /**
   * The frozen step list — **only** populated by `getSchedule`; always `[]` from
   * `listSchedules`, and legitimately `[]` for a schedule registered before this
   * field existed or one whose payload could not be decoded.
   *
   * Frozen at registration: a schedule stores concrete steps in Temporal, and
   * nothing re-reads them, so later vertex edits reach saved tasks and immediate
   * dispatches but not an already-registered schedule. Comparing these against
   * the source saved task's current resolution is how that staleness becomes
   * visible instead of silent.
   */
  steps?: TaskStepRequest[];
}

/**
 * The operator types the id, so `encodeURIComponent` is load-bearing rather than
 * defensive here: an unescaped `/` or `#` in a name would rewrite the URL into a
 * different route instead of producing a 404. Same reason as `vertexPath`.
 */
function schedulePath(id?: string, action?: "pause" | "resume"): string {
  const base = "/api/v1/schedules";
  if (!id) return apiUrl(base);
  const one = `${base}/${encodeURIComponent(id)}`;
  return apiUrl(action ? `${one}/${action}` : one);
}

/**
 * Every registered schedule for this robot — but never their steps.
 *
 * `steps` is always `[]` from this endpoint, and that is a property of Temporal
 * rather than an omission: a schedule *list* element carries only the workflow
 * type name, so reaching the frozen step list would cost a `describe` RPC per
 * row, paid on first paint for data most rows never show. `getSchedule` is the
 * path that carries them.
 */
export function listSchedules(signal?: AbortSignal): Promise<ScheduleState[]> {
  return requestJson<ScheduleState[]>(schedulePath(), { signal });
}

/**
 * One schedule, **including its frozen step list**.
 *
 * This is the only way to see what a schedule will actually do. Fetched on a
 * deliberate gesture — expanding the row — so the extra describe is paid once,
 * for the one schedule the operator asked about.
 */
export function getSchedule(
  id: string,
  signal?: AbortSignal,
): Promise<ScheduleState> {
  return requestJson<ScheduleState>(schedulePath(id), { signal });
}

// The four writes all drop the `{id, message}` envelope: it says nothing the
// caller does not already know from the request having succeeded, which is the
// same reasoning `deleteVertex` records. What a caller *does* need afterwards —
// the recomputed `next_run_times` — is only knowable by asking, so every one of
// these is followed by a list refresh rather than a local splice.

export function createSchedule(draft: ScheduleDraft): Promise<void> {
  return requestJson<void>(schedulePath(), {
    method: "POST",
    body: JSON.stringify(draft),
    parse: false,
  });
}

export function pauseSchedule(id: string): Promise<void> {
  return requestJson<void>(schedulePath(id, "pause"), {
    method: "POST",
    parse: false,
  });
}

export function resumeSchedule(id: string): Promise<void> {
  return requestJson<void>(schedulePath(id, "resume"), {
    method: "POST",
    parse: false,
  });
}

export function deleteSchedule(id: string): Promise<void> {
  return requestJson<void>(schedulePath(id), {
    method: "DELETE",
    parse: false,
  });
}
