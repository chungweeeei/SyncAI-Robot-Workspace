"use client";

import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { queryKeys } from "@/lib/api/query-keys";
import {
  createSchedule,
  deleteSchedule,
  listSchedules,
  pauseSchedule,
  resumeSchedule,
  type ScheduleDraft,
  type ScheduleState,
} from "@/lib/api/schedule";

export type SchedulesStatus = "loading" | "ok" | "error";

/**
 * How long to wait before re-reading the list after a pause / resume.
 *
 * Measured against a live backend: `POST .../pause` answers 200, but the very
 * next `GET /api/v1/schedules` still reports `paused: false` — Temporal's
 * describe lags the patch by a second or two. An immediate refresh therefore
 * shows the row exactly as it was, and the operator concludes the button did
 * nothing. Create and delete do *not* have this lag (a created schedule is in the
 * next list, a deleted one is gone), which is why only these two settle.
 */
const PAUSE_SETTLE_MS = 2000;

export interface UseSchedules {
  schedules: ScheduleState[];
  status: SchedulesStatus;
  /** The load failure, or the most recent write failure. Rendered verbatim. */
  error: string | null;
  /** True while a create / pause / resume / delete is in flight. */
  busy: boolean;
  /** True when the schedule was registered. */
  create: (draft: ScheduleDraft) => Promise<boolean>;
  pause: (id: string) => Promise<boolean>;
  resume: (id: string) => Promise<boolean>;
  remove: (id: string) => Promise<boolean>;
  refresh: () => void;
}

/**
 * The schedules registered on this robot, written through on every change.
 *
 * Shaped after useMapVertices, with two deliberate differences.
 *
 * Every mutation ends in a refresh, success or failure, and the authoritative
 * list always comes from the backend. useMapVertices splices because its POST/PUT
 * response *is* the stored row; here the responses are a bare `{id, message}`,
 * and `next_run_times` is computed by Temporal — when a resumed schedule fires
 * next is knowable only by asking. Refreshing after a *failure* too is what makes
 * a 404 from DELETE (Temporal had already dropped it) resolve into the row
 * disappearing, rather than sitting there next to an error about it. The one local
 * write is the optimistic paused flag, and PAUSE_SETTLE_MS explains why.
 *
 * Write failures live in their own slot rather than the query's, and the
 * refresh-on-every-write above is the reason: a rejected create triggers a
 * reload that *succeeds*, and if the failure lived where the query keeps its
 * error, that success would clear the very sentence the operator needs to read.
 * Splitting them, and preferring the write, is what makes "Schedule X already
 * exists" survive the reload that follows it.
 *
 * There is no timer. `next_run_times` moves on the minute at best, while the list
 * endpoint costs a Temporal list RPC plus a memo decode per schedule — a 1 Hz
 * poll would spend a request a second on data that changes hourly. Same trade
 * useMaps records, with an explicit Refresh as the escape hatch.
 */
export function useSchedules(): UseSchedules {
  const queryClient = useQueryClient();
  const query = useQuery({
    queryKey: queryKeys.schedules,
    queryFn: ({ signal }) => listSchedules(signal),
  });

  const [busy, setBusy] = React.useState(false);
  const [writeError, setWriteError] = React.useState<string | null>(null);

  const refresh = React.useCallback(
    () => void queryClient.invalidateQueries({ queryKey: queryKeys.schedules }),
    [queryClient],
  );

  /**
   * Run one write, holding `busy` and turning a rejection into `error`.
   *
   * Rejections are swallowed rather than rethrown because every caller is wired
   * straight to an onClick — a rethrow would be an unhandled rejection, and the
   * components read the outcome off `error` and the returned boolean.
   */
  const run = React.useCallback(
    async (action: () => Promise<void>): Promise<boolean> => {
      setBusy(true);
      setWriteError(null);
      try {
        await action();
        return true;
      } catch (cause) {
        setWriteError(cause instanceof Error ? cause.message : String(cause));
        return false;
      } finally {
        setBusy(false);
        refresh();
      }
    },
    [refresh],
  );

  /**
   * Pause / resume: flip the row in the cache, then reconcile once Temporal has
   * caught up. The optimistic flip is what makes the button feel like it did
   * something during the PAUSE_SETTLE_MS window, and it is safe to trust because
   * the write already returned 200 — this is a display lag, not an unconfirmed
   * write. A failure skips the flip and refreshes at once, so the row snaps back.
   */
  const runPaused = React.useCallback(
    async (id: string, paused: boolean, action: () => Promise<void>) => {
      setBusy(true);
      setWriteError(null);
      try {
        await action();
        queryClient.setQueryData<ScheduleState[]>(
          queryKeys.schedules,
          (current) =>
            current?.map((entry) =>
              entry.id === id ? { ...entry, paused } : entry,
            ),
        );
        window.setTimeout(refresh, PAUSE_SETTLE_MS);
        return true;
      } catch (cause) {
        setWriteError(cause instanceof Error ? cause.message : String(cause));
        refresh();
        return false;
      } finally {
        setBusy(false);
      }
    },
    [queryClient, refresh],
  );

  const create = React.useCallback(
    (draft: ScheduleDraft) => run(() => createSchedule(draft)),
    [run],
  );
  const pause = React.useCallback(
    (id: string) => runPaused(id, true, () => pauseSchedule(id)),
    [runPaused],
  );
  const resume = React.useCallback(
    (id: string) => runPaused(id, false, () => resumeSchedule(id)),
    [runPaused],
  );
  const remove = React.useCallback(
    (id: string) => run(() => deleteSchedule(id)),
    [run],
  );

  return {
    schedules: query.data ?? [],
    status: query.isPending ? "loading" : query.isError ? "error" : "ok",
    error: writeError ?? query.error?.message ?? null,
    busy,
    create,
    pause,
    resume,
    remove,
    refresh,
  };
}
