"use client";

import * as React from "react";
import { useQuery } from "@tanstack/react-query";

import { queryKeys } from "@/lib/api/query-keys";
import { fetchActiveTasks, type ActiveTask } from "@/lib/api/task";

// Slower than the 1 Hz robot-state poll next to it, on purpose. This poll's job
// is discovering runs *this tab did not start* — a schedule firing, another
// console, a run that predates the page load — and none of those are events an
// operator can perceive to the second: a schedule fires on the minute at best.
// A task this tab did start is already followed at 1 Hz by useTaskTracker, which
// is instant and needs no help from here.
//
// It is also deliberately longer than the backend's 1.5 s snapshot TTL, so a
// single tab misses the cache on every poll and every additional tab is
// absorbed for free. See ACTIVE_TASK_CACHE_TTL_S in the backend's workflow
// config for the arithmetic.
const ACTIVE_TASK_POLL_MS = 2000;

export type ActiveTasksStatus = "loading" | "ok" | "error";

export interface UseActiveTasks {
  /** Running executions, or the last good list while `status` is "error". */
  tasks: ActiveTask[];
  /**
   * "loading" until the first response, then "ok"/"error" for the most recent
   * fetch. Consumers MUST distinguish "error" from an empty list: a failed poll
   * rendered as "idle" is the one way this indicator can be actively dangerous,
   * because it would say the robot is standing still while it drives.
   */
  status: ActiveTasksStatus;
  /**
   * The backend's clock at the last successful read, ISO 8601. Elapsed times
   * are measured against this, not against the browser's clock.
   */
  asOf: string | null;
  /** Re-read now, for a caller that just changed the answer (e.g. a cancel). */
  refresh: () => void;
}

/**
 * What the robot is executing right now, whoever asked for it.
 *
 * Shaped after useRobotState — one interval, last-good-value on a transient
 * failure (the cache holds `data` through an errored refetch, and the tone says
 * it is a memory; blanking the list while a run is likely still going would be
 * the less honest of the two wrong answers) — because it plays the same role: a
 * small, always-on fact about the machine that the whole console reads. It is
 * mounted once, in ActiveTaskProvider, for the same reason that one is.
 */
export function useActiveTasks(
  pollMs: number = ACTIVE_TASK_POLL_MS,
): UseActiveTasks {
  const { data, isPending, isError, refetch } = useQuery({
    queryKey: queryKeys.activeTasks,
    queryFn: ({ signal }) => fetchActiveTasks(signal),
    refetchInterval: pollMs,
  });

  const refresh = React.useCallback(() => {
    void refetch();
  }, [refetch]);

  return {
    tasks: data?.tasks ?? [],
    status: isPending ? "loading" : isError ? "error" : "ok",
    asOf: data?.as_of ?? null,
    refresh,
  };
}
