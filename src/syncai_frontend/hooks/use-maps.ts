import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { fetchMaps } from "@/lib/api/map";
import { queryKeys } from "@/lib/api/query-keys";
import type { MapSummary } from "@/lib/types/map";

export type MapsStatus = "loading" | "ok" | "error";

export interface UseMaps {
  /** Latest successfully fetched catalogue, or null before the first success. */
  maps: MapSummary[] | null;
  /**
   * "loading" until the first response, then "ok"/"error" for the most recent
   * fetch. A transient error keeps the last good `maps` — same contract as
   * useRobotState.
   */
  status: MapsStatus;
  /** Re-fetch. Nothing mutates maps yet; activate/delete will use this. */
  refresh: () => void;
}

/**
 * The map catalogue, through the shared query cache.
 *
 * Not polled, unlike useRobotState: the set of directories under `map/` only
 * changes when someone saves or converts a map, which is a deliberate act and
 * never happens while an operator is looking at this screen. `refresh()` is the
 * escape hatch instead of a timer.
 *
 * Every observer shares one cache entry and one in-flight request — the
 * dashboard mounting useActiveMap in two components used to cost two GETs; now
 * the second render reads the first's answer. A fresh mount still refetches
 * (staleTime 0), which keeps the old picked-up-on-next-visit behaviour, just
 * with the cached copy painted while the request runs.
 */
export function useMaps(): UseMaps {
  const queryClient = useQueryClient();
  const { data, isPending, isError } = useQuery({
    queryKey: queryKeys.maps,
    queryFn: ({ signal }) => fetchMaps(signal),
  });

  // Invalidate rather than refetch(): the entry is shared, so a Refresh pressed
  // on one screen must update every mounted observer, not just this one.
  const refresh = React.useCallback(
    () => void queryClient.invalidateQueries({ queryKey: queryKeys.maps }),
    [queryClient],
  );

  return {
    maps: data ?? null,
    status: isPending ? "loading" : isError ? "error" : "ok",
    refresh,
  };
}

/**
 * The map the running stack loaded, or null until the catalogue answers.
 *
 * The `active` flag is the only way to ask this question: the backend resolves
 * it from the INI, and dropping the live `map` topic left no endpoint that
 * means "the current one". Wrapped in a hook rather than left as a `.find()` at
 * each call site so the answer has one definition — the same reason the flag is
 * server-derived instead of the UI parsing `RobotState.map`'s path.
 *
 * Inherits useMaps' refetch-on-mount policy, so a map swapped underneath a
 * long-open dashboard is not picked up until something calls `refresh()`. That
 * is the trade the disk-sourced endpoints already made; a swap means restarting
 * the stack, which drops the telemetry socket next to this anyway.
 */
export function useActiveMap(): { map: MapSummary | null; status: MapsStatus } {
  const { maps, status } = useMaps();
  const map = React.useMemo(
    () => maps?.find((entry) => entry.active) ?? null,
    [maps],
  );
  return { map, status };
}
