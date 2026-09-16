import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { fetchMaps } from "@/lib/api/map";
import { queryKeys } from "@/lib/api/query-keys";
import type { MapSummary } from "@/lib/types/map";

export type MapsStatus = "loading" | "ok" | "error";

/**
 * How often the catalogue is re-read while a gridmap conversion is running.
 *
 * Two seconds against a conversion that takes tens of seconds: fast enough that
 * "done" feels immediate, slow enough that the request is invisible next to the
 * telemetry socket. There is no push channel for this — the backend has no job
 * resource and a conversion finishing is not a ROS topic — and the poll is also
 * what makes the answer survive a reload, which a one-shot event would not.
 */
const CONVERTING_POLL_MS = 2000;

/**
 * The poll policy, shared by every observer of the maps query.
 *
 * The flag says when to stop, so this turns itself off with the last running
 * conversion rather than needing a timer anyone has to cancel. Factored out
 * because two hooks below mount the same query and a policy that differed
 * between them would make the poll depend on which screen happened to be open.
 */
function pollWhileConverting(
  maps: MapSummary[] | undefined,
): number | false {
  return maps?.some((map) => map.grid_status === "converting")
    ? CONVERTING_POLL_MS
    : false;
}

export interface UseMaps {
  /** Latest successfully fetched catalogue, or null before the first success. */
  maps: MapSummary[] | null;
  /**
   * "loading" until the first response, then "ok"/"error" for the most recent
   * fetch. A transient error keeps the last good `maps` — same contract as
   * useRobotState.
   */
  status: MapsStatus;
  /**
   * Re-fetch. The rename and delete controls invalidate this key directly
   * rather than call it; this is for a caller holding the hook's result.
   */
  refresh: () => void;
}

/**
 * The map catalogue, through the shared query cache.
 *
 * Not polled, unlike useRobotState — with one exception. The set of directories
 * under `map/` only changes when someone saves or converts a map, which is a
 * deliberate act and never happens while an operator is looking at this screen,
 * so `refresh()` is the escape hatch instead of a timer. The exception is a
 * running gridmap conversion: it is the one server-side process that changes
 * the catalogue on its own (`grid_status` leaves "converting", `grid` appears
 * or an error does), it has no other status surface, and the status itself says
 * when to stop — so the poll runs only while some map is converting and turns
 * itself off with the last one.
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
    refetchInterval: (query) => pollWhileConverting(query.state.data),
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
 * used to be free: a swap meant restarting the stack, which dropped the
 * telemetry socket next to this anyway. A live switch drops nothing, so
 * MapActivateControl invalidates this key itself — it is the only thing that
 * moves `active`, and nothing else would notice.
 */
export function useActiveMap(): { map: MapSummary | null; status: MapsStatus } {
  const { maps, status } = useMaps();
  const map = React.useMemo(
    () => maps?.find((entry) => entry.active) ?? null,
    [maps],
  );
  return { map, status };
}

/**
 * Watch one map's gridmap conversion, for a screen that started it.
 *
 * The mapping screen's reason for existing: a save kicks off a conversion that
 * takes tens of seconds, and until this hook the only place its outcome showed
 * up was the Maps screen — so an operator who saved a map and stayed put was
 * told "converting in the background" and then nothing, ever. What they needed
 * was not a new channel but an observer of the catalogue that already knows,
 * which is all this is.
 *
 * Mounts the same query entry as useMaps rather than a per-map endpoint (there
 * isn't one) — so the two share one request, and this hook inherits the
 * conversion poll without restating it. `enabled` is what keeps the mapping
 * screen from fetching a catalogue it has no other use for: nothing goes out
 * until a name is passed, i.e. until something has actually been saved.
 *
 * Returns null while no name is being watched, before the first response, and
 * for a name the catalogue does not list. All three mean "nothing to say yet",
 * which is what a caller renders as nothing.
 */
export function useMapConversion(name: string | null): MapSummary | null {
  const { data } = useQuery({
    queryKey: queryKeys.maps,
    queryFn: ({ signal }) => fetchMaps(signal),
    refetchInterval: (query) => pollWhileConverting(query.state.data),
    enabled: name !== null,
  });

  return React.useMemo(
    () => (name === null ? null : data?.find((entry) => entry.name === name) ?? null),
    [data, name],
  );
}
