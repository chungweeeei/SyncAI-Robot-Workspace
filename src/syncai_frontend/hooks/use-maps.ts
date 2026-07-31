import * as React from "react";

import { fetchMaps } from "@/lib/api/map";
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
 * Loads the map catalogue once per mount.
 *
 * Not polled, unlike useRobotState: the set of directories under `map/` only
 * changes when someone saves or converts a map, which is a deliberate act and
 * never happens while an operator is looking at this screen. `refresh()` is the
 * escape hatch instead of a timer.
 */
export function useMaps(): UseMaps {
  const [maps, setMaps] = React.useState<MapSummary[] | null>(null);
  const [status, setStatus] = React.useState<MapsStatus>("loading");
  const [nonce, setNonce] = React.useState(0);

  React.useEffect(() => {
    let active = true;
    const abort = new AbortController();

    fetchMaps(abort.signal)
      .then((data) => {
        if (!active) return;
        setMaps(data);
        setStatus("ok");
      })
      .catch(() => {
        if (active) setStatus("error");
      });

    return () => {
      active = false;
      abort.abort();
    };
  }, [nonce]);

  const refresh = React.useCallback(() => setNonce((n) => n + 1), []);

  return { maps, status, refresh };
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
 * Inherits useMaps' fetch-once-per-mount policy, so a map swapped underneath a
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
