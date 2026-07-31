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
