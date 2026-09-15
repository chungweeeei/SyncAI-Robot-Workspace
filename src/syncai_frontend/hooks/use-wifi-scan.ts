import * as React from "react";
import { useQuery } from "@tanstack/react-query";

import { scanWifiNetworks, type WifiNetwork } from "@/lib/api/network";
import { queryKeys } from "@/lib/api/query-keys";

export type WifiScanStatus = "loading" | "ok" | "error";

export interface UseWifiScan {
  /** The last successful scan, or null before the first. A failed rescan keeps it. */
  networks: WifiNetwork[] | null;
  /** "loading" until the first answer, then "ok"/"error" for the latest scan. */
  status: WifiScanStatus;
  /** True while any scan — the first or a rescan — is in flight. */
  scanning: boolean;
  /** The backend's 502 sentence for the latest scan, or null. */
  error: string | null;
  /** Start another scan (~45 s). A no-op while one is already running. */
  rescan: () => void;
}

/**
 * The networks the robot can see, through the shared query cache.
 *
 * A scan costs up to 45 s of nmcli on the robot, so the policy is the inverse
 * of the other queries here: fetch once when the Settings card first mounts,
 * then never on its own again. `staleTime: Infinity` is what makes that true —
 * a remount paints the cached list immediately with no background refetch, and
 * only `rescan()` starts another. It was preferred over `enabled: false` with a
 * Scan button because navigating to Settings is already the explicit act; an
 * operator opening this card almost always wants the list, and asking for a
 * second click buys nothing that the stale-time rule does not. `gcTime` is
 * raised so a detour to another screen does not evict a list that took 45 s to
 * build (the default 5 min is tuned for cheap GETs).
 *
 * `rescan` uses `refetch()` rather than the `invalidateQueries` the other hooks
 * prefer: there is exactly one observer of this key and the button means
 * "start now", not "mark stale and let observers decide".
 */
export function useWifiScan(): UseWifiScan {
  const { data, isPending, isError, isFetching, error, refetch } = useQuery({
    queryKey: queryKeys.wifiScan,
    // No `({ signal })` on purpose — see scanWifiNetworks for why an abort on
    // unmount would only waste the scan the robot is already running.
    queryFn: scanWifiNetworks,
    staleTime: Infinity,
    gcTime: 30 * 60_000,
  });

  const rescan = React.useCallback(() => {
    void refetch();
  }, [refetch]);

  return {
    networks: data ?? null,
    status: isPending ? "loading" : isError ? "error" : "ok",
    scanning: isFetching,
    error: error ? (error instanceof Error ? error.message : String(error)) : null,
    rescan,
  };
}
