import { useQuery } from "@tanstack/react-query";

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";
import { queryKeys } from "@/lib/api/query-keys";
import type { RobotState } from "@/lib/types/robot";

// The upstream detailed topic runs at 10 Hz, but RobotState.timestamp has only
// whole-second resolution and this endpoint is a frozen third-party contract, so
// 1 Hz is the useful poll rate: faster only re-fetches an indistinguishable
// snapshot. The 3D viewer uses the telemetry WebSocket for real motion.
const DEFAULT_POLL_MS = 1000;

export type RobotStateStatus = "loading" | "ok" | "error";

export interface UseRobotState {
  /** Latest successfully fetched state, or null before the first success. */
  state: RobotState | null;
  /**
   * "loading" until the first response, then "ok"/"error" reflecting the most
   * recent fetch. On a transient error the last good `state` is kept.
   */
  status: RobotStateStatus;
  /**
   * Client clock (ms) at the last successful fetch, null before the first one.
   * A fresh value on every frame is what drives the status strip's 1 Hz sweep,
   * so this changes even when the payload itself is identical — `state` alone
   * cannot say "a frame just arrived". (Query's dataUpdatedAt moves on every
   * successful fetch even when structural sharing keeps `data` the same object,
   * which is exactly that contract.)
   */
  updatedAt: number | null;
}

/**
 * Polls GET /api/v1/robot/state and returns the latest RobotState. The backend
 * responds 404 until the robot has published its first state, which surfaces
 * here as status "error" with state still null.
 *
 * TanStack Query owns the interval and the cache; the keep-the-last-good-state
 * behaviour on a transient failure comes from the cache holding `data` through
 * an errored refetch, same contract as the hand-rolled interval this replaces.
 */
export function useRobotState(pollMs: number = DEFAULT_POLL_MS): UseRobotState {
  const { data, dataUpdatedAt, isPending, isError } = useQuery({
    queryKey: queryKeys.robotState,
    queryFn: ({ signal }) =>
      requestJson<RobotState>(apiUrl("/api/v1/robot/state"), { signal }),
    refetchInterval: pollMs,
  });

  return {
    state: data ?? null,
    status: isPending ? "loading" : isError ? "error" : "ok",
    // dataUpdatedAt is 0 until the first success; the contract wants null.
    updatedAt: dataUpdatedAt || null,
  };
}
