import * as React from "react";

import { apiUrl } from "@/lib/api/config";
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
   * cannot say "a frame just arrived".
   */
  updatedAt: number | null;
}

/**
 * Polls GET /api/v1/robot/state and returns the latest RobotState. The backend
 * responds 404 until the robot has published its first state, which surfaces
 * here as status "error" with state still null.
 */
export function useRobotState(pollMs: number = DEFAULT_POLL_MS): UseRobotState {
  const [state, setState] = React.useState<RobotState | null>(null);
  const [status, setStatus] = React.useState<RobotStateStatus>("loading");
  const [updatedAt, setUpdatedAt] = React.useState<number | null>(null);

  React.useEffect(() => {
    let active = true;
    const tick = async () => {
      try {
        const res = await fetch(apiUrl("/api/v1/robot/state"));
        if (!active) return;
        if (!res.ok) {
          setStatus("error");
          return;
        }
        const data = (await res.json()) as RobotState;
        if (!active) return;
        setState(data);
        setStatus("ok");
        setUpdatedAt(Date.now());
      } catch {
        // Transient (network blip, backend restart): keep the last good state.
        if (active) setStatus("error");
      }
    };
    tick();
    const id = setInterval(tick, pollMs);
    return () => {
      active = false;
      clearInterval(id);
    };
  }, [pollMs]);

  return { state, status, updatedAt };
}
