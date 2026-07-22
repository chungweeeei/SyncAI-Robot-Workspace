import * as React from "react";

import { apiUrl } from "@/lib/api/config";
import type { RobotState } from "@/lib/types/robot";

// syncai_robot_state publishes at 1 Hz, so polling faster than that only
// re-fetches the same snapshot.
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
}

/**
 * Polls GET /api/v1/robot/state and returns the latest RobotState. The backend
 * responds 404 until the robot has published its first state, which surfaces
 * here as status "error" with state still null.
 */
export function useRobotState(pollMs: number = DEFAULT_POLL_MS): UseRobotState {
  const [state, setState] = React.useState<RobotState | null>(null);
  const [status, setStatus] = React.useState<RobotStateStatus>("loading");

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

  return { state, status };
}
