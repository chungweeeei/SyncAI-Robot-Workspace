// Client for the mapping-run surface: switching the operating mode and saving
// the map a run built.
// (backend: POST /api/v1/robot/mode in routers/robot.py,
//  POST /api/v1/maps in routers/map.py)

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";

/**
 * The modes an operator can switch *into*. MAINTENANCE is reported by
 * `RobotState.mode` but is not a target — it means "no session is up", which is
 * not a state you ask for.
 */
export type SwitchableMode = "MANUAL" | "AUTO";

export interface SwitchModeResult {
  mode: SwitchableMode;
  /**
   * True when the switch was dispatched and the stack — including the backend
   * serving this API — is being torn down and rebuilt. False means the robot
   * was already in the requested mode and nothing happened.
   */
  switching: boolean;
  message: string;
}

/**
 * Ask sys_manager to switch the operating mode (which byobu session is up).
 *
 * The one request that outlives its server: a real switch kills the byobu
 * session the backend is a pane of, so the *usual* outcome is a network error —
 * the connection drops before the response is written. Callers must read that
 * as "switch in progress", not failure (useModeSwitch does), and poll
 * GET /api/v1/robot/state until `mode` reports the target. The responses that
 * do arrive are the quick cases: the no-op (`switching: false`) and an
 * HTTP-level refusal.
 */
export function switchRobotMode(mode: SwitchableMode): Promise<SwitchModeResult> {
  return requestJson<SwitchModeResult>(apiUrl("/api/v1/robot/mode"), {
    method: "POST",
    body: JSON.stringify({ mode }),
  });
}

export interface SaveMapResult {
  name: string;
  /** True on any 200 — pgo wrote map.pcd. */
  has_pointcloud: boolean;
  /**
   * Whether the pcd → gridmap conversion was started in the background. The
   * map lists with `grid: null` until it finishes; if it never does (or this
   * is false), the conversion is a by-hand tools/pcd_to_gridmap.py run.
   */
  grid_pending: boolean;
  /** Operator-facing sentence; render it verbatim. */
  message: string;
}

/**
 * Mirrors the backend catalogue's name rule, so the Save button can refuse a
 * bad name before a request goes out. The server still validates — this is a
 * convenience, not the boundary.
 */
export const MAP_NAME_RE = /^[A-Za-z0-9._-]{1,64}$/;

/**
 * Save the current mapping run as `map/<name>/` on the robot.
 *
 * Only meaningful in MANUAL mode: pgo is the sole holder of the run's
 * keyframes and the sole serialiser, so in AUTO this is a 502 whose `detail`
 * says exactly that. The other operator-facing refusals are a 409 for a taken
 * name and pgo's own "NO POSES!" for a run that has not banked a keyframe yet.
 * The call can take a while — the robot is merging and writing a ~20 MB cloud.
 */
export function saveMap(name: string): Promise<SaveMapResult> {
  return requestJson<SaveMapResult>(apiUrl("/api/v1/maps"), {
    method: "POST",
    body: JSON.stringify({ name }),
  });
}
