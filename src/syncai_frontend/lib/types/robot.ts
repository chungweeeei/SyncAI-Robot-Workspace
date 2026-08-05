// Mirrors the backend Pydantic models in
// src/syncai_backend/syncai_backend/interfaces/rest/routers/robot.py
// (snake_case preserved so wiring GET /api/v1/robot/state later is a drop-in fetch)

export type RobotMode = "MAINTENANCE" | "MANUAL" | "AUTO";

/**
 * What the gait controller reports it is doing — not `RobotState.mode`, which is
 * which byobu session is up.
 *
 * Open strings rather than unions on purpose: the backend decodes the controller's
 * integers through a lookup with an `"UNKNOWN"` fallback, and it will legitimately
 * hit that fallback — CHAMP/ISSAC are real policies the REST command surface does
 * not expose, and MPC's motion code is genuinely unknown.
 *
 * The payload carries labels only; the raw integers stay on the ROS topic. So
 * `"UNKNOWN"` is as much as the console can ever say, and two different unmapped
 * codes are indistinguishable here — `ros2 topic echo /<robot_id>/robot_state
 * --field low_level_mode` is where you find out which one it was.
 */
export interface RobotLowLevelMode {
  policy: string;
  motion: string;
}

export interface RobotPose {
  x: number;
  y: number;
  z: number;
  /** heading in degrees */
  theta: number;
}

/**
 * A pose on the map floor: what a drag on the viewport produces and what the
 * backend takes for both a nav goal and an initial-pose estimate. Theta is in
 * degrees, CCW from +x — the whole REST vocabulary is degrees.
 */
export interface PlanarPose {
  x: number;
  y: number;
  theta: number;
}

/**
 * The planner's remaining global route, from the telemetry stream's `path`
 * frames. Flat map-frame metres — [x0, y0, x1, y1, …] — because the only
 * consumer walks it to build geometry, and a Float32Array of pairs is what that
 * loop wants rather than an array of objects.
 *
 * Heading is not carried: the viewport draws the route as a band on the floor,
 * not as a series of poses.
 *
 * An empty `points` is the explicit "no route" state, not a missing sample. The
 * backend synthesises it when plans stop arriving, because nothing in the nav
 * stack publishes an empty plan and arrival / cancel / abort are otherwise
 * indistinguishable silence.
 */
export interface PlannedPath {
  points: Float32Array;
  stamp: number;
}

export interface RobotLocalizationStatus {
  position: RobotPose;
  /** linear velocity in m/s */
  velocity: number;
}

export interface RobotNetworkStatus {
  ssid: string;
  bssid: string;
  /** signal strength in dBm */
  rssi: number;
  ip_address: string;
  mac_address: string;
}

export interface RobotBatteryStatus {
  battery_percentage: number;
}

/**
 * One joint's health. A subset of the ROS MotorState — q/dq/ddq/tau_est are not
 * exposed over REST, because robot_state is a 10 Hz snapshot whose samples
 * cannot be ordered. Live joint kinematics come from the telemetry WebSocket.
 */
export interface RobotMotorStatus {
  /** URDF joint name, matching the keys in lib/robot/g23-joints.ts */
  name: string;
  /** degrees Celsius */
  temperature: number;
  /** motor error code; 0 when healthy */
  error: number;
}

export interface RobotState {
  timestamp: number;
  robot_id: string;
  map: string;
  mode: RobotMode;
  low_level_mode: RobotLowLevelMode;
  localization_status: RobotLocalizationStatus;
  network_status: RobotNetworkStatus;
  battery_status: RobotBatteryStatus;
  /** empty while syncai_driver_manager is not publishing motor_states */
  motor_status: RobotMotorStatus[];
}

// Mirrors the ROS map_server YAML (map/warehouse.yaml)
export interface MapMetadata {
  /** meters per cell */
  resolution: number;
  /** world coordinates of the grid's bottom-left cell [x, y, yaw] */
  origin: [number, number, number];
  /** grid width in cells */
  width: number;
  /** grid height in cells */
  height: number;
}

// Occupancy values follow the ROS convention: 0 free, 100 occupied, -1 unknown
export type OccupancyGrid = Int8Array;

// Mirrors map/vertexes.json
export interface Vertex {
  name: string;
  pose: {
    x: number;
    y: number;
    theta: number;
  };
}
