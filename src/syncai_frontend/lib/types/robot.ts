// Mirrors the backend Pydantic models in
// src/syncai_backend/syncai_backend/interfaces/rest/routers/robot.py
// (snake_case preserved so wiring GET /api/v1/robot/state later is a drop-in fetch)

export type RobotMode = "MAINTENANCE" | "MANUAL" | "AUTO";

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
