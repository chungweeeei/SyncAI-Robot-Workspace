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

export interface RobotState {
  timestamp: number;
  robot_id: string;
  map: string;
  mode: RobotMode;
  localization_status: RobotLocalizationStatus;
  network_status: RobotNetworkStatus;
  battery_status: RobotBatteryStatus;
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
