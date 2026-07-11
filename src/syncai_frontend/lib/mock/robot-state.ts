import type { RobotState } from "@/lib/types/robot";

export const mockRobotState: RobotState = {
  timestamp: 1752220800,
  robot_id: "syncai-robot-01",
  map: "warehouse",
  mode: "AUTO",
  localization_status: {
    position: { x: 5.95, y: -2.14, z: 0.0, theta: -87.6 },
    velocity: 0.42,
  },
  network_status: {
    ssid: "SyncAI-LAN",
    bssid: "a4:cf:12:9b:3e:01",
    rssi: -58,
    ip_address: "192.168.0.32",
    mac_address: "dc:a6:32:1f:8c:45",
  },
  battery_status: {
    battery_percentage: 87,
  },
};
