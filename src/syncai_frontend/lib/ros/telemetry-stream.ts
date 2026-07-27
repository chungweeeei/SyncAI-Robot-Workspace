import { wsUrl } from "@/lib/api/config";
import type { RobotPose } from "@/lib/types/robot";
import type { StreamStatus } from "@/lib/types/pointcloud";

/**
 * Client for the internal telemetry WebSocket
 * (/api/v1/robot/telemetry/stream): the high-rate channel the 3D viewer uses
 * for pose and joint angles, deliberately separate from the frozen
 * GET /api/v1/robot/state third-party contract (1 Hz — unusable for motion)
 * and from the binary point-cloud stream (different wire format and
 * backpressure behaviour).
 *
 * Wire format: JSON text frames multiplexed by ``type``:
 *
 *   {"type": "pose",   "x", "y", "z", "yaw_deg", "stamp"}
 *   {"type": "joints", "joints": {"FL_HipX_joint": rad, ...}, "stamp"}
 */

interface PoseMessage {
  type: "pose";
  x: number;
  y: number;
  z: number;
  yaw_deg: number;
  stamp: number;
}

interface JointsMessage {
  type: "joints";
  joints: Record<string, number>;
  stamp: number;
}

type TelemetryMessage = PoseMessage | JointsMessage;

export interface TelemetryStreamHandlers {
  onPose?: (pose: RobotPose) => void;
  /** Joint angles in radians, keyed by URDF joint name. */
  onJoints?: (joints: Record<string, number>) => void;
  onStatus?: (status: StreamStatus) => void;
}

export interface TelemetryStream {
  close: () => void;
}

/**
 * Connect to the telemetry WebSocket and dispatch each message to its typed
 * handler. Reconnects automatically with a fixed backoff until closed — same
 * skeleton as createPointCloudStream, kept separate because the payloads
 * (JSON vs binary) share nothing.
 */
export function createTelemetryStream(
  handlers: TelemetryStreamHandlers,
  path = "/api/v1/robot/telemetry/stream",
): TelemetryStream {
  let ws: WebSocket | null = null;
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  let closed = false;

  const connect = () => {
    if (closed) return;
    handlers.onStatus?.("connecting");
    ws = new WebSocket(wsUrl(path));

    ws.onopen = () => handlers.onStatus?.("open");
    ws.onmessage = (ev) => {
      if (typeof ev.data !== "string") return;
      let msg: TelemetryMessage;
      try {
        msg = JSON.parse(ev.data) as TelemetryMessage;
      } catch {
        return; // malformed frame; skip rather than kill the stream
      }
      if (msg.type === "pose") {
        // The wire's yaw_deg is already degrees — the same unit (and name
        // apart) as RobotPose.theta, so this is a pure relabel.
        handlers.onPose?.({ x: msg.x, y: msg.y, z: msg.z, theta: msg.yaw_deg });
      } else if (msg.type === "joints") {
        handlers.onJoints?.(msg.joints);
      }
    };
    ws.onerror = () => handlers.onStatus?.("error");
    ws.onclose = () => {
      handlers.onStatus?.("closed");
      if (!closed) {
        reconnectTimer = setTimeout(connect, 2000);
      }
    };
  };

  connect();

  return {
    close: () => {
      closed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
    },
  };
}
