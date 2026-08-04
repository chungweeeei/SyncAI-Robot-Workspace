import {
  createReconnectingSocket,
  type ReconnectingSocket,
} from "@/lib/ros/socket";
import type { RobotPose } from "@/lib/types/robot";
import type { StreamStatus } from "@/lib/types/stream";

/**
 * Client for the internal telemetry WebSocket
 * (/api/v1/robot/telemetry/stream): the high-rate channel the 3D viewer uses
 * for pose and joint angles, deliberately separate from the frozen
 * GET /api/v1/robot/state third-party contract (whole-second timestamps,
 * polled — unusable for motion) and from the point-cloud stream (a big frame
 * there would head-of-line block pose here; see lib/ros/socket.ts).
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

/**
 * Connect to the telemetry WebSocket and dispatch each message to its typed
 * handler. Reconnect and status handling come from createReconnectingSocket;
 * only the JSON demultiplexing below is specific to this stream.
 */
export function createTelemetryStream(
  handlers: TelemetryStreamHandlers,
  path = "/api/v1/robot/telemetry/stream",
): ReconnectingSocket {
  return createReconnectingSocket(path, {
    onStatus: handlers.onStatus,
    onMessage: (data) => {
      if (typeof data !== "string") return;
      let msg: TelemetryMessage;
      try {
        msg = JSON.parse(data) as TelemetryMessage;
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
    },
  });
}
