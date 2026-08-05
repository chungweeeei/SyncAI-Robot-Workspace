import {
  createReconnectingSocket,
  type ReconnectingSocket,
} from "@/lib/ros/socket";
import type { PlannedPath, RobotPose } from "@/lib/types/robot";
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
 *   {"type": "path",   "points": [[x, y], ...], "stamp"}
 *
 * `path` shares this socket rather than getting its own: it is ~8 kB once per
 * replan (~0.333 Hz), nowhere near the size that made the point cloud a separate
 * connection.
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

interface PathMessage {
  type: "path";
  /** Map-frame [x, y] pairs, metres. Empty means "no route". */
  points: [number, number][];
  stamp: number;
}

type TelemetryMessage = PoseMessage | JointsMessage | PathMessage;

export interface TelemetryStreamHandlers {
  onPose?: (pose: RobotPose) => void;
  /** Joint angles in radians, keyed by URDF joint name. */
  onJoints?: (joints: Record<string, number>) => void;
  /** The planner's route. An empty `points` means the route is over. */
  onPath?: (path: PlannedPath) => void;
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
      } else if (msg.type === "path") {
        // Flattened here rather than in the consumer, for the same reason
        // yaw_deg is relabelled above: this is the wire-to-app translation
        // layer, and the pairs the JSON has to use are not the layout the
        // geometry builder walks.
        const points = new Float32Array(msg.points.length * 2);
        for (let i = 0; i < msg.points.length; i++) {
          points[i * 2] = msg.points[i][0];
          points[i * 2 + 1] = msg.points[i][1];
        }
        handlers.onPath?.({ points, stamp: msg.stamp });
      }
    },
  });
}
