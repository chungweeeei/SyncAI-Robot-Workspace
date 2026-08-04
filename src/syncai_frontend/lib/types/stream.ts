// Shared vocabulary for the backend WebSocket streams (live point cloud,
// telemetry). It used to live in pointcloud.ts, back when the cloud was the
// only stream; both streams report it now and neither owns it. The state
// machine that emits these values is in lib/ros/socket.ts.

/** Connection state of a reconnecting backend WebSocket. */
export type StreamStatus = "connecting" | "open" | "closed" | "error";
