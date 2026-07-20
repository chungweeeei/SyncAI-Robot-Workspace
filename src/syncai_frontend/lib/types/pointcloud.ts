// Types for the 3D point-cloud viewer. The wire format (both the live
// WebSocket stream and the static GET /api/v1/map/pointcloud) is:
//   [uint32 point_count][float32 x, y, z] * point_count   (little-endian)
// See src/syncai_backend/.../helpers/pointcloud.py (pack_xyz_f32) and
// routers/pointcloud.py for the producers.

/** A decoded point-cloud frame: xyz triplets in the map frame. */
export interface PointCloudFrame {
  /** number of points (positions.length / 3) */
  count: number;
  /** flat [x0, y0, z0, x1, y1, z1, ...] in map-frame metres */
  positions: Float32Array;
}

export type StreamStatus = "connecting" | "open" | "closed" | "error";
