// The map catalogue shape, as GET /api/v1/maps returns it. snake_case for the
// same reason lib/types/robot.ts is: it mirrors the backend's field names, so
// the client in lib/api/map.ts is a spread rather than a rename table.
//
// Two fields are *not* verbatim, and lib/api/map.ts is where they are fixed up:
// `grid.origin` arrives as {x, y, yaw} and becomes MapMetadata's tuple, and
// `thumbnail` arrives as a path and is made absolute against the backend's host.
//
// One map is one directory under the workspace's `map/`, as produced by the
// FAST-LIO2 PGO save (`map.pcd` + `poses.txt` + `patches/`) and then, separately,
// by tools/pcd_to_gridmap.py (`gridmap.pgm` + `gridmap.yaml`). Those are two
// steps a human runs at two different times, which is why `grid` is nullable.

import type { MapMetadata } from "@/lib/types/robot";

export interface MapSummary {
  /** Directory name under `map/` — the identity everything else keys off. */
  name: string;
  /**
   * Whether this is the map the running stack loaded.
   *
   * Server-derived on purpose. The only existing source of truth is
   * `RobotState.map`, which carries the raw INI value — a *path* like
   * `map/dp2f/gridmap.yaml` — while `map_vertices.map_name` uses bare names
   * (`dp2f`). Reconciling those two spellings is the backend's job; the UI must
   * not be the place that knows how to parse a map path.
   */
  active: boolean;
  /**
   * The gridmap.yaml fields, or null when the map has been saved from PGO but
   * `pcd_to_gridmap.py` has not been run over it yet. A map in that state cannot
   * be loaded by the nav stack, so the UI has to say so rather than show a hole.
   */
  grid: MapMetadata | null;
  /** `<img src>` for the map preview; null whenever `grid` is null. */
  thumbnail: string | null;
  /** `map.pcd` present — the cloud the 3D localizer relocalizes against. */
  has_pointcloud: boolean;
  /** Size of the whole `map/<name>/` directory, dominated by the .pcd. */
  size_bytes: number;
  /** ISO 8601, most recently modified file in the directory. */
  modified_at: string;
  /** Rows in `map_vertices` naming this map. */
  vertex_count: number;
}
