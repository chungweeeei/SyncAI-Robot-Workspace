// The map catalogue shape, as GET /api/v1/maps returns it. snake_case for the
// same reason lib/types/robot.ts is: it mirrors the backend's field names, so
// the client in lib/api/map.ts is a spread rather than a rename table.
//
// Two fields are *not* verbatim, and lib/api/map.ts is where they are fixed up:
// `grid.origin` arrives as {x, y, yaw} and becomes MapMetadata's tuple, and
// `thumbnail` arrives as a path and is made absolute against the backend's host.
//
// One map is one directory under the workspace's `map/`, as produced by the
// FAST-LIO2 PGO save (`map.pcd` + `poses.txt` + `patches/`) and then by the
// backend's pcd → gridmap conversion (`gridmap.pgm` + `gridmap.yaml`), which
// runs in the background after the save. The conversion can lag or fail — the
// window where only the pcd exists is why `grid` is nullable.

import type { MapMetadata } from "@/lib/types/robot";

/**
 * How a map's 2D gridmap stands — the backend's `GridStatus`.
 *
 * The three terminal states that leave a map the nav stack cannot load are
 * deliberately separate, because they ask different things of the operator:
 *
 * - `none` — nobody has converted this map yet. Press Build grid.
 * - `failed` — the pipeline rejected this cloud, and `grid_error` says why.
 *   Read it before pressing anything; the same recipe will fail the same way.
 * - `interrupted` — the backend went away mid-conversion (a restart, or a mode
 *   switch, which tears down the session the backend is a pane of). Nothing is
 *   coming to finish it and a retry is very likely to just work.
 *
 * Before these existed the catalogue had one boolean, so all three rendered as
 * "No 2D grid" and the reason a conversion failed lived only in the robot's
 * backend log. That is the bug this union closes.
 */
export type GridStatus =
  | "none"
  | "converting"
  | "ok"
  | "failed"
  | "interrupted";

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
  /**
   * How this map's gridmap stands. The conversion's only status surface — the
   * backend keeps no job resource — so a client that starts one watches this
   * on the catalogue until it leaves `"converting"`.
   */
  grid_status: GridStatus;
  /**
   * Why the last conversion failed, as the pipeline diagnosed it. Null for
   * every status other than `"failed"`. Written to be shown to an operator, so
   * render it verbatim like the backend's other `detail` sentences.
   */
  grid_error: string | null;
  /**
   * @deprecated True exactly when `grid_status` is `"converting"`. The backend
   * keeps it for curl/MCP callers written against it; read `grid_status`, which
   * also tells a failed conversion apart from a map nobody converted yet.
   */
  grid_converting: boolean;
  /** Size of the whole `map/<name>/` directory, dominated by the .pcd. */
  size_bytes: number;
  /** ISO 8601, most recently modified file in the directory. */
  modified_at: string;
  /** Rows in `map_vertices` naming this map. */
  vertex_count: number;
}

/**
 * The two pcd → gridmap recipes the re-convert endpoint accepts.
 *
 * No "auto": the backend removed size-based recipe picking after it misrouted
 * every conference-hall save (glass inflates the cloud's bounding box), so a
 * request either takes the z-band default or names traversability outright.
 * z-band is trinary and recoverable; traversability permanently walls off
 * everything it did not observe, which is why choosing it is an operator act.
 */
export type GridRecipe = "z-band" | "traversability";

/**
 * What the robot does when it visits a vertex — the router's `VertexType`.
 *
 * A closed union rather than a string, because the backend validates it at the
 * REST boundary and an unknown value comes back as a 422 whose `detail` is a
 * validation *array*, not a sentence an operator can read. Keeping the UI's
 * choices a fixed set is what stops that from ever being rendered.
 *
 * The DB column is still called `MapPoint.type` and the table `map_vertices`;
 * "vertex" is the REST vocabulary. That mismatch is deliberate upstream — see
 * the backend's CLAUDE.md — and this file follows the wire, not the table.
 */
export type VertexType = "GENERAL" | "ARTIFACT" | "CHARGER" | "HOME" | "WAITING";

/** `MapVertexResponse`, verbatim. */
export interface MapVertex {
  /** Server-assigned uuid, serialised as a string. Unique across all maps. */
  id: string;
  name: string;
  type: VertexType;
  /** The bare directory name, the same spelling `MapSummary.name` uses. */
  map_name: string;
  /** Map frame, metres. */
  x: number;
  y: number;
  /**
   * Heading in **degrees**, CCW from +x — the same convention as `PlanarPose`
   * and the whole REST vocabulary. Not radians, despite the grid origin's third
   * component (`MapMetadata.origin[2]`) being one.
   */
  theta: number;
}
