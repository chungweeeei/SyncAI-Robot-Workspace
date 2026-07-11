import type { MapMetadata, OccupancyGrid, Vertex } from "@/lib/types/robot";

// Real values from map/warehouse.yaml + warehouse.pgm (439x641 cells @ 0.05 m)
export const mockMapMetadata: MapMetadata = {
  resolution: 0.05,
  origin: [-11.362341, -7.165529, 0.0],
  width: 439,
  height: 641,
};

// Copied from map/vertexes.json
export const mockVertexes: Vertex[] = [
  { name: "conveyor01_point", pose: { x: 5.95, y: -2.14, theta: -87.6 } },
  { name: "dropoff_b", pose: { x: 6.97, y: 9.0, theta: 90.0 } },
  { name: "conveyor02_point", pose: { x: -6.16, y: -2.0, theta: -93.3 } },
  { name: "dropoff_a", pose: { x: -7.12, y: 9.74, theta: 88.0 } },
  { name: "patrol_a", pose: { x: 3.0, y: 0.0, theta: 0.0 } },
  { name: "patrol_b", pose: { x: 3.0, y: 8.86, theta: 90.0 } },
  { name: "patrol_c", pose: { x: 2.4, y: 20.0, theta: 90.0 } },
  { name: "patrol_d", pose: { x: -2.8, y: 19.5, theta: -180.0 } },
  { name: "patrol_e", pose: { x: -3.18, y: 10.5, theta: 0.0 } },
  { name: "patrol_f", pose: { x: -2.0, y: 0.0, theta: 0.0 } },
];

const FREE = 0;
const OCCUPIED = 100;
const UNKNOWN = -1;

// Rectangular shelf/rack obstacles in world coordinates [x0, y0, x1, y1],
// placed clear of every vertex in mockVertexes
const SHELVES: Array<[number, number, number, number]> = [
  [-9.0, 2.0, -5.0, 3.0],
  [-9.0, 5.0, -5.0, 6.0],
  [-9.0, 12.0, -5.0, 13.0],
  [0.0, 12.0, 4.0, 13.0],
  [5.0, 2.0, 9.0, 3.0],
  [5.0, 5.0, 9.0, 6.0],
  [-1.0, 16.0, 3.0, 17.0],
];

/**
 * Procedural mock occupancy grid, ROS convention: row-major from the
 * bottom-left cell (row 0 = world y at origin), 0 free / 100 occupied / -1 unknown.
 */
export function generateMockGrid(meta: MapMetadata): OccupancyGrid {
  const { width, height, resolution, origin } = meta;
  const grid = new Int8Array(width * height).fill(FREE);

  const toCol = (wx: number) =>
    Math.round((wx - origin[0]) / resolution);
  const toRow = (wy: number) =>
    Math.round((wy - origin[1]) / resolution);

  const fillRect = (
    x0: number,
    y0: number,
    x1: number,
    y1: number,
    value: number,
  ) => {
    const c0 = Math.max(0, toCol(x0));
    const c1 = Math.min(width - 1, toCol(x1));
    const r0 = Math.max(0, toRow(y0));
    const r1 = Math.min(height - 1, toRow(y1));
    for (let r = r0; r <= r1; r++) {
      for (let c = c0; c <= c1; c++) {
        grid[r * width + c] = value;
      }
    }
  };

  // Unknown band beyond the mapped area at the top of the warehouse
  const worldTop = origin[1] + height * resolution;
  fillRect(origin[0], 22.5, origin[0] + width * resolution, worldTop, UNKNOWN);

  // Outer walls (0.15 m thick)
  const wx0 = origin[0] + 0.5;
  const wy0 = origin[1] + 0.5;
  const wx1 = origin[0] + width * resolution - 0.5;
  const wy1 = 22.0;
  fillRect(wx0, wy0, wx1, wy0 + 0.15, OCCUPIED);
  fillRect(wx0, wy1 - 0.15, wx1, wy1, OCCUPIED);
  fillRect(wx0, wy0, wx0 + 0.15, wy1, OCCUPIED);
  fillRect(wx1 - 0.15, wy0, wx1, wy1, OCCUPIED);

  for (const [x0, y0, x1, y1] of SHELVES) {
    fillRect(x0, y0, x1, y1, OCCUPIED);
  }

  return grid;
}
