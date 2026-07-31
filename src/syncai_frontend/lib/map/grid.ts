// The occupancy grid as the .pgm on disk holds it, plus the stamp primitives the
// editor paints with. Pure: no React, no DOM, no canvas.

/** Cell extent of a grid. */
export interface GridSize {
  width: number;
  height: number;
}

/**
 * A gridmap in .pgm byte order.
 *
 * Row 0 is the **top** of the map (max y) — the order the P5 file stores and the
 * order `pcd_to_gridmap.py` writes after its `np.flipud`. A ROS OccupancyGrid is
 * the other way up; the flip is already accounted for by worldToGrid/gridToWorld
 * in lib/map/view.ts, so nothing here has to think about y.
 */
export interface MapGrid extends GridSize {
  data: Uint8Array;
}

export interface Cell {
  col: number;
  row: number;
}

export interface CellRect {
  col: number;
  row: number;
  w: number;
  h: number;
}

/**
 * The only three bytes this editor writes.
 *
 * These are not thresholds. syncai_map_server's loader (map_io.cpp, with the
 * `negate: 0` / `occupied_thresh: 0.65` / `free_thresh: 0.196` every gridmap.yaml
 * in this repo carries) classifies a byte `g` as occupied for `g <= 89`, unknown
 * for `90..205`, and free for `g >= 206`. So 255 loads as free too — and two of
 * the real maps contain 255s from a previous round of hand-editing in GIMP. Read
 * by range (`classify`), write 254, and the file stays byte-identical in kind to
 * what the conversion tool produces.
 */
export const OCCUPIED = 0;
export const UNKNOWN = 205;
export const FREE = 254;

export type GridValue = typeof OCCUPIED | typeof UNKNOWN | typeof FREE;

/** What the loader will make of a byte. */
export function classify(byte: number): GridValue {
  if (byte <= 89) return OCCUPIED;
  if (byte <= 205) return UNKNOWN;
  return FREE;
}

/**
 * Brush diameters, in cells.
 *
 * Odd only. An even diameter has no centre cell, so it is either off-centre by
 * half a cell or ambiguous about which side gets the extra row — not a tolerable
 * question on a tool whose whole job is deciding about individual 5 cm cells.
 */
export const BRUSH_SIZES = [1, 3, 7, 15, 31] as const;

/**
 * Where a stamp's cells go.
 *
 * The indirection is the seam the whole editor hangs off: the undo accumulator
 * plugs in here to record before/after, and the line and rect previews plug in a
 * collector so they can rasterize the exact cells they are about to commit
 * without touching the buffer.
 */
export type CellSink = (index: number, value: GridValue) => void;

export function cellIndex(size: GridSize, col: number, row: number): number {
  return row * size.width + col;
}

export function inBounds(size: GridSize, col: number, row: number): boolean {
  return col >= 0 && col < size.width && row >= 0 && row < size.height;
}

/**
 * A filled disc of the given diameter centred on (col, row).
 *
 * The `r * r + r` radius fudge is deliberate: a plain `dx² + dy² <= r²` renders
 * diameter 3 as a plus sign, which reads as a bug when you paint with it. The
 * fudge gives the 3×3 square a paint program draws for a small round brush, and
 * a recognisable disc from diameter 7 up.
 */
export function stampDisc(
  size: GridSize,
  col: number,
  row: number,
  diameter: number,
  value: GridValue,
  sink: CellSink,
): void {
  const r = (diameter - 1) / 2;
  const rr = r * r + r;

  const rowFrom = Math.max(-r, -row);
  const rowTo = Math.min(r, size.height - 1 - row);

  for (let dy = rowFrom; dy <= rowTo; dy += 1) {
    const span = r === 0 ? 0 : Math.floor(Math.sqrt(Math.max(rr - dy * dy, 0)));
    const colFrom = Math.max(-span, -col);
    const colTo = Math.min(span, size.width - 1 - col);
    const base = (row + dy) * size.width + col;
    for (let dx = colFrom; dx <= colTo; dx += 1) {
      sink(base + dx, value);
    }
  }
}

/**
 * A disc swept along the integer line from a to b — the Minkowski sum of the
 * line with the brush, i.e. exactly what a round brush leaves behind.
 *
 * This is also what a freehand drag uses between consecutive pointer samples: a
 * fast flick at high zoom puts 100+ CSS px between samples, and without
 * interpolation the stroke comes out dashed. Sharing one rasterizer between the
 * brush and the line tool is what keeps them from ever disagreeing.
 */
export function stampLine(
  size: GridSize,
  from: Cell,
  to: Cell,
  diameter: number,
  value: GridValue,
  sink: CellSink,
): void {
  let col = from.col;
  let row = from.row;
  const dCol = Math.abs(to.col - col);
  const dRow = Math.abs(to.row - row);
  const stepCol = col < to.col ? 1 : -1;
  const stepRow = row < to.row ? 1 : -1;
  let error = dCol - dRow;

  // Bresenham, stamping the brush at every step. Redundant writes where
  // consecutive discs overlap are absorbed by the sink (a repaint of the same
  // value is not an edit).
  for (;;) {
    stampDisc(size, col, row, diameter, value, sink);
    if (col === to.col && row === to.row) return;
    const e2 = 2 * error;
    if (e2 > -dRow) {
      error -= dRow;
      col += stepCol;
    }
    if (e2 < dCol) {
      error += dCol;
      row += stepRow;
    }
  }
}

/** Filled axis-aligned rectangle, clipped to the grid. Corners in either order. */
export function stampRect(
  size: GridSize,
  from: Cell,
  to: Cell,
  value: GridValue,
  sink: CellSink,
): void {
  const colFrom = Math.max(Math.min(from.col, to.col), 0);
  const colTo = Math.min(Math.max(from.col, to.col), size.width - 1);
  const rowFrom = Math.max(Math.min(from.row, to.row), 0);
  const rowTo = Math.min(Math.max(from.row, to.row), size.height - 1);

  for (let row = rowFrom; row <= rowTo; row += 1) {
    const base = row * size.width;
    for (let col = colFrom; col <= colTo; col += 1) {
      sink(base + col, value);
    }
  }
}

export interface ValueCounts {
  occupied: number;
  unknown: number;
  free: number;
}

/** Cell census, for the status bar. One pass; called on load and after a stroke. */
export function countValues(grid: MapGrid): ValueCounts {
  let occupied = 0;
  let unknown = 0;
  const { data } = grid;
  for (let i = 0; i < data.length; i += 1) {
    const byte = data[i];
    if (byte <= 89) occupied += 1;
    else if (byte <= 205) unknown += 1;
  }
  return { occupied, unknown, free: data.length - occupied - unknown };
}
