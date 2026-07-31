// Undo/redo for grid edits. Pure: no React, no DOM.
//
// A patch is a **sparse cell list**, not a dirty rectangle. The rect form is the
// obvious choice and it is wrong: a diagonal drag across a 1600-cell-wide map with
// a size-5 brush changes ~5 000 cells but has a ~1000x1000 bounding box, so a rect
// patch spends 2 MB recording 5 000 edits. Sparse costs 6 bytes per changed cell —
// it loses by about 3x on a genuinely dense edit (a 200x200 rect fill: 240 KB vs
// 80 KB) and wins by ~66x on the common one, and its worst case is bounded by the
// work actually done rather than by how far the pointer travelled.
//
// Full-buffer snapshots were the other alternative: 2.4 MB each on the largest map,
// so a useful history would be hundreds of megabytes on a Jetson.

import {
  OCCUPIED,
  UNKNOWN,
  classify,
  type CellRect,
  type CellSink,
  type GridValue,
  type MapGrid,
  type ValueCounts,
} from "@/lib/map/grid";

/** One undoable edit: every cell one stroke changed, with both sides recorded. */
export interface GridPatch {
  /** Row-major indices (row * width + col), in stamp order. */
  cells: Uint32Array;
  /** Parallel to `cells`: the byte before the stroke. */
  before: Uint8Array;
  /** Parallel to `cells`: the byte after it. */
  after: Uint8Array;
  /**
   * Union bounding box of `cells` — repaint metadata only. It says which region
   * of the mirror canvas to re-blit; it never defines the patch's extent.
   */
  bounds: CellRect;
}

export function patchBytes(patch: GridPatch): number {
  return patch.cells.length * 6;
}

/**
 * Write one side of a patch back into the grid.
 *
 * Undo and redo are the same function with a different side, which is the whole
 * reason `after` is stored explicitly instead of being recomputed: there is no
 * replay, no base state, and no asymmetry to get wrong.
 */
export function applyPatch(
  grid: MapGrid,
  patch: GridPatch,
  side: "before" | "after",
): void {
  const values = side === "before" ? patch.before : patch.after;
  const { cells } = patch;
  const { data } = grid;
  for (let i = 0; i < cells.length; i += 1) {
    data[cells[i]] = values[i];
  }
}

const INITIAL_CAPACITY = 4096;

/**
 * Collects one stroke as it is drawn.
 *
 * Its `sink` is what the stamp primitives in lib/map/grid.ts write through, so a
 * stroke's bookkeeping costs one function call per painted cell and nothing else.
 */
export class StrokeAccumulator {
  private readonly grid: MapGrid;
  /**
   * One byte per cell, allocated once for the editor's lifetime and cleared over
   * the stroke's bounds at commit. A Map<index, value> would be simpler, but a
   * "paint the whole map free" stroke would then be 2.4 M Map entries and >100 MB.
   */
  private readonly touched: Uint8Array;

  private cells: Uint32Array = new Uint32Array(INITIAL_CAPACITY);
  private before: Uint8Array = new Uint8Array(INITIAL_CAPACITY);
  private after: Uint8Array = new Uint8Array(INITIAL_CAPACITY);
  private count = 0;

  // Two rects, and both are needed. `bounds` accumulates over the whole stroke
  // and becomes the patch's repaint metadata; `frame` is taken and reset after
  // every blit. With only the first, a long drag re-blits an ever-growing region
  // on every frame — quadratic in stroke length, and the first thing you notice.
  private bounds: MutableRect | null = null;
  private frame: MutableRect | null = null;

  constructor(grid: MapGrid, touched: Uint8Array) {
    this.grid = grid;
    this.touched = touched;
  }

  readonly sink: CellSink = (index: number, value: GridValue) => {
    const { data } = this.grid;
    const current = data[index];
    // Painting a value onto itself is not an edit. Without this guard a stroke
    // over already-white space would push a patch that changes nothing, and the
    // next Ctrl+Z would appear to do nothing at all.
    if (current === value) return;

    if (!this.touched[index]) {
      this.touched[index] = 1;
      if (this.count === this.cells.length) this.grow();
      this.cells[this.count] = index;
      this.before[this.count] = current;
      this.after[this.count] = value;
      this.count += 1;
    } else {
      // A later pass of the same stroke over a cell it already changed: keep the
      // original `before` (first write wins, so undo restores the true pre-stroke
      // byte) but track the final value.
      this.after[this.indexOf(index)] = value;
    }

    data[index] = value;

    const col = index % this.grid.width;
    const row = (index - col) / this.grid.width;
    this.bounds = grow(this.bounds, col, row);
    this.frame = grow(this.frame, col, row);
  };

  get isEmpty(): boolean {
    return this.count === 0;
  }

  /** Cells changed since the last call — the region to blit into the mirror. */
  takeFrameDirty(): CellRect | null {
    const rect = this.frame;
    this.frame = null;
    return rect ? toRect(rect) : null;
  }

  /** Freeze the stroke. Returns null when it changed nothing. */
  commit(): GridPatch | null {
    if (!this.bounds || this.count === 0) {
      this.reset();
      return null;
    }

    const bounds = toRect(this.bounds);
    const patch: GridPatch = {
      cells: this.cells.slice(0, this.count),
      before: this.before.slice(0, this.count),
      after: this.after.slice(0, this.count),
      bounds,
    };
    this.reset();
    return patch;
  }

  /** Clear `touched` over the stroke's rows only, so a stroke costs what it drew. */
  private reset(): void {
    if (this.bounds) {
      const { col, row, w, h } = toRect(this.bounds);
      for (let r = row; r < row + h; r += 1) {
        const base = r * this.grid.width;
        this.touched.fill(0, base + col, base + col + w);
      }
    }
    this.count = 0;
    this.bounds = null;
    this.frame = null;
  }

  /**
   * Linear scan back through the stroke's own log. Only reached when a stroke
   * repaints a cell it already changed with a *different* value, which needs both
   * an overlapping pass and a mid-stroke value change — rare enough that an index
   * map (another 9.6 MB) would cost more than it saves.
   */
  private indexOf(index: number): number {
    for (let i = this.count - 1; i >= 0; i -= 1) {
      if (this.cells[i] === index) return i;
    }
    return this.count - 1;
  }

  private grow(): void {
    const size = this.cells.length * 2;
    const cells = new Uint32Array(size);
    const before = new Uint8Array(size);
    const after = new Uint8Array(size);
    cells.set(this.cells);
    before.set(this.before);
    after.set(this.after);
    this.cells = cells;
    this.before = before;
    this.after = after;
  }
}

interface MutableRect {
  minCol: number;
  minRow: number;
  maxCol: number;
  maxRow: number;
}

function grow(rect: MutableRect | null, col: number, row: number): MutableRect {
  if (!rect) return { minCol: col, minRow: row, maxCol: col, maxRow: row };
  if (col < rect.minCol) rect.minCol = col;
  if (col > rect.maxCol) rect.maxCol = col;
  if (row < rect.minRow) rect.minRow = row;
  if (row > rect.maxRow) rect.maxRow = row;
  return rect;
}

function toRect(rect: MutableRect): CellRect {
  return {
    col: rect.minCol,
    row: rect.minRow,
    w: rect.maxCol - rect.minCol + 1,
    h: rect.maxRow - rect.minRow + 1,
  };
}

/**
 * Move a cell census across a patch.
 *
 * Incremental because the alternative is a full pass over up to 2.4 M cells after
 * every stroke, and a stroke can be a single dab.
 */
export function applyCountsDelta(
  counts: ValueCounts,
  patch: GridPatch,
  side: "before" | "after",
): ValueCounts {
  const from = side === "before" ? patch.after : patch.before;
  const to = side === "before" ? patch.before : patch.after;
  const next = { ...counts };

  for (let i = 0; i < from.length; i += 1) {
    bump(next, classify(from[i]), -1);
    bump(next, classify(to[i]), 1);
  }
  return next;
}

function bump(counts: ValueCounts, value: GridValue, by: number): void {
  if (value === OCCUPIED) counts.occupied += by;
  else if (value === UNKNOWN) counts.unknown += by;
  else counts.free += by;
}

export interface UndoStack {
  undo: GridPatch[];
  redo: GridPatch[];
  /** Running total of both stacks, maintained incrementally. */
  bytes: number;
}

/**
 * Capped by bytes rather than entries: a single dab is ~1 KB and a full-map fill
 * is ~14 MB, so an entry count cannot bound memory. The entry ceiling is a second
 * guard so a thousand tiny dabs cannot grow the array without limit.
 */
export const UNDO_BUDGET_BYTES = 32 * 1024 * 1024;
export const UNDO_MAX_ENTRIES = 100;

export function createUndoStack(): UndoStack {
  return { undo: [], redo: [], bytes: 0 };
}

/** Push a new edit. Clears the redo stack — a new edit discards the redone future. */
export function pushPatch(stack: UndoStack, patch: GridPatch): void {
  stack.bytes += patchBytes(patch);
  for (const dropped of stack.redo) stack.bytes -= patchBytes(dropped);
  stack.redo = [];
  stack.undo.push(patch);

  while (
    stack.undo.length > UNDO_MAX_ENTRIES ||
    (stack.bytes > UNDO_BUDGET_BYTES && stack.undo.length > 1)
  ) {
    const evicted = stack.undo.shift();
    if (!evicted) break;
    stack.bytes -= patchBytes(evicted);
  }
}

export function popUndo(stack: UndoStack): GridPatch | null {
  const patch = stack.undo.pop();
  if (!patch) return null;
  stack.redo.push(patch);
  return patch;
}

export function popRedo(stack: UndoStack): GridPatch | null {
  const patch = stack.redo.pop();
  if (!patch) return null;
  stack.undo.push(patch);
  return patch;
}
