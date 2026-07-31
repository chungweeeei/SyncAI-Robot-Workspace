// The bridge from grid bytes to pixels. The only file under lib/map/ that touches
// the DOM.
//
// The mirror is a detached canvas holding the grid 1:1. It is a *mirror*, never a
// source: the Uint8Array is the sole truth, and nothing here ever calls
// getImageData. That keeps colour management and alpha premultiplication out of
// the trust path entirely — an edit is "mutate bytes, then blit", not a
// read-modify-write of pixels.
//
// A plain detached <canvas> rather than OffscreenCanvas: drawImage accepts both,
// the element needs no feature check, and OffscreenCanvas buys nothing without a
// worker (which would need SharedArrayBuffer and therefore COOP/COEP headers for
// the whole app, to avoid a cost we do not have — 2.4 M byte writes are single-digit
// milliseconds on the main thread).

import type { CellRect, MapGrid } from "@/lib/map/grid";

export interface GridMirror {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
}

export function createGridMirror(grid: MapGrid): GridMirror {
  const canvas = document.createElement("canvas");
  canvas.width = grid.width;
  canvas.height = grid.height;
  // alpha: false — every pixel written here is opaque, and it lets the compositor
  // skip a blend on every drawImage.
  const ctx = canvas.getContext("2d", { alpha: false });
  if (!ctx) throw new Error("2D canvas context unavailable");
  return { canvas, ctx };
}

/**
 * Copy a rectangle of grid bytes into the mirror as literal greyscale.
 *
 * `r = g = b = byte`, no palette, no theme. This is the one screen where the
 * operator has to be able to trust that the shade on screen is the byte that will
 * be written to the file: recolouring free space (say, cyan) would break the link
 * between "I see white" and "the planner reads 254 = free". Only the chrome drawn
 * over the grid — extent outline, brush ring, previews — follows the theme.
 *
 * A fresh ImageData per call is intentional. It is ~900 bytes for a size-15 dab and
 * 9.6 MB for the one full-grid blit at load (immediately released). A pooled scratch
 * tile would save nothing measurable and would add a stale-region failure mode.
 */
export function blitGridRect(
  mirror: GridMirror,
  grid: MapGrid,
  rect: CellRect,
): void {
  const col = Math.max(rect.col, 0);
  const row = Math.max(rect.row, 0);
  const w = Math.min(rect.w, grid.width - col);
  const h = Math.min(rect.h, grid.height - row);
  if (w <= 0 || h <= 0) return;

  const image = mirror.ctx.createImageData(w, h);
  const out = image.data;
  const { data, width } = grid;

  let o = 0;
  for (let r = 0; r < h; r += 1) {
    let i = (row + r) * width + col;
    for (let c = 0; c < w; c += 1, i += 1, o += 4) {
      const byte = data[i];
      out[o] = byte;
      out[o + 1] = byte;
      out[o + 2] = byte;
      out[o + 3] = 255;
    }
  }

  mirror.ctx.putImageData(image, col, row);
}

/** Blit the whole grid. Used once on load and after an undo of a huge patch. */
export function blitGrid(mirror: GridMirror, grid: MapGrid): void {
  blitGridRect(mirror, grid, { col: 0, row: 0, w: grid.width, h: grid.height });
}

/**
 * Release the backing store.
 *
 * A detached canvas's pixel buffer can outlive the last reference to the element,
 * and this one is up to 9.6 MB. Zeroing the dimensions frees it deterministically.
 */
export function disposeGridMirror(mirror: GridMirror): void {
  mirror.canvas.width = 0;
  mirror.canvas.height = 0;
}
