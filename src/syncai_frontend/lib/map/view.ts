// The editor's view transform, and the grid <-> world transforms.
//
// One rule holds this together, inherited from the 2D map canvas this replaces:
// there is exactly one transform, and draw() and every pointer handler read the
// same one. Anything that needs to turn a screen position into a cell (or back)
// goes through a function in here rather than deriving its own scale/offset.
//
// Every value is in CSS pixels — the unit `event.clientX` gives you. Device pixel
// ratio belongs in the canvas's `setTransform` and nowhere else; folding it in
// here would silently break every hit test.

import type { GridSize } from "@/lib/map/grid";
import type { MapMetadata } from "@/lib/types/robot";

/** Uniform scale plus the grid origin's offset inside the viewport, in CSS px. */
export interface View {
  scale: number;
  ox: number;
  oy: number;
}

export interface Size {
  width: number;
  height: number;
}

/** One cell = 32 CSS px. Enough to click a single 5 cm cell without hunting. */
export const MAX_SCALE = 32;

/** Cell gridlines appear at and above this scale; below it they are noise. */
export const CELL_GRID_MIN_SCALE = 8;

/**
 * ROS world coordinates -> grid pixel coordinates (y flipped: canvas rows
 * grow downward while the ROS grid's row 0 sits at the map origin).
 */
export function worldToGrid(wx: number, wy: number, meta: MapMetadata) {
  return {
    px: (wx - meta.origin[0]) / meta.resolution,
    py: meta.height - (wy - meta.origin[1]) / meta.resolution,
  };
}

/** Inverse of worldToGrid; turns a cell back into a pose in the map frame. */
export function gridToWorld(px: number, py: number, meta: MapMetadata) {
  return {
    wx: px * meta.resolution + meta.origin[0],
    wy: (meta.height - py) * meta.resolution + meta.origin[1],
  };
}

/** The scale at which the whole grid just fits the viewport. */
export function fitScale(rect: Size, size: GridSize): number {
  return Math.min(rect.width / size.width, rect.height / size.height);
}

/** Uniformly scaled and centred — the view the editor opens at. */
export function fitView(rect: Size, size: GridSize): View {
  const scale = fitScale(rect, size);
  return {
    scale,
    ox: (rect.width - size.width * scale) / 2,
    oy: (rect.height - size.height * scale) / 2,
  };
}

export function screenToGrid(view: View, cx: number, cy: number) {
  return { px: (cx - view.ox) / view.scale, py: (cy - view.oy) / view.scale };
}

export function gridToScreen(view: View, px: number, py: number) {
  return { cx: px * view.scale + view.ox, cy: py * view.scale + view.oy };
}

/** The cell under a container-local point, or null outside the grid. */
export function cellAt(view: View, size: GridSize, cx: number, cy: number) {
  const { px, py } = screenToGrid(view, cx, cy);
  const col = Math.floor(px);
  const row = Math.floor(py);
  if (col < 0 || col >= size.width || row < 0 || row >= size.height) return null;
  return { col, row };
}

/**
 * Keep the map reachable, not pinned.
 *
 * The rule is that the map must still straddle the viewport centre: its near edge
 * may not be dragged past the centre line on either axis. That leaves the map
 * always grabbable — there is no way to fling it off screen and lose it — while
 * still allowing a real pan at every zoom level.
 *
 * This replaced a stricter rule ("an edge may reach but never pass the viewport
 * edge, and an axis where the map fits is force-centred"), which had a bug that
 * looked like a broken feature: the editor opens at `fitScale`, at fit the map by
 * definition fits at least one axis and exactly fills the other, and zooming out
 * is floored at `fitScale` — so on the opening view *both* axes were force-centred
 * and dragging with the Pan tool was a guaranteed no-op. Operators reasonably
 * concluded panning did not work. Letting the map overscroll is what every image
 * and map editor does, and it costs only the guarantee that the map is always
 * flush to an edge, which nothing here needed.
 *
 * Note the consequence: `clampView` no longer re-centres anything. Whatever wants
 * a centred map has to say so — `fitView` does, and `0` / the Fit button is the
 * explicit way back to it.
 */
export function clampView(view: View, rect: Size, size: GridSize): View {
  return {
    scale: view.scale,
    ox: clampAxis(view.ox, rect.width, size.width * view.scale),
    oy: clampAxis(view.oy, rect.height, size.height * view.scale),
  };
}

/**
 * One axis of the straddle rule: the map's far edge cannot come short of the
 * viewport's midpoint, and its near edge cannot pass it.
 *
 * `extent` is the scaled map's length on this axis. When it is smaller than the
 * viewport the window is `extent` wide, which is still a usable amount of travel
 * and never zero — the case the previous force-centring turned into a dead drag.
 */
function clampAxis(offset: number, viewport: number, extent: number): number {
  const mid = viewport / 2;
  return Math.min(mid, Math.max(mid - extent, offset));
}

/**
 * Zoom by `factor` about a container-local point, keeping the grid position
 * under that point fixed.
 *
 * The lower bound is fitScale rather than some constant: it stops the map being
 * shrunk into a corner of the viewport, and it also bounds the downscale factor
 * to ~2x, which is the range the browser's filtered `drawImage` handles without
 * losing 1-cell-thick walls (see grid-canvas.tsx on imageSmoothingEnabled).
 */
export function zoomAt(
  view: View,
  cx: number,
  cy: number,
  factor: number,
  rect: Size,
  size: GridSize,
): View {
  const min = fitScale(rect, size);
  const scale = Math.min(Math.max(view.scale * factor, min), MAX_SCALE);
  const ratio = scale / view.scale;
  return clampView(
    { scale, ox: cx - (cx - view.ox) * ratio, oy: cy - (cy - view.oy) * ratio },
    rect,
    size,
  );
}

export function panBy(view: View, dx: number, dy: number, rect: Size, size: GridSize): View {
  return clampView({ scale: view.scale, ox: view.ox + dx, oy: view.oy + dy }, rect, size);
}

/**
 * Re-clamp a view for a new viewport size, holding the grid point that was at
 * the old viewport centre.
 *
 * Deliberately not a refit: a window resize (or opening a devtools pane) must not
 * throw away the zoom the operator set. "Fit" is the explicit way back.
 */
export function reanchorView(view: View, from: Size, to: Size, size: GridSize): View {
  const centre = screenToGrid(view, from.width / 2, from.height / 2);
  const min = fitScale(to, size);
  const scale = Math.min(Math.max(view.scale, min), MAX_SCALE);
  return clampView(
    {
      scale,
      ox: to.width / 2 - centre.px * scale,
      oy: to.height / 2 - centre.py * scale,
    },
    to,
    size,
  );
}
