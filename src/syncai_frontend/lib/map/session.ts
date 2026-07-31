// Everything mutable that one loaded map owns.
//
// Created once by the loading hook, passed by reference, and never React state:
// the buffer is up to 2.4 MB and re-creating it to satisfy immutability would cost
// a full copy per stroke for something React cannot diff usefully anyway.
//
// `repaint` is the awkward part, and it is deliberate. Undo/redo lives in the
// editor shell (it owns the history and the toolbar's enabled states) but it has to
// reach the mirror canvas and the frame scheduler, which only GridCanvas knows
// about. The canvas installs `repaint` on mount and clears it on cleanup, so the
// slot's lifetime is exactly the canvas's. The alternative — useImperativeHandle
// plus a second ref that can be null at a different time than this one — is one
// more thing to null-check for no gain.

import type { CellRect, MapGrid } from "@/lib/map/grid";
import type { MapMetadata } from "@/lib/types/robot";
import {
  createGridMirror,
  disposeGridMirror,
  blitGrid,
  type GridMirror,
} from "@/lib/map/render";
import { StrokeAccumulator } from "@/lib/map/patch";

/**
 * Distinguishes two sessions for the same map name — which happens on every
 * strict-mode double mount. The editor keys its stateful surface on this, so
 * per-session state (history, cell counts) is initialised by mounting rather than
 * by resetting it in an effect.
 */
let nextSessionId = 1;

export interface GridSession {
  id: number;
  name: string;
  /** Resolution / origin / extent, for the world-coordinate readout. */
  meta: MapMetadata;
  grid: MapGrid;
  mirror: GridMirror;
  /** Scratch for the stroke accumulator; allocated once per session. */
  accumulator: StrokeAccumulator;
  /** Installed by GridCanvas. Pass a rect to repaint part of the mirror. */
  repaint: ((dirty?: CellRect) => void) | null;
}

export function createGridSession(
  name: string,
  meta: MapMetadata,
  grid: MapGrid,
): GridSession {
  const mirror = createGridMirror(grid);
  blitGrid(mirror, grid);
  nextSessionId += 1;
  return {
    id: nextSessionId,
    name,
    meta,
    grid,
    mirror,
    accumulator: new StrokeAccumulator(grid, new Uint8Array(grid.width * grid.height)),
    repaint: null,
  };
}

export function disposeGridSession(session: GridSession): void {
  session.repaint = null;
  disposeGridMirror(session.mirror);
}
