// Mock gridmap bytes for the editor: the shared mock layout rasterized to the
// canonical .pgm values, at the map's real cell extent.
//
// Unlike the card's SVG thumbnail this is a real editable buffer with real byte
// values, so every stamp, patch and blit path is exercised against the same sizes
// the backend will eventually serve (up to 1602x1502 = 2.4 M cells). It goes away
// entirely when GET /api/v1/maps/{name}/gridmap lands — see lib/api/map.ts.

import { FREE, OCCUPIED, UNKNOWN, type MapGrid } from "@/lib/map/grid";
import { layoutSeed, mapLayout, type LayoutRect } from "@/lib/mock/map-layout";

function fill(grid: MapGrid, rect: LayoutRect, value: number): void {
  const colFrom = Math.max(rect.x, 0);
  const colTo = Math.min(rect.x + rect.w, grid.width);
  const rowFrom = Math.max(rect.y, 0);
  const rowTo = Math.min(rect.y + rect.h, grid.height);

  for (let row = rowFrom; row < rowTo; row += 1) {
    grid.data.fill(value, row * grid.width + colFrom, row * grid.width + colTo);
  }
}

/** Occupied border of `wall` cells, interior left as whatever it already is. */
function outline(grid: MapGrid, rect: LayoutRect, wall: number, interior: number): void {
  fill(grid, rect, OCCUPIED);
  fill(
    grid,
    { x: rect.x + wall, y: rect.y + wall, w: rect.w - 2 * wall, h: rect.h - 2 * wall },
    interior,
  );
}

/**
 * Unknown ground, a free hall inside a thin wall, and racks as thin outlines.
 *
 * Racks are outlined rather than filled because that is what a lidar map of one
 * looks like: the scanner sees the rack's faces and legs, and the volume behind
 * them is never observed, so it stays unknown. Filling them solid would also have
 * hidden the renderer's one real hazard — a map made only of thick blobs looks
 * fine however badly thin features are downscaled.
 *
 * Everything is drawn as filled bands rather than strokes: a stroke straddles its
 * boundary by half its width, and half a cell is not a thing.
 */
export function mockGrid(name: string, width: number, height: number): MapGrid {
  const grid: MapGrid = {
    width,
    height,
    data: new Uint8Array(width * height).fill(UNKNOWN),
  };

  const { hall, wall, racks } = mapLayout(width, height, layoutSeed(name));

  outline(grid, hall, wall, FREE);
  for (const rack of racks) outline(grid, rack, wall, UNKNOWN);

  return grid;
}
