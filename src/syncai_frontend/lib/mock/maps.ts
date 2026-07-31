// Mock map catalogue, standing in for a backend endpoint that does not exist yet.
//
// Same approach as the earlier lib/mock/map.ts: real numbers copied off disk, the
// pixels generated. Every field below except the thumbnails was read from
// map/<name>/gridmap.yaml, the .pgm header, and `du -sh` on 2026-07-31, so the
// page is laid out against the extents and aspect ratios it will actually get.
//
// Thumbnails are procedural SVG rather than the real gridmap_preview.png files:
// `map/` is gitignored deliberately, and committing downscaled copies would put
// facility layouts in the repo to make a mock look good.

import type { MapSummary } from "@/lib/types/map";
import { layoutSeed, mapLayout } from "@/lib/mock/map-layout";

/**
 * A stand-in gridmap image at the map's true cell extent, so the card's tile is
 * exercised with the real aspect ratio.
 *
 * The floor comes from the shared layout in lib/mock/map-layout.ts, which the
 * editor rasterizes to actual grid bytes — so the thumbnail and the editable grid
 * are the same map. Palette is the nav2 / .pgm convention the real preview uses,
 * because the card is built around it: unknown 205 grey ground, free space white,
 * obstacles near black. See MapThumbnail for why that matters in dark mode.
 */
function gridThumbnail(width: number, height: number, name: string): string {
  const { hall, wall, racks } = mapLayout(width, height, layoutSeed(name));

  // Racks are outlined with an unknown-grey interior, matching how the editor
  // rasterizes them and how a lidar actually sees a rack: faces observed, volume
  // behind them never scanned.
  const parts = [
    `<rect width="${width}" height="${height}" fill="#cdcdcd"/>`,
    `<rect x="${hall.x + wall / 2}" y="${hall.y + wall / 2}" ` +
      `width="${hall.w - wall}" height="${hall.h - wall}" ` +
      `fill="#ffffff" stroke="#1f1f1f" stroke-width="${wall}"/>`,
    ...racks.map(
      (r) =>
        `<rect x="${r.x + wall / 2}" y="${r.y + wall / 2}" ` +
        `width="${r.w - wall}" height="${r.h - wall}" ` +
        `fill="#cdcdcd" stroke="#1f1f1f" stroke-width="${wall}"/>`,
    ),
  ];

  const svg =
    `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" ` +
    `height="${height}" viewBox="0 0 ${width} ${height}">${parts.join("")}</svg>`;

  return `data:image/svg+xml,${encodeURIComponent(svg)}`;
}

/**
 * `dp2f` is active because that is what `[map] name` in
 * config/instances/robot01.ini points at.
 *
 * `dp3f` is the one entry with no counterpart on disk. It is here to hold the
 * no-gridmap state open while the page is built: that is what every map looks
 * like between `pgo/save_maps` and `pcd_to_gridmap.py`, and it is what `dp2f`
 * itself looked like until recently. Drop this entry when fetchMaps() goes real.
 */
export const mockMaps: MapSummary[] = [
  {
    name: "dp1f",
    active: false,
    grid: {
      resolution: 0.05,
      origin: [-19.412418, -42.546829, 0.0],
      width: 1602,
      height: 1502,
    },
    thumbnail: gridThumbnail(1602, 1502, "dp1f"),
    has_pointcloud: true,
    size_bytes: 45 * 1024 * 1024,
    modified_at: "2026-07-30T17:49:00Z",
    vertex_count: 12,
  },
  {
    name: "dp2f",
    active: true,
    grid: {
      resolution: 0.05,
      origin: [-6.940549, -11.097282, 0.0],
      width: 1613,
      height: 993,
    },
    thumbnail: gridThumbnail(1613, 993, "dp2f"),
    has_pointcloud: true,
    size_bytes: 43 * 1024 * 1024,
    modified_at: "2026-07-31T10:35:00Z",
    vertex_count: 8,
  },
  {
    name: "warehouse01",
    active: false,
    grid: {
      resolution: 0.05,
      origin: [-11.244558, -10.697074, 0.0],
      width: 468,
      height: 703,
    },
    thumbnail: gridThumbnail(468, 703, "warehouse01"),
    has_pointcloud: true,
    size_bytes: 40 * 1024 * 1024,
    modified_at: "2026-07-15T13:27:00Z",
    vertex_count: 10,
  },
  {
    name: "dp3f",
    active: false,
    grid: null,
    thumbnail: null,
    has_pointcloud: true,
    size_bytes: 28 * 1024 * 1024,
    modified_at: "2026-07-31T09:12:00Z",
    vertex_count: 0,
  },
];
