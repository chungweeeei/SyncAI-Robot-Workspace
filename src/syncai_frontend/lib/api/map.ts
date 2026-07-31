// Client for the map catalogue: GET /api/v1/maps and everything under
// /api/v1/maps/{name}/.
//
// The backend speaks snake_case and nests `origin` as an object; the app's
// MapMetadata is a tuple. The wire types below are the former, the exported
// functions return the latter, and the translation happens in one place so no
// component has to know the difference.
//
// There is no /api/v1/maps/{name}/gridmap: the grid arrives as the PNG from
// /image, which is the same cells losslessly encoded, a third smaller than the
// P5 on the wire, and decodable by the browser instead of by a parser we would
// have to write and keep tolerant of GIMP's `#` comment line.

import { apiUrl } from "@/lib/api/config";
import type { MapGrid } from "@/lib/map/grid";
import type { MapSummary } from "@/lib/types/map";

/** `GridInfoResponse` — note `origin` is {x, y, yaw}, not a tuple. */
interface WireGrid {
  resolution: number;
  origin: { x: number; y: number; yaw: number };
  width: number;
  height: number;
}

/** `MapSummaryResponse`. `thumbnail` is a path, not an absolute URL. */
interface WireSummary {
  name: string;
  active: boolean;
  grid: WireGrid | null;
  thumbnail: string | null;
  has_pointcloud: boolean;
  size_bytes: number;
  modified_at: string;
  vertex_count: number;
}

function toSummary(wire: WireSummary): MapSummary {
  return {
    ...wire,
    grid: wire.grid
      ? {
          resolution: wire.grid.resolution,
          origin: [wire.grid.origin.x, wire.grid.origin.y, wire.grid.origin.yaw],
          width: wire.grid.width,
          height: wire.grid.height,
        }
      : null,
    // Absolute, because <img src> on the card resolves against the *frontend's*
    // origin (:3001) and the backend answers on :3000.
    thumbnail: wire.thumbnail ? apiUrl(wire.thumbnail) : null,
  };
}

/**
 * Fetch, or throw with the backend's own message.
 *
 * The domain-exception handlers return `{detail}`, and those strings are written
 * to be read by an operator ("Map 'x' has no gridmap. Run tools/pcd_to_gridmap.py
 * over its map.pcd first."). The hooks render `error.message` verbatim, so
 * unwrapping `detail` here is what puts the actionable half on screen instead of
 * a status code.
 */
async function request(path: string, signal?: AbortSignal): Promise<Response> {
  const response = await fetch(apiUrl(path), { signal });
  if (response.ok) return response;

  let detail: string | null = null;
  try {
    detail = ((await response.json()) as { detail?: string }).detail ?? null;
  } catch {
    // A non-JSON error body (a proxy's 502 page, say) leaves detail null.
  }
  throw new Error(detail ?? `${path} failed: ${response.status}`);
}

export async function fetchMaps(signal?: AbortSignal): Promise<MapSummary[]> {
  const response = await request("/api/v1/maps", signal);
  const wire = (await response.json()) as WireSummary[];
  return wire.map(toSummary);
}

export interface MapGridResponse {
  summary: MapSummary;
  grid: MapGrid;
}

/**
 * Decode the map PNG back to one byte per cell.
 *
 * `colorSpaceConversion: "none"` is load-bearing, not defensive. Canvas is RGBA
 * sRGB, and by default the browser is free to colour-manage a decoded image on
 * the way in — which for a greyscale PNG means the 205 that marks *unknown*
 * comes back as 204 or 206 and `classify` in lib/map/grid.ts reads a different
 * cell kind. It would still look like a map, which is exactly why it has to be
 * turned off here rather than noticed later.
 *
 * Row order needs no flip: the PNG carries the .pgm's rows in file order (row 0
 * is the top / max y), getImageData hands them back the same way, and that is
 * what MapGrid documents.
 */
async function decodeGrid(blob: Blob): Promise<MapGrid> {
  const bitmap = await createImageBitmap(blob, {
    colorSpaceConversion: "none",
    premultiplyAlpha: "none",
  });

  try {
    const { width, height } = bitmap;
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;

    const ctx = canvas.getContext("2d", { willReadFrequently: true });
    if (!ctx) throw new Error("Could not decode the map image (no 2D context).");
    ctx.drawImage(bitmap, 0, 0);

    const rgba = ctx.getImageData(0, 0, width, height).data;
    const data = new Uint8Array(width * height);
    // Greyscale source, so r === g === b; one channel is the cell value.
    for (let i = 0; i < data.length; i += 1) data[i] = rgba[i * 4];

    return { width, height, data };
  } finally {
    bitmap.close();
  }
}

/**
 * One map's editable grid, with the summary it belongs to.
 *
 * Two requests rather than one: the geometry lives in gridmap.yaml and the cells
 * in the .pgm, and the editor needs both. They are consistent because /image's
 * ETag is a hash of the same .pgm the summary's width/height were read from — if
 * a save lands between the two calls, the mismatch is visible rather than silent.
 *
 * Rejects for a name that is not in the catalogue and for a map that has no
 * gridmap yet; the editor turns both into a guard screen rather than opening on
 * an empty canvas.
 */
export async function fetchMapGrid(
  name: string,
  signal?: AbortSignal,
): Promise<MapGridResponse> {
  const encoded = encodeURIComponent(name);

  const summary = toSummary(
    (await (await request(`/api/v1/maps/${encoded}`, signal)).json()) as WireSummary,
  );
  if (!summary.grid) {
    throw new Error(
      `"${name}" has no gridmap. Run tools/pcd_to_gridmap.py over its map.pcd first.`,
    );
  }

  const image = await request(`/api/v1/maps/${encoded}/image`, signal);
  return { summary, grid: await decodeGrid(await image.blob()) };
}

/**
 * Persist an edited grid.
 *
 * Still a stub: there is no endpoint, and there cannot be one built on the
 * existing ROS services — `SaveMap` re-reads a topic instead of accepting an
 * array. The catalogue is read-only on the backend for the same reason. Wiring
 * this up is a backend round's job, together with the `_raw` backup convention
 * already visible on disk.
 */
export async function saveMapGrid(name: string, grid: MapGrid): Promise<void> {
  console.log("save gridmap (stub)", name, grid.width, "x", grid.height);
}
