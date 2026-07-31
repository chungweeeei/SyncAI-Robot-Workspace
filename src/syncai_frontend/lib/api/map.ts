// Client for the map catalogue.
//
// ┌─ MOCK ─────────────────────────────────────────────────────────────────────┐
// │ There is no backend behind this file yet. The backend serves the *loaded*  │
// │ map only (GET /api/v1/map/info | /image | /pointcloud, all sourced from    │
// │ live ROS topics with no map name in the request); nothing enumerates the   │
// │ `map/` directory, and no map image is reachable per name over HTTP.        │
// │                                                                            │
// │ fetchMaps() and fetchMapGrid() therefore resolve lib/mock/. When the       │
// │ catalogue lands they become:                                               │
// │                                                                            │
// │   GET /api/v1/maps                     -> MapSummary[]                     │
// │   GET /api/v1/maps/{name}/thumbnail    -> PNG, referenced as the summary's  │
// │                                           `thumbnail` (apiUrl()-absolute)  │
// │   GET /api/v1/maps/{name}/gridmap      -> the raw .pgm bytes, through a     │
// │                                           parsePgm() that does not exist    │
// │                                           yet (see the plan: it has to      │
// │                                           tolerate `#` header comments,     │
// │                                           because GIMP writes one)          │
// │   PUT /api/v1/maps/{name}/gridmap      -> saveMapGrid, which is a stub      │
// │                                           today because nothing in the      │
// │                                           stack can persist an edited grid: │
// │                                           nav2_msgs/SaveMap takes a *topic  │
// │                                           name*, not grid data.             │
// │                                                                            │
// │ and this file is the only one that changes — no component imports the mock. │
// │ Deliberately no try-real-then-fall-back-to-mock path: two code paths means  │
// │ the page looks healthy against a backend that is down.                     │
// └────────────────────────────────────────────────────────────────────────────┘

import type { MapSummary } from "@/lib/types/map";
import type { MapGrid } from "@/lib/map/grid";
import { mockMaps } from "@/lib/mock/maps";
import { mockGrid } from "@/lib/mock/map-grid";

/** Enough delay that the skeleton state is real and gets exercised in dev. */
const MOCK_LATENCY_MS = 350;

function mockLatency<T>(produce: () => T, signal?: AbortSignal): Promise<T> {
  // Honours `signal` the way the real fetch will, so the hook teardown paths are
  // the same before and after the swap.
  return new Promise((resolve, reject) => {
    if (signal?.aborted) {
      reject(new DOMException("Aborted", "AbortError"));
      return;
    }
    const timer = setTimeout(() => resolve(produce()), MOCK_LATENCY_MS);
    signal?.addEventListener("abort", () => {
      clearTimeout(timer);
      reject(new DOMException("Aborted", "AbortError"));
    });
  });
}

export function fetchMaps(signal?: AbortSignal): Promise<MapSummary[]> {
  return mockLatency(() => mockMaps, signal);
}

export interface MapGridResponse {
  summary: MapSummary;
  grid: MapGrid;
}

/**
 * One map's editable grid.
 *
 * Rejects for a name that is not in the catalogue and for a map that has no
 * gridmap yet — the editor turns both into a guard screen rather than opening on
 * an empty canvas.
 */
export function fetchMapGrid(
  name: string,
  signal?: AbortSignal,
): Promise<MapGridResponse> {
  return mockLatency(() => {
    const summary = mockMaps.find((map) => map.name === name);
    if (!summary) throw new Error(`No map named "${name}" on this robot.`);
    if (!summary.grid) {
      throw new Error(
        `"${name}" has no gridmap. Run tools/pcd_to_gridmap.py over its map.pcd first.`,
      );
    }
    return {
      summary,
      grid: mockGrid(name, summary.grid.width, summary.grid.height),
    };
  }, signal);
}

/**
 * Persist an edited grid.
 *
 * Stub, in the shape of the network-settings save button: there is no endpoint,
 * and there cannot be one built on the existing ROS services — `SaveMap` re-reads
 * a topic instead of accepting an array. Wiring this up is the backend round's
 * job, together with the `_raw` backup convention already visible on disk.
 */
export async function saveMapGrid(name: string, grid: MapGrid): Promise<void> {
  console.log("save gridmap (stub)", name, grid.width, "x", grid.height);
  await mockLatency(() => undefined);
}
