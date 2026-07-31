// The shape of a mock map, described once and rendered twice.
//
// The catalogue card renders this to an SVG thumbnail (lib/mock/maps.ts) and the
// editor rasterizes the same description to grid bytes (lib/mock/map-grid.ts).
// Sharing the description is the point: two generators would mean the card and the
// editor showed different-looking maps for the same name, which is exactly the kind
// of mock artefact that gets mistaken for a bug. When the backend serves real
// gridmap bytes, both consumers drop away together.

/** Cell coordinates, .pgm order (row 0 = top). */
export interface LayoutRect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface MapLayout {
  width: number;
  height: number;
  /** Free space, bounded by a wall of `wall` cells. */
  hall: LayoutRect;
  wall: number;
  /** Racks, drawn as outlines: lidar sees their faces, not their insides. */
  racks: LayoutRect[];
}

/**
 * Wall thickness in cells — fixed, not a fraction of the extent.
 *
 * At 0.05 m/cell a real wall is 2 to 3 cells, and every feature on a real gridmap
 * is that thin. Scaling thickness with the map (an earlier version did) gave dp1f
 * 18-cell walls, which made the mock useless for the thing it most needs to check:
 * whether thin obstacles survive being drawn at fit scale, where 1602 cells are
 * squeezed into ~900 CSS px.
 */
export const WALL_CELLS = 2;

/**
 * Deterministic PRNG — the mock must not reshuffle itself between renders.
 *
 * The seed goes through a murmur3 finalizer before the LCG, and that is not
 * decoration: seeds 1, 2, 3 fed straight into an LCG stay affinely related for
 * every draw that follows, so the first draw came out a straight line in `seed`
 * and all three maps picked the same rack count. Avalanche the seed once and the
 * layouts actually differ.
 */
export function lcg(seed: number): () => number {
  let state = seed >>> 0;
  state ^= state >>> 16;
  state = Math.imul(state, 2246822507) >>> 0;
  state ^= state >>> 13;
  state = Math.imul(state, 3266489909) >>> 0;
  state ^= state >>> 16;

  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

/**
 * A warehouse-ish floor at the map's true cell extent: a walled hall with rows of
 * racks either side of an aisle, which is the shape the real dp1f / dp2f maps have.
 */
export function mapLayout(width: number, height: number, seed: number): MapLayout {
  const rand = lcg(seed);
  const short = Math.min(width, height);
  const inset = Math.round(short * 0.09);
  const wall = WALL_CELLS;

  const hall: LayoutRect = {
    x: inset,
    y: inset,
    w: width - 2 * inset,
    h: height - 2 * inset,
  };

  const rows = 3 + Math.floor(rand() * 3);
  const rackHeight = Math.round((hall.h / rows) * 0.32);
  const left = Math.round(inset + short * 0.06);
  const right = Math.round(width - inset - short * 0.06);
  const gap = Math.round(short * 0.14);

  const racks: LayoutRect[] = [];
  for (let row = 0; row < rows; row += 1) {
    const y = Math.round(inset + ((row + 0.5) * hall.h) / rows);
    const aisle = Math.round(width * (0.42 + rand() * 0.12));
    racks.push({ x: left, y, w: aisle - left, h: rackHeight });
    racks.push({ x: aisle + gap, y, w: right - aisle - gap, h: rackHeight });
  }

  return { width, height, hall, wall, racks };
}

/** Stable per-map seed, so a name always produces the same floor. */
export function layoutSeed(name: string): number {
  let hash = 0;
  for (let i = 0; i < name.length; i += 1) {
    hash = (Math.imul(hash, 31) + name.charCodeAt(i)) >>> 0;
  }
  return hash || 1;
}
