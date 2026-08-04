// Client for the vertex half of the map router
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/map.py).
//
// Separate from lib/api/map.ts even though both address /api/v1/maps/{name}/,
// because the two halves have nothing in common at the wire level: that file is
// entirely about the read/write asymmetry of the gridmap — a PNG in, raw cell
// bytes out, with a hand-rolled decode that must not be colour-managed — while
// this is plain JSON CRUD. Merging them would leave one file whose header has to
// explain both, and the binary rationale is the one that gets skimmed.
//
// The owning map is always in the URL and never in a body. The backend's request
// models deliberately have no `map_name` field, so a request cannot name a
// different map than the path it was posted to; this client keeps that property
// by taking `name` as its own first argument.

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";
import { normalizeTheta } from "@/lib/api/task";
import type { MapVertex, VertexType } from "@/lib/types/map";

/** What the operator supplies; `id` and `map_name` come from the server/URL. */
export interface VertexDraft {
  name: string;
  type: VertexType;
  x: number;
  y: number;
  /** Degrees. Normalised to (-180, 180] on the way out. */
  theta: number;
}

/** Fields to change. Omitted ones are left alone by the backend's `exclude_unset`. */
export type VertexChanges = Partial<VertexDraft>;

function vertexPath(name: string, id?: string): string {
  const base = `/api/v1/maps/${encodeURIComponent(name)}/vertices`;
  return apiUrl(id ? `${base}/${encodeURIComponent(id)}` : base);
}

export function listVertices(
  name: string,
  signal?: AbortSignal,
): Promise<MapVertex[]> {
  return requestJson<MapVertex[]>(vertexPath(name), { signal });
}

/**
 * Create one vertex.
 *
 * The endpoint is a *batch* one — it takes an array and answers with an array —
 * and this wraps a single draft rather than exposing that. The console places
 * one vertex per gesture, so a list-taking client would have every caller
 * writing `[draft]` and `result[0]`, and the day a real batch UI exists it can
 * call `requestJson` directly rather than inherit a shape nothing used.
 */
export async function createVertex(
  name: string,
  draft: VertexDraft,
): Promise<MapVertex> {
  const created = await requestJson<MapVertex[]>(vertexPath(name), {
    method: "POST",
    body: JSON.stringify([{ ...draft, theta: normalizeTheta(draft.theta) }]),
  });

  // One in, one out. A response that does not hold exactly that is the backend
  // having changed shape, and failing here beats returning `undefined` typed as
  // a MapVertex and crashing wherever it is first dereferenced.
  const vertex = created[0];
  if (!vertex) throw new Error("The backend accepted the vertex but returned none.");
  return vertex;
}

export function updateVertex(
  name: string,
  id: string,
  changes: VertexChanges,
): Promise<MapVertex> {
  return requestJson<MapVertex>(vertexPath(name, id), {
    method: "PUT",
    body: JSON.stringify(
      changes.theta === undefined
        ? changes
        : { ...changes, theta: normalizeTheta(changes.theta) },
    ),
  });
}

/**
 * Delete a vertex. The `{message}` envelope is dropped: it says nothing the
 * caller does not already know from the request having succeeded.
 */
export function deleteVertex(name: string, id: string): Promise<void> {
  return requestJson<void>(vertexPath(name, id), {
    method: "DELETE",
    parse: false,
  });
}
