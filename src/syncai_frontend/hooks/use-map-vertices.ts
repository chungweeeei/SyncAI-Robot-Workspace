import * as React from "react";

import {
  createVertex,
  deleteVertex,
  listVertices,
  updateVertex,
  type VertexChanges,
  type VertexDraft,
} from "@/lib/api/vertex";
import type { MapVertex } from "@/lib/types/map";

export type MapVerticesStatus = "loading" | "ok" | "error";

export interface UseMapVertices {
  vertices: MapVertex[];
  status: MapVerticesStatus;
  /** The load failure, or the most recent write failure. Rendered verbatim. */
  error: string | null;
  /** True while a create / update / delete is in flight. */
  busy: boolean;
  /** The created vertex, or null if the request failed (see `error`). */
  create: (draft: VertexDraft) => Promise<MapVertex | null>;
  update: (id: string, changes: VertexChanges) => Promise<MapVertex | null>;
  /** True when the row is gone. */
  remove: (id: string) => Promise<boolean>;
  clearError: () => void;
}

/**
 * One map's vertices, written through to the backend on every change.
 *
 * There is deliberately no local dirty state and no Save button, unlike the
 * gridmap this hook sits beside. A gridmap is one 2.4 MB file replaced whole, so
 * buffering it and writing once is the only sane shape; vertices are individual
 * rows with per-id PUT/DELETE, and staging a diff to replay later would invent a
 * partial-failure state — half the edits landed, the UI has to say which — for
 * no benefit on a robot LAN. Each action is one request, and `error` describes
 * the last one.
 *
 * `error` is shared by the load and the writes on purpose: the panel has exactly
 * one place to put a sentence, and the two cases are never live at once (a map
 * whose list failed to load has no vertices to edit).
 */
export function useMapVertices(name: string): UseMapVertices {
  /**
   * The list and the name it belongs to, stored together so a result for the
   * previous map can be told from one for the current map by comparing — rather
   * than by clearing state at the top of the effect, which is a cascading render
   * the compiler lint rejects. Same shape as `useMapGrid`'s `Loaded`.
   */
  const [loaded, setLoaded] = React.useState<{
    name: string;
    vertices: MapVertex[] | null;
  } | null>(null);
  const [error, setError] = React.useState<string | null>(null);
  const [busy, setBusy] = React.useState(false);

  React.useEffect(() => {
    let active = true;
    const abort = new AbortController();

    listVertices(name, abort.signal)
      .then((vertices) => {
        if (!active) return;
        setLoaded({ name, vertices });
        setError(null);
      })
      .catch((cause: unknown) => {
        if (!active || abort.signal.aborted) return;
        setLoaded({ name, vertices: null });
        setError(
          cause instanceof Error ? cause.message : "Failed to load the map vertices.",
        );
      });

    return () => {
      active = false;
      abort.abort();
    };
  }, [name]);

  /**
   * Rewrite the current map's list, ignoring a write that resolved after the
   * editor moved to another map.
   */
  const setVertices = React.useCallback(
    (next: (current: MapVertex[]) => MapVertex[]) => {
      setLoaded((current) =>
        current?.vertices
          ? { name: current.name, vertices: next(current.vertices) }
          : current,
      );
    },
    [],
  );

  /**
   * Run one write, holding `busy` and turning a rejection into `error`.
   *
   * Rejections are swallowed rather than rethrown because every caller is wired
   * straight to an onClick — a rethrow would be an unhandled rejection, and the
   * panel already reads the outcome off `error` and the returned value.
   */
  const run = React.useCallback(async <T,>(action: () => Promise<T>): Promise<T | null> => {
    setBusy(true);
    setError(null);
    try {
      return await action();
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : String(cause));
      return null;
    } finally {
      setBusy(false);
    }
  }, []);

  const create = React.useCallback(
    async (draft: VertexDraft) => {
      const created = await run(() => createVertex(name, draft));
      // Appended rather than refetching the list: the response *is* the row the
      // server stored, so a GET would cost a round trip to learn nothing.
      if (created) setVertices((current) => [...current, created]);
      return created;
    },
    [name, run, setVertices],
  );

  const update = React.useCallback(
    async (id: string, changes: VertexChanges) => {
      const updated = await run(() => updateVertex(name, id, changes));
      if (updated) {
        setVertices((current) =>
          current.map((vertex) => (vertex.id === id ? updated : vertex)),
        );
      }
      return updated;
    },
    [name, run, setVertices],
  );

  const remove = React.useCallback(
    async (id: string) => {
      const done = await run(async () => {
        await deleteVertex(name, id);
        return true as const;
      });
      if (done) setVertices((current) => current.filter((vertex) => vertex.id !== id));
      return done === true;
    },
    [name, run, setVertices],
  );

  const clearError = React.useCallback(() => setError(null), []);

  const current = loaded?.name === name ? loaded : null;

  return {
    vertices: current?.vertices ?? [],
    status: current ? (current.vertices ? "ok" : "error") : "loading",
    error,
    busy,
    create,
    update,
    remove,
    clearError,
  };
}
