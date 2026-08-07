import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { queryKeys } from "@/lib/api/query-keys";
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
 * The list lives in the query cache under the map's name. The key does two
 * jobs: it is the old `loaded.name === name` race guard (a response or a
 * write's echo that resolves after the editor moved to another map lands under
 * its own key and cannot patch the list now on screen), and it is shared with
 * useActiveMapVertices, so an edit made here is already current on the
 * dashboard — see lib/api/query-keys.ts.
 */
export function useMapVertices(name: string): UseMapVertices {
  const queryClient = useQueryClient();
  const query = useQuery({
    queryKey: queryKeys.mapVertices(name),
    queryFn: ({ signal }) => listVertices(name, signal),
  });

  const [writeError, setWriteError] = React.useState<string | null>(null);
  const [busy, setBusy] = React.useState(false);

  /** Rewrite this map's cached list; a no-op until the load has answered. */
  const setVertices = React.useCallback(
    (next: (current: MapVertex[]) => MapVertex[]) => {
      queryClient.setQueryData<MapVertex[]>(
        queryKeys.mapVertices(name),
        (current) => (current ? next(current) : current),
      );
    },
    [queryClient, name],
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
    setWriteError(null);
    try {
      return await action();
    } catch (cause) {
      setWriteError(cause instanceof Error ? cause.message : String(cause));
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

  const clearError = React.useCallback(() => setWriteError(null), []);

  return {
    vertices: query.data ?? [],
    status: query.isPending ? "loading" : query.isError ? "error" : "ok",
    // Shared slot preserved from before the migration: the panel has exactly
    // one place to put a sentence, and the two cases are never live at once (a
    // map whose list failed to load has no vertices to edit). The load half now
    // clears itself — a successful refetch resets the query's error.
    error: writeError ?? query.error?.message ?? null,
    busy,
    create,
    update,
    remove,
    clearError,
  };
}
