"use client";

import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { useActiveMap } from "@/hooks/use-maps";
import { queryKeys } from "@/lib/api/query-keys";
import { listVertices, updateVertex } from "@/lib/api/vertex";
import type { MapVertex } from "@/lib/types/map";
import type { PlanarPose } from "@/lib/types/robot";

export type ActiveVerticesStatus = "loading" | "ok" | "error" | "no-map";

export interface UseActiveMapVertices {
  /** The active map's directory name, or null when the robot has none loaded. */
  mapName: string | null;
  /** Sorted by name. Empty unless `status` is "ok". */
  vertices: MapVertex[];
  status: ActiveVerticesStatus;
  /** True while a re-place write is in flight. */
  busy: boolean;
  /** The last re-place failure, or null. Rendered verbatim. */
  writeError: string | null;
  /**
   * Move one vertex to a new pose. True when the row is stored.
   *
   * Position only — name and type are not writable here (see the note on this
   * hook about why the rest of the CRUD surface stays out of it).
   */
  moveVertex: (id: string, pose: PlanarPose) => Promise<boolean>;
  clearWriteError: () => void;
}

/**
 * The active map's vertices: the destinations a MOVE step can be prefilled from,
 * the markers the dashboard draws on the floor, and — through `moveVertex` — the
 * one field those consumers may write.
 *
 * Deliberately not useMapVertices, which the gridmap editor uses. That hook
 * needs a map name at first render, and the active map's name only exists after
 * the catalogue answers, so `map?.name ?? ""` would fire a request at
 * /api/v1/maps//vertices and turn a normal first paint into a failure banner
 * (`enabled` below is what holds that request). It also exposes create /
 * remove, and a task screen with a vertex-delete in reach is an invitation.
 *
 * Position is the exception, and only because the dashboard is where the mistake
 * is *visible*: a stop drawn half a metre inside a wall is obvious with the live
 * cloud drawn over it and invisible on the editor's flat raster, so requiring a
 * trip to another screen to nudge it would mean fixing it from the one view that
 * cannot see the problem. Name and type are not part of that argument and stay
 * where the rest of the CRUD lives.
 *
 * `writeError` is separate from `status` rather than shared the way
 * useMapVertices shares its `error`: here the two really can be live at once —
 * the list loads fine and a re-place fails — and a failed write must not make
 * the layer read as unloaded.
 *
 * There is no `refresh`. The cache entry is the same one the gridmap editor
 * writes through (see lib/api/query-keys.ts), so an edit made there is current
 * here the moment it lands; and the App Router unmounts this page on
 * navigation, so a fresh mount refetches anyway — which is every visit.
 */
export function useActiveMapVertices(): UseActiveMapVertices {
  const { map, status: mapsStatus } = useActiveMap();
  const name = map?.name ?? null;
  const queryClient = useQueryClient();

  const query = useQuery({
    queryKey: queryKeys.mapVertices(name ?? ""),
    queryFn: ({ signal }) => listVertices(name ?? "", signal),
    enabled: name !== null,
    // Sorted for the picker — the list endpoint answers in DB order, and a
    // dropdown whose order changes between mounts is unusable — but sorted in
    // `select`, per observer, so the shared cache entry keeps the editor's DB
    // order and its append-on-create semantics.
    select: sortByName,
  });

  const [busy, setBusy] = React.useState(false);
  const [writeError, setWriteError] = React.useState<string | null>(null);

  const moveVertex = React.useCallback(
    async (id: string, pose: PlanarPose) => {
      if (!name) return false;
      setBusy(true);
      setWriteError(null);
      try {
        const updated = await updateVertex(name, id, {
          x: pose.x,
          y: pose.y,
          theta: pose.theta,
        });
        // Patched into the cache from the row the server echoes back, for the
        // same reason useMapVertices splices: the response *is* the stored row,
        // so a GET would cost a round trip to learn nothing. Written under the
        // name the request was made against, so a write that lands after the
        // active map changed patches the list it belongs to — the keyed cache
        // is the old `loaded.name === name` guard.
        queryClient.setQueryData<MapVertex[]>(
          queryKeys.mapVertices(name),
          (current) =>
            current?.map((vertex) => (vertex.id === id ? updated : vertex)),
        );
        return true;
      } catch (cause) {
        setWriteError(
          cause instanceof Error ? cause.message : "Failed to move the vertex.",
        );
        return false;
      } finally {
        setBusy(false);
      }
    },
    [name, queryClient],
  );

  const clearWriteError = React.useCallback(() => setWriteError(null), []);

  const status: ActiveVerticesStatus =
    mapsStatus === "loading"
      ? "loading"
      : mapsStatus === "error"
        ? "error"
        : !name
          ? "no-map"
          : query.isPending
            ? "loading"
            : query.isError
              ? "error"
              : "ok";

  return {
    mapName: name,
    vertices: query.data ?? [],
    status,
    busy,
    writeError,
    moveVertex,
    clearWriteError,
  };
}

/** Module-level so `select` keeps one identity and the sort is not re-run per render. */
function sortByName(vertices: MapVertex[]): MapVertex[] {
  return [...vertices].sort((a, b) => a.name.localeCompare(b.name));
}
