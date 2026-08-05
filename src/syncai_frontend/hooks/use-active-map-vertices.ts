"use client";

import * as React from "react";

import { useActiveMap } from "@/hooks/use-maps";
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
 * /api/v1/maps//vertices and turn a normal first paint into a failure banner. It
 * also exposes create / remove, and a task screen with a vertex-delete in reach
 * is an invitation.
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
 * There is no `refresh`. It inherits useMaps' fetch-once-per-mount policy, and
 * the App Router unmounts this page on navigation — so a vertex placed in the
 * editor is picked up on the next visit, which is every visit. A re-place done
 * here patches the loaded list in place from the row the server echoes back, for
 * the same reason useMapVertices does: the response *is* the stored row, so a
 * GET would cost a round trip to learn nothing.
 */
export function useActiveMapVertices(): UseActiveMapVertices {
  const { map, status: mapsStatus } = useActiveMap();
  const name = map?.name ?? null;

  /**
   * The list and the map it belongs to, stored together so a response for the
   * previous map can be told from one for the current map by comparing — rather
   * than by clearing state at the top of the effect, which is the cascading
   * render the compiler lint rejects. Same shape as useMapVertices' `loaded`.
   */
  const [loaded, setLoaded] = React.useState<{
    name: string;
    vertices: MapVertex[] | null;
  } | null>(null);

  React.useEffect(() => {
    if (!name) return;
    let active = true;
    const abort = new AbortController();

    listVertices(name, abort.signal)
      .then((vertices) => {
        if (!active) return;
        // Sorted here rather than in the picker: the list endpoint answers in DB
        // order, and a dropdown whose order changes between mounts is unusable.
        setLoaded({
          name,
          vertices: [...vertices].sort((a, b) => a.name.localeCompare(b.name)),
        });
      })
      .catch(() => {
        if (!active || abort.signal.aborted) return;
        setLoaded({ name, vertices: null });
      });

    return () => {
      active = false;
      abort.abort();
    };
  }, [name]);

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
        setLoaded((current) =>
          // Guarded against a write that lands after the active map changed:
          // the list it would patch is not the list this row belongs to.
          current?.name === name && current.vertices
            ? {
                name,
                vertices: current.vertices.map((vertex) =>
                  vertex.id === id ? updated : vertex,
                ),
              }
            : current,
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
    [name],
  );

  const clearWriteError = React.useCallback(() => setWriteError(null), []);

  const current = name && loaded?.name === name ? loaded : null;

  const status: ActiveVerticesStatus =
    mapsStatus === "loading"
      ? "loading"
      : mapsStatus === "error"
        ? "error"
        : !name
          ? "no-map"
          : current
            ? current.vertices
              ? "ok"
              : "error"
            : "loading";

  return {
    mapName: name,
    vertices: current?.vertices ?? [],
    status,
    busy,
    writeError,
    moveVertex,
    clearWriteError,
  };
}
