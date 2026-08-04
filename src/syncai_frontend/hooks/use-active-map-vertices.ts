"use client";

import * as React from "react";

import { useActiveMap } from "@/hooks/use-maps";
import { listVertices } from "@/lib/api/vertex";
import type { MapVertex } from "@/lib/types/map";

export type ActiveVerticesStatus = "loading" | "ok" | "error" | "no-map";

export interface UseActiveMapVertices {
  /** The active map's directory name, or null when the robot has none loaded. */
  mapName: string | null;
  /** Sorted by name. Empty unless `status` is "ok". */
  vertices: MapVertex[];
  status: ActiveVerticesStatus;
}

/**
 * The active map's vertices, read-only — the destinations a MOVE step can be
 * prefilled from.
 *
 * Deliberately not useMapVertices, which the gridmap editor uses. Three reasons:
 * that hook needs a map name at first render and the active map's name only
 * exists after the catalogue answers, so `map?.name ?? ""` would fire a request
 * at /api/v1/maps//vertices and turn a normal first paint into a failure banner;
 * it exposes create / update / remove, and handing a task screen a vertex-delete
 * is an invitation; and its `error` is documented as shared between loads and
 * writes, half a contract with no writes here to fill it.
 *
 * There is no `refresh`. It inherits useMaps' fetch-once-per-mount policy, and
 * the App Router unmounts this page on navigation — so a vertex placed in the
 * editor is picked up on the next visit, which is every visit.
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

  return { mapName: name, vertices: current?.vertices ?? [], status };
}
