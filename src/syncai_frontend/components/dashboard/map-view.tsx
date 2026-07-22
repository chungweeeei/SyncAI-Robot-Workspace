"use client";

import * as React from "react";

import { MapCanvas } from "@/components/dashboard/map-canvas";
import { apiUrl } from "@/lib/api/config";
import type { MapMetadata, RobotPose, Vertex } from "@/lib/types/robot";

// Backend GET /api/v1/map/image: MapInfoResponse fields + a base64 PNG data URI
// (data:image/png;base64,...) already rendered upright by the server.
interface MapImageResponse {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number; z: number };
  image: string;
}

// Backend GET /api/v1/map/vertices: x/y/theta live at the top level.
interface VertexResponse {
  id: string;
  name: string;
  type: string;
  map_name: string;
  x: number;
  y: number;
  theta: number;
}

/**
 * Data-wiring wrapper for the 2D map: pulls the occupancy map (as a PNG image +
 * metadata) and the waypoint vertices from the backend, then hands them to the
 * pure-rendering MapCanvas. The live pose still comes from the parent.
 */
export function MapView({ pose }: { pose: RobotPose }) {
  const [map, setMap] = React.useState<{
    meta: MapMetadata;
    image: string;
  } | null>(null);
  const [vertexes, setVertexes] = React.useState<Vertex[]>([]);

  // Map image + metadata (once).
  React.useEffect(() => {
    const abort = new AbortController();
    fetch(apiUrl("/api/v1/map/image"), { signal: abort.signal })
      .then((res) => {
        if (!res.ok) throw new Error(`map image: ${res.status}`);
        return res.json() as Promise<MapImageResponse>;
      })
      .then((data) => {
        setMap({
          meta: {
            resolution: data.resolution,
            width: data.width,
            height: data.height,
            origin: [data.origin.x, data.origin.y, 0],
          },
          image: data.image,
        });
      })
      .catch(() => {
        // No map yet (map_server not up / no saved map): leave the loading
        // placeholder rather than crashing the panel.
      });
    return () => abort.abort();
  }, []);

  // Waypoint vertices (once).
  React.useEffect(() => {
    const abort = new AbortController();
    fetch(apiUrl("/api/v1/map/vertices"), { signal: abort.signal })
      .then((res) => {
        if (!res.ok) throw new Error(`vertices: ${res.status}`);
        return res.json() as Promise<VertexResponse[]>;
      })
      .then((data) => {
        setVertexes(
          data.map((v) => ({
            name: v.name,
            pose: { x: v.x, y: v.y, theta: v.theta },
          })),
        );
      })
      .catch(() => {
        /* no vertices; render the map without waypoints */
      });
    return () => abort.abort();
  }, []);

  if (!map) {
    return (
      <div className="flex h-full w-full items-center justify-center text-sm text-muted-foreground">
        Loading map…
      </div>
    );
  }

  return (
    <MapCanvas
      meta={map.meta}
      mapImageUrl={map.image}
      pose={pose}
      vertexes={vertexes}
    />
  );
}
