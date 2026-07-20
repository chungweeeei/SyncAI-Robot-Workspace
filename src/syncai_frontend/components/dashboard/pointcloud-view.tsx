"use client";

import * as React from "react";

import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { apiUrl } from "@/lib/api/config";
import { cn } from "@/lib/utils";
import type { MapMetadata, RobotPose } from "@/lib/types/robot";
import type { StreamStatus } from "@/lib/types/pointcloud";

interface MapImagePayload {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number; z: number };
  image: string;
}

interface RobotStatePayload {
  map: string;
  localization_status: { position: RobotPose };
}

const POSE_POLL_MS = 500;

const STATUS_LABEL: Record<StreamStatus, string> = {
  connecting: "Connecting…",
  open: "Live",
  closed: "Disconnected",
  error: "Error",
};

/**
 * Data-wiring wrapper for the 3D point-cloud viewer: loads the map image/info
 * once for the ground plane, polls the robot pose, and hosts the map-cloud
 * toggle. The live body_cloud stream itself is owned by PointCloudCanvas.
 */
export function PointCloudView({ className }: { className?: string }) {
  const [map, setMap] = React.useState<{
    meta: MapMetadata;
    image: string;
  } | null>(null);
  const [pose, setPose] = React.useState<RobotPose | undefined>(undefined);
  const [status, setStatus] = React.useState<StreamStatus>("connecting");
  const [showMapCloud, setShowMapCloud] = React.useState(false);
  const [mapName, setMapName] = React.useState<string | null>(null);

  // Map image + metadata (once).
  React.useEffect(() => {
    const abort = new AbortController();
    fetch(apiUrl("/api/v1/map/image"), { signal: abort.signal })
      .then((res) => {
        if (!res.ok) throw new Error(`map image: ${res.status}`);
        return res.json() as Promise<MapImagePayload>;
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
        // No 2D map (e.g. a raw cloud test with no map_server): the canvas
        // still renders the point cloud without a ground plane.
      });
    return () => abort.abort();
  }, []);

  // Robot pose (polled).
  React.useEffect(() => {
    let active = true;
    const tick = async () => {
      try {
        const res = await fetch(apiUrl("/api/v1/robot/state"));
        if (!res.ok) return;
        const data = (await res.json()) as RobotStatePayload;
        if (!active) return;
        setPose(data.localization_status.position);
        setMapName(data.map);
      } catch {
        /* transient; keep last pose */
      }
    };
    tick();
    const id = setInterval(tick, POSE_POLL_MS);
    return () => {
      active = false;
      clearInterval(id);
    };
  }, []);

  return (
    <div className={cn("relative h-full w-full", className)}>
      <PointCloudCanvas
        meta={map?.meta}
        mapImageUrl={map?.image}
        pose={pose}
        mapCloudName={showMapCloud ? mapName ?? undefined : undefined}
        onStatus={setStatus}
      />

      <div className="absolute left-2 top-2 flex items-center gap-2 rounded-md bg-background/80 px-2 py-1 text-xs backdrop-blur">
        <span
          className={cn(
            "inline-block h-2 w-2 rounded-full",
            status === "open"
              ? "bg-green-500"
              : status === "connecting"
                ? "bg-amber-500"
                : "bg-red-500",
          )}
        />
        <span className="text-muted-foreground">{STATUS_LABEL[status]}</span>
      </div>

      <button
        type="button"
        onClick={() => setShowMapCloud((v) => !v)}
        disabled={!mapName}
        className={cn(
          "absolute bottom-2 right-2 rounded-md border px-2 py-1 text-xs backdrop-blur transition-colors",
          "bg-background/80 hover:bg-accent disabled:opacity-50",
          showMapCloud && "border-primary text-primary",
        )}
      >
        {showMapCloud ? "Hide map cloud" : "Show map cloud"}
      </button>
    </div>
  );
}
