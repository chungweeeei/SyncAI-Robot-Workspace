"use client";

import * as React from "react";

import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { apiUrl } from "@/lib/api/config";
import { createTelemetryStream } from "@/lib/ros/telemetry-stream";
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

const STATUS_LABEL: Record<StreamStatus, string> = {
  connecting: "Connecting…",
  open: "Live",
  closed: "Disconnected",
  error: "Error",
};

/**
 * Data-wiring wrapper for the 3D point-cloud viewer: loads the map image/info
 * once for the ground plane, subscribes the telemetry WebSocket for pose and
 * joint angles, and hosts the map-cloud toggle. The live body_cloud stream
 * itself is owned by PointCloudCanvas.
 */
export function PointCloudView({ className }: { className?: string }) {
  const [map, setMap] = React.useState<{
    meta: MapMetadata;
    image: string;
  } | null>(null);
  const [pose, setPose] = React.useState<RobotPose | undefined>(undefined);
  const [joints, setJoints] = React.useState<
    Record<string, number> | undefined
  >(undefined);
  const [status, setStatus] = React.useState<StreamStatus>("connecting");
  const [showMapCloud, setShowMapCloud] = React.useState(false);
  const [cameraMode, setCameraMode] = React.useState<"move" | "focus">("move");

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

  // Robot pose + joints via the telemetry WebSocket (~20 Hz map-frame pose
  // from odom, joints at the gait controller's telemetry rate). This replaced
  // polling GET /api/v1/robot/state every 500 ms: that endpoint's upstream
  // topic is a 1 Hz aggregate, so no amount of client-side polling or easing
  // could make the motion continuous — and it is a frozen third-party
  // contract, so raising its rate was not an option.
  React.useEffect(() => {
    const stream = createTelemetryStream({
      onPose: setPose,
      onJoints: setJoints,
    });
    return () => stream.close();
  }, []);

  return (
    <div className={cn("relative h-full w-full", className)}>
      <PointCloudCanvas
        meta={map?.meta}
        mapImageUrl={map?.image}
        pose={pose}
        joints={joints}
        showMapCloud={showMapCloud}
        cameraMode={cameraMode}
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

      <div className="absolute bottom-2 left-2 flex overflow-hidden rounded-md border bg-background/80 text-xs backdrop-blur">
        {(["move", "focus"] as const).map((mode) => (
          <button
            key={mode}
            type="button"
            onClick={() => setCameraMode(mode)}
            className={cn(
              "px-2 py-1 capitalize transition-colors",
              cameraMode === mode
                ? "bg-primary text-primary-foreground"
                : "hover:bg-accent",
            )}
          >
            {mode}
          </button>
        ))}
      </div>

      <button
        type="button"
        onClick={() => setShowMapCloud((v) => !v)}
        className={cn(
          "absolute bottom-2 right-2 rounded-md border px-2 py-1 text-xs backdrop-blur transition-colors",
          "bg-background/80 hover:bg-accent",
          showMapCloud && "border-primary text-primary",
        )}
      >
        {showMapCloud ? "Hide map cloud" : "Show map cloud"}
      </button>
    </div>
  );
}
