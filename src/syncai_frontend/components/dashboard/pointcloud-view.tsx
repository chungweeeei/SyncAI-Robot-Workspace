"use client";

import * as React from "react";

import { Segmented, overlayPanel } from "@/components/console/instrument";
import { GoalControl } from "@/components/dashboard/goal-control";
import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { useGoalTask } from "@/hooks/use-goal-task";
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
  connecting: "Connecting",
  open: "Cloud live",
  closed: "Cloud down",
  error: "Cloud error",
};

const CAMERA_OPTIONS = [
  { value: "move" as const, label: "Move" },
  { value: "focus" as const, label: "Focus" },
];

/**
 * Data-wiring wrapper for the 3D point-cloud viewer: loads the map image/info
 * once for the ground plane, subscribes the telemetry WebSocket for pose and
 * joint angles, and hosts the map-cloud toggle. The live body_cloud stream
 * itself is owned by PointCloudCanvas.
 *
 * Goal dispatch shares `useGoalTask` and `GoalControl` with the 2D view, so a
 * goal placed here goes out as the same one-step MOVE task; only the picking
 * (a ground-plane raycast rather than grid pixels) is specific to 3D.
 */
export function PointCloudView({
  robotId,
  className,
}: {
  robotId: string;
  className?: string;
}) {
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

  const task = useGoalTask(robotId);

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
  // polling GET /api/v1/robot/state every 500 ms: that endpoint's timestamp has
  // whole-second resolution and it is a polled, frozen third-party contract, so
  // no amount of client-side polling or easing could make the motion continuous.
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
        goal={task.goal}
        goalMode={task.goalMode}
        onGoalCommit={task.commitGoal}
        onStatus={setStatus}
      />

      {/* Stream health for the cloud itself. The status strip's sweep covers the
        * 1 Hz state poll; this WebSocket is a separate link that can fail on its
        * own, so it gets its own indicator — in the same three tones. */}
      <div
        className={cn(
          overlayPanel,
          "absolute top-3 right-3 flex items-center gap-2 px-2 py-1.5",
        )}
      >
        <span
          className={cn(
            "inline-block size-2 rounded-full",
            status === "open"
              ? "bg-signal-live"
              : status === "connecting"
                ? "bg-signal-caution"
                : "bg-signal-warn",
          )}
        />
        <span className="instrument-label text-muted-foreground">
          {STATUS_LABEL[status]}
        </span>
      </div>

      <GoalControl task={task} className="absolute top-3 left-3" />

      {/* Viewport controls sit along the bottom edge, out of the way of the
        * goal readback and of the robot, which the camera keeps centred. */}
      <div className="absolute bottom-3 left-3 flex items-center gap-2">
        <Segmented
          value={cameraMode}
          options={CAMERA_OPTIONS}
          onChange={setCameraMode}
          className={overlayPanel}
        />
        <button
          type="button"
          aria-pressed={showMapCloud}
          onClick={() => setShowMapCloud((v) => !v)}
          className={cn(
            overlayPanel,
            "instrument-label h-6 px-2 transition-colors",
            showMapCloud
              ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
              : "text-muted-foreground hover:bg-elevated hover:text-foreground",
          )}
        >
          Map cloud
        </button>
      </div>
    </div>
  );
}
