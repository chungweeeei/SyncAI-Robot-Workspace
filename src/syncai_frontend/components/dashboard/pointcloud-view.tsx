"use client";

import * as React from "react";

import { Segmented, overlayPanel } from "@/components/console/instrument";
import { GoalControl } from "@/components/dashboard/goal-control";
import { InitialPoseControl } from "@/components/dashboard/initial-pose-control";
import {
  PointCloudCanvas,
  type PickMode,
} from "@/components/dashboard/pointcloud-canvas";
import { useGoalTask } from "@/hooks/use-goal-task";
import { useInitialPose } from "@/hooks/use-initial-pose";
import { useActiveMap } from "@/hooks/use-maps";
import { apiUrl } from "@/lib/api/config";
import { createTelemetryStream } from "@/lib/ros/telemetry-stream";
import { cn } from "@/lib/utils";
import type { PlanarPose, RobotPose } from "@/lib/types/robot";
import type { StreamStatus } from "@/lib/types/pointcloud";

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
 * Data-wiring wrapper for the 3D point-cloud viewer — the console's only
 * viewport since the 2D grid canvas was removed. Resolves the active map from
 * the catalogue for the ground plane, subscribes the telemetry WebSocket for
 * pose and joint angles, and hosts the map-cloud toggle. The live body_cloud
 * stream itself is owned by PointCloudCanvas.
 *
 * It also owns the pick mode. A drag on the ground can mean two things — a nav
 * goal or an initial-pose estimate — and the gesture is the same for both, so
 * exactly one may be armed at a time. Keeping that in one piece of state here
 * (rather than a boolean inside each flow's hook) is what makes arming one
 * disarm the other by construction; two booleans would eventually both be true.
 */
export function PointCloudView({
  robotId,
  className,
}: {
  robotId: string;
  className?: string;
}) {
  // Which map the stack loaded, and therefore which one's raster to lay under
  // the cloud. A failure or an unconverted map leaves activeMap.grid null and
  // the canvas renders the cloud with no ground plane, same as before.
  const { map: activeMap } = useActiveMap();
  const mapImageUrl = React.useMemo(
    () =>
      activeMap?.grid
        ? apiUrl(`/api/v1/maps/${encodeURIComponent(activeMap.name)}/image`)
        : undefined,
    [activeMap],
  );
  const [pose, setPose] = React.useState<RobotPose | undefined>(undefined);
  const [joints, setJoints] = React.useState<
    Record<string, number> | undefined
  >(undefined);
  const [status, setStatus] = React.useState<StreamStatus>("connecting");
  const [showMapCloud, setShowMapCloud] = React.useState(false);
  const [cameraMode, setCameraMode] = React.useState<"move" | "focus">("move");
  const [pickMode, setPickMode] = React.useState<PickMode | null>(null);

  const task = useGoalTask(robotId);
  const estimate = useInitialPose();

  const armPick = React.useCallback(
    (mode: PickMode) => setPickMode((cur) => (cur === mode ? null : mode)),
    [],
  );

  // Destructured because they are the stable parts of the hooks' return objects
  // (the objects themselves are fresh every render, so depending on those would
  // rebuild the callback on every telemetry frame).
  const { commitPose } = estimate;
  const { commitGoal } = task;

  // Single-shot, like RViz's nav-goal / pose-estimate tools: one drag, one pose,
  // then the mode disarms so a stray click on the map cannot restage it.
  const commitPick = React.useCallback(
    (picked: PlanarPose) => {
      if (pickMode === "initial-pose") commitPose(picked);
      else commitGoal(picked);
      setPickMode(null);
    },
    [pickMode, commitPose, commitGoal],
  );

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
        meta={activeMap?.grid ?? undefined}
        mapImageUrl={mapImageUrl}
        mapName={activeMap?.name}
        pose={pose}
        joints={joints}
        showMapCloud={showMapCloud}
        cameraMode={cameraMode}
        goal={task.goal}
        initialPose={estimate.pose}
        pickMode={pickMode}
        onPickCommit={commitPick}
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

      {/* Both pose tools in one column, goal first: it is the one used on every
        * run, while an initial pose is a recovery action. */}
      <div className="absolute top-3 left-3 flex w-56 flex-col gap-2">
        <GoalControl
          task={task}
          armed={pickMode === "goal"}
          onArm={() => armPick("goal")}
        />
        <InitialPoseControl
          estimate={estimate}
          armed={pickMode === "initial-pose"}
          onArm={() => armPick("initial-pose")}
        />
      </div>

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
