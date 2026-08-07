"use client";

import * as React from "react";

import { createTelemetryStream } from "@/lib/ros/telemetry-stream";
import type { PlannedPath, RobotPose } from "@/lib/types/robot";

export interface Telemetry {
  /** Map-frame pose at ~20 Hz. Undefined until the first frame. */
  pose: RobotPose | undefined;
  /** Joint angles (radians, by URDF name) at the gait controller's rate. */
  joints: Record<string, number> | undefined;
  /**
   * The planner's route, on the same stream as the pose. Undefined until the
   * first plan; an empty `points` arrives when the run ends, which is what takes
   * the band back off the floor — see the backend's TelemetryRepo.get_path on why
   * that clear has to be sent rather than inferred from silence.
   */
  path: PlannedPath | undefined;
}

/**
 * The telemetry WebSocket as React state: one socket per mounted caller, opened
 * on mount and closed on unmount, reconnection owned by the stream. Pose and
 * joints replaced polling GET /api/v1/robot/state for the 3D viewer: that
 * endpoint's timestamp has whole-second resolution and it is a polled, frozen
 * third-party contract, so no amount of client-side polling or easing could
 * make the motion continuous.
 *
 * Not a query, even though every other server read here goes through TanStack
 * Query: there is no request/response to cache, and the library's vocabulary
 * (stale, refetch, invalidate) has no meaning for a push stream.
 *
 * Also deliberately NOT how the point-cloud stream is consumed. These frames
 * are a few dozen numbers and their consumers are React props, so setState at
 * 20 Hz is cheap and correct; the cloud's frames are a few hundred KB and land
 * straight in three.js buffers through a ref (see PointCloudCanvas's live
 * stream effect) — routing those through state would re-render the tree once
 * per frame.
 */
export function useTelemetry(): Telemetry {
  const [pose, setPose] = React.useState<RobotPose | undefined>(undefined);
  const [joints, setJoints] = React.useState<
    Record<string, number> | undefined
  >(undefined);
  const [path, setPath] = React.useState<PlannedPath | undefined>(undefined);

  React.useEffect(() => {
    const stream = createTelemetryStream({
      onPose: setPose,
      onJoints: setJoints,
      onPath: setPath,
    });
    return () => stream.close();
  }, []);

  return { pose, joints, path };
}
