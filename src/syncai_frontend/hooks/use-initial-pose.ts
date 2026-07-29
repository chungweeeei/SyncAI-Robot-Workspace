"use client";

import * as React from "react";

import { setInitialPose } from "@/lib/api/robot";
import { normalizeTheta } from "@/lib/api/task";
import type { PlanarPose } from "@/lib/types/robot";

export interface InitialPoseEstimate {
  /** Staged pose, not yet published. */
  pose: PlanarPose | null;
  /** Called by the view when a drag produces a pose. */
  commitPose: (pose: PlanarPose) => void;
  /** True once the staged pose has gone out on `initialpose`. */
  published: boolean;
  /** True while the POST is in flight. */
  busy: boolean;
  error: string | null;
  publish: () => Promise<void>;
  clear: () => void;
}

/**
 * The drag-an-initial-pose flow (RViz's "2D Pose Estimate"): stage a pose, then
 * publish it to the localizer via POST /api/v1/robot/set_initial_pose.
 *
 * Staged-then-confirmed for the same reason as a nav goal, and then some: this
 * does not move the robot, it moves the robot's *belief about where it is*, and
 * a mis-drag teleports the whole map-frame estimate. The operator reads the
 * numbers first.
 *
 * There is no status to poll afterwards. `initialpose` is a plain topic with no
 * ack, and the localizer only takes it as an ICP initial guess — so `published`
 * means the sample was sent, and whether it took is answered by the pose feed
 * moving to the marker.
 */
export function useInitialPose(): InitialPoseEstimate {
  const [pose, setPose] = React.useState<PlanarPose | null>(null);
  const [published, setPublished] = React.useState(false);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const commitPose = React.useCallback((next: PlanarPose) => {
    setPose({ ...next, theta: normalizeTheta(next.theta) });
    setPublished(false);
    setError(null);
  }, []);

  const publish = React.useCallback(async () => {
    if (!pose) return;
    setBusy(true);
    setError(null);
    try {
      await setInitialPose(pose);
      setPublished(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }, [pose]);

  const clear = React.useCallback(() => {
    setPose(null);
    setPublished(false);
    setError(null);
  }, []);

  return { pose, commitPose, published, busy, error, publish, clear };
}
