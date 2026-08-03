"use client";

import * as React from "react";

import { setInitialPose } from "@/lib/api/robot";
import { normalizeTheta } from "@/lib/api/task";
import type { PlanarPose } from "@/lib/types/robot";

export interface InitialPoseEstimate {
  /** The pose the last drag produced — sent as soon as it was released. */
  pose: PlanarPose | null;
  /** Called by the view when a drag produces a pose; stages *and* sends it. */
  commitPose: (pose: PlanarPose) => void;
  /** True once the pose has gone out on `initialpose`. */
  published: boolean;
  /** True while the POST is in flight. */
  busy: boolean;
  error: string | null;
  /** Re-send the pose already placed — a retry, not the normal path. */
  publish: () => Promise<void>;
  clear: () => void;
}

/**
 * The drag-an-initial-pose flow (RViz's "2D Pose Estimate"): drag the robot to
 * where it actually is and the pose goes straight to the localizer via POST
 * /api/v1/robot/set_initial_pose on release.
 *
 * Unlike a nav goal this is *not* staged-then-confirmed. The gesture itself is
 * the whole statement — the operator drags the robot model to where the machine
 * is standing and turns it to face the way it faces, so the model on the floor
 * already shows exactly what would be published; a confirm step would only ask
 * them to re-read as numbers what they just placed by eye. A mis-drag is
 * corrected the same way it was made: drag again, which re-seeds the localizer.
 *
 * There is no status to poll afterwards. `initialpose` is a plain topic with no
 * ack, and the localizer only takes it as an ICP initial guess — so `published`
 * means the sample was sent, and whether it took is answered by the pose feed
 * moving to where the robot was dropped.
 */
export function useInitialPose(): InitialPoseEstimate {
  const [pose, setPose] = React.useState<PlanarPose | null>(null);
  const [published, setPublished] = React.useState(false);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const send = React.useCallback(async (next: PlanarPose) => {
    setBusy(true);
    setError(null);
    try {
      await setInitialPose(next);
      setPublished(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }, []);

  const commitPose = React.useCallback(
    (next: PlanarPose) => {
      const placed = { ...next, theta: normalizeTheta(next.theta) };
      setPose(placed);
      setPublished(false);
      // Sent from `placed`, not from the `pose` state set on the line above:
      // this runs in the same tick, so reading state here would post the
      // *previous* drag's pose (or nothing at all, on the first one).
      void send(placed);
    },
    [send],
  );

  const publish = React.useCallback(async () => {
    if (!pose) return;
    await send(pose);
  }, [pose, send]);

  const clear = React.useCallback(() => {
    setPose(null);
    setPublished(false);
    setError(null);
  }, []);

  return { pose, commitPose, published, busy, error, publish, clear };
}
