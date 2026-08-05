"use client";

import * as React from "react";

import { setInitialPose } from "@/lib/api/robot";
import { normalizeTheta } from "@/lib/api/task";
import type { PlanarPose } from "@/lib/types/robot";

/**
 * How long a published estimate stays on screen before it clears itself.
 *
 * Short on purpose. The first attempt at this was 5 s — long enough to read the
 * numbers twice — and in practice the operator reached for Clear before it ever
 * elapsed, which is the same complaint the auto-clear was written to answer. The
 * receipt only has to be *seen*: the numbers were placed by eye a moment ago,
 * and whether the seed took is answered by the robot in the viewport snapping to
 * the marker, not by this panel.
 */
const CLEAR_AFTER_PUBLISH_MS = 1500;

export interface InitialPoseEstimate {
  /**
   * The pose the last drag produced — sent as soon as it was released, and
   * dropped again a few seconds after it lands (see the auto-clear below).
   */
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
 * moving to where the robot was dropped. That is also why a published estimate
 * expires on its own after CLEAR_AFTER_PUBLISH_MS: with nothing to wait for,
 * leaving the read-back up would state an open action that is already over.
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

  /**
   * A published estimate tidies itself away.
   *
   * This flow has no terminal state to clear on — unlike a goal, which ends with
   * a task reaching SUCCEEDED — so before this the read-back and the amber
   * marker sat on screen until someone pressed Clear, and in practice nobody
   * did: the gesture is finished the moment the robot is dropped, and the panel
   * outliving it made a completed action look like an open one.
   *
   * Only a *success* is cleared. A failure keeps the pose, because that panel is
   * the only place the error and its Retry live, and the pose is what Retry
   * re-sends. A fresh drag sets `published` false first, which cancels any timer
   * already running, so the new estimate always gets its own full window.
   */
  React.useEffect(() => {
    if (!published) return;
    const timer = setTimeout(() => {
      setPose(null);
      setPublished(false);
    }, CLEAR_AFTER_PUBLISH_MS);
    return () => clearTimeout(timer);
  }, [published]);

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
