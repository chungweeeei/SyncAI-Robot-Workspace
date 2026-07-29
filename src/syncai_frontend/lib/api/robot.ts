// Client for the backend robot API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/robot.py).

import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import { normalizeTheta } from "@/lib/api/task";
import type { PlanarPose } from "@/lib/types/robot";

/**
 * Seed localization with an operator-supplied pose (RViz's "2D Pose Estimate").
 *
 * The backend publishes it on `initialpose`, which the FAST-LIO2 localizer takes
 * as an ICP initial guess. That is fire-and-forget on the ROS side: a 200 here
 * means the sample went out, not that localization converged on it — the answer
 * to "did it work" is the pose feed moving to where the operator put the marker.
 */
export async function setInitialPose(pose: PlanarPose): Promise<void> {
  const res = await fetch(apiUrl("/api/v1/robot/set_initial_pose"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      x: pose.x,
      y: pose.y,
      theta: normalizeTheta(pose.theta),
    }),
  });

  if (!res.ok) throw new Error(await errorDetail(res));
}
