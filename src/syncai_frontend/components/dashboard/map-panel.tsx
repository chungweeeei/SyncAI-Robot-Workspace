"use client";

import { MapView } from "@/components/dashboard/map-view";
import { PointCloudView } from "@/components/dashboard/pointcloud-view";
import type { RobotPose } from "@/lib/types/robot";

export type ViewMode = "2d" | "3d";

/**
 * The viewport. Picks the 2D occupancy grid or the live 3D point cloud; the
 * choice itself is owned by the page and surfaced in the rail's View group,
 * because a toggle floating over the map was covering the map.
 *
 * Both views can dispatch a nav goal, and each owns its own `useGoalTask`
 * state: switching views unmounts one and mounts the other, so a staged goal
 * does not survive the toggle. That is deliberate — a goal picked by clicking a
 * 3D floor point should not silently reappear as a pending goal in the 2D view.
 */
export function MapPanel({
  mode,
  pose,
  robotId,
}: {
  mode: ViewMode;
  pose: RobotPose;
  robotId: string;
}) {
  return mode === "2d" ? (
    <MapView pose={pose} robotId={robotId} />
  ) : (
    <PointCloudView robotId={robotId} />
  );
}
