"use client";

import * as React from "react";

import { Button } from "@/components/ui/button";
import { MapView } from "@/components/dashboard/map-view";
import { PointCloudView } from "@/components/dashboard/pointcloud-view";
import { cn } from "@/lib/utils";
import type { RobotPose } from "@/lib/types/robot";

type ViewMode = "2d" | "3d";

/**
 * Hosts the map card body with a 2D / 3D toggle. The 2D view is the familiar
 * top-down occupancy grid; the 3D view streams the live point cloud. 2D stays
 * the default so weak clients aren't pushed into WebGL unless asked.
 */
export function MapPanel({ pose }: { pose: RobotPose }) {
  const [mode, setMode] = React.useState<ViewMode>("2d");

  return (
    <div className="relative h-full w-full">
      <div className="absolute right-0 -top-9 z-10 flex gap-1">
        {(["2d", "3d"] as const).map((m) => (
          <Button
            key={m}
            size="sm"
            variant={mode === m ? "default" : "outline"}
            className={cn("h-7 px-3 text-xs")}
            onClick={() => setMode(m)}
          >
            {m.toUpperCase()}
          </Button>
        ))}
      </div>

      {mode === "2d" ? <MapView pose={pose} /> : <PointCloudView />}
    </div>
  );
}
