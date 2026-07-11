"use client";

import * as React from "react";

import { MapCanvas } from "@/components/dashboard/map-canvas";
import {
  generateMockGrid,
  mockMapMetadata,
  mockVertexes,
} from "@/lib/mock/map";
import type { RobotPose } from "@/lib/types/robot";

// Client-side wrapper: the occupancy grid is a typed array, which can't cross
// the RSC serialization boundary, so the mock grid is generated here.
export function MapView({ pose }: { pose: RobotPose }) {
  const grid = React.useMemo(() => generateMockGrid(mockMapMetadata), []);

  return (
    <MapCanvas
      grid={grid}
      meta={mockMapMetadata}
      pose={pose}
      vertexes={mockVertexes}
    />
  );
}
