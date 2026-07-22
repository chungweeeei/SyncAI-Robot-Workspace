"use client";

import * as React from "react";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
import type { MapMetadata, RobotPose, Vertex } from "@/lib/types/robot";

/**
 * ROS world coordinates -> grid pixel coordinates (y flipped: canvas rows
 * grow downward while the ROS grid's row 0 sits at the map origin).
 */
export function worldToGrid(wx: number, wy: number, meta: MapMetadata) {
  return {
    px: (wx - meta.origin[0]) / meta.resolution,
    py: meta.height - (wy - meta.origin[1]) / meta.resolution,
  };
}

interface Palette {
  robot: string;
  robotHeading: string;
  waypoint: string;
  label: string;
}

const PALETTES: Record<"light" | "dark", Palette> = {
  light: {
    robot: "#2563eb",
    robotHeading: "#ffffff",
    waypoint: "#d97706",
    label: "#525252",
  },
  dark: {
    robot: "#3b82f6",
    robotHeading: "#ffffff",
    waypoint: "#f59e0b",
    label: "#a3a3a3",
  },
};

interface MapCanvasProps {
  /** The occupancy map as a PNG data URI (already rendered upright server-side). */
  mapImageUrl: string;
  meta: MapMetadata;
  pose: RobotPose;
  vertexes?: Vertex[];
  showLabels?: boolean;
  className?: string;
}

export function MapCanvas({
  mapImageUrl,
  meta,
  pose,
  vertexes = [],
  showLabels = true,
  className,
}: MapCanvasProps) {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const canvasRef = React.useRef<HTMLCanvasElement>(null);
  const { resolvedTheme } = useTheme();

  React.useEffect(() => {
    const container = containerRef.current;
    const canvas = canvasRef.current;
    if (!container || !canvas) return;

    const palette = PALETTES[resolvedTheme === "dark" ? "dark" : "light"];

    // The map PNG loads asynchronously; draw() no-ops until it's ready, then
    // onload (and every ResizeObserver tick) repaints with the image in place.
    const mapImage = new Image();
    let mapLoaded = false;

    const draw = () => {
      if (!mapLoaded) return;
      const rect = container.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);

      const ctx = canvas.getContext("2d")!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, rect.width, rect.height);

      // Uniform fit-to-container scale, map centered
      const scale = Math.min(
        rect.width / meta.width,
        rect.height / meta.height,
      );
      const ox = (rect.width - meta.width * scale) / 2;
      const oy = (rect.height - meta.height * scale) / 2;

      const toCanvas = (wx: number, wy: number) => {
        const { px, py } = worldToGrid(wx, wy, meta);
        return { x: ox + px * scale, y: oy + py * scale };
      };

      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, ox, oy, meta.width * scale, meta.height * scale);

      // Waypoints
      for (const vertex of vertexes) {
        const { x, y } = toCanvas(vertex.pose.x, vertex.pose.y);
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fillStyle = palette.waypoint;
        ctx.fill();
        if (showLabels) {
          ctx.font = "10px sans-serif";
          ctx.textAlign = "center";
          ctx.fillStyle = palette.label;
          ctx.fillText(vertex.name, x, y - 6);
        }
      }

      // Robot: circle + heading triangle (theta negated: canvas y is flipped)
      const robot = toCanvas(pose.x, pose.y);
      ctx.save();
      ctx.translate(robot.x, robot.y);
      ctx.rotate((-pose.theta * Math.PI) / 180);
      ctx.beginPath();
      ctx.arc(0, 0, 8, 0, Math.PI * 2);
      ctx.fillStyle = palette.robot;
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(9, 0);
      ctx.lineTo(1, -4);
      ctx.lineTo(1, 4);
      ctx.closePath();
      ctx.fillStyle = palette.robotHeading;
      ctx.fill();
      ctx.restore();
    };

    mapImage.onload = () => {
      mapLoaded = true;
      draw();
    };
    mapImage.src = mapImageUrl;

    draw();
    const observer = new ResizeObserver(draw);
    observer.observe(container);
    return () => observer.disconnect();
  }, [mapImageUrl, meta, pose, vertexes, showLabels, resolvedTheme]);

  return (
    <div
      ref={containerRef}
      className={cn("relative h-full w-full", className)}
    >
      <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" />
    </div>
  );
}
