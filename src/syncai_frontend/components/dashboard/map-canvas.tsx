"use client";

import * as React from "react";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
import type { GoalPose } from "@/lib/api/task";
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

/** Inverse of worldToGrid; used to turn a pointer position back into a goal. */
export function gridToWorld(px: number, py: number, meta: MapMetadata) {
  return {
    wx: px * meta.resolution + meta.origin[0],
    wy: (meta.height - py) * meta.resolution + meta.origin[1],
  };
}

/**
 * The map is drawn uniformly scaled and centered in the container. The pointer
 * handlers need the same transform as draw(), so it lives here instead of
 * inside the draw closure.
 */
function fitView(
  rect: { width: number; height: number },
  meta: MapMetadata,
) {
  const scale = Math.min(rect.width / meta.width, rect.height / meta.height);
  return {
    scale,
    ox: (rect.width - meta.width * scale) / 2,
    oy: (rect.height - meta.height * scale) / 2,
  };
}

interface Palette {
  robot: string;
  robotHeading: string;
  waypoint: string;
  label: string;
  goal: string;
  goalDraft: string;
}

const PALETTES: Record<"light" | "dark", Palette> = {
  light: {
    robot: "#2563eb",
    robotHeading: "#ffffff",
    waypoint: "#d97706",
    label: "#525252",
    goal: "#059669",
    goalDraft: "#10b981",
  },
  dark: {
    robot: "#3b82f6",
    robotHeading: "#ffffff",
    waypoint: "#f59e0b",
    label: "#a3a3a3",
    goal: "#10b981",
    goalDraft: "#34d399",
  },
};

/** Drag distance (CSS px) below which the heading is not taken from the drag. */
const HEADING_DEADZONE_PX = 8;

interface MapCanvasProps {
  /** The occupancy map as a PNG data URI (already rendered upright server-side). */
  mapImageUrl: string;
  meta: MapMetadata;
  pose: RobotPose;
  vertexes?: Vertex[];
  showLabels?: boolean;
  /** Committed goal, drawn as an arrow until the caller clears it. */
  goal?: GoalPose | null;
  /** When true, a press-drag-release on the map produces a goal (RViz style). */
  goalMode?: boolean;
  /** Fired once on release with the dragged goal. */
  onGoalCommit?: (goal: GoalPose) => void;
  className?: string;
}

export function MapCanvas({
  mapImageUrl,
  meta,
  pose,
  vertexes = [],
  showLabels = true,
  goal = null,
  goalMode = false,
  onGoalCommit,
  className,
}: MapCanvasProps) {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const canvasRef = React.useRef<HTMLCanvasElement>(null);
  const { resolvedTheme } = useTheme();

  // Goal being dragged right now. Kept in state (not a ref) so every pointer
  // move repaints; the map image is cached separately so that repaint is cheap.
  const [draft, setDraft] = React.useState<GoalPose | null>(null);

  // The map PNG is decoded once per mapImageUrl. Loading it inside the draw
  // effect would restart the decode on every pose tick and blank the canvas
  // between frames, because draw() has to no-op until onload fires. The URL is
  // stored alongside the image so a stale decode is never painted under a new
  // map.
  const [loaded, setLoaded] = React.useState<{
    url: string;
    image: HTMLImageElement;
  } | null>(null);

  React.useEffect(() => {
    const image = new Image();
    let cancelled = false;
    image.onload = () => {
      if (cancelled) return;
      setLoaded({ url: mapImageUrl, image });
    };
    image.src = mapImageUrl;
    return () => {
      cancelled = true;
    };
  }, [mapImageUrl]);

  const mapImage = loaded?.url === mapImageUrl ? loaded.image : null;

  React.useEffect(() => {
    const container = containerRef.current;
    const canvas = canvasRef.current;
    if (!container || !canvas) return;

    const palette = PALETTES[resolvedTheme === "dark" ? "dark" : "light"];

    const draw = () => {
      if (!mapImage) return;

      const rect = container.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);

      const ctx = canvas.getContext("2d")!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, rect.width, rect.height);

      const { scale, ox, oy } = fitView(rect, meta);

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

      // Goal arrow, in screen-space pixels: at typical map scales a
      // metric-length arrow is a couple of pixels long and unreadable.
      const drawGoal = (target: GoalPose, color: string, dashed: boolean) => {
        const { x, y } = toCanvas(target.x, target.y);
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate((-target.theta * Math.PI) / 180);
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = 2;

        ctx.setLineDash(dashed ? [4, 3] : []);
        ctx.beginPath();
        ctx.arc(0, 0, 7, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.moveTo(7, 0);
        ctx.lineTo(22, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(30, 0);
        ctx.lineTo(20, -6);
        ctx.lineTo(20, 6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      };

      if (goal) drawGoal(goal, palette.goal, false);
      // The draft is only meaningful while goal mode is on: leaving the mode
      // mid-drag must not strand an arrow on the map.
      if (draft && goalMode) drawGoal(draft, palette.goalDraft, true);

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

    draw();
    const observer = new ResizeObserver(draw);
    observer.observe(container);
    return () => observer.disconnect();
  }, [
    mapImage,
    meta,
    pose,
    vertexes,
    showLabels,
    goal,
    goalMode,
    draft,
    resolvedTheme,
  ]);

  // --- Goal dragging -------------------------------------------------------
  // Anchor is kept in container-local CSS pixels so the heading can be taken
  // from the raw pointer delta (screen space) while the position is converted
  // to world coordinates.
  const anchorRef = React.useRef<{ cx: number; cy: number } | null>(null);

  const localPoint = (event: React.PointerEvent) => {
    const rect = containerRef.current!.getBoundingClientRect();
    return { cx: event.clientX - rect.left, cy: event.clientY - rect.top, rect };
  };

  const goalAt = (
    cx: number,
    cy: number,
    rect: DOMRect,
    theta: number,
  ): GoalPose => {
    const { scale, ox, oy } = fitView(rect, meta);
    const { wx, wy } = gridToWorld((cx - ox) / scale, (cy - oy) / scale, meta);
    return { x: wx, y: wy, theta };
  };

  /** The map is letterboxed inside the container; presses in the margin are
   *  outside the occupancy grid and would produce a goal off the map. */
  const insideMap = (cx: number, cy: number, rect: DOMRect) => {
    const { scale, ox, oy } = fitView(rect, meta);
    const px = (cx - ox) / scale;
    const py = (cy - oy) / scale;
    return px >= 0 && px <= meta.width && py >= 0 && py <= meta.height;
  };

  const handlePointerDown = (event: React.PointerEvent) => {
    if (!goalMode || event.button !== 0) return;
    const { cx, cy, rect } = localPoint(event);
    if (!insideMap(cx, cy, rect)) return;
    event.preventDefault();
    anchorRef.current = { cx, cy };
    event.currentTarget.setPointerCapture(event.pointerId);
    // Until the pointer moves, inherit the robot's current heading so a plain
    // click still yields a sane goal instead of snapping to 0deg.
    setDraft(goalAt(cx, cy, rect, pose.theta));
  };

  const handlePointerMove = (event: React.PointerEvent) => {
    const anchor = anchorRef.current;
    if (!anchor || !goalMode) return;
    const { cx, cy, rect } = localPoint(event);
    const dx = cx - anchor.cx;
    const dy = cy - anchor.cy;
    const theta =
      Math.hypot(dx, dy) < HEADING_DEADZONE_PX
        ? // Keep whatever heading the draft already has inside the deadzone.
          (draft?.theta ?? pose.theta)
        : // Canvas y grows downward, so negate dy to get the world angle.
          (Math.atan2(-dy, dx) * 180) / Math.PI;
    setDraft(goalAt(anchor.cx, anchor.cy, rect, theta));
  };

  const handlePointerUp = (event: React.PointerEvent) => {
    if (!anchorRef.current) return;
    anchorRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    if (draft && goalMode) onGoalCommit?.(draft);
    setDraft(null);
  };

  return (
    <div
      ref={containerRef}
      className={cn(
        "relative h-full w-full",
        goalMode && "cursor-crosshair touch-none",
        className,
      )}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
    >
      <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" />
    </div>
  );
}
