"use client";

import * as React from "react";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
import {
  stampDisc,
  stampLine,
  stampRect,
  type Cell,
  type CellSink,
  type GridValue,
} from "@/lib/map/grid";
import { blitGrid, blitGridRect } from "@/lib/map/render";
import type { GridPatch } from "@/lib/map/patch";
import type { GridSession } from "@/lib/map/session";
import { vertexGlyph } from "@/lib/map/vertex";
import {
  CELL_GRID_MIN_SCALE,
  cellAt,
  centerView,
  fitView,
  gridToScreen,
  gridToWorld,
  panBy,
  reanchorView,
  screenToGrid,
  worldToGrid,
  zoomAt,
  type View,
} from "@/lib/map/view";
import type { MapVertex } from "@/lib/types/map";
import type { MapMetadata, PlanarPose } from "@/lib/types/robot";

export type EditTool = "brush" | "line" | "rect" | "pan";

/**
 * What a press means in vertex mode, the counterpart to EditTool in grid mode.
 *
 * A separate axis rather than three more members of EditTool, because the two
 * sets share only "pan" and nothing carries over: an operator who left Brush
 * armed and switched to vertex mode would otherwise arrive holding a tool that
 * cannot exist there, and the shell would have to translate anyway. Two states
 * also mean the mode toggle does not disturb either one — which is what lets
 * grid mode keep the brush it had while vertex mode keeps its own resting Pan.
 */
export type VertexTool = "pan" | "place" | "select";

/**
 * What a press on the canvas means.
 *
 * "grid" paints cells; "vertex" places and aims map vertices and never touches
 * the cell buffer. The two share this component rather than getting a canvas
 * each because lib/map/view.ts's rule is that there is exactly one view
 * transform and every handler reads the same one — a second layer would have to
 * be handed `viewRef`, which is the one thing this component does not expose.
 */
export type EditMode = "grid" | "vertex";

/** What a completed vertex gesture produced. */
export interface VertexGesture {
  /** The vertex the press landed on, or null for a press on bare map. */
  id: string | null;
  /**
   * Position plus heading in degrees. For a press on an existing vertex the
   * position is that vertex's stored one, echoed back unchanged — see
   * handlePointerDown on why the press point is deliberately not used there.
   */
  pose: PlanarPose;
}

export interface CellProbe {
  col: number;
  row: number;
  /** The byte under the cursor, so the status bar never needs the buffer. */
  byte: number;
}

interface Palette {
  /** The area outside the map. */
  well: string;
  /** Hairline around the grid extent. */
  extent: string;
  /** Cell gridlines at high zoom. */
  cellGrid: string;
  /** Brush outline and shape preview — a commanded value the operator set. */
  cmd: string;
  /**
   * Stored map vertices. One hue for all five types (see lib/map/vertex.ts on
   * why the type is a glyph and not a colour), and not one of the signal hues:
   * a saved vertex is neither measured nor commanded nor faulted, so borrowing
   * a signal colour for it would weaken the ones that do mean something. The
   * *draft* vertex uses `cmd`, like every other value the operator is setting.
   */
  vertex: string;
  /**
   * The robot's own footprint. `signal-live` by the console's rule — it is a
   * measured, valid value, the one thing on this canvas that is neither stored
   * nor being commanded — which is also what keeps it from being read as a
   * vertex at a glance.
   */
  live: string;
}

/*
 * Only the chrome drawn *over* the grid follows the theme. The grid itself never
 * does: a gridmap is white free space and near-black obstacles in night mode too
 * (see lib/map/render.ts on why the bytes are blitted literally), so a marker
 * picked to look right against the dark panel would be invisible where it is
 * actually drawn. Hues are the globals.css signal values — `signal-cmd` for the
 * brush and previews, because they show a value the operator is about to commit.
 */
const PALETTES: Record<"light" | "dark", Palette> = {
  light: {
    well: "#e2e9ee",
    extent: "#8b9aa5",
    cellGrid: "#b9c6ce",
    cmd: "#0a6d94",
    vertex: "#2f4a58",
    live: "#12784a",
  },
  dark: {
    well: "#22282c",
    extent: "#5c6a74",
    cellGrid: "#3a444b",
    cmd: "#45c8f0",
    vertex: "#a8bcc7",
    live: "#4fd98d",
  },
};

/**
 * Drawn under every marker and label before the coloured shape.
 *
 * A vertex sits on cells the grid renders literally — white where the floor is
 * free, near-black where it is not (see lib/map/render.ts) — so no single hue is
 * legible everywhere a vertex can be placed. A translucent light halo behind the
 * mark is what makes the dark light-theme marker readable on top of an obstacle,
 * and it is drawn rather than themed because the *grid* under it never follows
 * the theme.
 */
const MARKER_HALO = "rgba(255, 255, 255, 0.85)";

/** Above this many preview cells, outline the shape instead of filling cells. */
const PREVIEW_CELL_LIMIT = 4000;

/** Smallest on-screen brush ring, so a 1-cell brush stays findable zoomed out. */
const MIN_RING_PX = 6;

/*
 * Vertex markers are sized in CSS pixels and do NOT scale with zoom, which is
 * the opposite of the brush ring. The ring scales because it has to be honest
 * about how many cells a click will paint; a vertex is a single pose, so scaling
 * it would only make it vanish at fit view — the zoom level where an operator is
 * most likely to be looking for it.
 */
const VERTEX_DOT_RADIUS = 4.5;
const VERTEX_ARROW_PX = 18;

/*
 * The robot's footprint, in metres, drawn to scale — the opposite choice from the
 * vertex markers above, and for the opposite reason: a vertex is a pose and has
 * no size, while "will this stop fit" is most of what the operator is asking when
 * they look at where the robot is standing. Full extents from the global
 * costmap's half-extents (planner_server_params.yaml, 0.35 x 0.22), which is the
 * shape the planner actually reasons with — note it disagrees with the local
 * costmap's (0.28 x 0.20); that drift is flagged in the package READMEs, and this
 * marker is a picture, not a clearance guarantee.
 */
const ROBOT_LENGTH_M = 0.7;
const ROBOT_WIDTH_M = 0.44;

/**
 * Smallest the footprint is ever drawn, measured along the robot's length.
 *
 * Same idea as MIN_RING_PX: at fit scale on the 1602x1502 maps a 0.7 m robot is
 * about 7 px, and the one thing this marker must never do is be untraceable at
 * the zoom where you are looking for it. The aspect ratio is held, so below this
 * size the shape stops being to scale and becomes a glyph — which is honest, in
 * that at 7 px nothing could be read as a clearance anyway.
 */
const MIN_ROBOT_LENGTH_PX = 14;
/** Click slop around a marker centre. Comfortably larger than the dot itself. */
const VERTEX_HIT_RADIUS = 11;
/** Below this drag distance the gesture is a click and the heading is kept. */
const HEADING_DEADZONE_PX = 10;

const ZOOM_PER_PX = 0.0015;
const WHEEL_LINE_PX = 16;

type Gesture =
  | { kind: "paint"; pointerId: number; last: Cell }
  | { kind: "shape"; pointerId: number; anchor: Cell; head: Cell }
  | {
      kind: "pan";
      pointerId: number;
      cx: number;
      cy: number;
      /** Press point, so release can tell a drag from a click. */
      ox: number;
      oy: number;
      /**
       * What a click — a release that never left the deadzone — should select,
       * or null when this pan resolves nothing. Only a left press with Pan armed
       * in vertex mode sets it: right/middle/Space pan work in every tool and
       * must not double as a way to change the selection.
       */
      pick: { id: string | null } | null;
    }
  /**
   * A rubber band over the vertex layer. Held in the ref like every other
   * gesture, for the same reason: it updates at pointer rate and the draw path
   * reads it directly, so pushing the rectangle through React state would
   * re-render the toolbar on every mouse move.
   */
  | {
      kind: "marquee";
      pointerId: number;
      /** Press point and current point, both container-local CSS px. */
      ox: number;
      oy: number;
      cx: number;
      cy: number;
      /** Shift was held: add to the selection rather than replace it. */
      additive: boolean;
    }
  /**
   * Placing or aiming a vertex. `theta` is held here rather than pushed to the
   * shell on every move: this component is memoized precisely so a gesture at
   * pointer rate cannot re-render the toolbar, and the draw path already reads
   * in-flight gesture state (drawPreview does the same for a line/rect).
   */
  | {
      kind: "vertex";
      pointerId: number;
      /** The vertex being aimed, or null when placing a new one. */
      id: string | null;
      /** Map frame, fixed for the whole gesture. */
      wx: number;
      wy: number;
      /** Press point in container-local CSS px, for the deadzone and the angle. */
      cx: number;
      cy: number;
      theta: number;
    };

export interface GridCanvasProps {
  session: GridSession;
  /** Grid paints cells; vertex places poses and never touches the buffer. */
  mode: EditMode;
  tool: EditTool;
  /** Read only in vertex mode, the way `tool` is read only in grid mode. */
  vertexTool: VertexTool;
  value: GridValue;
  /** Odd cell diameter from BRUSH_SIZES. */
  brush: number;
  /** True while the shell sees Space held — pans without changing the tool. */
  spacePan: boolean;
  /**
   * Bumped by the shell's Fit action. A nonce rather than a callback the shell
   * holds: the view lives in here, and handing out a setter would give the shell a
   * second way to reach it.
   */
  fitNonce: number;
  /**
   * A map-frame point to bring to the centre of the viewport, holding the zoom.
   *
   * Identity is the trigger — a fresh object means "do it now", the same job
   * `fitNonce` does with a counter — so the shell hands one over per request and
   * leaves it in place afterwards. Null means nothing has asked.
   *
   * It exists for the robot-position capture: that draft appears without a
   * pointer gesture, so its marker can land anywhere, including off screen, and
   * an operator with a staged pose they cannot see has no way to judge it.
   */
  focus: { x: number; y: number } | null;
  /**
   * Once per completed stroke, never mid-drag. The grid and the mirror are already
   * updated by then; the shell's only job is to record the patch.
   */
  onStrokeCommit: (patch: GridPatch) => void;
  /** Coalesced to at most one call per frame, and only when the cell changes. */
  onHover: (probe: CellProbe | null) => void;
  /** Coalesced the same way. */
  onScaleChange: (scale: number) => void;

  /** Vertices already stored for this map. Drawn in every mode. */
  vertices: MapVertex[];
  /**
   * Where the robot is standing on this map, or null when that is not knowable
   * (another map loaded, not localized, no state) — see useRobotMapPose.
   *
   * Drawn in every mode, like the vertices: it is the answer to "which end of the
   * corridor am I looking at", which is as useful with a brush in hand as it is
   * while placing stops. It is never interactive — nothing on this canvas can
   * move the robot, and hit-testing ignores it entirely.
   */
  robotPose: PlanarPose | null;
  /** Staged, not-yet-created vertex. Drawn in the commanded hue. */
  draft: PlanarPose | null;
  /**
   * Highlighted. One of them is the vertex the panel is editing; more than one
   * is a band selection, which the panel can only delete.
   *
   * A list rather than a Set because it is the shell's state and is handed
   * straight to React.memo — a Set rebuilt per render would defeat it, and the
   * only thing this component does with it is a membership test per frame,
   * which it builds its own Set for.
   */
  selectedIds: readonly string[];
  /** Fired on pointer-down, before any drag, so selection feels immediate. */
  onVertexPick: (id: string | null) => void;
  /** Shift-click on a marker: add it to, or drop it from, the selection. */
  onVertexToggle: (id: string) => void;
  /**
   * A finished rubber band, with the ids whose markers it enclosed — possibly
   * none, which is a real answer and clears the selection unless `additive`.
   */
  onMarquee: (ids: string[], additive: boolean) => void;
  /** Fired once on pointer-up with the finished pose. */
  onVertexGesture: (gesture: VertexGesture) => void;

  className?: string;
}

/**
 * The editable grid: the only component in the editor that touches pixels or
 * pointer events.
 *
 * The invariant inherited from the 2D map canvas this replaces is that there is
 * exactly one view transform and `draw()` and every handler read the same one.
 * Here it is mutable, so the rule is sharper: `viewRef.current` is the single
 * instance, handlers write it and then ask for a frame, and it is never copied into
 * React state — a pan at pointer rate must not re-render the toolbar.
 */
export const GridCanvas = React.memo(function GridCanvas(props: GridCanvasProps) {
  const { session, className } = props;
  const { resolvedTheme } = useTheme();

  const containerRef = React.useRef<HTMLDivElement>(null);
  const canvasRef = React.useRef<HTMLCanvasElement>(null);

  const viewRef = React.useRef<View | null>(null);
  /**
   * Cached container size. getBoundingClientRect() inside pointermove is a forced
   * layout read, and combined with the hover readout writing DOM text in the same
   * frame it is layout thrash on every mouse move.
   */
  const rectRef = React.useRef<{ width: number; height: number } | null>(null);
  const gestureRef = React.useRef<Gesture | null>(null);
  const hoverRef = React.useRef<CellProbe | null>(null);
  const rafRef = React.useRef(0);
  const drawRef = React.useRef<(() => void) | null>(null);
  const publishedHoverRef = React.useRef<string>("");
  const publishedScaleRef = React.useRef(0);

  /**
   * The draw/listener effect must not re-subscribe when a callback identity, the
   * tool or the brush size changes: it owns a ResizeObserver and a non-passive
   * wheel listener, and re-running it mid-drag would drop pointer capture and the
   * in-flight stroke. It reads everything current through here instead.
   */
  const propsRef = React.useRef(props);
  React.useEffect(() => {
    propsRef.current = props;
  });

  const theme: "light" | "dark" = resolvedTheme === "dark" ? "dark" : "light";
  const themeRef = React.useRef(theme);
  themeRef.current = theme;

  /**
   * Draw-on-change, coalesced to one paint per frame.
   *
   * Not a persistent rAF loop: nothing on this screen animates, the 3D viewport
   * already owns one, and this console can be running on the robot's own Jetson.
   * Not a bare draw() per event either — a trackpad pinch delivers a dozen wheel
   * events per frame, and a resize plus a wheel in the same frame must paint once.
   */
  const requestDraw = React.useCallback(() => {
    if (rafRef.current) return;
    rafRef.current = requestAnimationFrame(() => {
      rafRef.current = 0;
      drawRef.current?.();
    });
  }, []);

  const draw = React.useCallback(() => {
    const container = containerRef.current;
    const canvas = canvasRef.current;
    if (!container || !canvas) return;

    const bounds = container.getBoundingClientRect();
    // A flex child's first layout pass can be 0x0. Bailing is not just a skipped
    // frame here: fitView on a zero rect gives scale 0, screenToGrid then divides
    // by it, and the resulting NaN transform never recovers.
    if (bounds.width < 1 || bounds.height < 1) return;
    const rect = { width: bounds.width, height: bounds.height };
    rectRef.current = rect;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const bw = Math.round(rect.width * dpr);
    const bh = Math.round(rect.height * dpr);
    // Only on change: assigning width/height clears the surface and resets the
    // transform, which is a wasted full realloc at pointer rate.
    if (canvas.width !== bw || canvas.height !== bh) {
      canvas.width = bw;
      canvas.height = bh;
    }

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    // DPR lives here and nowhere else — the view transform stays in CSS pixels,
    // the same unit event.clientX reports, so hit-testing needs no DPR term.
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const palette = PALETTES[themeRef.current];
    ctx.fillStyle = palette.well;
    ctx.fillRect(0, 0, rect.width, rect.height);

    if (!viewRef.current) viewRef.current = fitView(rect, session.grid);
    const view = viewRef.current;
    const { grid, mirror } = session;
    const w = grid.width * view.scale;
    const h = grid.height * view.scale;

    // Nearest-neighbour at or above 1:1 — a smoothed cell boundary would lie about
    // which cell the next click lands in. Below 1:1 the browser's filtered
    // downscale is the honest choice: fit scale on the 1602x1502 maps is ~0.47, and
    // nearest sampling there drops every other cell, so 1-to-3-cell-thick walls
    // break into dotted noise exactly when you are trying to judge them.
    ctx.imageSmoothingEnabled = view.scale < 1;
    ctx.drawImage(mirror.canvas, view.ox, view.oy, w, h);

    ctx.lineWidth = 1;
    ctx.strokeStyle = palette.extent;
    ctx.strokeRect(view.ox - 0.5, view.oy - 0.5, w + 1, h + 1);

    if (view.scale >= CELL_GRID_MIN_SCALE) drawCellGrid(ctx, view, rect, grid, palette);

    const current = propsRef.current;
    drawPreview(ctx, view, session, gestureRef.current, current, palette);
    drawBrushRing(ctx, view, gestureRef.current, hoverRef.current, current, palette);
    // Under the vertex layer: the markers are what this screen edits, and a stop
    // placed where the robot is standing must not disappear beneath it.
    if (current.robotPose) {
      drawRobot(ctx, view, session.meta, current.robotPose, palette);
    }
    // Above the shape preview, so markers are never buried by it.
    drawVertices(ctx, view, session.meta, gestureRef.current, current, palette);
    // Last of all: the band is chrome over everything it is selecting.
    drawMarquee(ctx, gestureRef.current, palette);

    // Publish at most once per frame, and only on a real change: moving within one
    // cell at high zoom, or a pan that does not change the scale, costs nothing.
    const hover = hoverRef.current;
    const hoverKey = hover ? `${hover.col},${hover.row}` : "";
    if (publishedHoverRef.current !== hoverKey) {
      publishedHoverRef.current = hoverKey;
      current.onHover(hover);
    }
    const scalePercent = Math.round(view.scale * 100);
    if (publishedScaleRef.current !== scalePercent) {
      publishedScaleRef.current = scalePercent;
      current.onScaleChange(view.scale);
    }
  }, [session]);

  drawRef.current = draw;

  // Repaint on a theme change: the grid bytes do not move, but the chrome hues do.
  React.useEffect(() => {
    requestDraw();
  }, [theme, requestDraw]);

  /**
   * Repaint when the vertex layer changes.
   *
   * Necessary because the propsRef effect above deliberately does not draw: the
   * tool, the brush and the paint value only matter at the *next* pointer event,
   * so a render caused by one of them costs no frame. Vertex data is different —
   * it is on screen, and a create that lands while the pointer is still would
   * otherwise not appear until something else happened to ask for a frame.
   */
  React.useEffect(() => {
    requestDraw();
  }, [
    props.vertices,
    props.draft,
    props.selectedIds,
    props.mode,
    // Once a second at most, and only when the robot has actually moved — the
    // hook memoises the pose on its values, so a parked robot costs no frames.
    props.robotPose,
    requestDraw,
  ]);

  // Fit: drop the transform and let draw() rebuild it from the current rect.
  React.useEffect(() => {
    viewRef.current = null;
    requestDraw();
  }, [props.fitNonce, requestDraw]);

  /**
   * Centre on a point the shell asked for, if there is a view to move.
   *
   * Before the first paint there is neither a transform nor a measured rect, and
   * nothing to do: draw() then builds the fit view, which has the whole map —
   * and so the target — on screen anyway.
   */
  const focus = props.focus;
  React.useEffect(() => {
    if (!focus) return;
    const view = viewRef.current;
    const rect = rectRef.current;
    if (!view || !rect) return;
    const { px, py } = worldToGrid(focus.x, focus.y, session.meta);
    viewRef.current = centerView(view, px, py, rect, session.grid);
    requestDraw();
  }, [focus, session, requestDraw]);

  React.useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    // Undo/redo lives in the shell but has to reach the mirror and the frame
    // scheduler, which only this component owns. The slot's lifetime is exactly
    // this effect's.
    session.repaint = (dirty) => {
      if (dirty) blitGridRect(session.mirror, session.grid, dirty);
      else blitGrid(session.mirror, session.grid);
      requestDraw();
    };

    let previous = rectRef.current;
    const observer = new ResizeObserver(() => {
      const bounds = container.getBoundingClientRect();
      if (bounds.width < 1 || bounds.height < 1) return;
      const next = { width: bounds.width, height: bounds.height };
      // Hold the zoom across a resize rather than refitting — a window resize or a
      // devtools pane must not throw away the view the operator set up. "Fit" is
      // the explicit way back.
      if (viewRef.current && previous) {
        viewRef.current = reanchorView(viewRef.current, previous, next, session.grid);
      }
      previous = next;
      rectRef.current = next;
      requestDraw();
    });
    // The container only, and the canvas stays absolutely positioned: assigning
    // width on an in-flow canvas changes layout and would re-trigger this forever.
    observer.observe(container);

    // Wheel cannot be a JSX onWheel prop. React attaches wheel passively at the
    // root, so preventDefault() there is ignored with a console warning — and
    // without it a trackpad pinch (which arrives as wheel + ctrlKey) page-zooms
    // the whole console instead of the map.
    const onWheel = (event: WheelEvent) => {
      const rect = rectRef.current;
      const view = viewRef.current;
      if (!rect || !view) return;
      event.preventDefault();

      const bounds = container.getBoundingClientRect();
      const cx = event.clientX - bounds.left;
      const cy = event.clientY - bounds.top;
      // deltaMode 1 is lines (Firefox) and 2 is pages; a raw deltaY would zoom
      // ~16x per notch there. Exponential rather than 1 + k*delta so zoom is
      // multiplicative: N notches up then N down returns to the same scale.
      const unit =
        event.deltaMode === 1 ? WHEEL_LINE_PX : event.deltaMode === 2 ? rect.height : 1;
      const factor = Math.exp(-event.deltaY * unit * ZOOM_PER_PX);
      viewRef.current = zoomAt(view, cx, cy, factor, rect, session.grid);
      requestDraw();
    };
    container.addEventListener("wheel", onWheel, { passive: false });

    requestDraw();

    return () => {
      observer.disconnect();
      container.removeEventListener("wheel", onWheel);
      session.repaint = null;
      if (rafRef.current) {
        cancelAnimationFrame(rafRef.current);
        rafRef.current = 0;
      }
    };
  }, [session, requestDraw]);

  /** Container-local CSS pixels. */
  const localPoint = (event: React.PointerEvent) => {
    const bounds = event.currentTarget.getBoundingClientRect();
    rectRef.current = { width: bounds.width, height: bounds.height };
    return { cx: event.clientX - bounds.left, cy: event.clientY - bounds.top };
  };

  /**
   * The cell under a point, clamped into the grid.
   *
   * Clamped rather than nulled so a drag that wanders past the edge keeps painting
   * the edge row, which is what every paint program does. The press that *starts* a
   * stroke still has to land inside (see handlePointerDown).
   */
  const clampedCell = (view: View, cx: number, cy: number): Cell => {
    const { px, py } = screenToGrid(view, cx, cy);
    const grid = session.grid;
    return {
      col: Math.min(Math.max(Math.floor(px), 0), grid.width - 1),
      row: Math.min(Math.max(Math.floor(py), 0), grid.height - 1),
    };
  };

  /**
   * The topmost vertex marker under a point, or null.
   *
   * Hit-tested in screen space rather than in metres so the target stays the
   * same size at every zoom — the marker does not scale, so a world-space radius
   * would be unclickable zoomed out and enormous zoomed in. Last-to-first
   * because that is paint order reversed: the marker drawn on top wins.
   */
  const vertexAt = (view: View, cx: number, cy: number): MapVertex | null => {
    const { vertices } = propsRef.current;
    for (let i = vertices.length - 1; i >= 0; i -= 1) {
      const vertex = vertices[i];
      const at = vertexScreen(view, session.meta, vertex.x, vertex.y);
      if (Math.hypot(at.cx - cx, at.cy - cy) <= VERTEX_HIT_RADIUS) return vertex;
    }
    return null;
  };

  const probeAt = (cell: Cell | null): CellProbe | null => {
    if (!cell) return null;
    return {
      col: cell.col,
      row: cell.row,
      byte: session.grid.data[cell.row * session.grid.width + cell.col],
    };
  };

  const flushStroke = () => {
    const dirty = session.accumulator.takeFrameDirty();
    if (dirty) blitGridRect(session.mirror, session.grid, dirty);
    requestDraw();
  };

  const handlePointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    const view = viewRef.current;
    if (!view) return;
    if (event.button !== 0 && event.button !== 1 && event.button !== 2) return;

    const { mode, tool, vertexTool, value, brush, spacePan } = propsRef.current;
    const { cx, cy } = localPoint(event);
    // Right and middle drag always pan, in every mode and whatever the tool.
    //
    // This is the dashboard's bargain, adapted: there, left-drag moves the view
    // and an armed pick mode is what takes the button away. Same here — both
    // modes open in Pan (see DEFAULT_TOOL / DEFAULT_VERTEX_TOOL) and arming a
    // brush or the vertex Place tool is what claims the left button. Right and
    // middle keep panning regardless, so the operator never has to disarm to
    // reach another part of the map. Right is the one that is free: a 2D canvas
    // has no orbit to compete for it, unlike the point-cloud viewport where
    // right-drag is the orbit.
    //
    // Each mode reads its own tool and ignores the other's, which is what keeps
    // the mode toggle from carrying an armed tool across with it.
    const panning =
      event.button === 1 ||
      event.button === 2 ||
      spacePan ||
      (mode === "grid" && tool === "pan") ||
      (mode === "vertex" && vertexTool === "pan");

    if (!panning && mode === "vertex") {
      const { draft } = propsRef.current;
      const hit = vertexAt(view, cx, cy);

      if (vertexTool === "select") {
        // Shift on a marker toggles it, and arms nothing: building a selection
        // one awkward vertex at a time is the half of multi-select a rectangle
        // cannot do, and a re-aim drag starting from a Shift-press would be an
        // edit the operator was not asking for.
        if (hit && event.shiftKey) {
          event.preventDefault();
          propsRef.current.onVertexToggle(hit.id);
          return;
        }

        // Bare map with Select armed is a rubber band. It starts anywhere,
        // including the letterbox margin outside the grid — the band selects
        // markers, not cells, so there is nothing for it to be outside of.
        if (!hit) {
          event.preventDefault();
          event.currentTarget.setPointerCapture(event.pointerId);
          gestureRef.current = {
            kind: "marquee",
            pointerId: event.pointerId,
            ox: cx,
            oy: cy,
            cx,
            cy,
            additive: event.shiftKey,
          };
          requestDraw();
          return;
        }
        // A plain press on a marker falls through: selecting and re-aiming one
        // vertex works the same under both vertex tools, so the operator does
        // not have to remember which one they are holding to fix a heading.
      }

      // An existing vertex anchors at its *stored* position, not at the press
      // point. The press has to land within VERTEX_HIT_RADIUS of the marker, so
      // using it would silently move the vertex by up to 11 px worth of metres
      // every time the operator merely selected one to re-aim it.
      let anchor: { wx: number; wy: number };
      if (hit) {
        anchor = { wx: hit.x, wy: hit.y };
      } else {
        // Reaching here means the Place tool: a bare-map press with Select armed
        // became the marquee above. Only a press on the grid places a vertex; the
        // map is letterboxed and a press in the margin means nothing, exactly as
        // it does for a stroke.
        const cell = cellAt(view, session.grid, cx, cy);
        if (!cell) return;
        // Cell centre, not corner: gridToWorld(col, row) is the corner, and at
        // 0.05 m/cell reporting that as the pose is a 2.5 cm lie — the same
        // reason GridStatus offsets its readout by half a cell.
        anchor = gridToWorld(cell.col + 0.5, cell.row + 0.5, session.meta);
      }

      event.preventDefault();
      event.currentTarget.setPointerCapture(event.pointerId);
      propsRef.current.onVertexPick(hit?.id ?? null);

      gestureRef.current = {
        kind: "vertex",
        pointerId: event.pointerId,
        id: hit?.id ?? null,
        wx: anchor.wx,
        wy: anchor.wy,
        cx,
        cy,
        // Inside the deadzone the gesture is a click, and a click must not snap
        // the heading to 0°: keep the vertex's own, or the draft's if the
        // operator is placing a run of points facing the same way.
        theta: hit?.theta ?? draft?.theta ?? 0,
      };
      requestDraw();
      return;
    }

    if (!panning) {
      // Only a press that lands on the grid starts a stroke; the map is letterboxed
      // in the viewport and a press in the margin means nothing.
      const cell = cellAt(view, session.grid, cx, cy);
      if (!cell) return;

      event.preventDefault();
      event.currentTarget.setPointerCapture(event.pointerId);

      if (tool === "brush") {
        gestureRef.current = { kind: "paint", pointerId: event.pointerId, last: cell };
        stampDisc(session.grid, cell.col, cell.row, brush, value, session.accumulator.sink);
        flushStroke();
      } else {
        gestureRef.current = {
          kind: "shape",
          pointerId: event.pointerId,
          anchor: cell,
          head: cell,
        };
        requestDraw();
      }
      return;
    }

    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    gestureRef.current = {
      kind: "pan",
      pointerId: event.pointerId,
      cx,
      cy,
      ox: cx,
      oy: cy,
      pick:
        mode === "vertex" && vertexTool === "pan" && event.button === 0 && !spacePan
          ? { id: vertexAt(view, cx, cy)?.id ?? null }
          : null,
    };
    // An inline style rather than a class, because the className is React's and a
    // render lands mid-pan routinely: panning changes the hovered cell, draw()
    // publishes that to the shell, and the re-render would rewrite className and
    // drop the class again. `style.cursor` is not managed by React here, so it
    // survives. This is the only feedback that a right-drag grabbed the map, since
    // the Pan tool is not selected in that case.
    setPanCursor(true);
  };

  /** Held on the container itself; see the note in handlePointerDown. */
  const setPanCursor = (grabbing: boolean) => {
    const container = containerRef.current;
    if (container) container.style.cursor = grabbing ? "grabbing" : "";
  };

  const handlePointerMove = (event: React.PointerEvent<HTMLDivElement>) => {
    const view = viewRef.current;
    if (!view) return;

    const rect = rectRef.current;
    const { cx, cy } = localPoint(event);
    const gesture = gestureRef.current;

    hoverRef.current = probeAt(cellAt(view, session.grid, cx, cy));

    if (!gesture) {
      requestDraw();
      return;
    }

    if (gesture.kind === "pan") {
      if (!rect) return;
      viewRef.current = panBy(view, cx - gesture.cx, cy - gesture.cy, rect, session.grid);
      gesture.cx = cx;
      gesture.cy = cy;
      requestDraw();
      return;
    }

    if (gesture.kind === "shape") {
      gesture.head = clampedCell(view, cx, cy);
      requestDraw();
      return;
    }

    if (gesture.kind === "marquee") {
      gesture.cx = cx;
      gesture.cy = cy;
      requestDraw();
      return;
    }

    if (gesture.kind === "vertex") {
      const dragPx = Math.hypot(cx - gesture.cx, cy - gesture.cy);
      if (dragPx >= HEADING_DEADZONE_PX) {
        // The y term is negated because canvas rows grow downward while ROS y
        // grows upward — the same flip worldToGrid applies. Screen-space is
        // enough here (unlike the 3D viewport, which has to raycast the floor
        // because the camera can look from any azimuth): this canvas is always
        // axis-aligned with the map and never rotated.
        gesture.theta = (Math.atan2(-(cy - gesture.cy), cx - gesture.cx) * 180) / Math.PI;
      }
      requestDraw();
      return;
    }

    const { brush, value } = propsRef.current;
    // Recover the samples the browser merged into this event: a fast flick is
    // otherwise a handful of far-apart points, and interpolating between only
    // those loses the curve. Never getPredictedEvents() — predictions get
    // retracted, and retracted paint would be permanent.
    const points =
      typeof event.nativeEvent.getCoalescedEvents === "function"
        ? event.nativeEvent.getCoalescedEvents()
        : [];
    const bounds = event.currentTarget.getBoundingClientRect();
    const samples = points.length
      ? points.map((p) => ({ cx: p.clientX - bounds.left, cy: p.clientY - bounds.top }))
      : [{ cx, cy }];

    for (const sample of samples) {
      const cell = clampedCell(view, sample.cx, sample.cy);
      if (cell.col === gesture.last.col && cell.row === gesture.last.row) continue;
      stampLine(session.grid, gesture.last, cell, brush, value, session.accumulator.sink);
      gesture.last = cell;
    }
    flushStroke();
  };

  const endGesture = (event: React.PointerEvent<HTMLDivElement>) => {
    const gesture = gestureRef.current;
    const view = viewRef.current;
    if (!gesture) return;
    gestureRef.current = null;
    if (event.currentTarget.hasPointerCapture(gesture.pointerId)) {
      event.currentTarget.releasePointerCapture(gesture.pointerId);
    }

    if (gesture.kind === "pan") {
      setPanCursor(false);
      // A press that never really moved was a click, not a drag. With Pan armed
      // in vertex mode that is how a vertex is inspected without disarming, and
      // a click on bare map is how a selection is dropped. It reuses the heading
      // deadzone rather than introducing a second threshold, so "did this drag
      // mean anything" has one answer everywhere on this canvas.
      if (
        gesture.pick &&
        Math.hypot(gesture.cx - gesture.ox, gesture.cy - gesture.oy) <
          HEADING_DEADZONE_PX
      ) {
        propsRef.current.onVertexPick(gesture.pick.id);
      }
      return;
    }

    if (gesture.kind === "marquee") {
      const { ox, oy, cx, cy, additive } = gesture;
      const left = Math.min(ox, cx);
      const right = Math.max(ox, cx);
      const top = Math.min(oy, cy);
      const bottom = Math.max(oy, cy);

      // A band that never opened is a click on bare map, and a click on bare map
      // clears — the same verdict the Pan tool reaches above, so the two tools
      // cannot disagree about what pressing nothing means.
      if (right - left < HEADING_DEADZONE_PX && bottom - top < HEADING_DEADZONE_PX) {
        if (!additive) propsRef.current.onVertexPick(null);
        requestDraw();
        return;
      }

      // Marker centres, not their hit radii: a vertex is a pose and has no
      // extent, so "inside the band" is the only test that matches what the
      // operator drew a rectangle around.
      const ids: string[] = [];
      if (view) {
        for (const vertex of propsRef.current.vertices) {
          const at = vertexScreen(view, session.meta, vertex.x, vertex.y);
          if (at.cx >= left && at.cx <= right && at.cy >= top && at.cy <= bottom) {
            ids.push(vertex.id);
          }
        }
      }
      propsRef.current.onMarquee(ids, additive);
      requestDraw();
      return;
    }

    if (gesture.kind === "vertex") {
      // Nothing in the cell buffer moved, so the accumulator is not consulted
      // and no patch is pushed. Committing an empty patch here would put an
      // entry in the undo stack that undoes nothing, which is worse than no
      // undo at all — the operator would press it and watch a real edit survive.
      propsRef.current.onVertexGesture({
        id: gesture.id,
        pose: { x: gesture.wx, y: gesture.wy, theta: gesture.theta },
      });
      requestDraw();
      return;
    }

    const { brush, value } = propsRef.current;
    if (gesture.kind === "shape" && view) {
      const { anchor, head } = gesture;
      if (propsRef.current.tool === "rect") {
        stampRect(session.grid, anchor, head, value, session.accumulator.sink);
      } else {
        stampLine(session.grid, anchor, head, brush, value, session.accumulator.sink);
      }
      flushStroke();
    }

    const patch = session.accumulator.commit();
    if (patch) propsRef.current.onStrokeCommit(patch);
    requestDraw();
  };

  /**
   * A cancelled pointer commits the stroke rather than discarding it. The buffer
   * was already mutated by the time the browser took the pointer away, and leaving
   * an edit that undo cannot reach is strictly worse than an unexpectedly short
   * stroke.
   */
  const handlePointerCancel = endGesture;

  const panCursor =
    (props.mode === "grid" && props.tool === "pan") ||
    (props.mode === "vertex" && props.vertexTool === "pan") ||
    props.spacePan
      ? "cursor-grab active:cursor-grabbing"
      : "";

  return (
    <div
      ref={containerRef}
      // touch-none: without it a touch drag scrolls the page instead of painting,
      // and the browser fires pointercancel the moment it decides that is a scroll.
      className={cn(
        "relative h-full w-full touch-none overflow-hidden select-none",
        panCursor || "cursor-crosshair",
        className,
      )}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={endGesture}
      onPointerCancel={handlePointerCancel}
      // Right-drag pans, so the context menu has to go: without this, the menu
      // opens on mouseup over the canvas and swallows the pointerup that ends the
      // gesture, leaving the pan stuck to the cursor. There is nothing on this
      // canvas a browser context menu offers anyway — no text, no image to save.
      onContextMenu={(event) => event.preventDefault()}
      onPointerLeave={() => {
        if (gestureRef.current) return;
        hoverRef.current = null;
        requestDraw();
      }}
    >
      <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" />
    </div>
  );
});

function drawCellGrid(
  ctx: CanvasRenderingContext2D,
  view: View,
  rect: { width: number; height: number },
  grid: { width: number; height: number },
  palette: Palette,
): void {
  // Only the visible cell range: at scale 32 on a 1602-wide map, iterating every
  // column would be 1600 strokes for the ~30 on screen.
  const from = screenToGrid(view, 0, 0);
  const to = screenToGrid(view, rect.width, rect.height);
  const colFrom = Math.max(Math.ceil(from.px), 0);
  const colTo = Math.min(Math.floor(to.px), grid.width);
  const rowFrom = Math.max(Math.ceil(from.py), 0);
  const rowTo = Math.min(Math.floor(to.py), grid.height);

  ctx.strokeStyle = palette.cellGrid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let col = colFrom; col <= colTo; col += 1) {
    const x = Math.round(gridToScreen(view, col, 0).cx) + 0.5;
    ctx.moveTo(x, Math.max(view.oy, 0));
    ctx.lineTo(x, Math.min(view.oy + grid.height * view.scale, rect.height));
  }
  for (let row = rowFrom; row <= rowTo; row += 1) {
    const y = Math.round(gridToScreen(view, 0, row).cy) + 0.5;
    ctx.moveTo(Math.max(view.ox, 0), y);
    ctx.lineTo(Math.min(view.ox + grid.width * view.scale, rect.width), y);
  }
  ctx.stroke();
}

/**
 * The in-progress line or rect.
 *
 * A line's preview is rasterized by the *same* stampLine that will commit it, into
 * a collector sink rather than the buffer — so the cells highlighted and the cells
 * painted cannot disagree. Approximating it with ctx.lineWidth is the tempting
 * shortcut and it is a lie: at high zoom the operator would paint cells they were
 * never shown. A rect is exempt because its cell set *is* a screen rectangle.
 */
function drawPreview(
  ctx: CanvasRenderingContext2D,
  view: View,
  session: GridSession,
  gesture: Gesture | null,
  props: GridCanvasProps,
  palette: Palette,
): void {
  if (!gesture || gesture.kind !== "shape") return;
  const { anchor, head } = gesture;
  const { grid } = session;

  ctx.save();
  ctx.globalAlpha = 0.45;
  ctx.fillStyle = palette.cmd;

  if (props.tool === "rect") {
    const colFrom = Math.min(anchor.col, head.col);
    const rowFrom = Math.min(anchor.row, head.row);
    const cols = Math.abs(head.col - anchor.col) + 1;
    const rows = Math.abs(head.row - anchor.row) + 1;
    const { cx, cy } = gridToScreen(view, colFrom, rowFrom);
    ctx.fillRect(cx, cy, cols * view.scale, rows * view.scale);
    ctx.restore();
    return;
  }

  const cells: number[] = [];
  let overflow = false;
  const collect: CellSink = (index) => {
    if (cells.length >= PREVIEW_CELL_LIMIT) {
      overflow = true;
      return;
    }
    cells.push(index);
  };
  stampLine(grid, anchor, head, props.brush, props.value, collect);

  if (overflow) {
    // Zoomed far out, where individual cells are sub-pixel anyway.
    const a = gridToScreen(view, anchor.col + 0.5, anchor.row + 0.5);
    const b = gridToScreen(view, head.col + 0.5, head.row + 0.5);
    ctx.strokeStyle = palette.cmd;
    ctx.lineWidth = Math.max(props.brush * view.scale, 1);
    ctx.beginPath();
    ctx.moveTo(a.cx, a.cy);
    ctx.lineTo(b.cx, b.cy);
    ctx.stroke();
    ctx.restore();
    return;
  }

  const size = Math.max(view.scale, 1);
  for (const index of cells) {
    const col = index % grid.width;
    const row = (index - col) / grid.width;
    const { cx, cy } = gridToScreen(view, col, row);
    ctx.fillRect(cx, cy, size, size);
  }
  ctx.restore();
}

/**
 * The brush footprint, drawn at `brush * scale` CSS px.
 *
 * Brush size is in cells, which is right for a grid editor but surprising on
 * screen: a size-31 brush covers a huge area zoomed out and a small one zoomed in.
 * Scaling the ring is what keeps it honest about what a click will paint.
 */
function drawBrushRing(
  ctx: CanvasRenderingContext2D,
  view: View,
  gesture: Gesture | null,
  hover: CellProbe | null,
  props: GridCanvasProps,
  palette: Palette,
): void {
  // Nothing is being painted in vertex mode, so a footprint would be a promise
  // about cells that no press there will touch.
  if (props.mode === "vertex") return;
  if (!hover || props.tool === "pan" || props.spacePan) return;
  if (gesture?.kind === "pan") return;

  const diameter = props.tool === "rect" ? 1 : props.brush;
  const { cx, cy } = gridToScreen(view, hover.col + 0.5, hover.row + 0.5);
  const size = Math.max(diameter * view.scale, MIN_RING_PX);

  ctx.strokeStyle = palette.cmd;
  ctx.lineWidth = 1;
  ctx.strokeRect(
    Math.round(cx - size / 2) + 0.5,
    Math.round(cy - size / 2) + 0.5,
    size,
    size,
  );
}

/** A map-frame pose in container-local CSS pixels, via the one view transform. */
function vertexScreen(view: View, meta: MapMetadata, wx: number, wy: number) {
  const { px, py } = worldToGrid(wx, wy, meta);
  return gridToScreen(view, px, py);
}

/**
 * The vertex layer: stored vertices, the staged draft, and the one being aimed.
 *
 * Drawn in every mode, not only in vertex mode. Where the vertices are is
 * information a grid edit needs too — repainting a wall that a CHARGER stop sits
 * against is exactly when you want to see it — and hiding them would make the
 * mode switch feel like it loaded different data rather than changed what a
 * press does.
 */
function drawVertices(
  ctx: CanvasRenderingContext2D,
  view: View,
  meta: MapMetadata,
  gesture: Gesture | null,
  props: GridCanvasProps,
  palette: Palette,
): void {
  const aiming = gesture?.kind === "vertex" ? gesture : null;
  // Rebuilt per frame rather than kept in the shell: a Set handed down as a prop
  // would be a new identity every render and defeat this component's memo, and
  // the selection is at most a few dozen ids.
  const selected = new Set(props.selectedIds);

  ctx.save();
  ctx.lineJoin = "round";
  ctx.lineCap = "round";

  for (const vertex of props.vertices) {
    // Mid-gesture the aimed vertex follows the pointer rather than its stored
    // heading; committing is the panel's job, so the row itself has not moved.
    const live = aiming?.id === vertex.id;
    const lit = live || selected.has(vertex.id);
    const at = vertexScreen(view, meta, vertex.x, vertex.y);
    drawMarker(ctx, {
      cx: at.cx,
      cy: at.cy,
      theta: live ? aiming.theta : vertex.theta,
      colour: lit ? palette.cmd : palette.vertex,
      emphasis: lit,
      glyph: vertexGlyph(vertex.type),
      label: vertex.name,
      dashed: false,
    });
  }

  // A draft is drawn last and dashed: it is the one mark on screen that is not
  // in the database yet, and dashing says so without needing a second hue —
  // `cmd` already means "a value the operator is setting", the same as the brush
  // ring it shares the canvas with.
  const draft = aiming && aiming.id === null ? { ...aiming, x: aiming.wx, y: aiming.wy } : props.draft;
  if (draft) {
    const at = vertexScreen(view, meta, draft.x, draft.y);
    drawMarker(ctx, {
      cx: at.cx,
      cy: at.cy,
      theta: draft.theta,
      colour: palette.cmd,
      emphasis: true,
      glyph: null,
      label: null,
      dashed: true,
    });
  }

  ctx.restore();
}

/**
 * The rubber band, while one is being dragged.
 *
 * Drawn in `cmd` like the brush ring and the draft marker, because it is the
 * same kind of thing: a value the operator is in the middle of setting. The wash
 * is faint on purpose — the band's whole job is to let you see which markers are
 * about to be caught, and a solid fill would hide the ones under it.
 */
function drawMarquee(
  ctx: CanvasRenderingContext2D,
  gesture: Gesture | null,
  palette: Palette,
): void {
  if (!gesture || gesture.kind !== "marquee") return;

  const x = Math.min(gesture.ox, gesture.cx);
  const y = Math.min(gesture.oy, gesture.cy);
  const width = Math.abs(gesture.cx - gesture.ox);
  const height = Math.abs(gesture.cy - gesture.oy);

  ctx.save();
  ctx.fillStyle = palette.cmd;
  ctx.globalAlpha = 0.1;
  ctx.fillRect(x, y, width, height);
  ctx.globalAlpha = 1;

  ctx.strokeStyle = palette.cmd;
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 3]);
  // Half-pixel offset, like the extent hairline: without it a 1 px stroke lands
  // across two device rows and reads as a grey smear rather than a line.
  ctx.strokeRect(Math.round(x) + 0.5, Math.round(y) + 0.5, Math.round(width), Math.round(height));
  ctx.restore();
}

/**
 * The robot where it is standing right now: its footprint, pointed at its heading.
 *
 * A footprint outline rather than another ring-and-arrow marker, because it has to
 * be distinguishable from the vertices at a glance in a place where it will often
 * be sitting on top of one — the operator's usual reason for opening this screen
 * with the robot live is to mark the spot it is parked on. Shape, hue and size all
 * say "not a vertex": the vertices are pointed rings in the neutral vertex hue at a
 * fixed pixel size, this is a body in `live` green that grows with the zoom.
 *
 * The nose is part of the outline rather than a separate arrow so that the whole
 * mark is one shape — at the zoom levels where the body is only a dozen pixels
 * across, a detached arrowhead reads as a second object.
 */
function drawRobot(
  ctx: CanvasRenderingContext2D,
  view: View,
  meta: MapMetadata,
  pose: PlanarPose,
  palette: Palette,
): void {
  const at = vertexScreen(view, meta, pose.x, pose.y);
  // px per metre: cells per metre from the map, screen px per cell from the view.
  const pxPerM = view.scale / meta.resolution;
  const length = Math.max(ROBOT_LENGTH_M * pxPerM, MIN_ROBOT_LENGTH_PX);
  const width = length * (ROBOT_WIDTH_M / ROBOT_LENGTH_M);
  const halfLength = length / 2;
  const halfWidth = width / 2;

  ctx.save();
  ctx.translate(at.cx, at.cy);
  // Negated: the pose is CCW from +x in the map frame and screen y grows downward,
  // the same flip worldToGrid does for position.
  ctx.rotate(-(pose.theta * Math.PI) / 180);
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.setLineDash([]);

  // Body pointing down +x (i.e. +x is now "up the heading"): a rectangle whose
  // front edge is pulled out to a nose. The nose eats a fifth of the length, so a
  // square-on view still reads as a rectangle rather than as an arrow.
  const nose = halfLength * 0.4;
  const body = new Path2D();
  body.moveTo(halfLength, 0);
  body.lineTo(halfLength - nose, -halfWidth);
  body.lineTo(-halfLength, -halfWidth);
  body.lineTo(-halfLength, halfWidth);
  body.lineTo(halfLength - nose, halfWidth);
  body.closePath();

  // Haloed first, like every other mark here: the grid beneath is blitted
  // literally, so neither hue is legible over both free space and obstacles.
  ctx.strokeStyle = MARKER_HALO;
  ctx.lineWidth = 3.5;
  ctx.stroke(body);

  // A wash rather than a solid fill: this is the one mark that covers cells
  // instead of pointing at one, and an operator has to be able to see the
  // obstacle it is parked against through it.
  ctx.fillStyle = palette.live;
  ctx.globalAlpha = 0.22;
  ctx.fill(body);
  ctx.globalAlpha = 1;

  ctx.strokeStyle = palette.live;
  ctx.lineWidth = 1.5;
  ctx.stroke(body);

  ctx.restore();
}

interface MarkerSpec {
  cx: number;
  cy: number;
  /** Degrees, CCW from +x in the map frame. */
  theta: number;
  colour: string;
  /** Thicker ring and a filled centre — the selected or in-flight vertex. */
  emphasis: boolean;
  glyph: string | null;
  label: string | null;
  dashed: boolean;
}

/**
 * One marker: a ring, a heading arrow, and a `G · name` caption.
 *
 * Every stroke is laid down twice — once in MARKER_HALO at +2 px width, then in
 * the marker colour. Without it a light-theme marker disappears into an obstacle
 * and a dark-theme one disappears into free space, because the grid beneath is
 * blitted literally and does not follow the theme.
 */
function drawMarker(ctx: CanvasRenderingContext2D, spec: MarkerSpec): void {
  const { cx, cy, colour, emphasis, dashed } = spec;
  const radians = (spec.theta * Math.PI) / 180;
  // Screen y grows downward, map y grows upward — the same flip worldToGrid does.
  const tipX = cx + Math.cos(radians) * VERTEX_ARROW_PX;
  const tipY = cy - Math.sin(radians) * VERTEX_ARROW_PX;

  const shape = new Path2D();
  shape.moveTo(cx + VERTEX_DOT_RADIUS, cy);
  shape.arc(cx, cy, VERTEX_DOT_RADIUS, 0, Math.PI * 2);
  const arrow = new Path2D();
  arrow.moveTo(cx, cy);
  arrow.lineTo(tipX, tipY);
  // Two barbs at ±150° off the heading, so the head reads as an arrow at 18 px.
  for (const offset of [Math.PI * 0.83, -Math.PI * 0.83]) {
    arrow.moveTo(tipX, tipY);
    arrow.lineTo(
      tipX + Math.cos(radians + offset) * 6,
      tipY - Math.sin(radians + offset) * 6,
    );
  }

  const width = emphasis ? 2 : 1.5;

  ctx.setLineDash([]);
  ctx.strokeStyle = MARKER_HALO;
  ctx.lineWidth = width + 2;
  ctx.stroke(shape);
  ctx.stroke(arrow);

  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  if (dashed) ctx.setLineDash([3, 2]);
  ctx.stroke(shape);
  ctx.setLineDash([]);
  ctx.stroke(arrow);

  if (emphasis && !dashed) {
    ctx.fillStyle = colour;
    ctx.fill(shape);
  }

  const caption = [spec.glyph, spec.label].filter(Boolean).join(" · ");
  if (!caption) return;

  // Offset up-right of the dot so the caption never sits under the arrow when
  // the heading points right, which is the default for a click without a drag.
  const tx = cx + VERTEX_DOT_RADIUS + 4;
  const ty = cy - VERTEX_DOT_RADIUS - 4;
  ctx.font = "500 11px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.lineWidth = 3;
  ctx.strokeStyle = MARKER_HALO;
  ctx.strokeText(caption, tx, ty);
  ctx.fillStyle = colour;
  ctx.fillText(caption, tx, ty);
}
