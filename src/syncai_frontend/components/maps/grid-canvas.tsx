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
  },
  dark: {
    well: "#22282c",
    extent: "#5c6a74",
    cellGrid: "#3a444b",
    cmd: "#45c8f0",
    vertex: "#a8bcc7",
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
/** Click slop around a marker centre. Comfortably larger than the dot itself. */
const VERTEX_HIT_RADIUS = 11;
/** Below this drag distance the gesture is a click and the heading is kept. */
const HEADING_DEADZONE_PX = 10;

const ZOOM_PER_PX = 0.0015;
const WHEEL_LINE_PX = 16;

type Gesture =
  | { kind: "paint"; pointerId: number; last: Cell }
  | { kind: "shape"; pointerId: number; anchor: Cell; head: Cell }
  | { kind: "pan"; pointerId: number; cx: number; cy: number }
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
  /** Staged, not-yet-created vertex. Drawn in the commanded hue. */
  draft: PlanarPose | null;
  /** Highlighted, and the one the panel is editing. */
  selectedId: string | null;
  /**
   * True while the panel has armed a re-place of the selected vertex.
   *
   * It suppresses marker hit-testing for the duration, so the operator can drop
   * the vertex on top of another one without the press being captured by it.
   */
  placing: boolean;
  /** Fired on pointer-down, before any drag, so selection feels immediate. */
  onVertexPick: (id: string | null) => void;
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
    // Last, so markers sit above the shape preview rather than under it.
    drawVertices(ctx, view, session.meta, gestureRef.current, current, palette);

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
  }, [props.vertices, props.draft, props.selectedId, props.mode, requestDraw]);

  // Fit: drop the transform and let draw() rebuild it from the current rect.
  React.useEffect(() => {
    viewRef.current = null;
    requestDraw();
  }, [props.fitNonce, requestDraw]);

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

    const { mode, tool, value, brush, spacePan } = propsRef.current;
    const { cx, cy } = localPoint(event);
    // Right and middle drag always pan, in every mode and whatever the tool.
    //
    // This is the dashboard's bargain, adapted. There, left-drag moves the view
    // and an armed pick mode is what takes the button away; here the default
    // left-drag is the edit — painting is what this screen is *for* — so the view
    // gets its own button instead of the editing being demoted behind a mode.
    // Right is the one that is free: a 2D canvas has no orbit to compete for it,
    // unlike the point-cloud viewport where right-drag is the orbit.
    //
    // `tool` is only consulted in grid mode: the toolbar hides the tool row in
    // vertex mode but keeps the state, so an operator who left Pan selected and
    // then switched would otherwise find vertex mode unable to place anything.
    const panning =
      event.button === 1 ||
      event.button === 2 ||
      spacePan ||
      (mode === "grid" && tool === "pan");

    if (!panning && mode === "vertex") {
      const { placing, draft } = propsRef.current;
      // Suppressed while a re-place is armed, so the selected vertex can be
      // dropped on top of an existing one without the press being stolen by it.
      const hit = placing ? null : vertexAt(view, cx, cy);

      // An existing vertex anchors at its *stored* position, not at the press
      // point. The press has to land within VERTEX_HIT_RADIUS of the marker, so
      // using it would silently move the vertex by up to 11 px worth of metres
      // every time the operator merely selected one to re-aim it.
      let anchor: { wx: number; wy: number };
      if (hit) {
        anchor = { wx: hit.x, wy: hit.y };
      } else {
        // Only a press on the grid places a vertex; the map is letterboxed and a
        // press in the margin means nothing, exactly as it does for a stroke.
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
    gestureRef.current = { kind: "pan", pointerId: event.pointerId, cx, cy };
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
    (props.mode === "grid" && props.tool === "pan") || props.spacePan
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

  ctx.save();
  ctx.lineJoin = "round";
  ctx.lineCap = "round";

  for (const vertex of props.vertices) {
    // Mid-gesture the aimed vertex follows the pointer rather than its stored
    // heading; committing is the panel's job, so the row itself has not moved.
    const live = aiming?.id === vertex.id;
    const at = vertexScreen(view, meta, vertex.x, vertex.y);
    drawMarker(ctx, {
      cx: at.cx,
      cy: at.cy,
      theta: live ? aiming.theta : vertex.theta,
      colour: live || vertex.id === props.selectedId ? palette.cmd : palette.vertex,
      emphasis: live || vertex.id === props.selectedId,
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
