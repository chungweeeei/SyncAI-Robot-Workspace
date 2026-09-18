"use client";

import * as React from "react";

import {
  GridCanvas,
  type CellProbe,
  type EditMode,
  type EditTool,
  type VertexGesture,
  type VertexTool,
} from "@/components/maps/grid-canvas";
import { ManualControl } from "@/components/dashboard/manual-control";
import { GridStatus } from "@/components/maps/grid-status";
import { GridToolbar, type SaveState } from "@/components/maps/grid-toolbar";
import { VertexPanel } from "@/components/maps/vertex-panel";
import { useMapGrid } from "@/hooks/use-map-grid";
import { useMapVertices, type UseMapVertices } from "@/hooks/use-map-vertices";
import { useRobotMapPose } from "@/hooks/use-robot-map-pose";
import { saveMapGrid } from "@/lib/api/map";
import type { VertexChanges } from "@/lib/api/vertex";
import { FREE, countValues, type GridValue, type ValueCounts } from "@/lib/map/grid";
import {
  applyCountsDelta,
  applyPatch,
  createUndoStack,
  popRedo,
  popUndo,
  pushPatch,
  type GridPatch,
} from "@/lib/map/patch";
import type { GridSession } from "@/lib/map/session";
import { DEFAULT_VERTEX_TYPE } from "@/lib/map/vertex";
import type { VertexType } from "@/lib/types/map";
import type { PlanarPose } from "@/lib/types/robot";

const DEFAULT_BRUSH = 7;

/**
 * The editor opens in Pan, not in Brush.
 *
 * Opening armed with a brush means the first thing an operator does on a freshly
 * loaded map — drag it to the corner they came here to look at — is a stroke,
 * and on a 1602x1502 grid at fit scale that stroke is hundreds of cells wide
 * before they notice. Undo would reach it, but only if they realised; the map is
 * blitted literally and a Free stroke across free space is invisible.
 *
 * Painting therefore costs one click on the Tool row, which is the trade this
 * makes: an explicit arming gesture for the destructive default, in exchange for
 * "look around" being the safe thing that needs no decision. Right/middle-drag
 * and Space still pan whatever the tool is — Pan being the *default* does not
 * make it the only way.
 */
const DEFAULT_TOOL: EditTool = "pan";

/**
 * Vertex mode opens in Pan too, for the same reason and one more.
 *
 * The same reason: the first thing an operator does on arriving is drag the map
 * to the corner they came for, and until this existed that drag placed a vertex
 * — a whole staged draft, panel and all — because vertex mode had no unarmed
 * state at all. Every press on bare map was a placement.
 *
 * The one more: unlike a stray brush stroke, a stray vertex is not undoable.
 * The vertex layer writes through to the backend and has no history (see
 * hooks/use-map-vertices.ts), so the only reason the old behaviour was
 * survivable is that a draft still needs a name typed before it becomes a row.
 * That is a confirmation step, not a safe default.
 *
 * Placing therefore costs one click on the Tool row, and Escape gives the map
 * back — the same trade grid mode makes for its brush.
 */
const DEFAULT_VERTEX_TOOL: VertexTool = "pan";

/**
 * Loads the map and shows the guard states; EditorSurface does the editing.
 *
 * The split exists so that everything belonging to one loaded grid — the undo
 * history, the cell census — is initialised by *mounting* the surface rather than
 * by clearing state in an effect when the session changes. Patches index into a
 * specific buffer, so carrying a history across a load would corrupt the new one,
 * and a remount makes that impossible by construction.
 */
export function MapGridEditor({
  name,
  onDirtyChange,
}: {
  name: string;
  /** Lets the page guard its back button; see its comment on why it needs this. */
  onDirtyChange?: (dirty: boolean) => void;
}) {
  const { session, status, error } = useMapGrid(name);
  /**
   * Loaded here rather than inside EditorSurface, and that placement is the
   * point of the split. EditorSurface is keyed on the session, so it remounts
   * whenever the grid is reloaded; the vertex list belongs to the *map*, not to
   * one buffer of its cells, and would otherwise be refetched — and any staged
   * edit thrown away — by something that has nothing to do with it.
   */
  const vertices = useMapVertices(name);

  if (status === "error") {
    return (
      <div className="flex h-full items-center justify-center p-6">
        <div className="max-w-md rounded-md border border-hairline bg-panel p-4">
          <p className="instrument-label text-muted-foreground">Cannot edit</p>
          <p className="mt-2 text-sm">{error ?? "The map could not be loaded."}</p>
        </div>
      </div>
    );
  }

  if (!session) {
    return (
      <p className="instrument-label flex h-full items-center justify-center text-muted-foreground">
        Loading {name}…
      </p>
    );
  }

  return (
    <EditorSurface
      key={session.id}
      session={session}
      vertices={vertices}
      onDirtyChange={onDirtyChange}
    />
  );
}

/**
 * The editor shell: everything except pixels.
 *
 * It owns the tool state, the undo history, the keyboard shortcuts and the save
 * flow; GridCanvas owns the buffer, the mirror and the view. The two meet at
 * `GridSession` — see lib/map/session.ts for why the repaint hook lives there
 * rather than behind an imperative handle.
 */
function EditorSurface({
  session,
  vertices,
  onDirtyChange,
}: {
  session: GridSession;
  vertices: UseMapVertices;
  onDirtyChange?: (dirty: boolean) => void;
}) {
  const [mode, setMode] = React.useState<EditMode>("grid");
  const [tool, setTool] = React.useState<EditTool>(DEFAULT_TOOL);
  const [vertexTool, setVertexTool] = React.useState<VertexTool>(DEFAULT_VERTEX_TOOL);
  // Free by default: erasing phantom obstacles is the reason this screen exists.
  const [value, setValue] = React.useState<GridValue>(FREE);
  const [brush, setBrush] = React.useState<number>(DEFAULT_BRUSH);

  const [canUndo, setCanUndo] = React.useState(false);
  const [canRedo, setCanRedo] = React.useState(false);
  const [dirty, setDirty] = React.useState(false);
  const [save, setSaveState] = React.useState<SaveState>({ kind: "idle" });

  /**
   * Bumped by every edit, so a save can tell whether the buffer moved under it.
   *
   * `fetch` copies a BufferSource body synchronously at the call, so a stroke
   * painted while the request is in flight is *not* in what reached disk —
   * clearing `dirty` on that response would mark unsaved cells saved.
   */
  const revisionRef = React.useRef(0);

  const [hover, setHover] = React.useState<CellProbe | null>(null);
  const [scale, setScale] = React.useState(1);
  // Lazy initialiser, not an effect: one full pass over the grid at mount, then
  // maintained incrementally from each patch.
  const [counts, setCounts] = React.useState<ValueCounts>(() =>
    countValues(session.grid),
  );
  const [fitNonce, setFitNonce] = React.useState(0);
  const [spacePan, setSpacePan] = React.useState(false);

  const historyRef = React.useRef(createUndoStack());

  /*
   * Vertex-layer state. None of it feeds `dirty`, and that is the whole point of
   * the write-through design in useMapVertices: the page's back-button guard and
   * the toolbar's Unsaved chip keep describing the gridmap only, so a staged
   * vertex can never be mistaken for unsaved cells.
   */
  const [vertexType, setVertexType] = React.useState<VertexType>(DEFAULT_VERTEX_TYPE);
  const [draft, setDraft] = React.useState<PlanarPose | null>(null);
  /**
   * Every highlighted vertex, not just the one the form is editing.
   *
   * A list rather than a nullable id because the Select tool's band produces
   * sets, and rather than a Set because it is handed to GridCanvas, which is
   * memoized — React state keeps its identity between changes, a Set rebuilt in
   * a render would not. One entry is the ordinary case and behaves exactly as
   * the old single selection did; the panel only opens its editing form at
   * exactly one, because none of what that form does (rename, retype, re-aim)
   * has a sensible meaning spread across several.
   */
  const [selectedIds, setSelectedIds] = React.useState<string[]>([]);
  /**
   * A re-aim of the selected vertex, awaiting Save.
   *
   * Only ever a new *heading* now: a vertex is re-aimed by dragging on its own
   * marker, which anchors at the stored position (see GridCanvas's pointer-down).
   * Moving one to a different place is the dashboard's job — there the stop is
   * drawn over the live cloud, which is the view that can actually show it half
   * inside a wall.
   */
  const [stagedPose, setStagedPose] = React.useState<PlanarPose | null>(null);
  /** The last point the shell asked the canvas to centre on; see GridCanvas.focus. */
  const [focus, setFocus] = React.useState<{ x: number; y: number } | null>(null);

  /**
   * The robot's pose, when it is a pose on the map open here.
   *
   * It is both the source of the panel's capture button and what the canvas
   * draws the robot's footprint from, so the two can never disagree about where
   * the robot is — the mark on the map is the pose the button would stage.
   *
   * Read at 1 Hz from the console's shared poll, so this component re-renders at
   * that rate; the hook memoises on the values, so a parked robot costs one
   * bailed-out memo compare and no repaint. See useRobotMapPose for why it is
   * not the 20 Hz telemetry socket.
   */
  const { pose: robotPose, reason: robotPoseReason } = useRobotMapPose(session.name);

  // Destructured because the hook returns a fresh object each render: passing
  // `vertices.create` inline would give GridCanvas a new callback identity every
  // time and defeat its React.memo, which exists so a pan cannot re-render the
  // toolbar. The list itself is state, so its identity is stable between changes.
  //
  // status / error / busy are the panel's three, and deliberately reach nothing
  // else — GridCanvas never sees them, so a vertex write in flight cannot
  // re-render the canvas. All eight are prefixed because `save` and `dirty` in
  // this same scope describe the *gridmap*: a bare `error` or `busy` beside them
  // reads as if it did too.
  const {
    vertices: vertexList,
    status: vertexStatus,
    error: vertexError,
    busy: vertexBusy,
    create: createVertex,
    update: updateVertex,
    remove: removeVertex,
    clearError: clearVertexError,
  } = vertices;

  const selectedId = selectedIds.length === 1 ? selectedIds[0] : null;
  const selected = vertexList.find((vertex) => vertex.id === selectedId) ?? null;

  const clearVertexEdit = React.useCallback(() => {
    setDraft(null);
    setSelectedIds([]);
    setStagedPose(null);
  }, []);

  const changeMode = React.useCallback(
    (next: EditMode) => {
      setMode(next);
      // Both directions land unarmed. Arriving in vertex mode still holding
      // Place from last time means the drag that was meant as "show me the other
      // end of the corridor" stages a vertex instead — which is the whole thing
      // DEFAULT_VERTEX_TOOL exists to stop, and a mode toggle is exactly when it
      // would come back.
      setVertexTool(DEFAULT_VERTEX_TOOL);
      // Back to grid mode with a draft still staged would leave a dashed marker
      // on the canvas and no panel to commit or dismiss it.
      if (next === "grid") clearVertexEdit();
    },
    [clearVertexEdit],
  );

  /**
   * Make `id` the subject of the panel, dropping whatever the last one was.
   *
   * Selecting is also how you leave a draft or a staged re-aim: neither survives
   * a change of subject.
   */
  const selectVertex = React.useCallback(
    (id: string | null) => {
      clearVertexEdit();
      setSelectedIds(id ? [id] : []);
      clearVertexError();
    },
    [clearVertexEdit, clearVertexError],
  );

  /**
   * Shift-click on a marker: add it, or drop it if it is already in.
   *
   * The half of multi-select a rectangle cannot do — three stops scattered down
   * a corridor have no band that catches them and nothing else. Toggling down to
   * exactly one is not a special case: the panel simply opens its editing form
   * again, because that is what one selected vertex means everywhere else.
   */
  const toggleVertex = React.useCallback(
    (id: string) => {
      setDraft(null);
      setStagedPose(null);
      clearVertexError();
      setSelectedIds((current) =>
        current.includes(id)
          ? current.filter((other) => other !== id)
          : [...current, id],
      );
    },
    [clearVertexError],
  );

  /** A finished band. `additive` is Shift: union rather than replace. */
  const selectMany = React.useCallback(
    (ids: string[], additive: boolean) => {
      setDraft(null);
      setStagedPose(null);
      clearVertexError();
      setSelectedIds((current) =>
        additive
          ? [...current, ...ids.filter((id) => !current.includes(id))]
          : ids,
      );
    },
    [clearVertexError],
  );

  const handleVertexGesture = React.useCallback(
    ({ id, pose }: VertexGesture) => {
      if (id !== null) {
        // Re-aimed in place. The canvas echoes the stored heading back verbatim
        // when the drag stayed inside its deadzone, so this exact comparison
        // holds and a plain click-to-select does not arm the Save button.
        const existing = vertexList.find((vertex) => vertex.id === id);
        if (existing && existing.theta === pose.theta) return;
        setStagedPose(pose);
        return;
      }

      setSelectedIds([]);
      setStagedPose(null);
      setDraft(pose);
    },
    [vertexList],
  );

  /**
   * Stage a draft where the robot is standing, and go and look at it.
   *
   * A snapshot, not a live binding: `robotPose` keeps moving after this, and a
   * draft that crept across the map while its name was being typed would be a
   * vertex nobody placed. The same clearing as a press on the map — a draft and
   * a selection are mutually exclusive — so it is usable with a vertex already
   * open in the form.
   */
  const placeAtRobot = React.useCallback(() => {
    if (!robotPose) return;
    setSelectedIds([]);
    setStagedPose(null);
    setDraft(robotPose);
    // A fresh object every press, because identity is what triggers the canvas:
    // pressing again after panning away has to bring the marker back.
    setFocus({ x: robotPose.x, y: robotPose.y });
  }, [robotPose]);

  const createFromDraft = React.useCallback(
    async (name: string, type: VertexType) => {
      if (!draft) return;
      const created = await createVertex({ name, type, ...draft });
      // Cleared rather than selected: placing a run of stops is the common case,
      // and the list view is where the next one starts.
      if (created) setDraft(null);
    },
    [createVertex, draft],
  );

  const saveSelected = React.useCallback(
    async (changes: VertexChanges) => {
      if (!selectedId) return;
      if (await updateVertex(selectedId, changes)) setStagedPose(null);
    },
    [selectedId, updateVertex],
  );

  /**
   * Delete everything selected — one vertex from the form, or a whole band.
   *
   * One request per id, run in series. Not caution about the LAN: useMapVertices
   * has a single `busy` flag and a single `error` slot for the whole hook, so
   * concurrent writes would race the flag and leave the panel showing whichever
   * failure happened to land last. In series, the first failure stops the run
   * with the rest still selected, which is both an honest report and the state a
   * retry wants. There is no batch endpoint to use instead — only create takes a
   * list; PUT and DELETE are per id (see lib/api/vertex.ts).
   */
  const deleteSelected = React.useCallback(async () => {
    for (let index = 0; index < selectedIds.length; index += 1) {
      if (!(await removeVertex(selectedIds[index]))) {
        // The failed one stays selected with everything after it, so the count in
        // the panel is what is left to do rather than what was asked for.
        setSelectedIds(selectedIds.slice(index));
        return;
      }
    }
    clearVertexEdit();
  }, [selectedIds, removeVertex, clearVertexEdit]);

  React.useEffect(() => {
    onDirtyChange?.(dirty);
  }, [dirty, onDirtyChange]);

  const commitPatch = React.useCallback((patch: GridPatch) => {
    pushPatch(historyRef.current, patch);
    setCanUndo(true);
    setCanRedo(false);
    setDirty(true);
    revisionRef.current += 1;
    // The note describes the buffer as it was saved; once the buffer moves on it
    // is stale, and "Saved" next to a lit Unsaved chip is the one genuinely
    // confusing pair this panel can show.
    setSaveState({ kind: "idle" });
    setCounts((current) => applyCountsDelta(current, patch, "after"));
  }, []);

  const step = React.useCallback(
    (direction: "undo" | "redo") => {
      const stack = historyRef.current;
      const patch = direction === "undo" ? popUndo(stack) : popRedo(stack);
      if (!patch) return;

      const side = direction === "undo" ? "before" : "after";
      applyPatch(session.grid, patch, side);
      session.repaint?.(patch.bounds);
      setCounts((current) => applyCountsDelta(current, patch, side));
      setCanUndo(stack.undo.length > 0);
      setCanRedo(stack.redo.length > 0);
      // Still dirty after undoing to the start: the stack is byte-capped, so an
      // empty undo stack does not prove the buffer matches what was loaded.
      setDirty(true);
      revisionRef.current += 1;
      setSaveState({ kind: "idle" });
    },
    [session],
  );

  const undo = React.useCallback(() => step("undo"), [step]);
  const redo = React.useCallback(() => step("redo"), [step]);
  const fit = React.useCallback(() => setFitNonce((n) => n + 1), []);

  /**
   * Write the buffer back, and report what the running stack made of it.
   *
   * Nothing is refetched afterwards, deliberately: the local buffer *is* what was
   * written, byte for byte, so a refetch would re-download and re-decode ~2.4 MB
   * to arrive back where we are — and it would need a new GridSession (patches
   * index into a specific buffer), which means a remount, which would throw away
   * the operator's undo history as the reward for saving.
   */
  const onSave = React.useCallback(async () => {
    const sent = revisionRef.current;
    setSaveState({ kind: "saving" });

    try {
      const result = await saveMapGrid(session.name, session.grid);

      // Only the bytes as of `sent` are on disk; anything painted since is not.
      if (revisionRef.current === sent) setDirty(false);
      setSaveState({
        kind: "saved",
        active: result.active,
        reloaded: result.reloaded,
        message: result.message,
      });
    } catch (cause) {
      // Caught here rather than by the caller: this is wired straight to onClick,
      // so a rejection would be an unhandled one — and `dirty` has to stay true
      // so the button re-enables for a retry.
      setSaveState({
        kind: "failed",
        message:
          cause instanceof Error ? cause.message : "The gridmap could not be saved.",
      });
    }
  }, [session]);

  React.useEffect(() => {
    const isTypingTarget = (target: EventTarget | null) => {
      const element = target as HTMLElement | null;
      return Boolean(
        element &&
          (element.isContentEditable ||
            /^(INPUT|TEXTAREA|SELECT)$/.test(element.tagName ?? "")),
      );
    };

    const onKeyDown = (event: KeyboardEvent) => {
      /*
       * Escape is deliberately above the typing guard, unlike every other
       * shortcut here.
       *
       * A staged draft autofocuses VertexPanel's name field, so an Escape that
       * respected the guard would be dead in precisely the state an operator
       * presses it in — "I did not mean to place that". Nothing in this editor's
       * fields wants Escape for itself, so there is nothing to swallow.
       *
       * It does both halves of "put the mouse back": it drops whatever is staged
       * and disarms both tool axes. One press, not two, because the operator
       * pressing it wants the map back and does not care which of the two states
       * is the one holding it.
       */
      if (event.key === "Escape") {
        event.preventDefault();
        clearVertexEdit();
        setTool(DEFAULT_TOOL);
        setVertexTool(DEFAULT_VERTEX_TOOL);
        return;
      }

      // VertexPanel's name field is the case this was written in anticipation of:
      // `0` and Space are single-key shortcuts, and an editor's shortcuts are
      // exactly what silently eats typing. Ctrl+Z falls through to the field too,
      // becoming the browser's native text undo, which is what you want there.
      if (isTypingTarget(event.target)) return;

      const mod = event.ctrlKey || event.metaKey;
      if (mod && event.key.toLowerCase() === "z") {
        event.preventDefault();
        if (event.shiftKey) redo();
        else undo();
        return;
      }
      if (mod && event.key.toLowerCase() === "y") {
        event.preventDefault();
        redo();
        return;
      }
      if (event.code === "Space" && !event.repeat) {
        // Stop the page-scroll default even though this page does not scroll: it
        // would still scroll an ancestor if the layout ever gains one.
        event.preventDefault();
        setSpacePan(true);
        return;
      }
      if (event.key === "0" && !mod) {
        event.preventDefault();
        fit();
      }
    };

    const onKeyUp = (event: KeyboardEvent) => {
      if (event.code === "Space") setSpacePan(false);
    };
    // A window blur mid-Space would otherwise leave the editor stuck in pan mode.
    const onBlur = () => setSpacePan(false);

    // On window rather than the canvas: the shortcuts have to work without having
    // clicked the canvas first.
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
    };
  }, [undo, redo, fit, clearVertexEdit]);

  /**
   * Covers reload and tab close only. The App Router has no navigation blocker, so
   * an in-app link away from here cannot be intercepted — the page's back button
   * asks for confirmation itself.
   */
  React.useEffect(() => {
    if (!dirty) return;
    const onBeforeUnload = (event: BeforeUnloadEvent) => event.preventDefault();
    window.addEventListener("beforeunload", onBeforeUnload);
    return () => window.removeEventListener("beforeunload", onBeforeUnload);
  }, [dirty]);

  return (
    <div className="relative h-full w-full">
      <GridCanvas
        session={session}
        mode={mode}
        tool={tool}
        vertexTool={vertexTool}
        value={value}
        brush={brush}
        spacePan={spacePan}
        fitNonce={fitNonce}
        focus={focus}
        onStrokeCommit={commitPatch}
        onHover={setHover}
        onScaleChange={setScale}
        vertices={vertexList}
        robotPose={robotPose}
        draft={draft}
        selectedIds={selectedIds}
        onVertexPick={selectVertex}
        onVertexToggle={toggleVertex}
        onMarquee={selectMany}
        onVertexGesture={handleVertexGesture}
      />

      <GridToolbar
        className="absolute top-3 left-3"
        mode={mode}
        // changeMode, never setMode: going back to grid with a draft still staged
        // leaves a dashed marker on the canvas and no panel to commit or dismiss
        // it. Note that setMode typechecks fine here, so this one is on us.
        onModeChange={changeMode}
        tool={tool}
        onToolChange={setTool}
        vertexTool={vertexTool}
        onVertexToolChange={setVertexTool}
        value={value}
        onValueChange={setValue}
        brush={brush}
        onBrushChange={setBrush}
        canUndo={canUndo}
        canRedo={canRedo}
        onUndo={undo}
        onRedo={redo}
        onFit={fit}
        dirty={dirty}
        save={save}
        onSave={onSave}
      />
      {/* Top-right: the toolbar owns top-left and GridStatus bottom-left.
        *
        * Mounted only in vertex mode, because unmounting discards nothing that the
        * mode switch was not already discarding — changeMode("grid") clears draft
        * / selectedIds / stagedPose, and VertexForm is keyed on "draft"
        * or selected.id, so its local name/type state is already gone by then. The
        * one thing worth keeping across the toggle, `vertexType`, lives up here
        * for exactly that reason. Left mounted it would cover 240 px of map in the
        * mode where nothing in it is actionable. */}
      {mode === "vertex" && (
        <VertexPanel
          className="absolute top-3 right-3"
          vertices={vertexList}
          status={vertexStatus}
          error={vertexError}
          busy={vertexBusy}
          type={vertexType}
          onTypeChange={setVertexType}
          draft={draft}
          selected={selected}
          selectedIds={selectedIds}
          stagedPose={stagedPose}
          robotPose={robotPose}
          robotPoseReason={robotPoseReason}
          onUseRobotPose={placeAtRobot}
          onSelect={selectVertex}
          // A draft and a selection are mutually exclusive by construction, so
          // clearing the whole vertex edit *is* "drop the draft", and the same
          // call is what the band selection's Clear does.
          onCancelDraft={clearVertexEdit}
          onClearSelection={clearVertexEdit}
          onCreate={createFromDraft}
          onSave={saveSelected}
          onDelete={deleteSelected}
        />
      )}

      {/* The other half of "Use robot position": the pose you capture is the one
        * you drove the robot to, and without a drive panel here that meant
        * leaving the editor for the dashboard between every stop — which on a
        * dirty gridmap means answering the back button's confirm, losing the
        * view, and coming back to re-find the corridor.
        *
        * Vertex mode only, matching VertexPanel: it belongs to the stop-placing
        * flow, and a drive panel over a screen where the operator is painting
        * cells is a joystick nobody asked for. Leaving the mode therefore also
        * stops the robot — the panel unmounts, the channel closes, and the
        * backend's watchdog zeroes cmd_vel — which is the safe direction for a
        * mode switch to fail in.
        *
        * It comes up disarmed and takes no keyboard until it is armed (see
        * ManualControl), so it cannot eat this editor's shortcuts; its WASD/QE/AD
        * set does not overlap Space / 0 / Ctrl+Z in any case. */}
      {mode === "vertex" && <ManualControl className="absolute right-3 bottom-3" />}

      <GridStatus
        className="absolute bottom-3 left-3"
        meta={session.meta}
        hover={hover}
        scale={scale}
        counts={counts}
      />
    </div>
  );
}
