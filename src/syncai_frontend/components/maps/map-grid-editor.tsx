"use client";

import * as React from "react";

import {
  GridCanvas,
  type CellProbe,
  type EditMode,
  type EditTool,
  type VertexGesture,
} from "@/components/maps/grid-canvas";
import { GridStatus } from "@/components/maps/grid-status";
import { GridToolbar, type SaveState } from "@/components/maps/grid-toolbar";
import { VertexPanel } from "@/components/maps/vertex-panel";
import { useMapGrid } from "@/hooks/use-map-grid";
import { useMapVertices, type UseMapVertices } from "@/hooks/use-map-vertices";
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
  const [selectedId, setSelectedId] = React.useState<string | null>(null);
  /** A re-pose of the selected vertex, awaiting Save. */
  const [stagedPose, setStagedPose] = React.useState<PlanarPose | null>(null);
  const [placing, setPlacing] = React.useState(false);

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

  const selected = vertexList.find((vertex) => vertex.id === selectedId) ?? null;

  const clearVertexEdit = React.useCallback(() => {
    setDraft(null);
    setSelectedId(null);
    setStagedPose(null);
    setPlacing(false);
  }, []);

  const changeMode = React.useCallback(
    (next: EditMode) => {
      setMode(next);
      // Back to grid mode with a draft still staged would leave a dashed marker
      // on the canvas and no panel to commit or dismiss it.
      if (next === "grid") clearVertexEdit();
    },
    [clearVertexEdit],
  );

  /**
   * Arm — or disarm — a re-place of the selected vertex.
   *
   * A toggle rather than `setPlacing(true)`, and that is the only usable shape:
   * once armed, VertexForm relabels the button to "Press the map", so a
   * set-only handler would make pressing it again a no-op and leave the operator
   * with no way out except actually moving the vertex or changing mode. Same
   * pattern as the dashboard's pick modes (see `armPick` in pointcloud-view).
   */
  const armPlace = React.useCallback(() => setPlacing((armed) => !armed), []);

  /**
   * Make `id` the subject of the panel, dropping whatever the last one was.
   *
   * Selecting is also how you leave a draft, a staged pose or an armed re-place:
   * none of the three survives a change of subject.
   */
  const selectVertex = React.useCallback(
    (id: string | null) => {
      clearVertexEdit();
      setSelectedId(id);
      clearVertexError();
    },
    [clearVertexEdit, clearVertexError],
  );

  const pickVertex = React.useCallback(
    (id: string | null) => {
      // While a re-place is armed the canvas suppresses marker hit-testing, so
      // `id` is always null here — and acting on it would clear the very
      // selection the press is about to move.
      //
      // The guard lives on this wrapper rather than in selectVertex because the
      // panel calls the same "select" for its list rows AND for the edit form's
      // Close button, and those have to keep working while armed — Close is the
      // operator's way out, so making it dead exactly then would be the worst
      // possible moment for it.
      if (placing) return;
      selectVertex(id);
    },
    [placing, selectVertex],
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

      if (placing) {
        setStagedPose(pose);
        setPlacing(false);
        return;
      }

      setSelectedId(null);
      setStagedPose(null);
      setDraft(pose);
    },
    [placing, vertexList],
  );

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

  const deleteSelected = React.useCallback(async () => {
    if (!selectedId) return;
    if (await removeVertex(selectedId)) clearVertexEdit();
  }, [selectedId, removeVertex, clearVertexEdit]);

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
  }, [undo, redo, fit]);

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
        value={value}
        brush={brush}
        spacePan={spacePan}
        fitNonce={fitNonce}
        onStrokeCommit={commitPatch}
        onHover={setHover}
        onScaleChange={setScale}
        vertices={vertexList}
        draft={draft}
        selectedId={selectedId}
        placing={placing}
        onVertexPick={pickVertex}
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
        * / selectedId / stagedPose / placing, and VertexForm is keyed on "draft"
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
          stagedPose={stagedPose}
          placing={placing}
          // selectVertex, not pickVertex: this one also backs the form's Close.
          onSelect={selectVertex}
          onArmPlace={armPlace}
          // A draft and a selection are mutually exclusive by construction, so
          // clearing the whole vertex edit *is* "drop the draft".
          onCancelDraft={clearVertexEdit}
          onCreate={createFromDraft}
          onSave={saveSelected}
          onDelete={deleteSelected}
        />
      )}

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
