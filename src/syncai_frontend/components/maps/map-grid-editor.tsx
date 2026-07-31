"use client";

import * as React from "react";

import { GridCanvas, type CellProbe, type EditTool } from "@/components/maps/grid-canvas";
import { GridStatus } from "@/components/maps/grid-status";
import { GridToolbar } from "@/components/maps/grid-toolbar";
import { useMapGrid } from "@/hooks/use-map-grid";
import { saveMapGrid } from "@/lib/api/map";
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

const DEFAULT_BRUSH = 7;

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
    <EditorSurface key={session.id} session={session} onDirtyChange={onDirtyChange} />
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
  onDirtyChange,
}: {
  session: GridSession;
  onDirtyChange?: (dirty: boolean) => void;
}) {
  const [tool, setTool] = React.useState<EditTool>("brush");
  // Free by default: erasing phantom obstacles is the reason this screen exists.
  const [value, setValue] = React.useState<GridValue>(FREE);
  const [brush, setBrush] = React.useState<number>(DEFAULT_BRUSH);

  const [canUndo, setCanUndo] = React.useState(false);
  const [canRedo, setCanRedo] = React.useState(false);
  const [dirty, setDirty] = React.useState(false);
  const [saving, setSaving] = React.useState(false);

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

  React.useEffect(() => {
    onDirtyChange?.(dirty);
  }, [dirty, onDirtyChange]);

  const commitPatch = React.useCallback((patch: GridPatch) => {
    pushPatch(historyRef.current, patch);
    setCanUndo(true);
    setCanRedo(false);
    setDirty(true);
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
    },
    [session],
  );

  const undo = React.useCallback(() => step("undo"), [step]);
  const redo = React.useCallback(() => step("redo"), [step]);
  const fit = React.useCallback(() => setFitNonce((n) => n + 1), []);

  const save = React.useCallback(async () => {
    setSaving(true);
    try {
      await saveMapGrid(session.name, session.grid);
      setDirty(false);
    } finally {
      setSaving(false);
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
      // Nothing on this page has a text field yet, but single-key shortcuts in an
      // editor are exactly what silently eats typing the day one is added.
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
        tool={tool}
        value={value}
        brush={brush}
        spacePan={spacePan}
        fitNonce={fitNonce}
        onStrokeCommit={commitPatch}
        onHover={setHover}
        onScaleChange={setScale}
      />

      <GridToolbar
        className="absolute top-3 left-3"
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
        saving={saving}
        onSave={save}
      />
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
