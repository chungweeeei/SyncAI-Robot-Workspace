"use client";

import { MaximizeIcon, Redo2Icon, Undo2Icon } from "lucide-react";

import {
  Chip,
  Segmented,
  TONE_TEXT,
  overlayPanel,
  type Tone,
} from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import { BRUSH_SIZES, FREE, OCCUPIED, UNKNOWN, type GridValue } from "@/lib/map/grid";
import type { EditMode, EditTool } from "@/components/maps/grid-canvas";

const MODES: readonly { value: EditMode; label: string }[] = [
  { value: "grid", label: "Grid" },
  { value: "vertex", label: "Vertex" },
];

const TOOLS: readonly { value: EditTool; label: string }[] = [
  { value: "brush", label: "Brush" },
  { value: "line", label: "Line" },
  { value: "rect", label: "Rect" },
  { value: "pan", label: "Pan" },
];

/**
 * There is no eraser, and that is not an omission: on an occupancy grid "erase"
 * has to mean a specific value, and painting Free *is* the erase — it is what the
 * hand-editing this screen replaces was doing to phantom obstacles. Naming a
 * fourth tool "eraser" would only hide which of the three bytes it writes.
 */
const VALUES: readonly { value: `${GridValue}`; label: string }[] = [
  { value: `${FREE}`, label: "Free" },
  { value: `${UNKNOWN}`, label: "Unknown" },
  { value: `${OCCUPIED}`, label: "Obstacle" },
];

const SIZES = BRUSH_SIZES.map((size) => ({
  value: `${size}` as const,
  label: `${size}`,
}));

/**
 * What the last save attempt did, as a state rather than an event.
 *
 * The two things an operator must not miss — "not saved" and "saved but the
 * robot is still on the old map" — are properties of the editor as it stands,
 * so they are rendered in place next to the Unsaved chip. A toast is the wrong
 * container for them (and there is none in this app): anything that
 * auto-dismisses is guaranteed to dismiss the one message that matters.
 */
export type SaveState =
  | { kind: "idle" }
  | { kind: "saving" }
  /** Written to disk. `reloaded` is whether the running stack picked it up. */
  | { kind: "saved"; active: boolean; reloaded: boolean; message: string }
  | { kind: "failed"; message: string };

/**
 * `active` is what keeps this from crying wolf: `reloaded: false` covers both
 * "this isn't the map the stack is running, so of course nothing reloaded"
 * (benign, and shouting at it teaches operators to ignore the shout) and "it IS
 * the running map and load_map failed" (the case this whole surface exists for).
 */
function saveNote(
  save: SaveState,
): { tone: Tone; headline: string; detail?: string; alert: boolean } | null {
  if (save.kind === "failed") {
    return { tone: "warn", headline: "Not saved", detail: save.message, alert: true };
  }
  if (save.kind !== "saved") return null;
  if (save.reloaded) {
    return { tone: "live", headline: "Saved · map reloaded", alert: false };
  }
  if (!save.active) {
    return { tone: "neutral", headline: "Saved", detail: save.message, alert: false };
  }
  return {
    tone: "caution",
    headline: "Saved to disk — the robot is still using the old map.",
    detail: save.message,
    alert: false,
  };
}

export interface GridToolbarProps {
  mode: EditMode;
  onModeChange: (mode: EditMode) => void;
  tool: EditTool;
  onToolChange: (tool: EditTool) => void;
  value: GridValue;
  onValueChange: (value: GridValue) => void;
  brush: number;
  onBrushChange: (brush: number) => void;
  canUndo: boolean;
  canRedo: boolean;
  onUndo: () => void;
  onRedo: () => void;
  onFit: () => void;
  dirty: boolean;
  save: SaveState;
  onSave: () => void;
  className?: string;
}

function IconButton({
  label,
  icon: Icon,
  disabled,
  onClick,
}: {
  label: string;
  icon: typeof Undo2Icon;
  disabled?: boolean;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      title={label}
      aria-label={label}
      disabled={disabled}
      onClick={onClick}
      className="instrument-label flex h-6 items-center gap-1 rounded-sm border border-hairline px-1.5 transition-colors hover:bg-elevated disabled:opacity-40 disabled:hover:bg-transparent"
    >
      <Icon className="size-3.5" aria-hidden />
    </button>
  );
}

export function GridToolbar({
  mode,
  onModeChange,
  tool,
  onToolChange,
  value,
  onValueChange,
  brush,
  onBrushChange,
  canUndo,
  canRedo,
  onUndo,
  onRedo,
  onFit,
  dirty,
  save,
  onSave,
  className,
}: GridToolbarProps) {
  const shapeTool = tool === "brush" || tool === "line";
  const note = saveNote(save);

  return (
    // w-56 matches the dashboard's overlay controls, and is what "FREE / UNKNOWN /
    // OBSTACLE" needs: eight condensed caps plus padding, three times over.
    <div className={cn(overlayPanel, "flex w-56 flex-col gap-2 p-2.5", className)}>
      <Row label="Mode">
        <Segmented stretch value={mode} options={MODES} onChange={onModeChange} />
      </Row>

      {/* The paint controls describe a stroke, and vertex mode makes none. The
       * block below them stays in both modes, deliberately: the grid can be
       * dirty while the operator is placing vertices, and hiding Save because a
       * mode toggle moved is how unsaved cells get lost. */}
      {mode === "grid" && (
        <>
          <Row label="Tool">
            <Segmented stretch value={tool} options={TOOLS} onChange={onToolChange} />
          </Row>

          <Row label="Paint">
            <Segmented
              stretch
              value={`${value}` as `${GridValue}`}
              options={VALUES}
              onChange={(next) => onValueChange(Number(next) as GridValue)}
            />
          </Row>

          {/* Cells, not pixels — the number is the count of cells across, which is
           * what you are actually deciding about. Discrete sizes rather than a
           * slider: no slider exists in components/ui, and knowing you are painting
           * exactly 7 cells is worth more here than continuous control. */}
          <Row label={`Size · ${brush} cell${brush === 1 ? "" : "s"}`}>
            <Segmented
              stretch
              value={`${brush}` as (typeof SIZES)[number]["value"]}
              options={SIZES}
              onChange={(next) => onBrushChange(Number(next))}
              className={shapeTool ? undefined : "opacity-40"}
            />
          </Row>
        </>
      )}

      <div className="flex items-center gap-1.5 border-t border-hairline pt-2">
        <IconButton label="Undo" icon={Undo2Icon} disabled={!canUndo} onClick={onUndo} />
        <IconButton label="Redo" icon={Redo2Icon} disabled={!canRedo} onClick={onRedo} />
        <IconButton label="Fit to view" icon={MaximizeIcon} onClick={onFit} />
        {dirty && (
          <Chip tone="caution" className="ml-auto">
            Unsaved
          </Chip>
        )}
      </div>

      <button
        type="button"
        disabled={!dirty || save.kind === "saving"}
        onClick={onSave}
        className="instrument-label h-7 rounded-sm border border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd transition-colors hover:bg-signal-cmd/20 disabled:border-hairline disabled:bg-transparent disabled:text-muted-foreground"
      >
        {save.kind === "saving" ? "Saving…" : "Save"}
      </button>

      {note && (
        <p
          role={note.alert ? "alert" : "status"}
          className={cn(
            "text-[11px] leading-tight",
            note.tone === "neutral" ? "text-muted-foreground" : TONE_TEXT[note.tone],
          )}
        >
          {note.headline}
          {note.detail && (
            <span className="mt-0.5 block text-muted-foreground">{note.detail}</span>
          )}
        </p>
      )}

      {/*
       * Right-drag is listed first because it is the one an operator coming from
       * the dashboard expects to find: there the view moves under a plain drag, and
       * here the left button is the edit, so the view got its own button.
       *
       * The rest were all implemented and mentioned nowhere, which made them
       * effectively private. That matters most in vertex mode, where the Tool row
       * above is hidden and these drags are the only way to move the view.
       */}
      <p className="border-t border-hairline pt-2 text-[11px] leading-tight text-muted-foreground">
        Right-drag to move the view · also <Key>Space</Key> or middle-drag ·{" "}
        <Key>0</Key> to fit · scroll to zoom
      </p>
    </div>
  );
}

/** A key cap, sized to sit inside an 11px line without changing its height. */
function Key({ children }: { children: React.ReactNode }) {
  return (
    <kbd className="readout rounded-[3px] border border-hairline px-1 py-px text-[10px] text-foreground">
      {children}
    </kbd>
  );
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div>
      <p className="instrument-label mb-1 text-muted-foreground">{label}</p>
      {children}
    </div>
  );
}
