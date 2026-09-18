"use client";

import {
  BrushIcon,
  HandIcon,
  MapPinPlusIcon,
  MaximizeIcon,
  Redo2Icon,
  SlashIcon,
  SquareDashedMousePointerIcon,
  SquareIcon,
  Undo2Icon,
} from "lucide-react";

import {
  Chip,
  Segmented,
  TONE_TEXT,
  overlayPanel,
  type Tone,
} from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import { BRUSH_SIZES, FREE, OCCUPIED, UNKNOWN, type GridValue } from "@/lib/map/grid";
import type { EditMode, EditTool, VertexTool } from "@/components/maps/grid-canvas";

const MODES: readonly { value: EditMode; label: string }[] = [
  { value: "grid", label: "Grid" },
  { value: "vertex", label: "Vertex" },
];

/**
 * One armable tool: what it is worth, what it does, and the shape it wears.
 *
 * Both tool rows are icons, and for the same reason. Four or five verbs of
 * similar length and similar shape, set in the same condensed caps as every
 * other label in a 224 px panel, are a row of grey text that has to be read
 * before it can be used; a hand, a brush, a stroke and a square are four
 * silhouettes, and which one is armed is legible from the shape alone. That
 * matters most here, because the armed tool is the whole difference between
 * dragging the map and painting on it — the trade that DEFAULT_TOOL and
 * DEFAULT_VERTEX_TOOL exist to make safe.
 *
 * `label` is not lost by that: it is the accessible name, half the tooltip, and
 * is spelled out in the Row heading above the icons, so what is armed is always
 * readable in words somewhere on screen. Nothing here is icon-only.
 */
interface ToolOption<T extends string> {
  value: T;
  label: string;
  /** The rest of the tooltip, after the label: what a press will do. */
  hint: string;
  icon: typeof HandIcon;
}

/*
 * Pan is first because it is what the editor opens in (see DEFAULT_TOOL in
 * map-grid-editor.tsx). The row reads left-to-right as "here is where you
 * start, here is what you can arm", and leaving Pan in the trailing slot it
 * used to occupy would put the lit segment at the far end of the row on load —
 * which reads as an odd leftover setting rather than a deliberate resting state.
 *
 * Line is a slash rather than a dash: a horizontal bar is the minus glyph and
 * reads as "remove", which is the one thing no tool on this row does.
 */
const TOOLS: readonly ToolOption<EditTool>[] = [
  { value: "pan", label: "Pan", hint: "drag the map", icon: HandIcon },
  { value: "brush", label: "Brush", hint: "paint cells freehand", icon: BrushIcon },
  { value: "line", label: "Line", hint: "drag a straight stroke", icon: SlashIcon },
  { value: "rect", label: "Rect", hint: "drag a filled rectangle", icon: SquareIcon },
];

/** Vertex mode's counterpart to TOOLS. Pan leads, for the same reason. */
const VERTEX_TOOLS: readonly ToolOption<VertexTool>[] = [
  { value: "pan", label: "Pan", hint: "drag the map", icon: HandIcon },
  { value: "place", label: "Place", hint: "press the map to stage a vertex", icon: MapPinPlusIcon },
  {
    value: "select",
    label: "Select",
    hint: "drag a box over vertices; Shift adds",
    icon: SquareDashedMousePointerIcon,
  },
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
  vertexTool: VertexTool;
  onVertexToolChange: (tool: VertexTool) => void;
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

/**
 * A tool row: one icon segment per tool, styled as a `Segmented` control.
 *
 * Hand-rolled rather than made from `Segmented`, which takes a string label and
 * renders nothing else. Widening it to a ReactNode would give every caller a
 * control whose options need a separate accessible name — an icon segment has
 * no text for a screen reader to read — and these two rows are the only ones in
 * the console that want icons. The active styling is copied deliberately: this
 * reads as the same kind of control as the Mode row above it because it is one.
 */
function ToolRow<T extends string>({
  value,
  options,
  onChange,
}: {
  value: T;
  options: readonly ToolOption<T>[];
  onChange: (tool: T) => void;
}) {
  return (
    <div className="flex w-full overflow-hidden rounded-sm border border-hairline">
      {options.map(({ value: option, label, hint, icon: Icon }) => {
        const active = option === value;
        return (
          <button
            key={option}
            type="button"
            aria-pressed={active}
            title={`${label} — ${hint}`}
            aria-label={label}
            onClick={() => onChange(option)}
            className={cn(
              "flex h-6 min-w-0 flex-1 items-center justify-center border-l border-hairline transition-colors first:border-l-0",
              active
                ? "bg-signal-cmd/12 text-signal-cmd"
                : "text-muted-foreground hover:bg-elevated hover:text-foreground",
            )}
          >
            <Icon className="size-3.5" aria-hidden />
          </button>
        );
      })}
    </div>
  );
}

export function GridToolbar({
  mode,
  onModeChange,
  tool,
  onToolChange,
  vertexTool,
  onVertexToolChange,
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

      {/* Vertex mode's counterpart to the Tool row below: which of the three
       * things a left press can mean is armed. Escape returns it to Pan, and so
       * does every mode change. */}
      {mode === "vertex" && (
        <Row label={`Tool · ${labelOf(VERTEX_TOOLS, vertexTool)}`}>
          <ToolRow value={vertexTool} options={VERTEX_TOOLS} onChange={onVertexToolChange} />
        </Row>
      )}

      {/* The paint controls describe a stroke, and vertex mode makes none. The
       * block below them stays in both modes, deliberately: the grid can be
       * dirty while the operator is placing vertices, and hiding Save because a
       * mode toggle moved is how unsaved cells get lost. */}
      {mode === "grid" && (
        <>
          <Row label={`Tool · ${labelOf(TOOLS, tool)}`}>
            <ToolRow value={tool} options={TOOLS} onChange={onToolChange} />
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
       * The keyboard/mouse hint line that used to close this panel was removed on
       * request. The gestures it documented are all still live — right-drag and
       * middle-drag pan in every mode, Space pans while held, Escape disarms back
       * to Pan, 0 fits, scroll zooms — they are just undocumented on screen
       * again. Both modes now have a Pan tool of their own, so those drags are a
       * convenience rather than, as they were in vertex mode, the only way to
       * move the view at all.
       */}
    </div>
  );
}

/**
 * The armed tool's name, for the heading above its icons.
 *
 * The heading is where the icon row's labels come back as words — see
 * ToolOption. `?? ""` never fires in practice (the state is a union of exactly
 * these values) and is there so a tool added to one list and not the other
 * degrades to a bare "Tool ·" rather than "Tool · undefined".
 */
function labelOf<T extends string>(options: readonly ToolOption<T>[], value: T): string {
  return options.find((option) => option.value === value)?.label ?? "";
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div>
      <p className="instrument-label mb-1 text-muted-foreground">{label}</p>
      {children}
    </div>
  );
}
