"use client";

import { MaximizeIcon, Redo2Icon, Undo2Icon } from "lucide-react";

import { Chip, Segmented, overlayPanel } from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import { BRUSH_SIZES, FREE, OCCUPIED, UNKNOWN, type GridValue } from "@/lib/map/grid";
import type { EditTool } from "@/components/maps/grid-canvas";

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

export interface GridToolbarProps {
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
  saving: boolean;
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
  saving,
  onSave,
  className,
}: GridToolbarProps) {
  const shapeTool = tool === "brush" || tool === "line";

  return (
    // w-56 matches the dashboard's overlay controls, and is what "FREE / UNKNOWN /
    // OBSTACLE" needs: eight condensed caps plus padding, three times over.
    <div className={cn(overlayPanel, "flex w-56 flex-col gap-2 p-2.5", className)}>
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

      {/* Cells, not pixels — the number is the count of cells across, which is what
       * you are actually deciding about. Discrete sizes rather than a slider: no
       * slider exists in components/ui, and knowing you are painting exactly 7
       * cells is worth more here than continuous control. */}
      <Row label={`Size · ${brush} cell${brush === 1 ? "" : "s"}`}>
        <Segmented
          stretch
          value={`${brush}` as (typeof SIZES)[number]["value"]}
          options={SIZES}
          onChange={(next) => onBrushChange(Number(next))}
          className={shapeTool ? undefined : "opacity-40"}
        />
      </Row>

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
        disabled={!dirty || saving}
        onClick={onSave}
        className="instrument-label h-7 rounded-sm border border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd transition-colors hover:bg-signal-cmd/20 disabled:border-hairline disabled:bg-transparent disabled:text-muted-foreground"
      >
        {saving ? "Saving…" : "Save"}
      </button>
    </div>
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
