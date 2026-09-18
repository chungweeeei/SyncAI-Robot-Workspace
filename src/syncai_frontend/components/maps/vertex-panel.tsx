"use client";

import * as React from "react";
import { LocateFixedIcon, Trash2Icon } from "lucide-react";

import { Chip, Readout, Segmented, overlayPanel } from "@/components/console/instrument";
import { Input } from "@/components/ui/input";
import type { VertexChanges } from "@/lib/api/vertex";
import { VERTEX_TYPES, vertexGlyph } from "@/lib/map/vertex";
import { cn } from "@/lib/utils";
import type { MapVertex, VertexType } from "@/lib/types/map";
import type { PlanarPose } from "@/lib/types/robot";

/**
 * The vertex layer's operator surface: place, name, classify, re-aim, delete.
 *
 * It floats on the viewport for the same reason GoalControl does — the gesture
 * that produces a pose happens on the map, and moving the readback into a side
 * rail would put the numbers and the thing they describe on opposite sides of
 * the screen. Top right, because the toolbar owns top left and GridStatus owns
 * bottom left.
 *
 * Presentation only. Every write is the shell's, and every write goes straight
 * to the backend — there is no Save-all here and no dirty chip, unlike the grid
 * this panel sits beside. See hooks/use-map-vertices.ts for why.
 */

const TYPE_OPTIONS = VERTEX_TYPES.map(({ value, label }) => ({ value, label }));

export interface VertexPanelProps {
  vertices: MapVertex[];
  status: "loading" | "ok" | "error";
  /** Load or last-write failure. Rendered verbatim — the backend writes prose. */
  error: string | null;
  busy: boolean;

  /** The type the next placed vertex gets. */
  type: VertexType;
  onTypeChange: (type: VertexType) => void;

  /** A staged, uncreated vertex. Mutually exclusive with `selected` in practice. */
  draft: PlanarPose | null;
  /** The subject of the editing form: the selection, when it holds exactly one. */
  selected: MapVertex | null;
  /**
   * Everything highlighted on the canvas, from the Select tool's band.
   *
   * At one entry this is just `selected.id` and the form is what is shown; at
   * more, the form gives way to the band block, because renaming, retyping and
   * re-aiming have no meaning spread across a set. Delete does, and is the one
   * thing that block offers.
   */
  selectedIds: readonly string[];
  /**
   * A re-aim of `selected` that has not been written yet, from a drag on its own
   * marker. There is no move-to-another-place control here — see the note on
   * `stagedPose` in MapGridEditor for where that lives instead.
   */
  stagedPose: PlanarPose | null;

  /** Where the robot is standing on this map, or null when it cannot be used. */
  robotPose: PlanarPose | null;
  /** Why `robotPose` is null. Shown under the disabled control; see useRobotMapPose. */
  robotPoseReason: string | null;
  /** Stage a draft at `robotPose`. Only called while it is non-null. */
  onUseRobotPose: () => void;

  onSelect: (id: string | null) => void;
  onCancelDraft: () => void;
  /** Drop the band selection without touching anything on the map. */
  onClearSelection: () => void;
  onCreate: (name: string, type: VertexType) => void;
  onSave: (changes: VertexChanges) => void;
  /** Deletes everything in `selectedIds` — one vertex from the form, or a band. */
  onDelete: () => void;
  className?: string;
}

export function VertexPanel(props: VertexPanelProps) {
  const { vertices, status, error, draft, selected, selectedIds, className } = props;
  const band = selectedIds.length > 1;

  return (
    <div className={cn(overlayPanel, "flex w-60 flex-col gap-2 p-2.5", className)}>
      <div className="flex h-4 items-center justify-between gap-2">
        <span className="instrument-label text-muted-foreground">Vertices</span>
        {status === "loading" ? (
          <Chip>Loading</Chip>
        ) : (
          <Chip tone={vertices.length ? "neutral" : "caution"}>{vertices.length}</Chip>
        )}
      </div>

      {error && (
        <p role="alert" className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}

      {draft ? (
        // Keyed so the name field is a fresh mount per draft rather than state
        // cleared in an effect — the same reason EditorSurface keys on the
        // session. Re-dragging the same draft keeps the key, and so keeps the
        // name the operator already typed.
        <VertexForm
          key="draft"
          kind="create"
          pose={draft}
          initialName=""
          initialType={props.type}
          busy={props.busy}
          onTypeChange={props.onTypeChange}
          onSubmit={(name, type) => props.onCreate(name, type)}
          onCancel={props.onCancelDraft}
        />
      ) : band ? (
        <BandBlock
          vertices={vertices}
          selectedIds={selectedIds}
          busy={props.busy}
          onSelect={props.onSelect}
          onClear={props.onClearSelection}
          onDelete={props.onDelete}
        />
      ) : selected ? (
        <>
        <VertexForm
          key={selected.id}
          kind="edit"
          pose={props.stagedPose ?? selected}
          moved={props.stagedPose !== null}
          initialName={selected.name}
          initialType={selected.type}
          busy={props.busy}
          onSubmit={(name, type) => {
            const changes: VertexChanges = {};
            if (name !== selected.name) changes.name = name;
            if (type !== selected.type) changes.type = type;
            if (props.stagedPose) {
              changes.x = props.stagedPose.x;
              changes.y = props.stagedPose.y;
              changes.theta = props.stagedPose.theta;
            }
            props.onSave(changes);
          }}
          onCancel={() => props.onSelect(null)}
          onDelete={props.onDelete}
        />
        {/* Kept below the form so the selection can move without closing it
         * first — the form is keyed on the id, so picking another row remounts
         * it with that vertex's values. */}
        <VertexList
          vertices={vertices}
          selectedIds={selectedIds}
          onSelect={props.onSelect}
          empty={null}
        />
        </>
      ) : (
        <>
          <div>
            <p className="instrument-label mb-1 text-muted-foreground">Place as</p>
            <Segmented
              stretch
              value={props.type}
              options={TYPE_OPTIONS}
              onChange={props.onTypeChange}
            />
          </div>

          {/* The second way to produce a pose, for the stop you mark by driving
            * to it: the operator parks the robot on the spot — a dock, a charger,
            * a doorway they had to squeeze through — and takes the pose off the
            * robot instead of hunting for the cell it is standing on. It stages a
            * draft like a press on the map does, rather than creating the vertex
            * outright, so naming and typing it stay one flow with the placed
            * kind, and a mis-press is a Cancel rather than a row to delete. */}
          <div>
            <button
              type="button"
              disabled={!props.robotPose || props.busy}
              onClick={props.onUseRobotPose}
              className="instrument-label flex h-7 w-full items-center justify-center gap-1.5 rounded-sm border border-hairline text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50 disabled:hover:bg-transparent disabled:hover:text-muted-foreground"
            >
              <LocateFixedIcon className="size-3.5" aria-hidden />
              Use robot position
            </button>
            {!props.robotPose && props.robotPoseReason && (
              <p className="mt-1 text-[11px] leading-snug text-muted-foreground">
                {props.robotPoseReason}
              </p>
            )}
          </div>

          <VertexList
            vertices={vertices}
            selectedIds={selectedIds}
            onSelect={props.onSelect}
            empty={status === "ok" ? "No vertices on this map yet." : null}
          />

          {/* The panel is the only place the tool row's icons are spelled out.
            * Worth the four lines: "why does pressing the map do nothing" is the
            * question the unarmed default buys, and this is where an operator
            * looking at the vertex layer is already looking. */}
          <p className="text-[11px] leading-tight text-muted-foreground">
            Arm <span className="text-foreground">Place</span> to stage a vertex — press
            the map, drag to aim. <span className="text-foreground">Select</span> drags a
            box over several; Shift adds. Escape returns to Pan.
          </p>
        </>
      )}
    </div>
  );
}

/**
 * What a band selection can do, which is delete.
 *
 * Deliberately not a cut-down copy of VertexForm. A name, a type and a heading
 * are each one value, and offering them over a set would mean either a "mixed"
 * state for every field or silently flattening five vertices onto one operator's
 * last keystroke — and the vertex layer writes through with no undo (see
 * hooks/use-map-vertices.ts), so flattening would be permanent. Narrowing to one
 * vertex is a click on any row, and that is where editing lives.
 */
function BandBlock({
  vertices,
  selectedIds,
  busy,
  onSelect,
  onClear,
  onDelete,
}: {
  vertices: MapVertex[];
  selectedIds: readonly string[];
  busy: boolean;
  onSelect: (id: string) => void;
  onClear: () => void;
  onDelete: () => void;
}) {
  const count = selectedIds.length;
  // The band holds ids; the names are what the confirm has to show, and a
  // selection can outlive a row another screen deleted (the list is shared with
  // the dashboard — see lib/api/query-keys.ts), so it is resolved, not assumed.
  const names = vertices
    .filter((vertex) => selectedIds.includes(vertex.id))
    .map((vertex) => vertex.name);

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center justify-between gap-2">
        <span className="instrument-label text-signal-cmd">{count} selected</span>
        <button
          type="button"
          disabled={busy}
          onClick={onClear}
          className="instrument-label rounded-sm border border-hairline px-1.5 py-0.5 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50"
        >
          Clear
        </button>
      </div>

      <VertexList
        vertices={vertices}
        selectedIds={selectedIds}
        onSelect={onSelect}
        empty={null}
      />

      <button
        type="button"
        disabled={busy || count === 0}
        onClick={() => {
          // The same confirm the single delete uses, listing what it is about to
          // take: a count alone is not enough to check a band against, since the
          // band was drawn on the map and the map is behind this panel.
          const listed = names.length > 6 ? `${names.slice(0, 6).join(", ")}, …` : names.join(", ");
          if (window.confirm(`Delete ${count} vertices (${listed})? This cannot be undone.`)) {
            onDelete();
          }
        }}
        className="instrument-label flex h-7 items-center justify-center gap-1.5 rounded-sm border border-signal-warn/50 text-signal-warn transition-colors hover:bg-signal-warn/12 disabled:opacity-50"
      >
        <Trash2Icon className="size-3.5" aria-hidden />
        Delete {count}
      </button>
    </div>
  );
}

function VertexList({
  vertices,
  selectedIds,
  onSelect,
  empty,
}: {
  vertices: MapVertex[];
  /** Every highlighted row, so a band lights all of its members here too. */
  selectedIds: readonly string[];
  /** Picking a row always narrows to that one vertex, band or not. */
  onSelect: (id: string) => void;
  empty: string | null;
}) {
  if (!vertices.length) {
    return empty ? (
      <p className="text-[11px] leading-tight text-muted-foreground">{empty}</p>
    ) : null;
  }

  return (
    // Capped height with its own scroll: the panel floats over the canvas, and a
    // map with thirty stops would otherwise grow it past the viewport.
    <ul className="max-h-48 space-y-px overflow-y-auto border-t border-hairline pt-1.5">
      {vertices.map((vertex) => (
        <li key={vertex.id}>
          <button
            type="button"
            onClick={() => onSelect(vertex.id)}
            className={cn(
              "flex w-full items-center gap-1.5 rounded-sm px-1 py-0.5 text-left transition-colors",
              selectedIds.includes(vertex.id)
                ? "bg-signal-cmd/12 text-signal-cmd"
                : "hover:bg-elevated",
            )}
          >
            <span className="instrument-label w-3 shrink-0 text-muted-foreground">
              {vertexGlyph(vertex.type)}
            </span>
            <span className="readout min-w-0 flex-1 truncate text-[12px]">
              {vertex.name}
            </span>
            <span className="readout shrink-0 text-[11px] text-muted-foreground">
              {vertex.x.toFixed(1)}, {vertex.y.toFixed(1)}
            </span>
          </button>
        </li>
      ))}
    </ul>
  );
}

function VertexForm({
  kind,
  pose,
  moved = false,
  initialName,
  initialType,
  busy,
  onTypeChange,
  onSubmit,
  onCancel,
  onDelete,
}: {
  kind: "create" | "edit";
  pose: PlanarPose;
  /** Edit mode only: the heading shown is staged, not what the row holds. */
  moved?: boolean;
  initialName: string;
  initialType: VertexType;
  busy: boolean;
  /** Create mode only: keep the shell's next-placement type in step. */
  onTypeChange?: (type: VertexType) => void;
  onSubmit: (name: string, type: VertexType) => void;
  onCancel: () => void;
  onDelete?: () => void;
}) {
  const [name, setName] = React.useState(initialName);
  const [type, setType] = React.useState<VertexType>(initialType);

  const trimmed = name.trim();
  // The backend's `min_length=1` would reject a blank name, but as a 422 whose
  // detail is a validation *array* rather than a sentence. Refusing here is what
  // keeps that off the operator's screen.
  const submittable = trimmed.length > 0 && !busy;
  const dirty = kind === "create" || trimmed !== initialName || type !== initialType || moved;

  return (
    <form
      className="flex flex-col gap-2"
      onSubmit={(event) => {
        event.preventDefault();
        if (submittable) onSubmit(trimmed, type);
      }}
    >
      <div>
        <p className="instrument-label mb-1 text-muted-foreground">Name</p>
        <Input
          autoFocus
          value={name}
          onChange={(event) => setName(event.target.value)}
          placeholder="dock-a"
          // Squared off and shortened to match the overlay's chrome; the shared
          // Input is sized for the settings forms, which have room.
          className="h-7 rounded-sm text-[13px]"
        />
      </div>

      <div>
        <p className="instrument-label mb-1 text-muted-foreground">Type</p>
        <Segmented
          stretch
          value={type}
          options={TYPE_OPTIONS}
          onChange={(next) => {
            setType(next);
            onTypeChange?.(next);
          }}
        />
      </div>

      <div className="space-y-1 border-t border-hairline pt-2">
        <Readout label="X" value={pose.x.toFixed(2)} unit="m" tone="cmd" />
        <Readout label="Y" value={pose.y.toFixed(2)} unit="m" tone="cmd" />
        <Readout label="Heading" value={pose.theta.toFixed(1)} unit="°" tone="cmd" />
      </div>

      <div className="flex gap-1.5">
        <button
          type="submit"
          disabled={!submittable || !dirty}
          className="instrument-label h-7 flex-1 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
        >
          {kind === "create" ? "Create" : "Save"}
        </button>
        <button
          type="button"
          disabled={busy}
          onClick={onCancel}
          className="instrument-label h-7 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50"
        >
          {kind === "create" ? "Cancel" : "Close"}
        </button>
        {kind === "edit" && (
          <button
            type="button"
            disabled={busy}
            aria-label="Delete vertex"
            title="Delete vertex"
            onClick={() => {
              // A confirm rather than an undo: this panel writes through, so
              // there is no local history to step back over, and the page's back
              // button already asks the same way.
              if (window.confirm(`Delete "${initialName}"? This cannot be undone.`)) {
                onDelete?.();
              }
            }}
            className="instrument-label flex h-7 items-center rounded-sm border border-signal-warn/50 px-2 text-signal-warn transition-colors hover:bg-signal-warn/12 disabled:opacity-50"
          >
            <Trash2Icon className="size-3.5" aria-hidden />
          </button>
        )}
      </div>
    </form>
  );
}
