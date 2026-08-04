"use client";

import * as React from "react";
import { CrosshairIcon, Trash2Icon } from "lucide-react";

import { Chip, Readout, Segmented, overlayPanel } from "@/components/console/instrument";
import { Input } from "@/components/ui/input";
import type { VertexChanges } from "@/lib/api/vertex";
import { VERTEX_TYPES, vertexGlyph } from "@/lib/map/vertex";
import { cn } from "@/lib/utils";
import type { MapVertex, VertexType } from "@/lib/types/map";
import type { PlanarPose } from "@/lib/types/robot";

/**
 * The vertex layer's operator surface: place, name, classify, move, delete.
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
  selected: MapVertex | null;
  /** A re-pose of `selected` that has not been written yet. */
  stagedPose: PlanarPose | null;
  /** True while the next press on the map re-places `selected`. */
  placing: boolean;

  onSelect: (id: string | null) => void;
  onArmPlace: () => void;
  onCancelDraft: () => void;
  onCreate: (name: string, type: VertexType) => void;
  onSave: (changes: VertexChanges) => void;
  onDelete: () => void;
  className?: string;
}

export function VertexPanel(props: VertexPanelProps) {
  const { vertices, status, error, draft, selected, className } = props;

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
          placing={false}
          onTypeChange={props.onTypeChange}
          onSubmit={(name, type) => props.onCreate(name, type)}
          onCancel={props.onCancelDraft}
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
          placing={props.placing}
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
          onArmPlace={props.onArmPlace}
          onDelete={props.onDelete}
        />
        {/* Kept below the form so the selection can move without closing it
         * first — the form is keyed on the id, so picking another row remounts
         * it with that vertex's values. */}
        <VertexList
          vertices={vertices}
          selectedId={selected.id}
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

          <VertexList
            vertices={vertices}
            selectedId={null}
            onSelect={props.onSelect}
            empty={status === "ok" ? "No vertices on this map yet." : null}
          />

          <p className="text-[11px] leading-tight text-muted-foreground">
            In Vertex mode, press the map and drag to aim.
          </p>
        </>
      )}
    </div>
  );
}

function VertexList({
  vertices,
  selectedId,
  onSelect,
  empty,
}: {
  vertices: MapVertex[];
  selectedId: string | null;
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
              vertex.id === selectedId
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
  placing,
  onTypeChange,
  onSubmit,
  onCancel,
  onArmPlace,
  onDelete,
}: {
  kind: "create" | "edit";
  pose: PlanarPose;
  /** Edit mode only: the pose shown is staged, not what the row holds. */
  moved?: boolean;
  initialName: string;
  initialType: VertexType;
  busy: boolean;
  placing: boolean;
  /** Create mode only: keep the shell's next-placement type in step. */
  onTypeChange?: (type: VertexType) => void;
  onSubmit: (name: string, type: VertexType) => void;
  onCancel: () => void;
  onArmPlace?: () => void;
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

      {kind === "edit" && (
        <button
          type="button"
          disabled={busy}
          onClick={onArmPlace}
          className={cn(
            "instrument-label flex h-7 items-center justify-center gap-1.5 rounded-sm border transition-colors disabled:opacity-50",
            placing
              ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
              : "border-hairline text-muted-foreground hover:bg-elevated hover:text-foreground",
          )}
        >
          <CrosshairIcon className="size-3.5" aria-hidden />
          {placing ? "Press the map" : "Move"}
        </button>
      )}

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
