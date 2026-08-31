"use client";

import * as React from "react";
import { ChevronDownIcon, ChevronUpIcon, Trash2Icon } from "lucide-react";

import { Chip, Segmented } from "@/components/console/instrument";
import { IconButton } from "@/components/tasks/icon-button";
import { TaskStatusChip } from "@/components/console/task-chip";
import { VertexPicker } from "@/components/tasks/vertex-picker";
import { Input } from "@/components/ui/input";
import type { ActiveVerticesStatus } from "@/hooks/use-active-map-vertices";
import { normalizeTheta, type TaskStepState } from "@/lib/api/task";
import {
  SPEAK_TEXT_MAX,
  STEP_TYPES,
  formatDraftAngle,
  formatDraftPosition,
  stepDraftError,
  stepGlyph,
  type StepDraft,
} from "@/lib/task/step";
import type { MapVertex } from "@/lib/types/map";

const TYPE_OPTIONS = STEP_TYPES.map(({ value, label }) => ({ value, label }));

export interface StepRowProps {
  step: StepDraft;
  /** 0-based. Rendered as index + 1, and what the backend step id is built from. */
  index: number;
  total: number;
  vertices: MapVertex[];
  verticesStatus: ActiveVerticesStatus;
  mapName: string | null;
  disabled: boolean;
  /** Tracked status + error_msg for this step, or null when nothing is tracked. */
  state: TaskStepState | null;
  onPatch: (changes: Partial<Omit<StepDraft, "key">>) => void;
  onRemove: () => void;
  onMove: (delta: -1 | 1) => void;
}

export function StepRow({
  step,
  index,
  total,
  vertices,
  verticesStatus,
  mapName,
  disabled,
  state,
  onPatch,
  onRemove,
  onMove,
}: StepRowProps) {
  const rowError = stepDraftError(step);

  return (
    <li className="rounded-sm border border-hairline bg-elevated/40 px-2 py-2">
      {/* flex-wrap because the rail becomes a bottom bar on a narrow console and
       * this row carries an ordinal, a three-segment picker, a status chip and
       * three icon buttons — enough to overflow a phone-width panel otherwise. */}
      <div className="flex flex-wrap items-center gap-2">
        <span className="readout w-4 shrink-0 text-[12px] text-muted-foreground">
          {index + 1}
        </span>
        <span className="instrument-label w-3 shrink-0 text-muted-foreground">
          {stepGlyph(step.type)}
        </span>

        <Segmented
          value={step.type}
          options={TYPE_OPTIONS}
          disabled={disabled}
          onChange={(type) => onPatch({ type })}
        />

        {/* Loaded from a template whose vertex has since been deleted, so these
         * coordinates are the snapshot rather than a live pose. Not an error — the
         * row dispatches fine — but the operator should know the map no longer
         * agrees, and re-picking a vertex clears it. */}
        {step.vertexMissing && step.vertexId !== null && (
          <Chip tone="caution">vertex deleted</Chip>
        )}

        {state && <TaskStatusChip status={state.status} />}

        <div className="ml-auto flex shrink-0 items-center gap-0.5">
          <IconButton
            label="Move step up"
            disabled={disabled || index === 0}
            onClick={() => onMove(-1)}
          >
            <ChevronUpIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label="Move step down"
            disabled={disabled || index === total - 1}
            onClick={() => onMove(1)}
          >
            <ChevronDownIcon className="size-3.5" aria-hidden />
          </IconButton>
          <IconButton
            label="Remove step"
            disabled={disabled}
            onClick={onRemove}
            className="text-signal-warn hover:bg-signal-warn/12"
          >
            <Trash2Icon className="size-3.5" aria-hidden />
          </IconButton>
        </div>
      </div>

      {/* Coordinates and the spoken line are kept in the draft across a type
       * change, so switching to STANDUP and back does not lose what was typed —
       * the wire shape is derived from the type, not stored alongside it. */}
      {step.type === "MOVE" && (
        <div className="mt-2 space-y-1.5 pl-[26px]">
          <VertexPicker
            vertices={vertices}
            status={verticesStatus}
            mapName={mapName}
            value={step.vertexId}
            disabled={disabled}
            onPick={(vertex) =>
              // One patch, not four: two updates would render a frame whose
              // numbers are this vertex's but whose label is still the old one.
              // normalizeTheta on the way *in* because the vertex table has no
              // range constraint while MoveParams is (-180, 180] — a row written
              // by curl can hold exactly -180, which the task endpoint rejects.
              onPatch({
                x: formatDraftPosition(vertex.x),
                y: formatDraftPosition(vertex.y),
                theta: formatDraftAngle(normalizeTheta(vertex.theta)),
                vertexId: vertex.id,
                // Re-picking resolves the provenance, so the stale-snapshot
                // warning goes with it.
                vertexMissing: false,
              })
            }
          />

          <div className="grid grid-cols-3 gap-1.5">
            <CoordinateField
              label="X"
              unit="m"
              value={step.x}
              disabled={disabled}
              onChange={(x) => onPatch({ x, vertexId: null, vertexMissing: false })}
            />
            <CoordinateField
              label="Y"
              unit="m"
              value={step.y}
              disabled={disabled}
              onChange={(y) => onPatch({ y, vertexId: null, vertexMissing: false })}
            />
            <CoordinateField
              label="Heading"
              unit="°"
              value={step.theta}
              disabled={disabled}
              onChange={(theta) => onPatch({ theta, vertexId: null, vertexMissing: false })}
              hint={foldHint(step.theta)}
            />
          </div>
        </div>
      )}

      {step.type === "SPEAK" && (
        <div className="mt-2 pl-[26px]">
          <label className="block">
            <span className="instrument-label text-muted-foreground">Say</span>
            <Input
              value={step.text}
              disabled={disabled}
              // No maxLength attribute: it would silently truncate a paste, and
              // a hidden edit is worse than the row error below saying how far
              // over the limit the text is. Same stance as the coordinate
              // fields never rewriting under the cursor.
              onChange={(event) => onPatch({ text: event.target.value })}
              placeholder="Delivery arrived — please take your items."
              className="mt-0.5 h-7 rounded-sm text-[13px]"
            />
            {/* The counter appears only near the limit — a line short enough
             * to obviously fit does not need bookkeeping over it. */}
            {step.text.length > SPEAK_TEXT_MAX - 100 && (
              <span
                className={
                  step.text.trim().length > SPEAK_TEXT_MAX
                    ? "readout mt-0.5 block text-[11px] text-signal-warn"
                    : "readout mt-0.5 block text-[11px] text-signal-caution"
                }
              >
                {step.text.length} / {SPEAK_TEXT_MAX}
              </span>
            )}
          </label>
        </div>
      )}

      {rowError && (
        <p
          role="alert"
          className="mt-1.5 pl-[26px] text-[11px] leading-snug break-words text-signal-warn"
        >
          {rowError}
        </p>
      )}

      {state?.error_msg && (
        <p
          role="alert"
          className="mt-1.5 pl-[26px] text-[11px] leading-snug break-words text-signal-warn"
        >
          {state.error_msg}
        </p>
      )}
    </li>
  );
}

/**
 * What the heading will actually be sent as, shown only when the fold changes it.
 *
 * The field itself is never rewritten under the cursor — that would make typing
 * "180" one character at a time impossible, since "1" and "18" are both valid
 * angles the fold would leave alone but "1800" is not. Showing the result instead
 * means the fold is not a surprise discovered after dispatch.
 */
function foldHint(raw: string): string | null {
  const trimmed = raw.trim();
  if (!trimmed) return null;
  const value = Number(trimmed);
  if (!Number.isFinite(value)) return null;
  const folded = normalizeTheta(value);
  return folded === value ? null : `→ ${folded.toFixed(1)}°`;
}

function CoordinateField({
  label,
  unit,
  value,
  disabled,
  onChange,
  hint,
}: {
  label: string;
  unit: string;
  value: string;
  disabled: boolean;
  onChange: (value: string) => void;
  hint?: string | null;
}) {
  return (
    <label className="block">
      <span className="instrument-label text-muted-foreground">
        {label} <span className="font-normal">({unit})</span>
      </span>
      <Input
        // Not type="number": the spinners are useless at this size, and a browser
        // that clears the value on an intermediate string ("1e") would fight the
        // text-in-parse-once model the draft is built on. inputMode gets the
        // numeric keypad on a touch console without any of that.
        inputMode="decimal"
        value={value}
        disabled={disabled}
        onChange={(event) => onChange(event.target.value)}
        placeholder="0.00"
        // Squared off and shortened to match the console's chrome; the shared
        // Input is sized for the settings forms, which have room.
        className="readout mt-0.5 h-7 rounded-sm text-[13px]"
      />
      {hint && (
        <span className="readout mt-0.5 block text-[11px] text-signal-caution">
          {hint}
        </span>
      )}
    </label>
  );
}
