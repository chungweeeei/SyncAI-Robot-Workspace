"use client";

import { PlusIcon } from "lucide-react";

import { StepRow } from "@/components/tasks/step-row";
import type { ActiveVerticesStatus } from "@/hooks/use-active-map-vertices";
import type { StepType, TaskStepState } from "@/lib/api/task";
import { STEP_TYPES, stepIdFor, type StepDraft } from "@/lib/task/step";
import type { MapVertex } from "@/lib/types/map";

export interface StepListProps {
  steps: StepDraft[];
  vertices: MapVertex[];
  verticesStatus: ActiveVerticesStatus;
  mapName: string | null;
  /** True while a dispatched task is running — see the note below. */
  disabled: boolean;
  /** Per-step state of the tracked task, keyed by the derived step id. */
  stepStates: ReadonlyMap<string, TaskStepState>;
  onAdd: (type: StepType) => void;
  onPatch: (key: number, changes: Partial<Omit<StepDraft, "key">>) => void;
  onRemove: (key: number) => void;
  onMove: (key: number, delta: -1 | 1) => void;
}

/**
 * The ordered step list plus its add row. Presentation only.
 *
 * `disabled` is the whole list at once, not per control, and it is driven by a
 * task being in flight. The reason is that the backend step ids are positional
 * (`stepIdFor`): reorder or delete a row while a task is running and the tracked
 * statuses would silently attach to the wrong rows. Freezing the list is the
 * cheap fix. The rejected alternative — rendering a second, frozen, read-only
 * copy of the submitted steps below the editable one — is more code and puts each
 * status further from the row it describes.
 */
export function StepList({
  steps,
  vertices,
  verticesStatus,
  mapName,
  disabled,
  stepStates,
  onAdd,
  onPatch,
  onRemove,
  onMove,
}: StepListProps) {
  return (
    <div className="space-y-2">
      {steps.length === 0 ? (
        <p className="text-[11px] leading-tight text-muted-foreground">
          No steps yet. A task is the list below, run top to bottom.
        </p>
      ) : (
        <ul className="space-y-1.5">
          {steps.map((step, index) => (
            <StepRow
              // The client key, not the derived step id: that id is positional, so
              // reconciling on it would move a focused input's DOM mid-reorder.
              key={step.key}
              step={step}
              index={index}
              total={steps.length}
              vertices={vertices}
              verticesStatus={verticesStatus}
              mapName={mapName}
              disabled={disabled}
              state={stepStates.get(stepIdFor(index, step.type)) ?? null}
              onPatch={(changes) => onPatch(step.key, changes)}
              onRemove={() => onRemove(step.key)}
              onMove={(delta) => onMove(step.key, delta)}
            />
          ))}
        </ul>
      )}

      <div className="flex flex-wrap items-center gap-1.5 border-t border-hairline pt-2">
        <span className="instrument-label mr-0.5 text-muted-foreground">Add</span>
        {STEP_TYPES.map((spec) => (
          <button
            key={spec.value}
            type="button"
            disabled={disabled}
            title={spec.hint}
            onClick={() => onAdd(spec.value)}
            className="instrument-label flex h-7 items-center gap-1 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-40 disabled:hover:bg-transparent"
          >
            <PlusIcon className="size-3" aria-hidden />
            {spec.label}
          </button>
        ))}
      </div>
    </div>
  );
}
