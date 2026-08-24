"use client";

import * as React from "react";
import { SaveIcon } from "lucide-react";

import { Input } from "@/components/ui/input";
import { TASK_TEMPLATE_NAME_MAX } from "@/lib/api/task-template";

export interface SaveGroupProps {
  /** The template currently loaded in the editor, or null when authoring fresh. */
  editing: { id: string; name: string } | null;
  /** False when the step list is not savable (see the composer's own gate). */
  ready: boolean;
  /** Why not, as a muted line. Null when ready. */
  reason: string | null;
  busy: boolean;
  error: string | null;
  /** Names already in the library, for the duplicate hint. */
  existingNames: readonly string[];
  onCreate: (name: string) => void;
  onUpdate: (id: string, name: string) => void;
}

/**
 * Name the authored list and store it.
 *
 * **Two explicit buttons rather than dirty tracking.** Deciding whether to grey
 * out "Update" would mean deep-comparing the current drafts against the snapshot
 * they were loaded from — and the coordinates are *strings*, so "6.834" and
 * "6.8340" are equal numbers and different drafts. That comparison would exist
 * only to disable a button. Two buttons that each say what they do are more
 * honest, and they make "load A, tweak it, save it as B" a first-class action
 * instead of something you have to know is possible.
 *
 * Reset is by remounting: TaskConsole keys this component, so loading a different
 * template arrives as a fresh mount with that template's name in the field, rather
 * than as state cleared in an effect. Same trick as the vertex panel's form.
 */
export function SaveGroup({
  editing,
  ready,
  reason,
  busy,
  error,
  existingNames,
  onCreate,
  onUpdate,
}: SaveGroupProps) {
  const [name, setName] = React.useState(editing?.name ?? "");

  const trimmed = name.trim();
  // The backend strips and rejects a blank name, but as a 422 whose detail is a
  // validation *array* rather than a sentence. Refusing here keeps that off the
  // operator's screen — the same reason the vertex panel checks its own name.
  const nameOk = trimmed.length > 0 && trimmed.length <= TASK_TEMPLATE_NAME_MAX;

  // A warning, not a gate: duplicate names are allowed by design (the id is the
  // identity, exactly as for map vertices), and there is no migration path to add
  // a unique constraint later even if that changed. Saying so beats refusing.
  const duplicate =
    nameOk &&
    trimmed !== editing?.name &&
    existingNames.some((existing) => existing.toLowerCase() === trimmed.toLowerCase());

  const localReason = !nameOk && trimmed.length > 0 ? "That name is too long." : null;
  const canSave = ready && nameOk && !busy;

  return (
    <form
      className="space-y-2"
      onSubmit={(event) => {
        event.preventDefault();
        if (!canSave) return;
        // Enter submits the primary action, which is Update when a template is
        // loaded and Create otherwise.
        if (editing) onUpdate(editing.id, trimmed);
        else onCreate(trimmed);
      }}
    >
      <label className="block">
        <span className="instrument-label text-muted-foreground">Name</span>
        <Input
          value={name}
          disabled={busy}
          maxLength={TASK_TEMPLATE_NAME_MAX}
          onChange={(event) => setName(event.target.value)}
          placeholder="Morning patrol"
          className="readout mt-0.5 h-7 rounded-sm text-[13px]"
        />
      </label>

      {duplicate && (
        <p className="text-[11px] leading-tight text-signal-caution">
          Another template already has this name. Saving is still allowed — the
          two are told apart by id, not by name.
        </p>
      )}

      {error && (
        <p
          role="alert"
          className="text-[11px] leading-snug break-words text-signal-warn"
        >
          {error}
        </p>
      )}

      <div className="flex gap-1.5">
        {editing && (
          <button
            type="submit"
            disabled={!canSave}
            className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
          >
            <SaveIcon className="size-3.5" aria-hidden />
            <span className="min-w-0 truncate">Update “{editing.name}”</span>
          </button>
        )}
        <button
          type={editing ? "button" : "submit"}
          disabled={!canSave}
          onClick={editing ? () => onCreate(trimmed) : undefined}
          className={
            editing
              ? "instrument-label h-7 shrink-0 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-40"
              : "instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
          }
        >
          {editing ? "Save as new" : (
            <>
              <SaveIcon className="size-3.5" aria-hidden />
              Save as new
            </>
          )}
        </button>
      </div>

      {(localReason ?? reason) && (
        <p className="text-[11px] leading-tight text-muted-foreground">
          {localReason ?? reason}
        </p>
      )}
    </form>
  );
}
