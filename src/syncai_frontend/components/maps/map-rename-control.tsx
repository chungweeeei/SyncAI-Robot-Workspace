"use client";

import * as React from "react";
import { useQueryClient } from "@tanstack/react-query";

import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { renameMap } from "@/lib/api/map";
import { MAP_NAME_RE } from "@/lib/api/mapping";
import { queryKeys } from "@/lib/api/query-keys";
import type { MapSummary } from "@/lib/types/map";

const LOCKED_REASON =
  "The map in use cannot be renamed: the running stack loaded this name. Switch the robot to another map first.";
const CONVERTING_REASON =
  "Wait for the gridmap conversion to finish before renaming.";

/**
 * The card's title, and the way to change it — the card-side face of
 * PATCH /api/v1/maps/{name}.
 *
 * The name is edited in place: press Rename and the title becomes the input,
 * same face and size, with Save and Cancel beside it. Not a dropdown (the card
 * has no action menu, and a one-item menu is chrome for its own sake) and not a
 * modal (every name edit in this console is an inline form — SaveMapControl,
 * the vertex panel). Enter saves, Escape cancels.
 *
 * **The map in use cannot be renamed here, and that is the backend's rule, not
 * a UI nicety.** map_server and the localizer opened `map/<name>/…` at launch,
 * so renaming the directory under them would leave both holding a path that no
 * longer exists. The way out is no longer a stack restart — the card's Switch
 * control moves the robot to another map, and the old name is renameable the
 * moment it does — but a rename is not entitled to do that swap itself, which
 * is why the refusal stands rather than becoming a prompt. So the control is a
 * greyed look-alike on the active card, with the reason in its tooltip, and the
 * endpoint answers 409 `map_active` regardless. Also greyed mid-conversion, for
 * the same reason Edit and Rebuild are: the conversion thread holds the old
 * directory path.
 *
 * On success this instance goes away: MapLibrary keys cards by name, so the
 * refetch mounts a fresh card under the new name. The backend's sentence (how
 * many vertices and templates moved) therefore goes *up*, through `onRenamed`,
 * to a line the library keeps. Mutation-as-async-callback with local busy/error,
 * same as GridRebuildControl — this codebase does not use useMutation.
 */
export function MapRenameControl({
  map,
  onRenamed,
}: {
  map: MapSummary;
  /** The backend's sentence, for a surface that outlives this card. */
  onRenamed?: (message: string) => void;
}) {
  const queryClient = useQueryClient();
  const [editing, setEditing] = React.useState(false);
  const [name, setName] = React.useState(map.name);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const trimmed = name.trim();
  const valid = MAP_NAME_RE.test(trimmed);
  const changed = trimmed !== map.name;
  const locked = map.active;

  const begin = () => {
    setName(map.name);
    setError(null);
    setEditing(true);
  };
  const cancel = () => {
    setEditing(false);
    setError(null);
  };

  const submit = React.useCallback(async () => {
    if (!valid || !changed || busy) return;
    setBusy(true);
    setError(null);
    try {
      const result = await renameMap(map.name, trimmed);
      // The per-map vertex cache is keyed by the old name and nothing will
      // read it again; drop it rather than let it sit until eviction.
      queryClient.removeQueries({ queryKey: queryKeys.mapVertices(map.name) });
      onRenamed?.(result.message);
      setEditing(false);
      // The catalogue now lists the map under its new name; the refetch is
      // what replaces this card with one keyed on that name.
      void queryClient.invalidateQueries({ queryKey: queryKeys.maps });
    } catch (cause) {
      // 409 (taken, converting, in use) and 400 arrive as the backend's own
      // sentence, written to be shown.
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, [busy, changed, map.name, onRenamed, queryClient, trimmed, valid]);

  if (editing) {
    return (
      <div className="min-w-0 flex-1">
        <form
          // A form so Enter in the input saves, matching what a name-and-confirm
          // row is expected to do.
          onSubmit={(event) => {
            event.preventDefault();
            void submit();
          }}
          className="flex items-center gap-1.5"
        >
          <Input
            autoFocus
            value={name}
            onChange={(event) => setName(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === "Escape") {
                event.preventDefault();
                cancel();
              }
            }}
            aria-label="New map name"
            aria-invalid={trimmed.length > 0 && !valid ? true : undefined}
            disabled={busy}
            className="readout h-7 min-w-0 flex-1 rounded-sm px-1.5 text-[15px] font-medium md:text-[15px]"
          />
          <Button
            type="submit"
            size="xs"
            disabled={!valid || !changed || busy}
          >
            {busy ? "Saving…" : "Save"}
          </Button>
          <Button
            type="button"
            size="xs"
            variant="ghost"
            disabled={busy}
            onClick={cancel}
          >
            Cancel
          </Button>
        </form>

        {/* The rule, shown only while the name breaks it — same line as the
          * save-map row, because it is the same rule. */}
        {trimmed.length > 0 && !valid && (
          <p className="mt-1 text-[11px] leading-snug text-signal-caution">
            Letters, digits, dot, dash and underscore only, up to 64 characters.
          </p>
        )}
        {error && (
          <p role="alert" className="mt-1 text-[11px] leading-snug text-signal-warn">
            {error}
          </p>
        )}
      </div>
    );
  }

  return (
    <>
      <h2 className="readout min-w-0 flex-1 truncate text-[15px] font-medium">
        {map.name}
      </h2>
      {/* A look-alike span rather than a disabled button when the map is in
        * use: `disabled` would swallow the pointer events the tooltip needs,
        * and the reason is the whole point of showing the control at all. */}
      {locked || map.grid_status === "converting" ? (
        <span
          aria-disabled="true"
          title={locked ? LOCKED_REASON : CONVERTING_REASON}
          className="instrument-label flex h-5 shrink-0 cursor-not-allowed items-center rounded-sm px-1 text-muted-foreground opacity-40"
        >
          Rename
        </span>
      ) : (
        <button
          type="button"
          onClick={begin}
          aria-label={`Rename ${map.name}`}
          className="instrument-label flex h-5 shrink-0 items-center rounded-sm px-1 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
        >
          Rename
        </button>
      )}
    </>
  );
}
