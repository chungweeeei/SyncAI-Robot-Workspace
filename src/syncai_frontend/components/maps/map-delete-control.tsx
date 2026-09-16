"use client";

import * as React from "react";
import { useQueryClient } from "@tanstack/react-query";
import { ArrowLeftIcon, Trash2Icon, XIcon } from "lucide-react";

import {
  AlertDialog,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { Button } from "@/components/ui/button";
import { deleteMap } from "@/lib/api/map";
import { queryKeys } from "@/lib/api/query-keys";
import { cn } from "@/lib/utils";
import type { MapSummary } from "@/lib/types/map";

const LOCKED_REASON =
  "The map in use cannot be deleted: the running stack loaded this name. Switch the robot to another map first.";
const CONVERTING_REASON =
  "Wait for the gridmap conversion to finish before deleting.";

/**
 * The corner itself: geometry and surface, shared by the live button and the
 * greyed span so the two occupy exactly the same place. Its own `bg-panel`
 * because what is underneath is a gridmap image or `bg-elevated` depending on
 * the map, and a bare glyph would sink into one of them.
 */
const CORNER =
  "absolute top-2 right-2 z-10 flex size-6 items-center justify-center rounded-sm border border-hairline bg-panel/90 backdrop-blur-sm";

/**
 * The one consequence the title does not already state: the waypoints go too.
 *
 * Size used to lead this sentence and deliberately no longer does. An operator
 * deleting a map is not weighing disk space — the card already shows the size
 * for anyone who is — and putting megabytes first pushed the thing they cannot
 * get back to the end of the line. Vertices are hand-placed work that lives in
 * Postgres, not in the directory, so their going is the part that can surprise.
 *
 * Null for a map with none: there is no second consequence to warn about, and a
 * sentence saying "0 vertices" is a sentence the operator reads for nothing.
 */
function vertexWarning(map: MapSummary): string | null {
  if (map.vertex_count === 0) return null;
  if (map.vertex_count === 1) return "Its saved vertex goes too.";
  return `Its ${map.vertex_count} saved vertices go too.`;
}

/**
 * The card-side face of DELETE /api/v1/maps/{name}.
 *
 * An X pinned to the card's top-right corner, not a word in the header row.
 * Dismiss-this-thing is the one gesture that already has a universal glyph, and
 * the header row is a row of *labels* — Rename, Details — where a fourth word
 * would read as another view to open rather than the one action that destroys
 * something. The corner is also the only place on the card that is not already
 * carrying information.
 *
 * It overlays the thumbnail, which has two different backdrops (a fixed light
 * neutral under a real preview, `bg-elevated` under "No preview"), so the
 * trigger carries its own panel surface and hairline rather than trusting
 * either one for contrast. **The card is what gives it a positioned ancestor**
 * — this component owns the `absolute`, since its dialog is a sibling and a
 * wrapper element would have to be positioned instead.
 *
 * Behind an AlertDialog because this is the one map action with nothing behind
 * it — a rebuild keeps `gridmap_prev.pgm`, a rename can be renamed back, and
 * this is an `rmtree` of a mapping run plus its waypoints. The alert dialog
 * rather than a plain one for the reason that component's header gives: no
 * dismiss-by-backdrop, so the only ways out are the two buttons. An icon with
 * no label leans harder on that dialog: the title names the map, which is what
 * confirms the operator hit the corner they meant to.
 *
 * The dialog names the map and warns about the vertices (see `vertexWarning`),
 * and says nothing else. Every sentence it could add is one the operator reads
 * *instead of* the two facts that matter.
 *
 * **The map in use cannot be deleted, and that is the backend's rule.** The
 * running stack holds `map/<name>/…` paths it opened at launch, so where a
 * rename would leave map_server and the localizer a stale path, a delete leaves
 * them nothing. Switching the robot to another map lifts this the way it lifts
 * the rename — and note it lifts it properly, by loading the *other* map's
 * files, so the map being deleted is genuinely no longer open. The control is a
 * greyed look-alike on the active card with the reason in its tooltip, and the
 * endpoint answers 409 `map_active` regardless. Greyed mid-conversion too: the
 * conversion thread is writing into the directory.
 *
 * A refusal keeps the dialog open and shows the backend's sentence there. That
 * matters most for 409 `template_bound`, the only one that asks the operator to
 * go and do something — it names the task templates still bound to this map,
 * and closing the dialog would take the list away with it.
 *
 * On success this instance goes away: MapLibrary keys cards by name, so the
 * refetch unmounts the card. The backend's sentence therefore goes *up*, through
 * `onDeleted`, to a line the library keeps. Mutation-as-async-callback with
 * local busy/error, same as MapRenameControl — this codebase does not use
 * useMutation.
 */
export function MapDeleteControl({
  map,
  onDeleted,
}: {
  map: MapSummary;
  /** The backend's sentence, for a surface that outlives this card. */
  onDeleted?: (message: string) => void;
}) {
  const queryClient = useQueryClient();
  const [confirming, setConfirming] = React.useState(false);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const locked = map.active;
  const blocked = locked || map.grid_status === "converting";

  const submit = React.useCallback(async () => {
    if (busy) return;
    setBusy(true);
    setError(null);
    try {
      const result = await deleteMap(map.name);
      // Nothing will read this map's vertices again; drop the entry rather
      // than let it sit until eviction.
      queryClient.removeQueries({ queryKey: queryKeys.mapVertices(map.name) });
      onDeleted?.(result.message);
      setConfirming(false);
      // The catalogue no longer lists the map; the refetch is what unmounts
      // this card.
      void queryClient.invalidateQueries({ queryKey: queryKeys.maps });
    } catch (cause) {
      // 409 (in use, converting, a task template still bound) arrives as the
      // backend's own sentence, written to be shown. The dialog stays open.
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, [busy, map.name, onDeleted, queryClient]);

  return (
    <>
      {/* A look-alike span rather than a disabled button when the map is in
        * use: `disabled` would swallow the pointer events the tooltip needs,
        * and the reason is the whole point of showing the control at all. */}
      {blocked ? (
        <span
          aria-disabled="true"
          title={locked ? LOCKED_REASON : CONVERTING_REASON}
          className={cn(CORNER, "cursor-not-allowed text-muted-foreground opacity-40")}
        >
          <XIcon className="size-3.5" aria-hidden />
        </span>
      ) : (
        <button
          type="button"
          onClick={() => {
            setError(null);
            setConfirming(true);
          }}
          aria-label={`Delete ${map.name}`}
          title={`Delete ${map.name}`}
          className={cn(
            CORNER,
            "text-muted-foreground transition-colors hover:border-signal-warn/50 hover:bg-signal-warn/12 hover:text-signal-warn",
          )}
        >
          <XIcon className="size-3.5" aria-hidden />
        </button>
      )}

      <AlertDialog
        open={confirming}
        onOpenChange={(open) => {
          if (!open && !busy) {
            setConfirming(false);
            setError(null);
          }
        }}
      >
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Delete {map.name}?</AlertDialogTitle>
            <AlertDialogDescription>
              {vertexWarning(map)} Remapping the site is the only way back.
            </AlertDialogDescription>
          </AlertDialogHeader>

          {error && (
            <p
              role="alert"
              className="text-[11px] leading-tight text-signal-warn"
            >
              {error}
            </p>
          )}

          <AlertDialogFooter>
            {/* Icon *and* label, the house pattern (see vertex-move-dialog):
              * the glyph is what tells the two apart at a glance, the word is
              * what makes the dangerous one unmistakable. This is the last
              * screen before an rmtree — not the place to make an operator
              * decode two unlabelled buttons. */}
            <Button
              variant="ghost"
              size="sm"
              disabled={busy}
              onClick={() => {
                setConfirming(false);
                setError(null);
              }}
            >
              <ArrowLeftIcon data-icon="inline-start" />
              Keep
            </Button>
            <Button
              variant="destructive"
              size="sm"
              disabled={busy}
              onClick={() => void submit()}
            >
              <Trash2Icon data-icon="inline-start" />
              {busy ? "Deleting…" : "Delete"}
            </Button>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}
