"use client";

import * as React from "react";
import { useQueryClient } from "@tanstack/react-query";
import { ArrowLeftIcon, ArrowLeftRightIcon } from "lucide-react";

import {
  AlertDialog,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { Button } from "@/components/ui/button";
import { activateMap } from "@/lib/api/map";
import { queryKeys } from "@/lib/api/query-keys";
import { cn } from "@/lib/utils";
import type { MapSummary } from "@/lib/types/map";

const NO_GRID_REASON =
  "The robot cannot run on a map with no gridmap. Build one first — the planner and both costmaps read it, and map_server refuses to load anything else.";
const NO_CLOUD_REASON =
  "The robot cannot localize on a map with no map.pcd. Only a map saved from a mapping run can be switched to.";
const CONVERTING_REASON =
  "Wait for the gridmap conversion to finish before switching to this map.";

/**
 * The corner itself — MapDeleteControl's geometry and surface, on the opposite
 * side. Shared by the live button and the greyed span so the two occupy exactly
 * the same place, and carrying its own `bg-panel` for the same reason: what is
 * underneath is a gridmap image or `bg-elevated` depending on the map, and a
 * bare glyph would sink into one of them.
 *
 * `top-2 left-2` is deliberately the slot MapCard's in-use badge occupies. The
 * two are mutually exclusive — the badge renders only when `map.active`, this
 * only when it is not — so one 24px tile sits in that corner on every card,
 * always about the same question.
 */
const CORNER =
  "absolute top-2 left-2 z-10 flex size-6 items-center justify-center rounded-sm border border-hairline bg-panel/90 backdrop-blur-sm";

/**
 * The card-side face of POST /api/v1/maps/{name}/activate.
 *
 * A corner tile in the card's top-left, which is **the same slot MapCard's
 * in-use badge occupies** — and that is the design, not a collision. The two are
 * mutually exclusive, so exactly one 24px tile sits in that corner on every
 * card, always answering the same question: a solid check where the answer is
 * "this one", swap arrows where it is "press to make it this one".
 *
 * Two glyphs rather than one, and deliberately. The same check outlined instead
 * of solid would be a radio button, which is a fair description of the grid —
 * exactly one map is loaded — but an unfilled check reads at least as easily as
 * *already done, greyed out*, and this control already has a real greyed state
 * it must not be confused with (see `reason` below). Swap arrows cannot be read
 * as a state at all: they only ever mean "change this", which is the one thing
 * pressing it does. `ArrowLeftRight` specifically, keeping clear of
 * `RefreshCw` on the Rebuild-grid control two rows down.
 *
 * It also completes the card's corner language: left is the in-use question
 * (state or action), right is MapDeleteControl's X. Two tiles, same geometry,
 * same surface, opposite corners — which is why this borrows that component's
 * `CORNER` shape verbatim rather than inventing one.
 *
 * **Never rendered on the active card**, which is what frees the slot for the
 * badge. The endpoint answers 200 `switched: false` there anyway, so this is not
 * a guard. Greyed, with the reason in the tooltip, for the three maps that
 * cannot be switched to at all: no gridmap, no point cloud, mid-conversion. A
 * look-alike span rather than a `disabled` button for MapDeleteControl's reason
 * — `disabled` swallows the pointer events the tooltip needs.
 *
 * The icon carries no label, so the dialog does the talking: its title names the
 * map, which is what confirms the operator hit the corner they meant to.
 *
 * Behind an AlertDialog because the one consequence that matters is not
 * guessable from an unlabelled arrow: the pose resets to the new map's origin,
 * so the robot does not know where it is until the operator says. (A pose
 * measured in the old map's frame is not merely wrong in the new one, it is
 * meaningless — carried over, it can drop the robot inside a wall with every
 * indicator green. The dialog does not explain that; it just says what to do.)
 *
 * One sentence, and what was cut is the point. "The nav stack keeps running
 * throughout" was true and reassuring and earned nothing: it answers a worry
 * only someone who remembers the restart-based switch would have, and it is one
 * more line read *instead of* the one that asks for an action — the failure
 * MapDeleteControl's dialog is written against too. No dismiss-by-backdrop for
 * the same reason the delete uses one: the only ways out are the buttons.
 *
 * A refusal keeps the dialog open and shows the backend's sentence there, which
 * matters most for 409 `task_running` — it names the tasks still running, and
 * closing the dialog would take the list away with it.
 *
 * On success the card does *not* unmount (unlike rename and delete), but the
 * check badge moves to it from another card, so the sentence still goes up
 * through `onSwitched` to the library: it is about the robot, not about this
 * map, and it outlives the dialog it came from. Mutation-as-async-callback with
 * local busy/error, same as the other card controls — this codebase does not use
 * useMutation.
 */
export function MapActivateControl({
  map,
  onSwitched,
}: {
  map: MapSummary;
  /** The backend's sentence, for a surface that is not this card. */
  onSwitched?: (message: string) => void;
}) {
  const queryClient = useQueryClient();
  const [confirming, setConfirming] = React.useState(false);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const submit = React.useCallback(async () => {
    if (busy) return;
    setBusy(true);
    setError(null);
    try {
      const result = await activateMap(map.name);
      onSwitched?.(result.message);
      setConfirming(false);
      // `active` moved between two cards, and the dashboard reads the active
      // map's vertices — nothing else forces either refresh, because a live
      // swap (unlike the stack restart this replaced) drops no socket.
      void queryClient.invalidateQueries({ queryKey: queryKeys.maps });
      void queryClient.invalidateQueries({
        queryKey: queryKeys.mapVertices(map.name),
      });
      if (result.previous) {
        void queryClient.invalidateQueries({
          queryKey: queryKeys.mapVertices(result.previous),
        });
      }
    } catch (cause) {
      // Every refusal arrives as the backend's own sentence, written to be
      // shown. ActivateConflictError's code is not branched on here: each one
      // already says what to do, and none has a retry this control can offer
      // the way the rebuild's `gridmap_hand_edited` does.
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, [busy, map.name, onSwitched, queryClient]);

  // The active card says "in use" with its badge; it has no use for this.
  if (map.active) return null;

  const reason =
    map.grid_status === "converting"
      ? CONVERTING_REASON
      : !map.has_pointcloud
        ? NO_CLOUD_REASON
        : map.grid_status !== "ok"
          ? NO_GRID_REASON
          : null;

  return (
    <>
      {reason ? (
        <span
          aria-disabled="true"
          title={reason}
          className={cn(CORNER, "cursor-not-allowed text-muted-foreground opacity-40")}
        >
          <ArrowLeftRightIcon className="size-3.5" aria-hidden />
        </span>
      ) : (
        <button
          type="button"
          onClick={() => {
            setError(null);
            setConfirming(true);
          }}
          aria-label={`Switch the robot to ${map.name}`}
          title={`Switch the robot to ${map.name}`}
          className={cn(
            CORNER,
            "text-muted-foreground transition-colors hover:border-signal-cmd/50 hover:bg-signal-cmd/12 hover:text-signal-cmd",
          )}
        >
          <ArrowLeftRightIcon className="size-3.5" aria-hidden />
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
            <AlertDialogTitle>Switch to {map.name}?</AlertDialogTitle>
            <AlertDialogDescription>
              The robot&apos;s pose resets to this map&apos;s origin — set an
              initial pose on the dashboard afterwards.
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
            {/* Icon and label both, the house pattern: the glyph tells the two
              * apart at a glance, the word makes the committing one
              * unmistakable. Not `destructive` — nothing is destroyed, and
              * spending the red here would leave the delete dialog nothing
              * louder to say. */}
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
              Cancel
            </Button>
            <Button
              size="sm"
              disabled={busy}
              onClick={() => void submit()}
            >
              <ArrowLeftRightIcon data-icon="inline-start" />
              {busy ? "Switching…" : "Switch"}
            </Button>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}
