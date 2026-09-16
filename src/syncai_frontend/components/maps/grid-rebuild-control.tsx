"use client";

import * as React from "react";
import { ChevronDownIcon, RefreshCwIcon } from "lucide-react";
import { useQueryClient } from "@tanstack/react-query";

import {
  AlertDialog,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Button } from "@/components/ui/button";
import { ConvertConflictError, convertMapGrid } from "@/lib/api/map";
import { queryKeys } from "@/lib/api/query-keys";
import type { GridRecipe, MapSummary } from "@/lib/types/map";

/**
 * (Re)build a map's 2D gridmap — the card-side face of
 * POST /api/v1/maps/{name}/grid/convert.
 *
 * This is the formal home of what field operators used to do over SSH with a
 * one-off script: pick the recipe when the default is wrong for the site. The
 * dropdown carries exactly the two recipes and nothing else — band offsets,
 * gap_fill_size and the debug switch stay API-only on purpose, because they are
 * tuned with the pipeline's intermediate clouds in front of you, not guessed
 * from a card.
 *
 * The hand-edit conflict is the one flow with structure: the backend answers
 * 409 `gridmap_hand_edited` for a map whose grid holds operator edits, and this
 * control turns that into a confirm dialog and a retry with `overwriteEdits`
 * rather than a dead error — the edited grid survives as gridmap_prev.pgm
 * either way, and the dialog says so. A `conversion_running` 409 and every
 * other failure just render as the backend's own sentence.
 *
 * There is no local "converting" state to hold, and no completion to report
 * either: success invalidates the maps query, the catalogue answers with
 * `grid_status: "converting"`, and useMaps' poll carries the card through to
 * `ok` or to `failed` with its reason. What this control owns is the request —
 * everything after it belongs to the card. Mutation-as-async-callback with
 * local busy/error, same as SaveMapControl — this codebase does not use
 * useMutation.
 */
export function GridRebuildControl({ map }: { map: MapSummary }) {
  const queryClient = useQueryClient();
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);
  const [message, setMessage] = React.useState<string | null>(null);
  const [confirm, setConfirm] = React.useState<{
    recipe: GridRecipe;
    detail: string;
  } | null>(null);

  const convert = React.useCallback(
    async (recipe: GridRecipe, overwriteEdits: boolean) => {
      setBusy(true);
      setError(null);
      setMessage(null);
      try {
        const result = await convertMapGrid(map.name, {
          recipe,
          overwriteEdits,
        });
        setMessage(result.message);
        // The catalogue now reports grid_status "converting"; invalidating is
        // what starts useMaps' poll and flips this card to its Converting…
        // state, and then to the outcome when the poll sees it land.
        void queryClient.invalidateQueries({ queryKey: queryKeys.maps });
      } catch (cause) {
        if (
          cause instanceof ConvertConflictError &&
          cause.code === "gridmap_hand_edited" &&
          !overwriteEdits
        ) {
          setConfirm({ recipe, detail: cause.message });
        } else {
          setError(cause instanceof Error ? cause.message : String(cause));
        }
      } finally {
        setBusy(false);
      }
    },
    [map.name, queryClient],
  );

  // Every state but "converting" is rebuildable, failures included — a failed
  // conversion is in fact the state most likely to want this control, with the
  // other recipe.
  const disabled =
    busy || map.grid_status === "converting" || !map.has_pointcloud;

  return (
    <div className="mt-2 space-y-1.5">
      <DropdownMenu>
        <DropdownMenuTrigger
          disabled={disabled}
          className="instrument-label flex h-5 items-center gap-1 rounded-sm border border-hairline px-1.5 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:pointer-events-none disabled:opacity-40"
        >
          <RefreshCwIcon className="size-3" aria-hidden />
          {map.grid ? "Rebuild grid" : "Build grid"}
          <ChevronDownIcon className="size-3" aria-hidden />
        </DropdownMenuTrigger>
        <DropdownMenuContent className="w-64">
          <DropdownMenuItem onClick={() => void convert("z-band", false)}>
            <div>
              <p className="text-sm">Z-band (default)</p>
              <p className="text-[11px] leading-snug text-muted-foreground">
                Trinary map, unknown where nothing was seen. Recoverable by
                driving or hand-editing — the safe choice on a new site.
              </p>
            </div>
          </DropdownMenuItem>
          <DropdownMenuItem onClick={() => void convert("traversability", false)}>
            <div>
              <p className="text-sm">Traversability</p>
              <p className="text-[11px] leading-snug text-muted-foreground">
                For sites too large to hand-edit. Everything the lidar did not
                see becomes a permanent wall — pick it deliberately.
              </p>
            </div>
          </DropdownMenuItem>
        </DropdownMenuContent>
      </DropdownMenu>

      {/* The backend's sentences, verbatim — same contract as SaveMapControl. */}
      {message && (
        <p className="text-[11px] leading-tight text-muted-foreground">
          {message}
        </p>
      )}
      {error && (
        <p role="alert" className="text-[11px] leading-tight text-signal-warn">
          {error}
        </p>
      )}

      <AlertDialog
        open={confirm !== null}
        onOpenChange={(open) => {
          if (!open) setConfirm(null);
        }}
      >
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Overwrite hand edits?</AlertDialogTitle>
            <AlertDialogDescription>
              {confirm?.detail ??
                "This map's gridmap holds hand edits; rebuilding replaces it."}{" "}
              The edited grid is kept as{" "}
              <span className="readout">gridmap_prev.pgm</span> in the map
              directory.
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <Button variant="ghost" size="sm" onClick={() => setConfirm(null)}>
              Keep the edits
            </Button>
            <Button
              size="sm"
              disabled={busy}
              onClick={() => {
                if (!confirm) return;
                const { recipe } = confirm;
                setConfirm(null);
                void convert(recipe, true);
              }}
            >
              Rebuild anyway
            </Button>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  );
}
