"use client";

import { Loader2Icon, MapPinIcon, XIcon } from "lucide-react";

import { overlayPanel } from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import { vertexGlyph } from "@/lib/map/vertex";
import type { MapVertex } from "@/lib/types/map";

/**
 * The re-place flow's only chrome: what is in hand, how to put it down, and how
 * to back out.
 *
 * It exists because the gesture is otherwise silent. Arming a re-place takes a
 * marker off the map and hands its pose to the pointer — from the operator's
 * side, a stop vanished — so something has to name the stop that is in flight
 * and offer a way out that is not "drag it somewhere and hope".
 *
 * Presentation only, like GoalControl beside it: the write, its in-flight flag
 * and its error all live in useActiveMapVertices.
 */
export function VertexPlaceControl({
  vertex,
  busy,
  error,
  onCancel,
  onDismissError,
  className,
}: {
  /** The stop being re-placed, or null when nothing is armed. */
  vertex: MapVertex | null;
  busy: boolean;
  error: string | null;
  onCancel: () => void;
  onDismissError: () => void;
  className?: string;
}) {
  // Nothing armed and nothing to report is the resting state of this flow, and
  // it draws no chrome at all — a permanent empty panel over the map would cost
  // the viewport a corner for a mode that is off.
  if (!vertex && !error && !busy) return null;

  return (
    <div className={cn(overlayPanel, "w-full p-2.5", className)}>
      {vertex && (
        <>
          <div className="flex items-center gap-1.5">
            <MapPinIcon aria-hidden className="size-3.5 text-signal-cmd" />
            <span className="instrument-label truncate text-signal-cmd">
              {vertexGlyph(vertex.type)} · {vertex.name}
            </span>
          </div>
          <p className="mt-1.5 text-[11px] leading-snug text-muted-foreground">
            Press on the map to set the position, drag to aim it, release to
            save.
          </p>
        </>
      )}

      {busy && (
        <p className="mt-1.5 flex items-center gap-1.5 text-[11px] text-muted-foreground">
          <Loader2Icon
            aria-hidden
            className="size-3 animate-spin motion-reduce:animate-none"
          />
          Saving the new position…
        </p>
      )}

      {error && (
        <p className="mt-1.5 text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}

      {/* One button, and which one it is follows what there is to undo: an armed
        * re-place can be abandoned, a failed one can only be acknowledged. */}
      {vertex ? (
        <button
          type="button"
          onClick={onCancel}
          className="instrument-label mt-2.5 flex h-7 w-full items-center justify-center gap-1.5 rounded-sm border border-hairline text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
        >
          <XIcon className="size-3.5" />
          Cancel
        </button>
      ) : (
        error && (
          <button
            type="button"
            onClick={onDismissError}
            className="instrument-label mt-2.5 flex h-7 w-full items-center justify-center rounded-sm border border-hairline text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
          >
            Dismiss
          </button>
        )
      )}
    </div>
  );
}
