"use client";

import * as React from "react";
import { MapPinIcon, SendIcon } from "lucide-react";

import { Readout } from "@/components/console/instrument";
import {
  AlertDialog,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { Button } from "@/components/ui/button";
import { VERTEX_TYPES, vertexGlyph } from "@/lib/map/vertex";
import type { MapVertex } from "@/lib/types/map";

/**
 * "Move to this stop?" — the confirm behind a double-click on a vertex in the
 * viewport.
 *
 * The dialog is where the vertex's *name* lives. On the map a stop is a mark and
 * a one-letter type badge and nothing else: a caption per stop turns a dozen
 * markers into a wall of text, and the only moment the operator needs the name
 * is this one, when they are about to send the robot to it.
 *
 * Confirming dispatches the MOVE task outright rather than staging it — the
 * question this dialog asks *is* the confirmation the drag-a-goal flow gets from
 * its read-back and Send button, and asking it twice teaches an operator to
 * click through it. Everything after dispatch (task chip, errors, cancel) is
 * read from GoalControl, which is already the one place task state is shown.
 *
 * The second action, Reposition, edits the stop instead of driving to it. The
 * two live in one dialog because they answer the same thought — the operator
 * double-clicked a stop because they are looking at where it sits — and because
 * the alternative is a second gesture on the map for a rare action. It is styled
 * as the secondary of the two: sending the robot is what this dialog is for, and
 * moving the mark is what you do when the stop turns out to be in a wall.
 */
export function VertexMoveDialog({
  vertex,
  busy,
  running,
  onConfirm,
  onReplace,
  onClose,
}: {
  /** The stop being asked about, or null when the dialog is closed. */
  vertex: MapVertex | null;
  /** A submit is in flight. */
  busy: boolean;
  /** A task is already running, so this one cannot be dispatched. */
  running: boolean;
  onConfirm: (vertex: MapVertex) => void;
  /** Hand the stop's pose to the pointer so it can be put somewhere else. */
  onReplace: (vertex: MapVertex) => void;
  onClose: () => void;
}) {
  // The close transition outlives the prop going null, so the last stop asked
  // about is kept and rendered while the popup animates out. Without it the
  // dialog empties itself on the way off screen.
  //
  // Held in state and adjusted during render (React's own "derive state from
  // props" escape hatch) rather than in an effect: an effect would paint the
  // empty dialog for one frame first, which is the flash this exists to avoid.
  const [shown, setShown] = React.useState<MapVertex | null>(vertex);
  if (vertex && vertex !== shown) setShown(vertex);

  const spec = shown && VERTEX_TYPES.find((type) => type.value === shown.type);

  return (
    <AlertDialog
      open={vertex !== null}
      onOpenChange={(open) => {
        if (!open) onClose();
      }}
    >
      <AlertDialogContent>
        {shown && (
          <>
            <AlertDialogHeader>
              <AlertDialogTitle>Move to {shown.name}?</AlertDialogTitle>
              <AlertDialogDescription>
                {spec
                  ? `${vertexGlyph(shown.type)} · ${spec.label} — ${spec.hint}`
                  : `Type ${shown.type}.`}{" "}
                The robot drives there on its own once the task is sent.
              </AlertDialogDescription>
            </AlertDialogHeader>

            {/* Same three readouts, in the same order and the same commanded
              * hue, as the staged goal in GoalControl — this is the same pose,
              * reached a different way. */}
            <div className="space-y-1 rounded-md border border-hairline bg-elevated/50 p-2.5">
              <Readout label="X" value={shown.x.toFixed(2)} unit="m" tone="cmd" />
              <Readout label="Y" value={shown.y.toFixed(2)} unit="m" tone="cmd" />
              <Readout
                label="Heading"
                value={shown.theta.toFixed(1)}
                unit="°"
                tone="cmd"
              />
            </div>

            {running && (
              <p className="text-[11px] leading-snug text-signal-caution">
                A task is already running. Cancel it first — the robot takes one
                goal at a time.
              </p>
            )}

            <AlertDialogFooter>
              <Button
                variant="ghost"
                size="sm"
                // Deliberately live while a task runs: the robot being on its
                // way somewhere says nothing about whether this mark is in the
                // right place, and the re-place writes a row, not a command.
                className="mr-auto"
                onClick={() => onReplace(shown)}
              >
                <MapPinIcon data-icon="inline-start" />
                Reposition
              </Button>
              <Button variant="outline" size="sm" onClick={onClose}>
                Cancel
              </Button>
              <Button
                size="sm"
                disabled={busy || running}
                onClick={() => onConfirm(shown)}
              >
                <SendIcon data-icon="inline-start" />
                Move
              </Button>
            </AlertDialogFooter>
          </>
        )}
      </AlertDialogContent>
    </AlertDialog>
  );
}
