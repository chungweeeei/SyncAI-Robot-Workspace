"use client";

import { RotateCcwIcon } from "lucide-react";

import { InstrumentGroup } from "@/components/console/instrument";
import { Button } from "@/components/ui/button";

/**
 * Throw the current run away and start a new map, restarting nothing.
 *
 * The other exit from a mapping run, and the destructive one: SaveMapControl
 * above makes a run permanent, this discards it. Placed below that control on
 * purpose — the rail then reads mode → save → start over, and the button that
 * loses work never sits above the one that prevents the loss.
 *
 * Presentational, in ModeControl's idiom rather than SaveMapControl's: the
 * click goes out through `onRequest` and the page performs the reset, because
 * the page owns the one rule that may stop it (the run is unsaved) and is the
 * only thing that can re-arm that rule afterwards. SaveMapControl can own its
 * own request precisely because saving needs no such permission.
 */
export function ResetRunControl({
  enabled,
  busy,
  error,
  done,
  onRequest,
}: {
  /** False outside MANUAL — there is no run to reset and the POST would 502. */
  enabled: boolean;
  busy: boolean;
  /** The backend's own sentence for a failure; rendered verbatim. */
  error: string | null;
  /** The backend's own sentence for a success; rendered verbatim. */
  done: string | null;
  onRequest: () => void;
}) {
  return (
    <InstrumentGroup
      label="New map"
      caption={
        enabled
          ? "Discards the run in the robot's memory. Keep the robot still — the lidar re-levels itself against gravity."
          : "Starting a new map needs mapping mode."
      }
    >
      <Button
        type="button"
        variant="destructive"
        size="sm"
        disabled={!enabled || busy}
        onClick={onRequest}
        className="w-full"
      >
        <RotateCcwIcon data-icon="inline-start" />
        {busy ? "Resetting…" : "Start a new map"}
      </Button>

      {done && <p className="text-[11px] leading-snug text-signal-live">{done}</p>}

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}
