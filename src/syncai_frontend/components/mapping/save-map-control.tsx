"use client";

import * as React from "react";
import Link from "next/link";
import { SaveIcon } from "lucide-react";
import { useQueryClient } from "@tanstack/react-query";

import { InstrumentGroup } from "@/components/console/instrument";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { useMapConversion } from "@/hooks/use-maps";
import { MAP_NAME_RE, saveMap } from "@/lib/api/mapping";
import { queryKeys } from "@/lib/api/query-keys";
import type { MapSummary } from "@/lib/types/map";

/**
 * How the 2D-grid conversion of the map just saved is going.
 *
 * The other half of Save, and the reason this component exists: saving writes
 * the cloud, then the backend converts it to the gridmap the nav stack actually
 * loads — on a background thread, over tens of seconds, long after the POST has
 * answered. Until this line existed the flow ended at "converting in the
 * background", with the outcome visible only on the Maps screen, so an operator
 * who saved and stayed here was never told whether the map they just made is
 * usable. Saving and converting are one act to the person doing it; only the
 * backend has a reason to see two.
 *
 * Renders nothing until the catalogue has an entry to report, which covers the
 * seconds between the save landing and the first poll.
 */
function ConversionLine({ map }: { map: MapSummary | null }) {
  if (map === null) return null;

  switch (map.grid_status) {
    case "converting":
      return (
        <p className="text-[11px] leading-snug text-signal-active">
          Building the 2D grid…
        </p>
      );
    case "ok":
      return (
        <p className="text-[11px] leading-snug text-signal-live">
          2D grid ready — the map is loadable.{" "}
          <Link href="/maps" className="underline underline-offset-2">
            Open the map library
          </Link>
        </p>
      );
    case "failed":
      // The backend's own diagnosis, verbatim. It names what the pipeline
      // rejected ("intensity/normal gate selected no ground points"), which is
      // the only thing that tells an operator whether to try the other recipe
      // or go and look at the cloud.
      return (
        <p className="text-[11px] leading-snug text-signal-warn">
          The 2D grid failed to build:{" "}
          {map.grid_error ?? "the robot did not record a reason"}. The cloud is
          saved — rebuild the grid from the map library.
        </p>
      );
    case "interrupted":
      return (
        <p className="text-[11px] leading-snug text-signal-caution">
          The 2D grid was not finished — the backend stopped mid-conversion.
          The cloud is saved; rebuild the grid from the map library.
        </p>
      );
    default:
      // "none": the save answered before the conversion started, or it never
      // did (no map.pcd). The backend's save message already says to run the
      // conversion by hand, so a second sentence here would only repeat it.
      return null;
  }
}

/**
 * Name and save the map the current run has built.
 *
 * This is the only durable exit for a mapping run: pgo holds the keyframes in
 * RAM and nothing else serialises them, so until this succeeds the map exists
 * only while the mapping session does. The page reads `onSaved` to lift its
 * leave-without-saving guard.
 *
 * The success message is the backend's sentence verbatim — it is the one that
 * knows whether the 2D-grid conversion was started or has to be run by hand.
 * The conversion's *outcome* then arrives through the catalogue, watched by
 * useMapConversion, because the POST answers as soon as the cloud is on disk
 * and cannot speak for a thread that outlives it.
 */
export function SaveMapControl({
  enabled,
  onSaved,
}: {
  /** False outside MANUAL — there is no run to save and the POST would 502. */
  enabled: boolean;
  onSaved: () => void;
}) {
  const queryClient = useQueryClient();
  const [name, setName] = React.useState("");
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);
  const [saved, setSaved] = React.useState<string | null>(null);
  // The map whose conversion this screen is following. Held rather than derived
  // because "the one I just saved" is not something the catalogue can be asked:
  // it lists every map on the robot and says nothing about which one is this
  // operator's. Null until a save succeeds, which is also what keeps the
  // mapping screen from fetching the catalogue it otherwise has no use for.
  const [watching, setWatching] = React.useState<string | null>(null);

  const converting = useMapConversion(watching);
  const valid = MAP_NAME_RE.test(name);

  const save = React.useCallback(async () => {
    setBusy(true);
    setError(null);
    setSaved(null);
    setWatching(null);
    try {
      const result = await saveMap(name);
      setSaved(result.message);
      setName("");
      // Only when the backend says a conversion started. For grid_pending
      // false there is nothing running to watch, and the line would sit on
      // "none" saying nothing.
      if (result.grid_pending) setWatching(result.name);
      onSaved();
      // The Maps screen's card list is fed by this key; invalidating is what
      // makes the new map appear there without a manual refresh, and what gets
      // the conversion line its first reading here.
      void queryClient.invalidateQueries({ queryKey: queryKeys.maps });
    } catch (cause) {
      // 409 (name taken), pgo's "NO POSES!", the wrong-mode 502 — all arrive
      // as the backend's own sentence, written to be shown.
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, [name, onSaved, queryClient]);

  return (
    <InstrumentGroup
      label="Save map"
      caption={
        enabled
          ? "The run lives in the robot's memory until saved. Saving can take a minute on a large site."
          : "Saving needs mapping mode — there is no run to save in Nav."
      }
    >
      <form
        // A form so Enter in the input saves, matching what a name-and-confirm
        // row is expected to do.
        onSubmit={(event) => {
          event.preventDefault();
          if (enabled && valid && !busy) void save();
        }}
        className="flex items-center gap-2"
      >
        <Input
          value={name}
          onChange={(event) => setName(event.target.value)}
          placeholder="map name"
          aria-label="Map name"
          disabled={!enabled || busy}
          className="h-8 flex-1 text-sm"
        />
        <Button
          type="submit"
          size="sm"
          disabled={!enabled || !valid || busy}
        >
          <SaveIcon data-icon="inline-start" />
          {busy ? "Saving…" : "Save"}
        </Button>
      </form>

      {/* The rule, shown only while the name breaks it — a resting hint would
        * be one more line of chrome on a row whose job is obvious. */}
      {name.length > 0 && !valid && (
        <p className="text-[11px] leading-snug text-signal-caution">
          Letters, digits, dot, dash and underscore only, up to 64 characters.
        </p>
      )}

      {saved && (
        <p className="text-[11px] leading-snug text-signal-live">{saved}</p>
      )}

      {/* Under the save message, not replacing it: the two say different
        * things — that the cloud is on disk (permanent, and the thing that
        * makes the run safe to leave) and how the grid built from it went. */}
      <ConversionLine map={converting} />

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}
