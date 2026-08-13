"use client";

import * as React from "react";
import { SaveIcon } from "lucide-react";
import { useQueryClient } from "@tanstack/react-query";

import { InstrumentGroup } from "@/components/console/instrument";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { MAP_NAME_RE, saveMap } from "@/lib/api/mapping";
import { queryKeys } from "@/lib/api/query-keys";

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
 * The saved map appears on the Maps screen either way (the maps query is
 * invalidated here); without a grid yet, its card just has no thumbnail.
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

  const valid = MAP_NAME_RE.test(name);

  const save = React.useCallback(async () => {
    setBusy(true);
    setError(null);
    setSaved(null);
    try {
      const result = await saveMap(name);
      setSaved(result.message);
      setName("");
      onSaved();
      // The Maps screen's card list is fed by this key; invalidating is what
      // makes the new map appear there without a manual refresh.
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

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}
