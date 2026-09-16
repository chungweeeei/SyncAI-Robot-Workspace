"use client";

import * as React from "react";

import { MapCard } from "@/components/maps/map-card";
import { Skeleton } from "@/components/ui/skeleton";
import { useMaps } from "@/hooks/use-maps";

const GRID = "grid gap-4 sm:grid-cols-2 xl:grid-cols-3";

/** Same panel shape /settings uses when a state frame has not arrived. */
function Notice({
  label,
  children,
}: {
  label: string;
  children: React.ReactNode;
}) {
  return (
    <div className="rounded-md border border-hairline bg-panel p-4">
      <p className="instrument-label text-muted-foreground">{label}</p>
      <p className="mt-2 text-sm">{children}</p>
    </div>
  );
}

function LoadingGrid() {
  return (
    <div className={GRID} aria-busy>
      {[0, 1, 2].map((i) => (
        <div
          key={i}
          className="overflow-hidden rounded-sm border border-hairline bg-panel"
        >
          <Skeleton className="aspect-[4/3] rounded-none" />
          <div className="space-y-2 px-3 py-3">
            <Skeleton className="h-4 w-24" />
            <Skeleton className="h-3 w-full" />
            <Skeleton className="h-3 w-2/3" />
          </div>
        </div>
      ))}
    </div>
  );
}

/**
 * The map catalogue: what is on the robot, and which map it is running on.
 *
 * That last fact used to come with a caveat this page had to spell out, because
 * the choice was made in the instance INI and taking effect meant restarting the
 * stack. It no longer does — the card's Switch control re-points the running
 * localizer and map_server and rewrites the INI in one call.
 *
 * Renaming, deleting and switching are calls: each card carries all three
 * controls, and the backend's sentence about any of them is held *here*. For a
 * rename or a delete that is because the card is keyed by its name and both
 * outcomes unmount it with the refetch. A switch leaves its card mounted, but
 * its sentence is about the *robot* — where it ended up, and whether it still
 * needs an initial pose — so it belongs above the grid rather than inside the
 * card of a map that is now merely one of several.
 */
export function MapLibrary() {
  const { maps, status } = useMaps();
  const [lastResult, setLastResult] = React.useState<string | null>(null);

  if (!maps) {
    if (status === "error") {
      return (
        <Notice label="Maps unavailable">
          The robot&apos;s map list could not be read.
        </Notice>
      );
    }
    return <LoadingGrid />;
  }

  if (maps.length === 0) {
    return (
      <Notice label="No maps">
        This robot has no saved maps. They are written to{" "}
        <span className="readout">map/&lt;name&gt;/</span> when a mapping run is
        saved.
      </Notice>
    );
  }

  // Loaded map first — it is the one an operator is looking for — then by name.
  const ordered = [...maps].sort((a, b) => {
    if (a.active !== b.active) return a.active ? -1 : 1;
    return a.name.localeCompare(b.name);
  });

  return (
    <div>
      {/* The backend's sentence, verbatim — same contract as the other map
        * controls. It says how many vertices and templates followed the name,
        * or how many went with the deleted map, which neither card is around to
        * show. One line for both: they are the same kind of answer, and the
        * second one to arrive is the one worth reading. */}
      {lastResult && (
        <p
          role="status"
          className="mb-3 text-[11px] leading-tight text-muted-foreground"
        >
          {lastResult}
        </p>
      )}
      <div className={GRID}>
        {ordered.map((map) => (
          <MapCard
            key={map.name}
            map={map}
            onRenamed={setLastResult}
            onDeleted={setLastResult}
            onSwitched={setLastResult}
          />
        ))}
      </div>
    </div>
  );
}
