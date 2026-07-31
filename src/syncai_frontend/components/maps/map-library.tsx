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
 * The map catalogue. Read-only this round: there is no backend call that switches
 * or deletes a map, so the page shows what is on the robot and says where the
 * choice is actually made.
 */
export function MapLibrary() {
  const { maps, status } = useMaps();

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
    <div className={GRID}>
      {ordered.map((map) => (
        <MapCard key={map.name} map={map} />
      ))}
    </div>
  );
}
