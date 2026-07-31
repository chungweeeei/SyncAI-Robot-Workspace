"use client";

import * as React from "react";
import Link from "next/link";
import { BoxIcon, ChevronDownIcon, LayersIcon, PencilIcon } from "lucide-react";

import { Chip, Readout } from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import type { MapSummary } from "@/lib/types/map";

/** GiB/MiB, one decimal — a map is 20–50 MB of .pcd and nothing needs bytes. */
function formatSize(bytes: number): string {
  const mib = bytes / (1024 * 1024);
  if (mib >= 1024) return `${(mib / 1024).toFixed(1)} GiB`;
  return `${mib.toFixed(0)} MiB`;
}

/**
 * `2026-07-31 10:35` sliced straight out of the ISO string rather than run
 * through toLocaleString: this is a client component that Next still prerenders,
 * and a server/browser timezone difference would be a hydration mismatch. UTC
 * for everyone is the honest trade.
 */
function formatTimestamp(iso: string): string {
  return iso.slice(0, 16).replace("T", " ");
}

/**
 * The tile the map is read from.
 *
 * The backdrop is a fixed light neutral, not a theme surface. A gridmap image is
 * white-free / near-black-obstacle in both themes (the same constraint the 2D map
 * canvas recorded), so a dark tile would put black walls on near-black ground and
 * the map would read as an empty rectangle. The tile is a hair lighter than the
 * 205 unknown-grey so the map's own unknown region still reads as part of the map.
 */
function MapThumbnail({ map }: { map: MapSummary }) {
  if (!map.thumbnail) {
    return (
      <div className="flex aspect-[4/3] flex-col items-center justify-center gap-2 border-b border-hairline bg-elevated">
        <LayersIcon className="size-6 text-muted-foreground" aria-hidden />
        <span className="instrument-label text-muted-foreground">
          No preview
        </span>
      </div>
    );
  }

  return (
    <div className="aspect-[4/3] border-b border-hairline bg-[#e4e4e4] p-2">
      {/* Plain <img>: the real thumbnail URL is resolved at runtime by apiUrl()
       * from the page's own hostname, so next/image would need a
       * images.remotePatterns entry for a host that is not known at build time. */}
      {/* eslint-disable-next-line @next/next/no-img-element */}
      <img
        src={map.thumbnail}
        alt={`Occupancy grid of ${map.name}`}
        className="size-full object-contain"
      />
    </div>
  );
}

/**
 * One map in the catalogue.
 *
 * Instrument vocabulary rather than a shadcn Card: the values here are the same
 * kind of readout the telemetry rail carries, and `signal-cmd` for the loaded map
 * is the console's existing meaning for "the value in force" — the rail's location
 * tick and the active segment of a Segmented control are the same hue.
 *
 * The readouts collapse, and default to collapsed. Six of them per card is more
 * than this screen is for — the question it answers is "which maps are on the
 * robot and which one is loaded", which the thumbnail, the name and the chips
 * answer on their own. Extents and byte counts are what you open one card to
 * check, not what you scan four cards for.
 *
 * What never collapses: the "In use" and "No 2D grid" chips. A map the nav stack
 * cannot load must say so with the card shut, or the flag is worthless.
 */
export function MapCard({ map }: { map: MapSummary }) {
  const [open, setOpen] = React.useState(false);
  const grid = map.grid;
  const detailsId = `map-${map.name}-details`;

  return (
    <article
      aria-current={map.active ? "true" : undefined}
      className={cn(
        "overflow-hidden rounded-sm border",
        map.active
          ? "border-signal-cmd bg-signal-cmd/5"
          : "border-hairline bg-panel",
      )}
    >
      <MapThumbnail map={map} />

      <div className="px-3 py-3">
        <header className="flex items-center gap-2">
          <h2 className="readout min-w-0 flex-1 truncate text-[15px] font-medium">
            {map.name}
          </h2>
          {map.active && <Chip tone="cmd">In use</Chip>}
          <button
            type="button"
            onClick={() => setOpen((prev) => !prev)}
            aria-expanded={open}
            aria-controls={detailsId}
            className="instrument-label flex h-5 shrink-0 items-center gap-1 rounded-sm px-1 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
          >
            {open ? "Hide" : "Details"}
            <ChevronDownIcon
              aria-hidden
              className={cn("size-3 transition-transform", open && "rotate-180")}
            />
          </button>
        </header>

        <div className="mt-2.5 flex flex-wrap items-center gap-1.5">
          {map.has_pointcloud && (
            <Chip tone="neutral">
              <BoxIcon className="mr-1 size-3" aria-hidden />
              Point cloud
            </Chip>
          )}
          {!grid && <Chip tone="caution">No 2D grid</Chip>}

          {/* A map with no gridmap has nothing to paint on, so the link is a
           * disabled span rather than a route that would land on a guard screen. */}
          {grid ? (
            <Link
              href={`/maps/${encodeURIComponent(map.name)}/edit`}
              className="instrument-label ml-auto flex h-5 items-center gap-1 rounded-sm border border-hairline px-1.5 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
            >
              <PencilIcon className="size-3" aria-hidden />
              Edit
            </Link>
          ) : (
            <span className="instrument-label ml-auto flex h-5 items-center gap-1 rounded-sm border border-hairline px-1.5 text-muted-foreground opacity-40">
              <PencilIcon className="size-3" aria-hidden />
              Edit
            </span>
          )}
        </div>

        {/* `hidden` rather than unmounting: aria-controls above must keep pointing
         * at an element that exists, and the native attribute is what takes the
         * collapsed rows out of the accessibility tree. */}
        <div id={detailsId} hidden={!open}>
          <div className="mt-2.5 space-y-1.5 border-t border-hairline pt-2.5">
            {grid ? (
              <>
                <Readout
                  label="Extent"
                  value={`${(grid.width * grid.resolution).toFixed(1)} × ${(
                    grid.height * grid.resolution
                  ).toFixed(1)}`}
                  unit="m"
                />
                <Readout
                  label="Grid"
                  value={`${grid.width} × ${grid.height}`}
                  unit="cells"
                />
                <Readout
                  label="Resolution"
                  value={grid.resolution.toFixed(2)}
                  unit="m/cell"
                />
              </>
            ) : (
              <Readout label="Grid" value="—" tone="caution" />
            )}
            <Readout label="Vertices" value={map.vertex_count} />
            <Readout label="Size" value={formatSize(map.size_bytes)} />
            <Readout label="Saved" value={formatTimestamp(map.modified_at)} />
          </div>

          {!grid && (
            <p className="mt-2.5 text-[11px] leading-tight text-muted-foreground">
              Saved from LIO but never converted, so the nav stack cannot load it.
              Run <span className="readout">tools/pcd_to_gridmap.py</span> over
              its <span className="readout">map.pcd</span> to produce a gridmap.
            </p>
          )}
        </div>
      </div>
    </article>
  );
}
