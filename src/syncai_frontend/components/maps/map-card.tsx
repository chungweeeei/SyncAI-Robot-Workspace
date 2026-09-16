"use client";

import * as React from "react";
import Link from "next/link";
import {
  BoxIcon,
  CheckIcon,
  ChevronDownIcon,
  LayersIcon,
  PencilIcon,
} from "lucide-react";

import { Chip, Readout } from "@/components/console/instrument";
import { GridRebuildControl } from "@/components/maps/grid-rebuild-control";
import { MapActivateControl } from "@/components/maps/map-activate-control";
import { MapDeleteControl } from "@/components/maps/map-delete-control";
import { MapRenameControl } from "@/components/maps/map-rename-control";
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
 * The chip that carries a map's gridmap state, or nothing when it has a good one.
 *
 * Tones are the console's EFIS semantic, not a palette choice: `warn` is for a
 * fault and only `failed` is one — the pipeline rejected this cloud and the same
 * recipe will reject it again. `none` and `interrupted` are merely not-done, so
 * they take `caution` and are told apart by their labels, and `converting`
 * takes `active` because the system is mid-action.
 *
 * `ok` renders nothing at all. A chip on every healthy card would be four
 * chips saying "fine" on a screen whose question is which map is loaded.
 */
function GridStatusChip({ map }: { map: MapSummary }) {
  switch (map.grid_status) {
    case "converting":
      return <Chip tone="active">Converting…</Chip>;
    case "failed":
      return (
        <Chip tone="warn" title={map.grid_error ?? undefined}>
          Conversion failed
        </Chip>
      );
    case "interrupted":
      return <Chip tone="caution">Conversion stopped</Chip>;
    case "none":
      return <Chip tone="caution">No 2D grid</Chip>;
    default:
      return null;
  }
}

/**
 * What to do about a gridmap that is not there, or not the one that was asked
 * for — the actionable half of the chip above, next to the control that acts.
 *
 * Always visible rather than inside the card's collapsed details, and that is
 * the point of the whole change: these sentences used to be one generic
 * "never converted" line, and a conversion that actually failed said nothing
 * here and left its reason in the robot's backend log. A reason an operator
 * cannot see is a reason nobody reads.
 *
 * Silent for `ok` and for `converting`, where the chip already says everything
 * and a second line would only push the controls down for the minute it runs.
 *
 * Every other branch ends by naming what to do next, and none of them may name
 * Build grid unless it is actually on the card — GridRebuildControl renders
 * only for a map with a cloud, and telling an operator to press a control that
 * is not there is worse than the generic line this replaced.
 */
function GridStatusNote({ map }: { map: MapSummary }) {
  if (map.grid_status === "ok" || map.grid_status === "converting") return null;

  if (!map.has_pointcloud) {
    // No map.pcd, so there is nothing to convert from and no control to offer:
    // pgo either never wrote the cloud or it was removed from the directory by
    // hand. Re-saving the run is the only way back, and this run is gone.
    return (
      <p className="mt-2 text-[11px] leading-tight text-signal-warn">
        This map has no point cloud, so no 2D grid can be built from it. Only a
        fresh mapping run can replace it.
      </p>
    );
  }

  if (map.grid_status === "failed") {
    return (
      <p className="mt-2 text-[11px] leading-tight text-signal-warn">
        {map.grid_error ?? "The conversion failed for a reason the robot did not record."}{" "}
        <span className="text-muted-foreground">
          {map.grid
            ? "The map is still serving the grid it had before. Rebuild it with the other recipe to try again."
            : "Rebuilding with the same recipe will fail the same way — try the other one."}
        </span>
      </p>
    );
  }

  if (map.grid_status === "interrupted") {
    return (
      <p className="mt-2 text-[11px] leading-tight text-muted-foreground">
        The backend stopped before this conversion finished, so the grid was
        never built. Rebuild it — nothing is wrong with the cloud.
      </p>
    );
  }

  // "none" last rather than as a switch case, because it doubles as the
  // fallback: a status this build has not heard of means a backend ahead of
  // this frontend, and "no usable grid, build one" is the safe thing to say
  // about any state that is not one of the four handled above.
  return (
    <p className="mt-2 text-[11px] leading-tight text-muted-foreground">
      Saved from LIO but never converted, so the nav stack cannot load it. Build
      the grid to make it loadable.
    </p>
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
 * What never collapses: "In use", the gridmap-state chip, and the sentence that
 * says what to do about it. A map the nav stack cannot load must say so with the
 * card shut, or the flag is worthless — and a conversion in flight is why Edit
 * and Rebuild are greyed, so hiding it would make the card look broken instead
 * of busy. The note earns the same treatment for a stronger reason: a failed
 * conversion's only other record is a log line on the robot.
 *
 * "In use" gets three signals rather than one, because *which map is loaded* is
 * the question the screen exists to answer and one card in the grid has to win
 * it outright: a solid check in the thumbnail's corner, a ring around the card,
 * and the sort order (MapLibrary puts it first). It used to be a header chip
 * and a 5% tint, which is a difference you find by comparing cards instead of
 * one you see. The wording lives in the badge's tooltip and accessible name,
 * and `aria-current` carries the fact to a screen reader on its own.
 *
 * The title is the one editable value on the card (MapRenameControl). Delete is
 * the other thing that can happen to a whole map, and it lives in the corner
 * (MapDeleteControl) rather than in the header row — an X is the glyph for it,
 * and putting it among the word-labels would make the destructive action look
 * like one more view to open. Both are greyed on the map in use, for the same
 * reason: the stack was launched with that name, nothing here can re-point it,
 * and the backend refuses either way.
 */
export function MapCard({
  map,
  onRenamed,
  onDeleted,
  onSwitched,
}: {
  map: MapSummary;
  /** Forwarded to the rename control; see MapRenameControl for why it goes up. */
  onRenamed?: (message: string) => void;
  /** Same contract for the delete control — the card unmounts on success. */
  onDeleted?: (message: string) => void;
  /** The backend's sentence about where the robot ended up after a map switch. */
  onSwitched?: (message: string) => void;
}) {
  const [open, setOpen] = React.useState(false);
  const grid = map.grid;
  const detailsId = `map-${map.name}-details`;

  return (
    <article
      aria-current={map.active ? "true" : undefined}
      className={cn(
        // `relative` is what the corner overlays are pinned to; see
        // MapDeleteControl for why it owns the absolute rather than a wrapper.
        "relative overflow-hidden rounded-sm border",
        map.active
          ? // A ring *and* the border: the ring draws outside the box, so this
            // reads as a 2px outline without the 1px reflow a border-2 would
            // cause on one card in a grid of otherwise identical ones.
            "border-signal-cmd bg-signal-cmd/8 ring-1 ring-signal-cmd"
          : "border-hairline bg-panel",
      )}
    >
      {/* The answer to the only question this screen asks, in the corner the
        * delete X does not use — and the corner MapActivateControl takes over
        * on every card where the answer is "not this one".
        *
        * A check, not the words "In use". One card in the grid carries it, so
        * it is read as *which one*, not as something to parse — and the same
        * geometry as the delete X opposite makes the pair read as a system:
        * two 24px corner tiles, one solid (a state) and one outlined (an
        * action). The words survive as the tooltip and as the accessible name,
        * because a glyph alone cannot say which of "loaded", "selected" or
        * "converted" it means; `aria-current` on the card says it again.
        *
        * bg-primary is signal-cmd in both themes and ships a foreground already
        * paired to it, so no colour is spelled out here. */}
      {map.active ? (
        <span
          title="In use — the map the running stack loaded"
          aria-label="In use"
          className="absolute top-2 left-2 z-10 flex size-6 items-center justify-center rounded-sm bg-primary text-primary-foreground shadow-sm"
        >
          <CheckIcon className="size-3.5" aria-hidden />
        </span>
      ) : (
        // The same corner, the opposite job: the badge above is a state, this
        // is the action that moves it here. Swap arrows rather than an unfilled
        // check, which would read as "already done, greyed out" — see that
        // component for the full argument. Mutually exclusive with the badge by
        // construction: MapActivateControl renders nothing when the map is
        // active.
        <MapActivateControl map={map} onSwitched={onSwitched} />
      )}
      <MapDeleteControl map={map} onDeleted={onDeleted} />
      <MapThumbnail map={map} />

      <div className="px-3 py-3">
        <header className="flex items-center gap-2">
          <MapRenameControl map={map} onRenamed={onRenamed} />
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
          <GridStatusChip map={map} />

          {/* A map with no gridmap has nothing to paint on, so the link is a
           * disabled span rather than a route that would land on a guard screen.
           * Also disabled mid-conversion: the grid on disk is about to be
           * replaced, and an editor opened now would save cells onto a map with
           * different extents. A map whose re-conversion *failed* keeps its
           * editor — the grid it is serving is the archived one, which is a real
           * grid and the only one it has. */}
          {grid && map.grid_status !== "converting" ? (
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

        {/* Why the grid is missing or stale, then the control that fixes it.
         * Both below the chips rather than among them: each carries a full
         * sentence, and the rebuild control has its own status line (the
         * backend's reply, or an error). Only for maps with a cloud — without
         * map.pcd there is nothing to convert and the backend would 400. */}
        <GridStatusNote map={map} />
        {map.has_pointcloud && <GridRebuildControl map={map} />}

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
        </div>
      </div>
    </article>
  );
}
