"use client";

import * as React from "react";
import { useParams, useRouter } from "next/navigation";
import { ArrowLeftIcon } from "lucide-react";

import { MapGridEditor } from "@/components/maps/map-grid-editor";

/**
 * The gridmap editor.
 *
 * Replaces a step that was being done in GIMP: `map/dp2f/gridmap.pgm` on the robot
 * carries ~30 000 hand-painted white cells and a `gridmap_raw.pgm` backup beside it.
 *
 * Chrome only — MapGridEditor owns the editing. The canvas sizes itself from its
 * container, so this column has to give it a definite height (`min-h-0` + `flex-1`)
 * rather than letting `h-full` resolve against nothing. The shell's <main> does not
 * scroll and neither does this page.
 */
export default function MapEditPage() {
  const params = useParams<{ name: string }>();
  const router = useRouter();
  const name = params.name;

  /**
   * Mirrored out of the editor for one reason: the App Router has no navigation
   * blocker, so `beforeunload` in the editor catches a reload or a tab close but
   * cannot see a client-side navigation. The back button is the one in-app exit
   * from this screen, so it has to ask.
   */
  const [dirty, setDirty] = React.useState(false);

  const goBack = () => {
    if (
      dirty &&
      !window.confirm("Leave the editor? Unsaved edits to this gridmap will be lost.")
    ) {
      return;
    }
    router.push("/maps");
  };

  return (
    <div className="flex h-full min-h-0 flex-col">
      <header className="flex shrink-0 items-center gap-3 border-b border-hairline px-4 py-2.5">
        <button
          type="button"
          onClick={goBack}
          aria-label="Back to maps"
          className="flex size-7 shrink-0 items-center justify-center rounded-sm border border-hairline text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground"
        >
          <ArrowLeftIcon className="size-3.5" aria-hidden />
        </button>
        <div className="min-w-0">
          <p className="instrument-label text-muted-foreground">Gridmap editor</p>
          <h1 className="readout truncate text-[15px] font-medium">{name}</h1>
        </div>
      </header>

      <div className="min-h-0 flex-1">
        <MapGridEditor name={name} onDirtyChange={setDirty} />
      </div>
    </div>
  );
}
