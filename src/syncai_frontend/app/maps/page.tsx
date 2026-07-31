"use client";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { MapLibrary } from "@/components/maps/map-library";

export default function MapsPage() {
  const { state } = useConsoleRobotState();

  return (
    // Like /settings, this screen owns its scroll: the shell's <main> does not.
    <div className="h-full overflow-y-auto">
      <div className="mx-auto w-full max-w-5xl px-4 py-8">
        <header className="mb-6">
          <p className="instrument-label text-muted-foreground">Robot</p>
          <h1 className="mt-1.5 text-xl font-semibold tracking-tight">Maps</h1>
          <p className="mt-1 text-sm text-muted-foreground">
            Maps saved on{" "}
            <span className="readout">{state?.robot_id ?? "this robot"}</span>.
            The one in use is set in the robot&apos;s instance config and changes
            only when the stack restarts — nothing on this screen switches it.
          </p>
        </header>

        <MapLibrary />
      </div>
    </div>
  );
}
