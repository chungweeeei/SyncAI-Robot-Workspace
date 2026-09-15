"use client";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { AppearanceSettings } from "@/components/settings/appearance-settings";
import { NetworkSettings } from "@/components/settings/network-settings";

export default function SettingsPage() {
  const { state } = useConsoleRobotState();

  return (
    // Settings is the one screen that scrolls; the shell's <main> does not.
    <div className="h-full overflow-y-auto">
      <div className="mx-auto w-full max-w-2xl px-4 py-8">
        <header className="mb-6">
          <p className="instrument-label text-muted-foreground">Robot</p>
          <h1 className="mt-1.5 text-xl font-semibold tracking-tight">
            Settings
          </h1>
          <p className="mt-1 text-sm text-muted-foreground">
            Network and console preferences for{" "}
            <span className="readout">{state?.robot_id ?? "this robot"}</span>.
          </p>
        </header>

        <div className="grid gap-4">
          {/* Not gated on a state frame: scan and connect do not need one, and
              a console that has never heard from the robot is exactly the one
              that needs to fix its WiFi. The card degrades its own readout. */}
          <NetworkSettings />
          <AppearanceSettings />
        </div>
      </div>
    </div>
  );
}
