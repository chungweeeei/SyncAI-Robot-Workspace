"use client";

import { PageHeader } from "@/components/page-header";
import { AppearanceSettings } from "@/components/settings/appearance-settings";
import { NetworkSettings } from "@/components/settings/network-settings";
import { useRobotState } from "@/hooks/use-robot-state";

export default function SettingsPage() {
  const { state, status } = useRobotState();

  return (
    <>
      <PageHeader title="Settings" />
      <div className="mx-auto grid w-full max-w-2xl gap-4 p-4">
        {state ? (
          <NetworkSettings network={state.network_status} />
        ) : (
          <p className="text-sm text-muted-foreground">
            {status === "loading"
              ? "Connecting to robot…"
              : "Robot state unavailable. Waiting for the robot to publish…"}
          </p>
        )}
        <AppearanceSettings />
      </div>
    </>
  );
}
