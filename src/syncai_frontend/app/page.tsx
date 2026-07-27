"use client";

import { BatteryCard } from "@/components/dashboard/battery-card";
import { MapPanel } from "@/components/dashboard/map-panel";
import { MotionCard } from "@/components/dashboard/motion-card";
import { NetworkCard } from "@/components/dashboard/network-card";
import { RobotStatusCard } from "@/components/dashboard/robot-status-card";
import { PageHeader } from "@/components/page-header";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { useRobotState } from "@/hooks/use-robot-state";

export default function DashboardPage() {
  const { state, status } = useRobotState();

  return (
    <>
      <PageHeader title="Dashboard" />
      {state ? (
        <div className="flex flex-col gap-4 p-4">
          <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
            <RobotStatusCard state={state} />
            <BatteryCard battery={state.battery_status} />
            <MotionCard localization={state.localization_status} />
            <NetworkCard network={state.network_status} />
          </div>
          <Card>
            <CardHeader>
              <CardTitle className="text-sm font-medium">Map</CardTitle>
              <CardDescription>
                {state.map} · live robot position
              </CardDescription>
            </CardHeader>
            <CardContent className="h-[560px]">
              <MapPanel
                pose={state.localization_status.position}
                robotId={state.robot_id}
              />
            </CardContent>
          </Card>
        </div>
      ) : (
        <div className="p-4 text-sm text-muted-foreground">
          {status === "loading"
            ? "Connecting to robot…"
            : "Robot state unavailable. Waiting for the robot to publish…"}
        </div>
      )}
    </>
  );
}
