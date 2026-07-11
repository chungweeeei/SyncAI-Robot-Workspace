import { BatteryCard } from "@/components/dashboard/battery-card";
import { MapView } from "@/components/dashboard/map-view";
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
import { mockRobotState } from "@/lib/mock/robot-state";

export default function DashboardPage() {
  const state = mockRobotState;

  return (
    <>
      <PageHeader title="Dashboard" />
      <div className="grid gap-4 p-4 lg:grid-cols-3">
        <div className="grid content-start gap-4">
          <RobotStatusCard state={state} />
          <BatteryCard battery={state.battery_status} />
          <MotionCard localization={state.localization_status} />
          <NetworkCard network={state.network_status} />
        </div>
        <Card className="lg:col-span-2">
          <CardHeader>
            <CardTitle className="text-sm font-medium">Map</CardTitle>
            <CardDescription>
              {state.map} · live robot position
            </CardDescription>
          </CardHeader>
          <CardContent className="h-[560px] lg:h-full lg:min-h-[560px]">
            <MapView pose={state.localization_status.position} />
          </CardContent>
        </Card>
      </div>
    </>
  );
}
