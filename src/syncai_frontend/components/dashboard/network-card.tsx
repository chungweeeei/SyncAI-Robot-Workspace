import { WifiIcon } from "lucide-react";

import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { cn } from "@/lib/utils";
import type { RobotNetworkStatus } from "@/lib/types/robot";

function rssiToBars(rssi: number): number {
  if (rssi >= -50) return 4;
  if (rssi >= -60) return 3;
  if (rssi >= -70) return 2;
  return 1;
}

function SignalBars({ bars }: { bars: number }) {
  return (
    <span className="flex items-end gap-0.5" aria-label={`${bars} of 4 bars`}>
      {[1, 2, 3, 4].map((level) => (
        <span
          key={level}
          className={cn(
            "w-1 rounded-sm",
            level <= bars ? "bg-emerald-600" : "bg-muted",
          )}
          style={{ height: `${4 + level * 3}px` }}
        />
      ))}
    </span>
  );
}

export function NetworkCard({ network }: { network: RobotNetworkStatus }) {
  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2 text-sm font-medium">
          <WifiIcon className="size-4 text-muted-foreground" />
          Network
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center justify-between">
          <span className="text-lg font-semibold">{network.ssid}</span>
          <span className="flex items-center gap-2 text-sm text-muted-foreground">
            <SignalBars bars={rssiToBars(network.rssi)} />
            {network.rssi} dBm
          </span>
        </div>
        <dl className="space-y-1 text-sm">
          <div className="flex justify-between">
            <dt className="text-muted-foreground">IP address</dt>
            <dd className="font-medium tabular-nums">{network.ip_address}</dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">MAC address</dt>
            <dd className="font-medium tabular-nums">{network.mac_address}</dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">BSSID</dt>
            <dd className="font-medium tabular-nums">{network.bssid}</dd>
          </div>
        </dl>
      </CardContent>
    </Card>
  );
}
