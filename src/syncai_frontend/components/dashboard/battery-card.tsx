import { BatteryFullIcon, BatteryLowIcon } from "lucide-react";

import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Progress } from "@/components/ui/progress";
import { cn } from "@/lib/utils";
import type { RobotBatteryStatus } from "@/lib/types/robot";

export function BatteryCard({ battery }: { battery: RobotBatteryStatus }) {
  const pct = battery.battery_percentage;
  const low = pct < 20;
  const Icon = low ? BatteryLowIcon : BatteryFullIcon;

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2 text-sm font-medium">
          <Icon
            className={cn(
              "size-4",
              low ? "text-destructive" : "text-muted-foreground",
            )}
          />
          Battery
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <span
          className={cn(
            "text-3xl font-semibold tabular-nums",
            low && "text-destructive",
          )}
        >
          {pct}%
        </span>
        <Progress
          value={pct}
          className={cn(
            low
              ? "**:data-[slot=progress-indicator]:bg-destructive"
              : "**:data-[slot=progress-indicator]:bg-emerald-600",
          )}
        />
      </CardContent>
    </Card>
  );
}
