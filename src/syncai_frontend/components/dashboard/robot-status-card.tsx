import { BotIcon } from "lucide-react";

import { Badge } from "@/components/ui/badge";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import type { RobotMode, RobotState } from "@/lib/types/robot";

const modeBadgeClass: Record<RobotMode, string> = {
  AUTO: "bg-emerald-600 text-white dark:bg-emerald-500",
  MANUAL: "bg-amber-500 text-white dark:bg-amber-400 dark:text-black",
  MAINTENANCE: "bg-destructive/10 text-destructive dark:bg-destructive/20",
};

export function RobotStatusCard({ state }: { state: RobotState }) {
  const updatedAt = new Date(state.timestamp * 1000).toLocaleTimeString(
    "en-US",
  );

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2 text-sm font-medium">
          <BotIcon className="size-4 text-muted-foreground" />
          Robot Status
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center justify-between">
          <span className="text-lg font-semibold">{state.robot_id}</span>
          <Badge className={modeBadgeClass[state.mode]}>{state.mode}</Badge>
        </div>
        <dl className="space-y-1 text-sm">
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Map</dt>
            <dd className="font-medium">{state.map}</dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Last update</dt>
            <dd className="font-medium tabular-nums">{updatedAt}</dd>
          </div>
        </dl>
      </CardContent>
    </Card>
  );
}
