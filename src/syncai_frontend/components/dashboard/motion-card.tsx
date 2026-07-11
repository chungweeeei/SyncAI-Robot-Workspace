import { MoveIcon } from "lucide-react";

import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import type { RobotLocalizationStatus } from "@/lib/types/robot";

export function MotionCard({
  localization,
}: {
  localization: RobotLocalizationStatus;
}) {
  const { position, velocity } = localization;

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2 text-sm font-medium">
          <MoveIcon className="size-4 text-muted-foreground" />
          Motion
        </CardTitle>
      </CardHeader>
      <CardContent>
        <dl className="space-y-1 text-sm">
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Position X</dt>
            <dd className="font-medium tabular-nums">
              {position.x.toFixed(2)} m
            </dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Position Y</dt>
            <dd className="font-medium tabular-nums">
              {position.y.toFixed(2)} m
            </dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Heading</dt>
            <dd className="font-medium tabular-nums">
              {position.theta.toFixed(1)}°
            </dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Velocity</dt>
            <dd className="font-medium tabular-nums">
              {velocity.toFixed(2)} m/s
            </dd>
          </div>
        </dl>
      </CardContent>
    </Card>
  );
}
