"use client";

import { RadarIcon } from "lucide-react";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { PointCloudView } from "@/components/dashboard/pointcloud-view";
import { TelemetryRail } from "@/components/dashboard/telemetry-rail";
import { cn } from "@/lib/utils";

/**
 * The operator screen: viewport left, instrument rail right, nothing above.
 *
 * The map used to be a 560 px card under a row of four stat cards, which meant
 * the one thing the screen exists to show was the smallest part of it and moved
 * whenever the cards reflowed. Here the viewport takes every pixel the strip and
 * the rail do not need, and the page never scrolls.
 */
export default function DashboardPage() {
  const { state, status } = useConsoleRobotState();

  if (!state) {
    return <AwaitingTelemetry connecting={status === "loading"} />;
  }

  // Below lg the rail stacks under the viewport and the whole screen scrolls;
  // from lg it is a side rail and nothing scrolls but the rail itself.
  return (
    <div className="flex h-full flex-col overflow-y-auto lg:flex-row lg:overflow-hidden">
      <section
        aria-label="Map viewport"
        // A definite height at every breakpoint: the canvas sizes itself from
        // the container, and a flex item sized only by min-height leaves its
        // h-full children resolving against nothing.
        className="relative h-[55vh] shrink-0 lg:h-full lg:flex-1"
      >
        {/* The pose the viewport draws does not come from here: the canvas has
          * its own ~20 Hz telemetry WebSocket. This page's 1 Hz state is what
          * the rail and the strip read. */}
        <PointCloudView robotId={state.robot_id} />
      </section>

      <aside
        aria-label="Telemetry"
        className="w-full shrink-0 border-t border-hairline bg-panel lg:h-full lg:w-72 lg:overflow-y-auto lg:border-t-0 lg:border-l"
      >
        <TelemetryRail state={state} />
      </aside>
    </div>
  );
}

/**
 * No-telemetry state. It names the endpoint and the node, because the fix is
 * always on the robot side — the console has nothing to retry.
 */
function AwaitingTelemetry({ connecting }: { connecting: boolean }) {
  return (
    <div className="flex h-full items-center justify-center p-8">
      <div className="max-w-sm text-center">
        <RadarIcon
          aria-hidden
          className={cn(
            "mx-auto size-8",
            // A permanently animating icon on the error state would read as
            // progress that is not happening.
            connecting
              ? "animate-pulse text-signal-cmd motion-reduce:animate-none"
              : "text-signal-caution",
          )}
        />
        <h1 className="instrument-label mt-4 text-muted-foreground">
          {connecting ? "Linking" : "No telemetry"}
        </h1>
        <p className="mt-2 text-sm text-foreground">
          {connecting
            ? "Waiting for the first state frame."
            : "The robot has not published a state frame yet."}
        </p>
        <p className="readout mt-3 text-xs text-muted-foreground">
          GET /api/v1/robot/state
        </p>
        {!connecting && (
          <p className="mt-1 text-xs text-muted-foreground">
            Check that syncai_robot_state is running on the robot.
          </p>
        )}
      </div>
    </div>
  );
}
