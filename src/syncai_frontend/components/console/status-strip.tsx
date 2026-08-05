"use client";

import { ActiveTaskChip } from "@/components/console/active-task-chip";
import { useConsoleRobotState } from "@/components/console/robot-state-context";
import {
  Chip,
  SegmentMeter,
  SignalBars,
  StripDivider,
  type Tone,
} from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import type { RobotMode } from "@/lib/types/robot";

const MODE_TONE: Record<RobotMode, Tone> = {
  AUTO: "live",
  MANUAL: "caution",
  MAINTENANCE: "warn",
};

function rssiToBars(rssi: number): number {
  if (rssi >= -50) return 4;
  if (rssi >= -60) return 3;
  if (rssi >= -70) return 2;
  return 1;
}

function batteryTone(pct: number): Tone {
  if (pct < 20) return "warn";
  if (pct < 40) return "caution";
  return "live";
}

/** 24-hour clock with leading zeros — a readout, not prose. */
function clockOf(epochSeconds: number): string {
  return new Date(epochSeconds * 1000).toLocaleTimeString([], {
    hour12: false,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

/**
 * The console's masthead: which robot, what mode, and is the link alive. It is
 * mounted in the root layout, so it is present on every screen — including
 * before the first telemetry frame, where it holds the frame with dashes rather
 * than letting a bare "connecting…" sentence stand in for the whole UI.
 *
 * Deliberately no page title: the rail marks where you are, and repeating it
 * here would spend the widest row on the screen saying nothing measurable.
 *
 * The signature is the bottom edge. syncai_robot_state publishes at 1 Hz, so a
 * green highlight sweeps the hairline once per frame that lands; a frame that
 * does not arrive leaves the sweep unfinished and the edge goes amber. It is the
 * one animated thing in the console and it encodes the only fact that
 * invalidates everything else on screen. (Honoured `prefers-reduced-motion`
 * turns it into a static tinted edge — the STALE / NO SIGNAL chip carries the
 * same information in text.)
 */
export function StatusStrip() {
  const { state, status, updatedAt } = useConsoleRobotState();

  const link: { label: string; tone: Tone } =
    status === "ok"
      ? { label: "Link", tone: "live" }
      : status === "loading"
        ? { label: "Linking", tone: "neutral" }
        : state
          ? { label: "Stale", tone: "caution" }
          : { label: "No signal", tone: "warn" };

  const battery = state?.battery_status.battery_percentage;
  const network = state?.network_status;

  return (
    <header className="relative shrink-0 bg-panel">
      <div className="flex h-14 items-center gap-2.5 px-3 sm:gap-3.5 sm:px-4">
        <div className="flex min-w-0 items-center gap-2.5 sm:gap-3.5">
          <span className="readout truncate text-[15px] font-medium">
            {state?.robot_id ?? "—"}
          </span>
          {state && <Chip tone={MODE_TONE[state.mode]}>{state.mode}</Chip>}
        </div>

        <StripDivider className="hidden sm:block" />

        <div className="hidden min-w-0 items-baseline gap-2 sm:flex">
          <span className="instrument-label text-muted-foreground">Map</span>
          <span className="readout truncate text-[13px]">
            {state?.map ?? "—"}
          </span>
        </div>

        <StripDivider className="hidden sm:block" />

        {/* Whether the robot is executing anything, from Temporal rather than
          * from this tab — so it is true for a run another console started, or
          * one a schedule started with nobody watching, or one that began
          * before this page was loaded. `role="status"` so going from idle to
          * running is announced rather than just recoloured.
          *
          * Left cluster, beside the mode and the map: those three say what the
          * robot *is doing and where*, while the right cluster is the health of
          * the links and the machine. */}
        <div role="status" className="flex items-center gap-2">
          <span className="instrument-label hidden text-muted-foreground sm:inline">
            Task
          </span>
          <ActiveTaskChip />
        </div>

        <div className="ml-auto flex items-center gap-2.5 sm:gap-3.5">
          <div className="flex items-center gap-2">
            <Chip tone={link.tone}>{link.label}</Chip>
            {network && (
              <span className="hidden items-center gap-1.5 sm:flex">
                <SignalBars
                  bars={rssiToBars(network.rssi)}
                  tone={status === "ok" ? "live" : "caution"}
                />
                <span className="readout text-[13px] text-muted-foreground">
                  {network.rssi}
                  <span className="ml-0.5 text-[11px]">dBm</span>
                </span>
              </span>
            )}
          </div>

          <StripDivider />

          <div className="flex items-center gap-2">
            <span className="instrument-label hidden text-muted-foreground sm:inline">
              Batt
            </span>
            {battery === undefined ? (
              <span className="readout text-[13px] text-muted-foreground">—</span>
            ) : (
              <>
                <span
                  className={cn(
                    "readout text-[13px] font-medium",
                    batteryTone(battery) === "warn" && "text-signal-warn",
                    batteryTone(battery) === "caution" && "text-signal-caution",
                  )}
                >
                  {battery}
                  <span className="ml-0.5 text-[11px] text-muted-foreground">
                    %
                  </span>
                </span>
                <SegmentMeter value={battery} tone={batteryTone(battery)} />
              </>
            )}
          </div>

          <StripDivider className="hidden md:block" />

          <span className="readout hidden text-[13px] text-muted-foreground md:inline">
            {state ? clockOf(state.timestamp) : "--:--:--"}
          </span>
        </div>
      </div>

      {/* The strip's bottom edge is the heartbeat, so it replaces the border. */}
      <div
        aria-hidden
        className={cn(
          "relative h-px w-full overflow-hidden",
          status !== "error"
            ? "bg-hairline"
            : state
              ? "bg-signal-caution/50"
              : "bg-signal-warn/50",
        )}
      >
        {status === "ok" && (
          // Remounted on every frame (`key`) so the 1 s traverse restarts in
          // step with the arrival rather than free-running out of phase.
          <span
            key={updatedAt ?? 0}
            className="strip-sweep absolute inset-y-0 left-0 w-1/3 bg-gradient-to-r from-transparent via-signal-live to-transparent"
          />
        )}
      </div>
    </header>
  );
}
