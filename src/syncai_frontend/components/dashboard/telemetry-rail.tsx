"use client";

import {
  InstrumentGroup,
  PrimaryReadout,
  Readout,
  SignalBars,
} from "@/components/console/instrument";
import { LocomotionControl } from "@/components/dashboard/locomotion-control";
import { MotorStatus } from "@/components/dashboard/motor-status";
import { PostureControl } from "@/components/dashboard/posture-control";
import type { RobotState } from "@/lib/types/robot";

function rssiToBars(rssi: number): number {
  if (rssi >= -50) return 4;
  if (rssi >= -60) return 3;
  if (rssi >= -70) return 2;
  return 1;
}

/**
 * The instrument rail beside the viewport: pose, link, posture, locomotion, motors.
 *
 * These were four equal-weight cards above the map, which put the two numbers
 * an operator watches continuously (x/y and heading) at the same size as the
 * BSSID. Here the pose is the largest type on the screen after the robot id,
 * and everything else is a row.
 *
 * Battery, mode and map name are *not* here — they live in the status strip,
 * because they qualify the whole console rather than this screen.
 */
export function TelemetryRail({ state }: { state: RobotState }) {
  const { position, velocity } = state.localization_status;
  const network = state.network_status;
  // A false flag means the pose fields are a zeroed placeholder, not a
  // reading — a state frame arrives before the localizer converges (and all
  // through a mapping run) now that the backend no longer withholds it. The
  // numbers are masked rather than shown at 0.00: a dash cannot be misread as
  // the robot standing on the map origin.
  const localized = state.localization_valid;

  return (
    <div className="divide-y divide-hairline">
      <InstrumentGroup
        label="Pose"
        caption={localized ? undefined : "Not localized — no pose to report."}
      >
        <div className="mb-3 grid grid-cols-2 gap-3">
          <PrimaryReadout
            label="X"
            value={localized ? position.x.toFixed(2) : "—"}
            unit="m"
            tone={localized ? "live" : "neutral"}
          />
          <PrimaryReadout
            label="Y"
            value={localized ? position.y.toFixed(2) : "—"}
            unit="m"
            tone={localized ? "live" : "neutral"}
          />
        </div>
        <Readout
          label="Heading"
          value={localized ? position.theta.toFixed(1) : "—"}
          unit="°"
          tone={localized ? "live" : "neutral"}
        />
        <Readout
          label="Velocity"
          value={velocity.toFixed(2)}
          unit="m/s"
          tone="live"
        />
      </InstrumentGroup>

      <InstrumentGroup label="Link">
        <Readout label="SSID" value={network.ssid} />
        <Readout
          label="RSSI"
          value={
            <span className="inline-flex items-center gap-1.5">
              <SignalBars bars={rssiToBars(network.rssi)} />
              {network.rssi}
            </span>
          }
          unit="dBm"
        />
        <Readout label="IP" value={network.ip_address} />
        <Readout label="MAC" value={network.mac_address} />
        <Readout label="BSSID" value={network.bssid} />
      </InstrumentGroup>

      <PostureControl robotId={state.robot_id} />

      {/* Under Posture, because that is the order they are used in: the robot has
        * to be standing before a controller choice means anything. Posture is still
        * commanded-only; this one is half measured, which is why it reads the state
        * frame rather than holding everything locally. */}
      <LocomotionControl lowLevelMode={state.low_level_mode} />

      {/* Last: it is the longest group and the one an operator consults, rather
        * than watches. Pose and link stay above the fold on a short rail. */}
      <MotorStatus motors={state.motor_status} />
    </div>
  );
}
