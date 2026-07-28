"use client";

import {
  InstrumentGroup,
  PrimaryReadout,
  Readout,
  Segmented,
  SignalBars,
} from "@/components/console/instrument";
import type { RobotState } from "@/lib/types/robot";
import type { ViewMode } from "@/components/dashboard/map-panel";

function rssiToBars(rssi: number): number {
  if (rssi >= -50) return 4;
  if (rssi >= -60) return 3;
  if (rssi >= -70) return 2;
  return 1;
}

const VIEW_OPTIONS = [
  { value: "2d" as const, label: "2D grid" },
  { value: "3d" as const, label: "3D cloud" },
];

/**
 * The instrument rail beside the viewport: pose, link, view state.
 *
 * These were four equal-weight cards above the map, which put the two numbers
 * an operator watches continuously (x/y and heading) at the same size as the
 * BSSID. Here the pose is the largest type on the screen after the robot id,
 * and everything else is a row.
 *
 * Battery, mode and map name are *not* here — they live in the status strip,
 * because they qualify the whole console rather than this screen.
 */
export function TelemetryRail({
  state,
  viewMode,
  onViewModeChange,
}: {
  state: RobotState;
  viewMode: ViewMode;
  onViewModeChange: (mode: ViewMode) => void;
}) {
  const { position, velocity } = state.localization_status;
  const network = state.network_status;

  return (
    <div className="divide-y divide-hairline">
      <InstrumentGroup
        label="Pose"
        caption="map frame, from FAST-LIO2 via the LIO bridge"
      >
        <div className="mb-3 grid grid-cols-2 gap-3">
          <PrimaryReadout
            label="X"
            value={position.x.toFixed(2)}
            unit="m"
          />
          <PrimaryReadout
            label="Y"
            value={position.y.toFixed(2)}
            unit="m"
          />
        </div>
        <Readout
          label="Heading"
          value={position.theta.toFixed(1)}
          unit="°"
          tone="live"
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

      <InstrumentGroup
        label="View"
        caption="3D streams the live point cloud over WebGL; 2D is the saved occupancy grid."
      >
        <Segmented
          value={viewMode}
          options={VIEW_OPTIONS}
          onChange={onViewModeChange}
          className="w-full **:flex-1"
        />
      </InstrumentGroup>
    </div>
  );
}
