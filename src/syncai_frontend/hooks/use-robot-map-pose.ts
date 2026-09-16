"use client";

import * as React from "react";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { useActiveMap } from "@/hooks/use-maps";
import type { PlanarPose } from "@/lib/types/robot";

export interface UseRobotMapPose {
  /** Where the robot is standing on *this* map, or null when that is unknowable. */
  pose: PlanarPose | null;
  /** Why `pose` is null, as a sentence for the operator. Null when it is set. */
  reason: string | null;
}

/**
 * The robot's own pose, but only when it is a pose on the map named here.
 *
 * A vertex is map-frame metres, and the robot's pose is map-frame metres of
 * whichever map the running stack loaded — the two are the same numbers on two
 * different floors unless the names match. Every "no" this returns carries the
 * sentence that explains it, because the only place the answer is used is a
 * disabled button, and "greyed out with no reason" is the failure mode worth
 * spending a string on.
 *
 * Reads the console's shared 1 Hz poll rather than the telemetry socket, for the
 * `localization_valid` flag: the socket carries pose frames with no way to say
 * they stopped being true, so a dead localizer (or a mapping run, where the TF
 * chain never reaches base_link) would leave the last frame on screen looking
 * exactly like a live one. 1 Hz is also plenty — the robot is parked at the spot
 * being marked. Same reason as useModeSwitch for using the context: one poll for
 * the console.
 */
export function useRobotMapPose(name: string): UseRobotMapPose {
  const { state } = useConsoleRobotState();
  const { map: activeMap, status: mapsStatus } = useActiveMap();

  const position = state?.localization_status.position;
  // Read off as primitives, never as `state`: the payload's timestamp moves every
  // second, so keying the memo on the object would hand the editor a fresh pose
  // — and re-render it — once a second while the robot stands still.
  const hasState = state !== null;
  const valid = state?.localization_valid ?? false;
  const x = position?.x ?? 0;
  const y = position?.y ?? 0;
  const theta = position?.theta ?? 0;

  const activeName = activeMap?.name ?? null;

  return React.useMemo(() => {
    if (mapsStatus === "loading") {
      return { pose: null, reason: "Checking which map the robot is on." };
    }
    if (activeName === null) {
      return { pose: null, reason: "The robot has no map loaded." };
    }
    if (activeName !== name) {
      return {
        pose: null,
        reason: `The robot is running on "${activeName}", not this map.`,
      };
    }
    if (!hasState) {
      return { pose: null, reason: "No robot state yet." };
    }
    if (!valid) {
      // The pose fields are a zeroed placeholder in this case, not a stale
      // reading — see RobotState.localization_valid.
      return { pose: null, reason: "The robot is not localized yet." };
    }
    return { pose: { x, y, theta }, reason: null };
  }, [mapsStatus, activeName, name, hasState, valid, x, y, theta]);
}
