"use client";

import * as React from "react";

import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { PageHeader } from "@/components/page-header";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import type { MapMetadata, RobotPose } from "@/lib/types/robot";

/**
 * Backend-free preview of the G23 GLB inside the real 3D canvas.
 *
 * The dashboard gates every panel on GET /api/v1/robot/state, and the canvas
 * additionally keeps the robot hidden until the first pose arrives — so with no
 * robot (or no backend) on the network there is no way to look at the model at
 * all. This route drives PointCloudCanvas directly with a synthetic pose and no
 * live stream, which is enough to check the things that actually go wrong with
 * a baked asset: scale, up-axis, forward-axis, the base_link height offset, and
 * how the flat-shaded material reads under the scene lights in both themes.
 *
 * Deliberately not in the sidebar: it is a developer tool, not an operator
 * screen. Open /model-preview by hand.
 */

// A small synthetic map purely for framing. The canvas falls back to a 20 m
// span with no ground plane when `meta` is absent, which parks the camera far
// enough away that a 0.7 m robot is a few pixels tall. 4 m square centred on
// the origin frames it, and the (untextured) ground plane it brings along is
// itself useful — it is the reference that shows whether the feet sit on the
// floor rather than sinking through it.
const PREVIEW_MAP: MapMetadata = {
  resolution: 0.05,
  width: 80,
  height: 80,
  origin: [-2, -2, 0],
};

// Matches POSE_POLL_MS in pointcloud-view: the animated modes below fake the
// real feed's update rate, so what you see includes the pose easing exactly as
// the dashboard would apply it.
const POSE_TICK_MS = 500;

// Walk ticks faster than the body-pose modes: legs cycle at ~1 Hz, and a
// 500 ms sample of that is below Nyquist. 100 ms approximates the rate the
// telemetry WebSocket will deliver joint angles at.
const WALK_TICK_MS = 100;

const SPIN_DEG_PER_S = 45;
const CIRCLE_RADIUS_M = 1.5;
const CIRCLE_PERIOD_S = 12;

type Motion = "static" | "spin" | "circle" | "walk";

const MOTION_LABEL: Record<Motion, string> = {
  static: "Static",
  spin: "Spin in place",
  circle: "Drive a circle",
  walk: "Walk (trot)",
};

// Trot-gait parameters. Amplitudes sit comfortably inside the URDF limits
// (HipY [-2.67, 0.314], Knee [0.524, 2.792] — note the knee's zero pose is
// outside its own limits, so the swing is centred in-range, not on 0). The
// knee leads the hip by 90° so the foot traces a stepping loop rather than a
// straight pump.
const GAIT_FREQ_HZ = 1.0;
const HIP_Y_CENTER = -0.4;
const HIP_Y_AMP = 0.3;
const KNEE_CENTER = 0.9;
const KNEE_AMP = 0.35;

/**
 * Joint angles at time `t` for each mode. Non-walk modes return the baked
 * zero configuration (legs straight) so switching away from Walk resets the
 * legs instead of freezing them mid-stride.
 */
function jointsAt(motion: Motion, t: number): Record<string, number> {
  const out: Record<string, number> = {};
  const phase = 2 * Math.PI * GAIT_FREQ_HZ * t;
  for (const leg of ["FL", "FR", "HL", "HR"] as const) {
    // Trot: diagonal pairs move together (FL+HR vs FR+HL in antiphase).
    const p = phase + (leg === "FL" || leg === "HR" ? 0 : Math.PI);
    const walking = motion === "walk";
    out[`${leg}_HipX_joint`] = 0;
    out[`${leg}_HipY_joint`] = walking
      ? HIP_Y_CENTER + HIP_Y_AMP * Math.sin(p)
      : 0;
    out[`${leg}_Knee_joint`] = walking
      ? KNEE_CENTER + KNEE_AMP * Math.sin(p + Math.PI / 2)
      : 0;
  }
  return out;
}

/** Pose at time `t` (seconds since the motion started) for each mode. */
function poseAt(motion: Motion, t: number): RobotPose {
  // Walk keeps the body parked: legs-only motion makes phase/axis errors in
  // the joint rig readable without body movement confounding them.
  if (motion === "spin") {
    return { x: 0, y: 0, z: 0, theta: (t * SPIN_DEG_PER_S) % 360 };
  }
  if (motion === "circle") {
    const phase = (2 * Math.PI * t) / CIRCLE_PERIOD_S;
    return {
      x: CIRCLE_RADIUS_M * Math.cos(phase),
      y: CIRCLE_RADIUS_M * Math.sin(phase),
      z: 0,
      // Tangent to the circle, so the robot faces the way it travels — which is
      // what makes a wrong forward axis in the GLB obvious at a glance.
      theta: ((phase * 180) / Math.PI + 90) % 360,
    };
  }
  return { x: 0, y: 0, z: 0, theta: 0 };
}

export default function ModelPreviewPage() {
  const [motion, setMotion] = React.useState<Motion>("static");
  const [cameraMode, setCameraMode] = React.useState<"move" | "focus">("move");

  // The clock, not the pose, is the state: the pose is derived during render,
  // so switching motion takes effect immediately without a setState in the
  // effect body (which the react-hooks lint rule rejects, rightly — it would
  // cost a cascading render on every toggle). Both animated modes are periodic
  // in `t`, so a mode switch just enters mid-phase and the canvas eases into it.
  const [t, setT] = React.useState(0);
  React.useEffect(() => {
    if (motion === "static") return;
    const tick = motion === "walk" ? WALK_TICK_MS : POSE_TICK_MS;
    const id = setInterval(() => setT((prev) => prev + tick / 1000), tick);
    return () => clearInterval(id);
  }, [motion]);

  const pose = poseAt(motion, t);
  // Memoised so the canvas's joints effect only fires on a real tick, not on
  // every unrelated re-render (e.g. a camera-mode toggle).
  const joints = React.useMemo(() => jointsAt(motion, t), [motion, t]);

  return (
    <>
      <PageHeader title="Model preview" />
      <div className="flex flex-col gap-4 p-4">
        <Card>
          <CardHeader>
            <CardTitle className="text-sm font-medium">
              G23 model · public/models/g23.glb
            </CardTitle>
            <CardDescription>
              No robot required — synthetic pose, live point-cloud stream
              disabled.
            </CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-3">
            <div className="flex flex-wrap items-center gap-4">
              <div className="flex items-center gap-1">
                {(Object.keys(MOTION_LABEL) as Motion[]).map((m) => (
                  <Button
                    key={m}
                    size="sm"
                    variant={motion === m ? "default" : "outline"}
                    className="h-7 px-3 text-xs"
                    onClick={() => setMotion(m)}
                  >
                    {MOTION_LABEL[m]}
                  </Button>
                ))}
              </div>
              <div className="flex items-center gap-1">
                {(["move", "focus"] as const).map((mode) => (
                  <Button
                    key={mode}
                    size="sm"
                    variant={cameraMode === mode ? "default" : "outline"}
                    className="h-7 px-3 text-xs capitalize"
                    onClick={() => setCameraMode(mode)}
                  >
                    {mode}
                  </Button>
                ))}
              </div>
              <span className="font-mono text-xs text-muted-foreground">
                x {pose.x.toFixed(2)} · y {pose.y.toFixed(2)} · θ{" "}
                {pose.theta.toFixed(0)}°
              </span>
            </div>

            <div className="h-[560px] w-full overflow-hidden rounded-md border">
              <PointCloudCanvas
                meta={PREVIEW_MAP}
                pose={pose}
                joints={joints}
                cameraMode={cameraMode}
                liveStream={false}
              />
            </div>
          </CardContent>
        </Card>
      </div>
    </>
  );
}
