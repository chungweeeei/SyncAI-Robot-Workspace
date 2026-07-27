import * as THREE from "three";

// The 12 actuated joints of the G23, transcribed from
// src/syncai_bringup/description/G23.urdf. Each revolute joint rotates its
// child link about `axis` (URDF axes are unit ±x/±y here, no rpy on any joint
// origin), and the GLB baked by scripts/urdf2glb.py keeps node names equal to
// URDF link names — so animating a joint is: look the child link up by name,
// setFromAxisAngle(axis, q). The baked pose is the zero configuration, so q is
// applied absolutely, with no rest offset to add.
//
// Joint names match what syncai_driver_manager publishes in MotorState.name
// (kJointNames in syncai_driver_manager.cpp), which is the wire vocabulary the
// telemetry WebSocket will carry. The Ankle joints are fixed and never move.

export interface JointSpec {
  /** URDF child-link name == GLB node name to rotate. */
  childLink: string;
  /** Rotation axis in the child's parent frame (URDF <axis>). */
  axis: THREE.Vector3;
  /** URDF position limits [lower, upper] in radians, for clamping. */
  limits: [number, number];
}

const LEGS = ["FL", "FR", "HL", "HR"] as const;

// Per-leg joint template. All four legs are identical in axis and limits; only
// the link-name prefix differs.
const LEG_JOINTS: ReadonlyArray<{
  suffix: string;
  link: string;
  axis: [number, number, number];
  limits: [number, number];
}> = [
  { suffix: "HipX_joint", link: "HIP", axis: [-1, 0, 0], limits: [-0.523, 0.523] },
  { suffix: "HipY_joint", link: "THIGH", axis: [0, -1, 0], limits: [-2.67, 0.314] },
  { suffix: "Knee_joint", link: "SHANK", axis: [0, -1, 0], limits: [0.524, 2.792] },
];

/** joint name (e.g. "FL_Knee_joint") -> how to drive it on the GLB. */
export const G23_JOINTS: ReadonlyMap<string, JointSpec> = new Map(
  LEGS.flatMap((leg) =>
    LEG_JOINTS.map(({ suffix, link, axis, limits }): [string, JointSpec] => [
      `${leg}_${suffix}`,
      {
        childLink: `${leg}_${link}`,
        axis: new THREE.Vector3(...axis),
        limits,
      },
    ]),
  ),
);
