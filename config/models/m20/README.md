# m20 — DEEP Robotics' Lynx M20

**Starts, on first values; nothing has navigated yet** (2026-11-25). Every
package has its `params/models/m20.yaml`, so no launch refuses
`robot_model: m20` any more. What is in them is in three classes, and each
file says which a value is:

- **measured on robot01**: `lidar_height` 0.60, Point-LIO's sensor keys;
- **derived from `M20.urdf`** posed at robot01's standing joint reading (the
  body's sizes, below);
- **the operator's first figures or the G23's**, unmeasured on the M20: the
  speed limits, the accelerations, the step and slope rules, the gait floors.

The first test is **flat ground only**: the controllers' stair mode is off and
the planner lays no stair or gap link, because the M20's gait bridge has no
gait switch (`set_policy_mode` answers `success: false`).

## The body

`M20.urdf`'s collision geometry and meshes at the standing reading of robot01,
2026-11-24 (`syncai_m20_joint_state`'s `conversion.rs`, `STANDING`), in
`base_link`, the body's centre:

| | Extent | What sets it |
|---|---|---|
| Length | x −0.407 … +0.406 (shell ±0.41) | the wheels' rims and the shell's ends; DEEP Robotics' 820 mm |
| Width | y −0.236 … +0.218 | the legs' outer faces; the hips roll the wheels in under the body, so the stance sets it (0.254 a side with hipx at 0) |
| Height | shell top 0.085 above `base_link`, wheel bottoms 0.592 below | 0.68 by the URDF; 0.64–0.67 with `base_link` where the cloud measured it (0.55–0.58 above the floor) |

From it, with the padding the factory files use for each rule:

| Value | G23 | M20 | From |
|---|---|---|---|
| `local_costmap.footprint` (half-extents) | 0.35 × 0.22 | **0.41 × 0.24** | the extents |
| `robot_radius` ×2, `keepout.block_radius` | 0.25 | 0.25 | padded inscribed radius, 0.24 + 0.01 |
| `inflation_layer.inflation_radius` | 0.45 | **0.50** | ≥ padded circumscribed radius, 0.489 |
| `Octo3D.narrow_max_width` | 0.85 | **0.98** | padded diagonal, 0.84 × 0.50 |
| `Octo3D.stair_flights_turn_radius` | 0.455 | **0.52** | circumscribed radius at 0.03 padding (unused: no stairs) |
| `Octo3D.dynamic.block_radius` | 0.35 | **0.40** | the footprint's sideways reach at ±0.35 rad |
| `Octo3D.dynamic.self_radius` | 0.6 | **0.65** | the furthest wheel from the IMU (0.53) + 0.1; to adjust on the robot |
| `body_clearance` ×2 | 0.6 | **0.8** | the shell's top, 0.64–0.68, with more than a voxel to spare |
| `FollowPath.time_steps` | 32 | **26** | the horizon at `vx_max` 1.0 plus the 0.42 m nose inside the 4 m costmap |

A different stance height or hip roll changes all of it; nothing mounted on the
robot's back is in the URDF.

## Per package

| `params/models/m20.yaml` of | State |
|---|---|
| `syncai_bringup` | **in place** (2026-11-24): `robot_state_publisher` with no difference, and no `livox_lidar_publisher` block, so the launch starts no Livox driver. The 16 leg / wheel joints reach `/tf` from `syncai_m20_joint_state` (2026-11-24), which bringup starts for the M20: the vendor's `/JOINTS_DATA_10HZ` (its 10 Hz copy of `/JOINTS_DATA`, so the legs' TF is 10 Hz) converted with its own offsets and directions. `M20.urdf`'s fixed joints: the two cameras (CA9B document), `lidar_link` (identity: `/LIDAR/POINTS_UDP` is stamped with it, the merged cloud already in the body frame) copied from the vendor's `/tf_static` on 2026-11-24, and `imu` (0.0632, −0.0268, 0.0435), DEEP Robotics' MJCF `imu_site` in `sdk_deploy`, unmeasured on the robot — the link Point-LIO's body frame is, which `syncai_lio_bridge`'s `m20.yaml` names |
| `syncai_pointlio` | **in place** (2026-11-24): `lidar_type: 1` (the vendor's `PointCloud2`), `imu_acc_scale: 1.0` (its IMU reads m/s²), the extrinsic from DEEP Robotics' MJCF `imu_site` (unmeasured), `lidar_filter_num: 20`, `satu_acc: 39.0` above the 3.18 g measured driving on robot01 (no clip seen; not yet over stairs), and the G23's `lidar_min_range` as a first guess. Its inputs are `m20.yaml`'s `lidar_topic` / `imu_topic` (`/LIDAR/POINTS_UDP`, `/IMU`) (the node's lidar reader is best effort for every robot, since the cloud's writer is); it runs on robot01 (stationary: 97–99 % effective, 1.4 cm residual, < 1 cm drift in 25 s). The cloud's per-point `timestamp` is read since 2026-11-25 (`lidar_time_field`, `lidar_merge_holdback: 0.12`): the merged cloud's two lidars are often 100 ms apart in one message, and the stream is cut into scans by time, waiting only for the late lidar (`lidar_source_*`, by ring) (`syncai_pointlio`'s README, "A cloud of two lidars"; replayed offline on a bag, not yet run on the robot). Left: `lidar_filter_num` re-measured under the per-point update; the IMU's range from DEEP Robotics' spec, or a probe over stairs; and `lidar_min_range` checked **standing** — the wheels are then 0.53–0.71 m from the cloud's origin (centres at 0.62), past 0.5, so unless the vendor crops the robot from its cloud they are in `body_cloud` |
| `syncai_localizer` | **in place** (2026-11-25): `lidar_height` 0.60. The registration's tuning is the G23's, not replayed on M20 data |
| `syncai_global_planner` | **in place** (2026-11-25): `Octo3D`'s body from the table above, `max_step_height` 0.2 / `max_slope_deg` 20 (the operator's, unmeasured on the gait), and `stair_links: false` — no stair, detected stair or gap link. Plans only on a map the M20 made |
| `syncai_controller` | **in place** (2026-11-25): the footprint and the terrain layer's body values from the table; `vx_max` / `wz_max` 1.0 and RPP 3D's two (the operator's, as `cmd_vel` — see the driver's scales below); the G23's accelerations; `walk_vx_min` 0.25 / `in_place_wz_min` 0.4 (and RPP 3D's two), raised from 0.1 on 2026-11-25 after the M20 stood still at 0.1 m/s and did not turn at 0.14 rad/s in the first A ↔ B runs, then 17 runs with no stall — values that worked, the real floor unmeasured, and never 0, which also switches off MPPI's direct approach, approach slowdown and back-out; `slow_linear_vel` 0.30; `use_stair_mode: false` with `stair_policy_mode: -1`. The `Charging` blocks are the G23's |
| `syncai_task_runner` | **in place** (2026-11-25): `lidar_height` 0.60 |
| (`fastdds.xml`, this directory) | **in place** (2026-11): the M20 runs Fast DDS on the vendor's domain 0, where `/LIDAR/POINTS_UDP` (both lidars merged, per-point time) and `/IMU` arrive from the robot's other computers, on `lo` and the internal LAN (10.21.31.120) only, never Wi-Fi. Picked by `.env` (`SYNCAI_RMW` / `SYNCAI_DDS_CONFIG` / `SYNCAI_ROS_DOMAIN_ID`), not by `robot_model` (`../README.md`, "DDS"); the file's header has the measurements. Our `/tf` shares domain 0 with the vendor's: every frame of ours is `<robot_id>/`-prefixed but `map` — check that nothing of the vendor's publishes a `map` frame before the first localization |

Out of tree, each on its repo's `feature/m20`:

- **`syncai_driver_manager`** (2026-11): `robot_protocol: m20`; `set_motion_key`
  "0" / "2" and `cmd_vel` stand, lie down and drive the M20 over DEEP Robotics'
  UDP protocol (`10.21.31.103:30000`), every other key and service answers
  `success: false`, and of the G23's telemetry only `battery_state` is
  published (from the vendor's `/BATTERY_DATA`). **Its `max_vx` 2.0 / `max_wz`
  2.0 (the `cmd_vel` that maps to a full-scale axis command) are unmeasured**,
  so the controller's 1.0 m/s is half scale of an unknown top speed: hold a few
  `cmd_vel` plateaus against `/odom` and divide, as the G23's `scale_*` were
  calibrated, before trusting a speed limit.
- **`syncai_lio_bridge`** (2026-11-25): `lidar_frame: imu`, `imu_topic: /IMU`.
  Until then the bridge named the G23's `lidar_top_imu` and `livox/imu` for
  every robot and published no odometry on an M20.
- **`syncai_mapping`** (2026-11-24): `map_clean_lidar_height` 0.60 (measured),
  `map_clean_min_range` 0.5 (the G23's, a first guess; same standing check as
  Point-LIO's `lidar_min_range`).
- **`syncai_robot_state`** (2026-11-24): its input topics and `lidar_frame:
  lidar_link` (0.0435 m under the IMU that Point-LIO's body is, so the height
  it reports is that much off; `imu` is the exact link).

## Before the first run

1. Build the packages whose model file is new (a launch reads the installed
   share): `syncai_localizer`, `syncai_global_planner`, `syncai_controller`,
   `syncai_task_runner`, `syncai_lio_bridge`.
2. `[map] name:` a map the M20 made, with its `octomap.bt`.
3. `lio_bridge` first: its log must leave `waiting for <id>/base_link ->
   <id>/imu` and must not warn `no sample on imu_topic '/IMU'`; then
   `/<robot_id>/odom` against a teleop drive, which is also the calibration of
   the driver's `max_vx` / `max_wz`.
4. Then a short MOVE on level floor, watching `cmd_vel` against `odom`.

The pose the controller plans from is ~0.2 s old on this robot (the merged
cloud arrives ~133 ms after its stamp, and Point-LIO waits for the rear lidar),
0.2 m at 1.0 m/s. `lio_bridge` future-dates its TF, so nothing fails a lookup
for it; it shows as overshoot.

## Not per model yet

The nav session's windows are the G23's: `camera_feeder` has no
`/dev/syncai/camera0` to open on an M20 and respawns every 2 s, and
`landmark_detector` idles. `charge.xml`'s motion keys and dock geometry are
the G23's and must not be sent. The console's mesh
(`scripts/urdf2glb.py` on `M20.urdf`) is not built.
