# m20 — DEEP Robotics' Lynx M20

**Not runnable yet.** Only `syncai_driver_manager` and `syncai_bringup` have a
`params/models/m20.yaml` — bringup's (2026-11-24) runs `robot_state_publisher`
over `M20.urdf` and no Livox driver, which is all the M20 needs of it — so
every other launch refuses `robot_model: m20` and names the file it is
missing. That is the state
of the robot, not a gap in the mechanism: none of the values below has been
measured, and a placeholder here would be a number the stack drives on.

Add a package's `params/models/m20.yaml` when its values are known — the keys
`../model_keys.yaml` requires for it at the least, and a block with no key (see
that package's `g23.yaml`) only for a node the M20 really does not differ on.

| `params/models/m20.yaml` of | Needs first |
|---|---|
| `syncai_bringup` | **in place** (2026-11-24): `robot_state_publisher` with no difference, and no `livox_lidar_publisher` block, so the launch starts no Livox driver. The 16 leg / wheel joints reach `/tf` from `syncai_m20_joint_state` (2026-11-24), which bringup starts for the M20: the vendor's `/JOINTS_DATA` converted with its own offsets and directions. `M20.urdf`'s fixed joints: the two cameras (CA9B document), `lidar_link` (identity: `/LIDAR/POINTS_UDP` is stamped with it, the merged cloud already in the body frame) copied from the vendor's `/tf_static` on 2026-11-24, and `imu` (0.0632, −0.0268, 0.0435), DEEP Robotics' MJCF `imu_site` in `sdk_deploy`, unmeasured on the robot — not the G23's `lidar_top` / `lidar_top_imu`, which Point-LIO, `lio_bridge` and `robot_state` name today |
| `syncai_pointlio` | **`m20.yaml` in place** (2026-11-24): `lidar_type: 1` (the vendor's `PointCloud2`), `imu_acc_scale: 1.0` (its IMU reads m/s²), the extrinsic from DEEP Robotics' MJCF `imu_site` (unmeasured), `lidar_filter_num: 20`, `satu_acc: 39.0` above the 3.18 g measured driving on robot01 (no clip seen; not yet over stairs), and the G23's `lidar_min_range` as a first guess. Its inputs are `m20.yaml`'s `lidar_topic` / `imu_topic` (`/LIDAR/POINTS_UDP`, `/IMU`) (the node's lidar reader is best effort for every robot, since the cloud's writer is); it runs on robot01 (stationary: 97–99 % effective, 1.4 cm residual, < 1 cm drift in 25 s). Left: reading the cloud's per-point `timestamp` for motion compensation; the IMU's range from DEEP Robotics' spec, or a probe over stairs |
| `syncai_localizer` | `lidar_height` |
| `syncai_global_planner` | `Octo3D`'s body: `lidar_height`, `robot_radius`, `body_clearance`, `max_step_height`, `max_slope_deg`, and the radii derived from the footprint; whether the M20 takes stairs at all (`stair_links`) |
| `syncai_controller` | `local_costmap.footprint`, the terrain layer's body values, the speed limits with the M20 driver's velocity scales, the gait floors (`walk_vx_min` and `in_place_wz_min` exist because the G23's RL gait stands still below them — a wheeled base may want 0), and the stair block: `stair_policy_mode: 1` is the G23 gait controller's HIMLOCO |
| `syncai_task_runner` | `lidar_height` |
| (`fastdds.xml`, this directory) | **in place** (2026-11): the M20 runs Fast DDS on the vendor's domain 0, where `/LIDAR/POINTS_UDP` (both lidars merged, per-point time) and `/IMU` arrive from the robot's other computers, on `lo` and the internal LAN (10.21.31.120) only, never Wi-Fi. Picked by `.env` (`SYNCAI_RMW` / `SYNCAI_DDS_CONFIG` / `SYNCAI_ROS_DOMAIN_ID`), not by `robot_model` (`../README.md`, "DDS"); the file's header has the measurements. Our `/tf` shares domain 0 with the vendor's: every frame of ours is `<robot_id>/`-prefixed but `map` |

The M20's motion interface is started in `syncai_driver_manager` (branch
`feature/m20` of SyncAI-Robot-Driver-Manager, 2026-11): with `robot_model: m20`
its launch loads its own `params/models/m20.yaml` (`robot_protocol: m20`), and
the same `set_motion_key` "0" / "2" and `cmd_vel` stand, lie down and drive the
M20 over DEEP Robotics' UDP protocol (`10.21.31.103:30000`); every other key and
service answers `success: false`, and none of the G23 telemetry topics
(`imu`, `motor_states`, `battery_state`, `mode`) is published. Its
`m20_max_*` (cmd_vel → full-scale axis command) are unmeasured.

Not in any params file, and not started: `charge.xml`'s motion keys, and the
console's mesh (`scripts/urdf2glb.py` on `M20.urdf`).
