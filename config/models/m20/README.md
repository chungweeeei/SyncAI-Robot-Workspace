# m20 — DEEP Robotics' Lynx M20

**Not runnable yet.** This directory holds no params file, so every launch
refuses `robot_model: m20` and names the file it is missing. That is the state
of the robot, not a gap in the mechanism: none of the values below has been
measured, and a placeholder here would be a number the stack drives on.

Add a file when its values are known — the keys `../model_keys.yaml` requires
for it at the least, and a block with no key (see `../g23/`) only for a node the
M20 really does not differ on.

| File | Needs first |
|---|---|
| `bringup.yaml` | `livox_lidar_publisher.host_ip`, if the M20's host is not on 192.168.1.103. `M20.urdf` has no `lidar_top` / `lidar_top_imu` / `camera_optical_frame` yet: the driver's cloud and Point-LIO's body frame are unparented until the mount is measured and those joints are added |
| `pointlio.yaml` | `lidar_min_range` (the blind radius round the body, 0.5 on the G23) and `satu_acc` (sized to the G23's footfall impacts; a wheeled gait's are not measured) |
| `localizer.yaml` | `lidar_height` |
| `planner_server.yaml` | `Octo3D`'s body: `lidar_height`, `robot_radius`, `body_clearance`, `max_step_height`, `max_slope_deg`, and the radii derived from the footprint; whether the M20 takes stairs at all (`stair_links`) |
| `controller_server.yaml` | `local_costmap.footprint`, the terrain layer's body values, the speed limits with the M20 driver's velocity scales, the gait floors (`walk_vx_min` and `in_place_wz_min` exist because the G23's RL gait stands still below them — a wheeled base may want 0), and the stair block: `stair_policy_mode: 1` is the G23 gait controller's HIMLOCO |
| `task_runner.yaml` | `lidar_height` |
| `fastdds.xml` | **in place** (2026-11): the M20 runs Fast DDS on the vendor's domain 0, where `/LIDAR/POINTS_UDP` (both lidars merged, per-point time) and `/IMU` arrive from the robot's other computers, on `lo` and the internal LAN (10.21.31.120) only, never Wi-Fi. Picked by `.env` (`SYNCAI_RMW` / `SYNCAI_DDS_CONFIG` / `SYNCAI_ROS_DOMAIN_ID`), not by `robot_model` (`../README.md`, "DDS"); the file's header has the measurements. Our `/tf` shares domain 0 with the vendor's: every frame of ours is `<robot_id>/`-prefixed but `map` |

Outside this directory, and not started: a driver for the M20's motion
interface (`syncai_driver_manager` speaks the G23 gait controller's UDP
protocol and names its 12 joints), `charge.xml`'s motion keys, and the
console's mesh (`scripts/urdf2glb.py` on `M20.urdf`).
