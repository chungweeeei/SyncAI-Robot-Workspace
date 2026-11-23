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
| `cyclonedds.xml` | only if the M20's host needs other DDS wiring than loopback; picked by `SYNCAI_DDS_CONFIG` in `.env`, not by `robot_model` (`../README.md`) |

Outside this directory, and not started: a driver for the M20's motion
interface (`syncai_driver_manager` speaks the G23 gait controller's UDP
protocol and names its 12 joints), `charge.xml`'s motion keys, and the
console's mesh (`scripts/urdf2glb.py` on `M20.urdf`).
