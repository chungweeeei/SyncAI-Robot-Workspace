# config/models — what each robot model changes

A robot **model** (`g23`, `m20`) is named by the instance INI:

```ini
[system]
robot_id: robot01
robot_model: g23
```

`robot_id` says *which robot* this is (namespace, TF prefix, log directory);
`robot_model` says *what kind*: body size, gait limits, where the lidar sits. A
missing `robot_model` is read as `g23` with a warning, because every INI written
before 2026-11 has no such key and is a G23's.

## What a launch loads, in order

```
<package>/params/<node>_params.yaml          factory params
<package>/params/models/<robot_model>.yaml   the model's differences
config/params/<robot_id>/<node>.yaml         the robot's runtime overlay (gitignored,
                                             written by the backend's settings pages;
                                             controller_server and planner_server only)
{ frame overrides }                          set by the launch from robot_id
```

A later entry wins key by key, so a model file holds **only the keys that
differ** from the factory file, under the same `/**/<node>:` wildcard keys.

**A model's files are in the packages, not here** (since 2026-11-24; until then
they were `config/models/<model>/<node>.yaml`). Each sits beside the factory
file it lays over, the way `syncai_driver_manager` keeps its own: the package
owns its per-model values, and a key renamed in a factory file is renamed in
the model files in the same commit. This directory keeps what spans packages —
`model_keys.yaml`, `test_model_overlays.py`, this README — and the DDS files,
which are not per package.

| Package | Launch | Node keys |
|---|---|---|
| `syncai_bringup` | `bringup.launch.py` | `/**/robot_state_publisher`, `/**/livox_lidar_publisher` |
| `syncai_pointlio` | `pointlio.launch.py` | `/**/pointlio_node` |
| `syncai_localizer` | `localizer.launch.py` | `/**/localizer_node` |
| `syncai_global_planner` | `planner_server.launch.py` | `/**/planner_server`, `/**/global_costmap` |
| `syncai_controller` | `controller_server.launch.py` | `/**/controller_server`, `/**/local_costmap` |
| `syncai_task_runner` | `task_runner.launch.py` | `/**/task_runner` |

`bringup.launch.py` also picks the URDF by model (`g23` → `G23.urdf`, `m20` →
`M20.urdf`; `urdf_file:=` still overrides).

A launch reads the model's file from the package's **installed** share, so a
new or renamed model file needs a colcon build of its package before a launch
sees it (`--symlink-install` links each file, not the directory; an edit to a
file already installed needs none). `model_params_file:=` on any of the six
launches names one file directly, built or not. The old layout's lookup beside
the system INI went with it: an INI passed with `system_config:=` names a model
and the packages bring its files.

## Nothing falls back

Every model, the G23 included, has a file in all six packages, and a launch
**refuses to start** when the model's file for that node is missing, or when
the file does not parse. This is the opposite of the runtime overlay,
which is skipped with an error when it is unreadable: a bad settings-page write
falling back to the factory values is safe, while another robot coming up on
the G23's footprint, gait floors and stair policy looks healthy and is not.

A model that changes nothing for a node says so with a file that holds no key:

```yaml
/**/pointlio_node:
  ros__parameters: {}
```

The launch logs that and passes no file to the node (rcl is never handed an
empty block).

## The factory files are the G23's

The factory params files were tuned on the G23 and still carry its body and
gait values, which is why each package's `params/models/g23.yaml` holds no
key. That is an asymmetry, and it is written down in one place: `factory_model` in
`model_keys.yaml`. For every other model, a key its file leaves out is
inherited from the G23 without a word — so `model_keys.yaml` also lists the
keys a model **must** set (footprint, radii, lidar height, speed limits, gait
floors, stair policy), and `test_model_overlays.py` fails when a model's file
exists and leaves one out.

Moving the G23's own values out of the factory files into `g23.yaml` — so that
the factory files hold only what is the same on every robot — is a move of keys
between files, with nothing to change in a launch. `syncai_bringup` is the
first (2026-11-24): the Livox driver is the G23's lidar, so its whole
`/**/livox_lidar_publisher` block is in `g23.yaml` and the factory
`bringup.yaml` holds `robot_state_publisher` only. A node moved that way is
listed under `model_nodes` in `model_keys.yaml`: it has no factory block to
check against, so the test checks another model's keys for it against the
factory model's file, and another model may leave it out.

`syncai_pointlio` is the second, by key rather than by node (2026-11-24): the
seven that describe a robot's sensors (`lidar_type`, `imu_acc_scale`,
`lidar_min_range`, `satu_gyro`, `satu_acc`, `r_il`, `t_il`) are in each model's
file and not in the factory one. They are listed under `model_keys`, and the
rule is the strict one: **every** model's file sets every model key, the
factory model's included, because there is no factory value under them — a
key left out would run on the C++ default, not on the G23's.

## Values that change together

`test_model_overlays.py` checks the first three on the merged result (factory +
model file) of every model. The rest are rules the comments in the factory
files state and no code checks.

| Rule | Keys |
|---|---|
| One lidar height | `controller_server.lidar_height`, `local_costmap.terrain_layer.sensor_height`, `planner_server.Octo3D.lidar_height`, `localizer_node.lidar_height`, `task_runner.lidar_height` — and, outside this directory, `syncai_mapping`'s `map_clean_lidar_height` (its own `params/models/<robot_model>.yaml`: 0.481 G23, 0.60 M20), six C++ defaults and `scripts/map_repair/octomap_bt.py` |
| One robot radius | `Octo3D.robot_radius`, `terrain_layer.robot_radius`, `Octo3D.keepout.block_radius` (and the footprint's padded inscribed radius) |
| One stair policy | `FollowPath.*` and `FollowPathRPP3D.*`: `use_stair_mode`, `stair_policy_mode`, `flat_policy_mode` |
| Footprint | `local_costmap.footprint` ↔ `inflation_layer.inflation_radius` (≥ circumscribed radius), `Octo3D.narrow_max_width` (the diagonal), `Octo3D.stair_flights_turn_radius` (circumscribed radius), `Octo3D.dynamic.block_radius` |
| Surface rules | `Octo3D.{body_clearance,max_step_height,max_slope_deg}` = `terrain_layer.`'s |
| Speed calibration | `FollowPath.vx_max` / `wz_max` and `FollowPathRPP3D.desired_linear_vel` / `rotate_to_heading_angular_vel`, with `syncai_driver_manager`'s velocity scales |
| Gait floors | `FollowPath.walk_vx_min` = `slow_linear_vel` = `FollowPathRPP3D.min_approach_linear_velocity`; `in_place_wz_min` = `rotate_to_heading_min_angular_vel` |
| Charging | `charge_goal_checker` / `staging_goal_checker` wider than `Charging` / `ChargeBackOff` rest (`test_approach_law` reads the factory file only); `Charging.view_distance` = `charge.xml`'s `TrackPose offset_x` |

## What is not here

A params file cannot carry everything that differs between models. Today these
are still the G23's for every robot: `charge.xml`'s motion keys, the windows of
`config/sessions/*.yaml`, the C++ defaults behind the params, the devices in
`docker-compose.robots.yml`, and two out-of-tree packages' own params
(`syncai_camera_feeder`, `syncai_landmark_detection`) — each of those repos
takes the same `params/models/<robot_model>.yaml` layer in its package when it
needs one.

Four out-of-tree packages already have it, `params/models/g23.yaml` and
`m20.yaml` after their shared params file, refusing a model with no file as
these launches do:

- `syncai_driver_manager` (2026-11, the layout these six launches followed);
- `syncai_mapping` (2026-11-24, branch `feature/m20` of
  SyncAI-Robot-3D-Mapping): `map_clean_min_range` and `map_clean_lidar_height`,
  which its launch also refuses a model's file for leaving out. The M20's
  lidar height is 0.60 m, measured on robot01 (the floor 0.55–0.58 m below the
  vendor's merged cloud, the IMU 0.0435 m above it): the value the M20's nav
  params must carry as their `lidar_height` when they are written.
- `syncai_robot_state` (2026-11-24, branch `feature/m20` of SyncAI-Robot-State):
  its six input topics (an empty one is not subscribed — the M20's `mode`) and
  `lidar_frame` (G23 `lidar_top_imu`, M20 `lidar_link`). On the M20 only
  `wifi_status` and `safety_locked` have a publisher today; battery, joint and
  gait state are the vendor's drdds types, which it does not read.
- `syncai_lio_bridge` (2026-11-25, branch `feature/m20` of SyncAI-LIO-Bridge):
  `lidar_frame`, the URDF link of the IMU Point-LIO's body frame is (G23
  `lidar_top_imu`, M20 `imu`), and `imu_topic`, where its gyro is read for the
  odom topic's yaw rate (G23 `livox/imu`, M20 the vendor's absolute `/IMU`).
  Both were written into its launch and source for the G23; on an M20 the
  bridge waited for `base_link -> lidar_top_imu` for good and the stack had no
  odometry at all.

Being out of tree, none is in `model_keys.yaml`, and `test_model_overlays.py`
does not see them.

## DDS

`config/models/<model>/` is now only for DDS: a model's own `cyclonedds.xml`,
or a Fast DDS profile (`fastdds.xml`), and a README of what the model still
needs (`m20/README.md`). It is not picked by `robot_model`: the RMW's
environment is set by compose when the container is made, before anything
reads the INI, and the backend's and the recorder's containers need the same.
Three `.env` variables, set together:

| Variable | Default | |
|---|---|---|
| `SYNCAI_RMW` | `rmw_cyclonedds_cpp` | the RMW; `rmw_fastrtps_cpp` is in the image since 2026-11 |
| `SYNCAI_DDS_CONFIG` | `cyclonedds.xml` | the file, relative to `config/`; compose hands it to both `CYCLONEDDS_URI` and `FASTRTPS_DEFAULT_PROFILES_FILE`, and the RMW reads its own |
| `SYNCAI_ROS_DOMAIN_ID` | `1` | `ROS_DOMAIN_ID`; not `ROS_DOMAIN_ID` itself, which the Jetson host's shell exports as 69 and compose would prefer over `.env` |

The M20's:

```
SYNCAI_RMW=rmw_fastrtps_cpp
SYNCAI_DDS_CONFIG=models/m20/fastdds.xml
SYNCAI_ROS_DOMAIN_ID=0
```

Under Cyclone, `<Domain Id>` in the file and the domain variable are two
places that must agree; Fast DDS takes the domain from `ROS_DOMAIN_ID` alone.
The backend's compose names `config/cyclonedds.xml`, Cyclone and domain 1 by
fixed values — a host that sets these needs the same change there, or the
backend sees nothing.

## Adding a model

1. `params/models/<model>.yaml` in each of the six packages; start each from
   the keys `model_keys.yaml` requires for it, and rebuild those packages.
2. `<MODEL>.urdf` in `src/syncai_bringup/description/`, with `base_link` at
   the body's centre and a link for the IMU Point-LIO's body frame is, and its
   entry in `bringup.launch.py`'s `MODEL_URDF`. The links are named per model,
   not by convention (the G23's `lidar_top` / `lidar_top_imu` /
   `camera_optical_frame`, the M20's `lidar_link` / `imu`): name the IMU's in
   `syncai_lio_bridge`'s and `syncai_robot_state`'s
   `params/models/<model>.yaml` (`lidar_frame`), or the bridge publishes
   nothing.
3. `robot_model: <model>` in the robot's INI.
4. `python3 -m pytest config/models/test_model_overlays.py`.

A model's sizes are derived from its URDF, not copied from the G23's: the
footprint from the standing body's extents, and from it the inscribed radius
(`robot_radius`), the circumscribed one (`inflation_radius`,
`stair_flights_turn_radius`), the diagonal (`narrow_max_width`) and the
overlay's radii; `body_clearance` from the body's top above the ground.
`syncai_controller`'s `params/models/m20.yaml` has the M20's worked through.
