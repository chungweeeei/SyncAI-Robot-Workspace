# config/models — what each robot model changes

One directory per robot **model** (`g23`, `m20`), named by the instance INI:

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
<package>/params/<node>_params.yaml          factory params, tracked in the package
config/models/<robot_model>/<node>.yaml      this directory: the model's differences
config/params/<robot_id>/<node>.yaml         the robot's runtime overlay (gitignored,
                                             written by the backend's settings pages;
                                             controller_server and planner_server only)
{ frame overrides }                          set by the launch from robot_id
```

A later entry wins key by key, so a model file holds **only the keys that
differ** from the factory file, under the same `/**/<node>:` wildcard keys.

| File | Launch | Node keys |
|---|---|---|
| `bringup.yaml` | `syncai_bringup/bringup.launch.py` | `/**/robot_state_publisher`, `/**/livox_lidar_publisher` |
| `pointlio.yaml` | `syncai_pointlio/pointlio.launch.py` | `/**/pointlio_node` |
| `localizer.yaml` | `syncai_localizer/localizer.launch.py` | `/**/localizer_node` |
| `planner_server.yaml` | `syncai_global_planner/planner_server.launch.py` | `/**/planner_server`, `/**/global_costmap` |
| `controller_server.yaml` | `syncai_controller/controller_server.launch.py` | `/**/controller_server`, `/**/local_costmap` |
| `task_runner.yaml` | `syncai_task_runner/task_runner.launch.py` | `/**/task_runner` |

`bringup.launch.py` also picks the URDF by model (`g23` → `G23.urdf`, `m20` →
`M20.urdf`; `urdf_file:=` still overrides).

A launch looks for this directory beside the system INI, which is where it is
on a robot (`config/system.ini`), and otherwise in `~/robot_ws/config/models`:
an INI kept somewhere else and passed with `system_config:=` names a model
without having to carry the models with it. `model_params_file:=` on any of the
six launches names one file directly.

## Nothing falls back

Every model, the G23 included, has all six files, and a launch **refuses to
start** when the model's directory or its file for that node is missing, or
when the file does not parse. This is the opposite of the runtime overlay,
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
gait values, which is why `g23/` is six files with no key in them. That is an
asymmetry, and it is written down in one place: `factory_model` in
`model_keys.yaml`. For every other model, a key its file leaves out is
inherited from the G23 without a word — so `model_keys.yaml` also lists the
keys a model **must** set (footprint, radii, lidar height, speed limits, gait
floors, stair policy), and `test_model_overlays.py` fails when a model's file
exists and leaves one out.

Moving the G23's own values out of the factory files into `g23/` — so that the
factory files hold only what is the same on every robot — is the step after
this one; it is a move of keys between files, with nothing to change in a
launch.

## Values that change together

`test_model_overlays.py` checks the first three on the merged result (factory +
model file) of every model. The rest are rules the comments in the factory
files state and no code checks.

| Rule | Keys |
|---|---|
| One lidar height | `controller_server.lidar_height`, `local_costmap.terrain_layer.sensor_height`, `planner_server.Octo3D.lidar_height`, `localizer_node.lidar_height`, `task_runner.lidar_height` — and, outside this directory, `syncai_mapping`'s `map_clean_lidar_height`, six C++ defaults and `scripts/map_repair/octomap_bt.py` |
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
`docker-compose.robots.yml`, and the four out-of-tree packages' own params
(`syncai_driver_manager`, `syncai_robot_state`, `syncai_lio_bridge`,
`syncai_camera_feeder`) plus `syncai_mapping`'s — each of those repos takes the
same `config/models/<robot_model>/<node>.yaml` layer when it needs one.

## cyclonedds.xml

A model directory may hold its own `cyclonedds.xml`. It is not picked by
`robot_model`: `CYCLONEDDS_URI` is set by compose when the container is made,
before anything reads the INI, and the backend's and the recorder's containers
need the same file. `SYNCAI_DDS_CONFIG` in `.env` names it, relative to
`config/` (default `cyclonedds.xml`, the loopback-only file every robot ran on
until now):

```
SYNCAI_DDS_CONFIG=models/m20/cyclonedds.xml
```

`<Domain Id>` in the file and `ROS_DOMAIN_ID` in compose are still two places
that must agree, and the backend's compose names `config/cyclonedds.xml` by its
fixed path — a host that sets `SYNCAI_DDS_CONFIG` needs the same change there.

## Adding a model

1. `config/models/<model>/` with the six files; start each from the keys
   `model_keys.yaml` requires for it.
2. `<MODEL>.urdf` in `src/syncai_bringup/description/`, with `lidar_top`,
   `lidar_top_imu` and `camera_optical_frame`, and its entry in
   `bringup.launch.py`'s `MODEL_URDF`.
3. `robot_model: <model>` in the robot's INI.
4. `python3 -m pytest config/models/test_model_overlays.py`.
