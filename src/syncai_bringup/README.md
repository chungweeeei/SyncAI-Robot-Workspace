# syncai_bringup

The sensing / TF layer that everything else in the navigation stack assumes is
already running. It contains **no source code** — an `ament_cmake` package whose
only job is to install launch files and the robot description, and whose real
content is two launch files:

| Launch file | Used by | Brings up |
|---|---|---|
| `bringup_2d.launch.py` | `scripts/byobu_session.sh` (AMCL path) | Two-lidar scan merger → a single 360° `/<robot_id>/scan`, plus the `base_link → scan` static TF |
| `bringup_3d.launch.py` | `scripts/byobu_session_3d.sh` (FAST-LIO2 path) | `robot_state_publisher` over the G23 URDF (which carries the lidar mount extrinsic) plus the Livox MID360 driver |

They are **not alternatives on the same hardware** so much as two different
sensing front-ends: 2D feeds AMCL and the costmaps from merged laser scans; 3D
feeds the LIO chain from the MID360 and publishes the body TF tree.

```
share/syncai_bringup/
  launch/       bringup_2d.launch.py, bringup_3d.launch.py
  description/  G23.urdf
  meshes/       body / hip / thigh / shank STLs (visual only)
  params/       (empty placeholder)
  maps/         (empty placeholder)
  rviz/         (empty placeholder)
```

`params/`, `maps/` and `rviz/` hold nothing but `.gitkeep`. Params live with the
package that owns them (`syncai_planner/params/…`), maps live in the repo-root
`map/` directory referenced from `config/instances/robotNN.ini`, and there is no
checked-in RViz config. The directories are installed anyway, so dropping a file
in works without touching `CMakeLists.txt`.

## robot_id

Both launch files use the same helper as every other launch file in the stack:
read `[system] robot_id` from `config/system.ini` (relative path — processes run
with the workspace root as cwd), falling back to `default_robot` with a warning.
Override with `system_config:=config/instances/robot02.ini`.

The resolved value is used two different ways, and the distinction matters:

- as a **node namespace**, so relative topics become `/<robot_id>/…`;
- as an explicit **TF frame prefix**, because ROS does *not* namespace frame ids.

The `static_transform_publisher` in `bringup_2d` deliberately runs with **no
namespace**, so it publishes to the global `/tf` shared with the odometry tree,
while the frame names it emits carry the `<robot_id>/` prefix. `bringup_3d`'s
`robot_state_publisher` does the same thing via the `frame_prefix` parameter.

## bringup_2d — merged laser scan

```
/<robot_id>/scan_front ─┐
                        ├─ ros2_laser_scan_merger ─► /<robot_id>/cloud_in (PointCloud2)
/<robot_id>/scan_rear  ─┘                                    │
                                    pointcloud_to_laserscan ◄─┘
                                                    └─► /<robot_id>/scan (LaserScan)

+ static TF  <robot_id>/base_link ──(0, 0, scan_height)──► <robot_id>/scan
```

It includes `ros2_laser_scan_merger`'s `merge_2_scan.launch.py` (the vendored,
locally-modified copy under `src/third-party/`), forwarding `robot_id` as its
`namespace` argument, and adds the static TF itself.

The merged cloud and scan both live in a dedicated `<robot_id>/scan` frame. The
per-lidar offsets in the merger's params are the in-plane diagonal mount
positions with `ZOff: 0` — the mount **height lives entirely in this static TF**,
which is why `scan_height` is a launch argument here rather than a merger
parameter. Its default `0.1225` is the sim geometry (`0.175 × 0.7`).

| Argument | Default | Meaning |
|---|---|---|
| `system_config` | `config/system.ini` | INI providing `[system] robot_id` |
| `scan_height` | `0.1225` | Lidar mount height (m) for `base_link → scan` |

## bringup_3d — body TF tree + MID360

Two nodes, both namespaced by `robot_id`:

**`robot_state_publisher`** parses `description/G23.urdf` and publishes the body
TF tree with `frame_prefix: <robot_id>/`. What actually reaches TF:

- The 4 `*_Ankle` fixed joints and **`lidar_top_joint`** go to `/tf_static`
  immediately.
- The 12 revolute leg joints only appear on `/tf` once something publishes
  `/joint_states`. **Nothing in this workspace does**, so today the legs are
  absent from TF. That is harmless — the planar nav stack only needs
  `base_link`, and LIO only needs `lidar_top`.

The `lidar_top` link is the reason this node matters to the 3D path:

```xml
<joint name="lidar_top_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="lidar_top"/>
    <origin xyz="0.195 0.00 0.155" rpy="0 0.25 0"/>   <!-- 0.25 rad = physical tilt -->
</joint>
```

`syncai_lio_bridge` looks up `<robot_id>/base_link → <robot_id>/lidar_top` and
uses it to map the Point-LIO body pose onto `base_link`. Without this TF the LIO
bridge produces no odometry and the whole 3D stack is dead — so `bringup_3d`
must be running before the bridge.

**`livox_ros_driver2_node`** drives the MID360. Its settings are Python constants
at the top of the launch file, mirrored from
`livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`:

| Setting | Value | Note |
|---|---|---|
| `xfer_format` | `1` | Livox **CustomMsg**, not `PointCloud2` — this is what the FAST-LIO2 chain expects |
| `multi_topic` | `0` | all lidars share one topic |
| `data_src` | `0` | live lidar |
| `publish_freq` | `10.0` Hz | |
| `frame_id` | `<robot_id>/laser` | prefixed explicitly, since frames aren't namespaced |
| `user_config_path` | `livox_ros_driver2/share/config/MID360_config.json` | **the IP / broadcast-code config lives there**, not here |

Note the frame naming: the driver publishes in `<robot_id>/laser`, while the
mount extrinsic in the URDF is `<robot_id>/lidar_top`. The LIO chain works in its
own `pointlio_odom → pointlio_body` branch and the bridge reconciles it through
`lidar_top`; `laser` is only the raw sensor frame.

| Argument | Default | Meaning |
|---|---|---|
| `system_config` | `config/system.ini` | INI providing `[system] robot_id` |
| `urdf_file` | `G23.urdf` | File name under `description/` |
| `use_sim_time` | `false` | Set true when driving from Isaac Sim |
| `lidar_height` | `0.196` | **Currently unused** — see below |

`lidar_height` is vestigial. It dates from when this launch file published
`base_link → lidar_top` with a `static_transform_publisher`; the extrinsic moved
into the URDF (where it can also carry the 0.25 rad pitch, which a
height-only argument could not). The argument is still declared, and the file's
header comment still describes the static TF node, but `launch_setup` returns
only `[robot_state_publisher, livox_driver]`. Changing `lidar_height` does
nothing — edit `lidar_top_joint` in the URDF instead.

`use_sim_time` as a launch argument is an exception to the workspace rule that
`use_sim_time` is only ever set in a params YAML. The rule exists because a
launch override silently beats the YAML value; this package has no params YAML
for these nodes, so there is nothing to conflict with.

## The URDF

`description/G23.urdf` is a SolidWorks URDF-exporter output for the G23 quadruped:
`base_link` root, four legs (`FL`/`FR`/`HL`/`HR` × HIP → THIGH → SHANK → FOOT),
plus the `lidar_top` frame. Two things about it are deliberate:

- **Capsule collisions were swapped to cylinders** so `urdfdom` can parse the
  file. This is a TF-only consumer, so collision fidelity does not matter here.
- **Mesh paths are relative** (`../meshes/….stl`). `robot_state_publisher` only
  walks the kinematic tree and never resolves geometry, so this is harmless for
  TF — but it means the URDF as-installed will not render in RViz without
  rewriting the paths to `package://syncai_bringup/meshes/…`.

The `*_Ankle` joints carry `dont_collapse="true"` to keep the foot frames from
being folded into the shank links.

## Running

Normally started by the byobu scripts as window 0, before everything else:

```bash
ros2 launch syncai_bringup bringup_2d.launch.py      # 2D / AMCL session
ros2 launch syncai_bringup bringup_3d.launch.py      # 3D / FAST-LIO2 session

ros2 launch syncai_bringup bringup_3d.launch.py \
    system_config:=config/instances/robot02.ini use_sim_time:=true
```

Check the result:

```bash
ros2 run tf2_tools view_frames                       # or:
ros2 run tf2_ros tf2_echo <robot_id>/base_link <robot_id>/lidar_top
ros2 topic hz /<robot_id>/scan                       # 2D path
ros2 topic hz /<robot_id>/livox/lidar                # 3D path
```

Build is just a resource install, so `colcon build --packages-select
syncai_bringup` is cheap — but with `--symlink-install` the URDF and launch files
are symlinked, and edits take effect on the next launch with no rebuild at all.

## Gotchas

- **Startup order.** There is no lifecycle manager. `bringup_3d` publishes the
  `lidar_top` static TF that `syncai_lio_bridge` needs, and `bringup_2d`
  publishes the `scan` TF the costmaps need. Both are window 0 in the byobu
  scripts for that reason; later windows sleep before launching.
- **A frame prefix is not a namespace.** Adding `namespace=` to the
  `static_transform_publisher` would push it onto `/<robot_id>/tf` and silently
  disconnect it from the global TF tree.
- **`lidar_height` does nothing** (above). The launch file's own header comment
  is stale on this point.
- **The MID360's IP and broadcast code are not here** — they live in
  `MID360_config.json` inside the `livox_ros_driver2` share directory. A lidar
  that never publishes is usually that file, not this launch file.
- **`xfer_format: 1` is required** by the FAST-LIO2 chain, which consumes Livox
  `CustomMsg`. Switching to `0` (`PointCloud2`) for a tool that wants standard
  messages will silently break LIO.
- **`bringup_2d` is not needed on the real 3D robot.** The 3D costmap params
  still list `scan` as an observation source, but that entry is a leftover for
  setups that publish one (Isaac Sim); on the real robot the obstacle layers run
  off `pointlio/body_cloud` and the scan source simply receives nothing. The
  header comment in `scripts/byobu_session_3d.sh` lists `bringup_2d` among the
  session's components, but the script does not launch it.
