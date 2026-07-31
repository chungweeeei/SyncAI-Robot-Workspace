# syncai_bringup

The sensing / TF layer that everything else in the navigation stack assumes is
already running. It contains **no source code** — an `ament_cmake` package whose
only job is to install the robot description, one launch file and its params:

| Launch file | Used by | Brings up |
|---|---|---|
| `bringup.launch.py` | `scripts/byobu_session.py` | `robot_state_publisher` over the G23 URDF (which carries the lidar mount extrinsic) plus the Livox MID360 driver |

```
share/syncai_bringup/
  launch/       bringup.launch.py
  params/       bringup.yaml
  description/  G23.urdf
  meshes/       body / hip / thigh / shank STLs (visual only)
```

There is deliberately no `maps/` or `rviz/` here. Both existed as empty
`.gitkeep` placeholders inherited from the `nav2_bringup` skeleton and were
removed: maps live in the repo-root `map/` directory referenced from
`config/instances/robotNN.ini` (a path resolved against the workspace cwd, not a
package share), and no RViz config is checked in.

## robot_id

The launch file uses the same helper as every other launch file in the stack:
read `[system] robot_id` from `config/system.ini` (relative path — processes run
with the workspace root as cwd), falling back to `default_robot` with a warning.
Override with `system_config:=config/instances/robot02.ini`.

The resolved value is used two different ways, and the distinction matters:

- as a **node namespace**, so relative topics become `/<robot_id>/…`;
- as an explicit **TF frame prefix**, because ROS does *not* namespace frame ids.

Both nodes here are namespaced, and `robot_state_publisher` additionally gets
`frame_prefix: <robot_id>/` so the frames it emits carry the prefix too.

## bringup — body TF tree + MID360

Two nodes, both namespaced by `robot_id`:

**`robot_state_publisher`** parses `description/G23.urdf` and publishes the body
TF tree with `frame_prefix: <robot_id>/`. What actually reaches TF:

- The 4 `*_Ankle` fixed joints and **`lidar_top_joint`** go to `/tf_static`
  immediately.
- The 12 revolute leg joints only appear on `/tf` once something publishes
  `/joint_states`, and **nothing in this workspace does** — so the legs are
  absent from TF. That is deliberate, not a gap: live joint angles already come
  up from the gait controller on `syncai_driver_manager`'s `motor_states`
  (`syncai_common/MotorStates`), and the consumer that needs them — the
  frontend's 3D robot model — subscribes to that directly rather than going
  through TF. Bridging `motor_states` into `/joint_states` would only duplicate
  the same data into a tree nothing reads: the planar nav stack only needs
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
bridge produces no odometry and the whole 3D stack is dead — so `bringup`
must be running before the bridge.

**`livox_ros_driver2_node`** drives the MID360. Its settings live in
`params/bringup.yaml`, mirrored from
`livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`:

| Setting | Value | Set in | Note |
|---|---|---|---|
| `xfer_format` | `1` | YAML | Livox **CustomMsg**, not `PointCloud2` — this is what the FAST-LIO2 chain expects |
| `multi_topic` | `0` | YAML | all lidars share one topic |
| `data_src` | `0` | YAML | live lidar |
| `publish_freq` | `10.0` Hz | YAML | |
| `frame_id` | `<robot_id>/laser` | launch | prefixed explicitly, since frames aren't namespaced; the YAML carries the unprefixed fallback |
| `user_config_path` | `/tmp/syncai_bringup/<robot_id>_MID360_config.json` | launch | **generated** — see below |

### The MID360 network config

The driver takes its network wiring from a JSON file rather than ROS params, and
that file needs two addresses. Both are per-deployment, so the launch file
renders the JSON itself (`write_livox_config`) instead of pointing at the vendor
`MID360_config.json` in the `livox_ros_driver2` share directory:

| Address | Read from | What it is |
|---|---|---|
| lidar IP → `lidar_configs[0].ip` | `[sensor.lidar] ip` in `config/system.ini` | the MID360's own address — per-robot hardware identity, so it sits next to `robot_id` in the instance INI |
| host IP → `host_net_info.*_ip` | `host_ip` under `/**/livox_lidar_publisher` in `params/bringup.yaml` | this machine's address on the lidar subnet; the driver **binds its receive sockets** to it |

Everything else in the rendered file (UDP ports, `lidar_type: 8`,
`pcl_data_type: 1`, `pattern_mode: 0`) is fixed by the MID360 protocol and lives
as constants in the launch file. `extrinsic_parameter` stays all-zero on
purpose — the mount extrinsic is the URDF's `lidar_top_joint`, and setting it
here too would apply the rotation twice.

Why not just edit the vendor JSON? `livox_ros_driver2` is an unmodified
submodule: an in-place edit is a local change that no submodule update survives,
and it hardcodes one lidar IP for a repo that runs several robots. The rendered
file is keyed by `robot_id` so two robots on one host cannot clobber each other,
and it goes under `/tmp` because it is derived state with no value once the
process exits.

Both reads **raise** rather than fall back — unlike `robot_id`, a wrong or
missing IP produces no error from the driver, just silence on the point cloud
topic, which is much more expensive to debug than a failed launch.

Note the frame naming: the driver publishes in `<robot_id>/laser`, while the
mount extrinsic in the URDF is `<robot_id>/lidar_top`. The LIO chain broadcasts
its own odom → body branch off to the side (the frame names come from the
localizer config — `lio_odom`/`lio_body` upstream — which is why
`syncai_lio_bridge` reads them off the message header instead of hardcoding
them), and the bridge reconciles that branch onto the robot tree through
`lidar_top`. `laser` is only the raw sensor frame; nothing parents it.

| Argument | Default | Meaning |
|---|---|---|
| `system_config` | `config/system.ini` | INI providing `[system] robot_id` and `[sensor.lidar] ip` |
| `params_file` | `<share>/syncai_bringup/params/bringup.yaml` | Node parameters for both nodes |
| `urdf_file` | `G23.urdf` | File name under `description/` |

To move the lidar, edit `lidar_top_joint` in the URDF. There used to be a
`lidar_height` argument feeding a `static_transform_publisher` here; it was
removed along with that node when the extrinsic moved into the URDF, which can
also carry the 0.25 rad pitch that a height-only argument could not.

## Parameters

`params/bringup.yaml` uses `/**/<node_name>:` wildcard keys, so the same file
works at any namespace. It covers the two node names the launch file assigns:
`robot_state_publisher` and `livox_lidar_publisher`.

Four values are **not** in the file, because they cannot be static. The launch
file appends them after `params_file` in each `Node`'s parameter list, so they
take precedence:

| Parameter | Why it is computed |
|---|---|
| `robot_description` | the URDF text, read off disk |
| `frame_prefix` | needs the resolved `robot_id` |
| `frame_id` | same — TF frame names are not namespaced, so it ships as `<robot_id>/laser` |
| `user_config_path` | points at the MID360 JSON the launch file generates per `robot_id` |

One value in the file is **not** a driver parameter at all: `host_ip`. The livox
node declares a fixed parameter set and never reads it (the override is simply
ignored), but the launch file reads it back out of this file to render the MID360
JSON. It lives here because that is where every other livox setting already is.

`use_sim_time` is set **only** in the YAML (`false` — there is no `/clock` on
the robot), per the workspace rule: a launch-level override placed after the
params file would silently win over the YAML value. There is deliberately no
`use_sim_time` launch argument for that reason.

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

Window 0 of `config/sessions/stack.yaml`, before everything else:

```bash
ros2 launch syncai_bringup bringup.launch.py

ros2 launch syncai_bringup bringup.launch.py \
    system_config:=config/instances/robot02.ini
```

Check the result:

```bash
ros2 run tf2_tools view_frames                       # or:
ros2 run tf2_ros tf2_echo <robot_id>/base_link <robot_id>/lidar_top
ros2 topic hz /<robot_id>/livox/lidar
```

Build is just a resource install, so `colcon build --packages-select
syncai_bringup` is cheap — but with `--symlink-install` the URDF and launch files
are symlinked, and edits take effect on the next launch with no rebuild at all.

## Gotchas

- **Startup order.** There is no lifecycle manager. This launch publishes the
  `lidar_top` static TF that `syncai_lio_bridge` needs, which is why it is
  window 0 in the byobu script; later windows sleep before launching.
- **A namespace does not namespace TF.** `tf2_ros` publishes on the absolute
  names `/tf` and `/tf_static` (hardcoded in its broadcasters), so putting
  `robot_state_publisher` in the `<robot_id>` namespace does *not* isolate its
  transforms — everything lands in one global tree. That is precisely why
  `frame_prefix` exists: without it, two robots on a shared DDS domain would
  both broadcast a frame called `base_link`. The namespace still matters for
  the node's *relative* names (here, `/<robot_id>/robot_description`).
- **A lidar that never publishes is almost always the two IPs.** Check the
  rendered config, which the launch logs on startup:
  `cat /tmp/syncai_bringup/<robot_id>_MID360_config.json`. The lidar IP comes
  from the instance INI, the host IP from `params/bringup.yaml`; a host IP that
  is not on an interface of this machine binds the sockets nowhere and the driver
  reports nothing. Editing the vendor `MID360_config.json` in the
  `livox_ros_driver2` share directory has **no effect** — it is not read.
- **`xfer_format: 1` is required** by the FAST-LIO2 chain, which consumes Livox
  `CustomMsg`. Switching to `0` (`PointCloud2`) for a tool that wants standard
  messages will silently break LIO.
- **Nothing publishes `<robot_id>/scan` any more.** The 3D planner/controller
  params still list `scan` as an obstacle observation source; it was already
  inert on the real robot (the obstacle layers run off `pointlio/body_cloud`),
  and now neither the topic nor its TF frame exists at all.
