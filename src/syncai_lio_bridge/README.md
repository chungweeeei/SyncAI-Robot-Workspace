# syncai_lio_bridge

One node, `lio_bridge_node`, that turns the FAST-LIO2 / Point-LIO estimate into
the planar TF chain the nav stack expects. It is **the robot's only odometry
source and its localization provider** — it replaces wheel odometry *and* AMCL at
the same time.

```
  localizer            pointlio                      URDF (robot_state_publisher)
      │                    │                                    │
   map ──► <id>/pointlio_odom ──► <id>/pointlio_body      <id>/base_link ──► <id>/lidar_top
      (TF, after relocalize)   (TF + pointlio/lio_odom)          (static, from lidar_top_joint)
                                        │
                                        ▼
                               ┌─────────────────┐
                    livox/imu ─►   lio_bridge    │
                                └────────┬────────┘
                    ┌────────────────────┼────────────────────┐
                    ▼                    ▼                    ▼
          map ──► <id>/odom     <id>/odom ──► <id>/base_link   /<id>/odom
        (TF, AMCL replacement)        (TF, the odom chain)   (nav_msgs/Odometry)
```

Wheel odometry is gone — the Isaac Sim OmniGraph odom publishers are deliberately
disabled, and the real chassis publishes none. Nothing else in the stack
broadcasts `odom → base_link` or `map → odom`.

This is a C++ port of an earlier Python node; the numpy 4×4 homogeneous-matrix
math became `tf2::Transform` to drop the per-tick numpy allocation and
tf2-Python-binding cost.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `pointlio/lio_odom` | `nav_msgs/Odometry`, SensorData QoS | Point-LIO | Pose (`lio_odom → lio_body`) and body-frame linear velocity |
| `livox/imu` | `sensor_msgs/Imu`, SensorData QoS | Livox MID360 driver | Yaw rate |
| `base_frame → lidar_frame` | TF (static) | `syncai_bringup`'s URDF | The lidar mount extrinsic |
| `map_frame → <lio_odom frame>` | TF | `localizer`, **only after relocalize** | The map correction |

Both subscriptions use SensorData (best-effort) QoS, which is compatible with
either a reliable or a best-effort publisher.

The LIO odom frame is **not** a parameter: it is read from the incoming message's
`header.frame_id`. Point-LIO's `world_frame` is configured per-setup
(`pointlio_odom` on the Isaac config, `lio_odom` upstream), so taking it from the
header means a rename upstream needs no change here.

## Outputs

| Output | Type |
|---|---|
| `<odom_frame> → <base_frame>` | TF broadcast |
| `<map_frame> → <odom_frame>` | TF broadcast |
| `odom` | `nav_msgs/Odometry` |

All three are produced by a single timer at `publish_rate` (20 Hz by default),
independent of the ~10 Hz rate at which LIO actually updates — the timer
republishes the latest cached state.

### The math

**odom → base_link.** Point-LIO's "body" frame is physically the lidar, so the
lidar mount extrinsic maps it back to the robot base:

```
odom→base  =  (lio_odom→lio_body) · (base_link→lidar_top)⁻¹
```

then projected to 2D. The static extrinsic is looked up once and cached; until it
resolves, the node logs a throttled "waiting for base_link -> lidar_top" and
publishes nothing.

**map → odom.** The AMCL-style correction, computed exactly as AMCL does — the
map-frame pose minus the odom-frame pose:

```
map→base  =  (map→lio_odom) · (odom→base)
map→odom  =  P2D(map→base) · P2D(odom→base)⁻¹
```

Both operands are projected to 2D *before* the composition, so the result stays
planar and consistent with the `odom → base_link` that was just broadcast.

### Everything is projected to 2D

`project_2d()` keeps `x`, `y` and `yaw = atan2(R[1][0], R[0][0])`, and zeroes
`z`, roll and pitch. This happens before every broadcast so the planar nav stack
never sees a tilted frame — which matters here more than on a wheeled robot,
because the quadruped's body pitches and rolls with every gait cycle and the
lidar itself is mounted at a 0.25 rad tilt. A costmap fed a tilted `base_link`
would smear obstacles.

### Angular velocity comes from the IMU

Point-LIO leaves `twist.angular` empty, so `odom.twist.twist.angular.z` is taken
from the lidar IMU's gyro `z` instead. The IMU is co-located with the lidar and
the robot is treated as planar, so gyro z is the yaw rate. Linear x/y come from
LIO's own body-frame twist.

The twist is what consumers actually use: `syncai_controller` (via
`syncai_util::OdomSubscriber`), `syncai_task_runner`'s odom smoother, and
`syncai_robot_state`. None of them read the pose out of this topic — they take
pose from TF.

## Two-stage startup

The node deliberately does **not** wait for localization before publishing the
odom chain, mirroring AMCL, where `odom → base_link` exists before an initial
pose is given:

1. As soon as `pointlio/lio_odom` and the lidar extrinsic are available, it
   broadcasts `odom → base_link` and publishes the `odom` topic. The robot can be
   driven; the costmaps' `odom`-frame local costmap works.
2. `map → odom` additionally requires `map → <lio_odom frame>` from the
   localizer, which **only exists after `/localizer/relocalize` has been called**.
   Until then the timer logs a throttled "waiting for TF (relocalized yet?)" and
   returns early. The first successful correction logs
   `localization bridged: map -> <id>/odom = (x, y, yaw …)` exactly once — that
   line is the definitive "the robot is localized" signal.

This is why the byobu 3D session leaves the relocalize service call **pre-typed
but not executed**: you hit Enter once the robot is at its known start pose.

## Parameters

There is no params YAML; the launch file passes everything inline.

| Parameter | Default (node) | Launch value | Notes |
|---|---|---|---|
| `map_frame` | `map` | `map` | Never namespaced |
| `base_frame` | `base_link` | `<robot_id>/base_link` | |
| `odom_frame` | `odom` | `<robot_id>/odom` | |
| `lidar_frame` | `lidar_top` | `<robot_id>/lidar_top` | Must match the URDF's `lidar_top` link |
| `publish_rate` | `20.0` | `20.0` | Hz |
| `transform_tolerance` | `0.1` | `0.1` | Seconds the TF stamp is future-dated |
| `use_sim_time` | — | **`true`** | See the gotcha below |

`transform_tolerance` future-dates the broadcast stamp, the same trick AMCL uses:
consumers can interpolate up to that far ahead without hitting an extrapolation
error, which also bridges the gap between LIO's ~10 Hz updates and this node's
20 Hz republishing.

Frame names are given the `<robot_id>/` prefix explicitly by the launch file
because ROS namespaces topics but not TF frame ids; `map` stays plain.

## Running

```bash
ros2 launch syncai_lio_bridge lio_bridge.launch.py                       # use_sim_time:=true
ros2 launch syncai_lio_bridge lio_bridge.launch.py use_sim_time:=false   # real robot
ros2 launch syncai_lio_bridge lio_bridge.launch.py \
    system_config:=config/instances/robot02.ini
```

Window 2 of `scripts/byobu_session.sh`, after `sleep 4` — it needs the
localizer and `bringup`'s static TF up first.

Checking it:

```bash
ros2 topic hz /<robot_id>/odom
ros2 run tf2_ros tf2_echo map <robot_id>/odom          # zero-ish until relocalize
ros2 run tf2_ros tf2_echo <robot_id>/odom <robot_id>/base_link
ros2 topic echo /<robot_id>/pointlio/lio_odom --once   # is LIO producing at all?
```

The node's own log is the fastest diagnosis — each failure mode has a distinct
throttled message:

| Log line | Missing |
|---|---|
| `waiting for pointlio/lio_odom (LIO initializing?)` | Point-LIO is not publishing |
| `waiting for <id>/base_link -> <id>/lidar_top` | `bringup` / `robot_state_publisher` is not running |
| `waiting for TF (relocalized yet?)` | `/localizer/relocalize` has not been called |

## Gotchas

- **`use_sim_time` defaults to `true`.** On the real robot there is no `/clock`,
  so `now()` returns zero and every TF stamp and odom header is garbage. The
  byobu 3D session launches this node with no arguments, i.e. with the sim-time
  default — while the planner and controller 3D params set `use_sim_time: false`.
  Pass `use_sim_time:=false` explicitly on hardware.
- **`lidar_frame` must name a real frame.** It comes from the URDF's
  `lidar_top_joint`, published by `bringup`. Without it the node publishes
  *nothing at all* — not even the odom chain — because the extrinsic lookup gates
  the whole timer body.
- **The odom pose is a planar projection**, so `position.z` is always 0 and the
  orientation only has a yaw component. Do not use this topic as a 3D pose
  source; `pointlio/lio_odom` is the full 6-DOF estimate.
- **Odometry covariance is never populated** — the pose and twist covariance
  arrays stay all-zero. Anything doing proper uncertainty propagation would need
  them filled in.
- **The twist is copied from the LIO body frame into a message whose
  `child_frame_id` is `base_link`,** without rotating by the lidar mount
  extrinsic. With the 0.25 rad mount pitch this is roughly a 3% underestimate of
  forward speed — fine for the consumers listed above, but not exact.
- **LIO drift shows up as a moving `map → odom`.** That is the correction doing
  its job; a *jumping* correction instead points at the localizer accepting a bad
  scan match, not at this node.
- **Single-threaded spin.** The timer, both subscriptions and the TF listener all
  share the default executor, so a slow TF lookup delays the next publish tick.
  At 20 Hz with cached lookups this has not been a problem.
