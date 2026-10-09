# syncai_pointlio

The Point-LIO front end: point-by-point lidar-inertial odometry over the Livox
MID360's `CustomMsg` + IMU, in the "output model" (the IMU is a measurement,
body angular velocity and specific force are random-walk states, every lidar
point is processed at its own timestamp, so there is no scan-undistortion
step). It is the **only** source of the LIO odometry, the body-frame scan and
the `pointlio_odom → pointlio_body` TF that everything downstream consumes.

```
          /<id>/livox/lidar  (CustomMsg)      /<id>/livox/imu
                        │                           │
                        └──────────┬────────────────┘
                                   ▼
                              pointlio_node          /<id>/pointlio/…
                       TF  <id>/pointlio_odom ──► <id>/pointlio_body
                                   │
          ┌───────────────┬────────┴────────┬──────────────────┐
          ▼               ▼                 ▼                  ▼
   pgo_node (mapping) localizer_node   syncai_lio_bridge   costmaps / backend
   lio_odom+body_cloud lio_odom+body_cloud   lio_odom         body_cloud
   calls `reset`      map→pointlio_odom   → odom/base_link   obstacle layers,
                                                             live cloud WS
```

Ported into the workspace from `SyncAI-Fast-LIO2`'s `pointlio` package in
2026-09, the first of its nodes to move (`pgo` and `hba` followed as
`syncai_mapping`, then the `localizer` as `syncai_localizer`, and the fork is
no longer imported at all). The ROS surface did not change with the move — same executable
and node name, namespace, topics, service name and parameters — so no consumer
had to. `map_builder/` is the ported math (`point_ekf`, `imu_initializer`,
`lidar_processor`, `ikd_Tree`, refactored from HKU MaRS Point-LIO in the
`liangheming/FASTLIO2_ROS2` style, MIT); `pointlio_node.*` is the ROS shell.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `lidar` → `/<robot_id>/livox/lidar` | `livox_ros_driver2/CustomMsg` (`lidar_type: 0`) or `sensor_msgs/PointCloud2` (`lidar_type: 1`), depth 10 | `syncai_bringup`'s Livox driver | The scan. `CustomMsg` carries per-point time in `curvature`; a `PointCloud2` does not, so the update degrades to one batch per scan (warned once). |
| `imu` → `/<robot_id>/livox/imu` | `sensor_msgs/Imu`, depth 10 | the lidar's built-in IMU | Fused as a **measurement** (output model). Accelerations are multiplied by `imu_acc_scale` (Livox reports g-units). |

The two topics are **remapped by the launch file**, not parameters: the node
subscribes to the relative names `lidar` / `imu`, which would otherwise resolve
inside its own `/<robot_id>/pointlio` namespace. `ros2 node info` shows the
resolved names, and the node logs them at startup so a wrong remap shows up as
a wrong name rather than as "no data".

## Outputs

All relative to `/<robot_id>/pointlio/`, depth 10000, stamped with the lidar
clock (`cloud_end_time`, the last point's time). The topic publishers are gated
on having a subscriber; the TF is not.

| Output | Type | Notes |
|---|---|---|
| `lio_odom` | `nav_msgs/Odometry` | `world_frame → body_frame`, 6-DOF. Body-frame linear velocity and the output model's own angular-velocity estimate in `twist` (see the `syncai_lio_bridge` README for why the bridge uses the gyro instead). |
| `body_cloud` | `sensor_msgs/PointCloud2` in `body_frame` | The scan (every `lidar_filter_num`-th point), **motion-compensated into the body frame at the end-of-scan stamp** — the frame and stamp `lio_odom` carries (see "Deskew" below). What the `localizer`, both costmaps' obstacle layers and the backend's live-cloud WebSocket read. |
| `body_cloud_dense` | `sensor_msgs/PointCloud2` in `body_frame`, depth 10 | The same deskewed scan decimated by `dense_filter_num` (3× the points at the defaults), x / y / z / intensity only (16 B a point, ~0.12 MB a scan). For mapping: `pgo` (`syncai_mapping`) keyframes are taken from it. Never seen by the filter, so the LIO solution does not depend on it; built in the same ingest pass and published only while subscribed. |
| `world_cloud` | `sensor_msgs/PointCloud2` in `world_frame` | Display only: the deskewed `body_cloud` moved with the end-of-scan pose. |
| `lio_path` | `nav_msgs/Path` | Accumulated trajectory for rviz. Published empty, ungated, on a reset so rviz drops the old run. |
| TF `world_frame → body_frame` | `<robot_id>/pointlio_odom → <robot_id>/pointlio_body` | Broadcast from the timer once the builder reaches `MAPPING`. |
| `reset` | `syncai_common/srv/ResetLIO` | Start over in place — see below. |

Nothing is published until the builder reaches `MAPPING` (a static IMU
initialisation over `imu_init_num` samples, then the first map frame), so
silence right after launch is normal.

### The frames

`body_frame` is **physically the lidar**, not the robot base: the extrinsic the
filter estimates (`r_il` / `t_il`) is lidar → IMU, and the IMU sits in the
lidar housing. `syncai_lio_bridge` relies on exactly that when it applies the
`base_link → lidar_top` mount transform from the URDF.

The frame names are deliberately not `base_link` / `laser`, which they once
were. `base_link` already has a parent from `syncai_lio_bridge`
(`odom → base_link`), and tf2 keys its cache by **child** frame, so a second
broadcaster for the same child interleaves samples into one cache and the
parent you get back depends on the lookup time: latest (`Time()`) hit the
bridge's 20 Hz samples, the cloud's own stamp hit pointlio's 10 Hz ones. That
silently sent the backend's `body_cloud` transform through the bridge's
2D-projected chain (z = 0, roll = pitch = 0) while rviz2, which looks up at the
message stamp, went through the full 6-DOF chain — a ~15° pitch disagreement
between rviz2 and the operator UI. The launch file's frame overrides are where
this is enforced; the YAML values (`lidar` / `body`) are only fallbacks for
running the node bare.

### Deskew

Point-LIO itself needs no undistortion pass — every point group is projected
with the state of its own instant and updates it (`map_builder.cpp`). The
*published* clouds used to skip that: `body_cloud` was the raw scan put through
the lidar → IMU extrinsic only, so every point carried the robot's motion
during the 0.1 s scan. A keyframe taken while turning smeared each wall into a
fan; on `dp1f_1008_2` turning keyframes had twice the wall thickness of
straight ones (5–95 % width 0.56 m vs 0.22 m), which became double walls in
`map.pcd` and occupied noise beside every wall in the OctoMap.

Since 2026-10-09 `MapBuilder` records the post-update state after every group
of a MAPPING frame (plus the end state), and `deskewToEndBody` places each
published point with the state of the last group at or before its time, then
expresses it in the body frame at `cloud_end_time`. The group times are
VoxelGrid means of the per-point offsets, not a subset of them, hence "at or
before" rather than an exact match; the error is a millisecond of motion. The
MAP_INIT frame has no groups and is published with the end state alone, as
before. Each deskew runs only when its output has a subscriber.

### The reset contract

`reset` throws the ikd-tree map, the EKF and the sensor buffers away and
re-runs the static IMU initialisation, **without restarting the process**.
Nobody calls it directly in normal operation: `pgo_node`'s `reset_mapping`
(the console's "New map") owns the ordering — pause intake → call `reset` →
rebuild the pose graph → resume, dropping every cloud/odom pair with a stamp at
or before the `last_odom_time` this service returns. That one field is the
whole contract: it is the lidar stamp of the last odometry sample of the old
run, recorded in the timer past the `MAPPING` gate (not inside the
subscriber-gated publish, so it cannot depend on who is listening), and the
same clock domain as `lio_odom`'s header, so the consumer compares exactly.

Two things the code relies on and enforces in comments rather than locks:

- **The robot must be standing still.** The re-initialisation averages
  `imu_init_num` samples and aligns gravity from them; one done in motion
  yields a permanently tilted map and reports no error, because nothing
  downstream can tell a bad gravity vector from a good one. Deliberately not
  gated on a stationarity check — the node cannot tell a still robot from a
  gently vibrating one, and refusing a robot that *is* still is worse.
- **`resetCB` takes no lock against `timerCB`.** `main.cpp` spins the node
  with single-threaded `rclcpp::spin()`, so every callback shares one
  MutuallyExclusive group and the timer cannot be halfway through
  `MapBuilder::process()` while the builder is swapped. Moving to a
  `MultiThreadedExecutor`, or giving the service its own callback group, needs
  a mutex around `m_builder` / `m_kf` first.

The request is empty on purpose: a reset is not a reconfigure, and the
parameters / extrinsics stay as launched. The comments in
`syncai_common/srv/ResetLIO.srv` and `ResetMapping.srv` (both in
`SyncAI-Robot-Interface` since the pgo port) are the specification; the client
is `syncai_mapping`'s `pgo_node`.

## Parameters

`params/pointlio_params.yaml`, keyed `/**/pointlio_node:` so it applies at any
namespace. Every key is a `declare_parameter` whose default repeats the struct
default, so a missing key degrades to the code default instead of throwing.
The launch layers the two frame names on top.

| Parameter | Default (YAML) | Launch value | Notes |
|---|---|---|---|
| `world_frame` | `lidar` | `<robot_id>/pointlio_odom` | |
| `body_frame` | `body` | `<robot_id>/pointlio_body` | |
| `lidar_type` | `0` | | `0` Livox `CustomMsg`, `1` `PointCloud2` (kept for the Isaac Sim path; no per-point time) |
| `imu_acc_scale` | `10.0` | | Livox IMU accel is in g → m/s². Sensors already in m/s² need `1.0`; the filter's gravity is fixed at 9.81. `satu_acc` is compared **after** scaling. |
| `print_time_cost` | `false` | | Logs the per-frame `process()` time as a WARN |
| `lidar_filter_num` | `6` | | Keep every n-th point: the filter's input and `body_cloud` |
| `dense_filter_num` | `2` | | Keep every n-th point for `body_cloud_dense` only. A divisor of `lidar_filter_num` makes `body_cloud`'s points a subset (warned otherwise). 2 is the floor: at 1 a MID360 scan is ~320 KB, over CycloneDDS's default 208 KB receive buffer on `lo`. `<= 0` or `>= lidar_filter_num`: `body_cloud`'s points |
| `lidar_min_range` / `lidar_max_range` | `0.5` / `30.0` | | metres |
| `scan_resolution` / `map_resolution` | `0.25` / `0.3` | | Voxel sizes. Coarser than fastlio2's 0.15: every point group costs an EKF update, keep headroom on the Jetson. |
| `cube_len` / `det_range` / `move_thresh` | `300.0` / `60.0` / `1.5` | | Local-map management. **Write floats**: the node declares doubles, and a bare `300` is an int64 override → `InvalidParameterTypeException` at startup. |
| `gyr_cov_output` / `acc_cov_output` | `1000.0` / `500.0` | | Process noise driving the omg / acc random-walk states. At 1000 the output model tracks the raw gyro almost instantly, which is why the bridge's yaw rate is not the bias-free estimate it looks like. |
| `b_gyr_cov` / `b_acc_cov` | `0.0001` | | Bias random walk |
| `imu_meas_omg_cov` / `imu_meas_acc_cov` | `0.1` | | IMU-as-measurement noise |
| `satu_gyro` / `satu_acc` | `35.0` / `30.0` | | Saturation: axes at/over these are dropped from the IMU update (rad/s; m/s² after scaling, ≈3 g) |
| `lidar_meas_cov` / `plane_thr` | `0.01` / `0.1` | | Point-to-plane measurement |
| `batch_max_points` | `500` | | Same-timestamp groups larger than this are split into sequential chunks |
| `imu_init_num` | `20` | | Samples averaged by the static init (≈0.1 s at 200 Hz) |
| `near_search_num` | `5` | | ikd-tree neighbours per point |
| `gravity_align` | `true` | | Rotate the initial pose so gravity is −z |
| `esti_il` | `false` | | Estimate the lidar → IMU extrinsic online |
| `r_il` / `t_il` | identity / `[-0.011, -0.02329, 0.04412]` | | IMU ← lidar extrinsic. `r_il` is the row-major 3×3 flattened to 9 doubles (ROS parameters have no matrix type); the node checks the lengths and keeps the default with an ERROR log on a mismatch. |

## Running

```bash
ros2 launch syncai_pointlio pointlio.launch.py                       # config/system.ini from ~/robot_ws
ros2 launch syncai_pointlio pointlio.launch.py system_config:=/path/to/robot01.ini
```

The session specs run it in a pane of its own, immediately before the node
that consumes it: `localization` window in `start_nav.yaml` (**pointlio** →
localizer, after the `map_server` window) and `lio` window in `start_mapping.yaml`
(**pointlio** → `syncai_mapping`). The logs are `log/stack/<robot_id>/pointlio/`
and `log/stack/<robot_id>/mapping/pointlio/`. Both the fork's old `pgo_launch.py`
and `localizer_launch.py` used to `include()` this launch; neither
`mapping.launch.py` nor `localizer.launch.py` does, so **launching only
`syncai_mapping` or only `syncai_localizer` by hand starts no LIO** — run this
first.

Checking it:

```bash
ros2 node info /<robot_id>/pointlio/pointlio_node        # subs /<id>/livox/{lidar,imu}; pubs; srv reset
ros2 param get /<robot_id>/pointlio/pointlio_node cube_len   # 300.0 → the params file was found
ros2 topic hz /<robot_id>/pointlio/lio_odom               # ~10 Hz once MAPPING
ros2 service type /<robot_id>/pointlio/reset             # syncai_common/srv/ResetLIO
ros2 run tf2_ros tf2_echo <robot_id>/pointlio_odom <robot_id>/pointlio_body
```

| Log line | Meaning |
|---|---|
| `Lidar input: livox_ros_driver2/CustomMsg (/robot01/livox/lidar)` | The remap resolved; if this says `/robot01/pointlio/lidar` the launch was bypassed |
| `IMU Message is out of order` / `Lidar Message is out of order` | Buffer flushed after a backwards stamp (bag loop, driver restart) |
| `PointCloud2 input has no per-point time` | `lidar_type: 1` — batch update per scan, expected on Isaac |
| `[resetCB] LIO reset; re-initialising IMU (robot must be still).` | A `reset` landed (normally from `pgo/reset_mapping`) |
| `t_il needs 3 elements` / `r_il needs 9 elements` | Malformed extrinsic in the params file; the default was kept |

## Gotchas

- **Nothing here changes the ROS surface, and nothing should without a
  cross-repo commit.** `syncai_mapping/launch/mapping.launch.py` and
  `syncai_localizer/launch/localizer.launch.py` hardcode
  `/<robot_id>/pointlio/{body_cloud,lio_odom}`, `/<robot_id>/pointlio/reset`
  and the `<robot_id>/pointlio_odom` frame; the
  backend reads `pointlio/body_cloud`; the planner / controller costmaps
  source it; `syncai_lio_bridge` subscribes `pointlio/lio_odom`. Renaming any
  of those is a change in two repositories.
- **`syncai_mapping`'s `local_frame` must equal `world_frame` here.** `pgo_node` does
  not adopt the frame from the odom header (the localizer does); if the two
  disagree, `map → local_frame` lands on a frame nobody looks up.
- **The service type is `syncai_common/srv/ResetLIO`, not
  `interface/srv/ResetLIO`.** It moved with the port. A `pgo_node` built
  against the old `interface` type cannot call this node: `reset_mapping`
  fails cleanly ("Timed out waiting for the LIO reset; map kept"). Rebuild
  `syncai_common syncai_pointlio syncai_mapping` together, and clear the stale
  `install/pointlio` / `install/pgo` / `install/interface` from before the
  ports so the old headers and launches cannot be found by mistake.
- **Sophus is a manual dependency.** Header-only, source-built into
  `/usr/local` by the `Dockerfile`'s `deps-builder` stage with
  `SOPHUS_USE_BASIC_LOGGING=ON` (which this package's CMake also defines, so
  the headers do not pull `fmt` back in). There is no rosdep key for that
  build, so `package.xml` does not list it — the same treatment
  `syncai_mapping` gives GTSAM (and Sophus, for its `hba_node`). Recreating the container from the image keeps it; a hand-built
  one loses it.
- **An unoptimised build is unusable, not just slow.** CMake defaults
  `CMAKE_BUILD_TYPE` to Release (upstream forced it); the ikd-tree and the
  per-point EKF cannot hold the lidar rate on the Jetson at `-O0`.
  `MP_PROC_NUM=2` is the OpenMP thread count for the residual loop, sized
  alongside the localizer and the costmaps.
- **Lint is partly off.** `ament_cmake_copyright` and `cpplint` are disabled
  as they were upstream: `map_builder/` has no per-file licence headers (the
  package `LICENSE` covers it) and `ikd_Tree.*` is not clang-format clean.
  The node shell (`pointlio_node.*`, `main.cpp`) follows the workspace
  `.clang-format`.
- `lidar_type: 1` and `imu_acc_scale` exist for the Isaac Sim path, whose
  launch and params were removed from the fork in `890a54e` / `f3f752f`;
  they are in that repo's git history.
