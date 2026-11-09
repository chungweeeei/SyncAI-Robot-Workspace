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
   calls `reset`      map→pointlio_odom   → odom/base_link   terrain layer, Octo3D
                                                             overlay, live cloud WS
```

Ported into the workspace from `SyncAI-Fast-LIO2`'s `pointlio` package in
2026-09, the first of its nodes to move (`pgo` and `hba` followed as
`syncai_mapping`, then the `localizer` as `syncai_localizer`, and the fork is
no longer imported at all). The ROS surface did not change with the move — same executable
and node name, namespace, topics, service name and parameters — so no consumer
had to. `map_builder/` is the ported math (`point_ekf`, `imu_initializer`,
`lidar_processor`, refactored from HKU MaRS Point-LIO in the
`liangheming/FASTLIO2_ROS2` style, MIT, plus `ivox_map`, written here in
2026-11 — see "The local map"); `pointlio_node.*` is the ROS shell.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `lidar` → `/<robot_id>/livox/lidar` | `livox_ros_driver2/CustomMsg` (`lidar_type: 0`) or `sensor_msgs/PointCloud2` (`lidar_type: 1`), depth 100 | `syncai_bringup`'s Livox driver | The scan. `CustomMsg` carries per-point time in `curvature`; a `PointCloud2` does not, so the update degrades to one batch per scan (warned once). |
| `imu` → `/<robot_id>/livox/imu` | `sensor_msgs/Imu`, depth 2000 | the lidar's built-in IMU | Fused as a **measurement** (output model). Accelerations are multiplied by `imu_acc_scale` (Livox reports g-units). |

The two topics are **remapped by the launch file**, not parameters: the node
subscribes to the relative names `lidar` / `imu`, which would otherwise resolve
inside its own `/<robot_id>/pointlio` namespace. `ros2 node info` shows the
resolved names, and the node logs them at startup so a wrong remap shows up as
a wrong name rather than as "no data".

**Ingest has its own thread, and the queues are deep (2026-10).** Until then
the node spun single-threaded, so neither subscription callback ran while
`timerCB` did, and a KEEP_LAST reader history overwrites its oldest sample when
it fills — RELIABLE does not prevent that, the drop is the reader's own. At
depth 10 the IMU queue held 50 ms of 200 Hz data, so every `timerCB` longer
than that lost IMU samples, and the filter predicted across the hole with
nothing to say so. Replaying `dp1f_1012` (a gap-free bag) is where it was
suspected. Now the subscriptions sit in a callback group of their own under a
two-thread `MultiThreadedExecutor`, so they are drained while a scan is
processed; the depths (2000 / 100, ~10 s of each) are the backstop, and an IMU
gap over 30 ms is logged (see "Checking it").

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
| TF `world_frame → body_frame` | `<robot_id>/pointlio_odom → <robot_id>/pointlio_body` | Broadcast from the timer once the builder reaches `MAPPING`. |
| `reset` | `syncai_common/srv/ResetLIO` | Start over in place — see below. |

Nothing is published until the builder reaches `MAPPING` (a static IMU
initialisation over `imu_init_num` samples, then the first map frame), so
silence right after launch is normal.

### The frames

`body_frame` is **the lidar's built-in IMU**, not the robot base and not the
lidar itself: the filter's state is the IMU pose (`r_wi` / `t_wi`, which is
what the TF broadcasts), and every point is moved from the lidar into the IMU
frame through the fixed lidar → IMU extrinsic (`r_il` / `t_il`, ~5 cm on the
MID360) before it is published. Its static counterpart in the robot tree is
the URDF's `lidar_top_imu`, one `-t_il` from `lidar_top` (the lidar);
`syncai_lio_bridge` maps the body pose onto `base_link` through its
`lidar_frame`, which is still `lidar_top` until that repository moves to
`lidar_top_imu`. This README said "physically the lidar" until 2026-10, and the
bridge was built on that reading.

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
this is enforced; the YAML values (`pointlio_odom` / `pointlio_body`, the same
names unprefixed) are only fallbacks for running the node bare.

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

### The local map

Every scan point is matched against Point-LIO's **own** map, built from its
own past scans, never against `map.pcd`. Since 2026-11-05 that map is an
`IVoxMap` (`map_builder/ivox_map.*`), the design of upstream Point-LIO's
default branch `point-lio-with-grid-map` (itself faster-lio's iVox), in place
of the ikd-tree the port came with:

- **Grids and search.** `map_grid_resolution` (2 m) grids found by a hash of
  their integer key; a nearest search looks in the point's grid and its
  `map_nearby` (6) face neighbours only. That is approximate: on a random
  dense cloud one query in ten gets a different 5-neighbour set than an exact
  search would. Upstream accepts it for the speed.
- **Insertion: one point per voxel, the first.** Each `map_resolution`
  (0.2 m) voxel holds at most one point, the first that landed in it
  (upstream's first-wins), and `IVoxMap` enforces it itself from one set of
  the voxels it holds a point in — one set for the whole map, so the rule
  holds also where a voxel spans two grids (at 0.3 m, 2 m is not a multiple
  of the voxel); an evicted grid frees its voxels. Every scan, the first
  included, goes through it.
  Until 2026-11-09 the rule was judged outside the map
  (`LidarProcessor::incrCloudMap`, as upstream) from the 5 nearest neighbours
  the update had found — with the pose before that point's update — so a
  voxel took a second point whenever its first was not among them. On robot01
  the map grew all through a 70 h run (46 k → 281 k points, 83 → 290 per grid,
  still ~300 more per lap on the third day); the search visits every point of
  the nearby grids, so a scan's cost went 36 → 68 ms and scans were skipped
  while walking. Now the points per grid are bounded by the voxels in it.
  The voxel went 0.3 → 0.2 m with it: held strictly, 0.3 m left the map
  sparser than the leaky one and the planes fit worse. Replaying the 11-08
  run (32 min, three laps, a fresh start each time):

  | | old rule, 0.3 m | one per voxel, 0.3 m | one per voxel, 0.2 m |
  |---|---|---|---|
  | walking eff / worst scan p1 | 94 % / 88 % | 86 % / 78 % | 94 % / 89 % |
  | walking residual | 2.62 cm | 2.87 cm | 2.33 cm |
  | walking cost, median / p90 | 54 / 70 ms | 50 / 60 ms | 55 / 69 ms |
  | points at the end; added in laps 2, 3 | 42.6 k; 6.8 k, 5.2 k | 19.7 k; 2.2 k, 1.4 k | 38.4 k; 4.8 k, 3.3 k |

  All three stayed on the dock within 3 cm and skipped no scan. Whether the
  0.2 m map stops growing over a night is for a long run to show; a fresh
  map's cost is about the old one's, and it can no longer grow per voxel.
  The port before that had FAST-LIO2's rule — keep the point nearest each
  voxel's centre — whose kept point is an extreme over every scan ever
  inserted and walks off the true surface over a night of laps. The cost of
  first-wins: whatever the first observation got wrong, a person standing
  there included, stays until a `reset`. The health line's `refused` counts
  the points turned away.
- **The LRU.** Grids are ordered by their last insertion and the oldest is
  dropped whole past `map_capacity` grids. A search does not count as use (it
  runs on `MP_PROC_NUM` threads and must not write), so a grid the robot sees
  every lap but adds nothing new to ages like one it never sees. At upstream's
  1 000 000 it is a memory bound and drops nothing on any site this robot
  patrols. A capacity that forgets has to be sized off the health line's
  `map … grids`, and replayed, before it goes on a robot.
- **Where it differs from upstream.** Checked against the branch's code
  (the grid map came in `d4919a8`, 2024-01-18; its map code last changed in
  `97b0042`, 2025-02-20 — the 2026-06-13 commits are README only).
  Upstream's 2 m / 6 are its `config/*.yaml` and launch values; the code's
  own fallback for `ivox_nearby_type` is 18, which `mapping_horizon.launch`
  gets by not setting it. Its first-wins voxel is `filter_size_map` (0.4 on
  the Avia, 0.5 elsewhere); ours is `map_resolution` 0.2, and it is held by
  the map, where upstream judges it from the update's neighbours (see
  Insertion). Its first map is `init_map_size` points gathered
  over the first scans; ours is the whole first scan, one point per voxel like
  any other. Its search only puts the nearest neighbour first; ours sorts all
  k, because `updateChunk` reads the k-th distance for the √5 m gate. Its
  search reaches 5 m; ours stops at that gate's √5 m, since nothing farther
  can be in a plane the update uses (upstream's farther candidates fed its
  voxel test) — the same points are selected. It evicts as each new grid passes
  capacity; ours after the scan's batch, so a grid the scan just made is never
  the one dropped. Its `ivox3d.h` is faster-lio's (GPL-2.0), so `IVoxMap` is
  a rewrite, not a copy.
- **Tests.** `test/test_ivox_map.cpp` pins what the map returns and keeps
  against a brute-force oracle: the k nearest inside one grid and across its
  face neighbours, the corner grids a search does not look in, that a √5 m
  search selects what the gate keeps, the LRU's eviction order, and one point
  per voxel — a taken voxel whose point is not among the newcomer's nearest, a
  voxel spanning two grids, duplicates in one batch, an evicted grid's voxels
  freed, and a wall seen 300 times holding exactly the voxels it touched. The
  LIO core under `map_builder/` has no ROS in it and is
  built as the static library `pointlio_map_builder`, which the node and the
  test both link, so a change to the map or the filter is testable without a
  ROS graph.

### The reset contract

`reset` throws the local map, the EKF and the sensor buffers away and
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
- **`resetCB` takes no lock against `timerCB`.** The two share the node's
  default MutuallyExclusive callback group, so even under `main.cpp`'s
  two-thread `MultiThreadedExecutor` the timer cannot be halfway through
  `MapBuilder::process()` while the builder is swapped. Only the sensor
  subscriptions have a group of their own (and run concurrently; the buffers
  they share with the timer and the reset are behind `imu_mutex` /
  `lidar_mutex`). Moving the service or the timer into another group needs a
  mutex around `m_builder` / `m_kf` / `m_package` first.

The request is empty on purpose: a reset is not a reconfigure, and the
parameters / extrinsics stay as launched. The comments in
`syncai_common/srv/ResetLIO.srv` and `ResetMapping.srv` (both in
`SyncAI-Robot-Interface` since the pgo port) are the specification; the client
is `syncai_mapping`'s `pgo_node`.

### Gravity and the accelerometer's clip

**Gravity is fixed** (2026-11-09). The state's `g` is a 3-vector with no
process noise; the port started it with a 1e-4 covariance, so the updates
moved it. At rest `|g|` and the accelerometer bias along gravity explain the
same reading, so nothing tells them apart, and on robot01 walking pushed
`|g|` down ~0.014 an hour on both local-map rules — a gravity that weakens,
and with its direction free, a world that tilts. `gravity_init_cov` (default
`0.0`) is that initial covariance: at 0 the covariance rows of `g` start zero
and stay zero through every prediction and update, so `g` keeps 9.81 along
the direction the static init found until the next `reset`, and the bias
`ba` takes what `g` used to. What that costs: a wrong init (done in motion)
is kept for good, as it already was in practice, and the bias random walk
(`b_acc_cov`) is now all that follows a slow change in the accelerometer.
`|g|` on the health line therefore reads 9.810 always; a different value
means a non-zero `gravity_init_cov`.

**`satu_acc` sits just under the clip** (39.0 m/s², 2026-11-09). The MID360's
IMU clips at 4.004 g, 39.28 m/s² at `imu_acc_scale` 9.81. An axis at or over
`satu_acc` is dropped from the update; at the port's 30 m/s² (3.06 g) that
also dropped the gait's 3.06–4.0 g impacts, ~90 % of them real readings, so
the update saw a gait with its peaks cut off. Only the clipped samples are
dropped now, and `sat acc` on the health line counts just those. If
`imu_acc_scale` is ever 10 (the code default), the clip is 40.04 m/s² and
39.0 also drops 3.9–4.0 g readings — still under the clip, which is what
matters.

`test/test_fixed_gravity.cpp` runs a minute of a synthetic gait with the
position pinned and compares `g` bit for bit (and the same walk moving it at
the port's 1e-4), and feeds a 3.5 g impact and a clipped sample through
`updateIMU`.

### Falling behind

The timer takes one scan per 20 ms tick, and the sensor callbacks only buffer.
A scan that costs more than the lidar's 100 ms therefore leaves the next one
waiting, and until 2026-10-28 nothing bounded the wait: on robot01 that day,
with the eight online cores ~90 % busy (Jetson in `MODE_30W`, the
`robotdog-ai` inference workers beside the stack), the node ran at ~0.65× real
time and its output fell **more than five minutes** behind the lidar, the
buffers growing ~125 MB a minute. Nothing downstream can see that — every
stamp is the scan's own — so the localizer, `lio_bridge` and the costmaps all
worked on minutes-old data, and only a restart recovered it.

The backlog guard (`max_lidar_backlog`, `max_lidar_skip`) turns that into
a degradation: when the buffered scans span more than 0.3 s, the oldest are
skipped before the next one is taken, at most two in a row. A skipped scan loses
its points only; its IMU samples go into the next package, so the filter
predicts and IMU-updates through the ~0.2 s gap, and it catches up whenever it
runs faster than a third of real time. It never skips straight to the newest
scan: seconds of IMU-only propagation on a walking robot put the next scan's
points on the wrong planes, and a diverged solution is worse than a late one.
Below a third of real time the backlog still grows, and the WARN it logs every
5 s while it skips (see "Running") is the only sign of it — that is a CPU
problem, not a tuning one. An offline replay that wants every scan processed
however late sets `max_lidar_backlog: 0.0`.

### The health log

On 2026-10-30 robot01's Point-LIO diverged on the dock (to ~10⁷ m at
~11 km/s) after hours of slowly growing localizer corrections, and this node's
log said nothing until it was over: it never logged how well its scans were
matching, so the decline could only be read off the localizer's corrections,
and its cause not at all. Since 2026-11-02 it logs one `lio health:` line per
`health.summary_period` (1 s), from numbers the filter already has:

```
lio health: 10 scans in 1.0 s | pts 412 near 93% eff 81% (min 74%) res 1.21 cm | chunks 98/100 | imu 200 sat acc 0 gyro 0 max |a| 21.3 |w| 1.20 | v 0.62 max 0.71 m/s | std p 1.10 cm r 0.080 deg | |ba| 0.051 |bg| 0.0030 |g| 9.810 | cost 18.2 max 25.0 ms | map 812 grids 95210 pts evicted 0 refused 4120533
```

(Illustrative numbers: a healthy robot's have not been recorded yet.)

| Field | What it is | What a decline looks like |
|---|---|---|
| `pts` | Mean downsampled points per scan — what the update sees | Few: open space, an occluded lidar |
| `near` | Share with `near_search_num` map neighbours, the farthest within √5 m | Falling: the scan no longer lands on the map — the pose is already off, or the place is new |
| `eff` / `(min …)` | Share that fit a plane and passed the residual gate, i.e. the update's rows; window mean / worst scan | `near` high but `eff` low: surroundings that are not planar (grass, foliage, people) |
| `res` | Mean \|n·p + d\| of those points before their update | Growing: the map and the scan disagree more |
| `chunks a/b` | Update chunks with at least one matched point / all | `a < b`: stretches of the scan that were IMU propagation only |
| `imu`, `sat acc` / `gyro`, `max \|a\|` / `\|w\|` | IMU samples fused, samples with an axis at or over `satu_acc` / `satu_gyro` (dropped from the update), largest \|axis\| (m/s² after `imu_acc_scale`; rad/s) | Any saturation: samples at the IMU's clip (4.004 g), since `satu_acc` 39.0 sits just under it — the 2026-10-30 dock bag peaked at ~33 m/s², dropped at the old 30 |
| `v … max …` | Speed (world frame) at the last scan, and the window's largest | Over the robot's 0.8 m/s: running away |
| `std p` / `r` | Position / attitude 1σ from the covariance | Shrinking while `res` grows: an over-confident filter discounting its scans |
| `\|ba\|`, `\|bg\|`, `\|g\|` | Bias and gravity estimate norms | Biases walking off. `\|g\|` is 9.810 at `gravity_init_cov` 0 (see "Gravity and the accelerometer's clip"); it drifted when it was estimated |
| `cost` | `timerCB` per processed scan, mean / max, against the lidar's 100 ms | See "Falling behind" |
| `map … grids … pts evicted … refused …` | The local map's grids and points, the grids its LRU has dropped and the points it turned away because their voxel already held one, both since the last `reset` (see "The local map"); last on the line, so a parser of the earlier fields keeps their positions (`refused` came after `evicted`) | `pts` still growing on a route already driven: the scans are landing in voxels the earlier laps left empty — a pose off the surfaces it mapped. `refused` climbs by roughly the scan's points every scan once a place is mapped |

The line is a **WARN** with every reason that holds appended after ` -- `
(`speed 2.13 m/s over health.max_speed 1.50`, `3 IMU samples with |acc| 39.3 >=
satu_acc 39.0 dropped from the update`, `1 of 10 scans matched no plane (IMU
only)`, and — only with `health.min_effective_ratio` > 0 — `effective ratio
down to …`); otherwise INFO. A window with no scan at all after the node has
been mapping is one WARN, `no scan processed in the last 1.0 s (status …, lidar
buffer N scans, IMU buffer M samples)` — lidar 0 means no lidar arriving,
lidar > 0 the IMU not covering the scan's end — and `scans processed again`
once they are. A `reset` starts the window over. Skipped scans stay the
backlog guard's own line.

```bash
grep 'lio health' log/stack/<robot_id>/pointlio/current | tai64nlocal   # the timeline
grep -- 'lio health.* -- ' log/stack/<robot_id>/pointlio/current        # only the trouble
```

At 1 s it is ~300 B a second, ~26 MB a day — well inside the multilog budget
(16 MiB × 100). It is log only: nothing here touches the filter or what is
published. The localizer's `loc health` (its README) is the same idea from the
other side; it stops measuring once its rounds stop converging, this one does
not.

## Parameters

`params/pointlio_params.yaml`, keyed `/**/pointlio_node:` so it applies at any
namespace. Every key is a `declare_parameter` whose default repeats the struct
default, so a missing key degrades to the code default instead of throwing.
The launch layers the two frame names on top.

| Parameter | Default (YAML) | Launch value | Notes |
|---|---|---|---|
| `world_frame` | `pointlio_odom` | `<robot_id>/pointlio_odom` | The YAML said `lidar` until 2026-10, and the code default still does; `pointlio_odom` is what `syncai_mapping`'s `pgo_node` derives in a bare run |
| `body_frame` | `pointlio_body` | `<robot_id>/pointlio_body` | The lidar's IMU (see "The frames"); `body` until 2026-10, and still the code default |
| `lidar_type` | `0` | | `0` Livox `CustomMsg`, `1` `PointCloud2` (kept for the Isaac Sim path; no per-point time) |
| `imu_acc_scale` | `10.0` | | Livox IMU accel is in g → m/s². Sensors already in m/s² need `1.0`; the filter's gravity is 9.81 (and stays there, see `gravity_init_cov`). robot01's YAML says `9.81`. `satu_acc` is compared **after** scaling. |
| `print_time_cost` | `false` | | Logs, per frame, `process()` time and the whole `timerCB` (process + deskew + publish) as a WARN. The total is the one to compare with the input queues: the subscriptions wait for all of it. |
| `max_lidar_backlog` / `max_lidar_skip` | `0.3` / `2` | | The backlog guard (see "Falling behind"): beyond `max_lidar_backlog` seconds of buffered scans the oldest are skipped, at most `max_lidar_skip` in a row, their IMU still fused. **Write a float** for the first. `<= 0.0` processes every scan however late |
| `health.summary_period` / `health.max_speed` / `health.min_effective_ratio` | `1.0` / `1.5` / `0.0` | | The health log (see "The health log"): one `lio health:` line per period (`<= 0.0` = off), a WARN when the speed exceeds `max_speed` (the robot tops out at 0.8 m/s) or a scan's matched share falls under `min_effective_ratio` (`0.0` = unchecked until a healthy robot's ratio is measured). **Write floats** |
| `lidar_filter_num` | `6` | | Keep every n-th point: the filter's input and `body_cloud` |
| `dense_filter_num` | `2` | | Keep every n-th point for `body_cloud_dense` only. A divisor of `lidar_filter_num` makes `body_cloud`'s points a subset (warned otherwise). 2 is the floor: at 1 a MID360 scan is ~320 KB, over CycloneDDS's default 208 KB receive buffer on `lo`. `<= 0` or `>= lidar_filter_num`: `body_cloud`'s points |
| `lidar_min_range` / `lidar_max_range` | `0.5` / `30.0` | | metres |
| `scan_resolution` / `map_resolution` | `0.25` / `0.2` | | Voxel sizes: the scan's downsampling, and the local map's one-point-per-voxel. The scan is coarser than fastlio2's 0.15 because every point group costs an EKF update; keep headroom on the Jetson. The map's is finer than the scan's so that scans from different poses fill it in (see "The local map" for why 0.2 and not the port's 0.3). |
| `map_grid_resolution` / `map_capacity` / `map_nearby` | `2.0` / `1000000` / `6` | | The local map (see "The local map"): grid edge (m), LRU capacity (grids), search neighbourhood (`0` / `6` / `18` / `26`). Upstream Point-LIO's values. Out-of-range values log an ERROR and fall back to these. **Write the resolution as a float.** |
| `gyr_cov_output` / `acc_cov_output` | `1000.0` / `500.0` | | Process noise driving the omg / acc random-walk states. At 1000 the output model tracks the raw gyro almost instantly, which is why the bridge's yaw rate is not the bias-free estimate it looks like. |
| `b_gyr_cov` / `b_acc_cov` | `0.0001` | | Bias random walk |
| `imu_meas_omg_cov` / `imu_meas_acc_cov` | `0.1` | | IMU-as-measurement noise |
| `satu_gyro` / `satu_acc` | `35.0` / `39.0` | | Saturation: axes at/over these are dropped from the IMU update (rad/s; m/s² after scaling). `satu_acc` sits just under the IMU's 4.004 g clip; 30 (≈3 g) until 2026-11-09 (see "Gravity and the accelerometer's clip"). **Write floats** |
| `lidar_meas_cov` / `plane_thr` | `0.01` / `0.1` | | Point-to-plane measurement |
| `batch_max_points` | `500` | | Same-timestamp groups larger than this are split into sequential chunks |
| `imu_init_num` | `20` | | Samples averaged by the static init (≈0.1 s at 200 Hz) |
| `near_search_num` | `5` | | Map neighbours per point, for the plane fit |
| `gravity_align` | `true` | | Rotate the initial pose so gravity is −z |
| `gravity_init_cov` | `0.0` | | Initial covariance of the gravity state. `0` holds `g` fixed for good; the port's `0.0001` let the updates move it, and it drifted (see "Gravity and the accelerometer's clip"). Negative logs an ERROR and uses 0. **Write a float** |
| `esti_il` | `false` | | Estimate the lidar → IMU extrinsic online |
| `r_il` / `t_il` | identity / `[-0.011, -0.02329, 0.04412]` | | IMU ← lidar extrinsic. `r_il` is the row-major 3×3 flattened to 9 doubles (ROS parameters have no matrix type); the node checks the lengths and keeps the default with an ERROR log on a mismatch. |

## Running

```bash
ros2 launch syncai_pointlio pointlio.launch.py                       # config/system.ini from ~/robot_ws
ros2 launch syncai_pointlio pointlio.launch.py system_config:=/path/to/robot.ini
```

The session specs run it in a pane of its own, immediately before the node
that consumes it: `localization` window in `start_nav.yaml` (**pointlio** →
localizer, right after `bringup`) and `lio` window in `start_mapping.yaml`
(**pointlio** → `syncai_mapping`). The logs are `log/stack/<robot_id>/pointlio/`
and `log/stack/<robot_id>/mapping/pointlio/`. Both the fork's old `pgo_launch.py`
and `localizer_launch.py` used to `include()` this launch; neither
`mapping.launch.py` nor `localizer.launch.py` does, so **launching only
`syncai_mapping` or only `syncai_localizer` by hand starts no LIO** — run this
first.

Checking it:

```bash
ros2 node info /<robot_id>/pointlio/pointlio_node        # subs /<id>/livox/{lidar,imu}; pubs; srv reset
ros2 param get /<robot_id>/pointlio/pointlio_node lidar_max_range   # 30.0 (code default 20.0) → the params file was found
ros2 topic hz /<robot_id>/pointlio/lio_odom               # ~10 Hz once MAPPING
ros2 service type /<robot_id>/pointlio/reset             # syncai_common/srv/ResetLIO
ros2 run tf2_ros tf2_echo <robot_id>/pointlio_odom <robot_id>/pointlio_body
```

| Log line | Meaning |
|---|---|
| `Lidar input: livox_ros_driver2/CustomMsg (/robot01/livox/lidar)` | The remap resolved; if this says `/robot01/pointlio/lidar` the launch was bypassed |
| `IMU Message is out of order` / `Lidar Message is out of order` | Buffer flushed after a backwards stamp (bag loop, driver restart) |
| `lio health: 10 scans in 1.0 s \| pts … near … eff …` | The health log, once a second (see "The health log"); a WARN with ` -- <reasons>` when one holds |
| `lio health: no scan processed in the last 1.0 s (status …, lidar buffer N scans, IMU buffer M samples)` | Mapping stopped getting scans: lidar 0 = none arriving, lidar > 0 = the IMU not covering them. Once, until `lio health: scans processed again` |
| `IMU gap of N ms (samples lost before this node?)` | Two consecutive IMU stamps more than 30 ms apart (throttled to 1 s). The MID360 alone stays under ~23 ms; a gap past that is samples dropped by the driver, DDS or this node's queue — the filter coasted through it without a measurement |
| `PointCloud2 input has no per-point time` | `lidar_type: 1` — batch update per scan, expected on Isaac |
| `[resetCB] LIO reset; re-initialising IMU (robot must be still).` | A `reset` landed (normally from `pgo/reset_mapping`) |
| `t_il needs 3 elements` / `r_il needs 9 elements` | Malformed extrinsic in the params file; the default was kept |
| `lidar backlog up to 0.45 s (max_lidar_backlog 0.30 s): skipped N scans in the last 5 s …` | The node is not keeping up with the lidar and the guard is skipping scans (every 5 s while it lasts). A backlog that keeps growing from one line to the next means it is below 1 / (`max_lidar_skip` + 1) of real time — look at the CPU |
| `lidar backlog back under max_lidar_backlog` | No scan skipped for a whole window after a WARN above |

## Gotchas

- **Nothing here changes the ROS surface, and nothing should without a
  cross-repo commit.** `syncai_localizer/launch/localizer.launch.py` hardcodes
  `/<robot_id>/pointlio/{body_cloud,lio_odom}` and the
  `<robot_id>/pointlio_odom` frame; `syncai_mapping`'s `pgo_node` (its own
  repo, SyncAI-Robot-3D-Mapping, since 2026-10) derives
  `/<robot_id>/pointlio/{body_cloud_dense,lio_odom}`, `/<robot_id>/pointlio/reset`
  and that frame from its namespace; the backend reads `pointlio/body_cloud`;
  the planner / controller costmaps source it; `syncai_lio_bridge` (also its
  own repo) subscribes `pointlio/lio_odom`. Renaming any of those is a change
  in several repositories.
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
  `syncai_mapping` gives GTSAM. Recreating the container from the image keeps it; a hand-built
  one loses it.
- **An unoptimised build is unusable, not just slow.** CMake defaults
  `CMAKE_BUILD_TYPE` to Release (upstream forced it); the map search and the
  per-point EKF cannot hold the lidar rate on the Jetson at `-O0`.
  `MP_PROC_NUM=2` is the OpenMP thread count for the residual loop, sized
  alongside the localizer and the costmaps. It applies only to a chunk of 64
  points or more (`kMinParallelChunk`), i.e. `PointCloud2` input: a Livox
  scan's chunks are ~1 point each (a voxel's time is a mean, so timestamps are
  practically unique), and a 2-thread region per point gave the worker no
  work and kept it spinning between regions — on robot01 one worker per
  executor thread, ~0.4 core in all (2026-11-06).
- **Lint is partly off.** `ament_cmake_copyright` and `cpplint` are disabled
  as they were upstream: `map_builder/` has no per-file licence headers (the
  package `LICENSE` covers it) and the ported files are not clang-format
  clean. `ivox_map.*` is ours and does not copy faster-lio's iVox, which is
  GPL-2.0.
  The node shell (`pointlio_node.*`, `main.cpp`) follows the workspace
  `.clang-format`.
- `lidar_type: 1` and `imu_acc_scale` exist for the Isaac Sim path, whose
  launch and params were removed from the fork in `890a54e` / `f3f752f`;
  they are in that repo's git history.
