# syncai_localizer

Map-based relocalization for the navigation session. One node,
`localizer_node`, registers `syncai_pointlio`'s body-frame scan against the
saved `map.pcd` with two stages of GICP (rough, 0.25 m voxels, then refine,
0.1 m; [small_gicp](https://github.com/koide3/small_gicp) as the backend) and
broadcasts the result as the `map → <robot_id>/pointlio_odom` correction that
`syncai_lio_bridge` turns into `map → odom`. It serves `relocalize` /
`relocalize_check`, listens on `initialpose`, and applies the INI's
`[initial_pose]` as a boot guess. In the mapping session `syncai_mapping`'s
`pgo_node` owns the same TF instead; this node is never up at the same time.

```
   syncai_pointlio           /<id>/pointlio/{body_cloud, lio_odom}
        │                           │  (ApproximateTime sync)
        │                           ▼
        │                     localizer_node                 /<id>/…  (bare robot_id namespace)
        │          rough GICP → refine GICP, motion-gated, ≤ update_hz
        │             TF  map ──► <id>/pointlio_odom  (frame adopted from lio_odom)
        │       ┌──────────────┬──────────────┬──────────────┬─────────────┐
        │       ▼              ▼              ▼              ▼             ▼
        │  relocalize   relocalize_check   initialpose    map_cloud   [initial_pose]
        │  (backend:    (backend polls     (console /     (latched,   (INI, once, on
        │   map switch)  the outcome)       rviz 2D pose)  rviz)       first odom)
```

Ported into the workspace from `SyncAI-Fast-LIO2`'s `localizer` package in
2026-09, the last node left in that fork (`pointlio`, `pgo`, `hba` and its
`interface` srvs had gone the same way earlier that month), so the fork is no
longer imported at all. The ROS surface did not change with the move — same
executable and node name, namespace, services, topics, TF and parameters — and
the two service types were already `syncai_common`'s. `localizers/` is the
ported registration (`icp_localizer.*`, `commons.h`, upstream lineage, MIT);
`localizer_node.*` is the ROS shell.

## The namespace is the bare `robot_id`

The launch puts the node at **`/<robot_id>/localizer_node`**, not
`/<robot_id>/localizer/localizer_node`, so every relative name resolves one
level up from where most docs used to put it:

| Kind | Resolved name (`robot_id = robot01`) |
|---|---|
| node | `/robot01/localizer_node` |
| services | `/robot01/relocalize`, `/robot01/relocalize_check` |
| subscribed | `/robot01/initialpose` |
| published | `/robot01/map_cloud` |
| TF | `map → robot01/pointlio_odom` |

It was `<robot_id>/localizer` until fork commit `3f5f01b` (2026-07-29) dropped
the segment, and the backend has called the bare `relocalize` /
`relocalize_check` and published the bare `initialpose` from the robot's
namespace ever since — its map gateway records that writing
`localizer/relocalize` cost it a `stack_not_ready` refusal against a perfectly
healthy stack. The fork's README, the srv comments in `SyncAI-Robot-Interface`
and the workspace docs kept the old spelling for two months; the port kept the
*behaviour* (moving the namespace back is a cross-repository change the
backend would have to follow) and corrected the docs in this repo instead. The
srv comments in the interface repo still say `<robot_id>/localizer/…` and are
wrong. Check with `ros2 node info /<robot_id>/localizer_node`, not with
`ros2 service list`, which can keep a stale entry after a participant dies.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `cloud_topic` = `/<robot_id>/pointlio/body_cloud` | `sensor_msgs/PointCloud2`, depth 10 | `syncai_pointlio` | The scan to register (voxelised twice, at the rough and refine resolutions) |
| `odom_topic` = `/<robot_id>/pointlio/lio_odom` | `nav_msgs/Odometry`, depth 10 | `syncai_pointlio` | The LIO pose the correction is composed with; its `header.frame_id` becomes `local_frame` |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped`, depth 10 | the console, rviz "2D Pose Estimate", the backend's map switch | A 2D guess through `applyPlanarGuess` (below) |
| `relocalize` | `syncai_common/srv/Relocalize` | the backend (initial pose, map switch), a shell | (Re)load a PCD and take the request's raw 6-DOF pose as the next guess |
| `[map] pcd`, `[initial_pose]` | `config/system.ini` via the launch | | `map_path` (loaded during construction) and the boot guess |

The two topics are joined by a `message_filters` `ApproximateTime` synchroniser
(queue 10, age penalty 0.1). They are **absolute names set as parameters**,
not remappings: they live in pointlio's namespace, which a relative name from
`/<robot_id>` cannot reach, so the launch overrides both per `robot_id` and
they are the contract with `syncai_pointlio/launch/pointlio.launch.py`. The
node logs the resolved names at startup, so a bare run (subscribing to
`/pointlio/…`, which nothing publishes) shows up in the log rather than as
silence.

## Outputs

| Output | Type / QoS | Notes |
|---|---|---|
| TF `map → local_frame` | `<robot_id>/pointlio_odom` | Rebroadcast from a 10 ms timer on every tick, re-solved at most `update_hz` and only when the motion gate says so. `local_frame` is **adopted from the first odom message**; the YAML value is a placeholder. |
| `map_cloud` | `PointCloud2`, depth 1, TRANSIENT_LOCAL | The refine-resolution map, published once per `loadMap` (construction, `relocalize`, the `initialpose` fallback). For rviz; set the display's durability to Transient Local. |
| `relocalize_check` | `syncai_common/srv/IsValid` | `code: 1` always answers `valid: true` (liveness); anything else answers whether the first registration after the last guess converged. **The stack's only localization-quality signal.** |

Nothing is broadcast until the first cloud/odom pair has arrived; with no map
loaded the TF stays at identity.

### Three ways a guess arrives, two of them planar

1. **`relocalize`** takes the request's **raw 6-DOF** pose and (re)loads
   `pcd_path` as a side effect. Success is a **receipt, not a result**:
   registration runs on the timer with no deadline, and `relocalize_check` is
   the only thing that reports the outcome. It is also the first step of the
   backend's live map switch.
2. **`initialpose`** and 3. **the INI's `[initial_pose]`** (applied once, on
   the first odom) go through `applyPlanarGuess`: only x / y / yaw are taken,
   and roll / pitch / z are filled in from the **current estimate**. This is
   not a nicety. The lidar mount is tilted (~15° pitch) and the map is
   gravity-aligned, so the true `map → body` always carries that pitch; a flat
   guess tilts the whole scan against the map, far points exceed
   `rough_max_corr_dist`, rough GICP never passes its score, and because a
   pending guess also suspends the odom-feedback path the localizer freezes on
   the old offset and retries forever. Measured on the robot: an `initialpose`
   at (5.0, 0.982) hung; with the 14.6° pitch filled in the same spot
   converged on the first round.

The consequence for callers: a `relocalize` from a 2D source (the console's
gridmap click) must be **followed by an `initialpose` publish**, which
overwrites the flat guess with a tilt-correct one before the next round. The
backend's map switch does exactly that, in that order. Keep both paths.

### The motion gate

`update_hz` (5) is a ceiling, not a schedule. Parked for 60 s on robot01
(2026-09-21, dp2f map), the correction swung 33.7 / 29.0 mm / 1.07° while
Point-LIO underneath moved 7.8 / 6.9 mm / 0.27°: GICP's own noise floor being
resampled five times a second, which `syncai_robot_state` then sampled at a
random phase and the console showed as teleporting. So when the odom pose has
moved less than `min_update_trans` / `min_update_rot` since the last accepted
registration, the previous offset is rebroadcast and GICP is skipped; a
`max_update_interval` backstop re-registers anyway and blends the result in at
`static_blend_alpha`. Motion-triggered updates always use α = 1, so driving is
unchanged. A guess (any of the three) bypasses the gate and the blend and
opens a `post_reloc_settle` full-rate window, because the pose only converges
over the rounds that follow. The rejected alternatives (plain EMA, plain
deadband) and the numbers are in `params/localizer_params.yaml`; do not
re-propose them without new data. The chosen failure direction is "jitter
partly suppressed", never "frozen on a stale pose".

## Parameters

`params/localizer_params.yaml`, keyed `/**/localizer_node:` so it applies at
any namespace. Every key is a `declare_parameter` whose default repeats the
struct default, so a missing key degrades to the code default instead of
throwing. The launch layers the instance values on top.

| Parameter | Default (YAML) | Launch value | Notes |
|---|---|---|---|
| `cloud_topic` / `odom_topic` | `/pointlio/body_cloud` / `/pointlio/lio_odom` | `/<robot_id>/pointlio/…` | Absolute, into pointlio's namespace |
| `map_frame` | `map` | | Never namespaced |
| `local_frame` | `laser` | | Placeholder; replaced by the first odom's `frame_id` |
| `map_path` | `""` | `[map] pcd`, made absolute | Loaded in the constructor. Empty → the node runs but cannot localize until `relocalize` |
| `set_initial_pose`, `initial_pose.{x,y,yaw}` | `false`, `0.0` | from `[initial_pose]` if the section exists | Applied once on the first odom, through `applyPlanarGuess` |
| `update_hz` | `5.0` | | Ceiling on re-registration; was 1.0 until 2026-08-28 (staircase corrections at 0.7 m/s) |
| `min_update_trans` / `min_update_rot` | `0.05` m / `0.02` rad | | Motion gate thresholds |
| `max_update_interval` | `2.0` s | | Backstop re-registration when parked |
| `static_blend_alpha` | `0.1` | | EMA weight for a backstop refresh; `1.0` keeps the gate and drops the blend |
| `post_reloc_settle` | `3.0` s | | Full-rate window after a guess |
| `num_threads` | `4` | | GICP covariance / KD-tree / reduction threads. Read **once**: `RegistrationPCL` builds the tree at `setInputTarget`. Shares the CPU with pointlio |
| `num_neighbors` | `20` | | Local-covariance neighbours (pcl::GICP's correspondence randomness) |
| `rough_*` | `0.25` / `0.25` m, `20` iters, score `0.2`, corr `2.0` m, `GICP`, voxel `1.0` | | Absorbs a hand-entered guess. VGICP was tried and reverted (double solutions in degenerate geometry, 2026-08-28) |
| `refine_*` | `0.1` / `0.1` m, `20` iters, score `0.1`, corr `0.5` m, `GICP`, voxel `0.5` | | The fine stage |

**Write floats** for double parameters (`2.0`, not `2`): a bare integer is an
int64 override and the node dies with `InvalidParameterTypeException`.

## Running

```bash
ros2 launch syncai_localizer localizer.launch.py                        # ~/robot_ws/config/system.ini
ros2 launch syncai_localizer localizer.launch.py system_config:=/path/to/robot01.ini
```

The nav session (`config/sessions/start_nav.yaml`) runs it as the third pane
of the `localization` window, after map_server and `syncai_pointlio`, logging
to `log/stack/<robot_id>/localizer/`. It is deliberately **absent** from the
mapping session, and could not be present anyway: **the launch starts nothing
when `[map] pcd` is missing or absent on disk** (`nothing to launch` in the
log). That is intentional — the node loads the map in its constructor and a
localizer with no map fails every call silently — so do not make it start
anyway. Launching this alone starts no LIO; run `syncai_pointlio` first.

```bash
# coarse pose in the map frame, radians; loads the PCD again as a side effect
ros2 service call /<robot_id>/relocalize syncai_common/srv/Relocalize \
  "{pcd_path: '/abs/path/map/<name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
# then a planar guess, so roll/pitch/z are filled in from the current estimate
ros2 topic pub --once /<robot_id>/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
# did the first registration after that guess succeed?
ros2 service call /<robot_id>/relocalize_check syncai_common/srv/IsValid "{code: 0}"
```

Checking it:

```bash
ros2 node info /<robot_id>/localizer_node                 # subs /<id>/pointlio/{body_cloud,lio_odom}, initialpose; srvs relocalize, relocalize_check
ros2 param get /<robot_id>/localizer_node update_hz       # 5.0 → the params file was found
ros2 run tf2_ros tf2_echo map <robot_id>/pointlio_odom    # the correction; identity until a guess converges
```

| Log line | Meaning |
|---|---|
| `params: cloud_topic=/robot01/pointlio/body_cloud …` | The overrides landed; `/pointlio/…` here means the launch was bypassed |
| `map preloaded from …` | Constructor `loadMap` succeeded; `map_cloud` was published |
| `initial pose from config: … (applied on first odom)` | `[initial_pose]` was read; `guess from config applied` follows on the first odom |
| `guess from initialpose applied: … (roll/pitch/z from current estimate)` | An `initialpose` went through `applyPlanarGuess` |
| `rough : converged=no iters=20 …` every round | Stuck: the guess is too far off, or a flat guess on the tilted mount — publish an `initialpose` |
| `refine: converged=yes … fitness=0.0xx` | A registration was accepted; the TF moved |
| `initialpose ignored: map not loaded …` | No `map_path` and no `relocalize` yet; the message was dropped, not queued |

## Gotchas

- **The namespace is `/<robot_id>`, not `/<robot_id>/localizer`.** See above.
  The rviz config in `config/rviz2/` reads `/<robot_id>/map_cloud`.
- **`relocalize` success is a receipt.** Poll `relocalize_check`.
  `RobotState.localization_valid` is TF presence only and reads true against a
  map the robot was never localized in; do not turn it into a quality signal
  from anything but `relocalize_check`.
- **`relocCB` takes the raw 6-DOF pose; the other two paths are planar.** A
  `relocalize` from a 2D source must be followed by an `initialpose`, or the
  tilted mount leaves rough GICP retrying a flat guess forever.
- **`small_gicp` reports `converged` only on a small update step.** Hitting
  `max_iteration` is *not converged*, unlike PCL's ICP, and `align()` treats
  `hasConverged()` as a hard condition. Iteration counts below small_gicp's
  default of 20 silently stop the TF from ever updating; the log prints
  `converged=` and `fitness=` before the threshold check so the two failures
  can be told apart.
- **`setInput` allocates a fresh cloud every round on purpose.**
  `RegistrationPCL::setInputSource` early-returns on an identical pointer and
  would then register the new scan with the previous scan's covariances.
- **`num_threads` is read once, at construction.** Changing it at runtime does
  not rebuild the KD-tree.
- **Two-thread `MultiThreadedExecutor`, and it is load-bearing.** The timer +
  subscriptions sit on the default group; `relocalize`, `relocalize_check` and
  `initialpose` on their own MutuallyExclusive group, so a multi-second
  `loadMap` never gaps the TF rebroadcast. `ICPLocalizer::m_target_mutex`
  exists for that split (`loadMap` builds into locals and swaps under it;
  `align` holds it for one round). Collapsing to `rclcpp::spin()` is safe but
  reintroduces the gap; adding a third group needs a look at `NodeState`'s two
  mutexes first.
- **Lint is partly off.** `ament_cmake_copyright` and `cpplint` are disabled
  as they were upstream: `localizers/` has no per-file licence headers (the
  package `LICENSE` covers it). The node shell follows the workspace
  `.clang-format`.
- **A stale fork checkout builds a second `localizer` package.** If
  `src/third-party/FASTLIO2_ROS2` is still on disk from before this port,
  colcon builds its `localizer` too (same executable name, different package).
  Delete it, and `build/localizer` + `install/localizer`, so only one launch
  exists for this node.
