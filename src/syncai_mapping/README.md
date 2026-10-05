# syncai_mapping

The mapping back end, two nodes. `pgo_node` (the rest of this README until
"hba_node") is a pose graph over `syncai_pointlio`'s odometry and body clouds. It picks keyframes, detects and verifies loop
closures, smooths the graph with GTSAM iSAM2, broadcasts the resulting
`map → <robot_id>/pointlio_odom` correction while a map is being built, hands
the "map so far" to the operator console, and serves the three calls that
bracket a mapping run: `start_mapping`, `save_maps` and `reset_mapping`. It
comes up **idle** — nothing is banked until `start_mapping`, and a successful
`save_maps` ends the run — and reports that state on `mapping_status`. It
runs only in the mapping session; in navigation `syncai_localizer` owns the
same TF.

```
   syncai_pointlio            /<id>/pointlio/{body_cloud, lio_odom}
        │                           │  (ApproximateTime sync)
        │ reset ◄────── ResetLIO ───┤
        │                           ▼
        │                       pgo_node                  /<id>/pgo/…
        │             keyframes · loop closure · iSAM2
        │             TF  map ──► <id>/pointlio_odom
        │                ┌──────────┼──────────────┬────────────────┬──────────────┐
        │                ▼          ▼              ▼                ▼              ▼
        │          map_cloud   map_cloud_file  save_maps      start_mapping   mapping_status
        │          (rviz)      JSON notice ──► /dev/shm/…   map/<name>/    reset_mapping   IDLE / MAPPING /
        │                      (backend: live preview)     map.pcd, patches/, poses.txt   RESETTING (latched)
        │                                                  (ends the run)  (backend: "Start" / "New map")
```

Ported into the workspace from `SyncAI-Fast-LIO2`'s `pgo` package in 2026-09
(`hba` came over in the same month as this package's second node, below, and
the `localizer` last, as `syncai_localizer`; the fork is no longer imported).
The ROS surface did not change with the move — same node name, namespace,
services, topics, TF, hand-off directory and on-disk layout — so no consumer
had to. Two things did change: the two service **types** are
`syncai_common/srv/SaveMaps` and `ResetMapping` now (the backend, their only
caller, builds against that package and had to switch its imports), and
configuration is declared ROS parameters instead of a yaml-cpp `config_path`.
`pgos/` is the ported graph (`simple_pgo.*`, `commons.*`, upstream lineage,
MIT); `pgo_node.*` is the ROS shell.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `cloud_topic` = `/<robot_id>/pointlio/body_cloud` | `sensor_msgs/PointCloud2`, depth 10 | `syncai_pointlio` | The keyframe's body-frame scan |
| `odom_topic` = `/<robot_id>/pointlio/lio_odom` | `nav_msgs/Odometry`, depth 10 | `syncai_pointlio` | The keyframe's pose in `local_frame` |

The two are joined by a `message_filters` `ApproximateTime` synchroniser
(queue 10, age penalty 0.1) into one cloud+pose pair per lidar frame. They are
**absolute names set as parameters**, not remappings: they live in pointlio's
namespace, which a relative name from inside `/<robot_id>/pgo` cannot reach,
so the launch overrides both per robot_id. The node logs the resolved names at
startup, so a bare run (subscribing to `/pointlio/…`, which nothing publishes)
shows up in the log rather than as silence.

The synchroniser only buffers; a 50 ms timer takes the **oldest** pair and
drains the rest, so under load the graph falls behind by whole frames rather
than queueing them. Nothing reaches the graph unless the node is MAPPING (see
"The run lifecycle"): idle, it drops every pair before the copy and only
re-broadcasts an identity `map → local_frame`, so the console's live scan —
which looks that transform up — stays visible while the operator positions
the robot before Start.

## Outputs

All relative to `/<robot_id>/pgo/` except the TF.

| Output | Type / QoS | Notes |
|---|---|---|
| TF `map → local_frame` | `<robot_id>/pointlio_odom` | The loop-closure correction, rebroadcast from the timer on every pair (keyframe or not). `local_frame` **must equal pointlio's `world_frame`** — this node does not adopt the frame from the odometry header (the localizer does), so a mismatch lands the correction on a frame nobody looks up. |
| `map_cloud` | `PointCloud2`, depth 1 | The "map so far", every keyframe placed with its *current* corrected pose and voxelised at `map_cloud_resolution`. For rviz and `ros2 topic echo` only: see the hand-off below for why the console does not read it. |
| `map_cloud_file` | `std_msgs/String`, RELIABLE + TRANSIENT_LOCAL, depth 1 | A ~200 B JSON notice naming the same merge as a PCD file. What the backend reads. |
| `loop_markers` | `MarkerArray`, depth 10000 | Loop-closure nodes and edges for rviz. Subscriber-gated. |
| `mapping_status` | `syncai_common/msg/MappingStatus`, RELIABLE + TRANSIENT_LOCAL, depth 1 | The run state (`IDLE` / `MAPPING` / `RESETTING`) with the keyframe and loop-closure counts; on every transition and at 1 Hz. What the backend's `GET /api/v1/mapping` answers from. |
| `start_mapping` | `syncai_common/srv/StartMapping` | Begin a run (IDLE → MAPPING) — see "The run lifecycle". |
| `save_maps` | `syncai_common/srv/SaveMaps` | Serialise the keyframes and end the run (MAPPING → IDLE) — see below. |
| `reset_mapping` | `syncai_common/srv/ResetMapping` | Start a new map in place (MAPPING → MAPPING) — see below. |

Both cloud outputs are keyframe-triggered, subscriber-gated and rate-floored at
`map_cloud_pub_period` (3 s); with nobody listening on either, no merge runs.

### The map-cloud file hand-off

`map_cloud` stays for rviz, but the console's backend reads `map_cloud_file`
instead. A large site's merge is 16-45 MB, and CycloneDDS over UDP on `lo`
cannot deliver that through the kernel's default 208 KB socket buffer
(`net.core.rmem_max`): fragments drop, a BEST_EFFORT reader loses every
sample, and the console's preview used to stop once the map grew.

So the worker writes the merge as a binary PCD to
`<map_cloud_dir>/map_cloud_<seq>.pcd` (`.tmp` + rename, so a reader never sees
a partial file), keeps the newest two by `seq`, and publishes a notice with
exactly five fields:

```json
{"seq":12,"path":"/dev/shm/syncai_pgo/robot01/map_cloud_12.pcd","points":1830412,"frame_id":"map","stamp":{"sec":1727,"nanosec":0}}
```

Hand-formatted on purpose — this node grows no JSON dependency. The reader
must subscribe TRANSIENT_LOCAL, or nothing is replayed to a backend
(re)started mid-run. `map_cloud_dir` is `/dev/shm/syncai_pgo/<robot_id>` (the
launch appends the robot_id): the host's tmpfs, which the backend's container
sees at the same path only because **both compose services run `ipc: host`**
— without it the notices arrive and every read is ENOENT, and a private
`/dev/shm` is 64 MB, too small for two merges of a large site. A directory the
node cannot create disables the file output for the run and logs an error;
it is deliberately not fatal, because this node also owns the TF broadcast.

**An empty map is a message, not a non-event.** `reset_mapping`,
`start_mapping` and a successful `save_maps` all publish an empty
`PointCloud2` and a `"points":0,"path":""` notice, ungated and *before* the
files are removed; a consumer that skips it keeps showing the map the
operator was just told had been discarded — or, after a save, the map that is
now on disk and no longer live. The save is also what empties
`/dev/shm/syncai_pgo/<robot_id>`: nothing else deletes those files until the
next run, and the backend never does.

### `save_maps`

`file_path` must be an existing directory. Into it the node writes:

| File | Content |
|---|---|
| `map.pcd` | Every keyframe's body cloud placed with its loop-closure-corrected global pose and concatenated. Binary, **not** voxel-filtered. |
| `patches/<i>.pcd` | (with `save_patches`) one body-frame cloud per keyframe; the directory is removed and recreated |
| `poses.txt` | (with `save_patches`) one line per keyframe, `<i>.pcd tx ty tz qw qx qy qz`, bare basenames |

That layout is what the workspace's map catalogue expects under `map/<name>/`
and what `hba_node` (below) refines. Nothing written names the map or holds an
absolute path, which is what makes renaming a map directory one `os.rename`;
keep it that way. The handler holds `m_pgo_mutex` for its whole body, so a
reset that arrives during a save waits for it (and then finds the node idle
and is refused).

**A successful save ends the run.** Once `map.pcd` is written the node goes
IDLE: it frees the keyframes (the map is on disk; a long drive holds hundreds
of MB of them), publishes the empty map — which is what clears `/dev/shm` and
the console's "map so far" — and waits for the next `start_mapping`. Saving
the same run twice is therefore not possible; the backend converts from
`map.pcd` and never needed it. A save that fails (missing directory, no
keyframes, a write error) changes nothing; the write errors are caught and
reported as `success: false` with the run kept — they used to throw out of
the callback. `poses.txt` is now only opened when `save_patches` is true.

`pgo_node` holds its keyframes in RAM and this is the **only** thing that
writes them out. Whatever has not been saved when the process ends — or when
`reset_mapping` runs — is gone short of replaying a bag. That is why the
mapping session's `lio` window is not optional, and why a mode switch refuses
to rebuild the live mode.

### The run lifecycle

Since 2026-10 the node has an explicit run state, `NodeState::phase`, one of
`syncai_common/msg/MappingStatus`'s constants and latched on
`mapping_status`:

| State | Event | Next | What happens |
|---|---|---|---|
| — | launch | `IDLE` (`MAPPING` with `start_on_launch`) | `/dev/shm` cleared, status published |
| `IDLE` | a cloud/odom pair | `IDLE` | dropped before the copy; identity `map → local_frame` broadcast |
| `IDLE` | `start_mapping` | `RESETTING` → `MAPPING` | the four phases below, from the other precondition |
| `IDLE` | `reset_mapping` / `save_maps` | `IDLE` | refused: nothing to discard / nothing to save |
| `MAPPING` | a pair | `MAPPING` | the normal path |
| `MAPPING` | `reset_mapping` | `RESETTING` → `MAPPING` | discard and start over, as always |
| `MAPPING` | `start_mapping` | `MAPPING` | refused: "already mapping" — a stale console tab must not be able to discard a run with the wrong button |
| `MAPPING` | `save_maps` succeeds | `IDLE` | files written, keyframes freed, empty map published, `/dev/shm` cleared |
| `MAPPING` | `save_maps` fails | `MAPPING` | nothing changes |
| `RESETTING` | anything | `RESETTING` | pairs dropped silently; every service refused ("a start or reset is already running") |

A failed start or reset puts the phase it left back. The graph itself is never
in a half state: the only fallible step runs before anything is destroyed.

`start_mapping` and `reset_mapping` share one body (`beginRun`), four phases
ordered so that the only step that can fail runs before anything is
destroyed:

1. **Pause.** Under a brief `m_pgo_mutex` hold the precondition is checked
   (IDLE for a start, MAPPING for a reset — exact, because the save stores
   IDLE under the same mutex) and the phase goes `RESETTING`: nothing the
   front end publishes can reach the graph while pointlio's state changes
   underneath. This is the whole ordering fix — no sleep, no slack window;
   the odometry discontinuity has nowhere to land. A second concurrent call
   is rejected outright (`m_transitioning`), and an RAII guard puts the
   previous phase back on every early exit.
2. **Reset the front end** (`reset_lio: true`, what the console sends): call
   `pointlio/reset` over `syncai_common/srv/ResetLIO` and wait up to 5 s,
   **without** `m_pgo_mutex`, so the TF broadcast does not gap. The LIO answers
   with `last_odom_time`, the lidar stamp of the last odometry of the old run.
   Any failure returns here with the state intact and says so.
3. **Rebuild the graph** under the mutex (`stopRunLocked`, shared with the
   save): join an in-flight merge (so the old map cannot be published after
   the empty one), construct a fresh `SimplePGO` (the constructor *is* the
   reset — `gtsam::ISAM2` has no clear), swap the buffer, zero the counters,
   publish the empty map and a DELETEALL for the markers; then set
   `accept_after_time` and the phase `MAPPING`.
4. **Resume.** Pairs stamped `<= accept_after_time` — the old run's tail still
   sitting in the synchroniser — are dropped by `syncCB`. With a LIO reset
   that boundary is `last_odom_time`; a start without one gates on the last
   pair seen, so the run begins at the call; a reset without one keeps its
   historical `0.0`. Both sides of the comparison are the lidar header stamp,
   so it is exact.

**Why a start is a reset.** The graph's origin is the odometry origin
(`SimplePGO` anchors its first keyframe at `offset × local pose`), so "begin
a clean run from where the robot stands" is literally true only when pointlio
is reset too — which also frees the ikd-tree it has been growing since the
session came up.

**The robot must be standing still** for either. pointlio re-runs a static,
gravity-aligning IMU initialisation after its reset — publishing nothing until
it is done, so the live scan goes dark for those seconds and returns; one done
in motion tilts the new map for its whole life and reports no error anywhere.
Nothing in either node enforces that, by decision (a stationarity check
cannot tell a still robot from a vibrating one). Both response `message`s are
written as UI copy for exactly that reason — they are the last place the
warning can land.

`reset_lio: false` resets the graph alone over an unchanged odometry stream: a
bag-replay affordance, not something the console sends. The `.srv` comments in
`syncai_common` (`StartMapping.srv`, `ResetMapping.srv`, `ResetLIO.srv`) and
`MappingStatus.msg` are the specification.

### Threading

`main.cpp` runs a 3-thread `MultiThreadedExecutor` with three callback groups,
and that is correctness, not performance: the default group keeps the timer,
both subscriptions, the 1 s status timer and `save_maps` serialised as
`rclcpp::spin()` did; a second group runs `start_mapping` and `reset_mapping`
(MutuallyExclusive, so they serialise), which block on the `ResetLIO` future;
a third owns that client, so the future's response can be delivered while the
handler waits. Hence `m_pgo_mutex`, which guards `m_pgo` — a start, a reset
and a save all *replace* it rather than mutating it, so every reader takes
the lock:

| | holds `m_pgo_mutex` |
|---|---|
| `timerCB` | whole body |
| `saveMapsCB` | whole body, including the save → IDLE transition at its end |
| `beginRun` (start / reset) | phase 1 (a few lines) and phase 3 — never while waiting on the LIO future |
| `publishStatus` / `statusTimerCB` | nothing: the phase and the two counters it reports are atomics |

The intake gate is atomics, not a lock, so `syncCB` never has to order two
mutexes on its hot path; every *store* to the phase happens under
`m_pgo_mutex`, which is what makes the service preconditions exact. The merge runs on a dedicated `std::thread` from a
by-value snapshot of the keyframes, claimed by an atomic (`m_map_cloud_busy`)
and joined only while that is false; every exception is caught inside the
worker, because one escaping a `std::thread` is `std::terminate` — pgo gone,
TF gone, mid-run. `main()` holds a *named* `shared_ptr` because
`Executor::add_node` keeps a weak_ptr; a temporary would leave the executor
spinning over nothing while looking alive in `ps`.

## Parameters

`params/mapping_params.yaml`, keyed `/**/pgo_node:` so it applies at any
namespace. Every key is a `declare_parameter` whose default repeats the struct
default, so a missing key degrades to the code default instead of throwing —
a change from the yaml-cpp loader, where most keys were required. Read once:
the values are copied into the graph at construction and on every reset, so
`ros2 param set` afterwards changes nothing.

| Parameter | Default (YAML) | Launch value | Notes |
|---|---|---|---|
| `cloud_topic` | `/pointlio/body_cloud` | `/<robot_id>/pointlio/body_cloud` | absolute; see Inputs |
| `odom_topic` | `/pointlio/lio_odom` | `/<robot_id>/pointlio/lio_odom` | |
| `lio_reset_service` | `/pointlio/reset` | `/<robot_id>/pointlio/reset` | the `ResetLIO` client |
| `map_frame` | `map` | | never prefixed |
| `local_frame` | `lio_odom` | `<robot_id>/pointlio_odom` | must equal pointlio's `world_frame` |
| `map_cloud_dir` | `/dev/shm/syncai_pgo` | `<map_cloud_dir arg>/<robot_id>` | the launch's `map_cloud_dir:=` argument is the base |
| `key_pose_delta_deg` / `key_pose_delta_trans` | `10.0` / `0.5` | | keyframe every 10° or 0.5 m. **Write `10.0`**: a bare `10` is an int64 override and the node dies at startup with `InvalidParameterTypeException`. |
| `loop_search_radius` | `1.0` | | m, over past keyframes |
| `loop_time_tresh` | `60.0` | | s; candidates must be at least this old |
| `loop_score_tresh` | `0.15` | | ICP fitness (mean squared point distance, m²) above this is rejected |
| `loop_registration` | `gicp` | | `gicp` (small_gicp) or `icp` (the fork's PCL point-to-point). See "Loop verification" below |
| `loop_icp_max_corr_dist` | `1.0` | | m, max correspondence distance for either backend. The fork hard-coded 10 |
| `loop_gicp_num_threads` | `4` | | **int** |
| `loop_gicp_num_neighbors` | `20` | | **int**, GICP covariance neighbours |
| `loop_planar_correction` | `true` | | loop edge is a `PlanarLoopFactor` on the world-frame x / y / yaw only (`pgos/planar_loop_factor.h`); z / roll / pitch stay with the odometry. `false` = the fork's 6-DOF `BetweenFactor` |
| `loop_noise_var_roll_pitch_z` | `0.01` | | only with `loop_planar_correction: false`: BetweenFactor variance on roll / pitch / z. x / y / yaw always use the fitness score, as the fork did |
| `loop_submap_half_range` | `5` | | keyframes each side of the candidate. **An int** — `5.0` is the mirror-image type error. |
| `submap_resolution` | `0.1` | | m, voxel leaf of the ICP submap |
| `min_loop_detect_duration` | `5.0` | | s between loop searches |
| `map_cloud_resolution` | `0.2` | | m, voxel leaf of the published merge |
| `map_cloud_pub_period` | `3.0` | | s, floor between merges |
| `start_on_launch` | `false` | `start_on_launch:=` argument | `true` comes up MAPPING instead of IDLE: the bag-replay knob, never the session's value |

### Loop verification

A loop candidate (a past keyframe within `loop_search_radius`, at least
`loop_time_tresh` old) is verified by registering the current keyframe's body
cloud onto an 11-keyframe submap, both voxelised at `submap_resolution`, and
the result becomes the loop edge. Three things about that edge changed in
2026-10, after an offline replay of `record/dp1f_1002` (a 645 s, 100 m
corridor-and-hall run; the report and every number below are under
`map/dp1f_bagrun/report/`):

- **Backend.** The fork's PCL point-to-point ICP with
  `setMaxCorrespondenceDistance(10)` returned 1.5–3.7° of rotation and a z
  component 2–7× too large for all seven corridor loops of that run, although
  the raw Point-LIO poses of the two passes already agreed to 1–5 cm in z;
  the two end-of-run loops it got right scored 0.006–0.013 against 0.06–0.13
  for the bad ones, so `loop_score_tresh 0.15` rejected nothing (a corridor
  scan with no ICP at all scores 0.09–0.19, because most of its far points
  have no counterpart). Re-running those pairs offline, GICP / point-to-plane
  with a 1 m correspondence distance returned the right amount. Hence
  `loop_registration: gicp` (small_gicp's `RegistrationPCL`, the localizer's
  backend) and `loop_icp_max_corr_dist: 1.0`; `icp` keeps the old path.
- **Edge type.** The 6-DOF `BetweenFactor` let a slightly tilted loop bend the
  whole chain: with per-edge rotation variance 1e-6 and 30–60 m of lever arm
  the chain is softer in z than a loop factor whose six variances all equal
  the fitness, so the map came out with a 15 cm double floor along the
  corridor while every per-edge z constraint was honoured to 0.2 mm. Tight
  roll / pitch / z variances on the BetweenFactor were tried first and still
  leaked x into z (-2..-9 cm) because that factor's error is expressed in
  the lidar body frame, which on this robot is pitched 15°.
  `loop_planar_correction: true` therefore adds a `PlanarLoopFactor`
  (`pgos/planar_loop_factor.h`): a 3-dim error on the WORLD-frame Δx, Δy,
  Δyaw between the two keyframes, with the registration result projected to
  x / y / yaw about the world z axis first. z / roll / pitch stay with the
  odometry, which for a gravity-aligned LIO is the right owner. Rerun on the
  same bag: revisit height error identical to the raw LIO (3.5 cm mean),
  keyframe z within 2 cm of raw for 832 of 840 keyframes, single floor.
- **Log.** Every accepted loop prints two `[PGONode][loop]` lines (below), so
  the next bad closure can be read off the log instead of reconstructed from
  rviz markers.

Still open: the fitness gate counts all source points and so cannot tell a
good corridor loop from a bad one (inlier RMSE + inlier ratio would), and the
run above had little XY drift, so the x / y / yaw benefit of the loops is
unmeasured — validate on a bag that drifts.

## Running

```bash
ros2 launch syncai_mapping mapping.launch.py                        # config/system.ini from ~/robot_ws
ros2 launch syncai_mapping mapping.launch.py system_config:=/path/to/robot01.ini
ros2 launch syncai_mapping mapping.launch.py map_cloud_dir:=/dev/shm/syncai_pgo   # the base; /<robot_id> is appended
ros2 launch syncai_mapping mapping.launch.py start_on_launch:=true                # bag replay: no start_mapping needed
```

The mapping session runs it as the second pane of the `lio` window in
`start_mapping.yaml`, right after `syncai_pointlio`; the log is
`log/stack/<robot_id>/mapping/pgo/` (the directory keeps the node's name, not
the package's). Launching this alone starts no LIO — run
`syncai_pointlio` first.

The shell fallback for a run with no backend:

```bash
ros2 service call /<robot_id>/pgo/start_mapping syncai_common/srv/StartMapping "{reset_lio: true}"   # robot still; drive once the live scan is back
ros2 service call /<robot_id>/pgo/save_maps syncai_common/srv/SaveMaps \
  "{file_path: '/home/syncrobotic/robot_ws/map/<name>', save_patches: true}"   # directory must exist; ends the run
ros2 service call /<robot_id>/pgo/reset_mapping syncai_common/srv/ResetMapping "{reset_lio: true}"   # mid-run only; robot still!
```

Checking it:

```bash
ros2 node info /<robot_id>/pgo/pgo_node                # subs /<id>/pointlio/{body_cloud,lio_odom}; srvs start_mapping save_maps reset_mapping
ros2 param get /<robot_id>/pgo/pgo_node local_frame    # <robot_id>/pointlio_odom
ros2 param get /<robot_id>/pgo/pgo_node loop_score_tresh   # 0.15 → the params file was found
ros2 service type /<robot_id>/pgo/save_maps            # syncai_common/srv/SaveMaps
ros2 topic echo /<robot_id>/pgo/mapping_status --qos-durability transient_local --qos-reliability reliable   # state 0 = idle, 1 = mapping
ros2 run tf2_ros tf2_echo map <robot_id>/pointlio_odom # identity while idle
ros2 topic echo /<robot_id>/pgo/map_cloud_file --qos-durability transient_local --qos-reliability reliable
ls /dev/shm/syncai_pgo/<robot_id>                      # map_cloud_<seq>.pcd, newest two; empty while idle
```

| Log line | Meaning |
|---|---|
| `[PGONode] inputs: /robot01/pointlio/body_cloud + … \| TF map -> robot01/pointlio_odom \| …` | The overrides landed. Unprefixed names mean a bare run, or overrides passed before the params file. |
| `[PGONode] map_cloud merges are handed off as PCD files under …` | The hand-off directory is writable |
| `[PGONode] loop verification: gicp, max corr dist 1.00 m, fitness gate 0.150, planar correction on, …` | The loop settings that are actually in force (see "Loop verification") |
| `[PGONode][loop] target 44 -> source 252 \| fitness 0.0995 \| ICP moves source by (0.065, -0.024, 0.000) m, rot 0.21 deg \| z before: …` | A loop was accepted: what the registration asked for (world-frame displacement of the source keyframe and rotation) and both keyframes' z before the graph update. A z component that is not 0.000 with planar correction on, or a rotation of degrees rather than tenths, is the failure this line exists to catch |
| `[PGONode][loop] target 44 -> source 252 \| z after: … (source-target -0.047, source moved -0.000)` | The same pair after the update. `source moved` is the z the optimiser actually applied |
| `[PGONode] cannot create map_cloud_dir … the map_cloud_file hand-off is disabled for this run` | Not fatal; only the file output is off |
| `Received out of order message` | A pair older than the last accepted one was dropped (bag loop, driver restart) |
| `[PGONode] idle until start_mapping is called; …` | Normal at launch. A session that never calls Start maps nothing |
| `[startMappingCB] Mapping started. …` | A run began; pointlio is re-initialising — the robot must be still |
| `[resetMappingCB] Map discarded. … (dropped N key poses)` | A reset completed; N = 0 means the run had banked nothing |
| `[saveMapsCB] Map saved (N keyframes). Mapping stopped … (<dir>/map.pcd)` | The run ended; `/dev/shm` is empty and the node is idle |
| `LIO reset service … is not available` / `Timed out waiting for the LIO reset` | Start or reset refused (`nothing started` / `map kept`) — pointlio is down, or built against the old `interface` type |

## Gotchas

- **Nothing here changes the ROS surface, and nothing should without a
  cross-repo commit.** The backend calls `start_mapping` / `save_maps` /
  `reset_mapping`, reads `mapping_status`, `map_cloud_file` and the PCDs
  under `/dev/shm/syncai_pgo/<robot_id>`, and expects `map/<name>/`'s layout;
  `hba_node` expects `patches/` + `poses.txt`. The five absolute names in
  `mapping.launch.py` are the contract with
  `syncai_pointlio/launch/pointlio.launch.py`.
- **pgo comes up idle.** A mapping session with no `start_mapping` call maps
  nothing, silently (the launch log says so once). A backend older than the
  Start control therefore cannot map against this node — ship the two
  together, or run with `start_on_launch:=true` meanwhile. The shell fallback
  is the service call under "Running".
- **The service types are `syncai_common`'s, not `interface`'s.** They moved
  with the port. A backend still importing `interface.srv.SaveMaps` /
  `ResetMapping` finds no server: save-map and "New map" fail with a type
  mismatch, cleanly, until it switches to `syncai_common.srv`. Rebuild
  `syncai_common syncai_pointlio syncai_mapping` together, and clear the stale
  `install/pgo` / `install/interface` from before the port so the old headers
  and `pgo_launch.py` cannot be found by mistake.
- **`local_frame` must equal pointlio's `world_frame`.** Both are launch
  overrides now, in two launch files; if they drift, `map → local_frame` is
  broadcast onto a frame nobody looks up and the console's map view floats.
- **Types in the params file.** Every tuning value is a double except
  `loop_submap_half_range`; a bare integer for a double key, or a float for
  the int key, kills the node at startup. A misspelt key no longer throws —
  it silently uses the default — so read the startup log.
- **GTSAM is a manual dependency.** 4.2.0, source-built into `/usr/local` by
  the `Dockerfile`'s `deps-builder` stage (system Eigen, TBB on). There is no
  rosdep key for that build, so `package.xml` does not list it — the same
  treatment `syncai_pointlio` gives Sophus. Recreating the container from the
  image keeps it; a hand-built one loses it.
- **An unoptimised build cannot keep up.** CMake defaults `CMAKE_BUILD_TYPE`
  to Release (upstream forced it); iSAM2 plus an ICP verification per keyframe
  and a full-map merge on the worker are not `-O0` workloads on the Jetson.
- **Lint is partly off.** `ament_cmake_copyright` and `cpplint` are disabled
  as they were upstream: `pgos/` has no per-file licence headers (the package
  `LICENSE` covers it). All four `pgos/` files and the node shell follow the
  workspace `.clang-format`.
- `rviz/pgo.rviz` and `rviz/hba.rviz` from the fork were not carried over
  (the former's fixed frame was the upstream `lidar`); rviz configs live in
  `config/rviz2/`.

## hba_node: offline refinement

The second node: hierarchical bundle adjustment ([HBA](https://github.com/hku-mars/HBA)
/ [BALM](https://github.com/hku-mars/BALM)) over the `patches/` + `poses.txt`
pair a `save_maps` with `save_patches: true` wrote. It is **not in any
session** and nothing in the backend calls it; an operator runs it by hand
after a mapping drive, on a workstation or in the container, and decides
whether to adopt the refined poses. Ported from the fork's `hba` package in
2026-09 — the maths in `hba/` (`blam.*`, `hba.*`, `commons.*`) is upstream
code, formatted but otherwise untouched; `hba_node.*` is the ROS shell.
Three things changed with the port: the node runs at `/<robot_id>/hba` (it
had a bare `/hba`, the only node in the stack without the robot_id
namespace), its config is ROS parameters instead of a yaml-cpp `config_path`,
and the launch no longer starts an rviz2.

| Surface | Kind | Notes |
|---|---|---|
| `refine_map` | `syncai_common/srv/RefineMap` (`maps_path`) | Loads every `patches/<i>.pcd` named in `poses.txt` (voxelised at `scan_resolution`) and schedules the optimisation. **Returns as soon as the patches are loaded**; the optimisation runs on the 100 ms timer afterwards, `hba_iter` full passes, and holds the executor while it does — fine for an offline node that serves nothing else. |
| `save_poses` | `syncai_common/srv/SavePoses` (`file_path`) | Writes the refined poses in the `poses.txt` format to a separate file whose parent directory must exist. Replacing the map's own `poses.txt` is deliberately the operator's step, not the node's. |
| `map_points` | `sensor_msgs/PointCloud2` in `map`, depth 10 | The refined map, republished after every pass. Subscriber-gated. |

```bash
ros2 launch syncai_mapping hba.launch.py
ros2 service call /<robot_id>/hba/refine_map syncai_common/srv/RefineMap "{maps_path: '/home/syncrobotic/robot_ws/map/<name>'}"
# watch /<robot_id>/hba/map_points; the log prints ======HBA ITER n START/END======
ros2 service call /<robot_id>/hba/save_poses syncai_common/srv/SavePoses "{file_path: '/home/syncrobotic/robot_ws/map/<name>/poses_refined.txt'}"
```

Parameters (`params/hba_params.yaml`, keyed `/**/hba_node:`; none depend on
robot_id, so the launch passes the file through unchanged; read once at
startup):

| Parameter | Default | Type | Notes |
|---|---|---|---|
| `scan_resolution` | `0.1` | double | voxel leaf on load and for the preview |
| `window_size` / `stride` | `20` / `10` | int | sliding window of poses per local BA, and its step |
| `voxel_size` / `min_point_num` / `plane_thresh` | `0.5` / `10` / `0.01` | double / int / double | plane-feature voxelisation for the local BA |
| `max_layer` | `3` | int | hierarchy depth |
| `ba_max_iter` / `hba_iter` | `10` / `5` | int (`size_t` in the code, floored at 0) | LM iterations per local BA; full passes |
| `down_sample` | `0.1` | double | voxel leaf of the clouds handed to the local BA |

The same int-vs-double trap as `pgo_node`'s file applies. Two more things
worth knowing: `MP_PROC_NUM=4` (BALM's OpenMP thread count) is compiled in
for this target only and sized for an offline job with the box to itself,
and Sophus is this node's manual dependency, the same source build
`syncai_pointlio` uses.
