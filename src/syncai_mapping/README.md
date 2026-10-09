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
same TF. After a successful save it spawns `build_octomap`, this package's
third executable, which ray-casts the saved patches into an OctoMap and
exports the two layers the console's "3D map" draws — detached, so it
outlives the session (see "The OctoMap build").

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
        │                                                        │
        │                                          spawns, detached ▼
        │                                                  build_octomap ──► map/<name>/octomap.bt,
        │                                                                    octomap_{road,occupied}.pcd,
        │                                                                    octomap.recipe.json (status)
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
| `save_maps` | `syncai_common/srv/SaveMaps` | Serialise the keyframes and end the run (MAPPING → IDLE), then start the OctoMap build — see below. |
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
| `octomap.recipe.json` | (with `save_patches` and `octomap_enabled`) the OctoMap build's status, written `converting` before the response goes out — see "The OctoMap build" |
| `octomap.bt`, `octomap_road.pcd`, `octomap_occupied.pcd` | written minutes later by `build_octomap`, not by this handler |

A save with `save_patches` also deletes the previous `octomap.*` files and
any `octomap*.tmp` first: their input is being replaced.

That layout is what the workspace's map catalogue expects under `map/<name>/`
and what `hba_node` (below) refines. Nothing written names the map or holds an
absolute path — the sidecar included — which is what makes renaming a map
directory one `os.rename`; keep it that way. The handler holds `m_pgo_mutex` for its whole body, so a
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
The response `message` (UI copy, rendered verbatim) ends with one sentence
about the 3D map: being built, not built (no patches), or could not start.
None of those turns the save into a failure — the map is on disk.

`pgo_node` holds its keyframes in RAM and this is the **only** thing that
writes them out. Whatever has not been saved when the process ends — or when
`reset_mapping` runs — is gone short of replaying a bag. That is why the
mapping session's `lio` window is not optional, and why a mode switch refuses
to rebuild the live mode.

### The OctoMap build

Once a save has ended the run, the handler starts one OctoMap build of the
directory it just wrote: every patch ray-cast from its keyframe pose into an
`octomap::OcTree`, written as `octomap.bt`, and two point layers derived from
it for the operator console's "3D map" — nothing during a run changes, the
live preview is still the map cloud above. It ran by hand from
`scripts/octomap/` until 2026-10 (that prototype is gone; this is its code,
same insertion order and arithmetic, verified byte-identical on the `.bt`).

**Input: patches + poses, not `map.pcd`.** `map.pcd` has no sensor origins,
so OctoMap could only mark its points occupied and every free voxel would stay
unknown. Free space — the part that says where the floor is clear — exists
only if each scan is ray-cast from where it was taken, which is what
`poses.txt` gives every patch. Hence no OctoMap for a save without
`save_patches`. The ray origin is the keyframe pose, i.e. the IMU origin; the
lidar is 5.1 cm away (`syncai_pointlio`'s `t_il`), and since both rays end at
the same point that offset is under one voxel and inside `min_range` anyway.

Each patch goes through `insertPointCloud` with `discretize` on (a voxel hit
by several rays of one scan is updated once), rays truncated at `max_range`
(free space is carved only that far, and a point beyond it is not marked
occupied) and returns under `min_range` dropped (the robot's own body). The
tree is pruned and written as a binary `.bt` — max-likelihood occupancy
only, what octovis and `octomap_server` load.

**The two layers.** The free voxels themselves are useless to look at: every
one is air a ray passed through (42.6 M of them on `dp1f_1006` at 0.05 m) and
they bury the floor. So:

| File | Holds |
|---|---|
| `octomap_road.pcd` | per column, the **lowest free voxel within ±`floor_band` of the local floor** — observed-free space at ground level |
| `octomap_occupied.pcd` | occupied voxels from local floor − `floor_band` up to + `max_height` — walls, with the ceiling cut |

Both are binary `pcl::PointXYZ` (12 B a point: `FIELDS x y z`), voxel centres
in the `map` frame — what the backend's PCD reader and its point-cloud wire
format already take.

The *local* floor is the nearest keyframe's z minus `lidar_height`, looked up
per 1 m cell within `floor_radius`. One global floor level does not work:
`dp1f_1006`'s floor changes height by about 1 m across the site. A 4 m radius
left square holes in a hall wider than 8 m; 10 m does not.

The obvious road definition, "a free voxel with an occupied voxel under it",
is wrong, and was tried first: it found 391 m² on `dp1f_1006`, the lowest-free
one 4338 m². Rays to distant floor points cross the near floor at a grazing
angle and integrate misses into it, so the floor itself ends up free, and the
voxel under the lowest free one is usually *unknown* (never seen from below),
not occupied. Expect free floor beyond the walls too: rays through windows
and doorways observe it, and from above it shows as radial fans. That is also
why the road layer is **not a traversability map** — it is observed-free
space at floor height (under tables and outside included), with no slope or
step check. The drivable area is still the gridmap's.

An empty layer fails the build ("road layer is empty: lidar_height /
floor_band do not match this save"): it means the band missed the floor — a
`lidar_height` for another robot — and pcl cannot write an empty PCD anyway.

**Status: `octomap.recipe.json`**, the only surface — modelled on the
backend's `gridmap.recipe.json`, written `.tmp` + rename (a per-process temp
name, since this node and the build both write it):

```json
{"status":"converting","started_at":"2026-10-08T03:12:45Z","params":{"resolution":0.1,"max_range":20,"min_range":0.5,"lidar_height":0.481,"floor_band":0.25,"max_height":2,"floor_radius":10}}
{"status":"ok","started_at":"…","finished_at":"…","params":{…},"measurements":{"keyframes":1195,"skipped_patches":0,"points":3050000,"leaves":…,"occupied_leaves":…,"free_leaves":…,"road_voxels":…,"road_area_m2":4338,"occupied_voxels":…,"elapsed_s":420.3}}
{"status":"failed","started_at":"…","finished_at":"…","params":{…},"error":"road layer is empty: …"}
```

`converting` is written by this node **before** it spawns and before the save
responds, so a reader acting on the response always finds it. A build killed
hard (OOM, SIGKILL, `docker stop`) leaves it at `converting` for good —
`started_at` is there so a reader can age that into "interrupted"; this node
does not retry. Error text names files relative to the directory only. The
outputs are renamed into place together at the end, so a failed rebuild
leaves the previous ones (the sidecar's `failed` outranks them).

**A process, not a thread.** `posix_spawn` of
`<prefix>/lib/syncai_mapping/build_octomap` (found through the package index)
with `POSIX_SPAWN_SETSID` and the stop signals reset to default. Each reason
below decides it alone:

- *Memory.* A build is tens of millions of small octree nodes; in this
  process that heap would mostly never go back to the OS. The child also
  writes `oom_score_adj 500`, so under memory pressure the kernel kills the
  build rather than the node owning the TF and maybe an unsaved run.
- *Lifetime.* `switch_mode` kills the byobu session — a pty hang-up, SIGHUP,
  no destructor. Saving and switching straight to AUTO is the normal operator
  flow, so the build has to outlive this process: in its own session it has
  no controlling terminal to lose, and it ignores SIGPIPE so printing into a
  dead pane is harmless.
- *One entry point.* The same executable is the by-hand rebuild.

It runs at `nice 10`, one core (the apt `liboctomap` has no OpenMP), and its
progress goes to this node's pane log while the pane exists. This node reaps
it from the 1 s status timer (`waitpid` WNOHANG, one log line per outcome);
its destructor neither kills nor waits. Once this node is gone the build is
reparented to the container's PID 1 — `init: true` in the robot compose
service makes that tini, which reaps it.

Builds are serialised per host by a `flock` on
`/dev/shm/syncai_pgo/build_octomap.lock` (`/dev/shm` is shared by every
container on the robot through `ipc: host`): two saves in a row start two
builds, and the second waits — its sidecar says `converting` meanwhile.

| Event | `pgo_node` | `build_octomap` | sidecar |
|---|---|---|---|
| Ctrl-C in the pane | destructor logs "continues detached" | unaffected | finishes |
| `switch_mode` / `restart_mode` | gone (SIGHUP) | unaffected | finishes |
| `pkill -TERM build_octomap` | logs the exit | stops between keyframes, `.tmp`s removed | `failed: interrupted (SIGTERM)` |
| OOM | survives | killed first | stuck at `converting` |
| `docker stop` / reboot | gone | gone | stuck at `converting`; rebuild by hand |

Known edge, not handled: a second save into a directory whose build is still
running. The backend never saves into an existing map directory.

**By hand** — a rebuild of any saved map, with other parameters, or on a
workstation with only PCL and OctoMap (it is not a ROS node):

```bash
ros2 run syncai_mapping build_octomap map/<name>                      # the defaults below
ros2 run syncai_mapping build_octomap map/<name> --resolution 0.05 --lock none
#   --max-range 20 --min-range 0.5 --lidar-height 0.481 --floor-band 0.25
#   --max-height 2.0 --floor-radius 10 --nice 0 --started-at <ISO-8601>
# exit 0 ok · 1 usage · 2 no poses.txt / readable patch · 3 build or write failed · 4 interrupted
cat map/<name>/octomap.recipe.json
ps -o pid,ppid,sid,ni,rss,etime,cmd -C build_octomap
```

**Numbers.** `dp1f_1006` (853 s bag, 477 m path) with the prototype at
0.05 m, single-threaded in Docker on a Mac:

| | |
|---|---|
| Keyframes / points inserted | 1195 / 3.05 M |
| Leaves | 44.3 M (1.73 M occupied, 42.6 M free) |
| `.bt` | 25 MB |
| Road layer | 1.74 M voxels, 4338 m² |
| Wall time / peak RSS | about 7 min / about 4.5 GB |

Hence the 0.1 m default: roughly 1/8 the free leaves, a quarter of the road
points (what the console downloads), and a fraction of the memory — a display
layer does not need the gridmap's 0.05 m. Measure the Jetson at 0.1 m and
record it here.

**Viewing the `.bt` itself.** Homebrew's `octomap` ships no octovis; a native
build was made from OctoMap v1.10.0 against Homebrew `qt@5` (build the bundled
libQGLViewer with `qmake`, pass its `QGLViewer.framework` to CMake as
`QGLViewer_LIBRARY_DIR_OTHER`). Leave free voxels hidden or they bury the
floor. Open3D's WebRTC viewer was tried and dropped: it exists only in the
linux/x86_64 wheel, which on Apple Silicon means Rosetta, Xvfb and software GL,
and it never rendered.

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
TF gone, mid-run. The post-save OctoMap build is deliberately not a fourth
thread but a process (see "The OctoMap build"); the only state it leaves in
here is the list of spawned pids, touched by `saveMapsCB` and the status
timer — both in the default group, so it needs no lock. `main()` holds a *named* `shared_ptr` because
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
| `loop_noise_var_roll_pitch_z` | `0.01` | | only with `loop_planar_correction: false`: BetweenFactor variance on roll / pitch / z. x / y / yaw use the fitness score, as the fork did |
| `loop_noise_yaw_sigma_deg` | `1.5` | | only with `loop_planar_correction: true`: the `PlanarLoopFactor`'s yaw sigma, **degrees**; its x / y keep variance = fitness. The fork's fitness-as-rad² yaw (an 18-22 deg sigma) was outvoted by ~1 deg of accumulated odometry, so a loop never corrected yaw (dp1f_1008_2's east corridor: 8 deg asked, none applied, passes left 1.1-1.6 m apart). `<= 0` restores that |
| `loop_noise_xy_sigma_m` | `0.05` | | only with `loop_planar_correction: true`: the `PlanarLoopFactor`'s x / y sigma, **metres**. Fitness-as-m² (a 0.25-0.39 m sigma) left dp1f_1008_2's east corridor 0.4-0.5 m apart even with the yaw fixed; 0.1 m left 0.08-0.11 m, 0.05 m 0.01-0.04 m, and dp1f_1006 stays within 8 cm at either. `<= 0` = variance = fitness |
| `keyframe_tilt_sigma_deg` | `0.5` | | **degrees**; a `GravityPriorFactor` (`pgos/gravity_prior_factor.h`) on every keyframe holding its roll / pitch w.r.t. gravity to Point-LIO's value. Needed by the yaw sigma above: without it a yaw correction is realised partly as pitch and the chain climbs or sinks (dp1f_1008_2: keyframe z moved 0.8 m median, 2.2 m worst). `<= 0` = off |
| `loop_submap_half_range` | `5` | | keyframes each side of the candidate. **An int** — `5.0` is the mirror-image type error. |
| `submap_resolution` | `0.1` | | m, voxel leaf of the ICP submap |
| `min_loop_detect_duration` | `5.0` | | s between loop searches |
| `map_cloud_resolution` | `0.2` | | m, voxel leaf of the published merge |
| `map_cloud_pub_period` | `3.0` | | s, floor between merges |
| `start_on_launch` | `false` | `start_on_launch:=` argument | `true` comes up MAPPING instead of IDLE: the bag-replay knob, never the session's value |
| `octomap_enabled` | `true` | | spawn the OctoMap build after a save with `save_patches`; `false` spawns nothing and writes no sidecar |
| `octomap_resolution` | `0.1` | | m, voxel edge. The prototype's 0.05 cost 4.5 GB on `dp1f_1006` |
| `octomap_max_range` / `octomap_min_range` | `20.0` / `0.5` | | m; rays truncated / returns off the robot's body dropped |
| `octomap_lidar_height` | `0.481` | | m, lidar above the floor; keyframe z minus this is the local floor. Wrong → empty road layer → `failed` |
| `octomap_floor_band` / `octomap_max_height` | `0.25` / `2.0` | | m; road band around the local floor / top of the occupied layer |
| `octomap_floor_radius` | `10.0` | | m; how far to look for the keyframe that gives a cell its floor |
| `octomap_nice` | `10` | | **int**, the build's nice value |

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
cat map/<name>/octomap.recipe.json                     # after a save: converting → ok / failed
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
| `[PGONode] octomap after save: on \| 0.100 m, range 0.5-20.0 m, …` | The OctoMap settings in force |
| `[PGONode][octomap] build_octomap pid N started for <dir> (0.100 m, nice 10); outcome in octomap.recipe.json` | A save started a build |
| `[build_octomap] 100/1195 keyframes, 255000 points` … `[build_octomap] DONE keyframes=… road_voxels=… (… m2) …` | The build's own progress, printed into this pane while it exists |
| `[PGONode][octomap] build_octomap pid N finished <dir> in N s` | Reaped, exit 0 |
| `[PGONode][octomap] build_octomap pid N exited 3 …` / `… killed by signal 9 … (OOM?)` | Failed — the sidecar has the reason; after a signal it is left at `converting` |
| `[PGONode][octomap] build_octomap executable not found` | Not installed (a partial build); the sidecar says `failed` |
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
- **OctoMap is not a manual dependency.** Unlike GTSAM, `octomap` is a real
  rosdep key (`ros-humble-octomap`), installed by the `Dockerfile`'s dev
  stage. A container from an older image lacks it and `syncai_mapping` fails
  to configure; `sudo apt-get install ros-humble-octomap` until it is
  recreated.
- **The build outlives the session, by design.** A save followed by a mode
  switch is the normal case; `ps -C build_octomap` in the robot container
  shows it running on with PPID 1. A sidecar still `converting` with no such
  process means it was killed — rerun it by hand.
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
