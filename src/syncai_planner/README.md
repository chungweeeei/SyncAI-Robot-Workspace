# syncai_planner

The global planning half of the nav stack: an action server that owns the
**global costmap** and hosts `GlobalPlanner` plugins. Port of `nav2_planner`,
with `nav2_navfn_planner` and `nav2_smac_planner` merged in as plugins of this
package rather than living in their own.

```
syncai_task_runner ──ComputePathToPose (nav2_msgs)──►  planner_server
                   ──ComputePathThroughPoses────────►       │
                                                            │  global_costmap (internal node)
                              ┌───────────────┬─────────────┴─┐
                              ▼               ▼               ▼
                       SmacPlanner2D    NavfnPlanner   StraightLinePlanner
                      (cost-aware A*)  (Dijkstra / A*)  (reference/testing)
                              │
                              ▼
                     nav_msgs/Path ──► syncai_controller (FollowPath)
```

`SmacPlanner2D` is the configured planner; the other two are built and
registered but not loaded.

Two structural notes carry over from the rest of the stack:

1. **`PlannerServer` is a plain `rclcpp::Node`.** Setup is split between the
   constructor (declare parameters, construct the costmap object) and
   `configure()`, which `main.cpp` calls once the node is owned by a
   `shared_ptr`. nav2's `on_configure` **and** `on_activate` are merged there.
2. **The planner plugins and the costmap layers are separate plugin systems.**
   Both load through pluginlib against `syncai_nav_core` / `syncai_costmap_2d`
   respectively, and both are configured from the same params file under two
   different node keys.

## Layout

```
src/planner_server.cpp   the two action servers + the plan cycle
src/main.cpp             construct → configure() → spin
plugins/
  navfn_planner/navfn.cpp          the NavFn potential-field solver (1063 lines, upstream)
  navfn_planner/navfn_planner.cpp  the GlobalPlanner plugin wrapping it
  straight_line_planner/           a minimal reference plugin
  smac_planner/                    SmacPlanner2D + its search core
                                   (A*, smoother, collision checker, Node2D)
global_planner_plugin.xml          pluginlib manifest, exported to syncai_nav_core
params/planner_server_params.yaml
launch/planner_server.launch.py
```

Everything builds into one library, `libsyncai_planner.so`, which is both linked
by the `planner_server` executable and `dlopen`ed by pluginlib as the plugin
library.

## The plan cycle

`computePlan()` runs on the `SimpleActionServer`'s own spin thread
(`spin_thread=true`), so its blocking steps never starve the node's executor —
which in this process only handles parameter callbacks. The costmap is spun by
its own `NodeThread`; `main.cpp` explicitly warns not to add it to this
executor as well.

```
server inactive / cancel requested?   → bail
waitForCostmap()                      ← blocks at 100 Hz until isCurrent()
accept a preempting goal if pending
start = goal.use_start ? goal.start : costmap.getRobotPose()
transform start and goal into the costmap global frame
path = getPlan(start, goal, goal.planner_id)
path empty?                           → terminate_current()
publish plan, fill planning_time, succeeded_current()
```

Any exception from a plugin is caught and turned into `terminate_current()` with
a warning — a planner failure fails the goal, never the process.

`expected_planner_frequency` only controls a warning: if a cycle takes longer
than its reciprocal, the server logs "Planner loop missed its desired rate". It
does not throttle or abort anything.

**`getPlan()` selects the plugin by `planner_id`.** An unknown id logs an error
and returns an empty path; an *empty* id with exactly one plugin loaded picks
that one and warns once. The BT passes `planner_id="GridBased"` explicitly.

### ComputePathThroughPoses

The second action plans `start → goals[0] → goals[1] → …` as independent
segments and concatenates them, dropping the first pose of each subsequent
segment so the shared waypoint is not duplicated. An empty `goals` list is
rejected outright, and any segment that fails fails the whole request.

Its transform and validation steps are **inlined rather than reusing the
`ComputePathToPose` helpers** — those helpers call `terminate_current()` on
`action_server_pose_`, i.e. the wrong action server. Worth remembering before
refactoring the duplication away.

Nothing in this workspace calls this action today: `syncai_task_runner`'s
`move.xml` only uses `ComputePathToPose`, and the backend sequences multi-stop
tasks as separate `MOVE` steps. It is here for the patrol/nav-through-poses case.

## Planner plugins

### NavfnPlanner (`syncai_planner/NavfnPlanner`) — built, not loaded

The classic NavFn potential-field planner: propagate a navigation function
outward from the goal over the costmap, then follow the gradient back from the
start. It was the configured planner until `GridBased` was pointed at
`SmacPlanner2D`; it is still registered, so switching back is a one-line change
to `plugin:` in the params file.

| Parameter | Default | Notes |
|---|---|---|
| `tolerance` | `0.5` | If the goal cell is unreachable, search a square of ±tolerance around it for the nearest reachable pose |
| `use_astar` | `false` | `false` = Dijkstra (full expansion), `true` = A* (heuristic, faster, less optimal) |
| `allow_unknown` | `true` | Plan through `NO_INFORMATION` cells |
| `use_final_approach_orientation` | `false` | Set the last pose's yaw to the path's approach direction instead of the requested goal yaw |

These are code defaults only. NavFn is no longer configured anywhere —
`planner_server_params.yaml` mentions it in a trailing comment as the fallback
to point `GridBased.plugin` at, and that is the extent of its presence in the
config.

Three behaviours worth knowing:

- **Start == goal is special-cased.** It returns a single-pose path (or an empty
  one if that cell is lethal) rather than running the solver.
- **`smoothApproachToGoal()` fixes the last pose.** NavFn's path ends on the
  nearest *grid cell*, not the exact goal coordinate, so the final pose is either
  replaced by the true goal or the goal is appended — whichever keeps the path
  monotonic.
- **`use_final_approach_orientation: true`** makes the last pose face along the
  direction of travel, so the controller does not perform a final in-place
  rotation. Left `false` here, because the task API's `theta` is a real goal
  heading.

`navfn.cpp` is upstream nav2 code, essentially unmodified.

### StraightLinePlanner (`syncai_planner/StraightLinePlanner`)

Linear interpolation from start to goal at `interpolation_resolution` spacing,
**with no obstacle checking whatsoever**. It exists as a minimal reference
implementation of the `GlobalPlanner` interface and as a way to feed the
controller a known path for tuning. Not listed in `planner_plugins` today —
add it back to the params file when needed.

Do not select it on a real robot expecting collision avoidance — it will happily
plan through a wall.

### SmacPlanner2D (`syncai_planner/SmacPlanner2D`) — the configured planner

Port of `nav2_smac_planner` (humble, 1.1.20), reduced to its 2D plugin plus the
shared search core (`AStarAlgorithm<Node2D>`, `Smoother`,
`GridCollisionChecker`, `CostmapDownsampler`).

Cost-aware 8-connected grid A*: edge cost is
`length * (1.0 + cost_travel_multiplier * cell_cost / 252)`, so inflation cost
is inside the search rather than something a gradient walk has to fight
afterwards. That is the reason it replaced NavFn as `GridBased` — NavFn's
gradient descent cuts corners against the inflation gradient, this drifts to
the centreline of a corridor on its own. Collision checking is radius-only
(`setFootprint(..., radius=true, ...)` is hardcoded for 2D), so there is no
footprint / inflation coupling to tune here — clearance comes entirely from the
costmap's inflation layer.

| Parameter | Default | Config | Notes |
|---|---|---|---|
| `tolerance` | `0.125` | `0.5` | Goal tolerance in metres — accept a goal within this distance of the requested pose |
| `allow_unknown` | `true` | `false` | Plan through `NO_INFORMATION`. **Disabled** in the shipped config, so the prose about planning through unknown describes a behaviour that is off. The YAML records no rationale, but the consequence is worth knowing: the global costmap has `track_unknown_space: true` and the z-band gridmap converter deliberately returns glass-painted free space to unknown, so with this off A* cannot route through exactly the cells the converter declined to vouch for — turning it back on re-opens them |
| `cost_travel_multiplier` | `1.0` | `1.0` | Weight on costmap cost vs. distance, applied as `1.0 + cost_travel_multiplier × cell_cost / 252`; `0.0` degenerates to plain distance A*. Lowered from `2.0` (commit `8376bf4`): at 2.0 paths bowed visibly along the inflation gradient in open space; 1.0 keeps enough cost-awareness to stay off walls while distance dominates, so open-space legs come out near-straight — clearance still comes from the inflated footprint, not this weight. **Not** called `cost_penalty` — that was the Hybrid spelling |
| `downsample_costmap` | `false` | `false` | Publishes `downsampled_costmap` when on |
| `downsampling_factor` | `1` | `1` | |
| `max_iterations` | `1000000` | `1000000` | |
| `max_on_approach_iterations` | `1000` | `1000` | Refinement budget once inside tolerance |
| `max_planning_time` | `2.0` | `2.0` | Hard cutoff in `createPath()`, and the smoother gets whatever is left |
| `use_final_approach_orientation` | `false` | `false` | Same reasoning as NavFn's |
| `smoother.w_data` | `0.2` | `0.1` | **Sub-namespace** — read as `GridBased.smoother.*`. Anchors each waypoint to the raw A* path (keeps the search's obstacle clearance). Lowered from 0.2 in 2026-10 for rounder corners. One waypoint update above cost 252 aborts smoothing for the **whole** path, so a raw path that grazes the inscribed edge once comes out staircased everywhere: wall clearance from the inflation band is also what lets the smoother finish |
| `smoother.w_smooth` | `0.3` | `0.4` | Pulls each waypoint toward the midpoint of its neighbours (straightens). Nudged up from the upstream 0.3 to iron out the grid-A* staircase; the smoother rejects any update landing in lethal/inscribed cost, so it cannot smooth *through* an obstacle — but keep it well below ~0.5, past which the anchor term stops mattering and corners get cut tight against the inflation edge |
| `smoother.max_iterations` | `1000` | `1000` | |
| `smoother.do_refinement` | `true` | `true` | Up to four extra smoothing passes on the result |

Behaviours worth knowing:

- **Pose orientations are meaningful only by accident.** The search is over
  (x, y); `createPlan` writes an identity quaternion into every pose, the
  smoother then overwrites every segment longer than ten poses with the
  direction to the next pose (on its bail-out returns too), and the last pose
  is overwritten with the goal yaw. So a short path, a short segment, a
  coincident pair, or a NavFn / StraightLine path still carries `w = 1`. RPP
  does not read intermediate yaw, and `is_path_valid` derives its heading from
  the geometry rather than the field, so nothing depends on it today.
- **There is no `smooth_path` toggle.** 2D always smooths, with the smoother
  constructed as holonomic and with no turning-radius constraint.
- **The smoother follows `allow_unknown`** (2026-10). Upstream always lets a
  waypoint be smoothed onto an unknown cell. With `allow_unknown: false` that
  produced paths the search itself would have refused, which `is_path_valid`
  then reported blocked on its first check: the BT replanned into the same
  smoothed path once a second.
- **`is_path_valid` logs every "blocked" answer** at INFO, with the first
  blocked pose, how far ahead of the robot it is, and which test tripped: the
  centre cost (254 marked obstacle, 253 inscribed, 255 unknown, -1 off the
  map) or "footprint perimeter on LETHAL at heading …" (the padded rectangle,
  at the path heading or ± `heading_margin`, touches a marked obstacle the
  centre test did not see). Each one is a replan, so this line is where to
  start when a route changes unexpectedly.
- **Debug topic `unsmoothed_plan`** carries the pre-smoother path, published
  only while something is subscribed.

**What was dropped from the port.** Upstream's other two plugins are not here:

- **SmacPlannerLattice** — nobody plans to feed it motion-primitive JSONs, and
  it carried the `sample_primitives` data blob, the nlohmann_json dependency
  and a `NodeLattice` model.
- **SmacPlannerHybrid** — removed after evaluation, not merely unconfigured.
  G23 turns in place, so Hybrid-A*'s `minimum_turning_radius` (which Dubins
  cannot take as `0`) only bought detours, and RPP cannot follow the reversing
  paths Reeds-Shepp exists to produce. Its cost — 72-bin SE2 search, a Dijkstra
  obstacle heuristic, a precomputed Dubins lookup table and full-footprint
  collision checking at every angle bin — bought nothing this robot uses.

Removing Hybrid also took `NodeHybrid` and the whole `AnalyticExpansion` class
with it (every `AnalyticExpansion<Node2D>` entry point upstream is a
`return nullptr` stub, so `createPath()` lost a step that could never fire),
trimmed `NodeBasic` to the two members the queue actually needs, and left
`AStarAlgorithm`'s `initialize()` / `setStart()` / `setGoal()` defined only as
`Node2D` specializations. To restore either planner, take it from upstream plus
this port's de-lifecycle diff on `smac_planner_2d.cpp`, and re-add the
`AnalyticExpansion` call to `AStarAlgorithm::createPath()`.

The port follows the same de-lifecycle pattern as the rest of the stack:
upstream's `configure()` + `activate()` collapse into `initialize()`,
`deactivate()` + `cleanup()` into the destructor, the lifecycle publishers
became plain publishers (live from creation), and the dynamic-parameter
callback — kept, since live-tuning is the main reason to run smac — is
registered at the end of `initialize()`.

Two gotchas:

- **OMPL is still a hard dependency** (`ros-humble-ompl`), even without Hybrid:
  `Smoother` resamples the boundary segments of a path through a Dubins state
  space. It is in the Dockerfile's base stage, but a container created from an
  older image needs `sudo apt-get update && sudo apt-get install -y
  ros-humble-ompl` by hand — and that hand-install is wiped on container
  recreation, so rebuild the image when convenient.
- **Do not configure two SmacPlanner2D ids.** `Node2D` keeps
  `cost_travel_multiplier` and its neighbour-offset table in *static* storage,
  so a second instance overwrites the first's tuning — same limitation
  upstream, just less obvious here where all plugins live in one library.

## Parameters

**Server** (`/**/planner_server`):

| Parameter | Default | Notes |
|---|---|---|
| `planner_plugins` | `["GridBased"]` | IDs; each needs `<id>.plugin` naming the type |
| `expected_planner_frequency` | `1.0` (config: `20.0`) | Warning threshold only — a plan taking longer than 50 ms logs a missed-rate warning. Generous by design: the BT asks for a plan at most once a second, and only when the current path is blocked, so the warning is a canary for a pathological search, not a cadence anyone is trying to hit |
| `plan_republish_rate` | `1.0` (config: `1.0`) | Hz at which the last plan is re-sent on `plan`, restamped and trimmed to start at the pose nearest the robot (the same nearest-pose search `is_path_valid` uses, so what is shown is what is checked; the whole plan when the robot pose is unavailable); `0` = only when a plan is made. Needed since `move.xml` plans only when the path is blocked: without it a viewer subscribing mid-drive never sees the route. It keeps showing the last route after the goal ends, since the planner cannot tell when the navigation finished. Read once at startup |
| `is_path_valid.footprint_check` | `true` | Also test the padded footprint's perimeter against LETHAL at each pose ahead of the robot — RPP's own collision test, run on the global costmap. `false` restores the centre-cell test alone (what the service did until 2026-10). Dynamic |
| `is_path_valid.heading_margin` | `0.35` | Radians either side of the path heading the perimeter is also tested at. RPP tracks with up to `rotate_to_heading_min_angle` (0.25) of heading error before it rotates in place, and the gait adds yaw sway; at 0.35 the rectangle reaches 0.365 m sideways, which is what catches a person 0.3 m beside the route — and what rejects a straight passage narrower than 0.73 m (doors pass). `0` tests the path heading only. Dynamic, clamped to [0, π/2] |

`expected_planner_frequency` and the two `is_path_valid.*` parameters are what
the *server's* dynamic callback handles, and it takes the same mutex the plan
cycle holds (the `is_path_valid.*` values are atomics the service reads
without that mutex, so a check never waits behind a plan).
`SmacPlanner2D` registers its own callback, so `cost_travel_multiplier`,
`tolerance` and friends are live-tunable with `ros2 param set` (NavFn's are not
— it reads them once in `initialize()`). Swapping the *plugin* still needs a
restart either way.

**Global costmap** (`/**/global_costmap`) — the full costmap parameter set; see
`syncai_costmap_2d`'s README. The choices specific to a *global* costmap here:
`rolling_window: false`, `track_unknown_space: true`, `global_frame: map`, and
a low `update_frequency: 1.0` (the static map rarely changes).

**The inflation band is the planner's wall clearance.** In open space Smac's
path rides the edge of the band, `inflation_radius` from the wall; in a passage
narrower than twice that it takes the centre line. `inflation_radius: 0.8` /
`cost_scaling_factor: 1.5` since 2026-10 (from 0.5 / 2.0, which put the body
~0.27 m off walls; now ~0.57 m). The YAML has the arithmetic and the price
(narrow passages cost more, so longer ways round win more often). The smoother
cannot add clearance: it has no cost term and only refuses to land a waypoint
above cost 252, so pushing its weights toward smooth pulls corners *toward*
walls. The local costmap's band (0.4 / 3.0) is separate and unchanged.

The footprint is a rectangle with half-extents 0.35 × 0.22, **the same
rectangle as the local costmap** in `syncai_controller` (reconciled 2026-10;
the local one had been left at 0.28 × 0.20). Only the padding differs, on
purpose: `footprint_padding: 0.03` here against 0.01 there. The global
footprint has to be the larger one so that `is_path_valid`'s perimeter test
(this rectangle, this padding) is stricter than RPP's (same rectangle, the
local padding) and a path it passes is one RPP will drive. Change the rectangle
in both files or neither, and never let the local padding reach the global one.

That ordering holds between the two **perimeter** tests, and only since
2026-10. Before that `is_path_valid` tested the centre cell alone and this
README claimed the padding gap kept "the planner on the conservative side" —
it did not. Smac accepts any centre cost below 253, which bounds the centre's
distance to a lethal cell (the inscribed radius, 0.25 m) and says nothing
about the corners (0.455 m), so a blocker 0.25–0.45 m off the path was
Smac-valid, RPP refused it ("collision ahead!"), nothing replanned, and the
goal aborted on `failure_tolerance` without a detour ever being tried. The
0.02 m padding gap absorbs about 3° of heading error, not the 14° RPP tracks
with; `is_path_valid.heading_margin` is what covers that now.

`inflation_radius` (0.8) must stay at or above the padded footprint's
circumscribed radius, √(0.38² + 0.25²) ≈ 0.455 m: the inscribed core (cost
253) is what the centre test and the keepout filter's own inflation key on,
and it is only meaningful while the band around it is at least robot-sized. A
circular `robot_radius: 0.22` was tried before the rectangle and was oversized
enough that RPP rejected valid paths through ~0.6 m gaps.

### Notes on the current config

There used to be separate 2D (sim) and 3D (real robot) params files; the 2D
one went away with `bringup_2d`, and `planner_server_params.yaml` now carries
the 3D-path configuration: `use_sim_time: false`, `GridBased`
(`SmacPlanner2D`) only, obstacle
sources `scan` **and** `pointlio/body_cloud`.

The pointcloud source deliberately leaves `sensor_frame` empty so the
observation buffer uses the cloud's own header frame as the raytrace origin —
meaning no per-robot launch override, but also that it only works after
`syncai_localizer` has been given a pose (`/<robot_id>/relocalize`, the INI's
`[initial_pose]` or an `initialpose` message).

The global costmap runs the **keepout filter** (`filters: ["keepout_filter"]`,
since 2026-09). Its mask comes from `map/<name>/keepout.yaml`, served by
`syncai_map_server`'s `costmap_filter_info.launch.py` in the nav session's
`map_server` window (pane `keepout`); when a map has no mask yet that launch
writes a blank one of the gridmap's geometry first (all unknown, so the filter
changes nothing), which is why the mask server is up on every map and "Filter
mask was not received" in this log now means the pane is actually down. Two
things to know before drawing a mask: filters run *after* inflation, so the
`KeepoutFilter` inflates the mask itself with this costmap's inscribed radius
and `inflation_layer` values (2026-10 — draw the forbidden area only, no
margin; a zone then closes any gap narrower than the robot exactly as a wall
does), and only the global costmap has the filter — the controller's local
costmap does not, so the planner alone keeps the robot out and RPP cannot
reject a path for grazing a zone edge. Running the mask source
by hand, outside the session:

```bash
ros2 launch syncai_map_server costmap_filter_info.launch.py            # map/<name>/keepout.yaml from the INI
ros2 launch syncai_map_server costmap_filter_info.launch.py mask_yaml:=map/other/keepout.yaml
```

## Interfaces

With the node at `/<robot_id>`:

| Interface | Name | Type |
|---|---|---|
| Action | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` |
| Action | `compute_path_through_poses` | `nav2_msgs/ComputePathThroughPoses` |
| Service | `is_path_valid` | `nav2_msgs/IsPathValid` — is the part of a path from the pose nearest the robot onward free? Two tests per pose: the centre cell `< INSCRIBED` (the test SmacPlanner2D expands with; unknown counts as blocked, matching `allow_unknown: false`) and, with `is_path_valid.footprint_check`, no LETHAL cell under the padded footprint's perimeter at the path heading ± `heading_margin` (RPP's own collision test, so a path that passes is one RPP will drive; unknown under the perimeter does not count, since the local costmap never shows RPP unknown). Called once a second by `move.xml`'s `IsPathValid`, which replans only on `false`. Answers `false` for an empty path or a frame other than the costmap's; answers `true` when there is **no robot pose** (a TF miss must not force a replan whose own start lookup fails and whose recovery clears the global costmap). Served on the main executor, so it never queues behind a plan. Where the two tests still disagree with RPP: marks only the local costmap holds, RPP's in-place rotation sweep, heading error beyond the margin |
| Publisher | `plan` | `nav_msgs/Path` (visualization; skipped when nothing is subscribed). Sent when a plan is made, and the last one again at `plan_republish_rate`, from the robot's nearest pose onward |

Plus everything the internal costmap exposes under
`/<robot_id>/global_costmap/…` — `costmap`, `costmap_updates`,
`published_footprint`, `get_costmap`, and the three `clear_*_global_costmap`
services the BT's recovery branch calls.

## Running

```bash
ros2 launch syncai_planner planner_server.launch.py
ros2 launch syncai_planner planner_server.launch.py \
    system_config:=config/instances/robot02.ini
```

Both byobu sessions start it in the `plan_ctrl` window after `sleep 4` — the
costmap's static layer blocks on the latched map from `map_server`, and
`Costmap2DROS::activate()` blocks on TF.

The launch file does the same two namespace-critical things as the controller's:
**no `name=` on the `Node`** (the process hosts `planner_server` *and*
`global_costmap`; a launch-level name would remap both and strip the costmap of
its parameters), and explicit `<robot_id>/` prefixes for TF frame parameters,
which land under the `/**` wildcard and reach the costmap node.

Testing without the task runner:

```bash
ros2 action send_goal /<robot_id>/compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}}}, planner_id: GridBased}"

ros2 topic echo /<robot_id>/plan --once
ros2 service call /<robot_id>/global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

## Gotchas

- **`waitForCostmap()` has no timeout.** If an observation source stops
  publishing, the costmap never becomes current and the plan request hangs with
  the goal still active rather than failing. Same pattern as the controller's
  loop.
- **A plugin load failure calls `exit(-1)`** — a typo in a `.plugin` string
  takes the whole process down at startup.
- **Footprints must match the controller's local costmap.** Diverging footprints
  mean the planner produces paths RPP then rejects with "collision ahead!".
- **`StraightLinePlanner` ignores obstacles entirely.** If it is ever added
  back to `planner_plugins`, do not select it via `planner_id` on hardware.
- **The launch file overrides `obstacle_layer.scan.sensor_frame` to
  `<robot_id>/laser`, but the 2D scan lives in `<robot_id>/scan`.**
  the merger stamps its output with `<robot_id>/scan` (and the `base_link →
  scan` TF came from the now-removed `bringup_2d`, so on the 3D stack nothing
  publishes it at all); `syncai_controller`'s launch file overrides the same
  parameter to `<robot_id>/scan`. Nothing in the workspace broadcasts a
  `<robot_id>/laser` frame (the Livox driver only *stamps* messages with it in
  the 3D path). If that is right, the global costmap's scan observation source
  cannot resolve its raytrace origin and never marks — masked by the static
  layer carrying the obstacles that matter. Worth verifying against a live
  costmap before changing.
- **Dynamic parameter changes are refused mid-plan** by the shared mutex, and
  only `expected_planner_frequency` is handled at all — plugin parameters go to
  each plugin's own callback.

Upstream references: [`nav2_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_planner),
[`nav2_navfn_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_navfn_planner),
and [`nav2_smac_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_smac_planner).
