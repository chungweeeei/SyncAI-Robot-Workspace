# syncai_global_planner

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

Two planners are loaded: `SmacPlanner2D` as `GridBased`, and
`OctomapPlanner3D` (not drawn above; its own section below) as `Octo3D`. On
this branch (`feature/octomap-planner-3d`) a 3D map's tree (`move_3d.xml`)
asks for `Octo3D`, and a 2D map's (`move.xml`, as on `dev`) for `GridBased`. NavFn and StraightLine are built and
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
  octomap_planner_3d/              OctomapPlanner3D (planner id Octo3D): the
                                   ROS-free core (surface_map, surface_astar,
                                   path_postprocess), the plugin, the offline
                                   CLI, and standalone/CMakeLists.txt to build
                                   the CLI without ROS
global_planner_plugin.xml          pluginlib manifest, exported to syncai_nav_core
params/planner_server_params.yaml
launch/planner_server.launch.py
test/test_surface_map.cpp          OctomapPlanner3D's geometry rules on synthetic scenes
```

Everything builds into one library, `libsyncai_global_planner.so`, which is both
linked by the `planner_server` executable and `dlopen`ed by pluginlib as the
plugin library.

**The package was `syncai_planner` until 2026-10.** It was renamed when
`syncai_mppi_controller` arrived, so the two halves of the stack read as global
planner / controller, and the rename stopped at the package: the node is still
`planner_server`, the executable, the launch file, the
`compute_path_to_pose` / `compute_path_through_poses` actions, `is_path_valid`,
`plan`, `global_costmap` and the `planner` log directory all kept their names,
so `move.xml`, rviz and the backend saw nothing change. What did change is
every spelling of the package: the C++ namespace, the include directory, the
library, and the pluginlib lookup names (`syncai_global_planner/SmacPlanner2D`).
A workspace built before the rename keeps `build/syncai_planner` and
`install/syncai_planner`; delete both. Left in place, the old `install/` stays
on `AMENT_PREFIX_PATH` and keeps registering `syncai_planner/*`, so a params
file still naming the old plugin loads the stale `.so` instead of failing.

## The plan cycle

`computePlan()` runs on the `SimpleActionServer`'s own spin thread
(`spin_thread=true`), so its blocking steps never starve the node's executor —
which in this process only handles parameter callbacks. The costmap is spun by
its own `NodeThread`; `main.cpp` explicitly warns not to add it to this
executor as well.

```
server inactive / cancel requested?   → bail
accept a preempting goal if pending
planner reads the costmap?            (GlobalPlanner::usesCostmap(); Octo3D does not)
  waitForCostmap()                    ← polls at 100 Hz; takes a preempt (and re-checks the line above)
    cancel requested?                 → terminate_all()
    a static layer has no map?        → terminate_current()  (3D-only map: no gridmap.yaml)
    isCurrent()?                      → go on
    costmap_wait_timeout passed?      → terminate_current()
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
that one and warns once. The BT always names one explicitly: `Octo3D` in
`move_3d.xml` / `dock_3d.xml` (a 3D map), `GridBased` in `move.xml` / `dock.xml`.

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

### NavfnPlanner (`syncai_global_planner/NavfnPlanner`) — built, not loaded

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

### StraightLinePlanner (`syncai_global_planner/StraightLinePlanner`)

Linear interpolation from start to goal at `interpolation_resolution` spacing,
**with no obstacle checking whatsoever**. It exists as a minimal reference
implementation of the `GlobalPlanner` interface and as a way to feed the
controller a known path for tuning. Not listed in `planner_plugins` today —
add it back to the params file when needed.

Do not select it on a real robot expecting collision avoidance — it will happily
plan through a wall.

### SmacPlanner2D (`syncai_global_planner/SmacPlanner2D`) — the configured 2D planner (`GridBased`)

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

### OctomapPlanner3D (`syncai_global_planner/OctomapPlanner3D`) — planner id `Octo3D`, prototype (2026-10)

The quadruped's ground-constrained 3D planner, on `map/<name>/octomap.bt` —
the `octomap::OcTree` that `syncai_mapping`'s `clean_map --octomap 1` writes
beside `map.pcd` (0.1 m; free **and** occupied, so never-seen space is
unknown; hit floor-band voxels kept occupied). It is loaded beside
`GridBased` as a second id, and on this branch a 3D map's trees ask for it
(`move_3d.xml` / `dock_3d.xml`, with `controller_id="FollowPathRPP3D"`, the
controller that follows a path with heights and climbs stairs —
`syncai_controller/README.md`), so every navigation goal on a 3D map plans
here. Which trees a goal runs is read off this planner: it latches
**`planner_server/Octo3D/map_available`** (`std_msgs/Bool`), true while it has
a map, built or being built (a goal mid-build belongs to the 3D tree and
fails `map_loading`), false with no `octomap_file` or after a failed build;
the task_runner reads it per goal (2026-10-19). The 2D costmap is used for its frame id only, and
since 2026-10-15 the server does not wait for it to be current before an
`Octo3D` plan (`usesCostmap()` is `false`): a 3D-only map has no
`gridmap.yaml`, so `map_server` and the `keepout` pane come up idle (until
2026-10-19 they died at boot), the static layer never gets a map, and every goal used to hang in
`waitForCostmap()` without reaching this planner — no log line, until the BT
cancelled it. The global costmap stays at its 5 × 5 m default on such a map (no
map), which is noise for `Octo3D`; its four warnings about that — no static
map, robot out of bounds, sensor origin out of bounds, no keepout mask — are
logged **once each** on entering the state and an info on leaving it
(2026-10-19; until then they repeated every 1–10 s and buried the planner
log), each saying it is expected on a 3D-only map. A
`planner_id: GridBased` goal there hung the server's action thread for good
until 2026-10-19; it now fails at once ("static layer … has no map"), which
`NavigateToGoal` reports as `PLAN_FAILED` — see "The plan cycle".

**Why a surface map and not a free-space search.** The first attempt (the
device workspace's `octo_planner_3d`) ran a 26-neighbour A* over the free
voxels with "something occupied one cell below" as ground support and a
blacklist of wall-foot cells to keep it from climbing. That is a drone
planner with patches: a free cell beside a wall is "supported" at any height,
the body is a 0.3 m half-sphere, and nothing asks whether a step is one the
legs can take. Here the tree is reduced, once at startup, to a **multi-layer
2.5D surface map** — every xy column's floor surfaces, each judged with the
robot's geometry — and the search only ever moves between cells that passed.
So a path cannot leave the ground, climb a wall, cross a stair riser, a kerb
or a drop-off, or jump between floors; it takes ramps, which are runs of
one-cell steps on a plane under the slope limit. The one way between floors
is a stair the robot walked while mapping (see **Stairs** below).

**The build** (`surface_map.cpp`; ~1 s and 355 MB peak for dp1f_1012_v2's
7.3 M nodes, ~30 MB kept once the tree is freed; in a background thread so
`planner_server::configure()` and the other ids are not held up — plans
arriving meanwhile fail with `map_loading`):

1. One pass over the leaves into per-column `known` / `occupied` bitmasks,
   pruned blocks expanded by key range (never `search()` in a loop).
2. A **surface** is an occupied voxel with a non-occupied voxel above and a
   `body_clearance` (0.6 m) band of non-occupied voxels on top. Unknown in
   the band is tolerated (the lidar looks outward from 0.48 m, so the band
   is observed only where the robot passed), but the voxel right above the
   surface must have been observed free (`min_known_free_above: 1`): a floor
   the robot stood on always has, a ceiling seen from below never has — that
   one rule took dp1f_1012_v2 from 748 "components" to 153.
3. **Hole fill.** The MID360's lower field of view is −7°, so the floor
   nearer than ~4 m is never seen directly and the floor under the robot's
   own track is hit only from afar: ~5 % of those columns are empty. A column
   with no surface whose neighbours within 0.3 m agree on a height, from at
   least three of the four quadrants around it and with ≥ 50 % support, and
   with nothing occupied near that height, gets an *inferred* surface. The
   quadrant rule is what keeps a balcony edge from being extended into the
   void (one-sided support); each height group of the neighbours is judged
   on its own, because at a two-storey column the complete upper floor is
   the largest group and would otherwise hide the missing lower one.
4. **Step rule**: every 8-neighbour column that holds surfaces must hold a
   usable one within `max_step_height` (0.17 m, the legs' step capability
   and a standard riser; rounded to two cells at 0.1 m, so a 0.15 m kerb is
   one step and a 0.5 m ledge is not). A neighbour column with no surface at
   all is a *gap* (never observed), left to the footprint rule.
5. **Footprint**: of the columns within `robot_radius` (0.25 m, the 2D
   costmap's inscribed radius), at least `footprint_min_support` (80 %) must
   hold a usable surface inside the slope cone (tolerance grows with
   distance, or a 15° ramp rejects its own neighbours); a column whose
   surfaces are all out of reach or unusable — a wall foot, a drop, an
   obstacle top — blocks the cell outright, a column with no surface is a
   gap, counted (`gap_cnt`) and paid for in cost. If the surfaces span more
   than one step, the plane fitted through them must tilt ≤ `max_slope_deg`
   (20) with residuals ≤ one cell: that is what refuses a flight (two risers
   in a 0.5 m disc, ~30°) while a kerb, one step, is level ground. A fit,
   not max−min: a flat floor read cell by cell looks 11° tilted across a
   0.5 m disc. This is the implicit inflation of walls, drop-offs and
   unfillable holes; the gap share exists because the lidar sees the floor
   at grazing angles only and the observed floor is sparse — with every
   column required, 13 % of dp1f_1012's own keyframes stood on
   "untraversable" ground.
6. 8-connected components (no corner cutting), so a disconnected goal fails
   before the search and a goal snaps to the robot's floor, not the one below
   a balcony edge.
7. A clearance distance field propagated over the surface graph (not the
   2D grid, or a ground-floor wall would cost the floor above it) out to
   `clearance_cost_radius` (0.55 from the traversable boundary = 0.8 from
   the wall, the costmap's `inflation_radius`).

Every rejected surface keeps its reason (`clearance_occupied`,
`no_free_above`, `step`, `footprint_blocked`, `slope`, `residual`), which is
what the CLI dumps and a failed snap reports.

**Search** (`surface_astar.cpp`): A* over the traversable cells, 8 xy
neighbours each resolved to the layer within one step (unique, since two
layers of a column are never that close), diagonals only when both
orthogonal neighbours pass. Edge cost = 3D move length ×
`(1 + clearance_cost_weight · exp(−clearance_cost_scaling · d))` inside the
band (the costmap's inflation curve and Smac's `cost_travel_multiplier` in
one; a multiplier, because a per-node penalty makes diagonals cheaper per
metre and bends paths 45°), × 1.2 on inferred cells, × (1 + `gap_cost_weight`
· gaps / footprint columns) on cells with gaps under the disc. The 3D Euclidean
distance is admissible, so `heuristic_weight: 1.0` is optimal; a 25 m plan on
dp1f_1012_v2 takes 11 k iterations and 1 ms, a 46 m plan upstairs 22 k and
3 ms.

**Snap.** The server's start pose is the planar `map → base_link`, z = 0
always, so the robot's height is read from TF to `body_frame`
(`<robot_id>/pointlio_body`, the LIO chain that keeps z) minus
`lidar_height`; without the TF the request's z is used (right on the ground
floor, wrong upstairs). The goal's z is the request's (`goal_z_mode:
request`; a 2D backend's 0 lands on the ground floor, whose surface is
~0.5 m *below* the map origin — the first lidar pose) or the start's
(`start`). Both snap to the nearest traversable cell within `snap_xy_radius`
whose top face is within `snap_z_window` (1.0 m, far below the 5.7 m floor
spacing) of the hint; the goal only within the start's component. A failed
snap says what *is* there — `goal_unavailable: … surfaces in that column:
z=[-0.40 (untraversable: footprint_blocked), 4.80 (traversable, component
2)]` — or that the nearest surface is on another floor.

**Stairs** (`stair_links.cpp`, 2026-10). The geometry rules refuse every
stair on purpose — a riser is more than one step, a flight is steeper than
20° — and relaxing them map-wide would pass every 0.2 m kerb and drop-off
too. So stairs come from the map's own track instead: `poses.txt` (next to
`octomap.bt`, or `poses_file`) is where the robot stood while mapping, and a
stretch of it that leaves one large component (≥ `stair_anchor_min_area`,
10 m²) and arrives in another at least `stair_min_rise` (0.3 m) higher or
lower, within `stair_max_length` (60 m) of track, is a stair the legs have
taken. It becomes a **stair link** between the traversable cells at its ends,
carrying the keyframes' ground points with every loop cut out (turns and
waits on a landing) and smoothed (`stair_smooth_passes`; a hand-driven track
weaves, and the ground estimate inherits the body's pitch on a flight).
What could not be linked is logged at startup: a climb on the track longer
than `stair_max_length`, and large components at different heights no link
joins — the floors this map cannot get between. The search may take a link in either direction at its 3D
length × `stair_cost_multiplier` (2.0; ≥ 1 keeps the heuristic admissible),
components joined by links count as one for the snap and the
disconnected-check, and the path follows the link's points through the
flight; the line-of-sight shortcut is applied to the floor stretches only.
Only large components end a link because a stairwell's own floor rarely
survives the rules: on dp1f_1012 the three landings of the switchback stair
came out as a few square metres of islands or not at all (small, walled, the
air above them never observed), so the link runs floor to floor over the
whole well. Stairs the robot never walked stay refused; a kerb walked over
(< 0.3 m) makes no link. `stair_links: false` restores separate floors.

**Output** (`path_postprocess.cpp`): a line-of-sight shortcut checked with a
supercover traversal (every cell the segment touches; Bresenham skips the
corner cell a wall foot sits in), step-continuous along the segment, and
never closer to the boundary than the cells it replaces (Smac's "never above
252" rule); then a resample at `interpolation_resolution` with z read back
from the surface. Poses carry z = the floor's top face (+ `path_z_offset`),
yaw toward the next pose, the goal's orientation last. MPPI and the 2D RPP
ignore z; RPP 3D (`FollowPathRPP3D`) and the controller_server's goal check do not since 2026-10
— a path whose poses do not all share one z is matched to the robot in 3D
(a stair link's spiral is back over the same x / y every 8.1 m on
dp1f_1012), and RPP 3D climbs a stretch rising 0.3 m within 1 m in a stair mode
that switches the gait policy (`syncai_controller/README.md`, "Paths with
heights" and "Stairs").

**Measured on dp1f_1012_v2** (2026-10-13; 1F and 2F, 2 576 keyframes): 1.26 M
columns, 724 k surface cells (24 k inferred), 158 k traversable in 146
components, the two floors being 118 k and 33 k. 86 % of the keyframes stand
on a traversable cell, 90 % within one cell of one, 99.5 % on *a* surface.
What the rest is, by inspection: people's feet and ankles (0.3 m clusters
of occupied voxels 0.1–0.25 m above the floor at the robot's stops — the
operator's, seen once from ~3 m ahead and passed through 30–40 times, kept
by `clean_map`'s floor-band exemption until its `floor_band_dynamic` of
2026-10-14; they read as "the robot's own feet" at first, but the hits come
from a keyframe metres behind), the 1F–2F stairwell (a
switchback stair, four flights and three landings, 5.3 m up in a 4 × 3 m
well; refused by the geometry rules, so the floors are separate components
and only a stair link joins them), and a short ~30° ramp on 2F.

**Stairs on dp1f_1012** (2026-10-14, the map after `syncai_mapping`'s
local-floor and pool fixes): 3 stair links — kf 1677 → 1913 from 1F (z −0.10)
to 2F (5.20), 39 m and 87 points after the loop cut, and two across a 0.4 m
step on 2F. 1F → 2F plans (kf 0 → 2000: 85.6 m; kf 22 → 1913: 42 m, of
which the link is 39) take 15–30 ms and climb the four flights and three
landings in order; 2F → 1F takes the same link backwards.

**Map switch** (2026-10-19). The tree is read at startup, and nothing
watches the INI — the INI changing says nothing about when the rest of a
switch is done. Whoever switches maps calls, beside `map_server/load_map`:

```bash
ros2 service call /<robot_id>/planner_server/Octo3D/load_map nav2_msgs/srv/LoadMap \
    "{map_url: 'map/<name>/octomap.bt'}"
```

- **A result, not a receipt**: the call returns once the new surface map
  and its stair links are in (~1 s on dp1f_1012, a few s on a large site),
  so `RESULT_SUCCESS` means `Octo3D` plans on the new map. A caller's
  timeout must cover the build.
- **An empty `map_url` unloads** (2026-10-19): the switch to a 2D map. The
  surface map is dropped, `map_available` goes false (so goals run the 2D
  trees), the file parameters are emptied and `RESULT_SUCCESS` is answered.
  Without it a 3D → 2D switch left the old octree loaded and the task_runner
  running the 3D trees on it.
- `RESULT_MAP_DOES_NOT_EXIST` is answered **before anything changes**: the current map
  stays.
- Otherwise the current surface map is **dropped first** — plans fail with
  `map_loading` during the build, never on the old site's map, and only one
  tree is ever in memory — and a failed build (`RESULT_INVALID_MAP_DATA`)
  leaves **no map**, not the old one: every `Octo3D` plan then fails with
  `map_not_loaded` and the reason.
- `poses.txt` is always taken from beside the new tree (an explicit
  `poses_file` applies at boot only), and both paths are written back to
  `Octo3D.octomap_file` / `poses_file`, which `ros2 param set` otherwise
  refuses.
- The planner's own `clear_plan` is called, so `plan` stops re-sending the
  old map's route.
- One reload at a time on its own thread: a second call waits for the
  first, and the last one wins; a call during the boot build waits for it.
  Build-time parameters are not re-read — they still need a restart.

**The CLI.** `ros2 run syncai_global_planner octomap_planner_3d_cli` runs the
same core without the stack, and is the first thing to run on a new map:

```bash
octomap_planner_3d_cli map/<name>/octomap.bt --stats            # cells, rejects, components, floors, timings
   … --self-test                                                 # recompute random columns with OcTree::search()
   … --poses map/<name>/poses.txt --check-poses                  # the map's own track on traversable ground?
   … --poses … --inspect-misses 3 / --inspect-kf 19 / --inspect x y   # ASCII map of the cells around a point
   … --dump-image out.bmp [--image-z 5.1] [--image-region x0 y0 x1 y1 --image-scale 4] [--image-components]
   … --dump-pcd out.pcd                                          # x y z label reject unknown clearance component
   … --synthetic two_floors --write-bt out.bt                    # a test scene as a real .bt (the CI smoke's 3D fixture)
   … --plan sx sy sz gx gy gz --out path.csv / --plan-poses i j  # z are height hints
   … --plan-poses 0 1000 --plan-poses 2000 2250 --dump-html out.html   # 3D viewer: open in a browser
   … --poses … --plan-poses 0 2000 [--no-stair-links]           # with --poses, the track's stairs are links
   … --synthetic ramp|stairs|kerb|drop|wall|hole|two_floors      # the test scenes
   … --param max_slope_deg=25 --param heuristic_weight=1.5       # any SurfaceMapParams / SearchParams field
```

**Seeing a route.** Online, rviz2 from a workstation with
`config/rviz2/<robot_id>.rviz` already shows `plan` (`nav_msgs/Path`) over
the localizer's latched `map_cloud`, and the poses carry the floor's z, so an
`Octo3D` route lies on the floor of the point cloud; `Octo3D.publish_raw_plan`
adds the un-shortcut chain on `Octo3D/raw_plan`. Offline, `--dump-html`
writes a self-contained three.js page (~6 MB for a site map; the same page
style as the OctoMap viewer `clean_map`'s display build makes): the
traversable cells coloured by floor height, hole-filled ones tinted, a
subsample of the rejected cells in grey for the walls, and every `--plan` /
`--plan-poses` of the run as a tube with its start and goal, and, with
`--poses`, every stair link as a thick magenta tube with a cube on each
anchor — drawn on their own because a flight is all rejected cells, so
without them nothing in the page says where the stairs are (2026-10-15).
A "hide ground above z" slider looks at one floor of a multi-storey map;
it hides a route or a link only once its lowest point is above the cut, so
a 1F → 2F route stays whole while 1F is shown.
`--dump-image` draws the same routes on the top-down raster (blue, start
magenta, goal yellow), and `--dump-pcd` + `--out path.csv` are the raw
material for CloudCompare or a script.

On a workstation without ROS, `plugins/octomap_planner_3d/standalone/CMakeLists.txt`
builds the same binary against the system's octomap
(`cmake -S … -B /tmp/octo3d -DCMAKE_PREFIX_PATH=/opt/homebrew`); the core
headers include nothing of ROS on purpose.

**Known limits** (prototype):

- **`Octo3D` ignores the keepout (forbidden zones)** — by decision
  (2026-10-19), not yet built. The zones live only in the 2D global costmap
  (`KeepoutFilter`), whose cells `Octo3D` never reads, and the controllers'
  local costmap has no filter either — so on a 3D map **nothing keeps the
  robot out of a zone**. A map is 2D or 3D, never both (2026-10-19), and a
  3D map has no mask at all (no gridmap to size one from; the mask server
  runs idle), so the zone editor has no canvas there; a 2D map runs
  `move.xml` with `GridBased`, where the keepout works as on `dev`.
  Doing it needs a decision on floors first: a mask is one 2D image, a
  3D map has several storeys.
- `is_path_valid` and the republish trim run on the 2D costmap: fine on the
  ground floor, meaningless for a path upstairs or on the stairs (checked
  against the ground-floor gridmap). That is why `move_3d.xml`
  does not call `is_path_valid` and plans once per goal instead.
- A route over a stair link is only a route: climbing it is RPP 3D's stair mode
  (`FollowPathRPP3D`; `syncai_controller/README.md`, "Stairs"), which stops and
  switches the gait controller's RL policy over `set_policy_mode`. MPPI and
  the 2D RPP (`FollowPathRPP`) follow only its x / y. Climbing is untested on the robot.
- A stair link is the mapping run's track, drift and all, and it exists only
  where the robot walked; a stair blocked since (or walked only one way by a
  robot that cannot descend) is still linked both ways.
- A 2D backend's z = 0 goal is only right on the ground floor; upstairs it
  needs a real z or `goal_z_mode: start`.
- A map switch needs its own call: `map_server/load_map` reloads the gridmap
  only, and `Octo3D` reloads on **`planner_server/Octo3D/load_map`** (see
  "Map switch" below). The build-time parameters still need a
  planner_server restart.
- Shallow stairs with 0.1 m risers are a 19° ramp at this resolution; the
  tilt limit is the only guard, and a quadruped walks them anyway.
- The failure reason reaches `NavigateToGoal.error_msg` only as
  `PLAN_FAILED`; the text is in the planner log.

## Parameters

**Server** (`/**/planner_server`):

| Parameter | Default | Notes |
|---|---|---|
| `planner_plugins` | `["GridBased"]` (config: `["GridBased", "Octo3D"]`) | IDs; each needs `<id>.plugin` naming the type. With more than one loaded, an empty `planner_id` in a goal is an error rather than "the only plugin" |
| `expected_planner_frequency` | `1.0` (config: `20.0`) | Warning threshold only — a plan taking longer than 50 ms logs a missed-rate warning. Generous by design: the BT asks for a plan at most once a second, and only when the current path is blocked, so the warning is a canary for a pathological search, not a cadence anyone is trying to hit |
| `costmap_wait_timeout` | `5.0` (config: `5.0`) | Seconds a goal for a planner that reads the costmap (`usesCostmap()`; `GridBased`, not `Octo3D`) waits for it to be current before the goal is aborted; `<= 0` waits forever (upstream). Not the guard for a 3D-only map: an enabled static layer that has never received a map fails the goal at once (2026-10-19; until then such a goal hung the action thread with no way out), and a cancel ends the wait too. Dynamic |
| `plan_republish_rate` | `1.0` (config: `1.0`) | Hz at which the last plan is re-sent on `plan`, restamped and trimmed to start at the pose nearest the robot (the same nearest-pose search `is_path_valid` uses, so what is shown is what is checked; the whole plan when the robot pose is unavailable); `0` = only when a plan is made. Needed since `move.xml` plans only when the path is blocked: without it a viewer subscribing mid-drive never sees the route. It keeps showing the last route after a goal succeeds or fails, since the planner cannot tell when the navigation finished; a **cancelled** goal is the exception, reported by the task_runner through `clear_plan`. Read once at startup |
| `is_path_valid.footprint_check` | `true` | Also test the padded footprint's perimeter against LETHAL at each pose ahead of the robot — RPP's own collision test, run on the global costmap. `false` restores the centre-cell test alone (what the service did until 2026-10). Dynamic |
| `is_path_valid.heading_margin` | `0.35` | Radians either side of the path heading the perimeter is also tested at. RPP tracks with up to `rotate_to_heading_min_angle` (0.25) of heading error before it rotates in place, and the gait adds yaw sway; at 0.35 the rectangle reaches 0.365 m sideways, which is what catches a person 0.3 m beside the route — and what rejects a straight passage narrower than 0.73 m (doors pass). `0` tests the path heading only. Dynamic, clamped to [0, π/2] |

`expected_planner_frequency`, `costmap_wait_timeout` and the two `is_path_valid.*` parameters are what
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
(`SmacPlanner2D`) and `Octo3D` (`OctomapPlanner3D`), obstacle
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
reject a path for grazing a zone edge. That planner is `GridBased`:
**`Octo3D` does not read the keepout** (see its "Known limits"), so on a 3D
map, whose trees plan with `Octo3D`, there are no zones. Running the mask source
by hand, outside the session:

```bash
ros2 launch syncai_map_server costmap_filter_info.launch.py            # map/<name>/keepout.yaml from the INI
ros2 launch syncai_map_server costmap_filter_info.launch.py mask_yaml:=map/other/keepout.yaml
```

**OctomapPlanner3D** (`Octo3D.*`; the YAML carries the reasoning per line):

| Parameter | Default | Notes |
|---|---|---|
| `octomap_file` | `""` (launch: `map/<name>/octomap.bt`) | Derived by the launch file from the directory of `[map] pcd`; empty or missing → every `Octo3D` plan fails with `map_not_loaded`, nothing else is affected |
| `poses_file` | `""` (= `poses.txt` next to `octomap_file`) | The map's track, for the stair links; missing → a warning and no links. Build-time |
| `body_frame`, `lidar_height` | `pointlio_body` (launch: `<robot_id>/pointlio_body`), `0.481` | The frame the robot's real height is read from, and the sensor's height above the ground (`map_clean.recipe.json`'s `lidar_height`). Dynamic |
| `goal_z_mode` | `request` | `request`: the goal's own z picks the floor; `start`: the robot's floor. Dynamic |
| `robot_radius`, `body_clearance` | `0.25`, `0.6` | Footprint disc (= the costmap's inscribed radius) and the band above a surface the body needs. Build-time |
| `max_step_height`, `max_slope_deg` | `0.17`, `20.0` | The legs' step (two cells at 0.1 m); tilt of the fitted plane under a disc spanning more than one step. Build-time |
| `footprint_min_support` | `0.8` | Share of the disc's columns that must hold a usable surface; the rest may be gaps (never observed), not walls or drops. `1.0` = every column. Build-time |
| `max_unknown_clearance_cells`, `min_known_free_above` | `-1` (all), `1` | Unknown tolerated in the band; observed-free voxels required right above a surface. Build-time |
| `hole_fill_radius`, `hole_fill_min_support`, `hole_fill_passes` | `0.3`, `0.5`, `2` | See the build, step 3. Build-time |
| `clearance_cost_radius` | `0.55` | Reach of the clearance field from the traversable boundary. Build-time |
| `stair_links`, `stair_min_rise`, `stair_max_length`, `stair_anchor_z_window`, `stair_anchor_min_area`, `stair_smooth_passes` | `true`, `0.3`, `60.0`, `0.3`, `10.0` m², `2` | Stair links from the track (see **Stairs**). Build-time |
| `stair_cost_multiplier` | `2.0` | Factor on a link's 3D length (≥ 1). Dynamic |
| `clearance_cost_weight`, `clearance_cost_scaling` | `1.0`, `1.5` | Edge cost multiplier inside the band. Dynamic |
| `inferred_cost_multiplier` | `1.2` | Prefer observed floor. Dynamic |
| `gap_cost_weight` | `1.0` | Cost factor 1 + weight × gaps / disc columns. Dynamic |
| `heuristic_weight`, `max_iterations`, `max_planning_time` | `1.0`, `2000000`, `2.0` s | Weighted A* and the guards. Dynamic |
| `snap_xy_radius`, `snap_z_window` | `0.5`, `1.0` | How far a start / goal may be moved to reach traversable ground. Dynamic |
| `shortcut`, `interpolation_resolution`, `path_z_offset` | `true`, `0.1`, `0.0` | Post-processing. Dynamic |
| `publish_raw_plan` | `false` | Also publish the un-shortcut chain on `Octo3D/raw_plan` |

A build-time parameter set through `ros2 param set` is refused with the
reason; it needs a planner_server restart.

## Interfaces

With the node at `/<robot_id>`:

| Interface | Name | Type |
|---|---|---|
| Action | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` |
| Action | `compute_path_through_poses` | `nav2_msgs/ComputePathThroughPoses` |
| Service | `is_path_valid` | `nav2_msgs/IsPathValid` — is the part of a path from the pose nearest the robot onward free? Two tests per pose: the centre cell `< INSCRIBED` (the test SmacPlanner2D expands with; unknown counts as blocked, matching `allow_unknown: false`) and, with `is_path_valid.footprint_check`, no LETHAL cell under the padded footprint's perimeter at the path heading ± `heading_margin` (RPP's own collision test, so a path that passes is one RPP will drive; unknown under the perimeter does not count, since the local costmap never shows RPP unknown). Called once a second by the 2D `move.xml`'s `IsPathValid`, which replans only once the same spot has answered `false` for 10 s (`blocked_time`, 2026-10); the 3D tree (`Octo3D` + `FollowPathRPP3D`) plans once per goal and does not call it, since it judges on the 2D costmap. Answers `false` for an empty path or a frame other than the costmap's; answers `true` when there is **no robot pose** (a TF miss must not force a replan whose own start lookup fails and whose recovery clears the global costmap). Served on the main executor, so it never queues behind a plan. Where the two tests still disagree with RPP: marks only the local costmap holds, RPP's in-place rotation sweep, heading error beyond the margin |
| Service | `planner_server/Octo3D/load_map` | `nav2_msgs/LoadMap` (2026-10-19), map_server's own type — reload `Octo3D` from `map_url` (`map/<name>/octomap.bt`; `poses.txt` from beside it) on a map switch. See "Map switch" under `OctomapPlanner3D`. Its own callback group and executor thread, so a multi-second build never stalls `is_path_valid`, `clear_plan` or the republish |
| Service | `clear_plan` | `std_srvs/Empty` (2026-10-14) — forget the last plan and send an empty `plan` (frame `map`, not empty: the backend drops any other frame before it looks at the poses), then two more at `plan_republish_rate`, because the backend reads `plan` BEST_EFFORT and one dropped message would leave the console drawing the old route. Called by the task_runner when a navigation is **cancelled**, which until then left the cancelled route on `plan` until the next goal. A new plan stops the repeats. Main executor, the republish timer's thread |
| Publisher | `plan` | `nav_msgs/Path` (visualization; skipped when nothing is subscribed). Sent when a plan is made, and the last one again at `plan_republish_rate`, from the robot's nearest pose onward; empty after a `clear_plan` |
| Publisher | `Octo3D/raw_plan` | `nav_msgs/Path`, only with `Octo3D.publish_raw_plan`: the A* cell chain before the shortcut, for comparing against `plan` |

Plus everything the internal costmap exposes under
`/<robot_id>/global_costmap/…` — `costmap`, `costmap_updates`,
`published_footprint`, `get_costmap`, and the three `clear_*_global_costmap`
services the BT's recovery branch calls.

## Running

```bash
ros2 launch syncai_global_planner planner_server.launch.py
ros2 launch syncai_global_planner planner_server.launch.py \
    system_config:=config/instances/robot02.ini
```

A plan from the OctoMap surface planner without the task runner (the same
planner the BT asks for on a 3D map), and the same core offline on the
map's own tree (see OctomapPlanner3D above):

```bash
ros2 action send_goal /<robot_id>/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: Octo3D}"
ros2 run syncai_global_planner octomap_planner_3d_cli map/<name>/octomap.bt --stats \
  --poses map/<name>/poses.txt --check-poses
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
- **The global costmap's `scan` source is silent on the 3D stack.** Nothing
  publishes `scan` there (the merger went with `bringup_2d`), so only the
  `pointcloud` source marks. It is kept, like the controller's, for a setup
  that does publish one, and the launch file overrides its `sensor_frame` to
  `<robot_id>/scan`, the frame the merger stamped — the same value
  `syncai_controller`'s launch sets. Until 2026-10 this launch set
  `<robot_id>/laser` instead, a frame nothing in the workspace broadcast, so a
  scan could not have been raytraced here even had one arrived; the Livox
  driver no longer stamps `laser` either (it stamps `<robot_id>/lidar_top`).
  A setup that brings a scan back also needs the `base_link → scan` TF, which
  left with `bringup_2d` too.
- **Dynamic parameter changes are refused mid-plan** by the shared mutex, and
  only `expected_planner_frequency` is handled at all — plugin parameters go to
  each plugin's own callback.
- **`is_path_valid` judges an `Octo3D` path on the 2D costmap.** On the
  ground floor that is the same check `GridBased` paths get; upstairs the
  path is compared with the ground-floor gridmap and reads blocked, so a 2D
  tree with `Octo3D` swapped in would replan into the same path once a
  second. `move_3d.xml` therefore leaves `IsPathValid` out and
  plans once per goal — which also means no replan round a blocker. There
  is no 3D-aware validity check yet.
- **`Octo3D` does not follow a map switch by itself.** Nothing watches the
  INI: the caller of the switch (the backend) must call
  `planner_server/Octo3D/load_map` with the new `octomap.bt`, as it calls
  `map_server/load_map` for the gridmap (2026-10-19; until then a switch
  needed a planner_server restart, and without one `Octo3D` went on planning
  on the old site's map). A build-time parameter change still needs the
  restart — the session's `plan_ctrl` pane, or `restart_mode`.
- **`Octo3D` trips the `expected_planner_frequency` warning** on a long
  plan only in theory (a 50 m search is a few ms); if it does, it is the
  canary described under Parameters, not a fault.

Upstream references: [`nav2_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_planner),
[`nav2_navfn_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_navfn_planner),
and [`nav2_smac_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_smac_planner).
