# syncai_planner

The global planning half of the nav stack: an action server that owns the
**global costmap** and hosts `GlobalPlanner` plugins. Port of `nav2_planner`,
with `nav2_navfn_planner` merged in as a plugin of this package rather than
living in its own.

```
syncai_task_runner ──ComputePathToPose (nav2_msgs)──►  planner_server
                   ──ComputePathThroughPoses────────►       │
                                                            │  global_costmap (internal node)
                                            ┌───────────────┴───────────────┐
                                            ▼                               ▼
                                     NavfnPlanner                  StraightLinePlanner
                                  (Dijkstra / A*)                 (reference / testing)
                                            │
                                            ▼
                                   nav_msgs/Path ──► syncai_controller (FollowPath)
```

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
global_planner_plugin.xml          pluginlib manifest, exported to syncai_nav_core
params/planner_server_params.yaml       2D / sim
params/planner_server_3d_params.yaml    3D / real robot
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

### NavfnPlanner (`syncai_planner/NavfnPlanner`)

The classic NavFn potential-field planner: propagate a navigation function
outward from the goal over the costmap, then follow the gradient back from the
start.

| Parameter | Default | Notes |
|---|---|---|
| `tolerance` | `0.5` (config: `0.3`) | If the goal cell is unreachable, search a square of ±tolerance around it for the nearest reachable pose |
| `use_astar` | `false` | `false` = Dijkstra (full expansion), `true` = A* (heuristic, faster, less optimal) |
| `allow_unknown` | `true` | Plan through `NO_INFORMATION` cells |
| `use_final_approach_orientation` | `false` | Set the last pose's yaw to the path's approach direction instead of the requested goal yaw |

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
controller a known path for tuning. Loaded in the 2D params, dropped from the 3D
params.

Do not select it on a real robot expecting collision avoidance — it will happily
plan through a wall.

## Parameters

**Server** (`/**/planner_server`):

| Parameter | Default | Notes |
|---|---|---|
| `planner_plugins` | `["GridBased"]` | IDs; each needs `<id>.plugin` naming the type |
| `expected_planner_frequency` | `1.0` (config: `10.0`) | Warning threshold only |

`expected_planner_frequency` is the only dynamically reconfigurable parameter,
and the callback takes the same mutex the plan cycle holds.

**Global costmap** (`/**/global_costmap`) — the full costmap parameter set; see
`syncai_costmap_2d`'s README. The choices specific to a *global* costmap here:
`rolling_window: false`, `track_unknown_space: true`, `global_frame: map`, and
a low `update_frequency: 1.0` (the static map rarely changes).

The footprint is a rectangle with half-extents 0.28 × 0.20 and **must stay in
sync with the local costmap** in `syncai_controller`. A circular
`robot_radius: 0.22` was tried and was oversized enough that RPP rejected valid
paths through ~0.6 m gaps.

### 2D vs 3D params

| | `planner_server_params.yaml` | `planner_server_3d_params.yaml` |
|---|---|---|
| `use_sim_time` | `true` | `false` |
| `planner_plugins` | `GridBased`, `StraightLine` | `GridBased` only |
| Keepout filter | `filters: ["keepout_filter"]` | **not configured** |
| Obstacle sources | `scan` | `scan` **and** `pointlio/body_cloud` |

The keepout filter needs `costmap_filter_info_server` and a mask server running
alongside, otherwise it warns "Filter mask was not received" every 2 s:

```bash
ros2 launch syncai_map_server costmap_filter_info.launch.py
```

The 3D pointcloud source deliberately leaves `sensor_frame` empty so the
observation buffer uses the cloud's own header frame as the raytrace origin —
meaning no per-robot launch override, but also that it only works after
`/localizer/relocalize`.

## Interfaces

With the node at `/<robot_id>`:

| Interface | Name | Type |
|---|---|---|
| Action | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` |
| Action | `compute_path_through_poses` | `nav2_msgs/ComputePathThroughPoses` |
| Publisher | `plan` | `nav_msgs/Path` (visualization; skipped when nothing is subscribed) |

Plus everything the internal costmap exposes under
`/<robot_id>/global_costmap/…` — `costmap`, `costmap_updates`,
`published_footprint`, `get_costmap`, and the three `clear_*_global_costmap`
services the BT's recovery branch calls.

## Running

```bash
ros2 launch syncai_planner planner_server.launch.py
ros2 launch syncai_planner planner_server.launch.py \
    params_file:=$PWD/src/syncai_planner/params/planner_server_3d_params.yaml
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
- **`StraightLinePlanner` ignores obstacles entirely.** It is loaded by default
  in the 2D params; do not select it via `planner_id` on hardware.
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

Upstream references: [`nav2_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_planner)
and [`nav2_navfn_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_navfn_planner).
