# syncai_util

The bottom of the dependency graph: shared helpers every other C++ package in
the stack pulls in. Port of the parts of `nav2_util` this workspace actually
uses.

> Not header-only: six of the thirteen headers have compiled implementations in
> `src/`, and the package builds `libsyncai_util.so`. Consumers must link it,
> not just include the headers. Since 2026-10-20 it also depends on
> `syncai_common` (for `StairMode`'s service and messages), so the interface
> checkout is needed to build anything above it.

| Header | Kind | Provides | Used by |
|---|---|---|---|
| `simple_action_server.hpp` | header-only | The action-server wrapper behind every action in the stack | behavior_tree, controller, planner |
| `node_utils.hpp` | mixed | Parameter/plugin/namespace helpers | behavior_tree, controller, costmap_2d, planner, task_runner |
| `robot_utils.hpp` | compiled | `getCurrentPose`, pose transforms, twist validation | controller, costmap_2d, robot_state, task_runner |
| `geometry_utils.hpp` | header-only | Distances, path length, iterator helpers | controller, map_server, planner, task_runner |
| `node_thread.hpp` | compiled | Spin a node or executor on a background thread | controller, costmap_2d, planner |
| `odometry_utils.hpp` | compiled | `OdomSmoother` — time-windowed average twist | task_runner |
| `odom_subscriber.hpp` | header-only | `OdomSubscriber` — latest planar twist | controller |
| `line_iterator.hpp` | header-only | Bresenham ray tracing | costmap_2d |
| `path_height.hpp` | header-only | `pathHasHeight` (a path whose poses do not all share one z: OctomapPlanner3D's) and `lookupGroundZ` (the ground under the robot = the lidar frame's z minus its height, since `base_link` is planar). Moved here from `syncai_controller` on 2026-10-20 so MPPI could share it | controller (server goal check, RPP 3D), mppi_controller, costmap_2d (`TerrainLayer`) |
| `stair_mode.hpp` | compiled | `StairMode` — the stair state machine both controllers run on a path with heights: detects a climbing stretch, switches the gait policy over `set_policy_mode` standing still, settles, reports on the latched `stair_mode`. `initialize()` for a node (parameters under the plugin's name), `configure()` with callbacks for a test (`test/test_stair_mode.cpp`). Lifted out of RPP 3D on 2026-10-20; the climb itself stays each controller's. `findCorner()` (static, 2026-10-21) finds the first landing corner of a plan for both controllers' stair carrots. `setPlan()` keeps the plan with `medianHeights()` applied (static, 2026-10-24: each inner z the median of three poses), so a one-voxel spike or dip in the planner's heights no longer makes a ramp read as a stair — on dp's arched ramp one -0.5 pose read 0.3 m of rise for a 0.2 m slope; a flight's monotonic heights pass unchanged. `plannerPassages()` (2026-11-02) hands out the plan's narrow passages from the same `plan_crossings` subscription, matched to the same plan, for MPPI's speed cap | controller (RPP 3D), mppi_controller |
| `path_cusps.hpp` | header-only | `findCusps` and `splitAtCusps` (2026-11) — where a path turns back on itself (the direction `window`, 0.4 m, before and after a pose differs by more than `min_angle`, 2.0 rad, with the path within 0.3 m of level over 1 m either side so a switchback stair's landing is stair mode's), and the path cut into legs there, each ending on its cusp facing the next. A windowed angle rather than upstream MPPI's consecutive-step dot product, which misses an A* return leg one cell aside (two 90° corners). Made because robot01's out-and-back patrol stalled 0.84 m short of its turn with MPPI handed the path whole (the header has the mechanism); the controller server drives such a path leg by leg. `test/test_path_cusps.cpp` | controller (server) |
| `path_progress.hpp` | header-only | `firstLocalMinimum` and `PathProgress` (2026-11) — where the robot is on a path that may pass back over itself: read forward from the last place and stop at the **first** local minimum of the distance (a rise beyond `kDefaultHysteresis`, 0.3 m, ends the search), not the window's or the whole path's minimum. Made for the patrol route (`navigate_through_poses`): an out-and-back's return leg lies on top of the outgoing one and a loop ends on its start, and a nearest-pose search took the later leg. `PathProgress` keeps the place between calls (a new path, by stamp / size / end pose, starts over) for the holders of a whole path, and passes a cusp (`path_cusps.hpp`) once the robot is within 0.6 m of it facing the way the path leaves it — the turn in place on a cusp drifts past the 0.3 m hysteresis — or when told to (`passTo()`, the controller server on handing over a leg); MPPI and RPP 3D call `firstLocalMinimum` from their pruned front. `test/test_path_progress.cpp` | controller (server goal gate and feedback, RPP 3D), mppi_controller, planner (server, Octo3D's `isPathValid`), task_runner (feedback) |

## SimpleActionServer

The largest and most load-bearing piece (635 lines). Every action server in the
stack — `NavigateToPose`, `ComputePathToPose`, `FollowPath`, `DockToPose` —
is a `SimpleActionServer<ActionT>`.

It wraps `rclcpp_action::Server` with a **single worker thread and a one-slot
pending-goal queue**, which is what turns "handle N concurrent goals" into the
much simpler "run one goal to completion, optionally swapping in a replacement
mid-flight":

```cpp
action_server_ = std::make_unique<SimpleActionServer<FollowPath>>(
    shared_from_this(), "follow_path",
    std::bind(&ControllerServer::computeControl, this),   // execute callback
    nullptr,                                              // completion callback
    500ms,                                                // server_timeout
    true);                                                // spin_thread
action_server_->activate();
```

The execute callback is expected to **block until the goal is done**, polling the
server as it goes:

| Call | Meaning |
|---|---|
| `is_server_active()` | Deactivated or shutting down — bail out |
| `is_cancel_requested()` | A cancel arrived; call `terminate_all()` |
| `is_preempt_requested()` | A new goal is queued |
| `accept_pending_goal()` | Swap to it **in place**, without restarting the callback |
| `terminate_pending_goal()` | Reject the queued goal, keep running the current one |
| `succeeded_current(result)` / `terminate_current(result)` / `terminate_all(result)` | Finish |
| `publish_feedback(fb)` | |

`handle_goal` **always accepts** at the rclcpp layer; whether a goal is really
allowed is decided afterwards by the execute callback (or, in the task runner, by
`Navigator::onGoalReceived` and its mutex). `handle_accepted` queues the new goal
as pending and asks the running one to stop, then `work()` picks the pending goal
up on the same thread instead of starting a second one.

`spin_thread=true` gives the server its own callback group and executor thread,
which is why a blocking control or planning loop never starves the owning node's
main executor. All three servers in this stack use it.

## node_utils

Small functions with outsized consequences:

- **`declare_parameter_if_not_declared(node, name, value)`** — the idiom used by
  every plugin in the stack, so a plugin can declare its own parameters without
  clashing with a host that already declared them.
- **`get_plugin_type_param(node, plugin_name)`** — reads `<plugin_name>.plugin`
  and **calls `exit(-1)` if it is missing**. This one line is why a typo in a
  plugin name takes down the planner, controller or costmap process at startup
  rather than degrading.
- **`copy_all_parameters(parent, child)`** — used by `BtActionServer` to clone the
  host node's parameters onto the internal BT client node.
- **`add_namespaces(top, sub)`** — how `CostmapROS` builds its
  `/<robot_id>/global_costmap` sub-namespace.
- `sanitize_node_name`, `generate_internal_node_name`, `generate_internal_node`,
  `time_to_string` — for creating internal helper nodes with unique, legal names.

## robot_utils

- **`getCurrentPose(pose, tf, global_frame, base_frame, timeout, stamp)`** — the
  standard "where is the robot" lookup, used by the costmap, the controller, the
  task runner's feedback and `syncai_robot_state`. Returns `false` rather than
  throwing on a TF failure, which is why those callers all degrade gracefully.
- `transformPoseInTargetFrame`.

Removed on 2026-10-28 as never called: the two `getTransform` overloads,
`validateTwist`, `string_utils.hpp` (`split`, `strip_leading_slash`; AMCL's,
gone with the 2D bringup), the `Pose2D` overload of `euclidean_distance` and
the two odometry helpers' `getTwistStamped()`.

## The two odometry helpers

Easy to confuse, and the difference matters:

| | `OdomSubscriber` | `OdomSmoother` |
|---|---|---|
| Header | `odom_subscriber.hpp` (header-only) | `odometry_utils.hpp` (compiled) |
| Behaviour | Keeps the **latest** sample; zeroes everything but linear x/y and angular z | Averages a **time window** of samples (`filter_duration`, 0.3 s as constructed) |
| Used by | `syncai_controller` | `syncai_task_runner` (feedback ETA) |

The controller deliberately takes the *unsmoothed* one — which is exactly why
the controllers (MPPI, `RegulatedPurePursuitController3D`) clamp acceleration
against their own previous command rather than the measured twist: the raw Point-LIO twist carries the
quadruped's gait sway, and there is no smoother in that path. See
`syncai_controller`'s README.

## geometry_utils

Header-only and all `inline`:

- `orientationAroundZAxis(yaw)` → `geometry_msgs/Quaternion`
- `euclidean_distance(...)` overloaded for `Point`, `Pose` and `PoseStamped`,
  each with an `is_3d` flag (default planar)
- `calculate_path_length(path, start_index)` — the remaining-distance primitive
  behind the controller's feedback, the task runner's `distance_remaining`, and
  the controller's patrol-loop goal gate
- `min_by(begin, end, getter)` and
  `first_after_integrated_distance(begin, end, getter)` — the pair RPP uses to
  find the closest path pose within a bounded search window

## plan_crossings

**`plan_crossings.hpp`** (2026-11-02) — the format of the planner's
`plan_crossings` sidecar, one JSON object on a `std_msgs/String`:
`PlanCrossings` (`known`, the plan's stamp and pose count, `crossings`) and
`PlanCrossing` (kind, gait, evidence, rise, inclusive pose span, turns, the
plan's points over it; for kind `narrow`, a passage narrower than the robot
turns in, also `width` and `heading`), `toJson()` / `fromJson()` (nlohmann, behind the
`.cpp`; a message it cannot read is a `false` with a reason, never a throw or
a half-read). planner_server writes it (`GlobalPlanner::planAnnotations()`),
`StairMode` and the costmap's `TerrainLayer` read it. The header has the
schema and why it is not a `syncai_common` message.
`test/test_plan_crossings.cpp` is ROS-free.

## The rest

- **`line_iterator.hpp`** — Bresenham ray tracing, used by the costmap's
  footprint collision checkers (line cost along a footprint edge). The
  obstacle layer's marking and clearing along a beam used it too, until that
  layer went with 2D navigation (2026-10-28).
- `validate_messages.hpp` (the `validateMsg()` overloads for `OccupancyGrid`
  and the messages inside it) went with its only user, the costmap's static
  layer, on 2026-10-28.
- **`node_thread.hpp`** — RAII: construct it with a node or executor and it spins
  on a background thread; the destructor cancels and joins. Both costmap owners
  use it so `CostmapROS` runs independently of its host's executor.
- `occ_grid_values.hpp` (the `nav_msgs/OccupancyGrid` −1 / 0 / 100
  convention) went on 2026-10-28 with its last users, `syncai_map_server` and
  `Costmap`'s OccupancyGrid constructor.

## Build

```bash
colcon build --packages-select syncai_util
```

Nothing here is a node and there are no tests beyond the ament linters. Because
almost everything else depends on it, a change here means rebuilding broadly:

```bash
colcon build --packages-above syncai_util
```

## Gotchas

- **It is not header-only.** Linking is required; `ament_target_dependencies(...
  syncai_util)` handles it, but a consumer that only adds the include path will
  fail at link time on `getCurrentPose`, `NodeThread`, `OdomSmoother` and the
  string helpers.
- **`get_plugin_type_param` exits the process.** Any "plugin not found kills the
  server" behaviour documented in the planner, controller and costmap READMEs
  traces back here.
- **`SimpleActionServer`'s execute callback must block.** Returning early ends
  the goal — there is no "come back later" mode. A callback that throws
  terminates all goals and stops the worker thread.
- **Include guards still carry `nav2_util` / `NAV2_UTIL` names** in
  `node_thread.hpp`. Harmless in practice — nav2_util
  is not in this workspace — but it would collide if both were ever included in
  one translation unit.
- **`OdomSubscriber` silently drops non-planar twist components.** Vertical or
  roll/pitch velocity read as zero; that is intended for this planar stack but
  surprising if reused.
