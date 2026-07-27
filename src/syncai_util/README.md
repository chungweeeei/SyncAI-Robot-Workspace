# syncai_util

The bottom of the dependency graph: shared helpers every other C++ package in
the stack pulls in. Port of the parts of `nav2_util` this workspace actually
uses.

> Both `package.xml` and `CLAUDE.md` describe this as "header-only". It is not —
> five of the eleven headers have compiled implementations in `src/`, and the
> package builds `libsyncai_util.so`. Consumers must link it, not just include
> the headers.

| Header | Kind | Provides | Used by |
|---|---|---|---|
| `simple_action_server.hpp` | header-only | The action-server wrapper behind every action in the stack | behavior_tree, controller, planner |
| `node_utils.hpp` | mixed | Parameter/plugin/namespace helpers | behavior_tree, controller, costmap_2d, planner, task_runner |
| `robot_utils.hpp` | compiled | `getCurrentPose`, pose transforms, twist validation | controller, costmap_2d, robot_state, task_runner |
| `geometry_utils.hpp` | header-only | Distances, path length, iterator helpers | amcl, controller, map_server, planner, task_runner |
| `node_thread.hpp` | compiled | Spin a node or executor on a background thread | controller, costmap_2d, planner |
| `odometry_utils.hpp` | compiled | `OdomSmoother` — time-windowed average twist | task_runner |
| `odom_subscriber.hpp` | header-only | `OdomSubscriber` — latest planar twist | controller |
| `line_iterator.hpp` | header-only | Bresenham ray tracing | costmap_2d |
| `validate_messages.hpp` | header-only | NaN/inf/consistency checks for common msgs | amcl, costmap_2d |
| `string_utils.hpp` | compiled | `split`, `strip_leading_slash` | amcl |
| `occ_grid_values.hpp` | header-only | `OCC_GRID_UNKNOWN/FREE/OCCUPIED` (−1 / 0 / 100) | costmap_2d, map_server |

## SimpleActionServer

The largest and most load-bearing piece (635 lines). Every action server in the
stack — `NavigateToPose`, `ComputePathToPose`, `ComputePathThroughPoses`,
`FollowPath` — is a `SimpleActionServer<ActionT>`.

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
- **`add_namespaces(top, sub)`** — how `Costmap2DROS` builds its
  `/<robot_id>/global_costmap` sub-namespace.
- `sanitize_node_name`, `generate_internal_node_name`, `generate_internal_node`,
  `time_to_string` — for creating internal helper nodes with unique, legal names.

## robot_utils

- **`getCurrentPose(pose, tf, global_frame, base_frame, timeout, stamp)`** — the
  standard "where is the robot" lookup, used by the costmap, the controller, the
  task runner's feedback and `syncai_robot_state`. Returns `false` rather than
  throwing on a TF failure, which is why those callers all degrade gracefully.
- `transformPoseInTargetFrame`, the two `getTransform` overloads (same-time and
  cross-time), and `validateTwist` (NaN/inf guard).

## The two odometry helpers

Easy to confuse, and the difference matters:

| | `OdomSubscriber` | `OdomSmoother` |
|---|---|---|
| Header | `odom_subscriber.hpp` (header-only) | `odometry_utils.hpp` (compiled) |
| Behaviour | Keeps the **latest** sample; zeroes everything but linear x/y and angular z | Averages a **time window** of samples (`filter_duration`, 0.3 s as constructed) |
| Used by | `syncai_controller` | `syncai_task_runner` (feedback ETA) |

The controller deliberately takes the *unsmoothed* one — which is exactly why
`RegulatedPurePursuitController` clamps acceleration against its own previous
command rather than the measured twist: the raw Point-LIO twist carries the
quadruped's gait sway, and there is no smoother in that path. See
`syncai_controller`'s README.

## geometry_utils

Header-only and all `inline`:

- `orientationAroundZAxis(yaw)` → `geometry_msgs/Quaternion`
- `euclidean_distance(...)` overloaded for `Point`, `Pose`, `PoseStamped` and
  `Pose2D`, each with an `is_3d` flag (default 2D)
- `calculate_path_length(path, start_index)` — the remaining-distance primitive
  behind the controller's feedback, the task runner's `distance_remaining`, and
  the controller's patrol-loop goal gate
- `min_by(begin, end, getter)` and
  `first_after_integrated_distance(begin, end, getter)` — the pair RPP uses to
  find the closest path pose within a bounded search window

## The rest

- **`line_iterator.hpp`** — Bresenham ray tracing, used by the costmap's obstacle
  layer for marking and clearing along a beam.
- **`validate_messages.hpp`** — overloaded `validateMsg()` for `double`,
  `std::array<double, N>`, `Time`, `Header`, `Point`, `Quaternion`, `Pose`,
  `PoseWithCovariance(Stamped)`, `MapMetaData`, `OccupancyGrid`. Rejects NaN/inf,
  empty `frame_id`s, non-unit quaternions and grids whose `data` size does not
  match their metadata — the guard against a malformed message poisoning the
  costmap or AMCL.
- **`node_thread.hpp`** — RAII: construct it with a node or executor and it spins
  on a background thread; the destructor cancels and joins. Both costmap owners
  use it so `Costmap2DROS` runs independently of its host's executor.
- **`occ_grid_values.hpp`** — the `nav_msgs/OccupancyGrid` value convention
  (−1 unknown, 0 free, 100 occupied), distinct from `syncai_costmap_2d`'s
  internal 0–255 cost scale.

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
  `node_thread.hpp` and `occ_grid_values.hpp`. Harmless in practice — nav2_util
  is not in this workspace — but it would collide if both were ever included in
  one translation unit.
- **`OdomSubscriber` silently drops non-planar twist components.** Vertical or
  roll/pitch velocity read as zero; that is intended for this planar stack but
  surprising if reused.
