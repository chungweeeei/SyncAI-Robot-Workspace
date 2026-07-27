# syncai_task_runner

The **BT navigator**: the node that serves `nav2_msgs/NavigateToPose` and, for
each goal, ticks a behavior tree that drives the planner and controller. Port of
`nav2_bt_navigator`.

This is the top of the navigation stack — everything below it (planner,
controller, costmaps, BT nodes) exists to serve the tree ticked here.

> The repo `README.md` still calls this package `syncai_bt_navigator`. That
> package does not exist; this is it.

```
syncai_backend (RobotWorkflow MOVE step) ──NavigateToPose──►  task_runner
RViz "2D Goal Pose" ──/goal_pose topic────────────────────►       │
                                                                  │  NavigateToPoseNavigator
                                                                  ▼
                                                    BtActionServer<NavigateToPose>
                                                                  │  ticks behavior_trees/move.xml
                                              ┌───────────────────┴───────────────────┐
                                              ▼                                       ▼
                                  ComputePathToPose ──► syncai_planner    FollowPath ──► syncai_controller
                                  ClearEntireCostmap ─► syncai_costmap_2d (both costmaps)
```

## Structure

The package is deliberately split into a **host node** and **navigators**, even
though only one navigator exists today:

| Piece | Role |
|---|---|
| `TaskRunner` (`syncai_task_runner.cpp`) | The `rclcpp::Node`. Owns the shared TF buffer, the odom smoother, the navigator mutex, and the parameters. Hosts navigators; contains no navigation logic. |
| `Navigator<ActionT>` (`navigator.hpp`) | Header-only template base. Wraps a `syncai_behavior_tree::BtActionServer<ActionT>` and implements the goal-muxing that keeps two navigators from driving the robot at once. |
| `NavigateToPoseNavigator` | The one concrete navigator: binds `NavigateToPose`, loads `move.xml`, computes feedback. |
| `behavior_trees/move.xml` | The default tree |

`Navigator` is templated on the action type because a second navigator
(`NavigateThroughPoses`, a docking action, …) would bind a different one. The
`NavigatorMutex` and the `FeedbackUtils` plumbing only make sense in that light —
with one navigator the mutex can never contend.

**Three-phase startup**, same pattern as the planner and controller:

```cpp
auto node = std::make_shared<TaskRunner>();
node->initialize();   // needs shared_from_this(): TF, odom smoother, navigator
rclcpp::spin(node);
node->cleanup();      // tears the navigator down before the node dies
```

The constructor uses `automatically_declare_parameters_from_overrides(true)`, so
any key in the params YAML becomes a parameter even if no code declares it —
which is how `BtActionServer`'s parameters (`bt_loop_duration`,
`default_server_timeout`, `wait_for_service_timeout`, `always_reload_bt_xml`)
arrive. The flip side is that a typo'd key is silently accepted instead of
rejected.

## Per-goal flow

`Navigator::onGoalReceived` is the mux point:

```
another navigator running?      → reject the goal
subclass goalReceived(goal):
    loadBehaviorTree(goal.behavior_tree)   ← empty string = the default tree
    failed to load?             → reject
    initializeGoalPose(goal):
        reset number_recoveries to 0, stamp start_time_
        write goal.pose to the blackboard under goal_blackboard_id ("goal")
claim the navigator mutex
  ⋯ BtActionServer runs the engine loop, calling onLoop() each tick ⋯
onCompletion: release the mutex
```

The tree reads the goal from the blackboard, not from the action goal — that
indirection is what lets `move.xml` reference `goal="{goal}"` without knowing
anything about the action type.

### Feedback

`onLoop()` fires once per BT tick and publishes `NavigateToPose` feedback:

| Field | Computed from |
|---|---|
| `current_pose` | TF `global_frame → base_frame` |
| `distance_remaining` | Path length from the closest pose on the blackboard's `path` to its end |
| `estimated_time_remaining` | `distance_remaining / speed`, from the odom smoother; zero below 0.01 m/s or under 0.1 m remaining |
| `number_of_recoveries` | Blackboard `number_recoveries`, incremented by the `ClearEntireCostmap` BT nodes |
| `navigation_time` | Now minus `start_time_` |

The path lookup is wrapped in `try { … } catch (...) {}` — before the first
`ComputePathToPose` completes there is no `path` on the blackboard, so
`distance_remaining` and the ETA silently stay at zero for the first tick or two.

### Preemption

A new goal arriving mid-navigation is accepted **only if it uses the same BT XML**
(or leaves `behavior_tree` empty while the default tree is running). It then
replaces the goal pose in place, with no restart and no re-plan cycle — this is
what makes patrol-style goal updates cheap.

A goal requesting a *different* tree is rejected with a warning, because
switching trees would require cancelling the current goal rather than preempting
it. Cancel and re-send in that case.

### RViz goal poses

The navigator subscribes to `goal_pose` (`geometry_msgs/PoseStamped`) and
forwards anything it receives to **its own action server** via an internal
action client. That is how RViz's "2D Goal Pose" tool drives the stack without
knowing about the action — and it means a stray publish on that topic starts a
real navigation.

## The behavior tree

`behavior_trees/move.xml` — replanning at 0.333 Hz with contextual recovery:

```xml
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="0.333">
    <RecoveryNode number_of_retries="1" name="ComputePathToPose">
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </RateController>
  <RecoveryNode number_of_retries="1" name="FollowPath">
    <FollowPath path="{path}" controller_id="FollowPath"/>
    <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
  </RecoveryNode>
</PipelineSequence>
```

`PipelineSequence` is what makes this work: it re-ticks the planner branch every
round even while `FollowPath` is still `RUNNING`, so the path is refreshed
underneath the controller. A failure in either branch clears **that branch's own
costmap** and retries once — stale obstacles being the most common cause.

Adapted from nav2's `navigate_to_pose_w_replanning_and_recovery.xml`, minus the
outer system-level recovery branch (Spin / Wait / BackUp via `RoundRobin`,
abort on `GoalUpdated`). Those BT nodes and the behavior server are not ported;
see `syncai_nav_core`'s missing `Behavior` interface.

Per-goal tree override: set the action goal's `behavior_tree` field to an
absolute XML path, or change the `default_bt_xml` parameter.

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `global_frame` | `map` | Stays unprefixed |
| `base_frame` | `base_link` | Launch file overrides with `<robot_id>/base_link` |
| `odom_topic` | `odom` | Feeds the `OdomSmoother` (0.3 s window) |
| `transform_tolerance` | `0.1` | |
| `plugin_lib_names` | six BT node libraries | Must list every library whose tags `move.xml` uses |
| `default_bt_xml` | `<share>/behavior_trees/move.xml` | Declared lazily by the navigator |
| `goal_blackboard_id` / `path_blackboard_id` | `goal` / `path` | Must match the `{…}` names in the XML |

Plus the `BtActionServer` parameters, declared by that class and documented in
`syncai_behavior_tree`'s README: `bt_loop_duration` (50 ms here),
`default_server_timeout` (20 ms), `wait_for_service_timeout` (1000 ms),
`always_reload_bt_xml` (false).

## Interfaces

| Kind | Name | Type |
|---|---|---|
| Action server | `navigate_to_pose` | `nav2_msgs/NavigateToPose` |
| Subscriber | `goal_pose` | `geometry_msgs/PoseStamped` (RViz) |
| Action client | `compute_path_to_pose` | via the BT node |
| Action client | `follow_path` | via the BT node |
| Service client | `global_costmap/clear_entirely_global_costmap` | via the BT node |
| Service client | `local_costmap/clear_entirely_local_costmap` | via the BT node |

The action name is `Navigator::getName()`, i.e. `navigate_to_pose`; it is also
the name given to the internal client node created by `BtActionServer`.

## Running

```bash
ros2 launch syncai_task_runner task_runner.launch.py
ros2 launch syncai_task_runner task_runner.launch.py \
    system_config:=config/instances/robot02.ini
```

Both byobu sessions start it after `sleep 10` — the longest delay in the stack,
because the BT is constructed at startup and **every action and service it
references must already be up** (see gotchas).

```bash
ros2 action send_goal /<robot_id>/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0},
    orientation: {w: 1.0}}}}" --feedback

ros2 topic pub --once /<robot_id>/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
```

## Gotchas

- **Startup order is enforced by construction, not by retry.** `BtActionServer`
  builds the tree in its constructor, and each `BtActionNode` / `BtServiceNode`
  constructor throws if its server is not up within `wait_for_service_timeout`
  (1 s). So the planner, the controller and both costmap clear services must all
  be running first — otherwise `initialize()` returns false and `main.cpp` exits
  with `RCLCPP_FATAL`. The `sleep 10` in the byobu scripts is this constraint.
- **A BT tag with no matching library in `plugin_lib_names` fails at tree load**
  with "unknown node type", which reads like an XML error but is a params error.
  The list is duplicated in both `syncai_task_runner.cpp` (as the default) and
  the params YAML — keep them in sync.
- **`always_reload_bt_xml: false` caches the tree** after the first goal. Editing
  `move.xml` without flipping this, or restarting, changes nothing.
- **Publishing to `goal_pose` starts a real navigation.** It is not a preview or
  a visualization topic.
- **Preemption with a different BT is rejected, not queued.** The current goal
  keeps running and the pending one is terminated.
- **The `bt_loop_duration` comment in the params file is stale**: it says
  "10 ms => the whole tree is ticked at 100 Hz" but the value is `50`, i.e. 20 Hz.
  That value also halves into every BT node's per-tick spin budget.
- **`src/.gitkeep` and `include/syncai_task_runner/.gitkeep` are leftovers** from
  when those directories were empty.

Upstream reference: [`nav2_bt_navigator`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bt_navigator).
