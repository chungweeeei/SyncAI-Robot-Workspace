# syncai_task_runner

The **BT navigator**: the node that serves `nav2_msgs/NavigateToPose` and
`syncai_common/action/NavigateToGoal` and, for each goal, ticks a behavior tree
that drives the planner and controller. Port of `nav2_bt_navigator`.

The two actions are one navigation. `NavigateToGoal` (2026-10) is nav2's action
with a **reason** in the result (`error_code` / `error_msg`: did planning fail
or path following?) and `number_of_replans` in the feedback, because the
backend's MOVE step got `ABORTED` with an empty `std_msgs/Empty` result and
could not tell the operator why. The backend still sends the nav2 one; it
migrates at its own pace, and the nav2 action (and the rviz `goal_pose`
subscriber with it) goes only after that.

This is the top of the navigation stack — everything below it (planner,
controller, costmaps, BT nodes) exists to serve the tree ticked here.

> The repo `README.md` still calls this package `syncai_bt_navigator`. That
> package does not exist; this is it.

```
syncai_backend (RobotWorkflow MOVE step) ──NavigateToPose──►  task_runner
                      (NavigateToGoal once it migrates) ──►       │
RViz "2D Goal Pose" ──/goal_pose topic────────────────────►       │
                                                                  │  PoseNavigator<NavigateToPose> / <NavigateToGoal>
                                                                  ▼  (one NavigatorMutex: one navigation at a time)
                                                    BtActionServer<ActionT>, one per action
                                                                  │  each ticks its own copy of behavior_trees/move.xml
                                              ┌───────────────────┴───────────────────┐
                                              ▼                                       ▼
                                  ComputePathToPose ──► syncai_planner    FollowPath ──► syncai_controller
                                  ClearEntireCostmap ─► syncai_costmap_2d (both costmaps)
```

## Structure

The package is split into a **host node** and **navigators**:

| Piece | Role |
|---|---|
| `TaskRunner` (`syncai_task_runner.cpp`) | The `rclcpp::Node`. Owns the shared TF buffer, the odom smoother, the navigator mutex, and the parameters. Hosts navigators; contains no navigation logic. |
| `Navigator<ActionT>` (`navigator.hpp`) | Header-only template base. Wraps a `syncai_behavior_tree::BtActionServer<ActionT>` and implements the goal-muxing that keeps two navigators from driving the robot at once. |
| `PoseNavigator<ActionT>` (`navigators/pose_navigator.{hpp,cpp}`) | The concrete navigator: loads `move.xml`, computes feedback, fills the result. One template, two explicit instantiations — `NavigateToPoseNavigator` (`nav2_msgs`) and `NavigateToGoalNavigator` (`syncai_common`). The three places they differ (action name, whether the result has `error_code`, which one owns the rviz `goal_pose` subscriber) sit in a `PoseNavigatorTraits<ActionT>` specialisation behind `if constexpr`. The two actions share field names on purpose; that is what lets one body compile for both. |
| `behavior_trees/move.xml` | The default tree |

`Navigator` is templated on the action type because each navigator binds a
different one. With two of them the `NavigatorMutex` is load-bearing: the two
action servers each run their goal on their own `std::async` worker, so the
claim is a single check-and-set (`tryStartNavigating`), not a check followed by
a claim. The loser is `ABORTED` with an empty result — `SimpleActionServer`
accepts every goal and terminates it in the execute callback — and never
reaches `goalCompleted()`.

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
claim the navigator mutex       → another navigator holds it? reject (ABORTED, empty result)
subclass goalReceived(goal):
    loadBehaviorTree(goal.behavior_tree)   ← empty string = the default tree
    failed to load?             → release the mutex, reject (ABORTED, empty result)
    initializeGoalPose(goal):
        reset number_recoveries, failed_node, failure_msg, number_plans; stamp start_time_
        write goal.pose to the blackboard under goal_blackboard_id ("goal"); clear "path"
  ⋯ BtActionServer runs the engine loop, calling onLoop() each tick ⋯
onCompletion: release the mutex, then goalCompleted(result, status) fills the result
```

The tree reads the goal from the blackboard, not from the action goal — that
indirection is what lets `move.xml` reference `goal="{goal}"` without knowing
anything about the action type.

### Result (`NavigateToGoal` only)

`goalCompleted()` runs before the action server terminates the goal, so what it
writes is what the client receives. It fills the result only for a tree that
ran and **failed**; `SUCCEEDED` and `CANCELED` keep the default `NONE`.

| `error_code` | When | `error_msg` |
|---|---|---|
| `NONE` = 0 | success, cancel — **and** a goal that was preempted by a newer one or rejected before the tree ran (another navigator busy, BT file missing). Those two never reach `goalCompleted()`: the action server aborts them with an empty result | empty |
| `PLAN_FAILED` = 200 | the planner branch failed the tree: `ComputePathToPose` aborted twice (the second time after `ClearGlobalCostmap`) — no route, start or goal in an obstacle / keepout zone, TF | `Planning failed: no path to the goal (planner compute_path_to_pose aborted the goal)` |
| `FOLLOW_PATH_FAILED` = 100 | the controller branch failed the tree: `FollowPath` aborted on its last retry — blocked with no detour, no progress, robot pose lost | `Path following failed: the robot could not reach the goal along its path (controller follow_path aborted the goal)` |
| `UNKNOWN` = 1 | the tree failed with nothing attributable: a server never acknowledged a goal (the message names it), an exception inside the tree | the node's message, or a pointer to the task_runner log |

How it knows: the BT action nodes write `failed_node` / `failure_msg` to the
blackboard when their goal is aborted (`BtActionNode::report_failure()`, the
same channel as `number_recoveries`), the navigator resets both per goal and
reads them at the end, and a table in `pose_navigator.cpp` maps the node's
registration name to the code — the number belongs to the action, so the
mapping lives next to its server, not in the BT package. Last writer wins, and
`move.xml`'s header shows why that names the branch that actually failed the
tree. The ranges are nav2 Iron's (1xx controller, 2xx planner) so finer codes
slot in as `base + n` once Humble's `nav2_msgs` results are replaced by ones
that carry a reason; the two bases are never renumbered. The same reason is
logged at ERROR for **both** actions, so `navigate_to_pose` callers get it in
the task_runner log before they migrate.

ABORTED + `NONE` is therefore "superseded or not started", not a navigation
failure, and the backend must not read it as one.

### Feedback

`onLoop()` fires once per BT tick and publishes feedback (nav2's five fields on
both actions, the sixth on `NavigateToGoal` only):

| Field | Computed from |
|---|---|
| `current_pose` | TF `global_frame → base_frame` |
| `distance_remaining` | Path length from the closest pose on the blackboard's `path` to its end |
| `estimated_time_remaining` | `distance_remaining / speed`, from the odom smoother; zero below 0.01 m/s or under 0.1 m remaining |
| `number_of_recoveries` | Blackboard `number_recoveries`, incremented by the `ClearEntireCostmap` BT nodes |
| `navigation_time` | Now minus `start_time_` |
| `number_of_replans` (`NavigateToGoal`) | Blackboard `number_plans` (incremented by `ComputePathToPose` on success) minus one, floored at 0 — the detours, since the tree plans only when the path is blocked or the goal changed |

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

The superseded goal is `ABORTED` with an **empty** result
(`SimpleActionServer::accept_pending_goal`), so on `NavigateToGoal` it carries
`error_code` `NONE` — see Result above. Preemption works within one action;
a goal on the *other* action while one runs is a mutex rejection, not a preempt.

### RViz goal poses

The `navigate_to_pose` navigator subscribes to `goal_pose`
(`geometry_msgs/PoseStamped`) and forwards anything it receives to **its own
action server** via an internal action client. That is how RViz's "2D Goal
Pose" tool drives the stack without knowing about the action — and it means a
stray publish on that topic starts a real navigation. It stays on the nav2
action on purpose: an rviz click during a running `navigate_to_pose` goal
preempts it in place today, and on the other navigator it would be a mutex
rejection instead. When the nav2 action is retired the subscriber moves with
it, by flipping two flags in `PoseNavigatorTraits`.

## The behavior tree

`behavior_trees/move.xml` — replanning only when the current path becomes
invalid (or the goal changes), with contextual recovery:

```xml
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="1.0">
    <RecoveryNode number_of_retries="1" name="ComputePathToPose">
      <Fallback name="FallbackComputePathToPose">
        <ReactiveSequence name="CheckIfNewPathNeeded">
          <Inverter>
            <GlobalUpdatedGoal/>
          </Inverter>
          <IsPathValid path="{path}" server_timeout="100"/>
        </ReactiveSequence>
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </Fallback>
      <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </RateController>
  <RecoveryNode number_of_retries="3" retry_refill_time="60.0" name="FollowPath">
    <FollowPath path="{path}" controller_id="FollowPath"/>
    <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
  </RecoveryNode>
</PipelineSequence>
```

`PipelineSequence` is what makes this work: it re-ticks the planner branch every
round even while `FollowPath` is still `RUNNING`, so the path can be replaced
underneath the controller. Once a second that branch asks the planner's
`is_path_valid` whether the part of `{path}` still ahead is free — free
meaning no centre cell at or above INSCRIBED **and** no LETHAL cell under the
padded footprint's perimeter at the path heading ± 0.35 rad, which is the
test RPP applies before it drives, run on the global costmap while the
blocker is still 2.5 m out (2026-10; the centre test alone let a blocker
0.25–0.45 m beside the route through, RPP refused it, and the goal aborted
with no detour). Only a blocked path, a changed goal (`GlobalUpdatedGoal`,
i.e. a preempt) or no path at all runs `ComputePathToPose`; a service timeout
or a missing robot pose keeps the path. A failure in either branch clears
**that branch's own costmap** and retries — stale obstacles being the most
common cause.

`FollowPath` gets **three** retries (2026-10; one before), and the count is a
wait, not a robustness knob: this stack has no local avoidance (RPP is a
tracker) and no `Wait` recovery, so once RPP refuses to drive, how long the
robot stands waiting for a blocker to leave is `failure_tolerance` (3 s) ×
(retries + 1), with a local-costmap clear between attempts (re-marked within
0.2 s; RPP zeroes its acceleration baseline on every refusal, so there is no
lurch). One retry (~6 s) covered a person walking across and not one standing
and talking; three (~12 s) covers that, at the price of a task failure taking
~12 s rather than ~6 s to declare. The progress checker's 30 s remains the
outer bound. The planner branch keeps one retry.

The retries are per incident, not per goal. A `RecoveryNode` resets its retry
count only when it returns, and `FollowPath` is `RUNNING` for the whole goal,
so until 2026-10 the first blocker used the budget up and a second one, minutes
later, failed the goal with no retry at all. `retry_refill_time="60.0"` gives
the budget back to an attempt that ran at least 60 s before failing. It must
stay above the progress checker's `movement_time_allowance` (30 s): an attempt
that never gets going fails within that (3 s of `failure_tolerance`, or 30 s
without progress), so a dead end still exhausts its retries. The planner
branch needs no refill — `ComputePathToPose` returns every tick, which resets
its `RecoveryNode`.

Until 2026-10 the branch replanned unconditionally at 1 Hz, and a fresh plan
always replaced the path. The global costmap raytraces the 3D cloud in 2D, so
rays to points above a low blocker erase it once the robot can see past it, and
half-way round a detour the original route read shortest again: the robot
turned back into the blocker. The cost of the fix is that a blocker which leaves
no longer shortens the route; the detour is finished. The navigator clears
`{path}` at every new goal, because nothing else does and a still-free path from
the previous goal would otherwise be kept.

Adapted from nav2's `navigate_w_replanning_only_if_path_becomes_invalid.xml`, minus the
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
| `plugin_lib_names` | eight BT node libraries | Must list every library whose tags `move.xml` uses. `syncai_behavior_tree` builds a ninth, `syncai_initial_pose_received_condition_bt_node`, that is deliberately not listed — `move.xml` has no `InitialPoseReceived` tag, and loading a library whose tag nothing uses only costs startup time |
| `default_bt_xml` | `<share>/behavior_trees/move.xml` | Declared lazily by the navigator |
| `goal_blackboard_id` / `path_blackboard_id` | `goal` / `path` | Must match the `{…}` names in the XML |

Plus the `BtActionServer` parameters, declared by that class and documented in
`syncai_behavior_tree`'s README: `bt_loop_duration` (50 ms here),
`default_server_timeout` (20 ms), `wait_for_service_timeout` (1000 ms),
`always_reload_bt_xml` (false).

## Interfaces

| Kind | Name | Type |
|---|---|---|
| Action server | `navigate_to_pose` | `nav2_msgs/NavigateToPose` — what the backend sends today; result empty |
| Action server | `navigate_to_goal` | `syncai_common/NavigateToGoal` — the same navigation, result `error_code` / `error_msg`, feedback `+ number_of_replans` (2026-10) |
| Subscriber | `goal_pose` | `geometry_msgs/PoseStamped` (RViz) → `navigate_to_pose` |
| Action client | `compute_path_to_pose` | via the BT node |
| Action client | `follow_path` | via the BT node |
| Service client | `global_costmap/clear_entirely_global_costmap` | via the BT node |
| Service client | `local_costmap/clear_entirely_local_costmap` | via the BT node |

Each action name is its navigator's `getName()` (`PoseNavigatorTraits::kActionName`);
it is also the suffix of the internal client node `BtActionServer` creates
(`task_runner_navigate_to_pose_rclcpp_node`, `task_runner_navigate_to_goal_rclcpp_node`).
Both servers are built from the same `plugin_lib_names`, so a tag missing from
that list fails both trees the same way.

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

# Same goal, with a reason in the result when it fails (and number_of_replans
# in the feedback):
ros2 action send_goal /<robot_id>/navigate_to_goal syncai_common/action/NavigateToGoal \
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
- **One navigation at a time across both actions.** A `navigate_to_goal` goal
  sent while a `navigate_to_pose` one runs (or the reverse) is `ABORTED` with
  an empty result and a "rejecting request" log line; it is not queued and it
  does not preempt. Preempt on the action the running goal came in on.
- **A retried abort can leave a stale attribution.** `failed_node` is written
  on abort and never cleared on success, so if the tree later fails through a
  path that writes nothing (an exception), the result names the earlier,
  recovered failure. Accepted: clearing on every success would need every node
  to know the keys, for a code that is coarse anyway.
- **`src/.gitkeep` and `include/syncai_task_runner/.gitkeep` are leftovers** from
  when those directories were empty.

Upstream reference: [`nav2_bt_navigator`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bt_navigator).
