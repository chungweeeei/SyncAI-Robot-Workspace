# syncai_task_runner

The **BT navigator**: the node that serves `nav2_msgs/NavigateToPose` and
`syncai_common/action/NavigateToGoal` and, for each goal, ticks a behavior tree
that drives the planner and controller. Port of `nav2_bt_navigator`.

The two actions are one navigation. `NavigateToGoal` (2026-10) is nav2's action
with a **reason** in the result (`error_code` / `error_msg`: did planning fail
or path following?) and `number_of_replans` / `status_msg` in the feedback,
because the backend's MOVE step got `ABORTED` with an empty `std_msgs/Empty`
result and could not tell the operator why — nor, while it ran, what the robot
was doing. The backend still sends the nav2 one; it
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
                                  ComputePathToPose ──► syncai_global_planner    FollowPath ──► syncai_controller
                                  ClearEntireCostmap ─► syncai_costmap (both costmaps)
```

## Structure

The package is split into a **host node** and **navigators**:

| Piece | Role |
|---|---|
| `TaskRunner` (`syncai_task_runner.cpp`) | The `rclcpp::Node`. Owns the shared TF buffer, the odom smoother, the navigator mutex, and the parameters. Hosts navigators; contains no navigation logic. |
| `Navigator<ActionT>` (`navigator.hpp`) | Header-only template base. Wraps a `syncai_behavior_tree::BtActionServer<ActionT>` and implements the goal-muxing that keeps two navigators from driving the robot at once. |
| `PoseNavigator<ActionT>` (`navigators/pose_navigator.{hpp,cpp}`) | The concrete navigator: loads `move.xml` (or `patrol.xml`), computes feedback, fills the result. One template, four explicit instantiations — `NavigateToPoseNavigator` (`nav2_msgs`), `NavigateToGoalNavigator` (`syncai_common`) and, since 2026-11, the patrol route's `NavigateThroughPosesNavigator` (`nav2_msgs`) and `NavigateThroughGoalsNavigator` (`syncai_common`). The places they differ (action name, whether the result has `error_code`, which one owns the rviz `goal_pose` subscriber, one `pose` or a list of `poses`, the default tree) sit in a `PoseNavigatorTraits<ActionT>` specialisation behind `if constexpr`. The actions share field names on purpose; that is what lets one body compile for all four. |
| `behavior_trees/move.xml` | The default tree: `Octo3D` + `FollowPath` (MPPI) — see "Which tree a goal runs" |
| `behavior_trees/patrol.xml` | `navigate_through_poses`' default tree (2026-11): `move.xml` planning through the waypoints still ahead — see "The patrol route" |

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
        reset number_recoveries, failed_node, failure_msg, number_plans;
        set nav_status_msg to "Planning a path to the goal"; stamp start_time_
        write goal.pose to the blackboard under goal_blackboard_id ("goal"); clear "path"
  ⋯ BtActionServer runs the engine loop, calling onLoop() each tick ⋯
onCompletion: release the mutex, then goalCompleted(result, status) fills the result
               (SUCCEEDED / CANCELED: also calls the planner's clear_plan, fire and forget)
```

The tree reads the goal from the blackboard, not from the action goal — that
indirection is what lets `move.xml` reference `goal="{goal}"` without knowing
anything about the action type.

### Result (`NavigateToGoal` and `NavigateThroughGoals`)

`goalCompleted()` runs before the action server terminates the goal, so what it
writes is what the client receives. It fills the result only for a tree that
ran and **failed**; `SUCCEEDED` and `CANCELED` keep the default `NONE`.

| `error_code` | When | `error_msg` |
|---|---|---|
| `NONE` = 0 | success, cancel — **and** a goal that was preempted by a newer one or rejected before the tree ran (another navigator busy, BT file missing). Those two never reach `goalCompleted()`: the action server aborts them with an empty result | empty |
| `PLAN_FAILED` = 200 | the planner branch failed the tree — `ComputePathToPose` (or, for a route, `ComputePathThroughPoses`) aborted twice, the second time after the `clear_overlay` recovery — and the planner gave no finer reason | `Planning failed: no path to the goal (planner compute_path_to_pose aborted the goal)` |
| 201–224 (2026-11) | the same, with the planner's own reason: its result's `error_code`, copied unchanged — `GOAL_IN_KEEPOUT` 223, `GOAL_OCCUPIED` 206, `NO_VALID_PATH` 208, `MAP_NOT_LOADED` 220, … (`syncai_common`'s `ComputePathToPose.action` has the table, with whether a retry can help) | a sentence for the code, then the planner's: `Planning failed: the goal is inside a keepout zone (planner compute_path_to_pose aborted the goal: goal_in_keepout: the goal lies inside keepout zone 'z1'; … [start (…) -> goal (…)])` |
| `FOLLOW_PATH_FAILED` = 100 | the controller branch failed the tree: `FollowPath` aborted on its last retry — blocked with no detour, no progress, robot pose lost | `Path following failed: the robot could not reach the goal along its path (controller follow_path aborted the goal)` |
| `DOCK_FAILED` = 300 | no node maps to it. The code stays in the interface (the backend switches on the hundreds, and a code is never renumbered or reused) | — |
| `UNKNOWN` = 1 | the tree failed with nothing attributable: a server never acknowledged a goal (the message names it), an exception inside the tree | the node's message, or a pointer to the task_runner log |

How it knows: the BT action nodes write `failed_node` / `failure_msg` to the
blackboard when their goal is aborted (`BtActionNode::report_failure()`, the
same channel as `number_recoveries`), the navigator resets both per goal and
reads them at the end, and a table in `pose_navigator.cpp` maps the node's
registration name to the code — the number belongs to the action, so the
mapping lives next to its server, not in the BT package. Last writer wins, and
`move.xml`'s header shows why that names the branch that actually failed the
tree. The ranges are nav2 Iron's (1xx controller, 2xx planner) so finer codes
slot in as `base + n`; the bases are never renumbered. The planner's are the
first (2026-11): the planner actions are `syncai_common`'s twins with
`error_code` in the result, `report_failure()` writes it to `failure_code`
(0 for a node with none, so it always belongs to the same failure as
`failed_node`), and for the two planner nodes a code above 200 and below 300
replaces their row's 200, under a sentence of its own
(`kPlanningSentenceByCode`; a code missing there keeps the node's sentence but
is still copied). The controller's 1xx waits for a `FollowPath` result with a
reason. The same reason is
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
| `number_of_recoveries` | Blackboard `number_recoveries`, incremented by the `ClearEntireCostmap` BT nodes — including the new-goal clear at tree start, so on `move.xml` and `patrol.xml` (since 2026-11) it reads at least 1 and recoveries proper are the value minus one. Nothing in this stack, the backend or the console reads it: it is filled because nav2's feedback has the field |
| `navigation_time` | Now minus `start_time_` |
| `number_of_replans` (`NavigateToGoal`) | Blackboard `number_plans` (incremented by `ComputePathToPose` on success) minus one, floored at 0 — the detours, since the tree plans only when the path is blocked or the goal changed |
| `status_msg` (`NavigateToGoal`) | Blackboard `nav_status_msg`: one sentence the BT nodes write at each turn of `move.xml` — "Planning a path to the goal" (set per goal), "Following the path", "Path blocked; replanning", "Goal changed; replanning", and a retry sentence when the planner or the controller gives up. For the operator console, which otherwise could only say "running"; free text, never to be switched on. Not visible: the controller holding at a blocker inside `failure_tolerance` (Humble's `FollowPath` feedback cannot tell that from a stop), which reads "Following the path" until the abort |

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

Two guards keep the superseded goal's outcome from landing on its successor
(2026-11):

- **A preempt is accepted only while the tree still runs.** One pending at the
  tick that finished the tree stays pending: the finished goal gets its own
  result (SUCCEEDED, or its failure), and the new goal then runs from the start
  like any other — including a different tree, which mid-run would have been
  rejected. Before, it was accepted after that tick, so the new goal was
  reported SUCCEEDED (or FAILED) for the old goal's outcome and the old goal,
  which had arrived, came back ABORTED.
- **The old path's success is not the new goal's.** A preempt empties `{path}`
  until the new goal's plan lands, and `FollowPath` keeps driving the old path
  meanwhile. The navigator bumps `goal_generation` on the blackboard at every
  goal and preempt; `FollowPath` takes the controller's success only for a path
  sent under the current one, and otherwise stands at the old goal until the new
  path lands and sends it. Before, reaching the old goal in that window
  succeeded the tree — the new goal reported reached, at the old one's place.
- **A plan in flight is re-targeted.** A preempt that lands while the planner
  is still working on the old goal (its first plan, a blocked-path replan, or
  the plan of an earlier preempt) is re-sent by `ComputePathToPose` /
  `ComputePathThroughPoses` (`on_wait_for_result`), and the old goal's answer
  is dropped by its goal id; the planner starts the new goal as soon as the
  plan in hand is done. Before, that answer landed as the new goal's first path
  -- the robot turned onto a route to the old goal until the next 1 Hz check
  replanned, and could reach its end and succeed the new goal (2026-11).
  Accepted: `GlobalUpdatedGoal` was not ticked meanwhile, so it reports the
  change at the next 1 Hz check and the new goal is planned once more
  (`number_of_replans` one higher), as in nav2. A patrol re-sends too when the
  robot passes a waypoint mid-plan, since `remaining_goals` is the input.

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

### Which tree a goal runs

The goal's `behavior_tree`, resolved: a bare name against `behavior_trees/`,
an absolute path as is, empty = `default_bt_xml`, i.e. `move.xml`. Each goal
logs the file it runs. A preempt is compared against the running tree after
the same resolution, so a preempt naming a different `behavior_tree` is
refused.

From 2026-10-19 to 2026-10-28 a map was 2D (`gridmap.yaml`) or 3D
(`octomap.bt`), and on a 3D map — the planner's latched
`planner_server/Octo3D/map_available`, the `map_3d_topic` parameter — a
`<stem>_3d.xml` beside the resolved file replaced it (`move_3d.xml`). 2D navigation was removed on 2026-10-28: the 3D trees took
the plain names, the 2D ones (`GridBased` on the gridmap, exactly `dev`'s)
and the per-goal swap went, and this node no longer subscribes to
`map_available`.

`behavior_trees/move.xml` navigates on the map's `octomap.bt`, replanning
only when the current path becomes invalid or the goal changes, with
contextual recovery:

```xml
<Sequence name="NavigateFromFreshGlobalCostmap">
  <ForceSuccess>
    <ClearEntireCostmap name="ClearDynamicOverlay-NewGoal" service_name="planner_server/Octo3D/clear_overlay" server_timeout="1000"/>
  </ForceSuccess>
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="1" name="ComputePathToPose">
        <Fallback name="FallbackComputePathToPose">
          <ReactiveSequence name="CheckIfNewPathNeeded">
            <Inverter>
              <GlobalUpdatedGoal/>
            </Inverter>
            <IsPathValid path="{path}" server_timeout="100" blocked_time="10.0"/>
          </ReactiveSequence>
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="Octo3D"/>
        </Fallback>
        <ForceSuccess>
          <ClearEntireCostmap name="ClearDynamicOverlay-Context" service_name="planner_server/Octo3D/clear_overlay"/>
        </ForceSuccess>
      </RecoveryNode>
    </RateController>
    <RecoveryNode number_of_retries="4" retry_refill_time="60.0" name="FollowPath">
      <FollowPath path="{path}" controller_id="FollowPath"/>
      <ForceSuccess>
        <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
      </ForceSuccess>
    </RecoveryNode>
  </PipelineSequence>
</Sequence>
```

Every recovery clear is under `ForceSuccess` (2026-11), like the new-goal
one: `RecoveryNode` fails outright when its recovery child fails, so a clear
that timed out or found no service ended the goal with its retries unused.
The same in `patrol.xml`.

**Octo3D, judged on its own overlay.** `Octo3D` (OctomapPlanner3D on the
map's `octomap.bt`) routes over the floor surfaces and between floors over a
stair link; `FollowPath` is MPPI, 3D-aware since 2026-10-20 (it matches a
path with heights in 3D and climbs stairs in `syncai_util::StairMode`;
`FollowPathRPP3D` is the A/B). planner_server answers `is_path_valid` by
asking the planner that made the last plan (`GlobalPlanner::isPathValid()`,
2026-10-21), and `Octo3D` judges on its **dynamic overlay** — the lidar cloud
in the map frame, every return 0.1–0.6 m above the surface it lies on a mark
on that cell, rays clearing the marks they cross, marks forgotten after 30 s
(`syncai_global_planner/README.md`, "Dynamic obstacles"). From 2026-10-19 to
2026-10-21 the check was out of the 3D tree, with a `number_plans == 0`
plan-once check in its place, because it then ran on the 2D costmap, which
upstairs read a good path as blocked; nothing replanned on a 3D map then. A
replan detours: the search keeps off the marks and 0.35 m round them and
pays for the band beyond as it does for a wall, so `number_of_replans` is
meaningful. The check judges only the floor stretch the robot is on and
answers "valid" while the robot is on a stair, so a person at the stair head
is judged once the robot is on that floor and nothing replans mid-flight.

**Every new goal starts from a cleared obstacle memory** (2026-10-14). Both
`ClearEntireCostmap` nodes in the planner's half call
`planner_server/Octo3D/clear_overlay` (the costmap clear's own type, so the
node is the stock one), which drops the overlay's marks. A mark the robot
walks away from is not raytraced away, and though it ages out after 30 s a
goal sent sooner must not plan round a person who has left: on the 2D stack,
where the clear was the global costmap's, the return leg of an A ↔ B shuttle
on robot01 came out 60 m instead of 17.5 m, round 509 LETHAL cells the
previous leg had left in the corridor. It runs at tree start only — a preempt
keeps the tree running and keeps the memory, which is what makes a detour
stick — and under `ForceSuccess` with a 1 s timeout, so a clear that does not
answer in time leaves the goal to plan on the old memory rather than fail it.
The first plan after a clear waits for the next scan (`Octo3D`, 2026-11; at
most `dynamic.transform_tolerance` + 0.2 s), so it sees what is in view now.
Until then it planned on the empty overlay — through a person in view, and,
worse, the planner recovery's retry of a `goal_occupied` plan, sent ~1.5 ms
after its clear, planned straight through the obstacle on the goal, so a goal
someone stood on looped (blocked 10 s, replan, 206, clear, through) and never
failed `GOAL_OCCUPIED` (robot01, 2026-11-06). Now the retry fails 206 again
and the planner branch fails the goal with it. Because `ClearEntireCostmap` counts itself as a
recovery, `number_of_recoveries` reads at least 1 on every goal of the trees
that have it (`move.xml`, `patrol.xml`).

`PipelineSequence` is what makes this work: it re-ticks the planner branch every
round even while `FollowPath` is still `RUNNING`, so the path can be replaced
underneath the controller. Once a second that branch asks the planner's
`is_path_valid` whether the part of `{path}` still ahead is free (above).
Only a blocked path, a changed goal (`GlobalUpdatedGoal`,
i.e. a preempt) or no path at all runs `ComputePathToPose`; a service timeout
or a missing robot pose keeps the path. A failure in either branch clears
**that branch's own obstacle memory** (the overlay, the local costmap) and
retries — stale obstacles being the most common cause.

`FollowPath` gets **four** retries (2026-10-14; one until 2026-10, three in
between), and the count is a wait, not a robustness knob. The controller behind `FollowPath` is MPPI since
2026-10 (`syncai_mppi_controller`), which bends round a blocker beside the
path inside the local costmap; a blocker that closes the way leaves it with no
feasible trajectory, and there is no `Wait` recovery, so from then on how long
the robot stands waiting for it to leave is `failure_tolerance` (3 s) ×
(retries + 1), with a local-costmap clear between attempts (re-marked within
a scan; both controllers zero their acceleration baseline on every refusal,
so there is no lurch). RPP, `FollowPath` until then, refused at any blocker in
its projection; the 2D RPP went with 2D navigation (2026-10-28), and
`controller_id="FollowPathRPP3D"` in `move.xml` is the RPP A/B. One retry (~6 s) covered a person walking across and not one standing
and talking; three (~12 s) covered that, at the price of a task failure taking
~12 s rather than ~6 s to declare. Four (~16 s) since 2026-10-14, because the
wait has to outlast `IsPathValid`'s `blocked_time` (10 s): MPPI refuses a
person who steps in close at once, the 1 Hz check can take a little longer
to call the path blocked, and three retries would run out just before
the replan lands (estimated from `failure_tolerance`, not measured). Keep (retries + 1) × 3 s at least `blocked_time` +
4 s. The progress checker's 30 s remains the outer bound. The planner branch
keeps one retry.

**A blocked path is replanned only after 10 s** (`IsPathValid blocked_time`,
2026-10-14): every check for that long must find the same spot (within 0.5 m)
of the same path blocked; one clear answer ends the wait with no replan. A
single blocked answer used to be a replan, so a person crossing the route
bought a detour that then stuck for the goal. The task_runner log carries the
decision (`Path blocked at (x, y) for n of 10.0 s; keeping it`, `Path clear
again after n s blocked`, `... for n s; replanning`); the planner log has the
detail of each blocked answer.

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
always replaced the path. The 2D global costmap raytraced the 3D cloud in 2D,
so rays to points above a low blocker erased it once the robot could see past
it, and half-way round a detour the original route read shortest again: the
robot turned back into the blocker. The cost of the fix is that a blocker which leaves
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
| `plugin_lib_names` | nine BT node libraries | Must list every library whose tags `move.xml` and `patrol.xml` use — and lists every node library `syncai_behavior_tree` builds. A node (and its wait for its server) is constructed only when a tree using its tag is loaded |
| `default_nav_through_poses_bt_xml` | `<share>/behavior_trees/patrol.xml` | `navigate_through_poses`' default tree (2026-11), declared lazily by its navigator like `default_bt_xml` |
| `waypoint_passed_radius` | `1.0` m | `navigate_through_poses` only: a waypoint the robot comes within this of (x / y) is passed and dropped from `remaining_goals`, what a replan plans through; never the last one. Wide because a waypoint is passed through, not stopped on. A first value, unmeasured |
| `waypoint_passed_z_tolerance` | `1.0` m | 2026-11: and only when its z is within this of the robot's ground height, so a waypoint on another floor is never passed from below or above it. Storeys are 2.5 m and more apart |
| `body_frame` / `lidar_height` | `<robot_id>/pointlio_body` (launch) / `0.481` | The robot's ground height for the test above (`syncai_util::lookupGroundZ`), as controller_server and `Octo3D` read it; keep the three in step |
| `default_bt_xml` | `<share>/behavior_trees/move.xml` | Declared lazily by the navigator. A goal's `behavior_tree` may be a bare file name (`patrol.xml`), resolved against the same `behavior_trees/` directory, or an absolute path. (Until 2026-10-28 a 3D map swapped any of them for its `_3d.xml` sibling.) |
| `goal_blackboard_id` / `path_blackboard_id` | `goal` / `path` | Must match the `{…}` names in the XML |

Plus the `BtActionServer` parameters, declared by that class and documented in
`syncai_behavior_tree`'s README: `bt_loop_duration` (50 ms here),
`default_server_timeout` (100 ms; 20 until 2026-11), `wait_for_service_timeout` (1000 ms),
`always_reload_bt_xml` (false).

## The patrol route (`navigate_through_goals` / `navigate_through_poses`, 2026-11)

`syncai_common/NavigateThroughGoals` on `navigate_through_goals`, and
`nav2_msgs/NavigateThroughPoses` on `navigate_through_poses` beside it (the
same navigator, tree and rules; nav2's result is Empty, so it stays only until
nothing sends it): drive from the
robot through every pose of `poses`, in order, **without stopping or turning
at them**, and finish at the last one **like a MOVE** — xy within the goal
checker's 0.10 m and heading within its 0.2 rad. A waypoint's own heading is
ignored; only the last pose's is used. Each pose's z picks its floor
(`goal_z_mode: request`), so a route can cross floors over a stair.
`NavigateThroughGoals` is what the backend's PATROL step sends (2026-11); its
result is `NavigateToGoal`'s, mapped the same way (`ComputePathThroughPoses` →
200 or the planner's own 2xx — a waypoint in a keepout zone is
`GOAL_IN_KEEPOUT`, the detail naming the leg), and its feedback adds
`number_of_replans` / `status_msg`.

`patrol.xml` is `move.xml` with its planner branch planning through the
waypoints (`ComputePathThroughPoses`, `Octo3D`'s
`createPlanThroughPoses()`: leg by leg, each later leg from the cell the one
before ended on). Two blackboard keys carry the route
(`blackboard_keys.hpp`): `goals`, the client's list, rewritten only by a new
goal or a preempt — what `GlobalUpdatedGoal` watches — and `remaining_goals`,
what a replan plans through. The navigator drops a waypoint from the front of
`remaining_goals` once the robot has come within `waypoint_passed_radius`
(1.0 m) of it **on its floor** — the waypoint's z within
`waypoint_passed_z_tolerance` (1.0 m) of the robot's ground height (2026-11;
on x / y alone a robot on 1F dropped the 2F waypoint above it, and no ground
height means nothing is dropped that tick) — never the last, at its loop rate,
and reports what is left as
`number_of_poses_remaining`. So passing a waypoint is not a changed goal and
does not replan; a replan for a blocked path starts from the next waypoint.
A waypoint the robot misses by more than the radius (a wide detour) is
driven back to only if the path is replanned before the next one is passed.
nav2 does the dropping in the tree (`RemovePassedGoals`, ported here once and
removed unused in 2026-07); in the navigator it runs at 20 Hz instead of the
planner branch's 1 Hz and needs no TF buffer on the blackboard.

A route can pass back over itself — out and back along one corridor, or a
loop ending where it started. Every consumer of the path places the robot on
it by **forward-only progress** (`syncai_util/path_progress.hpp`): MPPI's and
RPP 3D's pruning, the controller server's goal gate and feedback, the
planner server's `plan` trim and `is_path_valid`, and this navigator's
`distance_remaining`. A whole-path nearest pose there could take the later
leg: a robot back at a loop's start read as at the start, and an
out-and-back's return leg lies on top of the outgoing one.

```bash
# navigate_through_goals reports why it failed (error_code / error_msg);
# navigate_through_poses nav2_msgs/action/NavigateThroughPoses takes the same goal.
ros2 action send_goal /<robot_id>/navigate_through_goals syncai_common/action/NavigateThroughGoals \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}},
            {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.0}, orientation: {w: 1.0}}},
            {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 1.0, w: 0.0}}}]}" \
  --feedback
```

## Interfaces

| Kind | Name | Type |
|---|---|---|
| Action server | `navigate_to_pose` | `nav2_msgs/NavigateToPose` — what the backend sends today; result empty |
| Action server | `navigate_to_goal` | `syncai_common/NavigateToGoal` — the same navigation, result `error_code` / `error_msg`, feedback `+ number_of_replans`, `status_msg` (2026-10) |
| Action server | `navigate_through_goals` | `syncai_common/NavigateThroughGoals` (2026-11) — the patrol route, as `navigate_through_poses` below, with `NavigateToGoal`'s result (`error_code` / `error_msg`) and feedback (`+ number_of_replans`, `status_msg`). The backend's PATROL step |
| Action server | `navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` (2026-11) — the patrol route through `poses`, `patrol.xml` by default; result empty, feedback `number_of_poses_remaining`. The backend's PATROL step |
| Action client | `compute_path_through_poses` | via the `ComputePathThroughPoses` BT node — `patrol.xml` only |
| Subscriber | `goal_pose` | `geometry_msgs/PoseStamped` (RViz) → `navigate_to_pose` |
| Action client | `compute_path_to_pose` | `syncai_common/ComputePathToPose` (2026-11), via the BT node |
| Action client | `follow_path` | via the BT node |
| Service client | `planner_server/Octo3D/clear_overlay` | `nav2_msgs/ClearEntireCostmap`, via the `ClearEntireCostmap` BT node — `move.xml`'s / `patrol.xml`'s two planner-side clears, dropping `Octo3D`'s dynamic-obstacle marks (2026-10-21; `global_costmap/clear_entirely_global_costmap` on the 2D trees until 2026-10-28) |
| Service client | `local_costmap/clear_entirely_local_costmap` | via the BT node |
| Service client | `clear_plan` | `std_srvs/Empty`, the planner's (2026-10-14). Called from `goalCompleted()` on **CANCELED** and, since 2026-10-22, **SUCCEEDED**, so the route the planner re-sends on `plan` once a second disappears from the console with the goal instead of staying up until the next one. Success was left out at first because the re-send is trimmed to the robot, but the trim starts at the nearest pose anywhere on the plan, so the stub grew back along the route once the robot moved off the goal. Not on failure (the route explains the failure); the planner drops that route itself once no navigation has used it for its `plan_idle_clear_time` (30 s, 2026-10-25). Fire and forget, on the node's default callback group; a planner that is not up gets a WARN and the route stays |

Each action name is its navigator's `getName()` (`PoseNavigatorTraits::kActionName`);
it is also the suffix of the internal client node `BtActionServer` creates
(`task_runner_navigate_to_pose_rclcpp_node`, `task_runner_navigate_to_goal_rclcpp_node`,
`task_runner_navigate_through_poses_rclcpp_node`, `task_runner_navigate_through_goals_rclcpp_node`). All four servers are built
from the same `plugin_lib_names`, so a tag missing from that list fails every
tree that uses it the same way. Each loads its default tree at startup, so
`patrol.xml`'s `compute_path_through_poses` client is waited for then, like
`move.xml`'s `compute_path_to_pose`.

## Running

```bash
ros2 launch syncai_task_runner task_runner.launch.py
ros2 launch syncai_task_runner task_runner.launch.py \
    system_config:=/path/to/other.ini
```

Both byobu sessions start it after `sleep 10` — the longest delay in the stack,
because the BT is constructed at startup and **every action and service it
references must already be up** (see gotchas).

```bash
ros2 action send_goal /<robot_id>/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0},
    orientation: {w: 1.0}}}}" --feedback

# Same goal, with a reason in the result when it fails (and number_of_replans
# and status_msg in the feedback):
ros2 action send_goal /<robot_id>/navigate_to_goal syncai_common/action/NavigateToGoal \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0},
    orientation: {w: 1.0}}}}" --feedback

ros2 topic pub --once /<robot_id>/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"

```

## Switching the gait (no tree does today)

The BT gait nodes (`SetMotionKey`, `Wait`, `WaitForMotionState`) were removed
with nothing left to use them; git history has them. What a tree that switches
the gait again has to know about the gait controller: `set_motion_key` is
one-way UDP, so its `success` means only that the datagram left; the
controller's own report on `mode` data[1] (MPC = 5, Locomotion = 1, read off
the robot on 2026-10-14) is the check, and the switch was seen after 49 ms on
robot01. A settling time after it (3 s was used) is a judgement, not a
measurement: nothing reports when the gait under MPC is steady. `MODE Z`
(stand, `key="0"`) is **not** a no-op for a robot already standing in MPC — on
robot01 it took the robot down and up again (2026-10-15). A cancel halts a tree
before any restore step runs, so a cancelled gait switch would leave MPC on.

## Gotchas

- **Startup order is enforced by construction, not by retry.** `BtActionServer`
  builds the tree in its constructor, and each `BtActionNode` / `BtServiceNode`
  constructor throws if its server is not up within `wait_for_service_timeout`
  (1 s). So the planner, the controller and both costmap clear services must all
  be running first — otherwise `initialize()` returns false and `main.cpp` exits
  with `RCLCPP_FATAL`. The `sleep 10` in the byobu scripts is this constraint.
  A tree that is not the default is the exception: its nodes are constructed
  at its **first goal**, and a server only it needs missing then fails *that goal* (`ABORTED`,
  `error_code` NONE, the task_runner log names the server), not the node.
  The 3D trees are built at the first goal on a 3D map the same way; their
  two global clears are `planner_server/Octo3D/clear_overlay`, served by the
  planner process that is already up for `is_path_valid`.
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
- **One navigation at a time across the three actions.** A `navigate_to_goal`
  goal sent while a `navigate_to_pose` or `navigate_through_poses` one runs (or
  any other pair) is `ABORTED` with an empty result and a "rejecting request"
  log line; it is not queued and it does not preempt. Preempt on the action
  the running goal came in on.
- **A retried abort can leave a stale attribution.** `failed_node` is written
  on abort and never cleared on success, so if the tree later fails through a
  path that writes nothing (an exception), the result names the earlier,
  recovered failure. Accepted: clearing on every success would need every node
  to know the keys, for a code that is coarse anyway.
- **`src/.gitkeep` and `include/syncai_task_runner/.gitkeep` are leftovers** from
  when those directories were empty.

Upstream reference: [`nav2_bt_navigator`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bt_navigator).
