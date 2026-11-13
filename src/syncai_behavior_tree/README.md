# syncai_behavior_tree

The behavior-tree layer of the navigation stack: a port of `nav2_behavior_tree`
onto BehaviorTree.CPP **v3.8**.

It ships three things, and it is worth keeping them apart because they are
consumed in completely different ways:

| | What | How it reaches a consumer |
|---|---|---|
| **Engine + base classes** | `BehaviorTreeEngine`, `BtActionServer`, `BtActionNode`, `BtServiceNode`, `bt_conversions` | `libsyncai_behavior_tree.so` + headers, linked at **build** time |
| **BT node plugins** | `ComputePathToPose`, `ComputePathThroughPoses`, `FollowPath`, `ClearEntireCostmap`, `PipelineSequence`, `RecoveryNode`, `RateController`, `IsPathValid`, `GlobalUpdatedGoal` | one shared library each, `dlopen`ed by **name at runtime** |

Nothing here is a ROS node. The only consumer today is **`syncai_task_runner`**,
which builds a `BtActionServer<nav2_msgs::action::NavigateToPose>` and ticks
`behavior_trees/move.xml`.

```
syncai_task_runner (rclcpp::Node)
  └─ Navigator<NavigateToPose>
       └─ BtActionServer<NavigateToPose>        ← this package
            ├─ syncai_util::SimpleActionServer   (serves /navigate_to_pose)
            ├─ BehaviorTreeEngine                (factory + fixed-rate tick loop)
            └─ BT::Tree from move.xml
                 ├─ PipelineSequence / RateController / RecoveryNode   ← plugins
                 ├─ ComputePathToPose  ──action──►  syncai_global_planner
                 ├─ FollowPath         ──action──►  syncai_controller
                 └─ ClearEntireCostmap ──service─►  syncai_costmap
```

## Layout

```
include/syncai_behavior_tree/
  behavior_tree_engine.hpp     factory wrapper + the run() tick loop
  bt_action_server.hpp         action server ⇄ tree glue (+ _impl.hpp, template)
  bt_action_node.hpp           base class for BT nodes that call a ROS action
  bt_service_node.hpp          base class for BT nodes that call a ROS service
  bt_conversions.hpp           BT::convertFromString<> for ROS msg types
  plugins/{action,control,condition,decorator}/…
src/behavior_tree_engine.cpp
plugins/…                      one .cpp per plugin library; each ends in BT_REGISTER_NODES
```

## BehaviorTreeEngine

Two responsibilities: own the `BT::BehaviorTreeFactory` (the string → C++ class
table that lets XML tags become node instances), and run the tick loop.

```cpp
BtStatus run(BT::Tree * tree,
             std::function<void()> onLoop,
             std::function<bool()> cancelRequested,
             std::chrono::milliseconds loopTimeout = 10ms);
```

Each iteration: check `cancelRequested()` (halt + return `CANCELED`), then
`tree->tickRoot()`, then `onLoop()`, then sleep out the rest of `loopTimeout`
(warning logged if the period was overrun). The loop exits when the root returns
`SUCCESS`/`FAILURE` → `BtStatus::SUCCEEDED`/`FAILED`. An exception thrown from
anywhere in the tree is caught here and becomes `FAILED`.

`loopTimeout` is the **whole loop period**, not a per-node budget, and it only
causes waiting when some node returns `RUNNING` — a tree of purely synchronous
nodes finishes inside one iteration.

`haltAllActions()` halts the root and then walks the tree halting any node still
`RUNNING`, so the same tree object can be re-run for the next goal.

## BtActionServer

The bridge between "a ROS action goal arrived" and "tick a tree until it
finishes". Constructed with a parent node, an action name, the plugin library
names, a default XML path, and four callbacks.

**What the constructor does** (in order — it does everything eagerly, there is no
lifecycle configure step in this stack):

1. Declares its parameters on the parent node.
2. Creates `client_node_`, a **separate, non-spinning** `rclcpp::Node` named
   `<parent>_<action_name>_rclcpp_node`, and copies all the parent's parameters
   onto it. Every BT node's action/service client is created on *this* node, not
   the parent — that is what keeps BT client callbacks off the parent's executor.
3. Creates the `SimpleActionServer`.
4. Builds the `BehaviorTreeEngine`, which `dlopen`s every plugin library.
5. Creates the blackboard and populates it.
6. Loads the default BT XML — **which constructs every BT node in the tree**.
7. Activates the action server (only now, so no goal can arrive before the tree
   exists).

Step 6 is the reason startup order matters: a `BtActionNode`/`BtServiceNode`
constructor **throws if its action/service server is not up** within
`wait_for_service_timeout`. Loading `move.xml` therefore requires the planner,
the controller, and both costmap clear services to already be running. This is
what the `sleep:` offsets in `config/sessions/*.yaml` are for.

**Per goal** (`executeCallback`):

```
on_goal_received_callback(goal)  →  false ⇒ terminate_current(), done
engine.run(tree, on_loop, is_canceling, bt_loop_duration)
        on_loop  = preempt check (→ on_preempt_callback) + on_loop_callback,
                   only after a tick that left the tree RUNNING
        is_cancel = server inactive OR cancel requested
haltAllActions()
on_completion_callback(result, status)
  SUCCEEDED → succeeded_current(result)
  FAILED    → terminate_current(result)
  CANCELED  → terminate_all(result)
```

`loadBehaviorTree()` can swap the tree at runtime. It is a no-op when the same
filename is already loaded, unless `always_reload_bt_xml` is set — handy while
editing a tree, since otherwise the XML is read once and cached.

### Parameters (declared on the parent node if absent)

| Parameter | Default | Meaning |
|---|---|---|
| `bt_loop_duration` | `10` ms | Tick period of the whole tree. Also sets every node's per-tick spin budget (half of it). |
| `default_server_timeout` | `20` ms (code); `100` in `syncai_task_runner`'s params since 2026-11 | How long a BT node waits for a goal ack / result / cancel before giving up |
| `wait_for_service_timeout` | `1000` ms | How long a BT node's **constructor** waits for its server to appear before throwing |
| `always_reload_bt_xml` | `false` | Re-read the XML from disk on every `loadBehaviorTree()` |
| `global_frame` | `map` | Declared here and copied to `client_node_` for BT nodes to read |
| `base_frame` | `base_link` | ditto |
| `transform_tolerance` | `0.1` | ditto |

### Blackboard contract

The blackboard is the only channel between the hosting node and the BT nodes.
Keys set by `BtActionServer` are required by every `BtActionNode`/`BtServiceNode`
constructor — a missing one is a runtime throw, not a compile error.

| Key | Type | Set by | Read by |
|---|---|---|---|
| `node` | `rclcpp::Node::SharedPtr` | `BtActionServer` (the `client_node_`) | every action/service BT node |
| `server_timeout` | `milliseconds` | `BtActionServer` | ditto (overridable per node via the `server_timeout` port) |
| `bt_loop_duration` | `milliseconds` | `BtActionServer` | ditto, to derive `max_timeout_` |
| `wait_for_service_timeout` | `milliseconds` | `BtActionServer` | ditto |
| `number_recoveries` | `int` | `syncai_task_runner::Navigator` (init) | incremented by `increment_recovery_count()` in recovery nodes; surfaced as action feedback |
| `failure_code` | `uint16_t` | ditto (init, and reset per goal) | 2026-11: the failed action's own `error_code`, written by every `report_failure()` (0 = none) — the planner nodes' is the planner's 201–224, which the navigator reports in place of `PLAN_FAILED`; read for the planner nodes only. Constant in `blackboard_keys.hpp` |
| `failed_node` / `failure_msg` | `std::string` | ditto (init, and reset per goal) | written by `BtActionNode::report_failure()` — `ComputePathToPose`, `ComputePathThroughPoses` and `FollowPath` on abort, the base class on an unacknowledged / rejected goal (with an empty `failed_node`) — and by `BtServiceNode`'s copy of it; read by the navigator in `goalCompleted()` to fill `NavigateToGoal`'s `error_code` / `error_msg`. Last writer wins; `move.xml`'s header has why that names the branch that failed the tree. Constants in `blackboard_keys.hpp` |
| `number_plans` | `int` | ditto (init, reset per goal) | incremented by `ComputePathToPose` on success; the navigator reports `plans − 1` as `number_of_replans` |
| `nav_status_msg` | `std::string` | ditto (init; set per goal to `kStatusPlanning`) | one English sentence of what the navigator is doing, for `NavigateToGoal`'s `status_msg` feedback (2026-10). Written by `IsPathValid` ("Path blocked; rerouting if it is still blocked after 10 s" while it keeps a blocked path (the number is `blocked_time`), "Path blocked; replanning" when it gives it up, "Following the path" over its own first sentence when the path clears; on a service answer only), `GlobalUpdatedGoal` ("Goal changed; replanning", once the goal has had a path), `ComputePathToPose` ("Following the path" on success, a retry sentence on abort), `FollowPath` (a retry sentence on abort, "Following the path" when the retry goes out), and the tree itself through BT.CPP's built-in `SetBlackboard` for a step whose node is too generic to know why it runs (`output_key` is the **bare** key name there). Last writer wins; the `ClearEntireCostmap` nodes write nothing. Free text, never branched on |
| `goal`, `path`, … | msg types | the tree itself, via ports | the tree itself |

`loadBehaviorTree()` re-writes the four server-owned keys onto **every**
blackboard in `tree_.blackboard_stack`, so subtrees get them too.

## The two base classes

Both derive straight from `BT::ActionNodeBase` (not from `SyncActionNode` /
`StatefulActionNode` / `AsyncActionNode`) so they can define their own tick
structure, and both are **template-method**: `tick()` is the fixed skeleton, and
a subclass only overrides hooks.

Neither ever blocks the tree thread for a full loop period. Both compute

```cpp
max_timeout_ = bt_loop_duration * 0.5;
```

and spin at most that long per tick, so the engine loop always gets back control
in time to check for cancellation. The cost is a "blind window" of roughly half a
period in which nothing is spinning; the benefit is a responsive loop.

### `BtActionNode<ActionT>` — wraps a ROS action

| Hook | Purpose | Default |
|---|---|---|
| `on_tick()` | Fill `goal_` from ports; may set `should_send_goal_ = false` to fail without sending | empty |
| `on_wait_for_result(feedback)` | Called each tick while waiting; may set `goal_updated_ = true` to re-send a modified goal | empty |
| `on_success()` | Write results to output ports | `SUCCESS` |
| `on_aborted()` | | `FAILURE` |
| `on_cancelled()` | | `SUCCESS` |

State machine: on the first tick (`IDLE`) it calls `on_tick()` and sends the goal
non-blocking; on later ticks it waits for the goal ack (up to `server_timeout`),
then `spin_some()`s its private callback group and returns `RUNNING` until a
result callback lands. `halt()` cancels the in-flight goal, waits for both the
cancel and the result future, calls `on_cancelled()`, and resets to `IDLE`.
A goal sent but **not yet acknowledged** is waited for first (up to
`server_timeout`) and cancelled once it has a handle (2026-11; upstream Humble
looked at the goal handle only, empty or the previous goal's until the ack, so
a halt in that window cancelled nothing and the server ran the goal behind a
CANCELED navigation).

Two failures are converted to a plain node `FAILURE` rather than propagating:
`send_goal failed` and `Goal was rejected by the action server`. Anything else
propagates up and the engine turns it into `BtStatus::FAILED`.

`report_failure(msg, attributed = true, code = 0)` (2026-10; `code` 2026-11)
is the base class's channel to the navigator for *why* a node failed: it
writes `failed_node` (the node's `registrationName()`, or `""` when
`attributed` is false), `failure_msg` and `failure_code` (the action result's
own `error_code` when it has one — the planner nodes pass the planner's — else
0, written every time so it always belongs to the same failure) to the
blackboard — the same channel as `increment_recovery_count()`, so no
output port and no XML wiring, and the tree stays action-agnostic (the
numeric error code belongs to the action being served, so the mapping from
node name to code lives in `syncai_task_runner`, next to that action). The base
`tick()` calls it unattributed on its own failures — the two
"did not acknowledge the goal" timeouts and the two converted exceptions —
because a server that never answered says nothing about planning or driving.
Subclasses call it from `on_aborted()`; nothing resets it on success, which is
deliberate (last writer wins, see `move.xml`).

Ports provided to every subclass by `providedBasicPorts()`: `server_name`
(remaps the action name) and `server_timeout`.

### `BtServiceNode<ServiceT>` — wraps a ROS service

| Hook | Purpose | Default |
|---|---|---|
| `on_tick()` | Fill `request_` from ports; may set `should_send_request_ = false` | empty |
| `on_completion(response)` | Interpret the response, return the final status | `SUCCESS` |
| `on_wait_for_result()` | Called on each tick that timed out waiting | empty |
| `on_timeout()` | Called once when the whole `server_timeout` budget is gone, just before `FAILURE` (2026-10) | empty — **on purpose**: the `ClearEntireCostmap` nodes must stay silent on a timeout (see `move.xml`); a node whose timeout *is* the failure overrides it to `report_failure(…, false)` |

A single `request_sent_` flag is the whole state machine. `check_future()` spins
up to `min(remaining budget, max_timeout_)` each tick: response arrived →
`on_completion()`; not yet but budget remains → `RUNNING`; budget exhausted →
`on_timeout()`, then `FAILURE`.

Note that `halt()` only resets the flag and returns to `IDLE` — a service call
cannot be cancelled, so an in-flight response is simply ignored.

`report_failure(msg, attributed = true)` exists here too since 2026-10, a
copy of `BtActionNode`'s with the same semantics (the two bases have no common
ancestor below `BT::ActionNodeBase`, and a free function would need the
blackboard handed in at every call site).

Ports provided by `providedBasicPorts()`: `service_name` and `server_timeout`.
`service_name` has a *description*, not a default, so a subclass passes its
default service name as the constructor's third argument (the
`ClearEntireCostmap` nodes pass none, so their XML must set it); the port still
overrides it.

## Node catalogue

Each plugin library registers its XML tag(s) in a `BT_REGISTER_NODES` block at
the bottom of its `.cpp`. **The library name and the XML tag are unrelated** —
the library name goes in `plugin_lib_names`, the tag goes in the XML.

| XML tag | Kind | Library | Ports |
|---|---|---|---|
| `ComputePathToPose` | action → `syncai_common/ComputePathToPose` on `compute_path_to_pose` (2026-11; nav2's goal, a result with the planner's `error_code`, passed on as `failure_code`) | `syncai_compute_path_to_pose_action_bt_node` | in `goal`, `planner_id`; out `path` (no `start` port: a plan always starts at the robot, see `syncai_global_planner`'s README). Bumps blackboard `number_plans` on success, `report_failure()`s on abort. A `goal` that changes while the plan is in flight (a preempt) is re-sent, and an answer to the superseded goal is dropped rather than written to `path` (2026-11) |
| `ComputePathThroughPoses` | action → `syncai_common/ComputePathThroughPoses` on `compute_path_through_poses` (2026-11, likewise) | `syncai_compute_path_through_poses_action_bt_node` | in `goals` (`std::vector<PoseStamped>`, blackboard only), `planner_id`; out `path`. `patrol.xml`'s (2026-11); `ComputePathToPose`'s twin, down to `number_plans`, `report_failure()` and the status sentences. nav2's `RemovePassedGoals` is not ported: the task_runner's navigator drops passed waypoints from `remaining_goals` itself (`blackboard_keys.hpp`) |
| `FollowPath` | action → `nav2_msgs/FollowPath` on `follow_path` | `syncai_follow_path_action_bt_node` | in `path`, `controller_id`, `goal_checker_id`. Never sends an empty path — the controller would throw "Invalid path, Path is empty." — and ignores an empty `path` while running. With no path to start from it **holds `RUNNING` without a goal** until one lands (2026-11; it failed until then, so a controller abort while the planner branch was replanning spent all of `move.xml`'s retries in as many ticks). `report_failure()`s on abort |
| `ClearEntireCostmap` | service → `nav2_msgs/ClearEntireCostmap` | `syncai_clear_costmap_service_bt_node` | in `service_name` |
| `ClearCostmapExceptRegion` | service | ditto | + in `reset_distance` (default 1) |
| `ClearCostmapAroundRobot` | service | ditto | + in `reset_distance` (default 1) |
| `PipelineSequence` | control | `syncai_pipeline_sequence_bt_node` | — |
| `RecoveryNode` | control | `syncai_recovery_node_bt_node` | in `number_of_retries` (default 1), `retry_refill_time` (s, default 0 = never) |
| `RateController` | decorator | `syncai_rate_controller_bt_node` | in `hz` (default 10.0) |
| `IsPathValid` | condition → `nav2_msgs/IsPathValid` on `is_path_valid` (syncai_global_planner) | `syncai_is_path_valid_condition_bt_node` | in `path`, `service_name` (default `is_path_valid`), `server_timeout`, `blocked_time` (s, default 0; `move.xml` / `patrol.xml` set 10), `blocked_match_distance` (m, default 0.5) |
| `GlobalUpdatedGoal` | condition | `syncai_globally_updated_goal_condition_bt_node` | — (reads blackboard `goal` and, since 2026-11, `goals`) |

Semantics of the non-obvious ones:

- **`PipelineSequence`** re-ticks *all* earlier children every round instead of
  parking on the first `RUNNING` child. That is what lets the planner branch
  keep checking (and replacing) the path while `FollowPath` is still driving. It returns `RUNNING` as
  soon as a child at index ≥ `last_child_ticked_` returns `RUNNING`; a `RUNNING`
  from an *earlier* child is skipped over. Any `FAILURE` halts all children.
- **`RecoveryNode`** requires **exactly two children** (throws otherwise): child 0
  is the work, child 1 is the recovery. Child 0 fails → tick child 1 → on its
  success, retry child 0, up to `number_of_retries`. Child 1 failing fails the
  whole node. `number_of_retries` is read **once in the constructor**, so it
  cannot be changed via the blackboard at runtime. The count resets only when
  the node itself returns, so around a child 0 that stays `RUNNING` for a whole
  goal (`FollowPath`) it is a budget per goal. `retry_refill_time` (not
  upstream; also read once) makes it per incident: when child 0 fails after
  running at least that many seconds since it was last started, the count goes
  back to 0 first, and an `INFO` from logger `RecoveryNode` says so. It is time
  and not "child 0 returned `RUNNING`" because a failing attempt is `RUNNING`
  too until it fails — refilling on that would retry a dead end forever. Pick
  it longer than any attempt that fails without getting going.
- **`RateController`** ticks its child only when the period has elapsed — *or*
  when the child is already `RUNNING`, so a long-running child is never starved.
  The timer resets when the child returns `SUCCESS`.
- **`IsPathValid`** is synchronous, as upstream: it blocks its tick for up to
  `server_timeout` and never returns `RUNNING`. An empty path answers `FAILURE`
  without a call; a timeout answers `SUCCESS` with a WARN (2026-10) — in doubt
  the tree **keeps** the path, because a replan replaces a route nobody has
  shown to be blocked, while a kept one is re-checked in a second and the
  controller holds the robot meanwhile. Until 2026-10-28 the usual cause was
  the 2D global costmap's 1 Hz update holding its mutex past
  `server_timeout`; the check is answered by `Octo3D` on its dynamic overlay
  since, so a WARN every tick now means that answer is slower than
  `server_timeout` and the timeout in `move.xml` must go up, not that the
  answer should flip back. Not upstream (2026-10): a
  `false` answers `FAILURE` only once the same spot (a blocked pose within
  `blocked_match_distance` of one blocked last time) of the same path (stamp +
  size) has been answered blocked for `blocked_time` seconds (steady clock,
  from the first blocked answer); the ones before answer `SUCCESS`, keep the
  path and log `Path blocked at (x, y) for n of N s; keeping it` in the
  task_runner log. A valid answer, an empty path or a new plan restarts the
  wait; a timeout leaves it running. It exists because a person
  crossing the route was a replan, and the detour then stuck for the goal.
- **`GlobalUpdatedGoal`** returns `SUCCESS` on the first tick after blackboard
  `goal` or `goals` changed. The very first tick only records a baseline
  (`FAILURE`). `goals` is watched again since 2026-11, when the
  `navigate_through_poses` navigator started writing it; `patrol.xml` plans
  through a separate `remaining_goals`, so passing a waypoint is not a changed
  goal.

The three `ClearCostmap*` nodes all call `increment_recovery_count()`, which bumps
the `number_recoveries` blackboard key that the navigator reports as feedback.
They deliberately do **not** `report_failure()`: a clear only runs because an
action node just failed, so the last action-node write already names the
right branch, and a service timeout here would overwrite it with a name that
maps to `UNKNOWN`.

### `bt_conversions.hpp`

Specialisations of `BT::convertFromString<>` so ROS types can be written as XML
attributes. All are semicolon-separated:

| Type | Format |
|---|---|
| `geometry_msgs/Point` | `x;y;z` |
| `geometry_msgs/Quaternion` | `x;y;z;w` |
| `geometry_msgs/PoseStamped` | `stamp;frame_id;px;py;pz;qx;qy;qz;qw` (9 fields) |
| `std::chrono::milliseconds` | integer milliseconds |

Include this header from any new plugin that takes such a port — a missing
specialisation shows up as an XML parse error at tree-load time.

## The tree in use

`syncai_task_runner/behavior_trees/move.xml`. Below is its `PipelineSequence`
(the new-goal `ClearEntireCostmap` on `planner_server/Octo3D/clear_overlay`
that precedes it is left out) — replanning only when the path becomes invalid
or the goal changes, with contextual recovery (the why is in the XML header
and `syncai_task_runner`'s README). `planner_id="Octo3D"`,
`controller_id="FollowPath"` (MPPI, 3D-aware), and the planner branch's
`ClearEntireCostmap` on `clear_overlay`, the planner's dynamic-obstacle memory
— which is also what its `IsPathValid` is judged on, since planner_server asks
the planner that made the last plan (2026-10-21). This was `move_3d.xml` until
2026-10-28; the 2D `move.xml` it replaced (`GridBased`, clears on
`global_costmap/clear_entirely_global_costmap`) went with 2D navigation.

```xml
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
      <ClearEntireCostmap name="ClearDynamicOverlay-Context" service_name="planner_server/Octo3D/clear_overlay"/>
    </RecoveryNode>
  </RateController>
  <RecoveryNode number_of_retries="4" retry_refill_time="60.0" name="FollowPath">
    <FollowPath path="{path}" controller_id="FollowPath"/>
    <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
  </RecoveryNode>
</PipelineSequence>
```

The service names are **relative**, so they resolve under the robot namespace
like everything else in the stack. Plugin libraries and timeouts live in
`syncai_task_runner/params/task_runner_params.yaml`.

Ported from nav2's `navigate_w_replanning_only_if_path_becomes_invalid.xml`, minus the
outer system-level recovery branch (Spin / Wait / BackUp via RoundRobin) — those
BT nodes and the behavior server are not ported yet.

## Adding a BT node

1. Write the header under `include/syncai_behavior_tree/plugins/<kind>/`,
   deriving from `BtActionNode<T>` / `BtServiceNode<T>` / a BT.CPP base class.
   If it takes ports, override `providedPorts()` and call `providedBasicPorts({…})`
   so `server_name` / `service_name` / `server_timeout` are kept.
2. Implement the hooks in `plugins/<kind>/<name>.cpp` and end the file with
   `BT_REGISTER_NODES(factory) { … }`.
   - Plain node → `factory.registerNodeType<T>("XmlTag")`.
   - A `BtActionNode` needs its action name, which the two-argument constructor
     signature can't supply, so register a `BT::NodeBuilder` lambda instead —
     see `compute_path_to_pose_action.cpp`.
   - If the node's failure should be attributable in a `NavigateToGoal` result,
     call `report_failure(...)` from `on_aborted()` and add the registration
     name to the navigator's node → code table (`pose_navigator.cpp` in
     `syncai_task_runner`); an unlisted node maps to `UNKNOWN` with its
     message, which is safe but uninformative.
3. In `CMakeLists.txt`: `add_library(<lib> SHARED plugins/…)` followed by
   `list(APPEND plugin_libs <lib>)`. The `foreach` below wires includes,
   dependencies and `BT_PLUGIN_EXPORT` automatically.
4. Add `<lib>` to `plugin_lib_names` in the consumer's params YAML
   (`syncai_task_runner/params/task_runner_params.yaml`) — the compiled library is
   invisible until it is listed there.
5. Use the tag in the BT XML.

## Build and try

```bash
colcon build --packages-select syncai_behavior_tree
```

Every node library it builds is one `syncai_task_runner` loads. Removed on
2026-10-28 as unused: the `InitialPoseReceived` condition (nav2's AMCL-era
"wait for an initial pose" gate, built but never loaded, with the
`initial_pose_received` blackboard key the navigator seeded for it) and
`examples/demo.cpp`, a standalone BT.CPP tutorial executable (`demo`) that
used none of this package.

Only the ament linters run under `colcon test`; there are no unit tests.

## Gotchas

- **Plugin library name ≠ XML tag.** Forgetting to add the library to
  `plugin_lib_names` produces an "unknown node type" error at tree load, which
  reads as an XML mistake but is a params mistake.
- **This package depends on `syncai_common`** — for client types only: the
  planner actions (2026-11). The rule that kept it out
  (commit `64a471b`) still holds: no BT node reads an action's error-code
  constants, the node → code table stays in `syncai_task_runner`. A change to
  `syncai_common` now rebuilds this package too.
- **`SetBlackboard output_key` is the bare key.** `output_key="nav_status_msg"`
  writes the key; `output_key="{nav_status_msg}"` reads the current sentence
  and uses *that* as the key name.
- **BT node constructors throw when their server is absent.** The tree is built
  at `BtActionServer` construction, so the whole hosting node fails to start.
  Start servers before the task runner.
- **`always_reload_bt_xml: false` caches the XML** after the first goal. Editing
  the tree without flipping this (or restarting) silently changes nothing.
- **`bt_loop_duration` does double duty**: tick period *and* (halved) the per-tick
  spin budget of every action/service node. Lowering it to speed the loop also
  shortens how long each node may wait for its server per tick.
- **`server_timeout` is per-goal-ack/result, not per-action.** A long-running
  action returns `RUNNING` for as long as it needs. The budget is for the ack,
  the cancel and result in `halt()`, and a whole service call. 20 ms (nav2's
  default) is one spin at `bt_loop_duration` 50, so one late answer failed the
  node; the task runner ships 100 since 2026-11 (its params file has why). The
  ack / service waits return `RUNNING` across ticks; `halt()` blocks for up to
  two of them.
- **`RecoveryNode` with anything other than two children throws** at tick time.

## Further reading

The porting-time design notes that were in `docs/` (BT.CPP's tick recursion,
the three action-node types, a walk through `BtServiceNode`) were removed on
2026-10-28; git history has them.

Upstream reference: [`nav2_behavior_tree`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_behavior_tree)
and the [BehaviorTree.CPP v3 docs](https://www.behaviortree.dev/). The vendored
BT.CPP source is at `src/third-party/behaviortree_cpp_v3/` (submodule, tag 3.8.8).
