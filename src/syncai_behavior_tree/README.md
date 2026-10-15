# syncai_behavior_tree

The behavior-tree layer of the navigation stack: a port of `nav2_behavior_tree`
onto BehaviorTree.CPP **v3.8**.

It ships three things, and it is worth keeping them apart because they are
consumed in completely different ways:

| | What | How it reaches a consumer |
|---|---|---|
| **Engine + base classes** | `BehaviorTreeEngine`, `BtActionServer`, `BtActionNode`, `BtServiceNode`, `bt_conversions` | `libsyncai_behavior_tree.so` + headers, linked at **build** time |
| **BT node plugins** | `ComputePathToPose`, `FollowPath`, `ClearEntireCostmap`, `PipelineSequence`, `RecoveryNode`, `RateController`, `IsPathValid`, `GlobalUpdatedGoal`, `InitialPoseReceived` | one shared library each, `dlopen`ed by **name at runtime** |
| **A standalone demo** | `examples/demo.cpp` | `ros2 run syncai_behavior_tree demo` |

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
                 └─ ClearEntireCostmap ──service─►  syncai_costmap_2d
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
examples/demo.cpp
docs/                          design notes (English) — see "Further reading"
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
        on_loop  = preempt check (→ on_preempt_callback) + on_loop_callback
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
| `default_server_timeout` | `20` ms | How long a BT node waits for a goal ack / result / cancel before giving up |
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
| `tf_buffer` | `shared_ptr<tf2_ros::Buffer>` | `syncai_task_runner::Navigator` | frame-aware nodes |
| `odom_smoother` | `shared_ptr<syncai_util::OdomSmoother>` | ditto | nodes needing current speed |
| `initial_pose_received` | `bool` | ditto | `InitialPoseReceived` condition |
| `number_recoveries` | `int` | ditto (init) | incremented by `increment_recovery_count()` in recovery nodes; surfaced as action feedback |
| `failed_node` / `failure_msg` | `std::string` | ditto (init, and reset per goal) | written by `BtActionNode::report_failure()` — `ComputePathToPose`, `FollowPath` and `DockToPose` on abort, the base class on an unacknowledged / rejected goal (with an empty `failed_node`) — and, since 2026-10, by `BtServiceNode`'s copy of it (`SetMotionKey` on a refused key; unattributed on a timeout) and by `ComputeStagingPose` / `Wait` by hand; read by the navigator in `goalCompleted()` to fill `NavigateToGoal`'s `error_code` / `error_msg`. Last writer wins; `move.xml`'s header has why that names the branch that failed the tree, `dock.xml`'s has the docking case. Constants in `blackboard_keys.hpp` |
| `number_plans` | `int` | ditto (init, reset per goal) | incremented by `ComputePathToPose` on success; the navigator reports `plans − 1` as `number_of_replans` |
| `nav_status_msg` | `std::string` | ditto (init; set per goal to `kStatusPlanning`) | one English sentence of what the navigator is doing, for `NavigateToGoal`'s `status_msg` feedback (2026-10). Written by `IsPathValid` ("Path blocked; rerouting if it is still blocked after 10 s" while it keeps a blocked path (the number is `blocked_time`), "Path blocked; replanning" when it gives it up, "Following the path" over its own first sentence when the path clears; on a service answer only), `GlobalUpdatedGoal` ("Goal changed; replanning", once the goal has had a path), `ComputePathToPose` ("Following the path" on success, a retry sentence on abort), `FollowPath` (a retry sentence on abort, "Following the path" when the retry goes out), `DockToPose` ("Approaching the dock", "Docking: 0.42 m from the dock" from the controller's feedback, "Docked", and the undock counterparts) and the docking trees themselves through BT.CPP's built-in `SetBlackboard` ("Switching the gait to MPC for docking" — written by the tree because `SetMotionKey` is generic and does not know why it is sending a key; `output_key` is the **bare** key name there). Last writer wins; the `ClearEntireCostmap` nodes write nothing. Free text, never branched on |
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

Two failures are converted to a plain node `FAILURE` rather than propagating:
`send_goal failed` and `Goal was rejected by the action server`. Anything else
propagates up and the engine turns it into `BtStatus::FAILED`.

`report_failure(msg, attributed = true)` (2026-10) is the base class's channel
to the navigator for *why* a node failed: it writes `failed_node` (the node's
`registrationName()`, or `""` when `attributed` is false) and `failure_msg` to
the blackboard — the same channel as `increment_recovery_count()`, so no
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
| `on_timeout()` | Called once when the whole `server_timeout` budget is gone, just before `FAILURE` (2026-10) | empty — **on purpose**: the `ClearEntireCostmap` nodes must stay silent on a timeout (see `move.xml`); a node whose timeout *is* the failure (`SetMotionKey`) overrides it to `report_failure(…, false)` |

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
default service name as the constructor's third argument (`SetMotionKey` →
`set_motion_key`); the port still overrides it.

## Node catalogue

Each plugin library registers its XML tag(s) in a `BT_REGISTER_NODES` block at
the bottom of its `.cpp`. **The library name and the XML tag are unrelated** —
the library name goes in `plugin_lib_names`, the tag goes in the XML.

| XML tag | Kind | Library | Ports |
|---|---|---|---|
| `ComputePathToPose` | action → `nav2_msgs/ComputePathToPose` on `compute_path_to_pose` | `syncai_compute_path_to_pose_action_bt_node` | in `goal`, `start`, `planner_id`; out `path`. Bumps blackboard `number_plans` on success, `report_failure()`s on abort |
| `FollowPath` | action → `nav2_msgs/FollowPath` on `follow_path` | `syncai_follow_path_action_bt_node` | in `path`, `controller_id`, `goal_checker_id`. Refuses to send an empty path (`FAILURE` with a WARN, 2026-10) and ignores an empty `path` while running — the controller would throw "Invalid path, Path is empty." and the retry would be spent on a path that cannot change until the planner branch runs again. `report_failure()`s on abort |
| `DockToPose` | action → `syncai_common/DockToPose` on `dock_to_pose` (syncai_controller) | `syncai_dock_to_pose_action_bt_node` | in `method` (default `position` = `dock_pose` as given; `charger` = dock against a tag on the charger, reserved: refused with `FAILURE` until its detector exists, as is any other value), `dock_pose` (`PoseStamped`, any frame; sent as the action's planar `frame_id` / `x` / `y` / `yaw`), `controller_id` (default `Docking`; `Undocking` backs out), `goal_checker_id` (default `dock_goal_checker`), `timeout` (s, default 0 = server default). Refuses a pose without a `frame_id`. Writes `nav_status_msg` from the controller's feedback; `report_failure()`s on abort with the controller's reason (`ABORTED` + `NONE` is spelled out as "busy or superseded"); `on_cancelled()` returns `FAILURE`. 2026-10 |
| `SetMotionKey` | service → `syncai_common/SetMotionKey` on `set_motion_key` (syncai_driver_manager) | `syncai_set_motion_key_service_bt_node` | in `key` (**required**: `0` stand, `1` RL, `2` lie down, `3` damping, `4` e-stop, `5` MPC). `FAILURE` + `report_failure()` when the driver refuses (`LOCKED`, unknown key); unattributed on a timeout. `success` means the UDP datagram left, not that the gait changed. 2026-10 |
| `ComputeStagingPose` | `SyncActionNode`, no ROS | `syncai_compute_staging_pose_action_bt_node` | in `dock_pose`, `distance` (m, default 1.0); out `staging_pose` = the dock pose moved straight back along its yaw. Fails (reason on the blackboard) on a missing pose, `distance <= 0`, or a quaternion that is not a rotation. 2026-10 |
| `ComputeBackOutPose` | `SyncActionNode`, no TF | `syncai_compute_back_out_pose_action_bt_node` | in `distance` (m, default 1.0); out `back_out_pose` = `(-distance, 0)`, identity orientation, in the BT client node's `base_frame` (`<robot_id>/base_link` from task_runner's launch). No lookup: `dock_to_pose` transforms it into odom at acceptance, so it means "behind where the robot stands when the back-out starts". `undock.xml`'s target. Fails (reason on the blackboard) on `distance <= 0` or an empty `base_frame`. 2026-10-15 |
| `Wait` | `StatefulActionNode`, no ROS | `syncai_wait_action_bt_node` | in `msec` (**required**). `RUNNING` until a `steady_clock` deadline, then `SUCCESS`. Preferred over BT.CPP's `Delay` decorator, which can only postpone the *next* node's first tick. The docking trees use it for settling time after a confirmed MPC switch and for the stand-up / lie-down, which cannot be confirmed. 2026-10 |
| `WaitForMotionState` | `StatefulActionNode`, subscribes `mode` (`std_msgs/Int32MultiArray`, syncai_driver_manager) | `syncai_wait_for_motion_state_action_bt_node` | in `state` (**required**; 5 = MPC), `index` (default 1 = motion state; 0 = policy), `timeout` (ms, default 5000), `topic` (default `mode`). `RUNNING` until the **latest** sample's `data[index]` equals `state` (a robot already there passes at once — `mode` has no fixed rate and no latch, so a freshness rule would time out a quiet link), then `SUCCESS` with the elapsed ms logged; `FAILURE` + the failure keys after `timeout`, naming the last state seen. Own callback group, spun without blocking each tick. 2026-10-14 |
| `ClearEntireCostmap` | service → `nav2_msgs/ClearEntireCostmap` | `syncai_clear_costmap_service_bt_node` | in `service_name` |
| `ClearCostmapExceptRegion` | service | ditto | + in `reset_distance` (default 1) |
| `ClearCostmapAroundRobot` | service | ditto | + in `reset_distance` (default 1) |
| `PipelineSequence` | control | `syncai_pipeline_sequence_bt_node` | — |
| `RecoveryNode` | control | `syncai_recovery_node_bt_node` | in `number_of_retries` (default 1), `retry_refill_time` (s, default 0 = never) |
| `RateController` | decorator | `syncai_rate_controller_bt_node` | in `hz` (default 10.0) |
| `IsPathValid` | condition → `nav2_msgs/IsPathValid` on `is_path_valid` (syncai_global_planner) | `syncai_is_path_valid_condition_bt_node` | in `path`, `service_name` (default `is_path_valid`), `server_timeout`, `blocked_time` (s, default 0; `move.xml` / `dock.xml` set 10), `blocked_match_distance` (m, default 0.5) |
| `GlobalUpdatedGoal` | condition | `syncai_globally_updated_goal_condition_bt_node` | — (reads blackboard `goal`) |
| `InitialPoseReceived` | condition | `syncai_initial_pose_received_condition_bt_node` | reads blackboard `initial_pose_received` |

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
  shown to be blocked (and the 2D-raytraced global costmap can have forgotten
  the blocker the detour was for), while a kept one is re-checked in a second
  and RPP stops the robot meanwhile. The timeout is the global costmap's 1 Hz
  update holding the costmap mutex past `server_timeout`; a WARN every tick
  means the two loops phase-locked and `server_timeout` in `move.xml` must go
  up, not that the answer should flip back. Not upstream (2026-10): a
  `false` answers `FAILURE` only once the same spot (a blocked pose within
  `blocked_match_distance` of one blocked last time) of the same path (stamp +
  size) has been answered blocked for `blocked_time` seconds (steady clock,
  from the first blocked answer); the ones before answer `SUCCESS`, keep the
  path and log `Path blocked at (x, y) for n of N s; keeping it` in the
  task_runner log. A valid answer, an empty path or a new plan restarts the
  wait; a timeout leaves it running. It exists because a person
  crossing the route was a replan, and the detour then stuck for the goal.
- **`GlobalUpdatedGoal`** returns `SUCCESS` on the first tick after blackboard
  `goal` changed. The very first tick only records a baseline (`FAILURE`).
  Upstream also watches `goals`; nothing here writes it, so this port does not.

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

`syncai_task_runner/behavior_trees/move.xml` — replanning only when the path
becomes invalid or the goal changes, with contextual recovery (the why is in the
XML header and `syncai_task_runner`'s README):

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
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </Fallback>
      <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
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

Two more trees since 2026-10, `syncai_task_runner/behavior_trees/dock.xml`
and `undock.xml`, selected per goal through `NavigateToGoal.behavior_tree`.
`dock.xml` is the tree above with `goal="{staging_pose}"` (from
`ComputeStagingPose`) as its first half, then `SetMotionKey key="5"` → `Wait`
→ `DockToPose`, with `SetMotionKey key="1"` restoring RL locomotion on
failure; `undock.xml` is stand (`key="0"`) → `Wait` → MPC → `Wait` →
`ComputeBackOutPose` → `DockToPose controller_id="Undocking"` → RL (no goal pose: 1 m behind the robot, 2026-10-15). Besides
the docking nodes they use BT.CPP's built-in `Sequence`, `Fallback`,
`ForceFailure` and `SetBlackboard` (whose `output_key` is the bare key name,
not `{key}`). The XML headers carry the rationale; `syncai_task_runner`'s
README has the operator-level description.

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
source install/setup.bash
ros2 run syncai_behavior_tree demo
```

`examples/demo.cpp` is a **standalone BT.CPP tutorial**, not part of the nav
stack: it builds its own factory with a few `SyncActionNode`s, shows a custom
`convertFromString` for a user struct, port ⇄ blackboard remapping, an
`ActionNodeBase` that returns `RUNNING`, and a `<SubTree>` pulled in via
`<include>`. It does not use `BehaviorTreeEngine`, any plugin, or any ROS action.

Only the ament linters run under `colcon test`; there are no unit tests.

## Gotchas

- **Plugin library name ≠ XML tag.** Forgetting to add the library to
  `plugin_lib_names` produces an "unknown node type" error at tree load, which
  reads as an XML mistake but is a params mistake.
- **This package depends on `syncai_common` since 2026-10** — for the client
  types of `DockToPose` and `SetMotionKey` only. The rule that kept it out
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
- **`server_timeout` is per-goal-ack/result, not per-action.** A 20 ms default is
  fine for an ack; a long-running action returns `RUNNING` for as long as it needs.
- **`RecoveryNode` with anything other than two children throws** at tick time.

## Further reading

`docs/` holds longer design notes (in English) written while porting:

- [`docs/behavior_tree_tick_notes.md`](docs/behavior_tree_tick_notes.md) — how
  `tickRoot()` → `executeTick()` → `tick()` recursion works, `SequenceNode`'s
  `current_child_idx_` progress memory, and when `loopTimeout` actually matters.
- [`docs/action_node_types.md`](docs/action_node_types.md) — `SyncActionNode` vs
  `StatefulActionNode` vs `AsyncActionNode`, threading and cancellation for each,
  and why real ROS actions use `BtActionNode` instead.
- [`docs/bt_service_node.md`](docs/bt_service_node.md) — a line-by-line walk
  through `BtServiceNode`: the template-method structure, the two timeouts,
  why `spin_until_future_complete` is required for the future to progress, and
  the resulting latency analysis.

Note: the first two documents refer to `examples/sync_action_demo.cpp`,
`stateful_action_demo.cpp` and `async_action_demo.cpp`, which are not in the tree
— only `examples/demo.cpp` exists. The explanations stand on their own; the file
references do not.

Upstream reference: [`nav2_behavior_tree`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_behavior_tree)
and the [BehaviorTree.CPP v3 docs](https://www.behaviortree.dev/). The vendored
BT.CPP source is at `src/third-party/behaviortree_cpp_v3/` (submodule, tag 3.8.8).
