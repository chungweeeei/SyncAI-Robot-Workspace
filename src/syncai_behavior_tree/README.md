# syncai_behavior_tree

The behavior-tree layer of the navigation stack: a port of `nav2_behavior_tree`
onto BehaviorTree.CPP **v3.8**.

It ships three things, and it is worth keeping them apart because they are
consumed in completely different ways:

| | What | How it reaches a consumer |
|---|---|---|
| **Engine + base classes** | `BehaviorTreeEngine`, `BtActionServer`, `BtActionNode`, `BtServiceNode`, `bt_conversions` | `libsyncai_behavior_tree.so` + headers, linked at **build** time |
| **BT node plugins** | `ComputePathToPose`, `FollowPath`, `ClearEntireCostmap`, `PipelineSequence`, `RecoveryNode`, `RateController`, `InitialPoseReceived` | one shared library each, `dlopen`ed by **name at runtime** |
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
                 ├─ ComputePathToPose  ──action──►  syncai_planner
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
docs/                          design notes (Chinese) — see "Further reading"
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
what the `sleep` offsets in `scripts/byobu_session*.sh` are for.

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

Ports provided to every subclass by `providedBasicPorts()`: `server_name`
(remaps the action name) and `server_timeout`.

### `BtServiceNode<ServiceT>` — wraps a ROS service

| Hook | Purpose | Default |
|---|---|---|
| `on_tick()` | Fill `request_` from ports; may set `should_send_request_ = false` | empty |
| `on_completion(response)` | Interpret the response, return the final status | `SUCCESS` |
| `on_wait_for_result()` | Called on each tick that timed out waiting | empty |

A single `request_sent_` flag is the whole state machine. `check_future()` spins
up to `min(remaining budget, max_timeout_)` each tick: response arrived →
`on_completion()`; not yet but budget remains → `RUNNING`; budget exhausted →
`FAILURE`.

Note that `halt()` only resets the flag and returns to `IDLE` — a service call
cannot be cancelled, so an in-flight response is simply ignored.

Ports provided by `providedBasicPorts()`: `service_name` and `server_timeout`.

## Node catalogue

Each plugin library registers its XML tag(s) in a `BT_REGISTER_NODES` block at
the bottom of its `.cpp`. **The library name and the XML tag are unrelated** —
the library name goes in `plugin_lib_names`, the tag goes in the XML.

| XML tag | Kind | Library | Ports |
|---|---|---|---|
| `ComputePathToPose` | action → `nav2_msgs/ComputePathToPose` on `compute_path_to_pose` | `syncai_compute_path_to_pose_action_bt_node` | in `goal`, `start`, `planner_id`; out `path` |
| `FollowPath` | action → `nav2_msgs/FollowPath` on `follow_path` | `syncai_follow_path_action_bt_node` | in `path`, `controller_id`, `goal_checker_id` |
| `ClearEntireCostmap` | service → `nav2_msgs/ClearEntireCostmap` | `syncai_clear_costmap_service_bt_node` | in `service_name` |
| `ClearCostmapExceptRegion` | service | ditto | + in `reset_distance` (default 1) |
| `ClearCostmapAroundRobot` | service | ditto | + in `reset_distance` (default 1) |
| `PipelineSequence` | control | `syncai_pipeline_sequence_bt_node` | — |
| `RecoveryNode` | control | `syncai_recovery_node_bt_node` | in `number_of_retries` (default 1) |
| `RateController` | decorator | `syncai_rate_controller_bt_node` | in `hz` (default 10.0) |
| `InitialPoseReceived` | condition | `syncai_initial_pose_received_condition_bt_node` | reads blackboard `initial_pose_received` |

Semantics of the three non-obvious ones:

- **`PipelineSequence`** re-ticks *all* earlier children every round instead of
  parking on the first `RUNNING` child. That is what lets `ComputePathToPose`
  keep replanning while `FollowPath` is still driving. It returns `RUNNING` as
  soon as a child at index ≥ `last_child_ticked_` returns `RUNNING`; a `RUNNING`
  from an *earlier* child is skipped over. Any `FAILURE` halts all children.
- **`RecoveryNode`** requires **exactly two children** (throws otherwise): child 0
  is the work, child 1 is the recovery. Child 0 fails → tick child 1 → on its
  success, retry child 0, up to `number_of_retries`. Child 1 failing fails the
  whole node. `number_of_retries` is read **once in the constructor**, so it
  cannot be changed via the blackboard at runtime.
- **`RateController`** ticks its child only when the period has elapsed — *or*
  when the child is already `RUNNING`, so a long-running child is never starved.
  The timer resets when the child returns `SUCCESS`.

The three `ClearCostmap*` nodes all call `increment_recovery_count()`, which bumps
the `number_recoveries` blackboard key that the navigator reports as feedback.

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

`syncai_task_runner/behavior_trees/move.xml` — replanning at 0.333 Hz with
contextual recovery:

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

The service names are **relative**, so they resolve under the robot namespace
like everything else in the stack. Plugin libraries and timeouts live in
`syncai_task_runner/params/task_runner_params.yaml`.

Ported from nav2's `navigate_to_pose_w_replanning_and_recovery.xml`, minus the
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

`docs/` holds longer design notes (in Chinese) written while porting:

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
