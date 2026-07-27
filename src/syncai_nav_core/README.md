# syncai_nav_core

The abstract plugin interfaces the nav stack is built around. Four pure-virtual
classes and one exception type — **no implementation, no library target, no
node**. Port of `nav2_core`.

`CMakeLists.txt` installs `include/` and exports it; that is the whole build.
Its value is that it is the one package both the servers and their plugins
depend on, so neither has to depend on the other.

```
                syncai_nav_core  (interfaces only)
                 ▲      ▲      ▲
      implements │      │      │ loads via pluginlib
                 │      │      │
   syncai_planner plugins    syncai_planner / syncai_controller (servers)
   syncai_controller plugins
```

## The interfaces

| Header | Class | Implemented by |
|---|---|---|
| `global_planner.hpp` | `GlobalPlanner` | `syncai_planner`: `NavfnPlanner`, `StraightLinePlanner` |
| `controller.hpp` | `Controller` | `syncai_controller`: `RegulatedPurePursuitController` |
| `goal_checker.hpp` | `GoalChecker` | `syncai_controller`: `SimpleGoalChecker`, `StoppedGoalChecker`, `PositionGoalChecker` |
| `progress_checker.hpp` | `ProgressChecker` | `syncai_controller`: `SimpleProgressChecker`, `PoseProgressChecker` |
| `exceptions.hpp` | `PlannerException` | thrown by the controller path |

**`GlobalPlanner`** — `initialize(node, name, tf, costmap_ros)` then
`createPlan(start, goal) → nav_msgs::msg::Path`. An empty path means failure;
there is no separate error channel.

**`Controller`** — `initialize(node, name, tf, costmap_ros)`, `setPlan(path)`,
`computeVelocityCommands(pose, velocity, goal_checker) → TwistStamped`, and
`setSpeedLimit(limit, percentage)`. The goal checker is passed **into** the
controller so it can read the tolerances (RPP uses the xy tolerance to decide
when to switch into rotate-to-goal-heading) — the two plugins are coupled by
design, through this interface rather than directly.

**`GoalChecker`** — `initialize(node, plugin_name, costmap_ros)`, `reset()`,
`isGoalReached(query_pose, goal_pose, velocity)`, and `getTolerances(...)`.
`getTolerances` fills any unmeasured field with
`std::numeric_limits<double>::lowest()` as a sentinel, so a caller can tell
"tolerance not applicable" from "tolerance is zero".

**`ProgressChecker`** — `initialize(node, plugin_name)`, `check(current_pose)`,
`reset()`. The only interface that takes neither TF nor a costmap.

**`PlannerException`** — a single `std::runtime_error` subclass. Despite the
name it is thrown from the **controller** side: `ControllerServer` uses it for
"Failed to obtain robot pose" / "Failed to make progress" / "Controller patience
exceeded", and RPP for "collision ahead!" and its transform failures.
`PlannerServer` does not throw it at all. nav2 splits these into a hierarchy of
`PlannerException` / `ControllerException` subtypes; this port kept one.

## What changed from nav2_core

Two differences, both consequences of the workspace dropping lifecycle nodes:

1. **Plugins receive a plain `rclcpp::Node::SharedPtr`**, not a
   `nav2_util::LifecycleNode::WeakPtr`. Plugins call `declare_parameter` /
   `get_parameter` / `create_publisher` on it directly, and can hold the shared
   pointer without weak-pointer locking.
2. **There is no `activate()` / `deactivate()` / `cleanup()`.** `initialize()`
   does the full setup — parameters, publishers, dynamic-parameter callbacks —
   and teardown happens in the plugin's destructor. A plugin ported from nav2
   needs its `on_activate`/`on_cleanup` bodies folded into those two places.

Also note the naming: nav2 calls the setup method `configure()`, this port calls
it `initialize()`.

## What is not ported

`nav2_core` additionally defines `Behavior`, `Smoother`, `WaypointTaskExecutor`,
`GlobalPlanner`'s exception hierarchy and several others. Only the four above
exist here, because only they have implementations and consumers in this stack.
The absence of `Behavior` is why `syncai_task_runner`'s BT has no Spin / Wait /
BackUp recovery branch — there is no behavior server to run them.

## Writing a plugin

Plugins live in the package that implements them, not here. The wiring has three
parts and all three must agree:

1. **Inherit and export** in the implementing `.cpp`:

   ```cpp
   PLUGINLIB_EXPORT_CLASS(syncai_controller::MyGoalChecker, syncai_nav_core::GoalChecker)
   ```

2. **Declare it in the package's plugin XML** and register that XML against
   **`syncai_nav_core`** as the base package:

   ```xml
   <!-- plugins.xml -->
   <class type="syncai_controller::MyGoalChecker"
          base_class_type="syncai_nav_core::GoalChecker">
     <description>…</description>
   </class>
   ```

   ```cmake
   pluginlib_export_plugin_description_file(syncai_nav_core plugins.xml)
   ```

   ```xml
   <!-- package.xml -->
   <export>
     <syncai_nav_core plugin="${prefix}/plugins.xml" />
   </export>
   ```

3. **Name it in the params file** under the owning server's plugin list, with
   `<id>.plugin` giving the type:

   ```yaml
   goal_checker_plugins: ["goal_checker"]
   goal_checker:
     plugin: "syncai_controller::MyGoalChecker"
   ```

The servers construct their loaders as
`pluginlib::ClassLoader<T>("syncai_nav_core", "syncai_nav_core::T")` — the first
argument is this package, which is why every plugin XML in the workspace is
registered against it regardless of which package the plugin lives in.

## Gotchas

- **This package must be rebuilt and its dependents rebuilt together.** Changing
  a pure-virtual signature silently breaks the ABI between a server and an
  already-built plugin `.so`; the symptom is a load failure or a crash at the
  first virtual call, not a compile error. Use
  `colcon build --packages-up-to syncai_planner syncai_controller`.
- **A plugin load failure is fatal.** Both servers call `exit(-1)` on a
  `pluginlib::PluginlibException`, so a typo in a `.plugin` string takes the
  whole process down at startup.
- **The two packages spell plugin names differently in params.** `syncai_planner`
  uses the pluginlib *lookup name* (`syncai_planner/NavfnPlanner`, declared via
  the `name=` attribute in its XML), while `syncai_controller` and
  `syncai_costmap_2d` use the fully-qualified *type*
  (`syncai_controller::RegulatedPurePursuitController`). Both are valid — the
  type always works, the lookup name only when the XML declares `name=` — but
  copying a params line between packages will not work unchanged.
- **`syncai_nav_core` depends on `syncai_costmap_2d`,** because three of the four
  interfaces take a `Costmap2DROS`. So "interfaces only" does not mean
  dependency-free — the costmap package has to build first.

Upstream reference: [`nav2_core`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_core).
