# syncai_controller

The path-following half of the nav stack: a `FollowPath` action server that owns
a rolling local costmap and drives a pluggable controller at a fixed rate. Ported
from `nav2_controller`, with `nav2_regulated_pure_pursuit_controller` merged in
as a plugin of this package.

```
syncai_task_runner ──FollowPath (nav2_msgs)──►  controller_server
                                                   │
                          ┌────────────────────────┼────────────────────────┐
                          ▼                        ▼                        ▼
                  progress checker           goal checker            Controller plugin
              "still moving?"           "close enough?"        Regulated Pure Pursuit
                          └────────────────────────┼────────────────────────┘
                                                   │  local_costmap (internal node)
                                                   ▼
                                       /<robot_id>/cmd_vel  ──►  syncai_driver_manager
```

Two structural differences from nav2 shape everything below:

1. **`ControllerServer` is a plain `rclcpp::Node`,** not a lifecycle node. Setup
   is split into a constructor (declare parameters, construct the costmap object)
   and `configure()` (everything needing `shared_from_this()`), called from
   `main.cpp` right after the node is owned by a `shared_ptr`. nav2's
   `on_configure` **and** `on_activate` are merged into that one call, so the
   server is live the moment it is constructed.
2. **There is no velocity smoother in this stack.** `cmd_vel` goes straight from
   here to `syncai_driver_manager`, so the controller clamps its own linear and
   angular acceleration. See "Acceleration clamping" — it is the single most
   load-bearing local modification to RPP.

## Layout

```
src/controller_server.cpp        the action server + control loop
src/main.cpp                     node construction + configure() + spin
plugins/
  regulated_pure_pursuit_controller/   the controller (911 lines)
  simple_progress_checker.cpp          ProgressChecker
  pose_progress_checker.cpp            ProgressChecker (adds rotation)
  simple_goal_checker.cpp              GoalChecker
  stopped_goal_checker.cpp             GoalChecker (adds "must be stopped")
  position_goal_checker.cpp            GoalChecker (position only)
plugins.xml                      pluginlib manifest, exported to syncai_nav_core
params/controller_server_params.yaml       2D / sim
params/controller_server_3d_params.yaml    3D / real robot
launch/controller_server.launch.py
```

The build produces **two** shared libraries: `libsyncai_controller.so` (the
server, linked by `controller_server`) and `libsyncai_controller_plugins.so`
(everything under `plugins/`, loaded at runtime by pluginlib). Interfaces come
from `syncai_nav_core`, which is also the pluginlib *package* name every class
registers against.

## The control loop

`computeControl()` runs on the `SimpleActionServer`'s own spin thread (the
server is created with `spin_thread=true`), so the blocking loop below never
starves the node's main executor, which handles odom, speed-limit and parameter
callbacks.

```
findControllerId / findGoalCheckerId    ← from the goal, or the sole plugin if unset
setPlannerPath(goal.path)               ← controller->setPlan, cache end_pose_, reset goal checker
progress_checker_->reset()

loop at controller_frequency:
    server inactive?            → return
    cancel requested?           → terminate_all + zero velocity, return
    wait until costmap_ros_->isCurrent()      ← spin at 100 Hz; a clear-costmap
                                                 makes the costmap non-current
    updateGlobalPath()          ← accept a preempting goal in place, no restart
    computeAndPublishVelocity()
    isGoalReached()?            → break

publishZeroVelocity() (if publish_zero_velocity)
succeeded_current()
```

`computeAndPublishVelocity()` does the real work: get the robot pose from the
costmap, run the progress checker (throws `PlannerException` on "Failed to make
progress"), threshold the odom twist, call
`controller->computeVelocityCommands(pose, twist, goal_checker)`, publish
feedback (`speed` and `distance_to_goal`, the latter measured along the remaining
path from the closest pose), and publish the twist.

**`failure_tolerance`** softens controller exceptions: with a positive value a
failed cycle publishes zero velocity and keeps trying until that many seconds
have passed since the last *valid* command, then gives up. `-1.0` retries
forever, `0.0` fails on the first exception.

**`publishVelocity` skips publishing when nothing is subscribed** to `cmd_vel` —
worth knowing when debugging with `ros2 topic echo`, since attaching the echo
changes whether messages are sent at all.

### Goal checking is gated on path progress

`isGoalReached()` differs from nav2 and the difference matters. nav2 asks the
goal checker whether the robot is near `end_pose_`; on a **patrol loop whose last
waypoint is its own start**, that is true on the very first control cycle and the
goal "succeeds" in about a second without the robot moving.

So this port first computes the remaining path length from the path pose closest
to the robot, and only consults the goal checker once that drops below
**`goal_reached_max_remaining_path`** (default 1.0 m; `<= 0` disables the gate).
The closest-pose search uses a strict `<` so that on a closed loop a robot sitting
on the shared start/end point resolves to the *start* index, leaving the full loop
length remaining.

A second fix lives here too: `end_pose_` is re-stamped with `Time(0)` before being
transformed into the costmap's global frame. Using the path's original stamp made
long goals die with a TF "extrapolation into the past" error, so `isGoalReached()`
never returned true.

## Regulated Pure Pursuit

Standard pure pursuit picks a lookahead point ("carrot") on the path, fits a
circular arc to it, and commands `v` and `ω = v · curvature`. The *regulated*
variant scales `v` down for three reasons before that. Each control cycle:

1. **Transform and prune** the global plan into the base frame
   (`transformGlobalPlan`), discarding poses outside the local costmap and
   erasing everything already passed. This is the only pruning in the pipeline.
2. **Pick the lookahead distance** — either the fixed `lookahead_dist`, or
   `|v| · lookahead_time` clamped to `[min_lookahead_dist, max_lookahead_dist]`
   when `use_velocity_scaled_lookahead_dist` is on.
3. **Find the carrot** — the first pose beyond the lookahead distance, refined by
   a circle/segment intersection when `use_interpolation` is on.
4. **Branch** on one of three behaviours:
   - carrot closer than the goal tolerance → **rotate to goal heading**;
   - carrot more than `rotate_to_heading_min_angle` off the nose → **rotate to
     path heading**;
   - otherwise → **normal tracking** through `applyConstraints`.
5. **Clamp linear acceleration** (see below), then recompute
   `ω = v · curvature` from the clamped `v` so the tracked arc keeps its shape.
6. **Forward-simulate the resulting arc** on the costmap
   (`isCollisionImminent`) up to `max_allowed_time_to_collision_up_to_carrot`; a
   predicted footprint collision throws `PlannerException("…collision ahead!")`.

`applyConstraints` is the "regulated" part — three independent speed limits, the
strictest wins:

| Regulator | Effect | Off switch |
|---|---|---|
| **Curvature** | Below `regulated_linear_scaling_min_radius`, scale linearly with turn radius — slow into corners | `use_regulated_linear_velocity_scaling` |
| **Obstacle proximity** | Invert the inflation-layer cost to a distance; below `cost_scaling_dist`, scale down | `use_cost_regulated_linear_velocity_scaling` |
| **Approach** | Within `approach_velocity_scaling_dist` of the path end, scale linearly toward `min_approach_linear_velocity` | — (always on) |

The first two are floored at `regulated_linear_scaling_min_speed` *before* the
approach scaling is applied, so the approach ramp can still bring the robot below
that floor at the goal.

Debug publishers, all namespaced under the plugin's node: `received_global_plan`
(the pruned, transformed plan), `lookahead_point` (the carrot), and
`lookahead_collision_arc` (the forward-simulated arc).

### Acceleration clamping — the local modification

Both clamps use **the controller's own previous command** as their baseline, not
the measured odom twist:

```cpp
min_feasible = last_cmd_vel_.linear.x - max_linear_accel_ * control_duration_;
max_feasible = last_cmd_vel_.linear.x + max_linear_accel_ * control_duration_;
linear_vel   = std::clamp(linear_vel, min_feasible, max_feasible);
```

The reason is the quadruped. With no `OdomSmoother` in the stack, the measured
twist is the raw Point-LIO output, which carries the gait's body sway
(±0.2–0.5 m/s). Clamping against that measurement produced a positive feedback
loop — command oscillates → robot sways harder → measurement oscillates harder —
and with `allow_reversing: false` it could push the command negative. Clamping
against the last command keeps the *command trajectory* kinematically feasible
without coupling to gait noise.

`last_cmd_vel_` is zeroed in `setPlan()`, so every new goal ramps from a
standstill. `rotateToHeading()` applies the same pattern with `max_angular_accel`,
plus a `sqrt(2·α·θ)` cap so the in-place rotation decelerates into its target
instead of overshooting.

## Plugins

Three plugin types, all loaded through pluginlib against `syncai_nav_core`
interfaces. Selection is by ID: the parameter names an ID, and `<id>.plugin`
names the C++ type.

**Controllers** (`syncai_nav_core::Controller`, parameter `controller_plugins`) —
one entry only: `RegulatedPurePursuitController`. The IDs are what a `FollowPath`
goal's `controller_id` field selects; with a single plugin loaded, an empty
`controller_id` picks it and warns once.

**Progress checkers** (`syncai_nav_core::ProgressChecker`, parameter
`progress_checker_plugin` — singular, only one is loaded):

| Plugin | Fails when |
|---|---|
| `SimpleProgressChecker` | The robot has not moved `required_movement_radius` metres within `movement_time_allowance` seconds |
| `PoseProgressChecker` | Same, but rotating by `required_movement_angle` also counts as progress — useful when a long in-place rotation would otherwise trip the checker |

**Goal checkers** (`syncai_nav_core::GoalChecker`, parameter
`goal_checker_plugins` — a list; a `FollowPath` goal's `goal_checker_id` selects
one):

| Plugin | Checks |
|---|---|
| `SimpleGoalChecker` | xy distance, then yaw. `stateful: true` latches the xy check once satisfied, so the final in-place rotation cannot fail it by drifting. `symmetric_yaw_tolerance` also accepts goal yaw + 180° for symmetric robots. |
| `StoppedGoalChecker` | `SimpleGoalChecker` **and** speed below `trans_stopped_velocity` / `rot_stopped_velocity` |
| `PositionGoalChecker` | Position only — orientation ignored entirely |

Only `SimpleGoalChecker` is configured today. Note the coupling to RPP:
`getTolerances()` feeds `goal_dist_tol_`, which is what decides when RPP switches
into rotate-to-goal-heading — so widening `xy_goal_tolerance` also makes the robot
start its final rotation earlier.

## Parameters

Server-level (`/**/controller_server`):

| Parameter | Default | Notes |
|---|---|---|
| `controller_frequency` | `20.0` | Control loop rate; also `1/f` is RPP's `control_duration_`, the `dt` in both accel clamps |
| `min_x_velocity_threshold` | `0.0001` | Odom twist below this reads as zero |
| `min_y_velocity_threshold` | `0.0001` | Config sets `0.5` — a differential/quadruped base has no meaningful lateral velocity, so this discards it |
| `min_theta_velocity_threshold` | `0.0001` | |
| `failure_tolerance` | `0.0` | Seconds to tolerate controller exceptions; `-1.0` = forever |
| `publish_zero_velocity` | `true` | Send one stop command on success |
| `goal_reached_max_remaining_path` | `1.0` | The patrol-loop gate above; `<= 0` disables |
| `speed_limit_topic` | `speed_limit` | `nav2_msgs/SpeedLimit`, forwarded to every controller's `setSpeedLimit()` |
| `progress_checker_plugin` / `goal_checker_plugins` / `controller_plugins` | see above | |

`controller_frequency`, the three velocity thresholds, `failure_tolerance` and
`goal_reached_max_remaining_path` are **dynamically reconfigurable** — but the
callback takes the same mutex the control loop holds, so a change is rejected
with a warning while a goal is executing. Plugin parameters (anything with a `.`
in the name) are skipped by that lock and handled by each plugin's own callback.

The tuned RPP values live in the params files with their rationale attached; a
few worth reading before touching:

- `min_lookahead_dist: 0.5` — raised from 0.3 because pure-pursuit steering gain
  scales with `1/lookahead²`, so a short low-speed lookahead made startup weave.
- `regulated_linear_scaling_min_speed: 0.3` — raised to stop the robot crawling,
  but it overrides curvature regulation right where the arc is sharpest (just
  after rotate-to-heading exits). Watch for corner-cutting.
- `rotate_to_heading_min_angle: 0.25` — lowered from 45°, which used to hand over
  to path tracking so far off-heading that the robot drove away and re-triggered
  the rotation.
- `max_linear_accel: 1.0` — the no-velocity-smoother clamp; 0 → 0.8 m/s in ~0.8 s.

### 2D vs 3D params

| | `controller_server_params.yaml` | `controller_server_3d_params.yaml` |
|---|---|---|
| `use_sim_time` | `true` (Isaac Sim) | `false` (real robot, no `/clock`) |
| `xy_goal_tolerance` | `0.05` | `0.1` — the real robot's floor speed is ~0.3 m/s, so it cannot creep into a 5 cm window |
| Local costmap sources | `scan` | `scan` **and** `pointlio/body_cloud` |

On the real 3D robot the `scan` source receives nothing (no scan merger runs
there); it is kept for setups that do publish one. The pointcloud source
deliberately leaves `sensor_frame` empty so the observation buffer uses the
cloud's own header frame as the raytrace origin — meaning it needs no per-robot
launch override, but also that it only works **after** `/localizer/relocalize`
has established the TF chain.

## Running

```bash
ros2 launch syncai_controller controller_server.launch.py
ros2 launch syncai_controller controller_server.launch.py \
    params_file:=$PWD/src/syncai_controller/params/controller_server_3d_params.yaml
ros2 launch syncai_controller controller_server.launch.py \
    system_config:=config/instances/robot02.ini
```

Both byobu sessions start it in the `plan_ctrl` window after `sleep 4`; the
costmap needs TF and the sensor topics before it can go current.

The launch file does two namespace-related things, both load-bearing:

- **No `name=` on the `Node`.** The process hosts *two* nodes —
  `controller_server` and its internal `local_costmap` — and a launch-level name
  would remap both to the same name, silently stripping the costmap of every
  parameter. The params file uses `/**/` wildcard keys so both match at any
  namespace.
- **TF frames are overridden explicitly** (`global_frame`, `robot_base_frame`,
  `obstacle_layer.scan.sensor_frame`) with the `<robot_id>/` prefix, because ROS
  namespaces topics but not frame ids. These land under the `/**` wildcard and so
  reach the costmap node; `controller_server` ignores them as undeclared.

`use_sim_time` is deliberately **not** set in the launch file — a launch override
placed after the params file would silently win over the YAML value.

Testing it directly, without the task runner:

```bash
ros2 action send_goal /<robot_id>/follow_path nav2_msgs/action/FollowPath \
  "{path: {header: {frame_id: map}, poses: [...]}, controller_id: FollowPath}" --feedback
ros2 topic echo /<robot_id>/cmd_vel
ros2 topic echo /<robot_id>/lookahead_point      # is the carrot where you expect?
```

## Gotchas

- **Nothing smooths `cmd_vel`.** `max_linear_accel` / `max_angular_accel` are the
  only thing between the controller and the gait controller. Raising
  `desired_linear_vel` without checking them gives a step command.
- **The accel clamp baseline is the last command, not measured speed** — so if
  the robot physically fails to track the command, the clamp will not notice and
  will keep ramping. That is intentional (gait noise), but it means the clamp is
  not a safety feature.
- **Footprints must match the global costmap.** `syncai_planner`'s
  `global_costmap` uses the same rectangle; if they diverge, RPP rejects paths
  the planner considers valid, spamming "collision ahead!".
- **`isCurrent()` can hang the loop.** The `while (!costmap_ros_->isCurrent())`
  spin has no timeout: if an observation source stops publishing, the control
  loop stalls there with the goal still active rather than failing.
- **A plugin load failure calls `exit(-1)`.** A typo in a `.plugin` type string
  takes the whole process down at startup rather than degrading.
- **The byobu session launches this server on its default params file** —
  `config/sessions/stack.yaml` passes no `params_file:=` override, for either
  the planner or the controller. That is correct now that the separate
  `*_3d_params.yaml` variants have been merged back into the defaults; it was a
  bug while they were separate (the 3D session silently ran with
  `use_sim_time: true` and no pointcloud observation source).
- **Dynamic parameter changes are refused mid-goal**, with a warning rather than
  an error. Set them while idle.

Upstream references: [`nav2_controller`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_controller)
and [`nav2_regulated_pure_pursuit_controller`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller),
whose README has the full RPP algorithm write-up and diagrams.
