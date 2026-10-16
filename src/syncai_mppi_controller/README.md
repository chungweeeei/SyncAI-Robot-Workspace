# syncai_mppi_controller

The MPPI controller: a `syncai_nav_core::Controller` plugin that
`syncai_controller`'s `controller_server` loads next to Regulated Pure
Pursuit. It is a port of nav2's
[`nav2_mppi_controller`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller)
from the `humble` branch (1.1.20, commit `3c3db59`), brought in during 2026-10
to give the stack **local obstacle avoidance**. RPP has none: it follows the
path and refuses to move when the path is blocked.

```
controller_server (syncai_controller)          one process, one local_costmap
  ├─ FollowPath     → syncai_mppi_controller::MPPIController   ← this package
  └─ FollowPathRPP  → syncai_controller::RegulatedPurePursuitController
```

## Why this is a controller and not a "local planner"

Each cycle MPPI samples `batch_size` noisy control sequences (v, ω) around
its previous best. It rolls them out through the motion model, scores the
resulting trajectories with critics (obstacles, path, goal, …) and averages
the sequences, softmax-weighted by score. The first control of that average
**is** `cmd_vel`. The trajectory exists only to be scored. Passing it to RPP
to track would throw away MPPI's velocity profile and run two loops that keep
correcting each other.

So the stack has two planning layers, not three:

- **`syncai_global_planner`** picks a corridor (Smac2D, replanning once the
  path is blocked).
- **The controller** follows that corridor inside the 4×4 m local costmap.
  With MPPI it can bend around a blocker standing beside the path; with RPP it
  can only stop.

The re-route machinery in `move.xml` (`is_path_valid`, FollowPath retries) is
unchanged and works alongside MPPI:

- When the corridor itself is closed, the global replan handles it.
- When MPPI finds no feasible trajectory, it throws `PlannerException`
  ("Optimizer fail to compute path"). The server zeroes `cmd_vel` and the
  failure goes through `failure_tolerance` → the RecoveryNode retries → abort,
  exactly as an RPP refusal does.

## What changed from upstream

Everything under `src/critics/`, the optimizer, the motion models, the noise
generator, the path handler and `tools/utils.hpp` is upstream code with
renamed symbols only:

- `nav2_costmap_2d` → `syncai_costmap_2d`
- `nav2_core` → `syncai_nav_core`
- `nav2_util` → `syncai_util`
- `rclcpp_lifecycle::LifecycleNode` → `rclcpp::Node`

The internal namespace `mppi::` stays as upstream has it. The deliberate
differences are these:

| Change | Why |
|---|---|
| `configure()` + `activate()` → `initialize()`; no `cleanup()` / `deactivate()` | `syncai_nav_core` has no lifecycle. Publishers are live and the dynamic-parameter callback (`ParametersHandler::start()`) is registered before `initialize()` returns. The Optimizer's destructor still stops the noise generator's thread. |
| **Acceleration clamp** on the output (`max_linear_accel` 1.0, `max_angular_accel` 3.2) | Humble's MPPI bounds velocity only. `ax_max` and the other acceleration limits arrived upstream after Humble. This stack has no velocity smoother, and RPP clamps for the same reason. The reference is the last *command*, which is zeroed by `reset()` and by any throw (the same catch-all rule as RPP's), so the first cycle after a refusal ramps up from rest. |
| **Walking floor** (`walk_vx_min` 0.30; 2026-10-15) | RL does not walk below its floor either: on robot01 MPPI commanded 0.13 m/s and the robot stood, and MPPI slows into that band near every goal and blocker. A \|vx\| between `in_place_vx_max` and the floor is raised to the floor, sign kept; under `in_place_vx_max` (a stop, or an in-place turn) nothing changes. Applied to the optimizer's output **before** the acceleration clamp, because after it a stop from 0.30 would ramp into the band and be raised straight back — the robot could never stop. The ramp out of rest therefore still passes under the floor for ~0.3 s, as RPP's does. 0.30 is RPP's `min_approach_linear_velocity`. `walk_vx_min: 0` disables it. |
| **In-place turn floor** (`in_place_wz_min` 0.3, `in_place_wz_deadband` 0.05, `in_place_vx_max` 0.1; 2026-10-14) | RL does not turn in place below its floor, and MPPI tapers its turn into the goal yaw: on robot01 it held −0.115 rad/s with odom at zero until the progress checker aborted. With \|vx\| under `in_place_vx_max`, a \|wz\| between the deadband and the floor is raised to the floor, sign kept; while walking nothing changes. Applied after the acceleration clamp, so the first cycle of a turn out of rest steps straight to the floor. 0.3 is the gait's floor taken as the `cmd_vel` value, pre-scale (2026-10-15); it was 0.21 = 0.3 / `scale_turn` 1.40 for a day, on the never-confirmed assumption that the floor applies after the scale. `in_place_wz_min: 0` disables it. |
| **Final rotation** (`final_rotation_wz` 0.65; 2026-10-15) | Inside the goal checker's `xy_goal_tolerance` of the path's end, the optimizer is skipped and the robot turns in place to the goal heading — RPP's rotate-to-goal-heading with RPP's numbers: `wz` heads for `final_rotation_wz` at `max_angular_accel`, is capped at `sqrt(2·α·θ)` to stop on the heading and lifted to `in_place_wz_min` last; `vx` ramps to 0. Left to the optimizer, `GoalCritic` (the exact point) and `GoalAngleCritic` (the yaw) pulled against each other once the position was accepted: a small `vx` kept the turn floor off, `wz` went out at ~0.1 rad/s in jerks, and on robot01 the progress checker (a turn in place is no progress) aborted a goal the robot was standing on. Latched until the next `setPlan()` / `reset()`, the events that clear the stateful goal checker, so a robot that drifts while turning is not handed back. The cost is heading accuracy: the robot enters the `yaw_goal_tolerance` window at up to 0.65 rad/s instead of tapering into it. `final_rotation_wz: 0` disables it. |
| **Direct approach** (`approach_dist` 0.5, `approach_rotate_min_angle` 0.25; 2026-10-16) | Inside `approach_dist` of the path's end, the first cycle in which the optimizer asks to stand (raw \|vx\| under `in_place_vx_max`) hands the rest of the approach to RPP's law: turn in place onto the goal point (`rotateInPlace()`, the final rotation's turn) while its bearing is over `approach_rotate_min_angle`, else walk at `walk_vx_min` along the pure-pursuit arc through it, until the final rotation takes over at `xy_goal_tolerance`. On robot01 the robot stood 0.19 m from a goal 90° to its left, MPPI at vx 0.02–0.10 and odom zero, and the progress checker aborted it twice until it was cancelled: inside 0.5 m only `GoalCritic` still pulls, and its mean distance over the 2 s horizon pays a slow approach best (d·√2 / 2 s for a point d straight ahead), so the walking floor, which leaves a command under `in_place_vx_max` alone, never saw it. Opened on the optimizer's stall rather than on the distance, so every approach MPPI manages by itself is unchanged. The takeover gives up `CostCritic`'s avoidance, so the turn and the walk are checked against LETHAL footprints every cycle (RPP's `inCollision()` rule); a blocked way drops the latch and hands the robot back to the optimizer — the behaviour before this existed. Latched like the final rotation; cleared by it, by a block, and by `setPlan()` / `reset()`. Needs `walk_vx_min` and `final_rotation_wz` on; `approach_dist: 0` disables it. |
| `reset()` also clears the clamp baseline | The server calls it once per goal and after a cycle without a robot pose (`syncai_nav_core::Controller::reset()`). |
| New **`local_plan`** topic (`nav_msgs/Path`, optimal trajectory, every cycle with a subscriber) | This is the "local path" to show in rviz or the console. Upstream draws it only as markers, and only with `visualize: true`, which also publishes every candidate trajectory and costs real CPU. |
| `trajectories` is relative (upstream: `/trajectories`) | The robot_id convention: every topic inherits the namespace. |
| Libraries are `libsyncai_mppi_controller.so` / `libsyncai_mppi_critics.so` | So they can never be confused with an apt nav2's `libmppi_controller.so`. |
| No `test/` or `benchmark/` | Same as the other nav ports here: linters only. |

## Topics (all relative to `/<robot_id>`)

| Topic | Type | When |
|---|---|---|
| `local_plan` | `nav_msgs/Path` | every cycle with a subscriber; frame = local costmap's (`<robot_id>/odom`) |
| `transformed_global_plan` | `nav_msgs/Path` | `visualize: true` |
| `trajectories` | `visualization_msgs/MarkerArray` | `visualize: true` (candidates + optimal; heavy) |

## Parameters

These live under the controller id (`FollowPath.*`) in
`syncai_controller/params/controller_server_params.yaml`, which is where every
value and its rationale are recorded. The table lists what is tuned there and
how it differs from upstream's default.

| Parameter | Here | Upstream | Note |
|---|---|---|---|
| `motion_model` | `DiffDrive` | `DiffDrive` | The quadruped can strafe, but `syncai_driver_manager`'s `scale_left` / `scale_right` are unmeasured. Switch to `Omni` together with that calibration. |
| `vx_max` / `vx_min` | 0.60 / 0.0 | 0.5 / -0.35 | Same calibration as RPP's `desired_linear_vel` and the driver_manager scales. No reversing, as with RPP's `allow_reversing: false`. |
| `wz_max` | 0.65 | 1.9 | Same calibration as RPP's `rotate_to_heading_angular_vel`. |
| `time_steps` × `model_dt` | 40 × 0.05 = 2.0 s | 56 × 0.05 | Sized to the 4×4 m local costmap: 2.0 s at 0.60 m/s is 1.2 m, which leaves the footprint room inside the 2 m half-width. With `consider_footprint`, a rolled-out footprint that crosses the costmap edge reads as LETHAL (`footprintCost()` treats an off-map vertex as lethal), so a horizon longer than the window makes MPPI slow down for no obstacle. |
| `batch_size` | 1000 | 1000 | Measure CPU on the Orin before raising. |
| `model_dt` vs `controller_frequency` | 0.05 / 20 Hz | — | **Keep them equal.** `Optimizer::setOffset()` turns the warm-start shift on only when they match. It warns when the controller period is shorter than `model_dt`, and **throws** when it is longer, from inside `initialize()`, which takes `controller_server` down at startup. So lowering `controller_frequency` alone, for CPU, crashes the server; raise `model_dt` with it. |
| `max_linear_accel` / `max_angular_accel` | 1.0 / 3.2 | not in Humble | Added here, see above. |
| `walk_vx_min` | 0.30 | not upstream | The walking floor, see above. Same value as RPP's `min_approach_linear_velocity` / `regulated_linear_scaling_min_speed`; change them together. Its lower edge is `in_place_vx_max`. |
| `in_place_wz_min` / `in_place_wz_deadband` / `in_place_vx_max` | 0.3 / 0.05 / 0.1 | not upstream | The in-place turn floor, see above. Same value as RPP's `rotate_to_heading_min_angular_vel`; change them together. `in_place_vx_max` is also the walking floor's lower edge. |
| `final_rotation_wz` | 0.65 | not upstream | The final rotation, see above. Same value as RPP's `rotate_to_heading_angular_vel` and `wz_max`; with it on, `GoalAngleCritic` (whose `threshold_to_consider` equals `xy_goal_tolerance`) never scores. Also the direct approach's turn rate and the cap on its walking `wz`. |
| `approach_dist` / `approach_rotate_min_angle` | 0.5 / 0.25 | not upstream | The direct approach, see above. 0.5 is where `PathAlignCritic` / `PathAngleCritic` / `PreferForwardCritic` stop scoring; 0.25 is RPP's `rotate_to_heading_min_angle`, change them together. The walk ends at `xy_goal_tolerance`. |
| `critics` | Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle, PreferForward | — | `CostCritic.consider_footprint: true`, because the footprint is a rectangle. Weights start from upstream's README example. |

`CostCritic` reads `cost_scaling_factor` / `inflation_radius` from the local
costmap's `InflationLayer` (`inflation_layer_name` empty = the first one), so
those two stay in the costmap block and are not repeated here.

## Gotchas

- **Two plugins, one costmap mutex.** Both controllers lock the local costmap
  for the whole of a cycle. Only the controller named in the FollowPath goal
  runs, so loading both costs memory, not CPU.
- **`ParametersHandler` warns "Parameter … not found"** on every dynamic
  parameter set on the server that is not one of MPPI's (for example
  `FollowPathRPP.*` or `failure_tolerance`). Upstream behaves the same way, and
  the set still succeeds.
- **Build flags.** The x86 flags in `CMakeLists.txt` (`-mavx2` …) are tested
  with `check_cxx_compiler_flag` and drop out on arm64. The whole package is
  built `-O3 -ffast-math` as upstream builds it. It is deliberately not
  `-march=native`, because `install/` is shared between the build container
  and the robot.
- Needs `libxtensor-dev` + `libxsimd-dev` (the `Dockerfile`'s dev stage).
