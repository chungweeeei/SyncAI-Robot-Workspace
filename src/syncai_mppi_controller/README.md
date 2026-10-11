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
