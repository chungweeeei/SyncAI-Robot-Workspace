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
  ├─ FollowPathRPP  → syncai_controller::RegulatedPurePursuitController
  └─ FollowPathRPP3D → syncai_controller::RegulatedPurePursuitController3D
                       (feature/octomap-planner-3d only)
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

This describes the 2D tree (`move.xml`, as on `dev`). On
`feature/octomap-planner-3d` a 3D map runs `move_3d.xml` instead, which names
`Octo3D` and `FollowPathRPP3D` and leaves `is_path_valid` out, so MPPI is
unused on such a map, and a blocker gets neither a local bend nor a global
replan: RPP 3D stops, and the FollowPath retries are the wait.

The re-route machinery in the 2D `move.xml` (`is_path_valid`, FollowPath
retries) is unchanged and works alongside MPPI:

- When the corridor itself is closed, the global replan handles it.
- When MPPI finds no feasible trajectory, it throws `PlannerException`
  ("Optimizer fail to compute path"; Humble's `std::runtime_error` until
  2026-10-21, see below). The server zeroes `cmd_vel` and the failure goes
  through `failure_tolerance` → the RecoveryNode retries → abort, exactly as
  an RPP refusal does.

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
| **In-place turn floor** (`in_place_wz_min` 0.45, `in_place_wz_deadband` 0.05, `in_place_vx_max` 0.1; 2026-10-14) | RL does not turn in place below its floor, and MPPI tapers its turn into the goal yaw: on robot01 it held −0.115 rad/s with odom at zero until the progress checker aborted. With \|vx\| under `in_place_vx_max`, a \|wz\| between the deadband and the floor is raised to the floor, sign kept; while walking nothing changes. Applied after the acceleration clamp, so the first cycle of a turn out of rest steps straight to the floor. The value is the `cmd_vel` value, pre-scale. 0.3 from 2026-10-15 (0.21 = 0.3 / `scale_turn` 1.40 for a day before that) proved not to be above the gait's floor: on robot01 (2026-10-21) MPPI held vx 0.02–0.07 with wz ±0.30 in front of a blocker for ~9 s, odom turned at 0–0.16 rad/s, and the optimizer swung between passing left and right until the 10 s replan, while 0.65 rad/s turns ran at 0.5–0.9. 0.45 since, between the two; the floor itself is unmeasured. `in_place_wz_min: 0` disables it. |
| **Final rotation** (`final_rotation_wz` 0.65; 2026-10-15) | Inside the goal checker's `xy_goal_tolerance` of the path's end, the optimizer is skipped and the robot turns in place to the goal heading — RPP's rotate-to-goal-heading with RPP's numbers: `wz` heads for `final_rotation_wz` at `max_angular_accel`, is capped at `sqrt(2·α·θ)` to stop on the heading and lifted to `in_place_wz_min` last; `vx` ramps to 0. Left to the optimizer, `GoalCritic` (the exact point) and `GoalAngleCritic` (the yaw) pulled against each other once the position was accepted: a small `vx` kept the turn floor off, `wz` went out at ~0.1 rad/s in jerks, and on robot01 the progress checker (a turn in place is no progress) aborted a goal the robot was standing on. Latched until the next `setPlan()` / `reset()`, the events that clear the stateful goal checker, so a robot that drifts while turning is not handed back. The cost is heading accuracy: the robot enters the `yaw_goal_tolerance` window at up to 0.65 rad/s instead of tapering into it. `final_rotation_wz: 0` disables it. |
| **Direct approach** (`approach_dist` 0.5, `approach_rotate_min_angle` 0.25; 2026-10-16) | Inside `approach_dist` of the path's end, the first cycle in which the optimizer asks to stand (raw \|vx\| under `in_place_vx_max`) hands the rest of the approach to RPP's law: turn in place onto the goal point (`rotateInPlace()`, the final rotation's turn) while its bearing is over `approach_rotate_min_angle`, else walk at `walk_vx_min` along the pure-pursuit arc through it, until the final rotation takes over at `xy_goal_tolerance`. On robot01 the robot stood 0.19 m from a goal 90° to its left, MPPI at vx 0.02–0.10 and odom zero, and the progress checker aborted it twice until it was cancelled: inside 0.5 m only `GoalCritic` still pulls, and its mean distance over the 2 s horizon pays a slow approach best (d·√2 / 2 s for a point d straight ahead), so the walking floor, which leaves a command under `in_place_vx_max` alone, never saw it. Opened on the optimizer's stall rather than on the distance, so every approach MPPI manages by itself is unchanged. The takeover gives up `CostCritic`'s avoidance, so the turn and the walk are checked against LETHAL footprints every cycle (RPP's `inCollision()` rule); a blocked way drops the latch and hands the robot back to the optimizer — the behaviour before this existed. Latched like the final rotation; cleared by it, by a block, and by `setPlan()` / `reset()`. Needs `walk_vx_min` and `final_rotation_wz` on; `approach_dist: 0` disables it. |
| **3D path matching** (2026-10-20) | A path whose poses do not all share one z — OctomapPlanner3D's, with its stair links — is matched to the robot in 3D by the `PathHandler`: the robot's z is the ground under it (`syncai_util/path_height.hpp`'s `lookupGroundZ`: the server's `body_frame` z minus `lidar_height`, since `base_link` is planar), the nearest-pose search counts height, and the poses handed to the optimizer have z = 0 again. On a spiral stair the turn 2.5 m above the robot is as close on x / y as the one it stands on and the 10 m search window holds both; RPP 3D had the same fix first. A failed lookup throws the `PlannerException` the server tolerates for `failure_tolerance` (the file's other throws end the goal). A path without heights — every 2D planner — takes the planar code path unchanged, which is what lets one controller serve a 2D map and a 3D one. |
| **Stair mode** (`stair_linear_vel` 0.5, `stair_lookahead_dist` 0.4 and the `use_stair_mode` … `flat_policy_mode` block; 2026-10-20) | RPP 3D's stair mode in MPPI, so `move_3d.xml` names `FollowPath` as `move.xml` does. Detection, the policy switch over `set_policy_mode` and the latched `stair_mode` report are `syncai_util::StairMode`, one state machine shared with `FollowPathRPP3D` and declared under this plugin's name with the same values (`syncai_controller`'s README "Stairs" has the numbers). `climbStairs()` runs first in every cycle: while the switch is pending it brakes with the acceleration clamp and tells the state machine when the robot stands; on the flight it skips the optimizer and tracks the first pose of the pruned path `stair_lookahead_dist` out at `stair_linear_vel` with the direct approach's law (`rotateInPlace()` past `approach_rotate_min_angle`, else `trackPoint()`, the walk the approach uses too) — **without reading the costmap**: no obstacle avoidance on a flight, by decision. On a 3D map the local costmap runs the terrain layer, which judges a stair lethal (riser over `max_step_height`, flight over `max_slope_deg`) and opens nothing along the path, so neither the optimizer nor a footprint check could take a step onto one; that lethal is also what keeps the optimizer off a stair the robot was not routed over. Blind from the hold at the stair's foot (`stair_enter_dist`) to the hold past its top (`stair_exit_behind_dist`), which leaves the footprint clear of the flight's lethal cells at the hand-back. The optimizer is reset on the way in and out. A path without heights never enters it (a robot left in stair mode by a cancelled goal is taken back out by the next level plan). Untested on the robot, as RPP 3D's is. |
| **The optimizer's failure is a `PlannerException`** (`Optimizer::fallback()`; 2026-10-21) | When every candidate collides, Humble throws `std::runtime_error("Optimizer fail to compute path")`. `controller_server` applies `failure_tolerance` to `PlannerException` only, so that ended FollowPath on the first such cycle. On robot01 that was three aborts in 4.4 s beside one blocker, each spending one of `move.xml`'s retries and clearing the local costmap, while this README, the server's and the params file already described the 3 s window. Now the cycle publishes zero `cmd_vel` with a reset optimizer, and the goal ends only if no cycle succeeds for `failure_tolerance`. The optimizer's other throw, `setOffset()`'s at `initialize()`, stays a `runtime_error`. |
| **Path points are judged by the footprint** (`utils::findPathCosts()`; 2026-10-21) | `path_pts_valid` decides when `PathAlignCritic` stands down (`max_path_occupancy_ratio`) and which points `PathFollowCritic`'s carrot skips. Upstream flags a point only when its centre cell is INSCRIBED or worse, i.e. with an obstacle within the inscribed radius (0.23 m) of the path's centre line. A blocker 0.25–0.45 m beside the path (the planner's `is_path_valid` logs it as "footprint perimeter on LETHAL (centre cost ~185)") left every point valid. On `feature/octomap-planner-3d`, where `PathAlignCritic` scores from 5 path points (here it waits for 20, which on a 2D map only a cruise on a straight reaches), it then held every candidate on a line the footprint could not drive, and robot01 walked up to three such blockers and stopped beside each until `IsPathValid`'s 10 s replan or a FollowPath abort moved it on; with this change it went round three blockers of four by itself, and the fourth stalled on the in-place turn floor (see that row). A point is now also invalid when the padded footprint on it, facing along the path (the bearing to the next point, not the pose orientation), touches LETHAL, or unknown while unknown is not tracked: `CostCritic`'s collision rule for a candidate that tracks the path. There is no heading margin, so a door the robot fits through straight stays valid. Points within the circumscribed radius of the window's edge keep the centre test, because `footprintCost()` reads an off-map vertex as LETHAL. Skipped when the costmap uses a radius. |
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
| `stair_mode` | `std_msgs/Bool`, latched | the stair state machine's report (`syncai_util::StairMode`); `FollowPathRPP3D` publishes the same topic from its own instance |

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
| `in_place_wz_min` / `in_place_wz_deadband` / `in_place_vx_max` | 0.45 / 0.05 / 0.1 | not upstream | The in-place turn floor, see above (0.3 until 2026-10-21). Same value as RPP's `rotate_to_heading_min_angular_vel`; change them together. `in_place_vx_max` is also the walking floor's lower edge. |
| `final_rotation_wz` | 0.65 | not upstream | The final rotation, see above. Same value as RPP's `rotate_to_heading_angular_vel` and `wz_max`; with it on, `GoalAngleCritic` (whose `threshold_to_consider` equals `xy_goal_tolerance`) never scores. Also the direct approach's turn rate and the cap on its walking `wz`. |
| `approach_dist` / `approach_rotate_min_angle` | 0.5 / 0.25 | not upstream | The direct approach, see above. 0.5 is where `PathAlignCritic` / `PathAngleCritic` / `PreferForwardCritic` stop scoring; 0.25 is RPP's `rotate_to_heading_min_angle`, change them together. The walk ends at `xy_goal_tolerance`. |
| `use_stair_mode` … `flat_policy_mode` | `FollowPathRPP3D`'s values | not upstream | `syncai_util::StairMode`'s parameters, see above; one calibration with RPP 3D's block, change them together. |
| `stair_linear_vel` / `stair_lookahead_dist` | 0.5 / 0.4 | not upstream | The climb: the carrot's distance and the speed on the flight. Needs `final_rotation_wz` on (the turn onto the carrot). |
| `critics` | Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle, PreferForward | — | `CostCritic.consider_footprint: true`, because the footprint is a rectangle. Weights start from upstream's README example. |

`CostCritic` reads `cost_scaling_factor` / `inflation_radius` from the local
costmap's `InflationLayer` (`inflation_layer_name` empty = the first one), so
those two stay in the costmap block and are not repeated here. The radius must
be at least the footprint's circumscribed radius (0.427 m): the critic checks
the footprint only where the centre's cell has a cost, so a smaller band lets
a corner clip an obstacle the centre is just outside of. 0.45 since 2026-10-21
(0.4 before, which had that hole); the price is that MPPI stops in front of
doors narrower than ~1.0 m rather than ~0.9 m, accepted as obstacle avoidance
over narrow doors.

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
