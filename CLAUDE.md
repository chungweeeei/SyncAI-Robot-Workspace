# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A ROS 2 Humble software stack for the SyncAI robot (G23 quadruped / AMR chassis):
sensor drivers, LIO odometry, a **non-lifecycle port of Navigation2**, and the
byobu session manager that brings them up. The operator-facing half — the
Temporal-backed task orchestration API and the console it serves — is two other
repositories, and what is left here is one side of a contract; see "Out of
tree".

Three things shape almost every decision here:

1. **No lifecycle nodes.** The nav2 servers (map server, costmap, planner,
   controller, BT navigator — and originally AMCL, since replaced by FAST-LIO2
   localization) were re-implemented as plain `rclcpp::Node`s. There is no
   lifecycle manager — every node comes up active. This means **startup order
   matters** and is handled by `sleep` offsets in the byobu session specs
   (`config/sessions/*.yaml`).
2. **Everything is namespaced by `robot_id`.** A single DDS domain can host
   several robots, so node namespaces, topics, and TF frames are all prefixed.
   See "The robot_id convention" below — this is the single most common source of
   mistakes when editing launch files or params.
3. **The operator API is not in this repo.** `syncai_backend` moved to
   `chungweeeei/SyncAI-Robot-Backend` in 2026-09 and `syncai_frontend` to
   `chungweeeei/SyncAI-Robot-Frontend` just before it. Every REST route, WebSocket and 409 code named in this file is
   documentation of a *consumer* of this stack, not of code you can edit here —
   and changing a service, topic, message or map-directory layout the backend
   calls is a cross-repository change. See "Out of tree" for what that means in
   practice.

`README.md` covers clone/build/run from a user's perspective. This file covers
the conventions and gotchas you need to edit the code correctly. Every package
under `src/` also carries its own `README.md` with its parameter tables, topic
names and package-local gotchas — read it before editing that package, and
update it in the same change when you move a parameter or a topic.

## Package map

### Navigation (C++, nav2 port)

```
NavigateToPose (nav2_msgs) / NavigateToGoal (syncai_common) → syncai_task_runner   (BT navigator; ticks behavior_trees/move.xml)
                                   │
                compute_path_to_pose│follow_path
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
          syncai_global_planner                    syncai_controller
            (NavFn, pluginlib)           (MPPI / RPP plugins)
                    │                              │
                    └────── syncai_costmap_2d ─────┘
                       (global costmap / local costmap,
                        layered plugins + keepout filter)
```

| Package | Role |
|---|---|
| `syncai_nav_core` | Header-only abstract plugin interfaces (port of `nav2_core`) |
| `syncai_util` | Shared helper library (geometry, occupancy-grid values, odometry window, `SimpleActionServer`, node / robot utils, and since 2026-10-20 `path_height.hpp` — `pathHasHeight` / `lookupGroundZ`, moved here from `syncai_controller` — and `StairMode`, the stair state machine both controllers run; it depends on `syncai_common` for that, so the interface checkout is needed to build anything above it) — a `SHARED` target, not header-only, so a change here rebuilds every nav package. `test/test_stair_mode.cpp` (gtest, ROS-free through `configure()`'s callbacks) is its first test |
| `syncai_common` | Shared msg / srv / action interfaces (`RobotState`, `SetMotionKey`, `ExecuteTask`, `ResetLIO`, `SaveMaps`, `ResetMapping`, `RefineMap`, `SavePoses`, `Relocalize`, `IsValid`, …). The last seven came over from the FASTLIO2_ROS2 fork's `interface` package, which no longer exists, with the pointlio / pgo / hba ports: `SaveMaps` / `ResetMapping` are served by `syncai_mapping` (`RefineMap` / `SavePoses` were `hba_node`'s, and have no server since it was removed in 2026-10), `ResetLIO` by `syncai_pointlio`, and `Relocalize` / `IsValid` by `syncai_localizer`. Every interface the backend calls is here now, and every server of one of those is in this workspace. `StartMapping` (srv) and `MappingStatus` (msg) are the exception to the fork story: they are new in 2026-10, for `syncai_mapping`'s run lifecycle, and have no counterpart in the fork's `interface`. So is `NavigateToGoal` (action, 2026-10): `nav2_msgs/NavigateToPose` with `error_code` / `error_msg` in the result (`PLAN_FAILED` 200, `FOLLOW_PATH_FAILED` 100, `UNKNOWN` 1, nav2 Iron's ranges) and `number_of_replans` / `status_msg` (one English sentence of what the navigator is doing — planning, following, replanning because the path is blocked, retrying) in the feedback, served by `syncai_task_runner` next to the unchanged nav2 action — the first interface here to use `geometry_msgs`. `DockToPose` (action, 2026-10) is the third: the last metre onto or off a dock, served by `syncai_controller`'s `controller_server` on `dock_to_pose` and called only by the `DockToPose` BT node from `dock.xml` / `undock.xml`; its 3xx codes (`TRANSFORM_FAILED` 301 … `INVALID_PLUGIN` 305) sit inside `NavigateToGoal`'s new `DOCK_FAILED=300` range, and direction is the `controller_id` (`Docking` / `Undocking`), not a goal field. Its goal pose is **planar** since 2026-10-14 — `frame_id` (empty → `map`) / `x` / `y` / `yaw` instead of a `PoseStamped` — because a hand-written quaternion is what a CLI test goal gets wrong; the BT node converts the trees' `PoseStamped`. |
| `syncai_costmap_2d` | Costmaps with layered plugins (static / obstacle / terrain / inflation / keepout filter). **`TerrainLayer`** (2026-10-20) is the local costmap's sensor source **on a 3D map**, in place of the obstacle layer, which stays the source on a 2D map — both are loaded and exactly one runs, by each layer's `map_type` (`"2d"` / `"3d"`; `"any"`, the default, for every other use) against the planner's latched `planner_server/Octo3D/map_available` (`MapTypeGate`), the signal the task_runner picks its trees by, so a map switch switches the layer with no restart; never both, since they disagree on a ramp. It is a rolling 2.5D grid of the lidar's returns (`TerrainGrid`, ROS-free, `test/test_terrain_grid.cpp`) with a ground height per cell (the 0.25 quantile of 10 s of returns — the MID360's −7° lower FOV never sees the floor within ~3.9 m, so the floor under the robot was seen seconds ago from further off; the robot's own ground where nothing was), judged with Octo3D's `SurfaceMapParams` values: a return 0.1–0.6 m above its cell's ground is an obstacle for 2 s (a box, a person, a wall foot — the obstacle layer's 0.58 m band saw none of the low ones), a surface the robot cannot reach over ≤ 0.17 m steps from its own cell is an island (a box top seen without its floor), a neighbour more than a step away is a ledge or a drop-off, a disc steeper than 20° a slope; a ramp and a kerb are free, and a **stair is lethal, with nothing opening it** — the controllers' stair mode climbs a flight without reading the costmap (no obstacle avoidance on stairs, decided 2026-10-20; a same-day draft's corridor along the climbing path was removed). The planner's `plan` (Octo3D's heights) only seeds the reach flood, so a ramp whose foot was never seen is still floor. A costmap clear drops the costs and keeps the returns. Unlike upstream, `KeepoutFilter` **inflates the mask itself** (2026-10) with the costmap's inscribed radius and its `InflationLayer`'s radius / decay, because filters run after inflation — so a mask is the forbidden area only, and a zone closes a gap narrower than the robot exactly as a wall does. |
| `syncai_global_planner` | `ComputePathToPose` action server — `syncai_planner` until 2026-10, renamed at the package only (namespace, include dir, library, pluginlib names `syncai_global_planner/<Name>`); the node `planner_server`, its actions / services / topics and the `planner` log directory kept their names, and a pre-rename `build/syncai_planner` + `install/syncai_planner` must be deleted or the old plugin names keep resolving to the stale `.so`. Four pluginlib planners are built (NavFn, StraightLine, SmacPlanner2D, OctomapPlanner3D); **SmacPlanner2D is loaded** under the id `GridBased` that the 2D `move.xml` on `dev` asks for, and since 2026-10 **OctomapPlanner3D beside it as a second id, `Octo3D`** (prototype) — **which a 3D map's trees ask for on this branch** (`feature/octomap-planner-3d`: `move_3d.xml` / `dock_3d.xml`, with `FollowPath` — MPPI, 3D-aware since 2026-10-20; `FollowPathRPP3D` the day before — picked per goal by the task_runner from this planner's latched `planner_server/Octo3D/map_available`, 2026-10-19; see `syncai_task_runner`), so every navigation goal on a 3D map plans on the octree here, while a 2D map runs `dev`'s `move.xml` on `GridBased`: the quadruped's ground-constrained 3D planner on `map/<name>/octomap.bt` (the tree `syncai_mapping`'s `clean_map --octomap 1` writes; the launch derives the path from the directory of `[map] pcd`, no INI key). It reduces the tree once at startup, in a background thread, to a multi-layer 2.5D surface map — every column's floor surfaces judged with the robot's geometry (`body_clearance` 0.6, `robot_radius` 0.25 = the costmap's inscribed radius, `max_step_height` 0.17 = the legs' step, two cells, since 2026-10-14 — 0.1 before, which refused stairs that the stair links now handle; `max_slope_deg` 20 on a plane fitted only when the disc spans more than one step, so a kerb is level ground and a flight is not; `footprint_min_support` 0.8 — up to 20 % of the disc's columns may be *gaps*, columns nothing was ever seen in, paid for in cost, while a wall, drop or obstacle under the disc still blocks outright, because with every column required 13 % of the robot's own keyframes stood on "untraversable" ground; a hole-fill for the ~5 % of floor columns the MID360's −7° lower FOV never sees under the robot's own track, a "voxel above must be observed free" rule that removes ceilings seen from below) — and runs A\* over the cells that passed, so a path cannot leave the ground, climb a wall, cross a stair riser / kerb / drop-off or change floors except over a **stair link**, and takes ramps; the 2D costmap is used for its frame id only — and since 2026-10-15 `planner_server` does not wait for it to be current before an `Octo3D` plan (`GlobalPlanner::usesCostmap()`, `false` there), because a 3D-only map has no `gridmap.yaml`, its static layer never gets a map, and every goal hung in the timeout-less `waitForCostmap()` until then (a `GridBased` goal on such a map hung the action thread for good until 2026-10-19 and now fails at once, `PLAN_FAILED`, because the wait ends on a static layer that has never received a map, on a cancel, or after `costmap_wait_timeout` (5 s)) — the start height comes from TF to `<robot_id>/pointlio_body` (the planar `map → base_link` has z = 0) and a goal's z picks the floor (`goal_z_mode`). Stair links (2026-10-14): the map's own `poses.txt` (next to `octomap.bt`, or `poses_file`) is read at startup, and every stretch of that track leaving one large component (≥ 10 m²) for another ≥ 0.3 m higher or lower becomes a link the A\* may take both ways (3D length × `stair_cost_multiplier` 2.0) — so a stair the robot walked while mapping is a route between floors, and every other stair and drop-off stays refused (a kerb up to `max_step_height` is a step). Since 2026-10-19 the track only says *where* the stair is: a single flight (≤ `stair_straight_max_flight` 8 m of track) is laid as a **row of straight links** across the stair's width (a plane fitted to the flight's surfaces gives the uphill direction; from every lower-floor cell within `stair_straight_half_width` 2 m of the walked foot a line is marched up it to the upper floor), so the A\* picks the entry and a route crosses the stair where its own line meets it, facing the risers, and the line-of-sight shortcut spans the link (`stairCrossing`: one segment floor → flight → floor, within 15° of the uphill direction) — dp's 2F flight went from one walked link 9° off the stair's axis (a 27° turn at the stair head) to 24 straight ones and a route that is one straight line. A stretch that is not one straight flight (a switchback stairwell, a plane fit that disagrees with the walk) keeps the **walked link**, the keyframes' ground points with the landing turns cut out and smoothed, and a startup note says why; `stair_straight: false` is the old behaviour. Floors no link joins and climbs longer than `stair_max_length` are logged at startup. A link spans the whole stairwell because its landings rarely survive the geometry rules; climbing it is RPP's stair mode (see `syncai_controller`), untested on the robot. The core (`plugins/octomap_planner_3d/surface_map`, `surface_astar`, `path_postprocess`, `stair_links`) is ROS-free; `octomap_planner_3d_cli` runs it offline (`--stats`, `--check-poses` against `poses.txt`, `--dump-image`, `--plan`), also buildable without ROS from `plugins/octomap_planner_3d/standalone/`, and `test/test_surface_map.cpp` (gtest, synthetic ramp / stairs / kerb / wall / holes / two floors, and a walked stair / kerb for the links) is the package's first real test. A missing `octomap.bt` logs one error and fails only `Octo3D`'s plans — and latches `map_available` false, so goals run the 2D trees. Limits: `is_path_valid` judges its paths on the 2D costmap (wrong upstairs — why `move_3d.xml` does not call it), a map switch reloads the tree through `planner_server/Octo3D/load_map` (`nav2_msgs/LoadMap`, map_server's type, `map_url` = the new `octomap.bt`; 2026-10-19 — a restart until then), which the backend must call beside `map_server/load_map`: it answers once the new map is built, drops the old one first and leaves none on failure, and with two ids an empty `planner_id` is an error. Measured on dp1f_1012_v2 (2026-10-13): 86 % of the keyframes on traversable ground, the rest being people's feet and ankles in the map (the operator's, 0.1–0.25 m up, inside `clean_map`'s floor-band exemption until its `floor_band_dynamic` of 2026-10-14; the planner's 2026-10-14 rules bring the figure to 89 % on the same map, 89.1 % with the feet gone), the switchback stairwell between the floors (refused by the geometry rules; on dp1f_1012 one stair link joins 1F and 2F over it, 39 m, and a 1F → 2F plan takes ~30 ms) and a ~30° ramp on 2F; a 25 m plan takes ~1 ms. `cost_travel_multiplier: 1.0` (lowered from 2.0 — at 2.0 paths bowed along the inflation gradient in open space) and an explicit smoother block. Wall clearance comes from the **global costmap's inflation band** (`inflation_radius: 0.8` / `cost_scaling_factor: 1.5` since 2026-10; the path rides the band edge in open space), not from the smoother, which has no cost term and cuts corners toward walls the more smoothing it is given. The last plan is re-sent on `plan` at `plan_republish_rate` (1 Hz), trimmed to start at the pose nearest the robot, because the BT plans only when the path is blocked (2D tree) or once per goal (`move_3d.xml`); a **cancelled** navigation empties it (the task_runner calls the planner's `clear_plan`, 2026-10-14), so the console stops drawing the abandoned route. "Blocked", for its `is_path_valid` service, is a centre cell at `>= INSCRIBED` **or** a LETHAL cell under the padded footprint's perimeter at the path heading ± `is_path_valid.heading_margin` (0.35 rad) — RPP's own collision test, run on the global costmap while the blocker is still 2.5 m out (2026-10; until then the centre test alone let a blocker 0.25–0.45 m beside the route through, RPP refused it, and the goal aborted with no detour — the READMEs' old claim that "the planner is the conservative side" was false). A passage narrower than 0.73 m now reads blocked every second while Smac keeps returning the same path; `is_path_valid.footprint_check: false` restores the centre test. Its global costmap runs the **keepout filter** (`filters: ["keepout_filter"]`, 2026-09) — the global costmap only; the controller's local costmap does not, so the planner alone keeps the robot out of a forbidden zone — and only `GridBased` does: **`Octo3D` reads no costmap cells and ignores the keepout** (decided 2026-10-19, not built), so a 3D map has no zones: it has no mask either (no gridmap), and a map is 2D or 3D, never both (2026-10-19), so a 2D map keeps `dev`'s keepout intact. |
| `syncai_controller` | `FollowPath` action server, owning the 4×4 m rolling local costmap (3×3 until MPPI). Two controllers are loaded on `dev`: id **`FollowPath` is MPPI** (`syncai_mppi_controller`, 2026-10), and **`FollowPathRPP`** is the Regulated Pure Pursuit merged into this package, which was `FollowPath` until then. This branch loads a third, **`FollowPathRPP3D`** (`RegulatedPurePursuitController3D`, split out on 2026-10-19 so `FollowPathRPP` stays `dev`'s 2D RPP byte for byte): a **copy** of RPP, not a subclass, because the stair logic runs through `computeVelocityCommands()` — a fix to the shared pursuit / regulation / collision code goes into both files, and its params block repeats `FollowPathRPP`'s values as one calibration, except `desired_linear_vel` (0.8 m/s against 0.60, 2026-10-19, unmeasured). The goal's `controller_id` picks one, so an empty id now fails the goal instead of defaulting. **On this branch a 3D map's trees (`move_3d.xml`, `dock_3d.xml`'s staging leg) name `FollowPath` (MPPI) since 2026-10-20, as the 2D trees do** (with `Octo3D`; `FollowPathRPP3D` from 2026-10-19 until then, now the A/B on a 3D map as `FollowPathRPP` is on a 2D one): both RPP 3D and MPPI match a path with heights in 3D (robot height = `<robot_id>/pointlio_body`'s z − `lidar_height`, since `base_link` is planar; `syncai_util/path_height.hpp`), the goal check (the server's, so any controller) adds `goal_z_tolerance` (0.5 m), and both climb a stretch rising 0.3 m within 1 m in a **stair mode** — stop, `set_policy_mode` to HIMLOCO (`syncai_driver_manager` serves it), settle 3 s, climb at 0.5 m/s (0.2 until 2026-10-19, which crawled), switch back — one state machine, `syncai_util::StairMode` (lifted out of RPP 3D on 2026-10-20), declared under each plugin's name with the same values; the climb itself is each controller's (RPP 3D caps its carrot and speed, MPPI skips its optimizer for a pure-pursuit carrot), and **both climb blind** — no costmap or collision check from the hold at the stair's foot to the hold past its top (no obstacle avoidance on stairs, decided 2026-10-20; the terrain layer judges a flight lethal). Untested on the robot. A path without heights takes either controller's planar code path unchanged — one controller for both map types was the requirement, and MPPI is it: on a 2D map it runs on the local costmap's obstacle layer as on `dev`, on a 3D map on the terrain layer (see `syncai_costmap_2d`). `FollowPathRPP` has neither. Both clamp their own acceleration, because there is no velocity smoother in the stack. MPPI's `vx_max: 0.60` / `wz_max: 0.65` and RPP's `desired_linear_vel: 0.60` / `rotate_to_heading_angular_vel: 0.65` are one calibration with `syncai_driver_manager`'s velocity scales — change them together. Since 2026-10 the same node also serves `syncai_common/action/DockToPose` on `dock_to_pose` with a third plugin loaded twice: **`Docking`** and **`Undocking`** (`syncai_controller::DockingController`, a port of nav2's graceful / opennav_docking smooth control law), paired with `dock_goal_checker` (5 cm / 0.05 rad). Direction is the controller configuration (`Undocking` = `allow_backward`, no initial rotation), not a goal field, because the `Controller` interface carries a path and nothing else. The dock pose is transformed into the local costmap's odom frame **once** at acceptance and tracked there, so localizer corrections never move the target mid-approach. **The docking controller reads no costmap and avoids nothing** — the dock is itself a lethal cell; avoidance is `dock.xml`'s staging leg, and the last metre is blind by decision. The two action servers are mutually exclusive on the server's one (now `timed_`) mutex (`CONTROLLER_BUSY` 302; the loser publishes no zero twist), the dock loop skips the `isCurrent()` wait, and an empty `goal_checker_id` now resolves to the **first** entry of `goal_checker_plugins` (needed because the list has two entries and `move.xml` sets none). `v_linear_min` / `v_angular_min` are the gait's floors under MPC, pre-scale: **0.05 m/s** measured 2026-10-14 (RL's linear floor is ~0.3 m/s, which is why the trees switch to MPC first), applied after the law's angular bound, and **0.15 rad/s** since 2026-10-15 — the 0.03 measured with it did not hold for a sustained in-place turn (both robot01 undocks stalled at 0.09), so `dock_goal_checker`'s yaw window went 0.05 → 0.1 with it. The same day the reverse was fixed: the target yaw was flipped by π on top of the egocentric coordinates' own backward flip, so a straight back-out was unstable — it veered, U-turned and arrived 2.3 rad off. The reverse floor is assumed equal to the forward one, and it and the `Undocking` cap are unmeasured. Nothing in the launch or the session spec changed. |
| `syncai_mppi_controller` | **Local obstacle avoidance** (2026-10): a port of nav2 Humble's `nav2_mppi_controller` (1.1.20) as a `syncai_nav_core::Controller` plugin that `controller_server` loads beside RPP. It is a controller, not a separate "local planner" layer: it samples control sequences on the local costmap, scores them with critics and outputs `cmd_vel` directly, so nothing tracks its trajectory afterwards. The optimal trajectory is published on `local_plan` (`nav_msgs/Path`, not upstream). Upstream code under renamed symbols, with six additions: no lifecycle (`initialize()` = configure + activate), an **acceleration clamp** on its output (`max_linear_accel` 1.0 / `max_angular_accel` 3.2), because Humble's MPPI bounds velocity only and this stack has no smoother, a **walking floor** (`walk_vx_min` 0.30, 2026-10-15; RPP's `min_approach_linear_velocity`), applied before the clamp, because RL stood still at MPPI's 0.13 m/s, and an **in-place turn floor** (`in_place_wz_min` 0.45, 2026-10-14; RPP's `rotate_to_heading_min_angular_vel` is the same floor), because RL does not turn in place below it and MPPI tapered into the goal yaw under it and stalled there — the `cmd_vel` value, pre-scale: 0.45 since 2026-10-21, when 0.3 (2026-10-15; 0.21 = 0.3 / `scale_turn` for a day before that) turned the robot at 0–0.16 rad/s for ~9 s in front of a blocker while MPPI swung between passing left and right; the floor itself lies in 0.3–0.65, unmeasured, a **final rotation** (`final_rotation_wz` 0.65, 2026-10-15; RPP's rotate-to-goal-heading): inside `xy_goal_tolerance` of the goal the optimizer is skipped and the robot turns in place, because there `GoalCritic` and `GoalAngleCritic` pulled against each other, a small `vx` kept the turn floor off, and ~0.1 rad/s turns stalled the goal until the progress checker aborted, and a **direct approach** (`approach_dist` 0.5, 2026-10-16; RPP's fixed-speed approach): inside 0.5 m of the goal, once the optimizer asks to stand (raw |vx| under `in_place_vx_max`, which the walking floor leaves alone), the robot turns onto the goal point and walks to `xy_goal_tolerance` at `walk_vx_min`, because there only `GoalCritic` still pulls and its mean-distance cost pays a crawl best — robot01 stood 0.19 m from a goal beside it until the progress checker aborted; the turn and the walk are checked against LETHAL footprints, and a blocked way hands the robot back to the optimizer. Two more since 2026-10-20, so that `move_3d.xml` names it as `move.xml` does: **3D path matching** (the `PathHandler` puts the robot at the ground height under it and finds the nearest pose in 3D when the path has heights, RPP 3D's fix for the spiral stair whose turns lie over each other) and the **stair mode** (`syncai_util::StairMode` shared with RPP 3D, `stair_linear_vel` 0.5 (also its floor on the flight, `stair_min_linear_vel`) / `stair_lookahead_dist` 0.6 (RPP 3D's 0.4 until 2026-10-20, sized for a spiral; a flight is a straight link now) its own): `climbStairs()` runs first in the cycle, brakes with the clamp while the policy switch is pending, and on the flight skips the optimizer for the direct approach's carrot law, without reading the costmap — no obstacle avoidance on stairs, by decision; the terrain layer judges a flight lethal, which is also what keeps the optimizer off a stair it was not routed over. A path without heights enters neither, so MPPI on a 2D map (obstacle layer) is `dev`'s. Two changes on 2026-10-21 to when it gives up and when it lets go of the path: the optimizer's "Optimizer fail to compute path" is a `PlannerException` (Humble's `std::runtime_error` skipped `failure_tolerance` and aborted FollowPath on the first cycle in which every candidate collided), and a path point counts as blocked for `PathAlignCritic` / `PathFollowCritic` when the footprint on it, facing along the path, touches LETHAL — the centre-cell test alone never flagged a blocker 0.25–0.45 m beside the path, so `PathAlignCritic` held the robot on the blocked line until a replan or an abort moved it on. `model_dt` must equal 1 / `controller_frequency`; a longer controller period throws from `initialize()` and takes the server down. Needs `libxtensor-dev` / `libxsimd-dev` (dev stage). |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`). Six docking nodes since 2026-10 — `DockToPose` (action client of `dock_to_pose`), `SetMotionKey` (service client of `syncai_driver_manager`'s gait switch), `ComputeStagingPose` (the dock pose moved 1 m back along its yaw, pure math), `ComputeBackOutPose` (2026-10-15: 1 m behind the robot in its own `base_frame`, no TF — `undock.xml`'s target), `Wait` (a `StatefulActionNode` timer, preferred over BT.CPP's `Delay` decorator) and, since 2026-10-14, `WaitForMotionState` (subscribes `mode` and returns `SUCCESS` once `data[1]` equals the requested motion state — MPC is 5 — or `FAILURE` after its timeout) — `DockToPose` drives to the pose it is given and nothing else: its `method` port (`position` / a reserved `charger`) was removed on 2026-10-15, because a charger dock will be a behavior tree of its own whose detection node produces that pose, so neither the node, the action nor `controller_server` changes for it — and with them a dependency on `syncai_common`, for those client types only: no BT node reads an action's error-code constants, the node → code table stays in `syncai_task_runner`. `BtServiceNode` gained `report_failure()` and an `on_timeout()` hook (empty by default so `ClearEntireCostmap` stays silent). |
| `syncai_task_runner` | The BT navigator. Serves `nav2_msgs/NavigateToPose` and, since 2026-10, `syncai_common/action/NavigateToGoal` on `navigate_to_goal` — one `PoseNavigator<ActionT>` template, two instantiations, same tree, one navigation at a time across both. The new action's result carries a coarse failure reason: the BT action nodes write `failed_node` / `failure_msg` to the blackboard on abort (`BtActionNode::report_failure()`, the `number_recoveries` pattern), and the navigator maps the node name to `PLAN_FAILED` / `FOLLOW_PATH_FAILED` (or `UNKNOWN`) in `goalCompleted()`. Its feedback's `status_msg` comes from the same channel: the BT nodes write `nav_status_msg` at each turn of `move.xml` (`IsPathValid` / `GlobalUpdatedGoal` when a replan starts, `ComputePathToPose` / `FollowPath` on success, abort and retry) and the navigator copies it in `onLoop()`; a goal that is preempted or rejected before the tree runs is ABORTED with an **empty** result (`error_code` NONE), which the backend must not read as a navigation failure. The nav2 action is what the backend still sends; it goes only after the backend has migrated, and rviz's `goal_pose` subscriber moves with it. Hosts the `Navigator<ActionT>` abstraction and `behavior_trees/*.xml`. **On this branch (`feature/octomap-planner-3d`) the tree depends on the map** (2026-10-19): a map is 2D (`gridmap.yaml`) or 3D (`octomap.bt`), never both, and per goal the navigator swaps a requested tree (empty = `move.xml`) for its `<stem>_3d.xml` sibling when the planner's latched `planner_server/Octo3D/map_available` (`map_3d_topic`) says the map is 3D — `move.xml` → `move_3d.xml`, `dock.xml` → `dock_3d.xml` (staging leg only), `undock.xml` unchanged; per goal because `Octo3D/load_map` switches maps without restarting the task_runner, and a preempt after the map changed type is a different tree, so it is refused. `move.xml` / `dock.xml` are `dev`'s files byte for byte. **`move_3d.xml` navigates in 3D**: `planner_id="Octo3D"`, `controller_id="FollowPath"` (MPPI, 3D-aware since 2026-10-20; `FollowPathRPP3D` the day before), and `IsPathValid` replaced by a `number_plans == 0` check (BT.CPP's `BlackboardCheckInt`; the navigator resets the count per goal), so it plans once per goal and again only when the goal changes — `is_path_valid` judges on the 2D costmap and reads a path upstairs as blocked, and a replan on the static octree returns the same route. No replan detours round a blocker then: MPPI bends round one inside its local costmap (whose terrain layer now sees low obstacles and drops), stops at one that closes the way, and the four `FollowPath` retries (~16 s) are the wait. Until 2026-10-19 that tree *was* `move.xml` on this branch, so a 2D map failed every goal and a 3D map every dock (`dock.xml`'s `GridBased` staging leg). The rest of this paragraph is the 2D tree, `move.xml`. (The 2D `move.xml` checks the path once a second against the planner's `is_path_valid` and replans only when it is blocked — at the same spot for 10 s since 2026-10 (`IsPathValid blocked_time`), so a person crossing or stopping on the route no longer buys a detour — or the goal changed — it replanned unconditionally at 1 Hz until 2026-10, and the robot turned back into blockers mid-detour once the 2D-raytraced global costmap forgot them; an `is_path_valid` timeout or a missing robot pose **keeps** the path for the same reason). `FollowPath` gets four retries with a 60 s refill. MPPI (`FollowPath` since 2026-10) bends round a blocker beside the path, but there is no `Wait` recovery, so once a blocker closes the way, (retries + 1) × `failure_tolerance` (~16 s) is how long the robot waits for it to leave — sized to outlast `IsPathValid`'s 10 s `blocked_time`, so the replan comes first; change the two together.) `bt_loop_duration: 50` ms ticks the tree at 20 Hz and doubles as every BT node's per-tick spin budget (halved), so it is a latency knob, not just a rate. There is no `syncai_bt_navigator` package. Two more trees since 2026-10, selected per goal by `NavigateToGoal.behavior_tree` (a bare file name now resolves against `behavior_trees/`; absolute paths still work), **`dock.xml`** with the **dock pose** as the goal's `pose`, **`undock.xml`** with none: **`dock.xml`** = `move.xml`'s structure to a staging pose 1 m in front of the dock (`ComputeStagingPose`, inside the planner branch so a preempt recomputes it) → `SetMotionKey key="5"` (MPC) → `WaitForMotionState state="5"` (the controller's own report on `mode` data[1], 5 s timeout) → `Wait 3000` (settling; 2000 until 2026-10-15) → `DockToPose controller_id="Docking"`, restoring RL (`key="1"`) only on a switch or approach failure, and ending **standing on the dock in MPC** (the lie-down it sent from 2026-10-14 was removed on 2026-10-15); **`undock.xml`** = MPC (no stand-up step since 2026-10-15: its `key="0"` / `MODE Z` took a robot already standing in MPC down and up again) → `WaitForMotionState` + `Wait 3000` → `ComputeBackOutPose` (1 m behind the robot in its own base frame, which `dock_to_pose` transforms into odom once at acceptance — the goal's `pose` is ignored since 2026-10-15) → `DockToPose controller_id="Undocking"` backing straight out → RL (`key="1"`, confirmed by `WaitForMotionState state="1"`). The MPC switch is confirmed since 2026-10-14 (`set_motion_key` is one-way UDP; MPC reports 5). The navigator maps `DockToPose` / `SetMotionKey` / `ComputeStagingPose` / `ComputeBackOutPose` / `WaitForMotionState` to `DOCK_FAILED=300`; a dock goal's staging leg still reports 100 / 200. The docking nodes' servers are first waited for at the first dock goal, not at startup. |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server. `costmap_filter_info.launch.py` (the info server + a second `map_server` named `filter_mask_server`) is the nav session's `keepout` pane since 2026-09; it serves `map/<name>/keepout.yaml`, derived from the INI's `[map] map`, and **generates a blank one of the gridmap's geometry when that file is absent** (all unknown — a free mask cell would overwrite unknown costmap cells, so white is not a no-op), so the mask server is up on every map. It started nothing instead until 2026-09-30. On a map with **no gridmap** (3D-only, `Octo3D`) both launches start their nodes **idle** since 2026-10-19 — `yaml_filename: ""`, nothing published on `map` / the mask topic, `load_map` still served — instead of `map_server` throwing in its constructor and the keepout launch starting nothing; no blank mask is generated there (no geometry to size it from), so "Filter mask was not received" is expected on such a map. An existing `keepout.yaml` whose image is missing also starts the mask server idle. Reload after editing a mask with `filter_mask_server/load_map`, no restart. |

### Localization & sensing

| Package | Role |
|---|---|
| `syncai_pointlio` | **The LIO front end.** Point-LIO (output model) over the Livox `CustomMsg` + IMU, ported in-tree from `FASTLIO2_ROS2`'s `pointlio` in 2026-09 with the ROS surface unchanged: node `pointlio_node` at `/<robot_id>/pointlio`, `lio_odom` / `body_cloud` / `world_cloud` / `lio_path`, TF `<robot_id>/pointlio_odom → <robot_id>/pointlio_body`, and `reset` (`syncai_common/srv/ResetLIO` — the type moved with it). Two additions since 2026-10: every published cloud is **motion-compensated** into the body frame at the end-of-scan stamp (`MapBuilder::deskewToEndBody`, from the per-group post-update states; it used to be the raw scan through the extrinsic only, which smeared every wall in a turning keyframe), and `body_cloud_dense` carries the same scan at `dense_filter_num: 2` instead of `lidar_filter_num: 6` (x/y/z/intensity, ~0.12 MB a scan, published only while subscribed) for `syncai_mapping` — the filter still sees exactly the points it saw before. `syncai_mapping` and `syncai_localizer` consume it through absolute names their launches inject; `lio_bridge` and both costmaps consume it too. Two-thread `MultiThreadedExecutor` since 2026-10 (single-threaded until then, and the IMU reader — depth 10, 50 ms — dropped samples whenever the timer outlasted it): the lidar / IMU subscriptions have a callback group of their own, while the timer and `reset` share the default one, because `resetCB` takes no lock against the timer. Its launch is the one definition of how the node is configured, and the session specs run it in its own pane — no other launch `include()`s it. |
| `syncai_mapping` | **The mapping back end, one node and a post-save job — not in this repo.** It moved to `chungweeeei/SyncAI-Robot-3D-Mapping` in 2026-10 — still C++ (rclcpp), the one package to leave without an rclrs rewrite — and is materialised back into `src/` by `vcs import < dependencies.repos` (branch `dev`); `git log -- src/syncai_mapping` has its in-tree history. The package name, both executables and `mapping.launch.py` did not move, so `start_mapping.yaml` is unchanged, but its parameter names did: the keyframe, loop and map-cleaning keys were renamed there (`key_pose_delta_trans` → `keyframe_delta_m`, `loop_max_corr_dist` → `loop_gicp_max_corr_dist`, `map_clean_min_hits` → `map_clean_sparse_min_hits`, …), along with `clean_map`'s flags and the keys of `map_clean.recipe.json`'s `params`; an old key is silently ignored. Its own repo also carries a `dependencies.repos` (syncai_common + small_gicp) for building it standalone — keep those pins in step. Edit it in that checkout and commit there; what follows is the package as it left. `pgo_node`, ported in-tree from `FASTLIO2_ROS2`'s `pgo` in 2026-09 with the ROS surface unchanged: node `pgo_node` at `/<robot_id>/pgo`, services `start_mapping` / `save_maps` / `reset_mapping` (typed by `syncai_common` now — the backend is their only caller and must import from there), topics `map_cloud` (rviz) / `map_cloud_file` (the JSON notice the backend reads) / `mapping_status` (latched run state) / `loop_markers`, TF `map → <robot_id>/pointlio_odom` while mapping, and the `/dev/shm/syncai_pgo/<robot_id>` PCD hand-off. The node comes up **IDLE** since 2026-10 and banks nothing until `start_mapping`, re-broadcasting that TF as identity meanwhile so the console's live scan survives the wait (`start_on_launch: true` is the bag-replay escape hatch); a successful `save_maps` ends the run and returns it to IDLE — see "Out of tree" and the README's "The run lifecycle". Two things did change: configuration is declared ROS parameters (`params/mapping_params.yaml`, `/**/pgo_node:`, five robot_id-dependent overrides from the launch) instead of a yaml-cpp `config_path` with a `/tmp` rewrite, and there is no `interface` dependency left. Runs a 3-thread `MultiThreadedExecutor` on purpose — `start_mapping` and `reset_mapping` both block on the `ResetLIO` client — so `m_pgo_mutex` is load-bearing; the README carries the discipline. Its `local_frame` must equal pointlio's `world_frame`: it does not adopt the frame from the odom header. Loop closure changed in 2026-10 after an offline replay showed the fork's loop ICP lifting every revisited corridor 10–14 cm in z: loop pairs are registered with `small_gicp` (`loop_registration: gicp`, 1.0 m correspondence distance instead of the fork's 10) and the loop edge is a `PlanarLoopFactor` on world-frame x / y / yaw only (`loop_planar_correction: true`), so z / roll / pitch stay with the odometry; the fork's behaviour is still reachable through those parameters, and the README's "Loop verification" has the evidence. The loop edge's weight changed on 2026-10-09 after the dp1f_1008_2 replay left a corridor 1.2–2 m apart with the loop found: x / y and yaw have fixed sigmas (`loop_noise_xy_sigma_m` 0.05, `loop_noise_yaw_sigma_deg` 1.5) instead of the fork's "variance = fitness", which the odometry outvoted, and every keyframe carries a `GravityPriorFactor` (`keyframe_tilt_sigma_deg` 0.5) holding its roll / pitch to Point-LIO's, without which a yaw correction leaked into pitch and moved keyframe z by up to 2.2 m. The fork's `hba` was ported alongside it as a second node (`hba_node`, offline bundle adjustment run by hand) and **removed in 2026-10**: nothing called it, and it was the package's only Sophus consumer — `git log -- src/syncai_mapping/src/hba_node.cpp` has it. The second executable, `clean_map` (2026-10), is what a successful `save_maps` with `save_patches` spawns: it counts per voxel the keyframes that hit it and those whose rays passed through it, and **replaces `map.pcd`** — minutes after the save, atomically — without people (`map_clean_dynamic_*`) and one-off returns (`map_clean_sparse_min_hits`), with its state in `map_clean.recipe.json`, which `pgo_node` writes `converting` before the save responds. Offline on purpose: a person is only known once later scans see through the voxel, and only the final loop-closed poses are safe to judge with. It is a **detached process** (`posix_spawn`, own session, nice 10, `oom_score_adj 500`, serialised per host by a `flock` under `/dev/shm/syncai_pgo/`), so it survives the mode switch that kills the mapping session; `pgo_node` reaps it from its 1 s status timer, a new save into the same directory stops it, and `ros2 run syncai_mapping clean_map map/<name>` is the by-hand re-clean. `pgo_node` takes its keyframes from `pointlio/body_cloud_dense` (the cleaning needs the density). Needs GTSAM, plus `small_gicp` for `pgo_node`'s loop registration and OctoMap (`ros-humble-octomap`, a real rosdep key, for its voxel keys and ray traversal only) and `nlohmann-json3-dev` (rosdep `nlohmann-json-dev`, for the recipe) for `clean_map`. |
| `syncai_localizer` | **The relocalizer, nav session only.** Two-stage GICP (`small_gicp`, rough 0.25 m then refine 0.1 m) of `pointlio/body_cloud` against the `[map] pcd`, broadcasting `map → <robot_id>/pointlio_odom` (the frame is adopted from the first odom header) and serving `relocalize` / `relocalize_check` (`syncai_common/srv/Relocalize` / `IsValid`) plus the `initialpose` subscriber and a latched `map_cloud`. Ported in-tree from `FASTLIO2_ROS2`'s `localizer` in 2026-09, the fork's last package, with the ROS surface unchanged — **and that surface is the bare `/<robot_id>` namespace**: the node is `/<robot_id>/localizer_node` and the names resolve to `/<robot_id>/relocalize`, `/<robot_id>/relocalize_check`, `/<robot_id>/initialpose`, `/<robot_id>/map_cloud`. Docs used to say `/<robot_id>/localizer/…` from before fork commit `3f5f01b` (2026-07) dropped the segment; the backend calls the bare names, so the port kept them and the docs were corrected. Registration is motion-gated (`min_update_trans` / `max_update_interval` / `static_blend_alpha`, measured on robot01 2026-09-21) and capped at `update_hz` 5; the tuning history is in `params/localizer_params.yaml`. The launch **starts nothing** when `[map] pcd` is missing on disk, on purpose. 2-thread `MultiThreadedExecutor`: services + `initialpose` on their own group so a multi-second `loadMap` never gaps the TF; `ICPLocalizer::m_target_mutex` exists for that split. `auto_localize` (`syncai_common/srv/AutoLocalize`, 2026-10; the backend called it until 2026-10-15 and now reaches the same search through `reset_odometry`'s `skip_seed`) is the kidnap recovery: a hint-free search over the map's `poses.txt` keyframes × a yaw sweep (`GlobalLocalizer`, its own GICP pair and KD-trees, never `ICPLocalizer`'s), then a wait for the timer to register against the winner. Unlike `relocalize` it **blocks and its success is a result**; both halves have a deadline (`auto_loc.search_timeout_sec` / `converge_timeout_sec`), and a candidate the timer does not converge on is withdrawn rather than retried forever. Its acceptance values are unmeasured first guesses. `reset_odometry` (`syncai_common/srv/ResetOdometry`, 2026-10) is the recovery for a **diverged Point-LIO**, which neither `pointlio/reset` nor `auto_localize` is on its own: a bare reset leaves this node's offset solved against the old odom origin, and the search's scan window is posed by the diverged odometry. It seeds from a hint or the last *trusted* registration (odometry slower than `reset_odom.max_lio_speed` since the one before), pauses intake, calls `pointlio/reset`, gates on its `last_odom_time` exactly as `pgo_node` does, writes the seeded offset straight away so `map → body` stays continuous, waits for convergence, and optionally falls back to the global search. `skip_seed` (2026-10-15) tries no seed: after the reset it waits for a full `auto_loc.accum_scans` window of new-run scans and goes straight to the global search, holding the pre-reset pose as the offset meanwhile only for TF continuity — the kidnap case, where the last trusted registration is wrong or (after boot) missing, and what the backend's `POST /api/v1/localization/auto` calls, because it cannot tell whether the carry also diverged the LIO. Blocks, and the robot must be stationary. Its `ResetLIO` client is why the executor has **three** threads since then (the handler blocks on the client's future). |
| `syncai_lio_bridge` | **The only odometry source — not in this repo.** It moved to `chungweeeei/SyncAI-LIO-Bridge` in 2026-10, the third package that month to go out to its own repo and be rewritten against **rclrs** (Rust, `ament_cargo`), and is materialised back into `src/` by `vcs import < dependencies.repos` (branch `dev`); `git log -- src/syncai_lio_bridge` has the C++ node. The ROS surface did not move: same package name, same `lio_bridge_node`, and `lio_bridge.launch.py` is byte-for-byte the rclcpp one (every parameter inline, no params file — so the bare-`/**` key the other two Rust packages need does not arise), so `start_nav.yaml` is unchanged. Wheel odom is retired. Converts the FAST-LIO2 chain (`map → pointlio_odom → pointlio_body`) into `odom → base_link` TF + `/<robot_id>/odom` + the AMCL-style `map → odom` correction, all projected to 2D (x, y, yaw) so the planar nav stack never sees a tilted frame. Angular velocity comes from the lidar IMU gyro — by choice, not as a fallback: an older note here said LIO leaves `twist.angular` empty, and it does not; the package README has why the gyro is preferred. The port does its own TF (rclrs has no `tf2_ros`): a `/tf` + `/tf_static` subscription keeping only the newest transform per child frame, and a plain `/tf` publisher. Edit it in that checkout and commit there. |
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` + the Livox driver. The URDF carries the lidar mount extrinsic to **`lidar_top`, the lidar itself** (the origin of its points, and the frame the driver stamps `livox/lidar` with since 2026-10 — `<robot_id>/laser`, which nothing parented, until then), and under it **`lidar_top_imu`**, the lidar's built-in IMU, at `-t_il` from `syncai_pointlio`'s params: that is what Point-LIO's body frame physically is, so it is the frame to map `pointlio_body` onto `base_link` through. `syncai_lio_bridge` / `syncai_robot_state` still name `lidar_top` as their `lidar_frame` (~5 cm off, as always) until their repos move to `lidar_top_imu`; `camera_optical_frame` hangs off `lidar_top`, so a camera ↔ lidar calibration result goes in as it is. The fleet runs **both MID360 and MID360s**; the driver has no ROS parameter for the model, so `[sensor.lidar] type` (`mid360`/`mid360s`) picks the JSON schema. The driver's network JSON is **generated** per `robot_id` and model into `/tmp/syncai_bringup/` from `[sensor.lidar] ip` + `type` (INI) + `host_ip` (params YAML) — the vendor `MID360_config.json` / `MID360s_config.json` in the driver's share dir is not read. The old 2D/AMCL `bringup_2d.launch.py` (laser scan merger) was removed. **No camera** since 2026-10-17: the `vizionsdk_ros2` node it could start under `use_camera:=true` was removed — see below. |

**Camera.** Exactly one process may hold the V4L2 device at a time (the
second opener does not fail at `open()`, it dies at `S_FMT` with "Device or
resource busy" long after the pane has scrolled), and that process is
`syncai_camera_feeder` (see "Hardware & system"): it fans the undecoded MJPEG
out to both the WebRTC worker and ROS, and the nav session starts it
(`camera_feeder` window of `start_nav.yaml`, 2026-10-12; the mapping session
does not). Nothing else in the stack opens the camera. Two former owners were
removed in 2026-10, and git history has both:
- `bringup.launch.py`'s `vizionsdk_ros2` node (`use_camera:=true`, default
  off, removed 2026-10-17) — a second opener in window 0 could only
  crash-loop the feeder, which its launch respawns every 2 s. What it learned
  about the hardware (no IMU, no intrinsics, 60 fps the only common rate,
  white balance latched in the camera) is kept in `syncai_bringup`'s README.
- The **host-side** `scripts/publish_camera_crop.sh` (GStreamer, hardware
  crop/scale/H.264 on the Tegra, `rtspclientsink` to a remote MediaMTX), with
  its `publish_camera_crop.env` and the per-robot fisheye crop values in it.

The camera is not configured in the instance INI.

### Hardware & system

| Package | Role |
|---|---|
| `syncai_driver_manager` | **UDP bridge to the gait controller — not in this repo.** It moved to `chungweeeei/SyncAI-Robot-Driver-Manager` in 2026-10 and is materialised back into `src/` by `vcs import < dependencies.repos` (branch `dev`), the same arrangement `syncai_common` has; it was rewritten there against **rclrs** (Rust, `ament_cargo`), so the C++ node this table used to describe is history now — `git log -- src/syncai_driver_manager` has it. The ROS surface did not move with it: same package name, same `driver_manager_node` executable, same `driver_manager.launch.py`, so both session specs are unchanged. Sends `cmd_vel` with a per-direction velocity-scale correction (the gait controller tracks commands asymmetrically). The six scales (`scale_fwd` … `scale_turn_r`) are **ROS parameters** loaded from `params/driver_manager_params.yaml` (1.40 fwd / 1.40 turn today) and survive restarts; a runtime `set_speed_scale` override is what does *not* persist. That file is keyed on a bare `/**` there, not the `/**/driver_manager` the rest of this workspace uses — rclrs matches a params key only when it is exactly `/**` or exactly the node's fully-qualified name, with no wildcard expansion, so the old key matched nothing and every scale fell back to its code default. Receives ASCII telemetry, and owns the safe-shutdown path (safety lock + MODE X / lie down) — which still has **zero call sites** (`SafetyLock::trigger` is called only from tests; the joint-overheat monitor that would engage it is a TODO in `telemetry.rs`). The lock's state is published on a latched `safety_locked` (`std_msgs/Bool`, RELIABLE + TRANSIENT_LOCAL depth 1) — new with the Rust port — and surfaces as `RobotState.low_level_mode.safety_state`, so that field reads `false` today for want of a trigger, not because anything was checked. Edit it in that checkout and commit there. |
| `syncai_robot_state` | **Aggregates odom / battery / wifi / motor_states / the gait controller's `mode` / `safety_locked` / TF into `syncai_common/RobotState` — not in this repo.** It moved to `chungweeeei/SyncAI-Robot-State` in 2026-10, the second package that month to go out to its own repo and be rewritten against **rclrs** (Rust, `ament_cargo`), and is materialised back into `src/` by `vcs import < dependencies.repos` (branch `dev`); `git log -- src/syncai_robot_state` has the C++ node. The ROS surface did not move: same package name, same `robot_state_node` executable, same `robot_state.launch.py` (still reading `[system] robot_id` and `[map] map` from the INI to override `robot_id` / `base_frame` / `map`), so both session specs are unchanged. The code default is 10 Hz but the shipped params file sets `publish_rate: 1.0`, so it runs at **1 Hz** — and that file is keyed on a bare `/**` there, not `/**/syncai_robot_state`, for the same rclrs reason `syncai_driver_manager`'s is. `RobotState.mode` is no longer hardcoded `AUTO`: it polls `syncai_sys_manager`'s `get_mode` (`mode_poll_rate`, 1 Hz, one request in flight at most) and reads `AUTO` until the first answer. Also derives the `state` field: `UNINITIALIZED` (no pose) / `WARNING` (battery <20%, cleared above 25% — latched with hysteresis) / `IDLE`. Reports only — no threshold here commands the robot. Two more things the port makes explicit: `transform_tolerance` has no effect (it looks up in its own in-memory `/tf` buffer and does **not** expire a stale transform), and the `WifiStatus` → JSON flattening is `serde_json` now, which left `nlohmann-json3-dev` to `syncai_mapping`'s `clean_map` alone. |
| `syncai_sys_manager` | Python. Five managers behind ROS services: wifi (`scan_wifi` / `connect_wifi`, `wifi_status` at 1 Hz from a cache refreshed every 5 s), mDNS (`avahi-publish <robot_id>.local`), conf (declares `robot_id`), monitor (host memory / disk to stdout at 1 Hz), and **node** (`NodeManager` — byobu session lifecycle, `switch_mode` / `get_mode`; see "Running the stack"). Also ships the host udev rules (`udev/99-syncai-devices.rules`). |
| `syncai_camera_feeder` | **The single opener of `/dev/syncai/camera0` — not in this repo, and never was.** Written in `chungweeeei/SyncAI-Robot-Camera-Feeder` against **rclrs** (Rust, `ament_cargo`, the fourth) and GStreamer, and materialised into `src/` by `vcs import < dependencies.repos` (branch `dev`). One `v4l2src` (MJPEG 1280×720@60) teed, undecoded, into RTP/JPEG on `udp://127.0.0.1:5008` for the WebRTC worker and `image_raw/compressed` (`sensor_msgs/CompressedImage`, 30 Hz by `ros_framerate`, SensorData QoS, capture-time stamps, frame `<robot_id>/camera_optical_frame`); it exits on any pipeline error and its launch respawns it. Its params file is keyed on a bare `/**`, the rclrs rule. Links GStreamer through the `gstreamer` / `gstreamer-app` crates, which is why the `dev` stage installs `libgstreamer1.0-dev` / `libgstreamer-plugins-base1.0-dev` (2026-10-12; until then it built only where they had been installed by hand). **Started by the nav session only** (`camera_feeder` window of `start_nav.yaml`, 2026-10-12; not in `start_mapping.yaml`), and nothing else may open the device beside it — see "Camera". Edit it in that checkout and commit there. |

### Application layer

There is none in this repo any more. `syncai_backend` — FastAPI **and** rclpy in
one process (`MultiThreadedExecutor`) on port **3000**, Temporal worker for task
orchestration, TTS, task templates and schedules, the map catalogue and the
teleop / telemetry / point-cloud WebSockets — **was removed from this workspace
in 2026-09** and lives in `SyncAI-Robot-Backend`. `syncai_frontend` (the Next.js
operator console, dev server on 3001) went the same way weeks before it, and was
never an ament package, so neither removal changed the ROS build beyond deleting
a package from it. Neither 3000 nor 3001 belongs to anything here now.

Its internals are documented in that repo. What the split means for the code
that stayed — which robot-side services the backend calls, and which of this
stack's behaviours it depends on — is under "Out of tree" below.

What `src/` holds: the `syncai_*` packages in the tables above, plus
`src/third-party/`. Six of those packages are not tracked here; all six are
materialised back into `src/` by one file, `dependencies.repos` (every entry on
branch `dev`, over HTTPS — the repos are public):

| Package | Repo |
|---|---|
| `syncai_common` | `chungweeeei/SyncAI-Robot-Interface` |
| `syncai_mapping` | `chungweeeei/SyncAI-Robot-3D-Mapping` |
| `syncai_driver_manager` | `chungweeeei/SyncAI-Robot-Driver-Manager` |
| `syncai_robot_state` | `chungweeeei/SyncAI-Robot-State` |
| `syncai_lio_bridge` | `chungweeeei/SyncAI-LIO-Bridge` |
| `syncai_camera_feeder` | `chungweeeei/SyncAI-Robot-Camera-Feeder` |

`syncai_common` left in the 2026-09 split, so the backend can build against the
message definitions without checking out this workspace. The other four left in
2026-10: three were rewritten against rclrs on the way out and are the
workspace's three `ament_cargo` packages, and `syncai_mapping` went as the C++
it was. Until 2026-10 each had its own `.repos` file (`interface.repos`,
`driver-manager.repos`, `robot-state.repos`, `lio-bridge.repos`); they were
merged into `dependencies.repos` when `syncai_mapping` became the fifth, so
`vcs import < dependencies.repos` is the one command and a `--force` import
re-checks out all six. `syncai_camera_feeder` (2026-10), the sixth and the
fourth `ament_cargo` package, never left anything: it was written in its own
repo against rclrs from the start. Edit any of them in its own checkout; a change made in
those directories is untracked, and the next `--force` import overwrites it.
The next package to move out gets an entry there, not a new file.

**The Rust packages build on an underlay baked into the image.** Besides the
toolchain (rustup, `cargo-ament-build`, `colcon-cargo` / `colcon-ros-cargo`,
`libclang-dev`), the `rust-underlay` stage builds a ros2-rust underlay into
`/opt/ros2_rust_underlay` (2026-10): ros2-rust's own `ros2_rust_humble.repos`
minus `examples`, plus `rclrs` from source (crates.io 0.7.0 does not compile
against the generator on main), with the three ros2-rust repos pinned to SHAs
in `ARG`s that must be bumped together. `rclrs`'s and `rosidl_runtime_rs`'s
`build.rs` are patched there to rerun on `AMENT_PREFIX_PATH` (upstream reruns
only on `ROS_DISTRO` and keeps stale link-search paths, so a build without
`install/setup.bash` sourced followed by one with it failed to link
livox_ros_driver2's typesupport; patched and verified both ways 2026-10-12). It exists because `rclrs`'s bindings
for `std_msgs` / `sensor_msgs` / `geometry_msgs` / `std_srvs` / `nav_msgs` /
`syncai_common` are generated by `rosidl_generator_rs`, which only runs on
interface packages built after it — the apt copies ship none. It is in the
image, not a `.repos` import, because it is toolchain nobody here edits. It is
sourced between `/opt/ros/humble` and the workspace (`~/.bashrc`,
`scripts/build.sh`, robot01's `command:`), so its rebuilt `common_interfaces` /
`rcl_interfaces` overlay the apt message packages for the **whole** workspace,
C++ included — same Humble branch. `syncai_common` is still built in the
workspace and gets its Rust crate from the underlay's generator. Verified
2026-10-02: underlay 29 packages in the image, then all 22 workspace packages
including both Rust ones (before `syncai_lio_bridge` became the third). A workspace that was ever built with these interface
packages in `src/` keeps stale `build/` + `install/` copies of them (and a
`syncai_common` CMake cache pointing at those) that shadow the underlay — delete
them, or configure fails on a missing `register_rs.cmake`. `tf2_msgs` (needed by `syncai_robot_state` and
`syncai_lio_bridge`) is the one dependency that does **not** need it:
`ros-humble-tf2-msgs` already ships generated Rust bindings, which is what both
nodes' hand-rolled `/tf` handling links against (rclrs has no
`tf2_ros` binding).

`syncai_ros_mcp` — a vendored MCP server that exposed the ROS 2 graph and the
backend's REST API as MCP tools over HTTP on port 8000 — **was removed**, because
this version has no use for it: it was in no session spec and not in compose, so
nothing ever started it, and nothing in the stack imported or called it. Recover
it from git history rather than re-deriving it if the agent work resumes. Its
`FastMCP` pip dependency went with it, which leaves Sophus / GTSAM (source builds:
Sophus for `syncai_pointlio`, GTSAM for `syncai_mapping`'s `pgo_node`; `syncai_localizer` needs neither) as the only
manual dependency `rosdep` does not cover.

### Third-party (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`. Unmodified. |
| `direct_visual_lidar_calibration` | `koide3/direct_visual_lidar_calibration`, pinned to a main SHA (no tags upstream; 2026-10). Targetless camera → lidar extrinsic calibration, run by hand on bags recorded in `CALIBRATION` mode (see "Running the stack"). Has **git submodules**, so `third-party.repos` is imported `--recursive` (CI does; `scripts/build.sh` refuses a checkout without them). Its source-built deps are image stages since 2026-10 — `ceres` (2.2.0; it needs `ceres::Manifold`, jammy ships 2.0), `iridescence` (v1.0.3, its GUI), and `libgtsam_unstable` from the `gtsam` stage, which its own `FindGTSAM` demands — plus `pcl_ros` / `cv_bridge` / `fmt` / glog / glfw from apt in `dev`. Builds only against GTSAM 4.2. |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver |
| `small_gicp` | Pinned to upstream tag `v1.0.1`. Unmodified. The registration backend (`RegistrationPCL`, a `pcl::Registration` subclass) of `syncai_localizer` and, since 2026-10, of `syncai_mapping`'s loop verification. Ships its own `package.xml` with `<build_type>cmake</build_type>`, so colcon builds it as a plain CMake package and both find it with `find_package(small_gicp)`; the ordering comes from `<depend>small_gicp</depend>` in their manifests. |
| `vizionsdk-ros2` | `TechNexion-Vision/vizionsdk-ros2` (branch `main`). ROS 2 wrapper (`vizionsdk_ros2/vizionsdk_camera_node`) for the TechNexion camera. **Nothing starts it** since `bringup.launch.py` dropped its `use_camera` node on 2026-10-17 (see "Camera"); it is still imported and built. Needs the closed-source VizionSDK `.deb`, which the `Dockerfile` downloads from the TechNexion GitHub release (`VIZIONSDK_VERSION`); there is no rosdep key for it. |

All six are checked out by vcstool from `third-party.repos` — they were git
submodules until `fca520b`, and `.gitmodules` is gone; bumping one is a
one-line `version:` edit there plus `vcs import --recursive < third-party.repos --force`,
not the old two-commit dance. The import target is the workspace root, not
`src`: the keys already carry `src/`. The last vendored package,
`ros2_laser_scan_merger`, went away with the 2D/AMCL path (commit `99141a6`).

**`FASTLIO2_ROS2` is not in the list any more.** The FAST-LIO2 fork
(`chungweeeei/SyncAI-Fast-LIO2`, the one SSH remote, pinned to a SHA on `dev`)
was imported at `src/third-party/FASTLIO2_ROS2` until 2026-09, when its four
packages were ported in-tree one by one — `pointlio` as `syncai_pointlio`,
`pgo` and `hba` as `syncai_mapping`'s two nodes (`hba_node` since removed), `interface` into
`syncai_common`, and finally `localizer` as `syncai_localizer` — and the entry
went with the last of them. `scripts/build.sh` no longer checks for the
directory, and a full container build with it deleted (fresh image, 22
packages, 2026-09-28) is the state the workspace is verified in. A checkout
left on disk from before is gitignored and still builds
a second `localizer` package with its own `localizer_node`; delete it
(`rm -rf src/third-party/FASTLIO2_ROS2 build/localizer install/localizer`)
rather than let two launches exist for one node. The fork itself is history
now: nothing here reads it, and its README / CLAUDE.md still describe the
workspace as its consumer.

## The robot_id convention

Every launch file reads `[system] robot_id` from **`config/system.ini`** via a
`read_robot_id()` helper, falling back to `default_robot` with a warning. The
resolved value is used as the **node namespace**.

`config/system.ini` is tracked but **empty**. Per-robot identity comes from
`config/instances/robotNN.ini`, which docker-compose bind-mounts over
`config/system.ini` inside the container. That file has exactly four sections:
`[system]` (`robot_id`), `[map]` (`name`, plus `pcd` / `map` paths written with
configparser interpolation as `map/%(name)s/map.pcd` and
`map/%(name)s/gridmap.yaml`, so switching maps is a one-line `name:` edit —
which is what the backend's map-switch route writes, and why it rewrites that
one line in place rather than round-tripping the file through configparser.
Nothing in this repo writes this file; that it stays rewritable one line at a
time is part of the contract with the backend),
`[initial_pose]`, and `[sensor.lidar]` `ip` + `type` (the lidar's address and
model, `mid360` or `mid360s` — `syncai_bringup` renders the livox driver's
config JSON from them). The former `[artifacts]` section is gone with the
conveyor integration, and the camera is **not** configured here (see "Camera").
The model is load-bearing: the JSON *section name* is what Livox-SDK2 maps to a
device type, and that decides the command handler, so a wrong `type` yields no
point cloud and no error. A missing `type` warns and falls back to `mid360`, for
back-compat with instance INIs written before the MID360s arrived.

The C++ packages' launch files use the relative path `config/system.ini`, which
works because **processes are expected to run with the workspace root as their
cwd** — the session specs start every pane in the workspace root (no window
sets `cwd:` today; the frontend window was the one that did, and it left with
the package), and `colcon.meta` and `ruff.toml` rely on the same assumption.
`syncai_sys_manager`, the one Python package left, instead defaults to the
absolute `~/robot_ws/config/system.ini` (`SYNCAI_SYSTEM_INI` /
`system_config:=`), so it does not depend on the cwd — and so does the backend,
from its own container, which is why that path is an interface too.

Three rules follow from namespacing:

- **Topics are written as relative names** in params YAML and in code
  (`map`, `scan`, `pointlio/body_cloud`), so they inherit the namespace
  automatically. Never hardcode `/<robot_id>/…` in a subscriber — a backend
  subscriber that used an absolute topic name is a bug that has already been
  fixed once, back when that code was in this repo. There are **no** exceptions:
  an absolute, fleet-wide `/robot_state` was tried and reverted, because a single
  DDS domain hosts several robots and every per-robot consumer (the backend, from
  its own container, included) is scoped to exactly one.
- **TF frame names are NOT namespaced by ROS.** Launch files therefore override
  frame parameters explicitly (`robot_base_frame: <robot_id>/base_link`,
  `frame_id: <robot_id>/lidar_top` on the Livox driver). The values in the YAML are only fallbacks
  for running a node without its launch file. `global_frame` stays plain `map`.
- **Params files use `/**/<node_name>:` wildcard keys**, so the same file works
  at any namespace. When a process hosts more than one node (e.g.
  `planner_server` + its internal `global_costmap`), the launch `Node` must have
  **no `name=`** — a launch-level name remaps *both* nodes to the same name and
  the internal costmap silently loses all its parameters. Extra dicts passed to
  such a Node land under the `/**` wildcard and reach both nodes.

`use_sim_time` is set **only** in the params YAML, never in launch. A launch
override placed after the params file would silently win over the YAML value.

## Build

**Builds run inside the robot container, not on the host** — and the container
is typically a live robot. Do not run `colcon build` automatically after editing;
leave building to the user unless they explicitly ask.

```bash
docker compose up -d            # infra + robot01 (docker-compose.yml `include`s the robots file)
docker compose exec robot01 bash
# inside, workspace is at /home/syncrobotic/robot_ws and is the cwd
colcon build --symlink-install
colcon build --packages-select syncai_global_planner
source install/setup.bash

# or from the host, in a throwaway container made from the same image:
docker compose -f docker-compose.build.yml run --rm build [colcon args...]
```

`docker-compose.build.yml` is **standalone** (own project `name:
syncai-build`, not `include`d — a build is not something to `up`) and its one
service runs `scripts/build.sh` as its *entrypoint*, which is why
`run build --packages-select x` appends to `colcon build --symlink-install`
instead of replacing it. It is a **release build** (`BUILD_TYPE`, default
`Release`: `-DCMAKE_BUILD_TYPE` plus `--cargo-args --release`; a plain
`colcon build` leaves every package but pointlio / mapping / localizer
unoptimised), and an own `--cmake-args` / `--cargo-args` replaces those, so
repeat the build type in it. Same image as robot01 on purpose: the artefacts land
in the shared bind-mounted `build/` + `install/`, so the toolchain and headers
must be what robot01 has. It therefore only `rosdep check`s by default
(`BUILD_ROSDEP=check`) — an apt install into a throwaway container never
reaches robot01, so a missing dep is a `Dockerfile` change. The script also
restores the `livox_ros_driver2` `package.xml` when a `vcs import` has deleted
it, and refuses to start on an empty vcs checkout — a `dir:repos` table covering
the six `src/third-party/` dirs plus the six `dependencies.repos` ones
(`src/syncai_common`, `src/syncai_mapping`, `src/syncai_driver_manager`,
`src/syncai_robot_state`, `src/syncai_lio_bridge`, `src/syncai_camera_feeder`),
naming the `.repos`
file to import per missing directory (and, separately, a
`direct_visual_lidar_calibration` checkout without its submodules), because "you forgot to import" is not what
colcon's own failure looks like. The first-party checks earn their place
more than the rest: an absent `src/syncai_common` fails every package at once,
while an absent `src/syncai_mapping`, `src/syncai_driver_manager`, `src/syncai_robot_state`,
`src/syncai_lio_bridge` or `src/syncai_camera_feeder` fails *nothing* — colcon happily builds a workspace with
nothing to save a map with, no gait-controller bridge, nothing publishing `RobotState` and no odometry
source, so the robot stands still at the first `cmd_vel`, the console's
telemetry never arrives, and the costmaps wait forever on an `odom → base_link`
nobody broadcasts. That table is where the
next package to move out gets added. There are no carve-outs in what it builds any more: since the
frontend and then the backend left, the workspace and what colcon discovers are
the same set. It is equally runnable inside robot01 (`scripts/build.sh`). After
colcon it deletes the four rclrs nodes' `install/<pkg>/share/<pkg>/rust/`
source copy (cargo-ament-build always installs one, for downstream Rust
crates nothing here has) together with their `rust_packages` index markers —
a marker left without its directory makes cargo fail every later Rust build;
a plain `colcon build` leaves the copy in place. Its
`build:` block duplicates `x-robot-common` (compose `extends` would drag the
devices / nvidia runtime / X11 mounts along) — keep the two in sync.

Recreating a robot container wipes hand-installed build dependencies (the ones
not in the image). Re-run `rosdep install --from-paths src --ignore-src -r -y`
plus any manual deps (Sophus / GTSAM are built from source; `syncai_pointlio`
needs Sophus, `syncai_mapping` GTSAM; `direct_visual_lidar_calibration` also
needs Ceres, Iridescence and GTSAM's unstable library — all four come from the
image's builder stages).

The `Dockerfile` is multi-stage, and since 2026-10 every slow build is a stage
of its own that depends only on `base`, so BuildKit runs them in parallel and
no `dev` edit invalidates them: `base` (ros-base + cyclonedds + uid-1000 user)
→ `deps-builder` (compiler + Boost / TBB / Eigen, builds nothing) → `livox` /
`gtsam` / `sophus` / `ceres` / `iridescence` (one library each, installed with
`DESTDIR=/out`; the last two and GTSAM's `GTSAM_BUILD_UNSTABLE=ON` are for
`direct_visual_lidar_calibration` alone, 2026-10; GTSAM is
the slow one — keep these stages free of anything that changes often so their
cache survives); `base` → `rust-underlay` (rustup under `/opt/rust`,
`cargo-ament-build`, and the ros2-rust underlay — rclrs, `rosidl_generator_rs`
and the rebuilt standard interfaces — in `/opt/ros2_rust_underlay`; it was two
stanzas of `dev` until then, so a GStreamer or Node edit re-ran the whole
underlay build); `base` → `dev` (colcon, byobu, Node.js, the VizionSDK `.deb`,
`ros-humble-octomap` for `syncai_mapping`'s `clean_map`, `libclang-dev` +
`colcon-cargo` / `colcon-ros-cargo` for the workspace's Rust packages, and one
`COPY --from` per stage above; the workspace is bind-mounted at `~/robot_ws` and
built by hand). There is **no rviz2** or `pointcloud_to_laserscan` in the
image since 2026-10 (the robot has no display; nothing here depends on the
other), and `pcl_ros` is back only as a `<depend>` of
`direct_visual_lidar_calibration`, and `syncai_costmap_2d`'s `rviz:=true` test launch is the only
thing here that would start rviz2. `ros-humble-map-msgs` and
`ros-humble-laser-geometry` are listed explicitly because rviz2 and
`pointcloud_to_laserscan` were what used to pull them in (`syncai_costmap_2d`
needs both; `rosdep check` names them if they go missing). Most of `dev`'s largest layer is
VTK + Qt5 regardless: `libpcl-dev` hard-Depends on them, nothing here links
them, and apt cannot leave them out. `chmod -R a+w /opt/rust` runs once, at the end of
`rust-underlay`: a second walk is what made the old underlay layer carry a
duplicate 516 MB toolchain. Compose builds `target: dev`.
`dev` carries **no Python web stack** any more: fastapi / uvicorn / sqlalchemy /
temporalio / open3d / kokoro-onnx were installed here from
`src/syncai_backend/requirements.txt` until 2026-09, and the `COPY` that read
that file would now fail on a fresh clone. Do not re-add a pip package here for
a process that runs in another container — `python3-opencv` and `python3-dotenv`
went the same way, and what is left is a pure ROS image.
The production stages (`ws-builder` / `nav-runtime` / `backend-runtime`) were
**removed** during the dev phase; `scripts/release/` still references them and
a `docker-compose.prod.yml` that is not in the tree, so the release path does
not work today — recover the stages from git history when it is time to ship
to the IPC.

`colcon.meta` at the workspace root carries per-package cmake args. colcon only
finds it because its `--metas` default is the relative `./colcon.meta` — same
cwd-is-the-workspace-root assumption as the launch files. Today its only entry is
`livox_ros_driver2`, which needs `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`: since
the Mid-360s/Jazzy commit, its `CMakeLists.txt` branches on `DISTRO_ROS`, and
without it the pre-Humble branch resolves the message-typesupport include dirs
off a target name that no longer exists and configure dies with `NOTFOUND`. The
flags belong here rather than in the driver's own tree because upstream only
ever configures itself through its own `build.sh`, which passes them; the
checkout stays pinned and unmodified.

**After every `vcs import` that touches `livox_ros_driver2`, regenerate its
`package.xml`** — the repo ships only `package_ROS1.xml` / `package_ROS2.xml`,
`build.sh` is what copies one into place, and `package.xml` is in that repo's
own `.gitignore`, so a re-import deletes it and every ament package in the
workspace fails to configure:

```bash
cp -f src/third-party/livox_ros_driver2/package_ROS2.xml \
      src/third-party/livox_ros_driver2/package.xml
```

`Livox-SDK2` deliberately has **no** `package.xml` — colcon picks it up as a
plain CMake package via `project(livox_sdk2)`. Adding one would turn it into an
ament package and require a build type and dependency list it does not have.

**Every colcon run prints an `ERROR` for `behaviortree_cpp_v3` — ignore it.**
It reads `Exception in package identification extension 'ament_cargo' in
'src/third-party/behaviortree_cpp_v3' … Only one <build_type> element is
permitted.` and comes with a traceback, but nothing is broken: BT.CPP's
`package.xml` has two `<build_type>`s gated by `$ROS_VERSION` conditions (valid
format 3), and `colcon-ros-cargo` 0.2.0's identification calls
`get_build_type()` before evaluating them. colcon then falls back to the ROS
identification, which lists it as `ros.ament_cmake` and builds it normally.
Upstream fixed this in colcon-ros-cargo PR #32 (2024-12) but has not released it
(PyPI is still 0.2.0). Main at `dc26ba5` was checked on 2026-10-11: with it, the
`ERROR` is gone, the Rust packages show up as `ros.ament_cargo`, and
`syncai_robot_state` builds. Bump the pin (`COLCON_CARGO_PIP` in the
`Dockerfile`) when a 0.2.1 / 0.3.0 ships, not to a git SHA: that `ARG` also
feeds `rust-underlay`, so changing it invalidates that stage's cache. Do not
silence the error by editing BT.CPP's `package.xml` either, because the
checkout is pinned and unmodified and the next `vcs import --force` would
revert the edit.

## Running the stack

The stack comes up as a **byobu session, one per operating mode**, built by
`NodeManager` in
`src/syncai_sys_manager/syncai_sys_manager/managers/node_manager.py`. There is no
launcher script any more — `scripts/byobu_session*.sh` and the
`scripts/byobu_session.py` that replaced them are both gone, and so is
`scripts/tailog.sh`. A ROS node is the entrypoint instead, which is what lets the
operator console bring the stack up and switch modes remotely.

An operating mode **is** a session:

| `RobotMode` | Session spec | byobu session | Log subtree |
|---|---|---|---|
| `AUTO` = 2 (default) | `config/sessions/start_nav.yaml` | `syncai-dev` | `log/stack/<robot_id>/<name>/` |
| `MANUAL` = 1 | `config/sessions/start_mapping.yaml` | `syncai-mapping` | `log/stack/<robot_id>/mapping/<name>/` |
| `CALIBRATION` = 3 | `config/sessions/calib_lidar_camera.yaml` | `syncai-calib` | `log/stack/<robot_id>/calib/<name>/` |
| `MAINTENANCE` = 0 | — | none | — |

`MAINTENANCE` is not a mode you switch *into*; it is what `get_mode` reports when
no session exists. The live mode is never stored — it is derived on demand
from which session byobu actually has, so it stays correct across a `sys_manager`
restart and does not drift when someone builds or kills a session by hand.
`setup_session()` adopts a running session rather than rebuilding it, which is
what makes restarting `sys_manager` mid-mapping-run harmless.

`CALIBRATION` = 3 (2026-10, `config/sessions/calib_lidar_camera.yaml`, session
`syncai-calib`, logs under `calib/`) is bringup with the livox driver on
`PointCloud2` (`livox_xfer_format:=0`) plus the camera feeder, for recording
lidar-camera calibration bags — nothing else, not even `driver_manager` or
`robot_state`, so lie the robot down before switching to it. It is a mode
because it holds the same lidar UDP ports and camera the other two do, and
`switch_mode`'s kill-all-then-build is what keeps them exclusive. The value is
new in `SyncAI-Robot-Interface`'s `RobotMode`; the backend's mode route accepts
`MANUAL` / `AUTO` only, so it is a `ros2 service call` for now. The procedure
it serves — board views for the intrinsics (AUTO is enough for those), bags
and a rotation-only fit for the extrinsic — is
`docs/lidar-camera-calibration.md`, with its tools in `scripts/calibration/`
and the `lidar-camera-calibration` skill under `.claude/skills/`.

Three services, namespaced under `robot_id`:

```bash
ros2 service call /<robot_id>/get_mode     syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode  syncai_common/srv/SwitchMode "{mode: 2}"
ros2 service call /<robot_id>/restart_mode syncai_common/srv/RestartMode "{}"
```

`switch_mode` kills *every* known session before building the target one (if both
were somehow up, leaving one behind would leave `get_mode` ambiguous), and
refuses to rebuild the mode that is already live — in `MANUAL` that would drop an
unsaved map on the floor, because `pgo_node` (`syncai_mapping`) accumulates its
keyframes in RAM and `save_maps` is the only thing that serialises them. That refusal is why
**starting a new map is not a mode switch**: `pgo/reset_mapping` (the backend's
`POST /api/v1/mapping/reset` is one caller) does it in place instead, with
nothing restarted — see "Out of tree". Rebuilding the live mode's session on
purpose (a wedged stack) is `restart_mode`: same kill-all → build path, refused
in `MAINTENANCE`, with both sessions up, and always in `MANUAL` (no override —
it cannot tell whether the map was saved), so in practice it restarts `AUTO`. The
backend does not call it today. `sys_manager` itself runs
**outside** both specs — it is the robot container's main process
(`command:` in `docker-compose.robots.yml` runs `ros2 launch syncai_sys_manager
sys_manager.launch.py`), so `docker compose up -d robot01` brings the stack up
on its own: `setup_session()` sees nothing running and builds `AUTO`. Add
`sys_manager` back as a window in a spec and `kill_session` would kill the pane
it is running in.

The window list is **data**: `config/sessions/*.yaml` holds windows / panes /
commands / `sleep` offsets / `multilog` names, and `NodeManager` holds the byobu
plumbing (the two shell scripts it descends from shared ~85 lines of identical
bash). The schema is documented in the `NodeManager` docstring (`session`,
`select`, `windows[{name, cwd?, panes[{cmd, sleep?, log?, enter?}]}]`). `sleep`
is where the startup ordering lives, since there is no lifecycle manager. The
nav session's windows, in order: `bringup` → `map_server` (map_server +
keepout) → `localization` (pointlio + localizer) → `lio_bridge` → `plan_ctrl`
(planner + controller) →
`task_runner` → `driver_manager` → `robot_state` → `camera_feeder`. The `keepout` pane
(`costmap_filter_info.launch.py`) writes a blank `map/<name>/keepout.yaml` +
`.pgm` of the gridmap's geometry when the map has none, then serves it — so
every pane of a normal boot stays up, and "Filter mask was not received" in the
planner log (logged once since 2026-10-19, with an info when a mask arrives;
expected on a 3D-only map, where the pane runs idle) means the pane is actually down (until 2026-09-30 it exited on
purpose on a map without a mask, and that warning was the expected noise).
`pointlio` got its own pane
in 2026-09, when `syncai_pointlio` moved in-tree; it used to come up inside the
localizer pane through an `include()`. The localizer pane runs
`syncai_localizer` since the end of that month (log directory still
`localizer`); every pane of both specs is a workspace package now. Neither spec has a `frontend` or a
`backend` window any more — both are served from their own containers, so a
mode switch interrupts neither. That is a real change and not only a tidy-up:
each used to be a pane of the session being killed, which is why every spec
once carried its own dev server, and why a conversion or a recording the
backend was supervising died with a `switch_mode`. Sessions are built
**detached** — no `attach-session`, because the caller is a ROS node with no
TTY.

The two specs are counterparts, not variants. `start_nav.yaml` *localizes*
against an existing map — `syncai_localizer` loads one during construction and
dies without it (map_server only idles without a gridmap since 2026-10-19) — so it cannot build the map it needs;
`start_mapping.yaml` is the other half of that loop: `bringup` → `lio`
(`syncai_pointlio` + `syncai_mapping`, both ours — the latter imported from its
own repo since 2026-10 — and the fork contributes nothing to this session) → `driver_manager` → `robot_state`, and deliberately **no** map_server /
localizer / lio_bridge / planner / controller / task_runner. `robot_state`
is in the mapping session although its pose lookup never succeeds there, for its
`mode` field alone — that is what the console's mode chip reads to know the
switch to `MANUAL` landed. Everything else the console does during a mapping run
(teleop, the live map-cloud hand-off on `pgo/map_cloud_file`, save-map) reaches
these nodes over DDS from the backend's container. Their logs go to separate subtrees, which is what
stops the two specs from interleaving two multilogs into one directory. The 2D / AMCL session was retired along with
`bringup_2d.launch.py`. Neither spec has an rviz2 window (the robot has no
display; `config/rviz2/<robot_id>.rviz` is for running rviz2 from a
workstation). The nav spec's `camera_feeder` window (2026-10-12) is the only
camera window; the mapping spec has none.

There is no log-reading helper. Read a subsystem back by hand — multilog writes
`current` plus gzipped rotations (16 MiB × 10):

```bash
tail -f log/stack/<robot_id>/planner/current
zcat log/stack/<robot_id>/planner/@*.s | less
```

There is no keyboard-teleop window any more: the console's teleop channel
(the backend's WebSocket `/api/v1/robot/teleop` → `cmd_vel`) covers it without a
pane where a stray keypress is a motion command. With no backend running,
`teleop_twist_keyboard` remapped onto `/<robot_id>/cmd_vel` is the fallback. The schema still supports
`enter: false` (pre-type a command without executing it) but no spec uses it
today. Both specs set `select:` to `bringup` because its log is the first thing
to read when the robot comes up — it answers "is the lidar alive?".

Logging is deliberately split: `ROS_LOG_DIR` points at a tmpfs (ephemeral), while
the byobu `pipe-pane` capture is the persistent, gzip-rotated record.

## Out of tree: the backend and the console

Two repositories used to be directories here:

| Repo | Was | What it does |
|---|---|---|
| `chungweeeei/SyncAI-Robot-Backend` | `src/syncai_backend` (removed 2026-09) | FastAPI + rclpy in one process on port **3000**: Temporal task orchestration, task templates and schedules, the map catalogue and both pcd → gridmap recipes, TTS, bag recording, and the teleop / telemetry / point-cloud WebSockets. |
| `chungweeeei/SyncAI-Robot-Frontend` | `src/syncai_frontend` (removed 2026-09) | Next.js: dashboard with the 3D point cloud, `/mapping`, `/maps` + the gridmap editor, `/recordings`, `/tasks`, `/settings`. Talks to the backend and to nothing else on the robot. |

Neither is imported, built or launched from here. The backend runs in its own
container on the robot, with host networking on DDS domain 1, so it discovers
the nodes in `src/` exactly as it did when it was a pane of the byobu session.
The infra it needs — postgres and temporal — is still in this repo's
`docker-compose.yml`, because the robot is where it is deployed.

### What that means when editing this repo

**This stack's ROS surface is the backend's API, one layer down.** Renaming a
service, changing a message field, moving a file inside `map/<name>/` or
changing a node's namespace is a cross-repository change: say so in the commit
message and keep the old shape working until the backend has moved. The surface
it actually uses:

| Here | Used by the backend for |
|---|---|
| `syncai_common` msg/srv/action (`SyncAI-Robot-Interface`) | every call below — it builds against that repo, not this checkout. `SaveMaps`, `ResetMapping`, `Relocalize` and `IsValid` are there since the pgo / hba ports (2026-09), when the fork's `interface` package was retired: every `from interface.srv import …` in the backend had to become `from syncai_common.srv import …`. (`ResetLIO`, `RefineMap` and `SavePoses` are there too, but the backend calls none of them — and the last two have no server since `hba_node` was removed in 2026-10.) |
| `robot_state` topic (`syncai_robot_state`) | the telemetry WebSocket and `GET /api/v1/robot/state` |
| `switch_mode` / `get_mode` (`syncai_sys_manager`) | `POST /api/v1/robot/mode`; its gateway waits 45 s / 70 s for them |
| `scan_wifi` / `connect_wifi` / `wifi_status` (`syncai_sys_manager`) | the network router |
| `cmd_vel`, `set_motion_key` (`syncai_driver_manager`) | teleop WebSocket, STANDUP / LIEDOWN steps |
| `NavigateToPose` on `task_runner` — and, since 2026-10, `syncai_common/action/NavigateToGoal` on `navigate_to_goal`, the same navigation with `error_code` / `error_msg` in the result and `status_msg` in the feedback | the MOVE step of a task. The backend still sends the nav2 action; the new one exists so it can show *why* a MOVE failed and, while it runs, what the robot is doing (`status_msg`, shown beside the step's status), and the old one stays until the backend has moved. **A DOCK or UNDOCK step is the same action** with `behavior_tree: dock.xml` / `undock.xml` (bare name or absolute share path) and the **dock pose** as `pose` for a DOCK — an UNDOCK sends none (its `pose` is ignored since 2026-10-15; it backs out 1 m from where the robot stands); a failure of the docking branch is `DOCK_FAILED` (300), the staging leg's is still 100 / 200 |
| `relocalize`, `relocalize_check`, `initialpose` (`syncai_localizer`; types `syncai_common/srv/Relocalize` / `IsValid`). **Bare names in the `/<robot_id>` namespace** — `/<robot_id>/relocalize`, not `/<robot_id>/localizer/relocalize`; the backend's map gateway calls them bare and records that the prefixed spelling cost it a `stack_not_ready` against a healthy stack | initial pose, and map switch |
| `map_server/load_map` | map switch, and reload after a re-conversion |
| `planner_server/Octo3D/load_map` (`syncai_global_planner`, `feature/octomap-planner-3d`; `nav2_msgs/srv/LoadMap`, `map_url` = `map/<name>/octomap.bt`) | the map switch (`POST /api/v1/maps/{name}/activate`), on the backend's `feature/octo3d-map-switch` (2026-10-19; not merged): the `octomap.bt` for a 3D map, `""` (unload) for a 2D one — without either, `Octo3D` keeps the previous map's tree and the task_runner its 3D trees. Blocks until the new surface map is built (seconds; size the timeout for it) and its success is a result; a missing file is refused with the current map kept, any other failure leaves no map. An **empty `map_url` unloads** the 3D map — the call a switch to a 2D map must make, or the task_runner keeps running the 3D trees on the old octree. On a 3D-only map `map_server/load_map` fails, which is expected there |
| `auto_localize` (`syncai_localizer`; `syncai_common/srv/AutoLocalize`, bare `/<robot_id>/auto_localize`) | nothing since 2026-10-15 (the backend's `POST /api/v1/localization/auto` called it until then, and now calls `reset_odometry` with `skip_seed`, which ends in this same search). Blocks up to the sum of its two deadlines (15 s by default) |
| `reset_odometry` (`syncai_localizer`; `syncai_common/srv/ResetOdometry`, bare `/<robot_id>/reset_odometry`) | `POST /api/v1/localization/auto` (2026-10-15), with `skip_seed: true`: reset Point-LIO, then the hint-free search on the new run — so finding a kidnapped robot always zeroes the odometry, and `odom → base_link` jumps back to the origin once. Blocks up to 17 s (32 s with `global_fallback`, 27 s with `skip_seed`; the backend waits 40 s), and the caller must have stopped the robot first — nothing enforces it. `skip_seed` and the response's `z` / `fitness` / `inlier_ratio` / `candidates` are new fields of `SyncAI-Robot-Interface`, so the backend needs that pin first |
| `pgo/start_mapping`, `pgo/save_maps`, `pgo/reset_mapping`, `pgo/mapping_status`, `pgo/map_cloud_file` (`syncai_mapping`; types `syncai_common/srv/StartMapping` / `SaveMaps` / `ResetMapping`, `syncai_common/msg/MappingStatus`) | begin a run, save a map (which ends the run), start a new one mid-run, report the run state (latched), hand over the live merge (a PCD in the shared `/dev/shm/syncai_pgo/<robot_id>`, named by the notice — needs `ipc: host` on both containers; `pgo/map_cloud` itself is rviz-only now) |
| `pointlio/body_cloud` (`syncai_pointlio`) | the live cloud WebSocket |
| `config/instances/robotNN.ini` (`[map] name`, `[initial_pose]`) | the only file in this repo the backend **writes** |
| `map/<name>/` on disk | the map catalogue: `map.pcd`, `poses.txt`, `patches/`, `gridmap.*`, and optionally `keepout.yaml` + `keepout.pgm` (the forbidden-zone mask, read by `filter_mask_server` since 2026-09; same yaml + image format as `gridmap.*`, black = keepout, drawn as the forbidden area only — the `KeepoutFilter` inflates it itself with the costmap's footprint and inflation parameters since 2026-10, because costmap filters run after inflation; a margin drawn into the mask is applied twice). The nav session **writes a blank one** (all unknown, same geometry as `gridmap.*`, both files tmp + rename) when a map has none at boot, so a map that has been booted into always has the pair — the canvas the editor draws on. The backend does not write the mask today; when it does, `filter_mask_server/load_map` with `map_url: map/<name>/keepout.yaml` is the reload path — no session restart — and a map switch needs the same call, since the nav session derives the path once at boot (a never-booted map has no mask yet; the caller then writes the blank one itself, grey not white) Since 2026-10 a save with `save_patches` also gets `map_clean.recipe.json` (`converting` → `ok` / `failed`, with `started_at`, the params and the measurements; no map name, no absolute path), and **`map.pcd` is replaced** by `clean_map`'s copy without people and one-off returns when it reaches `ok` — atomic rename, minutes after the save returns; `patches/` stay raw |

**Facts about this stack the backend depends on.** These belong here because
they are properties of the nodes in `src/` and `src/third-party/`, not of the
backend:

- **`NavigateToGoal` ABORTED with `error_code` NONE is "superseded or not
  started", not a navigation failure.** A preempting goal aborts the old one
  with a default result (`SimpleActionServer::accept_pending_goal`), and a
  goal rejected before the tree runs (the other navigator busy, the
  `behavior_tree` file missing) never reaches the navigator's
  `goalCompleted()`. Only a tree that ran and failed carries `PLAN_FAILED` /
  `FOLLOW_PATH_FAILED` / `DOCK_FAILED` / `UNKNOWN`; CANCELED carries NONE too.
  The three branch codes are the base of nav2 Iron's 1xx / 2xx ranges (and
  our 3xx) and are never renumbered — switch on the hundreds.
- **A successful dock ends standing on the dock in MPC; so does a cancelled
  one.** `dock.xml` switches to MPC before the approach and restores RL
  (`set_motion_key` key `1`) only when the switch or the approach fails. It
  does **not** lie the robot down (2026-10-15; from 2026-10-14 it sent key
  `2` and waited 4 s after the approach), so a task that wants the robot
  resting on the contacts needs the backend's **LIEDOWN step after the DOCK**
  again; a cancel halts the tree before any restore. The
  stack does not switch back by itself: the next step is `undock.xml`, or the
  backend sends key `1` before a MOVE. `undock.xml` is
  the reverse: from standing on the dock in MPC (a robot laid down after the
  dock needs the backend's STANDUP first — the tree has no stand-up step
  since 2026-10-15) it backs out blind 1 m
  from where it stands, and **ends in RL** facing the dock — the way to
  leave the dock, because a MOVE straight off it starts with the footprint on
  the dock's lethal cells. `dock.xml` takes the **dock pose** as `pose`;
  `undock.xml` ignores `pose` (2026-10-15), so a re-sent undock backs out a
  further metre from wherever the robot is.
- **A MOVE after a DOCK does not undock by itself; "the robot is docked" is
  the backend's state.** Decided 2026-10-15: auto-undock (STANDUP if it lay
  down, then UNDOCK, before the MOVE) belongs in the backend's workflow,
  which already sequences DOCK / LIEDOWN / MOVE and persists its state. A
  `task_runner`-side flag was rejected: it cannot see a LIEDOWN the backend
  sends straight to `set_motion_key`, it lives in RAM and is lost on a
  restart, and it would hide a blind 1 m back-out inside a MOVE. The three
  trees stay stateless, one action each.
- **The last metre is blind.** The docking controller reads no costmap (the
  dock is itself lethal in it); avoidance is the staging leg. Something that
  steps between the robot and the dock after staging is pushed against until
  the controller's progress checker (30 s) or `failure_tolerance` ends the
  goal. The switch to MPC is confirmed from the controller's own report
  (MPC = 5 on `mode` data[1], 2026-10-14) and followed by 3 s of settling
  (2 s until 2026-10-15, in both trees); a
  controller that never reports it within 5 s fails the dock (`DOCK_FAILED`,
  RL restored).
- **A dock goal's first use waits for `dock_to_pose` and `set_motion_key`.**
  The docking trees are not the default tree, so their nodes are constructed
  at the first dock goal; a missing server then fails *that goal* (ABORTED,
  `error_code` NONE — "not started") rather than task_runner.
- **`relocalize` returning success is a receipt, not a result.** Registration
  runs async at 5 Hz with no deadline; `relocalize_check` is the only surface
  that reports the outcome. `RobotState.localization_valid` is TF-presence only
  and reads true against a map the robot was never localized in — do not "fix"
  it into a quality signal without a source for one.
- **`relocCB` takes the request's raw 6-DOF**, bypassing the `applyPlanarGuess`
  tilt correction that the `initialpose` path applies. A relocalize call must
  therefore be followed by an `initialpose` publish, or the tilted lidar mount
  leaves the localizer retrying a flat guess forever. This is why the backend's
  map switch does both, in that order.
- **`map_server` and the localizer open their files during construction**; the
  localizer dies without its `map.pcd`, and `map_server` throws on an
  unreadable gridmap but comes up idle (`yaml_filename: ""`, no `map`
  published) when `[map] map` names one that does not exist — a 3D-only map,
  2026-10-19 — so the backend's `map_server/load_map` on a map switch to such a
  map fails, and must not be read as a broken stack. That is why the nav session cannot build the map it needs,
  and why the active map's directory cannot be renamed or deleted under them.
- **No file inside a map directory names the map or holds an absolute path** —
  `gridmap.yaml` says `image: gridmap.pgm`, `poses.txt` lists bare patch
  basenames. That property is what makes a rename one `os.rename`; a future
  sidecar written by anything here that embeds the path breaks it silently.
- **A mapping run is bracketed by `pgo/start_mapping` and `pgo/save_maps`;
  `pgo/reset_mapping` restarts one in place** (2026-10). `pgo_node` comes up
  IDLE in a mapping session and banks nothing until Start; a successful save
  ends the run (keyframes freed, empty map published, `/dev/shm` cleared,
  IDLE again); the state is latched on `pgo/mapping_status`
  (`syncai_common/msg/MappingStatus`: IDLE / MAPPING / RESETTING). A mode
  switch is not how any of that is done: `switch_mode` refuses to rebuild the
  live mode, and rebuilding `MANUAL` would drop an unsaved map, since
  `pgo_node` accumulates keyframes in RAM. Start and reset run the same
  sequence from opposite preconditions (Start is refused while MAPPING, reset
  and save while IDLE): pause intake, reset the LIO front end over `pointlio/reset`
  (`syncai_common/srv/ResetLIO`, served by `syncai_pointlio`, called by
  `syncai_mapping` — both in this tree since 2026-09, and both `.srv` files of
  the contract in `syncai_common`), rebuilds the pose graph and drops everything at or before
  the boundary the front end reported — pausing first is the whole design, so the odometry discontinuity
  has nowhere to land. The only fallible step runs before anything is
  destroyed: there is no half-reset. **The robot must be stationary**, because
  pointlio re-runs a static, gravity-aligning IMU init and one done in motion
  tilts the new map for its whole life with no error anywhere. Nothing enforces
  that, in either repo.
- **An empty map-cloud merge is a message, not a non-event** — pgo (`syncai_mapping`) sends one
  from `reset_mapping`, `start_mapping` **and a successful `save_maps`** on both
  outputs (an empty PointCloud2 on `pgo/map_cloud`, a `points: 0` notice on
  `pgo/map_cloud_file`), and a consumer that skips it keeps showing the old
  map. The save's is also what empties `/dev/shm/syncai_pgo/<robot_id>`; the
  backend never deletes those files.
- **`map.pcd` changes after `save_maps` returns** (2026-10). It is the raw
  save until `map_clean.recipe.json` says `ok`, then `clean_map`'s copy
  without people and one-off returns (atomic rename; a reader that already
  has it open keeps the old file). A gridmap converted while the sidecar says
  `converting` was made from the raw map and should be redone when it flips —
  the backend does not do that yet. A hard kill (OOM, `docker stop`) leaves it
  at `converting` with the raw map in place: age it out by `started_at`. There
  is no ROS service to re-clean; `ros2 run syncai_mapping clean_map
  map/<name>` is the rerun, idempotent because it starts from `patches/`.
- **The live merge is a file, not a topic payload.** `pgo/map_cloud` still
  carries the PointCloud2 for rviz, but a large site's merge is 16-45 MB and
  CycloneDDS over UDP on `lo` cannot deliver that through the kernel's default
  208 KB receive buffer (`net.core.rmem_max`) — fragments drop and a BEST_EFFORT
  reader loses every sample, so the preview stopped once the map grew. pgo
  (`syncai_mapping`; the README there has the full mechanism)
  writes the merge to `/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd`
  (tmp + rename, newest two kept) and publishes a ~200 B JSON notice on
  `pgo/map_cloud_file` (RELIABLE, TRANSIENT_LOCAL). Both compose services run
  `ipc: host` so that path is the same file on both sides; without it the
  notices arrive and every read is ENOENT.
- **There are two pcd → gridmap recipes and no automatic pick between them**
  (z-band slicing with a pose-connectivity filter, the default; and
  traversability segmentation, which produces no unknown cells). Both live in
  the backend now. The decision not to choose automatically is recorded there:
  a wrong z-band map is recoverable, a wrong traversability map is permanently
  walled, and the bbox metric tried in 2026-08 misrouted all three
  conference-hall saves. Do not add a heuristic here either.
- **`switch_mode` no longer kills the backend.** It used to be a pane of the
  session being torn down, which is why the backend writes a conversion's status
  to a sidecar on disk rather than keeping it in memory, and why bag recording
  deliberately shared the session's process group. Both of those are now
  belt-and-braces rather than load-bearing — do not remove them from that repo
  on the strength of this change alone.

### The robot mesh is still built from here

`scripts/urdf2glb.py` bakes `src/syncai_bringup/description/G23.urdf` into a GLB
the console loads as `public/models/g23.glb`; regenerate and hand it over after
editing the URDF. Two invariants are load-bearing on the console side — GLB node
names must equal URDF link names (the canvas looks links up by name to apply
joint angles) and coordinates stay ROS Z-up. This is the only reason the
`Dockerfile` still installs Node.js.

### Reading the route names in this file

Package READMEs and session specs still name REST routes (`POST /api/v1/maps`,
409 `map_active`, …) where that is the clearest way to say which operator action
reaches a given service. Treat them as pointers into the backend repo, not as
something to verify or edit here.

## Infrastructure

- **`docker-compose.yml`** — shared services: `postgres` (5432, bind-mounted to
  `./data/postgres`), `pgadmin` (5050), `temporal` (7233), `temporal_ui` (8081).
  **None of the four has a consumer in this repo** — they are the backend's
  database and workflow engine — and they stay here because the robot is where
  they are deployed and `docker compose up -d` on the robot has to bring them
  up. Moving them to the backend repo is a live decision, not a cleanup: doing
  it means moving `./data/postgres` with them. There is **no** `mediamtx`
  service: the camera leaves the robot through `syncai_camera_feeder` and the
  WebRTC worker (see "Camera"). The host-side `scripts/publish_camera_crop.sh`
  that pushed it over RTSP to a MediaMTX on a remote server was removed in
  2026-10, with its `publish_camera_crop.env`.
- **`docker-compose.robots.yml`** — robot containers, pulled into
  `docker-compose.yml` via `include:`. One service, `robot01` (`network_mode:
  host`, loopback-only unicast DDS), with **no compose profile**: a plain
  `docker compose up -d` starts infra and robot together. It used to sit behind
  a `real` profile that existed only to be mutually exclusive with the Isaac Sim
  fleet's `sim` profile (`*-sim` services dual-homed on a `syncai-lan` macvlan).
  The fleet was **removed** — recover it from git history rather than
  re-deriving it if the simulator comes back — and the profile with it, because
  on the robot itself its only remaining effect was to make `up -d` silently
  skip the robot. To start infra alone, name the four services. Robot containers also
  bind-mount the host D-Bus socket (so `nmcli` reaches the host NetworkManager —
  needs `apparmor=unconfined` + sudo) and the avahi socket's directory
  `/run/avahi-daemon` (so `libnss-mdns` resolves `*.local`; the directory rather
  than the socket file, because a file bind mount goes stale when
  `avahi-daemon.socket` restarts), run with `runtime: nvidia` and `init: true` (tini as PID 1, to reap the
  `clean_map` a mode switch orphans; takes effect on a container recreate), and pass through the cameras
  as `/dev/syncai/camera0` / `camera1` (stable udev symlinks from
  `src/syncai_sys_manager/udev/99-syncai-devices.rules`, keyed on serial so the
  two cameras cannot swap on reboot) plus `/dev/snd` (for TTS, which now plays
  from the backend's container — this one keeps the passthrough because `aplay`
  in here is how the speaker is diagnosed), with the host
  `video` and `audio` gids added (`VIDEO_GID` / `AUDIO_GID`, default 44 / 29,
  overridable in `.env`) and `device_cgroup_rules` for the video4linux (81) and
  ALSA (116) majors so a USB replug onto a new minor does not need a container
  recreate. X11 is passed through (`/tmp/.X11-unix` rw, `/tmp/.docker.xauth`
  ro, `DISPLAY=:1`; the file header has the xauth-cookie recipe). `ROS_LOG_DIR`
  points at a 128 MiB tmpfs. `TEMPORAL_ADDRESS` / `POSTGRES_HOST` are **not** in
  this service any more — they left with the backend — but the rule behind their
  value still applies to whatever container does talk to the infra: under
  `network_mode: host`, compose service names do not resolve, so it is
  `127.0.0.1` and the host-published ports.
- **`ROS_DOMAIN_ID` is a literal `"1"` in compose, not `${ROS_DOMAIN_ID:-1}`.**
  Compose interpolation gives the invoking shell precedence over `.env`, and
  the Jetson host's `~/.bashrc` exports `ROS_DOMAIN_ID=69`, so an overridable
  default silently resolved to 69. The flip side: anything started on the
  **host** (rviz2, `ros2` CLI) inherits 69 and will not see the containers
  until that export is changed.
- **CycloneDDS** is the RMW, configured by hand: `config/cyclonedds.xml` —
  loopback only (`lo`), `AllowMulticast=false`, one explicit unicast peer at
  `127.0.0.1`. It is the single config left; `cyclonedds_standalone.xml` went
  away with the sim/macvlan split. It pins `<Domain Id="1">`, so the domain is
  **not** inherited from `ROS_DOMAIN_ID` — changing the compose env alone
  silently leaves DDS on domain 1 and nothing discovers anything. Change both.
  Gotchas that have bitten before: a single interface with `multicast=false`
  disables multicast **globally**; peer pings only use the top-priority
  interface; unicast `<Peers>` require the remote side to have
  `ParticipantIndex=auto`.
- tmpfs `mode:` in compose must be written as an octal literal (`0o1777`) — a
  bare `1777` is parsed as decimal.

## Conventions

- C++ formatting is pinned by `.clang-format` (ROS 2 style, 100 columns).
  Python is linted by the root `ruff.toml` (py310, rule set pinned on purpose
  — no package under `src/` carries its own ruff/pyproject config, so this file
  governs `syncai_sys_manager` and the loose `scripts/` tools, which is all the
  Python left here; it exists because a ruff upgrade's growing defaults produced
  471 spurious in-editor warnings overnight).
- Comments and docs are written in **English**. Comments in this codebase
  explain **why**, often at length, and frequently record a past bug or a
  rejected alternative. Match that density when editing — a bare parameter
  change with no rationale is out of place here. The last Chinese remnants
  (BT plugin comments, `ExecuteTask.action`, the camera script's log strings,
  the `doc/` proposals) were translated in 2026-09; anything new in another
  language is a regression, log strings included. There are **no translations
  in the tree today** — `doc/webrtc-worker-proposal.zh-TW.md` was the only one
  and went with its English original in `3dd5f7c`. The rule if one is ever
  added back: a `*.zh-TW.md` sits beside an English file that stays the
  canonical copy and carries the content, and you edit the English one first. A
  zh-TW file that has drifted is worse than none, so either update both or
  delete the translation.
- `build/`, `install/`, `log/`, `data/`, `.env`, `record/` (hand-recorded
  rosbags), the whole of `/map/` (LIO output: `map.pcd`, `poses.txt`,
  `patches/`, generated `gridmap.*`), `/models/` (TTS weights, which nothing
  here downloads any more) and `/config/calibration/` (each robot's own camera
  intrinsics, made on the robot — see `docs/lidar-camera-calibration.md`) are gitignored, as are `src/syncai_common/`,
  `src/syncai_mapping/`, `src/syncai_driver_manager/`, `src/syncai_robot_state/`,
  `src/syncai_lio_bridge/`, `src/syncai_camera_feeder/` and `src/syncai_backend/` — the first six because
  they are materialised by `vcs import < dependencies.repos`, the last so a clone of the backend kept there for development
  can never be committed back in. `.env` holds secrets — never commit it.
- `scripts/`: `attach.sh` (host-side: attaches to whichever byobu session is
  live in the container — the session name follows the mode, so a hardcoded
  alias is wrong half the time), `build.sh` (in-image: the build
  steps behind `docker-compose.build.yml`, see Build), `urdf2glb.py` (robot mesh for the
  console's 3D view, see "Out of tree"), `calibration/` (camera intrinsics and
  lidar-camera extrinsic, 2026-10: see `docs/lidar-camera-calibration.md`;
  `collect_board.py` / `snapshot.py` need rclpy, the rest only numpy + OpenCV +
  PyYAML and also run on the host), and `release/` (`build_images.sh` / `save_images.sh` /
  `load_and_up.sh` / `.env.example` — an offline release bundle for the
  customer IPC that is currently **non-functional**: it needs the removed
  production Dockerfile stages and a `docker-compose.prod.yml` that is not in
  the tree, and its `.env.example` says `ROS_DOMAIN_ID=0` while the stack pins
  domain 1). `skills-lock.json` at the root is Claude Code tooling metadata,
  not stack config.
- `docs/` holds operating procedures — one today, `lidar-camera-calibration.md`
  (2026-10: camera intrinsics and the camera → lidar extrinsic, the robot01
  results, and why each step is there). Plans have passed through it and been
  deleted once done (`mapping-dynamic-removal.md`, `planner-params-exposure.md`,
  in `807861a`). There is no `doc/` any more: its five agent / MCP design
  proposals went in `0adf8da`, the WebRTC-worker proposal in `3dd5f7c` (that
  work lives in `chungweeeei/SyncAI-WebRTC-Worker`), and with them the FAST-LIO2
  design notes (`fastlio2-pgo-pipeline.md`, which
  `config/sessions/start_mapping.yaml` still cites by section number, noting the
  file is gone). Check git history for any of them.

## Tests

`syncai_sys_manager` has `test_wifi_manager.py`; the C++ packages have the
ament linter tests that come with the package templates plus three gtests,
all ROS-free: `syncai_global_planner`'s `test_surface_map`, and since
2026-10-20 `syncai_util`'s `test_stair_mode` and `syncai_costmap_2d`'s
`test_terrain_grid`. That is the whole suite in this repo. The substantial one — ~40 pytest files covering every REST
router, the Temporal workflow / activities / worker, both pcd → gridmap recipes,
the map catalogue, TF and the telemetry / point-cloud streams — went with
`syncai_backend` in 2026-09 and runs in that repo, against the interfaces from
`SyncAI-Robot-Interface`.

**CI** (`.github/workflows/ci.yml`, 2026-10) runs two jobs on every push to
`main` / `dev` and every PR. `lint` (bare runner): `ruff check` against
`ruff.toml`, and clang-format 14 over the C++ lines the push or PR *changed* —
not whole files, because ~90 of the tracked C++ files predate `.clang-format`
and do not pass it; a wholesale reformat is its own commit, after which the
check can widen. `build` (an **arm64** runner, because the robot is a Jetson):
the `dev` image, pulled from `ghcr.io/<repo>:dev-<Dockerfile sha256>` and
built (Actions layer cache, then pushed) only when no run has built that
Dockerfile yet (2026-10-20; fork PRs build without pushing), both `.repos` files
imported, `scripts/build.sh` through `docker-compose.build.yml` (the same path
a developer uses), then `test_wifi_manager.py`, then a **smoke test**
(`.github/scripts/smoke_test.sh`): every pane of `start_nav.yaml`,
`start_mapping.yaml` and `calib_lidar_camera.yaml`, read from the specs with their `sleep` offsets, started
against a generated 5 × 5 m fixture map with `robot_id: ci_smoke`, and failed if
any launch has exited or logged a crash after 45 s — the check for a
Release-only undefined symbol at plugin load (PR #32) that a green build misses.
Since 2026-10-19 the nav spec runs a second time on a **3D-only fixture**
(`map/ci_smoke_3d/`: `map.pcd` + an `octomap.bt` written by
`octomap_planner_3d_cli --synthetic two_floors --write-bt`, no `gridmap.yaml`),
switched to by rewriting the INI's `name:` line in place (so the workflow
mounts it read-write), and that pass must also show `map_server` and the
`keepout` pane starting idle, `Octo3D` building its surface map and the
task_runner switching its goals to the `*_3d.xml` trees — a
3D-only map crashed `map_server` until `397f264`, which the 2D pass never saw.
`camera_feeder` is skipped (no camera); nothing else needs hardware to stay up.
It refuses to run outside CI or against another INI, because it starts
`driver_manager` too. The ament linter tests are not run — they were never
enforced and would fail. The first run, and any run after
a change to the gtsam / sophus / livox / rust-underlay stages, rebuilds GTSAM
and the underlay (most of an hour); later runs pay for colcon only.

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
