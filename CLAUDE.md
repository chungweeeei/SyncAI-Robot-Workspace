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
   controller, BT navigator) were re-implemented as plain `rclcpp::Node`s.
   There is no lifecycle manager — every node comes up active. This means
   **startup order matters** and is handled by `sleep` offsets in the byobu
   session specs (`config/sessions/*.yaml`).
2. **Everything is namespaced by `robot_id`.** A single DDS domain can host
   several robots, so node namespaces, topics, and TF frames are all prefixed.
   See "The robot_id convention" below — this is the single most common source of
   mistakes when editing launch files or params.
3. **The operator API is not in this repo.** `syncai_backend` lives in
   `chungweeeei/SyncAI-Robot-Backend` and `syncai_frontend` in
   `chungweeeei/SyncAI-Robot-Frontend`. Every REST route, WebSocket and 409
   code named in this file is documentation of a *consumer* of this stack, not
   of code you can edit here — and changing a service, topic, message or
   map-directory layout the backend calls is a cross-repository change. See
   "Out of tree".

`README.md` covers clone/build/run from a user's perspective. This file covers
the conventions and gotchas you need to edit the code correctly. Every package
under `src/` also carries its own `README.md` with its parameter tables, topic
names, package-local gotchas and the history behind its values — read it before
editing that package, and update it in the same change when you move a
parameter or a topic. This file keeps the current state and the reasons that
constrain an edit; dated history belongs in the package READMEs and git log.

## Package map

### Navigation (C++, nav2 port)

```
NavigateToPose (nav2_msgs) / NavigateToGoal (syncai_common) → syncai_task_runner   (BT navigator; behavior_trees/*.xml)
                                   │
                compute_path_to_pose│follow_path
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
          syncai_global_planner                    syncai_controller
       (SmacPlanner2D / Octo3D)          (MPPI / RPP plugins)
                    │                              │
                    └────── syncai_costmap_2d ─────┘
                       (global costmap / local costmap,
                        layered plugins + keepout filter)
```

**This branch (`feature/octomap-planner-3d`) runs two map types.** A map is
2D (`gridmap.yaml`) or 3D (`octomap.bt`), never both. The planner latches
`planner_server/Octo3D/map_available`, and that one signal picks, per goal,
the task_runner's trees (`move.xml` / `dock.xml` = `dev`'s, byte for byte, or
their `*_3d.xml` siblings) and the local costmap's sensor layer (obstacle or
terrain), so a map switch needs no restart. 2D: `GridBased` (Smac) + MPPI on
the obstacle layer, as on `dev`. 3D: `Octo3D` + MPPI on the terrain layer, with
stairs climbed in stair mode. One controller (MPPI) serves both; every 3D
branch in it is gated on the path having heights (`pathHasHeight`), so a 2D
path takes `dev`'s code unchanged — keep it that way. Run on robot01 end to
end: 3D navigation, stair climbing, the 2D ↔ 3D map switch through the
backend, and a 2D regression with the MPPI tuning.

| Package | Role |
|---|---|
| `syncai_nav_core` | Header-only abstract plugin interfaces (port of `nav2_core`). `GlobalPlanner` adds `usesCostmap()` and `isPathValid()` for planners that do not plan on the costmap (`Octo3D`). |
| `syncai_util` | Shared helper library: geometry, occupancy-grid values, odometry window, `SimpleActionServer`, node / robot utils, `path_height.hpp` (`pathHasHeight` / `lookupGroundZ`) and `StairMode`, the stair state machine both 3D-aware controllers run. A `SHARED` target, not header-only, so a change here rebuilds every nav package; it depends on `syncai_common`, so the interface checkout is needed to build anything above it. `test/test_stair_mode.cpp` is ROS-free through `configure()`'s callbacks. |
| `syncai_common` | Shared msg / srv / action interfaces — **not in this repo** (see below). Every interface the backend calls is there, and every server of one is in this workspace: `StartMapping` / `SaveMaps` / `ResetMapping` / `MappingStatus` (`syncai_mapping`), `ResetLIO` (`syncai_pointlio`), `Relocalize` / `IsValid` / `AutoLocalize` / `ResetOdometry` (`syncai_localizer`). `RefineMap` / `SavePoses` have no server (the fork's `hba_node` was removed). `NavigateToGoal` is `nav2_msgs/NavigateToPose` plus `error_code` / `error_msg` in the result (`FOLLOW_PATH_FAILED` 100, `PLAN_FAILED` 200, `DOCK_FAILED` 300, `UNKNOWN` 1 — nav2 Iron's ranges) and `number_of_replans` / `status_msg` (one English sentence of what the navigator is doing) in the feedback. `DockToPose` is the last metre onto or off a dock, served by `controller_server` on `dock_to_pose` and called only by the `DockToPose` BT node; its 3xx codes sit inside `DOCK_FAILED`, direction is the `controller_id` (`Docking` / `Undocking`), and its goal is **planar** (`frame_id` — empty → `map` — / `x` / `y` / `yaw`), because a hand-written quaternion is what a CLI test goal gets wrong. |
| `syncai_costmap_2d` | Costmaps with layered plugins (static / obstacle / terrain / inflation / keepout filter). The local costmap loads both an obstacle layer and a **`TerrainLayer`**, and exactly one runs, by each layer's `map_type` (`"2d"` / `"3d"`; `"any"` by default) against `planner_server/Octo3D/map_available` (`MapTypeGate`). Never both: they disagree on a ramp. `TerrainLayer` is a rolling 2.5D grid of the lidar's returns (`TerrainGrid`, ROS-free, `test/test_terrain_grid.cpp`) judged with Octo3D's `SurfaceMapParams`: each cell's ground is the 0.25 quantile of 10 s of returns (the MID360's −7° lower FOV never sees the floor within ~3.9 m, so the floor under the robot is remembered from further off); a return 0.1–0.6 m above it is an obstacle — for a cell with too few returns for a ground of its own, above the nearest known ground within `sparse_ground_radius` (0.4 m) that the robot can walk onto from its own cell (never over the path's seeds, which reach a stair's treads), else the robot's own, or a slope's just-seen cells read as a wall; a return more than 0.75 m (`body_clearance` + `max_ground_lift`) over its cell's lowest is overhead and never stored, or a stair's underside becomes the ground and boxes the robot in; a surface not reachable over ≤ 0.17 m steps is an island, a larger step a ledge or drop-off, a disc steeper than 20° a slope; a ramp and a kerb are free, and a **stair is lethal, with nothing opening it** — stair mode climbs without the costmap. **Ray clearing**: a sample is removed from its cell when a later ray passes within 0.05 m of its height, so a departed person leaves the ground estimate too; a sample at the cell's ground only by a ray passing *under* it, and nothing within `ray_clear_end_margin` (0.6 m) of a ray's end, where it grazes its own surface. Where no ray comes, `obstacle_memory` (2 s). The planner's `plan` only seeds the reach flood. A costmap clear drops the costs and keeps the returns. Unlike upstream, `KeepoutFilter` **inflates the mask itself** with the costmap's inscribed radius and its `InflationLayer`'s radius / decay, because filters run after inflation — so a mask is the forbidden area only. |
| `syncai_global_planner` | `ComputePathToPose` action server (package renamed from `syncai_planner`; the node `planner_server`, its actions / services / topics and the `planner` log directory kept their names — delete a stale `build/syncai_planner` + `install/syncai_planner`, or the old pluginlib names keep resolving to the stale `.so`). Four planners are built (NavFn, StraightLine, SmacPlanner2D, OctomapPlanner3D); two are loaded: **`GridBased`** (Smac, the 2D trees) and **`Octo3D`** (OctomapPlanner3D, the 3D trees), so an empty `planner_id` is an error. Wall clearance comes from the **global costmap's inflation band** (`inflation_radius` 0.8 / `cost_scaling_factor` 1.5), not from the smoother, which has no cost term and cuts corners toward walls the more smoothing it is given; `cost_travel_multiplier: 1.0`, because 2.0 bowed paths along the inflation gradient. The last plan is re-sent on `plan` at 1 Hz, trimmed to start at the pose nearest the robot, because the BT plans only when the path is blocked or the goal changes; the task_runner calls `clear_plan` on a cancelled or succeeded navigation, and a failed one keeps its route up to show where it failed — until no navigation has planned or checked it (`is_path_valid`) for `plan_idle_clear_time` (30 s), when the planner drops it itself. `is_path_valid` on a `GridBased` path: "blocked" is a centre cell `>= INSCRIBED` **or** a LETHAL cell under the padded footprint's perimeter at the path heading ± 0.35 rad — RPP's own collision test, on the global costmap — so a passage under 0.73 m reads blocked (`is_path_valid.footprint_check: false` restores the centre test). The global costmap runs the **keepout filter** and only `GridBased` reads it; **`Octo3D` has its own zones**, `map/<name>/keepout.json` beside the tree (vector polygons in the map frame, each with a z band naming its floor — one 2D mask cannot tell a site's storeys apart), rasterised onto the surface map as a `KeepoutMask` (core = the polygon's cells on the band's floor, dilated by `keepout.block_radius` 0.25 = `robot_radius`, as `KeepoutFilter` inflates the 2D mask, costed beyond like a wall) that rides beside the dynamic overlay's mask in every `BlockedView`: the goal snap, the search, the shortcut and `is_path_valid` keep off a zone as off a person, a stair link with a point in a core is **cut** (a zone over a stair foot forbids the stair), a start inside a core fails `start_in_keepout`, and `clear_overlay` leaves zones alone. Read with the tree at boot and on every `load_map`; `planner_server/Octo3D/load_keepout` (`nav2_msgs/LoadMap`, `""` = none) reloads an edited file without a rebuild (parse error keeps the current zones); a broken file beside a good tree loads the map without zones and logs an error. **`Octo3D`** (prototype) plans on `map/<name>/octomap.bt` (written by `syncai_mapping`'s `clean_map --octomap 1`; the launch derives the path from `[map] pcd`'s directory). It reduces the tree once, in a background thread, to a multi-layer 2.5D surface map judged with the robot's geometry (`body_clearance` 0.6, `robot_radius` 0.25 = the costmap's inscribed radius, `max_step_height` 0.17, `max_slope_deg` 20 on a plane fitted only when the disc spans more than one step so a kerb is level, `footprint_min_support` 0.8 — up to 20 % of a disc may be never-observed gaps, paid in cost, since the lidar never sees some floor under the robot's own track — plus a hole-fill and a "voxel above must be observed free" rule against ceilings seen from below), and runs A\* over it: a path cannot leave the ground, climb a wall, cross a riser / kerb / drop-off or change floors except over a **stair link**. It does not wait for the costmap (`usesCostmap()` false — a 3D-only map has no gridmap, and its static layer never gets one); a `GridBased` goal on such a map fails at once with `PLAN_FAILED` (`costmap_wait_timeout` 5 s). The start height comes from TF to `<robot_id>/pointlio_body` (the planar `map → base_link` has z = 0); a goal's z picks the floor (`goal_z_mode`). **Stair links** come from the map's `poses.txt`: a stretch of the mapping track leaving one large component (≥ 10 m²) for another ≥ 0.3 m higher or lower is a stair the robot walked; since 2026-10-25 a stair the track never took is also **detected** off the surface map (`stair_detect`: rejected surfaces between two floors' edges, laid from a stand-in walk through their middle by the same straight / flights readers), and those are the only routes. The track says *where* it is; the geometry is read off the map: a single flight (≤ 8 m) is laid as a **row of straight links** across its width, so a route crosses square to the risers (`stairCrossing`); a switchback stairwell as **straight flights joined by explicit 90° landing corners** (`stair_flights`; `StairLink::turns` lists the corners for the controllers); anything else keeps the **walked link**, smoothed and centred in the well, with a startup note saying why. A floor sample on a crossing takes the walked cell's height, not the nearest surface (a dip there reads as a descent to stair mode). **Dynamic obstacles**: a map-frame overlay of `pointlio/body_cloud` (`dynamic_overlay.hpp`, ROS-free) marks every return 0.1–0.6 m above a **traversable** cell (walls' returns fall in the footprint ring and are never marked; returns within `dynamic.self_radius` 0.6 m are the robot's legs), ray-cleared to 3 m, forgotten after 30 s. A plan refuses cells within `dynamic.block_radius` 0.35 m of a mark (the start's disc exempt) and pays for the band beyond. Its TF message filter runs on the **global costmap's callback group** — the thread that takes `/tf`; a thread of its own wedged the planner's TF on robot01. `is_path_valid` on an `Octo3D` path is answered by the planner on this overlay (planner_server remembers the last plan's `planner_id`): the floor stretch the robot is on only, and always valid while it is on a stair, because a replan mid-flight snaps to the wrong floor. `planner_server/Octo3D/clear_overlay` (`nav2_msgs/ClearEntireCostmap`) drops the marks. `planner_server/Octo3D/load_map` (`nav2_msgs/LoadMap`) switches the tree (see "Out of tree"). A missing `octomap.bt` logs one error and latches `map_available` false. The core (`surface_map`, `surface_astar`, `path_postprocess`, `stair_links`, `dynamic_overlay`, `keepout_zones` — the last needs nlohmann-json) is ROS-free; `octomap_planner_3d_cli` runs it offline (`--stats`, `--check-poses`, `--dump-image`, `--plan`, `--dynamic`, `--keepout`, `--synthetic … --write-bt`), also standalone from `plugins/octomap_planner_3d/standalone/`, and `test/test_surface_map.cpp` covers it. |
| `syncai_controller` | `FollowPath` action server, owning the 4×4 m rolling local costmap. Three path controllers are loaded: **`FollowPath` is MPPI** (`syncai_mppi_controller`), **`FollowPathRPP`** is `dev`'s 2D Regulated Pure Pursuit, and **`FollowPathRPP3D`** (`RegulatedPurePursuitController3D`, the A/B on a 3D map) is a **copy** of RPP, not a subclass, because the stair logic runs through `computeVelocityCommands()` — a fix to the shared pursuit / regulation / collision code goes into both files, and its params block repeats `FollowPathRPP`'s values except `desired_linear_vel` (0.8). The goal's `controller_id` picks one; an empty id fails the goal. Both trees name `FollowPath`. On a path with heights, MPPI and RPP 3D match the robot in 3D (robot height = `<robot_id>/pointlio_body`'s z − `lidar_height`, since `base_link` is planar), and the server's goal check adds `goal_z_tolerance` (0.5 m). **Stair mode** (`syncai_util::StairMode`, declared under each plugin's name with the same values): where the path rises 0.3 m within 1 m, stop about `stair_standoff` (1.0 m) before the stair, `set_policy_mode` to HIMLOCO (`syncai_driver_manager` serves it), settle 3 s, climb at 0.5 m/s, and switch back once the path `stair_exit_ahead_dist` (3.5 m) ahead is level — so a U landing keeps the stair policy and it goes back only on the top floor; keep `stair_standoff` + `stair_enter_dist` under that. **Both climb blind**: no costmap or collision check from the hold before the stair to the hold past its top (no obstacle avoidance on stairs, decided). At a planner landing corner (`StairMode::findCorner()`) the robot stops 0.15 m before it and turns in place. A path going **down is walked backwards** (`stair_descend_backwards`; HIMLOCO descends facing the risers): `StairMode::slopeAhead()` reads the direction off the **global plan** `setPlan()` was given — never the local path, whose heights both controllers zero in the planar frame. Both clamp their own acceleration (no velocity smoother in the stack). MPPI's `wz_max` 0.65 and RPP's `desired_linear_vel` 0.60 / `rotate_to_heading_angular_vel` 0.65 are one calibration with `syncai_driver_manager`'s velocity scales — change them together (MPPI's `vx_max` 0.80 matches RPP 3D; unmeasured). The same node serves **`dock_to_pose`** with `syncai_controller::DockingController` (nav2's graceful / opennav_docking control law) loaded twice, **`Docking`** and **`Undocking`** (`allow_backward`, no initial rotation — direction is configuration because the `Controller` interface carries a path and nothing else), paired with `dock_goal_checker`. The dock pose is transformed into the local costmap's odom frame **once** at acceptance, so localizer corrections never move the target. **It reads no costmap** — the dock is itself lethal; avoidance is the staging leg. The two action servers exclude each other on one `timed_` mutex (`CONTROLLER_BUSY` 302); an empty `goal_checker_id` resolves to the **first** of `goal_checker_plugins`. `v_linear_min` 0.05 m/s / `v_angular_min` 0.15 rad/s are the gait's floors under MPC, pre-scale (the angular one is above the 0.03 first measured, which did not hold a sustained turn); the reverse floor and the `Undocking` cap are unmeasured. |
| `syncai_mppi_controller` | **Local obstacle avoidance**: nav2 Humble's `nav2_mppi_controller` (1.1.20) under renamed symbols as a `syncai_nav_core::Controller` plugin. It outputs `cmd_vel` directly; the optimal trajectory is on `local_plan`. Additions over upstream, each because the gait does not do what MPPI assumes: no lifecycle; an **acceleration clamp** (`max_linear_accel` 1.0 / `max_angular_accel` 3.2 — no smoother here); a **walking floor** (`walk_vx_min` 0.30, RL stands still below it); an **in-place turn floor** (`in_place_wz_min` 0.45 pre-scale, RL does not turn below it); a **final rotation** inside `xy_goal_tolerance` (`final_rotation_wz` 0.65, the optimizer skipped, because `GoalCritic` and `GoalAngleCritic` stalled it there); a **direct approach** inside `approach_dist` 0.5 m once the optimizer asks to stand (turn onto the goal, walk at `walk_vx_min`, checked against LETHAL footprints); **3D path matching** and **stair mode** (`climbStairs()` runs first in the cycle and on the flight skips the optimizer for a carrot law, `stair_linear_vel` 0.5 / `stair_lookahead_dist` 0.6, speed changes on an S-curve); the **critics' goal is the navigation goal** (`CriticData::goal`), nullopt until the pruned path reaches it, not the last pose of the pruned path, which a corner brings into `GoalCritic`'s range mid-path; the optimizer's failure is a `PlannerException`, so `failure_tolerance` applies; and a path point is blocked for `PathAlignCritic` / `PathFollowCritic` when the footprint on it touches LETHAL, not only the centre cell. `TwirlingCritic` (5.0) is on against left-right weaving. A path without heights takes none of the 3D code. `model_dt` must equal 1 / `controller_frequency`; a longer controller period throws from `initialize()` and takes the server down. Needs `libxtensor-dev` / `libxsimd-dev` (dev stage). |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`), plus the docking nodes: `DockToPose` (client of `dock_to_pose`; drives to the pose it is given and nothing else — a charger dock will be its own tree whose detection node produces the pose), `SetMotionKey` (client of `syncai_driver_manager`'s gait switch), `ComputeStagingPose` (dock pose 1 m back along its yaw), `ComputeBackOutPose` (1 m behind the robot in its own `base_frame`, no TF), `Wait` (a `StatefulActionNode` timer, preferred over BT.CPP's `Delay`) and `WaitForMotionState` (`SUCCESS` once `mode` `data[1]` equals the requested state — MPC is 5 — `FAILURE` after its timeout). No BT node reads an action's error-code constants; the node → code table is in `syncai_task_runner`. `BtActionNode` / `BtServiceNode` have `report_failure()`; `BtServiceNode::on_timeout()` is empty by default so `ClearEntireCostmap` stays silent. |
| `syncai_task_runner` | The BT navigator (there is no `syncai_bt_navigator`). Serves `nav2_msgs/NavigateToPose` and `syncai_common/action/NavigateToGoal` on `navigate_to_goal` — one `PoseNavigator<ActionT>` template, two instantiations, one navigation at a time across both. The nav2 action is what the backend still sends and stays until it has migrated (rviz's `goal_pose` subscriber with it). Failure reasons: BT action nodes write `failed_node` / `failure_msg` to the blackboard on abort, and `goalCompleted()` maps the node name to `PLAN_FAILED` / `FOLLOW_PATH_FAILED` / `DOCK_FAILED` / `UNKNOWN`; `status_msg` comes from `nav_status_msg`, written by the nodes at each turn of the tree. A goal preempted or rejected before the tree runs is ABORTED with an **empty** result (`error_code` NONE). Trees are selected per goal by `NavigateToGoal.behavior_tree` (bare name → `behavior_trees/`; absolute paths work), empty = `move.xml`, then swapped for its `<stem>_3d.xml` sibling when `map_available` says the map is 3D (`undock.xml` has none); a preempt after the map changed type is refused. **`move.xml`** checks the path once a second against `is_path_valid` and replans only when it is blocked at the same spot for 10 s (`IsPathValid blocked_time`) or the goal changed — replanning at 1 Hz turned the robot back into blockers mid-detour; an `is_path_valid` timeout or a missing pose **keeps** the path. `FollowPath` gets four retries with a 60 s refill and there is no `Wait` recovery, so (retries + 1) × `failure_tolerance` (~16 s) is how long the robot waits for a blocker that closes the way — sized to outlast `blocked_time` so the replan comes first; change the two together. **`move_3d.xml`** differs only in `planner_id="Octo3D"` and its two `ClearEntireCostmap` nodes calling `planner_server/Octo3D/clear_overlay`. **`dock.xml`** (goal `pose` = the dock pose) = `move.xml`'s structure to a staging pose (`ComputeStagingPose`, inside the planner branch so a preempt recomputes it) → `SetMotionKey 5` (MPC) → `WaitForMotionState 5` (5 s) → `Wait 3000` → `DockToPose Docking`, restoring RL (`key 1`) only on a switch or approach failure; it ends **standing on the dock in MPC**. `dock_3d.xml` differs in the staging leg only. **`undock.xml`** (goal `pose` ignored) = MPC → `WaitForMotionState` + `Wait 3000` → `ComputeBackOutPose` → `DockToPose Undocking` → RL, confirmed by `WaitForMotionState 1`. The docking nodes' servers are first waited for at the first dock goal. `bt_loop_duration: 50` ms ticks at 20 Hz and doubles as every BT node's per-tick spin budget (halved) — a latency knob, not just a rate. |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server. `costmap_filter_info.launch.py` (the info server + a second `map_server`, `filter_mask_server`) is the nav session's `keepout` pane: it serves `map/<name>/keepout.yaml`, derived from `[map] map`, and **writes a blank one of the gridmap's geometry when it is absent** (all unknown — a free mask cell would overwrite unknown costmap cells, so white is not a no-op). On a map with **no gridmap** (3D-only) both launches start their nodes **idle** (`yaml_filename: ""`, nothing published, `load_map` still served), and "Filter mask was not received" is expected; an existing `keepout.yaml` whose image is missing also starts the mask server idle. Reload an edited mask with `filter_mask_server/load_map`, no restart. |

### Localization & sensing

| Package | Role |
|---|---|
| `syncai_pointlio` | **The LIO front end.** Point-LIO (output model) over the Livox `CustomMsg` + IMU: node `pointlio_node` at `/<robot_id>/pointlio`, `lio_odom` / `body_cloud` / `world_cloud` / `lio_path`, TF `<robot_id>/pointlio_odom → <robot_id>/pointlio_body`, and `reset` (`ResetLIO`). Every published cloud is **motion-compensated** into the body frame at the end-of-scan stamp (`MapBuilder::deskewToEndBody`); `body_cloud_dense` is the same scan at `dense_filter_num: 2`, published only while subscribed, for `syncai_mapping` — the filter still sees `lidar_filter_num: 6`. Consumers: `syncai_mapping` and `syncai_localizer` (through absolute names their launches inject), `lio_bridge`, both costmaps, the planner's overlay. Two-thread `MultiThreadedExecutor`: lidar / IMU subscriptions on their own callback group so the IMU reader never drops samples behind the timer, while the timer and `reset` share the default one, because `resetCB` takes no lock against the timer. Its launch is the one definition of the node's configuration; nothing `include()`s it. |
| `syncai_mapping` | **The mapping back end — not in this repo** (`chungweeeei/SyncAI-Robot-3D-Mapping`, C++, imported by `dependencies.repos`; edit it there, and keep its own `dependencies.repos` pins in step). Its parameter names were renamed there (`keyframe_delta_m`, `loop_gicp_max_corr_dist`, `map_clean_sparse_min_hits`, …); an old key is silently ignored. `pgo_node` at `/<robot_id>/pgo`: services `start_mapping` / `save_maps` / `reset_mapping`, topics `map_cloud` (rviz) / `map_cloud_file` (the notice the backend reads) / `mapping_status` (latched) / `loop_markers`, TF `map → <robot_id>/pointlio_odom` (identity while IDLE, so the console's live scan survives), and the `/dev/shm/syncai_pgo/<robot_id>` PCD hand-off. It comes up **IDLE** and banks nothing until `start_mapping` (`start_on_launch: true` for bag replay). 3-thread executor because `start_mapping` / `reset_mapping` block on the `ResetLIO` client, so `m_pgo_mutex` is load-bearing. Its `local_frame` must equal pointlio's `world_frame` (not adopted from the odom header). Loop edges are `small_gicp`-registered and planar (x / y / yaw), with fixed sigmas and a per-keyframe gravity prior — the fork's ICP lifted revisited corridors in z. `clean_map`, spawned by a successful `save_maps` with `save_patches`, **replaces `map.pcd`** minutes later without people and one-off returns (state in `map_clean.recipe.json`); it is a **detached process** (own session, nice 10, `oom_score_adj 500`, one per host by `flock`), so it survives the mode switch. `ros2 run syncai_mapping clean_map map/<name>` re-cleans by hand. Needs GTSAM, `small_gicp`, OctoMap and `nlohmann-json3-dev`. |
| `syncai_localizer` | **The relocalizer, nav session only.** Two-stage GICP (`small_gicp`, 0.25 m then 0.1 m) of `pointlio/body_cloud` against `[map] pcd`, broadcasting `map → <robot_id>/pointlio_odom` (frame adopted from the first odom header). **Its names are in the bare `/<robot_id>` namespace** — `/<robot_id>/localizer_node`, `/<robot_id>/relocalize`, `relocalize_check`, `initialpose`, `map_cloud`, `auto_localize`, `reset_odometry` — not `/<robot_id>/localizer/…`; the backend calls the bare names. Registration is motion-gated (`min_update_trans` / `max_update_interval` / `static_blend_alpha`) and capped at 5 Hz; the tuning history is in `params/localizer_params.yaml`. The launch **starts nothing** when `[map] pcd` is missing, on purpose. Three-thread executor: services + `initialpose` on their own group so a multi-second `loadMap` never gaps the TF (`ICPLocalizer::m_target_mutex` exists for that split), and a third because `reset_odometry` blocks on its `ResetLIO` client. `auto_localize` is the kidnap search (poses.txt keyframes × a yaw sweep, `GlobalLocalizer`, its own GICP pair); unlike `relocalize` it **blocks and its success is a result**, with deadlines on both halves. `reset_odometry` recovers a **diverged Point-LIO**: seed from a hint or the last *trusted* registration, pause intake, call `pointlio/reset`, gate on its `last_odom_time`, keep `map → body` continuous, converge, optionally fall back to the global search; `skip_seed` goes straight to the search after a full scan window (what the backend calls). Robot must be stationary. Each round's correction is applied **per direction** by localizability (`localizers/localizability.*`, from GICP's Hessian: a corridor's along-axis correction stays with Point-LIO), gated against the last accepted pose (`jump_gate.*`, 5-round streak cap), on the newest `track_accum_scans` scans; a guess is withdrawn after `guess_timeout_sec`. Thresholds are first guesses, not replay-verified; `test/test_localizability.cpp`. The boot guess is the last trusted pose it saved (`config/state/<robot_id>/last_pose.json`, tmp + fsync + rename, written only once a guess has converged on the loaded map) when that file is for the same map, else `[initial_pose]`; the file carries the ground's z, so a robot shut down on 2F boots on 2F (README, "The last pose file"). |
| `syncai_lio_bridge` | **The only odometry source — not in this repo** (`chungweeeei/SyncAI-LIO-Bridge`, Rust / rclrs; edit it there). Converts `map → pointlio_odom → pointlio_body` into `odom → base_link` TF + `/<robot_id>/odom` + the AMCL-style `map → odom` correction, all projected to 2D (x, y, yaw) so the planar nav stack never sees a tilted frame. Angular velocity from the lidar IMU gyro, by choice (its README has why). It does its own TF (rclrs has no `tf2_ros`). `lio_bridge.launch.py` sets every parameter inline, no params file. |
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` + the Livox driver. The URDF carries **`lidar_top`, the lidar itself** (the frame the driver stamps `livox/lidar` with) and under it **`lidar_top_imu`**, the lidar's IMU at `-t_il` from `syncai_pointlio`'s params — what Point-LIO's body frame physically is. `syncai_lio_bridge` / `syncai_robot_state` still name `lidar_top` as their `lidar_frame` (~5 cm off) until their repos move to `lidar_top_imu`; `camera_optical_frame` hangs off `lidar_top`. The fleet runs **MID360 and MID360s**: `[sensor.lidar] type` picks the JSON schema, and the driver's network JSON is **generated** per `robot_id` into `/tmp/syncai_bringup/` from `[sensor.lidar] ip` + `type` + `host_ip` (params) — the vendor JSON in the driver's share dir is not read. No camera node (see "Camera"). |

**Camera.** Exactly one process may hold the V4L2 device (the second opener
does not fail at `open()`; it dies at `S_FMT` with "Device or resource busy"
long after the pane has scrolled), and that process is `syncai_camera_feeder`,
started by the nav session only. Nothing else in the stack opens the camera —
do not add a second opener (a `vizionsdk_ros2` node in `bringup.launch.py`
used to be one; what it learned about the hardware is in `syncai_bringup`'s
README). The camera is not configured in the instance INI.

### Hardware & system

| Package | Role |
|---|---|
| `syncai_driver_manager` | **UDP bridge to the gait controller — not in this repo** (`chungweeeei/SyncAI-Robot-Driver-Manager`, Rust / rclrs; edit it there). Sends `cmd_vel` with per-direction velocity scales (`scale_fwd` … `scale_turn_r`, ROS parameters in `params/driver_manager_params.yaml`; a runtime `set_speed_scale` override does not persist), serves `set_motion_key` / `set_policy_mode`, receives telemetry, and owns the safe-shutdown path (safety lock + lie down) — which has **zero call sites** today, so `safety_locked` / `RobotState.low_level_mode.safety_state` read `false` for want of a trigger, not because anything was checked. |
| `syncai_robot_state` | **Aggregates odom / battery / wifi / motor_states / `mode` / `safety_locked` / TF into `syncai_common/RobotState` — not in this repo** (`chungweeeei/SyncAI-Robot-State`, Rust / rclrs; edit it there). Runs at **1 Hz** (`publish_rate: 1.0` in its params). `RobotState.mode` polls `syncai_sys_manager`'s `get_mode` and reads `AUTO` until the first answer; `state` is `UNINITIALIZED` (no pose) / `WARNING` (battery < 20 %, cleared above 25 %) / `IDLE`. Reports only. `transform_tolerance` has no effect: it does not expire a stale transform. |
| `syncai_sys_manager` | Python. Five managers behind ROS services: wifi (`scan_wifi` / `connect_wifi`, `wifi_status` at 1 Hz from a cache refreshed every 5 s), mDNS (`avahi-publish <robot_id>.local`), conf (declares `robot_id`), monitor (host memory / disk to stdout at 1 Hz), and **node** (`NodeManager` — byobu session lifecycle, `switch_mode` / `get_mode` / `restart_mode`; see "Running the stack"). Also ships the host udev rules (`udev/99-syncai-devices.rules`). |
| `syncai_camera_feeder` | **The single opener of `/dev/syncai/camera0` — not in this repo** (`chungweeeei/SyncAI-Robot-Camera-Feeder`, Rust / rclrs + GStreamer; edit it there). One `v4l2src` (MJPEG 1280×720@60) teed, undecoded, into RTP/JPEG on `udp://127.0.0.1:5008` for the WebRTC worker and `image_raw/compressed` (30 Hz, SensorData QoS, frame `<robot_id>/camera_optical_frame`); it exits on any pipeline error and its launch respawns it. Needs the `dev` stage's `libgstreamer1.0-dev` / `libgstreamer-plugins-base1.0-dev`. |

**rclrs params files are keyed on a bare `/**`**, not the `/**/<node>` the rest
of this workspace uses: rclrs matches a key only when it is exactly `/**` or
the node's fully-qualified name, so a wildcard key silently matches nothing and
every parameter falls back to its code default.

### Out-of-tree packages and the Rust underlay

What `src/` holds: the `syncai_*` packages above plus `src/third-party/`. Six
of those packages are not tracked here and are materialised back into `src/`
by `vcs import < dependencies.repos` (every entry on branch `dev`, over HTTPS):

| Package | Repo |
|---|---|
| `syncai_common` | `chungweeeei/SyncAI-Robot-Interface` |
| `syncai_mapping` | `chungweeeei/SyncAI-Robot-3D-Mapping` |
| `syncai_driver_manager` | `chungweeeei/SyncAI-Robot-Driver-Manager` |
| `syncai_robot_state` | `chungweeeei/SyncAI-Robot-State` |
| `syncai_lio_bridge` | `chungweeeei/SyncAI-LIO-Bridge` |
| `syncai_camera_feeder` | `chungweeeei/SyncAI-Robot-Camera-Feeder` |

Each kept its package name, executables and launch file when it moved, so the
session specs did not change. Edit them in their own checkouts: a change made
in those directories is untracked, and the next `--force` import overwrites it.
The next package to move out gets an entry in `dependencies.repos` (and in
`scripts/build.sh`'s checkout table), not a new file.

**The four Rust packages build on an underlay baked into the image.** The
`rust-underlay` stage builds ros2-rust (`rclrs` from source,
`rosidl_generator_rs` and the standard interfaces rebuilt with Rust bindings)
into `/opt/ros2_rust_underlay`, with the three ros2-rust repos pinned to SHAs in
`ARG`s that must be bumped together. It exists because the Rust bindings are
generated only for interface packages built after the generator — the apt
copies ship none. `rclrs`'s and `rosidl_runtime_rs`'s `build.rs` are patched to
rerun on `AMENT_PREFIX_PATH` (upstream keeps stale link-search paths). It is
sourced between `/opt/ros/humble` and the workspace (`~/.bashrc`,
`scripts/build.sh`, robot01's `command:`), so its rebuilt `common_interfaces`
overlay the apt ones for the **whole** workspace, C++ included. A workspace
that once built those interface packages in `src/` keeps stale `build/` +
`install/` copies that shadow the underlay — delete them, or configure fails on
a missing `register_rs.cmake`. `tf2_msgs` needs no underlay: the apt package
already ships Rust bindings.

### Third-party (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`. Unmodified. |
| `direct_visual_lidar_calibration` | `koide3/direct_visual_lidar_calibration`, pinned to a main SHA. Targetless camera → lidar extrinsic calibration, run by hand on bags recorded in `CALIBRATION` mode. Has **git submodules**, so `third-party.repos` is imported `--recursive` (`scripts/build.sh` refuses a checkout without them). Needs Ceres 2.2 (jammy ships 2.0), Iridescence and `libgtsam_unstable` from the image's stages, plus `pcl_ros` / `cv_bridge` / `fmt` / glog / glfw from apt. Builds only against GTSAM 4.2. |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver |
| `small_gicp` | Pinned to upstream tag `v1.0.1`. Unmodified. The registration backend of `syncai_localizer` and `syncai_mapping`'s loop verification. Ships its own `package.xml` with `<build_type>cmake</build_type>`, so colcon builds it as a plain CMake package found by `find_package(small_gicp)`. |
| `vizionsdk-ros2` | TechNexion camera wrapper. **Nothing starts it**; it is still imported and built. Needs the closed-source VizionSDK `.deb`, which the `Dockerfile` downloads (`VIZIONSDK_VERSION`); there is no rosdep key for it. |

All six are checked out by vcstool from `third-party.repos`; bumping one is a
one-line `version:` edit there plus `vcs import --recursive < third-party.repos --force`.
The import target is the workspace root, not `src`: the keys already carry
`src/`. The FAST-LIO2 fork once imported at `src/third-party/FASTLIO2_ROS2` was
ported in-tree (`syncai_pointlio`, `syncai_mapping`, `syncai_localizer`,
`syncai_common`); a checkout left on disk still builds a duplicate `localizer`
package — delete it (`rm -rf src/third-party/FASTLIO2_ROS2 build/localizer
install/localizer`).

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
which is what the backend's map-switch route writes, in place, without
round-tripping the file through configparser. Nothing in this repo writes this
file; that it stays rewritable one line at a time is part of the contract with
the backend), `[initial_pose]` (only the localizer's fallback: its boot guess
is the last pose it saved, `config/state/<robot_id>/last_pose.json`, when that
file is for the same map — the INI is still never written here), and
`[sensor.lidar]` `ip` + `type` (`mid360` or
`mid360s`). The model is load-bearing: the JSON *section name* is what
Livox-SDK2 maps to a device type, so a wrong `type` yields no point cloud and no
error. A missing `type` warns and falls back to `mid360`.

The C++ packages' launch files use the relative path `config/system.ini`, which
works because **processes are expected to run with the workspace root as their
cwd** — the session specs start every pane there, and `colcon.meta` and
`ruff.toml` rely on the same assumption. `syncai_sys_manager` instead defaults
to the absolute `~/robot_ws/config/system.ini` (`SYNCAI_SYSTEM_INI` /
`system_config:=`) — and so does the backend, from its own container, which is
why that path is an interface too.

Three rules follow from namespacing:

- **Topics are written as relative names** in params YAML and in code
  (`map`, `scan`, `pointlio/body_cloud`), so they inherit the namespace
  automatically. Never hardcode `/<robot_id>/…` in a subscriber. There are
  **no** exceptions: an absolute, fleet-wide `/robot_state` was tried and
  reverted, because every per-robot consumer is scoped to exactly one robot.
- **TF frame names are NOT namespaced by ROS.** Launch files therefore override
  frame parameters explicitly (`robot_base_frame: <robot_id>/base_link`,
  `frame_id: <robot_id>/lidar_top` on the Livox driver). The values in the YAML are only fallbacks
  for running a node without its launch file. `global_frame` stays plain `map`.
- **Params files use `/**/<node_name>:` wildcard keys** (except the rclrs
  packages', see above), so the same file works at any namespace. When a
  process hosts more than one node (e.g. `planner_server` + its internal
  `global_costmap`), the launch `Node` must have **no `name=`** — a
  launch-level name remaps *both* nodes to the same name and the internal
  costmap silently loses all its parameters. Extra dicts passed to such a Node
  land under the `/**` wildcard and reach both nodes.

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
syncai-build`, not `include`d) and its one service runs `scripts/build.sh` as
its *entrypoint*, so `run build --packages-select x` appends to
`colcon build --symlink-install` instead of replacing it. It is a **release
build** (`BUILD_TYPE`, default `Release`: `-DCMAKE_BUILD_TYPE` plus
`--cargo-args --release`; a plain `colcon build` leaves most packages
unoptimised), and an own `--cmake-args` / `--cargo-args` replaces those, so
repeat the build type in it. Same image as robot01 on purpose: the artefacts
land in the shared bind-mounted `build/` + `install/`. It therefore only
`rosdep check`s by default (`BUILD_ROSDEP=check`) — an apt install into a
throwaway container never reaches robot01, so a missing dep is a `Dockerfile`
change. The script restores the `livox_ros_driver2` `package.xml` when a
`vcs import` has deleted it, and refuses to start on an empty vcs checkout,
naming the `.repos` file to import per missing directory (a `dir:repos` table
of the six third-party and six first-party checkouts). The first-party checks
matter most: an absent `src/syncai_common` fails every package, but an absent
`syncai_mapping` / `_driver_manager` / `_robot_state` / `_lio_bridge` /
`_camera_feeder` fails *nothing* — colcon builds a workspace with no way to
save a map, no gait bridge, no `RobotState`, no odometry and no camera. After
colcon it deletes the rclrs nodes' installed `share/<pkg>/rust/` source copy
together with its `rust_packages` index marker — a marker left without its
directory makes cargo fail every later Rust build. Its `build:` block
duplicates `x-robot-common` (compose `extends` would drag the devices / nvidia
runtime / X11 mounts along) — keep the two in sync.

Recreating a robot container wipes hand-installed build dependencies. Re-run
`rosdep install --from-paths src --ignore-src -r -y`; Sophus
(`syncai_pointlio`), GTSAM (`syncai_mapping`), Ceres and Iridescence
(`direct_visual_lidar_calibration`) are source builds that come from the
image's stages and are the only dependencies `rosdep` does not cover.

The `Dockerfile` is multi-stage, and every slow build is a stage that depends
only on `base`, so BuildKit runs them in parallel and no `dev` edit invalidates
them: `base` (ros-base + cyclonedds + uid-1000 user) → `deps-builder` →
`livox` / `gtsam` / `sophus` / `ceres` / `iridescence` (one library each,
installed with `DESTDIR=/out`; GTSAM is the slow one — keep these stages free of
anything that changes often); `base` → `rust-underlay`; `base` → `dev`
(colcon, byobu, Node.js, the VizionSDK `.deb`, `ros-humble-octomap`,
`libclang-dev` + `colcon-cargo` / `colcon-ros-cargo`, and one `COPY --from` per
stage above). Compose builds `target: dev`. There is **no rviz2** or
`pointcloud_to_laserscan` in the image (the robot has no display);
`ros-humble-map-msgs` and `ros-humble-laser-geometry` are listed explicitly
because those used to pull them in. Most of `dev`'s largest layer is VTK + Qt5
regardless: `libpcl-dev` hard-Depends on them. `chmod -R a+w /opt/rust` runs
once, at the end of `rust-underlay` — a second walk duplicates the 516 MB
toolchain in another layer. `dev` is a pure ROS image: do not add a pip package
for a process that runs in another container. The production stages were
removed during the dev phase, so `scripts/release/` does not work today —
recover them from git history when it is time to ship to the IPC.

`colcon.meta` at the workspace root carries per-package cmake args. colcon only
finds it because its `--metas` default is the relative `./colcon.meta` — same
cwd-is-the-workspace-root assumption as the launch files. Its only entry is
`livox_ros_driver2`, which needs `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`:
without it its `CMakeLists.txt` takes the pre-Humble branch and configure dies
with `NOTFOUND`. The flags live here because upstream only configures itself
through its own `build.sh`; the checkout stays pinned and unmodified.

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
permitted.` with a traceback, but nothing is broken: BT.CPP's `package.xml` has
two `<build_type>`s gated by `$ROS_VERSION` conditions, `colcon-ros-cargo`
0.2.0 reads them before evaluating the conditions, and colcon falls back to the
ROS identification and builds it normally. Upstream fixed it (PR #32) but has
not released it. Bump the pin (`COLCON_CARGO_PIP` in the `Dockerfile`) when a
0.2.1 / 0.3.0 ships, not to a git SHA — that `ARG` also feeds `rust-underlay`.
Do not edit BT.CPP's `package.xml` either: the next `vcs import --force`
reverts it.

## Running the stack

The stack comes up as a **byobu session, one per operating mode**, built by
`NodeManager` in
`src/syncai_sys_manager/syncai_sys_manager/managers/node_manager.py`. There is
no launcher script: a ROS node is the entrypoint, which is what lets the
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

`CALIBRATION` is bringup with the livox driver on `PointCloud2`
(`livox_xfer_format:=0`) plus the camera feeder, for recording lidar-camera
calibration bags — not even `driver_manager` or `robot_state`, so lie the
robot down before switching to it. It is a mode because it holds the same lidar
UDP ports and camera the other two do, and `switch_mode`'s kill-all-then-build
keeps them exclusive. The backend's mode route accepts `MANUAL` / `AUTO` only,
so it is a `ros2 service call`. The procedure is
`docs/lidar-camera-calibration.md`, with its tools in `scripts/calibration/`
and the `lidar-camera-calibration` skill under `.claude/skills/`.

Three services, namespaced under `robot_id`:

```bash
ros2 service call /<robot_id>/get_mode     syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode  syncai_common/srv/SwitchMode "{mode: 2}"
ros2 service call /<robot_id>/restart_mode syncai_common/srv/RestartMode "{}"
```

`switch_mode` kills *every* known session before building the target one, and
refuses to rebuild the mode that is already live — in `MANUAL` that would drop
an unsaved map, because `pgo_node` accumulates its keyframes in RAM and
`save_maps` is the only thing that serialises them. That is why **starting a
new map is not a mode switch**: `pgo/reset_mapping` does it in place. Rebuilding
the live session on purpose (a wedged stack) is `restart_mode`: refused in
`MAINTENANCE`, with two sessions up, and always in `MANUAL` (it cannot tell
whether the map was saved), so in practice it restarts `AUTO`. The backend does
not call it today. `sys_manager` itself runs **outside** the specs — it is the
robot container's main process (`command:` in `docker-compose.robots.yml`), so
`docker compose up -d robot01` brings the stack up on its own. Add it as a
window in a spec and `kill_session` would kill the pane it is running in.

The window list is **data**: `config/sessions/*.yaml` holds windows / panes /
commands / `sleep` offsets / `multilog` names, and `NodeManager` holds the byobu
plumbing. The schema is documented in the `NodeManager` docstring (`session`,
`select`, `windows[{name, cwd?, panes[{cmd, sleep?, log?, enter?}]}]`). `sleep`
is where the startup ordering lives. The nav session's windows, in order:
`bringup` → `map_server` (map_server + keepout) → `localization` (pointlio +
localizer) → `lio_bridge` → `plan_ctrl` (planner + controller) → `task_runner`
→ `driver_manager` → `robot_state` → `camera_feeder`. Every pane of a normal
boot stays up, so "Filter mask was not received" in the planner log means the
`keepout` pane is actually down (except on a 3D-only map, where it runs idle).
Sessions are built **detached** — the caller is a ROS node with no TTY. Neither
spec has a backend, frontend or rviz2 window: the first two run in their own
containers (so a mode switch interrupts neither), and the robot has no display
(`config/rviz2/<robot_id>.rviz` is for a workstation).

The specs are counterparts, not variants. `start_nav.yaml` *localizes* against
an existing map — `syncai_localizer` loads one during construction and dies
without it — so it cannot build the map it needs; `start_mapping.yaml` is the
other half: `bringup` → `lio` (`syncai_pointlio` + `syncai_mapping`) →
`driver_manager` → `robot_state`, and deliberately **no** map_server /
localizer / lio_bridge / planner / controller / task_runner / camera.
`robot_state` is there for its `mode` field alone — the console's mode chip
reads it to know the switch to `MANUAL` landed. Both specs set `select:` to
`bringup` because its log answers "is the lidar alive?".

Logs: multilog writes `current` plus gzipped rotations (16 MiB × 10). There is
no log-reading helper:

```bash
tail -f log/stack/<robot_id>/planner/current
zcat log/stack/<robot_id>/planner/@*.s | less
```

`ROS_LOG_DIR` points at a tmpfs (ephemeral); the byobu `pipe-pane` capture is
the persistent record. There is no keyboard-teleop window — a pane where a
stray keypress is a motion command; the console's teleop covers it, and with no
backend `teleop_twist_keyboard` remapped onto `/<robot_id>/cmd_vel` is the
fallback.

## Out of tree: the backend and the console

| Repo | What it does |
|---|---|
| `chungweeeei/SyncAI-Robot-Backend` | FastAPI + rclpy in one process on port **3000**: Temporal task orchestration, task templates and schedules, the map catalogue and both pcd → gridmap recipes, TTS, bag recording, and the teleop / telemetry / point-cloud WebSockets. |
| `chungweeeei/SyncAI-Robot-Frontend` | Next.js console (dev server on 3001): dashboard with the 3D point cloud, `/mapping`, `/maps` + the gridmap editor, `/recordings`, `/tasks`, `/settings`. Talks to the backend and to nothing else on the robot. |

Neither is imported, built or launched from here. The backend runs in its own
container on the robot, with host networking on DDS domain 1, and discovers the
nodes in `src/` over DDS. The infra it needs — postgres and temporal — is in
this repo's `docker-compose.yml`, because the robot is where it is deployed.

### What that means when editing this repo

**This stack's ROS surface is the backend's API, one layer down.** Renaming a
service, changing a message field, moving a file inside `map/<name>/` or
changing a node's namespace is a cross-repository change: say so in the commit
message and keep the old shape working until the backend has moved. The surface
it actually uses:

| Here | Used by the backend for |
|---|---|
| `syncai_common` msg/srv/action (`SyncAI-Robot-Interface`) | every call below — it builds against that repo, not this checkout |
| `robot_state` topic (`syncai_robot_state`) | the telemetry WebSocket and `GET /api/v1/robot/state` |
| `switch_mode` / `get_mode` (`syncai_sys_manager`) | `POST /api/v1/robot/mode`; its gateway waits 45 s / 70 s for them |
| `scan_wifi` / `connect_wifi` / `wifi_status` (`syncai_sys_manager`) | the network router |
| `cmd_vel`, `set_motion_key` (`syncai_driver_manager`) | teleop WebSocket, STANDUP / LIEDOWN steps |
| `NavigateToPose` on `task_runner`, and `NavigateToGoal` on `navigate_to_goal` (the same, with `error_code` / `error_msg` and `status_msg`) | the MOVE step; the backend still sends the nav2 action, which stays until it has moved. **DOCK / UNDOCK are the same action** with `behavior_tree: dock.xml` / `undock.xml` and, for a DOCK, the **dock pose** as `pose` (UNDOCK's is ignored: it backs out 1 m from where the robot stands); a docking-branch failure is `DOCK_FAILED` (300), the staging leg's 100 / 200 |
| `relocalize`, `relocalize_check`, `initialpose` (`syncai_localizer`). **Bare names in the `/<robot_id>` namespace** — the prefixed `/<robot_id>/localizer/…` spelling cost the backend a `stack_not_ready` against a healthy stack | initial pose, and map switch |
| `map_server/load_map` | map switch, and reload after a re-conversion. Fails on a 3D-only map, which is expected there |
| `planner_server/Octo3D/load_map` (`nav2_msgs/srv/LoadMap`) | the map switch: `map_url` = `map/<name>/octomap.bt` for a 3D map, **`""` (unload) for a 2D one** — without it `Octo3D` keeps the previous tree and the task_runner keeps running the 3D trees. Blocks until the surface map is built (seconds; size the timeout) and its success is a result; a missing file is refused with the current map kept, any other failure leaves no map. Reads the new map's `keepout.json` itself |
| `planner_server/Octo3D/load_keepout` (`nav2_msgs/srv/LoadMap`) | nothing today; the reload of a 3D map's zones after the backend writes `map/<name>/keepout.json` (`map_url` = the file, `""` = none). Blocks for a plan in flight (≤ 2 s); `RESULT_MAP_DOES_NOT_EXIST` / `RESULT_INVALID_MAP_DATA` keep the current zones, `RESULT_UNDEFINED_FAILURE` = no 3D map loaded. The running path is re-judged by `is_path_valid` within 1 s, no `clear_plan` |
| `auto_localize` (`syncai_localizer`, bare name) | nothing today (the backend reaches the same search through `reset_odometry`'s `skip_seed`). Blocks up to its two deadlines (15 s) |
| `reset_odometry` (`syncai_localizer`, bare name) | `POST /api/v1/localization/auto`, with `skip_seed: true`: reset Point-LIO, then the global search — so finding a kidnapped robot always zeroes the odometry, and `odom → base_link` jumps to the origin once. Blocks up to 17 s (32 s with `global_fallback`, 27 s with `skip_seed`; the backend waits 40 s); the caller must have stopped the robot, nothing enforces it |
| `pgo/start_mapping`, `pgo/save_maps`, `pgo/reset_mapping`, `pgo/mapping_status`, `pgo/map_cloud_file` (`syncai_mapping`) | begin a run, save a map (which ends the run), start over mid-run, the run state (latched), the live merge (a PCD in the shared `/dev/shm/syncai_pgo/<robot_id>`, named by the notice — needs `ipc: host` on both containers; `pgo/map_cloud` is rviz-only) |
| `pointlio/body_cloud` (`syncai_pointlio`) | the live cloud WebSocket |
| `config/instances/robotNN.ini` (`[map] name`, `[initial_pose]`) | the only file in this repo the backend **writes** |
| `map/<name>/` on disk | the map catalogue: `map.pcd`, `poses.txt`, `patches/`, `gridmap.*` or `octomap.bt`, `map_clean.recipe.json`, and optionally the forbidden zones: for a 2D map `keepout.yaml` + `keepout.pgm` (the mask: same format as `gridmap.*`, black = keepout, drawn as the forbidden area only — the filter inflates it itself, so a drawn margin is applied twice), for a 3D map **`keepout.json`** (`{"version": 1, "zones": [{"id", "points": [{"x", "y"}, …], "name"?, "z_min"?, "z_max"?}]}`, metres in the map frame, the optional z band naming the floor — `floor_z ± 0.5`, absent = every floor — `id` required and unique, no margin either; the full rules in `syncai_global_planner`'s README). **One file for 2D and 3D**: it is the file the backend already writes for 2D zones (from which it rasterises the mask), `points` shape, plus the three optional keys — so a 2D-editor file works on a 3D map, and the backend must keep `name` / `z_min` / `z_max` when its 2D editor rewrites it. The nav session **writes a blank mask** (all unknown — grey, not white — same geometry as `gridmap.*`, tmp + rename) when a 2D map has none at boot, so a booted map always has the canvas the editor draws on; it writes no `keepout.json` (absent = no zones). The backend writes `keepout.json` for 2D maps today but not for 3D-only ones; reload paths: `filter_mask_server/load_map` / `planner_server/Octo3D/load_keepout` are the reload paths. A 2D map switch needs the mask call too, since the nav session derives the mask path once at boot; a 3D switch does not, `load_map` reads the new map's `keepout.json` itself |

**Facts about this stack the backend depends on.** These belong here because
they are properties of the nodes in `src/`, not of the backend:

- **`NavigateToGoal` ABORTED with `error_code` NONE is "superseded or not
  started", not a navigation failure.** A preempting goal aborts the old one
  with a default result (`SimpleActionServer::accept_pending_goal`), and a
  goal rejected before the tree runs (the other navigator busy, the
  `behavior_tree` file missing, a docking server missing at the first dock
  goal) never reaches `goalCompleted()`. Only a tree that ran and failed
  carries `PLAN_FAILED` / `FOLLOW_PATH_FAILED` / `DOCK_FAILED` / `UNKNOWN`;
  CANCELED carries NONE too. The codes are never renumbered — switch on the
  hundreds.
- **A dock ends standing on the dock in MPC, and so does a cancelled one.**
  `dock.xml` restores RL (`set_motion_key` key `1`) only when the switch or the
  approach fails, and does **not** lie the robot down: a task that wants the
  robot resting on the contacts needs a **LIEDOWN step after the DOCK**. The
  stack does not switch back by itself: the next step is `undock.xml`, or the
  backend sends key `1` before a MOVE. `undock.xml` starts from standing in MPC
  (a robot laid down needs STANDUP first — the tree has no stand-up step),
  backs out blind 1 m from where it stands, and **ends in RL** facing the dock
  — the way to leave, because a MOVE straight off the dock starts with the
  footprint on its lethal cells. A re-sent undock backs out a further metre.
- **A MOVE after a DOCK does not undock by itself; "the robot is docked" is
  the backend's state** (decided). A `task_runner`-side flag cannot see a
  LIEDOWN the backend sends straight to `set_motion_key`, is lost on a restart,
  and would hide a blind 1 m back-out inside a MOVE. The trees stay stateless,
  one action each.
- **The last metre is blind.** The docking controller reads no costmap;
  avoidance is the staging leg. Something that steps between the robot and the
  dock after staging is pushed against until the progress checker (30 s) or
  `failure_tolerance` ends the goal. The MPC switch is confirmed from the
  controller's own report (`mode` data[1] = 5, since `set_motion_key` is
  one-way UDP) and followed by 3 s of settling; no report within 5 s fails the
  dock (`DOCK_FAILED`, RL restored).
- **`relocalize` returning success is a receipt, not a result.** Registration
  runs async at 5 Hz with no deadline; `relocalize_check` is the only surface
  that reports the outcome. `RobotState.localization_valid` is TF-presence only
  and reads true against a map the robot was never localized in — do not "fix"
  it into a quality signal without a source for one.
- **`relocCB` takes the request's raw 6-DOF**, bypassing the `applyPlanarGuess`
  tilt correction that the `initialpose` path applies. A relocalize call must
  therefore be followed by an `initialpose` publish, or the tilted lidar mount
  leaves the localizer retrying a flat guess. This is why the backend's map
  switch does both, in that order.
- **`map_server` and the localizer open their files during construction**; the
  localizer dies without its `map.pcd`, and `map_server` throws on an
  unreadable gridmap but comes up idle when `[map] map` names one that does not
  exist (a 3D-only map). That is why the nav session cannot build the map it
  needs, and why the active map's directory cannot be renamed or deleted under
  them.
- **No file inside a map directory names the map or holds an absolute path** —
  `gridmap.yaml` says `image: gridmap.pgm`, `poses.txt` lists bare patch
  basenames, the recipe has neither. That property is what makes a rename one
  `os.rename`; a future sidecar that embeds the path breaks it silently.
- **A mapping run is bracketed by `pgo/start_mapping` and `pgo/save_maps`;
  `pgo/reset_mapping` restarts one in place.** `pgo_node` comes up IDLE and
  banks nothing until Start; a successful save ends the run (keyframes freed,
  empty map published, `/dev/shm` cleared, IDLE again); the state is latched on
  `pgo/mapping_status` (IDLE / MAPPING / RESETTING). Start and reset run the
  same sequence from opposite preconditions (Start refused while MAPPING, reset
  and save while IDLE): pause intake, reset Point-LIO over `pointlio/reset`,
  rebuild the pose graph and drop everything at or before the boundary the
  front end reported — pausing first is the whole design, so the odometry
  discontinuity has nowhere to land. The only fallible step runs before
  anything is destroyed: there is no half-reset. **The robot must be
  stationary**, because Point-LIO re-runs a static, gravity-aligning IMU init,
  and one done in motion tilts the new map for its whole life with no error
  anywhere. Nothing enforces that, in either repo.
- **An empty map-cloud merge is a message, not a non-event** — pgo sends one
  from `reset_mapping`, `start_mapping` **and a successful `save_maps`** on both
  outputs (an empty PointCloud2, a `points: 0` notice), and a consumer that
  skips it keeps showing the old map. The save's is also what empties
  `/dev/shm/syncai_pgo/<robot_id>`; the backend never deletes those files.
- **`map.pcd` changes after `save_maps` returns.** It is the raw save until
  `map_clean.recipe.json` says `ok`, then `clean_map`'s copy (atomic rename; a
  reader that already has it open keeps the old file). A gridmap converted
  while the sidecar says `converting` was made from the raw map and should be
  redone — the backend does not do that yet. A hard kill leaves it at
  `converting` with the raw map in place: age it out by `started_at`. The rerun
  is `ros2 run syncai_mapping clean_map map/<name>`, idempotent because it
  starts from `patches/`.
- **The live merge is a file, not a topic payload.** A large site's merge is
  16-45 MB, and CycloneDDS over UDP on `lo` cannot deliver that through the
  kernel's default 208 KB receive buffer — a BEST_EFFORT reader loses every
  sample. pgo writes it to `/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd`
  (tmp + rename, newest two kept) and publishes a ~200 B JSON notice on
  `pgo/map_cloud_file` (RELIABLE, TRANSIENT_LOCAL). Both compose services run
  `ipc: host`; without it the notices arrive and every read is ENOENT.
- **There are two pcd → gridmap recipes and no automatic pick between them**
  (z-band slicing with a pose-connectivity filter, the default; and
  traversability segmentation, which produces no unknown cells). Both live in
  the backend. A wrong z-band map is recoverable, a wrong traversability map is
  permanently walled, and the one heuristic tried misrouted all three
  conference-hall saves. Do not add a heuristic here either.
- **`switch_mode` does not kill the backend** (it used to be a pane of the
  session). The backend's on-disk conversion status and its bag recording's
  process-group handling are now belt-and-braces rather than load-bearing — do
  not remove them from that repo on the strength of this alone.

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
  **None of the four has a consumer in this repo** — they are the backend's —
  and they stay here because `docker compose up -d` on the robot has to bring
  them up. Moving them to the backend repo is a live decision, not a cleanup:
  it means moving `./data/postgres` with them.
- **`docker-compose.robots.yml`** — robot containers, pulled into
  `docker-compose.yml` via `include:`. One service, `robot01` (`network_mode:
  host`), with **no compose profile**: a plain `docker compose up -d` starts
  infra and robot together; to start infra alone, name the four services.
  Robot containers bind-mount the host D-Bus socket (so `nmcli` reaches the
  host NetworkManager — needs `apparmor=unconfined` + sudo) and the directory
  `/run/avahi-daemon` (so `libnss-mdns` resolves `*.local`; the directory,
  because a file bind mount goes stale when `avahi-daemon.socket` restarts),
  run with `runtime: nvidia` and `init: true` (tini as PID 1, to reap the
  `clean_map` a mode switch orphans), and pass through the cameras as
  `/dev/syncai/camera0` / `camera1` (udev symlinks keyed on serial, from
  `src/syncai_sys_manager/udev/99-syncai-devices.rules`) plus `/dev/snd`
  (`aplay` in here is how the speaker is diagnosed), with the host `video` /
  `audio` gids (`VIDEO_GID` / `AUDIO_GID`, default 44 / 29) and
  `device_cgroup_rules` for the video4linux (81) and ALSA (116) majors so a USB
  replug onto a new minor needs no recreate. X11 is passed through (the file
  header has the xauth-cookie recipe). `ROS_LOG_DIR` is a 128 MiB tmpfs. Under
  `network_mode: host` compose service names do not resolve: anything that
  talks to the infra uses `127.0.0.1` and the host-published ports.
- **`ROS_DOMAIN_ID` is a literal `"1"` in compose, not `${ROS_DOMAIN_ID:-1}`.**
  Compose interpolation gives the invoking shell precedence over `.env`, and
  the Jetson host's `~/.bashrc` exports `ROS_DOMAIN_ID=69`, so an overridable
  default silently resolved to 69. The flip side: anything started on the
  **host** (rviz2, `ros2` CLI) inherits 69 and will not see the containers
  until that export is changed.
- **CycloneDDS** is the RMW, configured by hand: `config/cyclonedds.xml` —
  loopback only (`lo`), `AllowMulticast=false`, one explicit unicast peer at
  `127.0.0.1`. It pins `<Domain Id="1">`, so the domain is **not** inherited
  from `ROS_DOMAIN_ID` — changing the compose env alone silently leaves DDS on
  domain 1 and nothing discovers anything. Change both. Gotchas that have
  bitten before: a single interface with `multicast=false` disables multicast
  **globally**; peer pings only use the top-priority interface; unicast
  `<Peers>` require the remote side to have `ParticipantIndex=auto`.
- tmpfs `mode:` in compose must be written as an octal literal (`0o1777`) — a
  bare `1777` is parsed as decimal.

## Conventions

- C++ formatting is pinned by `.clang-format` (ROS 2 style, 100 columns).
  Python is linted by the root `ruff.toml` (py310, rule set pinned on purpose
  so a ruff upgrade's growing defaults do not flood the editor); no package
  under `src/` carries its own ruff/pyproject config.
- Comments and docs are written in **English**, log strings included. Comments
  in this codebase explain **why**, often at length, and frequently record a
  past bug or a rejected alternative. Match that density when editing — a bare
  parameter change with no rationale is out of place here. If a translation is
  ever added, a `*.zh-TW.md` sits beside an English file that stays the
  canonical copy; update both or delete the translation.
- `build/`, `install/`, `log/`, `data/`, `.env`, `record/` (hand-recorded
  rosbags), `/map/`, `/models/`, `/config/calibration/` (each robot's own
  camera intrinsics, made on the robot) and `/config/state/` (per-robot runtime
  state: the localizer's `last_pose.json`) are gitignored, as are the six
  `dependencies.repos` checkouts and `src/syncai_backend/` (so a clone of the
  backend kept there for development can never be committed back in). `.env`
  holds secrets — never commit it.
- `scripts/`: `attach.sh` (host-side: attaches to whichever byobu session is
  live — the session name follows the mode), `sync.sh` (host-side, git only:
  `git pull` of the workspace plus a fast-forward of every `dependencies.repos`
  checkout on its pinned branch — a plain pull never touches them; a checkout
  on another branch is fetched only), `build.sh` (in-image, see Build),
  `urdf2glb.py` (see "Out of tree"), `calibration/` (camera intrinsics and
  lidar-camera extrinsic; `collect_board.py` / `snapshot.py` need rclpy, the
  rest only numpy + OpenCV + PyYAML and also run on the host), and `release/`
  (an offline release bundle for the customer IPC, currently
  **non-functional**: it needs the removed production Dockerfile stages and a
  `docker-compose.prod.yml` that is not in the tree, and its `.env.example`
  says `ROS_DOMAIN_ID=0` while the stack pins domain 1). `skills-lock.json` is
  Claude Code tooling metadata, not stack config.
- `docs/` holds operating procedures: `lidar-camera-calibration.md` and
  `localizer-offline-replay.md`. Plans that pass through it are deleted once
  done; git history has the old ones (and the former `doc/` design proposals).

## Tests

`syncai_sys_manager` has `test_wifi_manager.py`; the C++ packages have the
ament linter tests that come with the package templates (never enforced, and
they would fail) plus five ROS-free gtests: `syncai_global_planner`'s
`test_surface_map` (surface map, search, stair links, dynamic overlay),
`syncai_util`'s `test_stair_mode`, `syncai_costmap_2d`'s `test_terrain_grid`
and `syncai_localizer`'s `test_localizability` and `test_last_pose_store`. CI does not run them; use
`colcon test`. `syncai_mapping`'s gtests live in its own repo, and the backend's
pytest suite in that repo.

**CI** (`.github/workflows/ci.yml`) runs on every push to `main` / `dev` and
every PR. `lint` (bare runner): `ruff check`, and clang-format 14 over the C++
lines the push or PR *changed* — not whole files, because ~90 tracked C++ files
predate `.clang-format`; a wholesale reformat is its own commit, after which the
check can widen. `build` (an **arm64** runner, because the robot is a Jetson):
the `dev` image, pulled from `ghcr.io/<repo>:dev-<Dockerfile sha256>` and built
only when no run has built that Dockerfile yet; both `.repos` files imported;
`scripts/build.sh` through `docker-compose.build.yml`, compiling through
**ccache** (turned on by CMake's `CMAKE_<LANG>_COMPILER_LAUNCHER` environment
variables in the workflow only, so robot01's builds are unchanged; the cache
is in the Actions cache, saved by `main` / `dev` pushes only, one entry per
branch); `test_wifi_manager.py`;
then a **smoke test** (`.github/scripts/smoke_test.sh`): every pane of the three
session specs, started with their `sleep` offsets against a generated 5 × 5 m
fixture map (`robot_id: ci_smoke`), failed if any launch has exited or logged a
crash after 45 s — the check for a Release-only undefined symbol at plugin load
that a green build misses. The nav spec runs a second time on a **3D-only
fixture** (`map/ci_smoke_3d/`, an `octomap.bt` from
`octomap_planner_3d_cli --synthetic two_floors --write-bt`, no gridmap),
switched to by rewriting the INI's `name:` line in place, and must also show
`map_server` and `keepout` starting idle, `Octo3D` building its surface map
and reading the fixture's `keepout.json` (one zone), and the task_runner
switching to the `*_3d.xml` trees. `camera_feeder` is skipped
(no camera). The script refuses to run outside CI or against another INI,
because it starts `driver_manager` too. A change to the gtsam / sophus / livox /
rust-underlay stages rebuilds GTSAM and the underlay (most of an hour).

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
