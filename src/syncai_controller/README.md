# syncai_controller

The path-following half of the nav stack: a `FollowPath` action server that owns
a rolling local costmap and drives a pluggable controller at a fixed rate. Ported
from `nav2_controller`, with `nav2_regulated_pure_pursuit_controller` merged in
as a plugin of this package.

Two path followers are loaded (2026-10-28):

- **`FollowPath` is MPPI**, from `syncai_mppi_controller` (its README has the
  port). It does local obstacle avoidance: it samples (v, ω) sequences on the
  local costmap and bends round a blocker beside the path. It matches a path
  with heights in 3D and climbs stairs (since 2026-10-20).
- **`FollowPathRPP3D` is RPP 3D** (`RegulatedPurePursuitController3D`), the
  Regulated Pure Pursuit this README mostly describes: RPP tracks the path and
  stops when the path is blocked, plus 3D matching of a path with heights and
  the stair mode (both below). It began (2026-10-19) as a **copy** of the 2D
  `RegulatedPurePursuitController` (`FollowPathRPP`, the `FollowPath` before
  MPPI, `dev`'s plugin unchanged), not a subclass — the stair logic runs
  through `computeVelocityCommands()`, which the 2D class had no hooks for.
  The 2D plugin went with 2D navigation on 2026-10-28, so RPP 3D is the only
  RPP and there is no second file to keep in step. Its values are the 2D
  RPP's calibration (one with `syncai_driver_manager`'s scales — change them
  together), except `desired_linear_vel`: **0.8 m/s** against the 2D RPP's
  0.60 since 2026-10-19, unmeasured on the robot (× `scale_fwd` 1.40 =
  1.12 m/s to the gait). Since 2026-10-20 its stair state machine is
  `syncai_util::StairMode`, shared with MPPI (below).

The FollowPath goal's `controller_id` picks between them, so
`controller_id` in `move.xml` is the whole A/B. `move.xml` names `FollowPath`
(MPPI) with `planner_id="Octo3D"`: MPPI matches a path with heights in 3D and
climbs stairs with the same `StairMode`, with its optimizer skipped on the
flight and running everywhere else, and a path without heights (a level
floor) takes its planar code path, `dev`'s, unchanged. Until 2026-10-28 that
tree was `move_3d.xml`, run on a 3D map in place of the 2D `move.xml`
(`GridBased`), where `FollowPathRPP` was the A/B.

From 2026-10 to 2026-11-12 the same node also served `DockToPose`, the last
metre onto and off a charging dock; it went with docking (see "Docking").

```
syncai_task_runner ──FollowPath (nav2_msgs)──►  controller_server
                                                   │
                          ┌────────────────────────┼────────────────────────┐
                          ▼                        ▼                        ▼
                  progress checker           goal checker            Controller plugins
              "still moving?"           goal_checker        FollowPath      = MPPI
                                                            FollowPathRPP3D = RPP 3D
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
  regulated_pure_pursuit_controller_3d/  RPP 3D: a copy of it plus 3D matching and
                                         the stair mode (feature/octomap-planner-3d)
  simple_progress_checker.cpp          ProgressChecker
  simple_goal_checker.cpp              GoalChecker
plugins.xml                      pluginlib manifest, exported to syncai_nav_core
params/controller_server_params.yaml       the only params file (the former 2D/sim and
                                           3D/real-robot variants were merged back into it)
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
starves the node's main executor, which handles odom and parameter callbacks.

```
findControllerId / findGoalCheckerId    ← from the goal, or the sole plugin if unset
setPlannerPath(goal.path)               ← controller->setPlan, cache end_pose_, reset goal checker
progress_checker_->reset()

loop at controller_frequency:
    server inactive?            → return
    cancel requested?           → terminate_all + zero velocity, return
    waitForCurrentCostmap()     ← zero + controller reset, then 100 Hz until
                                  current; cancel honoured; throws (goal fails)
                                  past failure_tolerance (>= 1 s) without a
                                  valid command. A clear makes the costmap
                                  non-current for one update, a stale cloud
                                  (terrain_layer cloud_timeout) until it returns
    updateGlobalPath()          ← accept a preempting goal in place, no restart;
                                  an unknown controller / goal checker in it
                                  terminates the goal → zero velocity, return
                                  (2026-11; it drove the old path on before)
    computeAndPublishVelocity()
    isGoalReached()?            → break

publishZeroVelocity()   (always: driver_manager has no cmd_vel watchdog)
succeeded_current()
```

`computeAndPublishVelocity()` does the real work: get the robot pose from the
costmap, run the progress checker (throws `PlannerException` on "Failed to make
progress"), threshold the odom twist, call
`controller->computeVelocityCommands(pose, twist, goal_checker)`, publish
feedback (`speed` and `distance_to_goal`, the latter measured along the remaining
path from the closest pose), and publish the twist. Since 2026-10 everything up
to the controller call is `computeVelocityCycle()` (split out for the dock loop,
gone 2026-11-12); only the feedback and the publish follow it.

**A goal holds `dynamic_params_lock_` throughout** (a `std::timed_mutex` since
2026-10), taken with a 100 ms bound: long enough to wait out a parameter
callback, far shorter than any goal. The bound was for the dock loop
(2026-10 → 2026-11-12), the one other holder that held it for seconds; the only
holder besides a goal now is a parameter callback (microseconds), so a
`follow_path refused: dynamic_params_lock_ held for over 100 ms` means something
holds it that should not. The refused goal publishes no zero twist — `cmd_vel`
belongs to whoever holds the lock.

**`failure_tolerance`** softens controller exceptions: with a positive value a
failed cycle publishes zero velocity and keeps trying until that many seconds
have passed since the last *valid* command, then gives up. `-1.0` retries
forever, `0.0` fails on the first exception.

It covers two things: an exception out of `computeVelocityCommands()`, and —
since 2026-10, not upstream — a failed **robot-pose lookup**. Upstream throws
"Failed to obtain robot pose" unconditionally, so one stale `odom → base_link`
failed `FollowPath` at once while the same TF miss inside RPP's
`transformGlobalPlan()` got the whole grace window, and it used up the BT's
retry. A pose failure now publishes zero, skips the progress checker and the
feedback (both need the pose) and calls the controller's `reset()`: RPP's
catch-all cannot zero its accel baseline on a cycle in which it is never
called, and without the reset the first cycle after the TF returned would
lurch from 0 to near the pre-outage speed. It does **not** cover the progress
checker's "Failed to make progress", which is meant to end the goal.

**`publishVelocity` skips publishing when nothing is subscribed** to `cmd_vel` —
worth knowing when debugging with `ros2 topic echo`, since attaching the echo
changes whether messages are sent at all.

### Reading the log after a failed goal

Every way out of a goal, and every way a goal can stall without ending, has a
line that says why, with the numbers that decided it. In order of appearance:

| Line | Level | What it tells you |
|---|---|---|
| `FollowPath goal: …` / `FollowPath replan: …` | INFO | What the controller was handed: controller, goal checker, pose count, length, frame, `with heights` (3D matching on), end pose. Read every later line against the last of these |
| `Waiting N s for the local costmap to be current …` | WARN, 2 s | The loop is held before the controller runs: a costmap layer has no fresh data (on this stack, `pointlio/body_cloud`). Nothing is commanded while it lasts — see "Gotchas" |
| `Robot pose (… in …) unavailable: commanding zero, N s …` | WARN, 1 s | `odom → base_link` missing; the TF helper's own ERROR above it has the TF reason |
| `Controller 'X' failed: <reason> -- commanding zero, N s …` | WARN, 1 s | The plugin refused this cycle (MPPI's optimizer, RPP's collision with distance and (v, w), a stair refusal); `N` against `failure_tolerance` is how close the goal is to aborting |
| `N m from the end pose but on another level: ground z … vs end pose z …` | WARN, 2 s | A path with heights whose end is within 1 m in x / y but not within `goal_z_tolerance` in z: the robot is on the goal and the goal is not reached (`lidar_height`, LIO z drift, or the wrong floor) |
| `Goal not checked: cannot transform the end pose …` | WARN, 5 s | The goal check skipped for want of TF |
| `[progress_checker] No progress: the robot stayed within R m of (x, y) for N s …` | ERROR | The progress checker's abort, with where and how long; the cause is in the controller's lines before it |
| `FollowPath aborted (controller 'X'): <reason>` | ERROR | The goal's end. `Controller patience exceeded: … last error: …` repeats the error that ran the window out, so the throttled WARNs above never hide it |
| `Reached the goal: d m and θ rad from the end pose` | INFO | Success, with the residual the goal checker accepted |

The WARNs are throttled (period in the table) because each fires every 50 ms
cycle while its condition lasts; the abort repeats the deciding error in full.

### Goal checking is gated on path progress

`isGoalReached()` differs from nav2 and the difference matters. nav2 asks the
goal checker whether the robot is near `end_pose_`; on a **patrol loop whose last
waypoint is its own start**, that is true on the very first control cycle and the
goal "succeeds" in about a second without the robot moving.

So this port first computes the remaining path length from the robot's place on
the path, and only consults the goal checker once that drops below
**`goal_reached_max_remaining_path`** (default 1.0 m; `<= 0` disables the gate).
The place is **forward-only progress** (`syncai_util::PathProgress`, 2026-11),
updated every cycle before any check in `isGoalReached()` can return, reset with
the path, and also what the feedback's `distance_to_goal` is measured from. It was
a whole-path nearest pose with a strict `<`, which put a robot on the shared
start/end point of a closed loop at the *start* — right as it set off, and wrong
when it came back: the patrol route (`navigate_through_poses`) ends there, and
the gate would never have opened. The feedback's search also compared the
local costmap's frame with the path's.

**RPP 3D's and MPPI's own nearest-pose searches** take the first local minimum of
the distance from their pruned front (`syncai_util::firstLocalMinimum`, 2026-11),
not the minimum over `max_robot_pose_search_dist`: an out-and-back route's return
leg lies on top of the outgoing one and inside that 10 m window for the last 5 m
before the turn, and pruning onto it cannot be undone.

### A path that turns back on itself is driven leg by leg

Forward-only progress passes a U-turn only once the robot has been within its
0.3 m hysteresis of the tip, and nothing made a controller go there. On robot01
(2026-11-02) a patrol P1 → P2 → P3 → P2 → P4, handed to MPPI whole, stopped
**0.84 m short of P3** and stood there with no log line until it was cancelled;
the same points as four MOVEs went through. MPPI's critics place each candidate
trajectory on the pruned path by its nearest pose over all of it
(`findPathFurthestReachedPoint`): within `prune_distance` of the tip the return leg
is in view, the slow candidates match it, and `PathFollowCritic`'s carrot lands at
2 D − `prune_distance` (D = distance to the tip) — under the robot at D ≈ 1 m —
and `PathAngleCritic` then turned the robot towards a carrot behind it. RPP 3D's
lookahead (the first pose L away) is on the return leg for D < L, so it would turn
back short of the tip instead (reasoned, not observed).

So `setPlannerPath()` cuts the path at its cusps (`syncai_util/path_cusps.hpp`:
a turn sharper than **`cusp_min_angle`**, 2.0 rad, between the **`cusp_window`**,
0.4 m, of path before and after a pose, on one floor) and hands the plugin one
leg at a time. Every leg but the last ends on its cusp facing the next leg, so the
plugin's own final rotation turns the robot there, exactly as at the end of a
MOVE; once the goal checker accepts that end, `isGoalReached()` hands over the
next leg (`Reached cusp k/n …: handing over leg …`), and `logPath()` lists the
cusps when the path arrives. `current_path_` stays whole: the feedback and the
remaining-length gate (measured to the current leg's end) read along all of it.
The robot therefore stops and turns in place on a turn-back waypoint, the one
place a patrol stops short of its end. A replan restarts at its first leg. Not handled: a route that crosses itself
on one floor (an X), where MPPI's nearest-pose match can still jump to the later
crossing leg.

The server's own place on the path passes the cusp on the handover
(`PathProgress::passTo()`), not by distance alone: the goal checker latches the
position at 0.10 m and the turn in place drifts (0.29 m from the tip in the sim,
2026-11-02). A place left on the outgoing leg never moves again, so beyond the
0.3 m hysteresis the remaining-length gate would have stayed shut for the whole
return leg and the goal never been reached. The planner's and the task_runner's
`PathProgress`, which do not see the legs, pass a cusp once the robot is within
0.6 m of it facing the way the path leaves it (`path_progress.hpp`).

A second fix lives here too: `end_pose_` is re-stamped with `Time(0)` before being
transformed into the costmap's global frame. Using the path's original stamp made
long goals die with a TF "extrapolation into the past" error, so `isGoalReached()`
never returned true.

### Paths with heights (OctomapPlanner3D)

Every 2D planner writes `z = 0` on every pose; `OctomapPlanner3D` (`Octo3D`)
writes the ground height, and its stair links climb. That breaks two planar
assumptions (2026-10), both measured on dp1f_1012's spiral stair, where the path is
back over the same x / y every **8.1 m, 2.5 m higher**:

- **RPP's nearest-pose search** (`transformGlobalPlan()`) spans
  `max_robot_pose_search_dist` (10 m) of path, so on x / y alone the turn above or
  below the robot is as near as the one it stands on — it could prune a whole turn
  and steer by the wrong one, and the local window (everything within the costmap's
  half-width, 2 m) took in the next turn too.
- **The goal check** compares x / y only, so a 2F goal can be "reached" from the
  1F corridor under it, on the way to the stairs.

So a path whose poses do not all share one z (`pathHasHeight()`, in
`syncai_util/include/syncai_util/path_height.hpp`, moved there in 2026-10 so
`syncai_mppi_controller` can share it) is matched in **3D**: RPP 3D's
(`FollowPathRPP3D`) nearest pose and window end, the patrol gate's nearest pose here, and the goal is only
reached within **`goal_z_tolerance`** (0.5 m) of the end pose's height. A path of
all-equal z — every 2D planner — takes the old planar code paths unchanged; a 3D
distance to a `z = 0` path on 2F would otherwise put every pose 5 m away. The
robot's height is **not in its pose**: lio_bridge projects `odom -> base_link`
(and `map -> odom`) onto the plane. It is `body_frame`'s z in the path's frame
minus `lidar_height` — Point-LIO's body, the one frame that still climbs, and the
planner's own start-height lookup with the same two values. Without that TF, a
path with heights fails RPP 3D's cycle (the server holds the robot for
`failure_tolerance`) and never reaches its goal, rather than being followed on
x / y. MPPI (`FollowPath`) matches the same way since 2026-10-20 (its
`PathHandler`, with the same `lookupGroundZ` and the same failure); the 2D RPP
(`FollowPathRPP`, removed 2026-10-28) was not height-aware. The server's goal
check is shared, so it applies to a path with heights whichever
controller follows it.

Climbing itself is the stair mode, next.

### Stairs (stair mode)

On a path with heights, RPP 3D and MPPI climb a stretch that rises
**`stair_min_rise`** (0.3 m) within **`stair_enter_dist`** (1 m) ahead in *stair
mode* (2026-10). Since 2026-10-20 that 1 m window is looked for anywhere from
the robot out to the controller's stopping distance from its commanded speed
(`StairMode::update()`'s `stop_dist`: MPPI's S-curve brake 0.75 v² / a, RPP
3D's constant deceleration v² / 2a — 0.48 / 0.32 m from 0.8 m/s), so the hold
begins that much earlier. The threshold slope (~17°) does not change; only how
far off it is seen. With the window alone, the brake from 0.8 m/s took all of
the 0.48 m a 30° ramp left (0.18 m at 20°), and the robot stood at the foot or
on the rise. Since 2026-10-22 it is never looked for from closer than the
padded footprint's reach along the body axis (`StairMode::bodyReach()`, 0.38 m
on the G23; `update()`'s `body_reach`): the local costmap's terrain layer
judges a flight lethal, so a controller reaching a stair at a crawl stops on its
own with the footprint's front edge at the first riser and its stopping
distance at 0, and the 1 m window from the centre ~0.6 m back may miss the
rise — robot01 on MPPI stood 8 s at the foot of dp's stairwell, in no stair
mode, with the window 0.28 m of rise against 0.3, until the optimizer crept on.
And since the same day the robot is meant to stand **`stair_standoff`** (1.0 m,
from its centre along the path; 1.5 m for its first hours; 0 would be the old aim
at the foot) before the stair when it switches the policy: the window is looked for that much
further out than the stopping distance. The aim at the foot had no margin for a
gait that overruns its brake — coming to dp's 2F step down at 0.8 m/s, robot01
kept 0.73 m/s for a second of the hold's brake, travelled 1.05 m instead of
0.48, and was switched to HIMLOCO standing on the first step. Not a minimum: a
slow approach stands up to the window's slack further back (~0.7 m at that
step), and an overrun eats into it (1.5 m stood the robot 1.1 m from the path's
edge of dp's 2F step up, from ~0.8 m/s). The floor from there to the stair
is walked as the stair is, blind on the stair carrot (accepted 2026-10-22; a
descent turns round and backs it). Keep `stair_standoff` + `stair_enter_dist`
under `stair_exit_ahead_dist`, or the robot would leave stair mode where it
stopped.
The detection, the policy switch and the latched report are
one state machine, **`syncai_util::StairMode`** (`stair_mode.hpp`; lifted out
of RPP 3D on 2026-10-20 when MPPI needed the same behaviour, so there is one
calibration of the gait's stair policy, not two), declared under each plugin's
name with the same values; the climb itself — the carrot and the speed cap
here, the optimizer bypass in `syncai_mppi_controller` — is each controller's,
and so is the braking, done with each one's own acceleration clamp:

1. **Stop and switch the gait policy.** The robot brakes at the clamps' rates; once
   commanded to a stop it calls the driver's **`set_policy_mode`** with
   **`stair_policy_mode`** (1, HIMLOCO — the index table is in
   `syncai_common/msg/RobotLowLevelMode.msg`: 0 PPO, 1 HIMLOCO, 2 CHAMP, 3 ISSAC)
   and stands **`stair_policy_settle_time`** (3 s, in the params file and in code). The call is fire and forget:
   the driver's UDP has no acknowledgement. If the service is not up, the cycle
   fails instead (the robot does not climb on the level-ground policy). If
   `<robot_id>/mode` already reports the stair policy, nothing is sent and the
   robot does not stop.
2. **Climb** at **`stair_linear_vel`** (0.5 m/s since 2026-10-19 — 0.2 made the
   stair tests crawl; 0.5 is unmeasured on HIMLOCO) with a **`stair_lookahead_dist`**
   (0.4 m; MPPI's is 0.6, see `syncai_mppi_controller`'s README) carrot. The short carrot keeps RPP off the inside of a spiral's turns
   (its narrowest treads) and keeps the collision projection (carrot + 0.36 m of
   footprint, ~0.76 m) short. Until 2026-10-20 that length also mattered
   against the local costmap's obstacle floor (0.58 m above the robot's
   ground), which the treads rose past after ~0.76 m. Since 2026-10-20 the
   collision check is **off** from the hold before the stair (`stair_standoff`
   back since 2026-10-22, so the floor up to it too) to the hold
   past its top — the climb is blind, by decision, in both controllers: on a
   3D map the local costmap runs `TerrainLayer` (`syncai_costmap`), which
   judges the flight itself lethal and opens nothing along the path, so the
   check would refuse every stair. Nothing on the stairs is avoided.
   **Corners** (2026-10-21): OctomapPlanner3D lays a switchback stairwell as
   straight flights joined by 90° corners on the landings (`syncai_global_planner`'s
   README, "Stairs"), and both controllers take them as the planner meant —
   straight up, stop, turn in place, straight on. `StairMode::findCorner()` finds
   the first pose of the pruned plan whose heading over 0.2 m behind and 0.2 m
   ahead differs by more than **`stair_turn_angle`** (45°); the carrot is held at
   that pose, the speed goes to zero (MPPI's S-curve, RPP 3D's clamp ramp) so the
   robot stops **`stair_turn_dist`** (0.15 m) before it, and from the stop the
   carrot is let past the corner, so its bearing (~90°) turns the robot in place.
   That turn is latched: in past `approach_rotate_min_angle` /
   `rotate_to_heading_min_angle` (14°), out only under **`stair_turn_done_angle`**
   (5°), because a flight is climbed straight or not at all. The same rule lines
   the robot up on the planner's 0.5 m lead-in when the floor route meets it at
   an angle. Without it a 0.6 m carrot rounded a 90° corner from ~0.5 m out,
   which on a landing a metre deep is the whole landing.
3. **Leave** once the path from **`stair_exit_behind_dist`** (0.8 m) behind to
   **`stair_exit_ahead_dist`** (3.5 m; 2.5 until 2026-10-21, when a U landing's
   ~2.8 m of level path — out, across, in — read level from its first corner and
   switched the policy on the landing) ahead rises less than `stair_min_rise`:
   stop, switch back, settle. Back to **`flat_policy_mode`**, or with -1 (default)
   to whatever the controller reported on `<robot_id>/mode` (data[0]) when stair
   mode began; with no report heard, it stays on the stair policy rather than
   guess an index. The long ahead window is what keeps a landing in stair mode:
   HIMLOCO turns in place and climbs the whole stairwell, and the policy goes
   back only on the top floor (decided 2026-10-21).

   **Down backwards** (`stair_descend_backwards`, 2026-10-21): HIMLOCO descends
   a flight facing the risers as it climbs one, so a path that goes down is
   walked in reverse. Each cycle `StairMode::slopeAhead()` reads which way the
   stair ahead goes — the first `stair_min_rise` of height change within 10 m
   of path — and a level stretch (a landing, the floor past the bottom) keeps
   the last reading, so a descent stays backwards across its landings and a
   well that goes down to a landing and up again is taken each flight its own
   way. In reverse the bearing is measured from the robot's tail (MPPI: the
   carrot's bearing from `yaw + π`; RPP 3D: `shouldRotateToPath()` from the
   tail and `sign = -1`), the speed is `-stair_linear_vel` on the same S-curve
   / clamp, and the pursuit arc is the same one (`wz` from |v|). At the top the
   first stair turn is the 180° that puts the tail on the lead-in; the landing
   corners are stopped at and turned on exactly as climbing; and in the hold
   at the bottom, once stopped and before the policy goes back, the robot
   turns round to the path (latched the same way), so the level-ground
   tracking takes over facing it. Unmeasured.

The windows come from dp1f_1012's Octo3D paths: level paths on 1F and 2F rise at
most 0.10–0.20 m within 1 m, the spiral stair 0.68 m, the short 2F ramp 0.47 m —
so the ramp is taken in stair mode too (1.9 m of it), and each stair path enters
stair mode exactly once. A change of mode not yet sent is dropped if a replan
stops asking for it; once sent it completes. A `reset()` mid-switch — a new
goal, or the server's own reset on a missing robot pose or a stale local
costmap — keeps it pending, and the robot holds and settles again for the full
`stair_policy_settle_time` once it next stands (2026-11; before, it counted as
done, so a one-second Point-LIO stall at the stair's foot let the robot climb
on a policy sent a moment earlier). `on_stairs_` outlives the goal on purpose — it is the
policy the gait controller is on — and the next goal's first level cycle switches
back. The state is latched on **`stair_mode`** (`std_msgs/Bool`).

**The planner's word gates the rise (2026-11-02).** The planner publishes
which stretches of its plan are stairs on **`plan_crossings`**
(`syncai_util/plan_crossings.hpp`: per crossing its kind, the gait it is
taken in and its pose span on the plan, tied to the plan by stamp and pose
count), and `StairMode` subscribes to it. With the word for the plan in hand,
the rise test above enters or keeps stair mode only where its window overlaps
a crossing laid in the **stair** gait; a rise the planner lays as a flat-gait
crossing — a walked door sill, a short slope, dp's 2F step (0.2 m over 0.5 m)
— or no crossing at all is walked on the flat policy, with one log line per
plan saying so ("Path rises … but the planner lays no stair there"). The
heights still say **where**: a stair crossing straightened by the planner
(`stairCrossing`) starts at the floor vertex before the flight, so the span
says whether and the rise where, and the hold comes at the same pose as
before. Without the word for this plan — a planner that keeps none (`known:
false`), a sidecar for an older plan, or none yet — the rise test alone
decides, exactly as before. The landing corners the planner lays (`turns` in
the sidecar) are not read yet: `findCorner()` still finds them from the
path's geometry. Of a crossing the stair mode reads two bits and nothing
else (2026-11-07): its gait, for this gate, and `slow`, handed to MPPI's
speed cap (`StairMode::plannerSlow()`); what the planner called it — a
kerb, a ramp, a narrow door — is for the log. And the plan a plugin holds
may be one **leg** of the planner's (the server cuts a path at its cusps,
above): the server hands each leg with its place in the plan
(`Controller::setPlanLeg`), the stair mode matches the sidecar on the
plan's stamp and pose count and shifts its spans to the leg. Until
2026-11-07 every leg of a cut plan failed the match — the word was for a
plan of another length — and a patrol out through a door and back lost the
door's speed cap and the sill's flat gait on every leg.

**Run on robot01**: HIMLOCO climbs the stair with the policy switched standing
still (no `Stand` first) and the settle time above. The state
machine is unit-tested in `syncai_util/test/test_stair_mode.cpp` on a
synthetic flight (enter, hold, send, settle, leave, the replan-before-send
cancel, the reset-after-send completion, a missing service, a level path,
and since 2026-11-02 the planner's word: a flat-gait crossing over the flight
never holds, a stair crossing that starts early holds where the heights say,
a word for another plan is ignored, and a flat-gait word takes a robot left
on the stair policy off it).

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
   predicted footprint collision throws `PlannerException("…collision ahead: …")`.

Any throw out of that cycle — this one, or a transform failure further up —
leaves the accel-clamp baseline at zero, because nothing was commanded; see
"Acceleration clamping".

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

`last_cmd_vel_` is zeroed in `reset()` — called once per goal by
`ControllerServer::computeControl()`, so every new goal ramps from a standstill,
and on every cycle a robot-pose failure makes the server stop the robot without
calling RPP (see `failure_tolerance` above).
`rotateToHeading()` applies the same pattern with `max_angular_accel`, plus a
`sqrt(2·α·θ)` cap so the in-place rotation decelerates into its target instead of
overshooting. Last of all (not upstream, 2026-10-14) it raises the rotation to
`rotate_to_heading_min_angular_vel` (0.45), toward the target: RL does not turn
in place below its floor, and the ramp's first cycle (0.16 rad/s) and the stop
cap's last ~0.014 rad are both under it. The value is the `cmd_vel` value,
pre-scale. It was 0.3 from 2026-10-15 (0.21 = 0.3 / `scale_turn` 1.40 for a day
before that) and went to 0.45 on 2026-10-21, because 0.3 is not above the
gait's floor: on robot01 MPPI's ±0.30 rad/s in front of a blocker turned the
robot at 0–0.16 rad/s for ~9 s, while 0.65 rad/s turns ran at 0.5–0.9. The
floor itself lies somewhere in 0.3–0.65 and is unmeasured. MPPI's
`in_place_wz_min` and `FollowPathRPP3D`'s copy of this parameter are the same
floor, so change all three together. 0 disables it.

It is also zeroed on **any throw out of the compute cycle**, which is the only
thing `computeVelocityCommands()` does: it takes `mutex_`, calls
`computeVelocityCommandsImpl()` (the real body), and on the way out of a
`catch (...)` writes the stop baseline and rethrows. A cycle that throws commands
a stop — the server answers with a zero `cmd_vel` while `failure_tolerance`
lasts, and past that patience fails the goal and calls `publishZeroVelocity()` —
so the baseline has to follow the stop rather than the `(v, ω)` that was
abandoned halfway through being computed. Nothing else would notice: the baseline
is the last *command* precisely because it is not the measurement.

Without it the clamp window stayed open around the pre-collision speed, and the
first cycle after the obstacle cleared stepped `0 → ≥0.55 m/s` in 50 ms — about
11× `max_linear_accel`, right where something had just been detected ahead —
with the same shape on the angular side coming out of rotate-to-heading
(`0 → ≥0.49 rad/s`).

The wrapper is catch-all rather than one zeroing per `throw`: `"collision
ahead!"` is the frequent case, but `transformGlobalPlan()`'s empty-plan and
TF-failure throws and `costAtPose()`'s "costmap too small" all reach the server
by the same path, and a `throw` added later would otherwise reintroduce the stale
baseline silently. `reset()` covers none of them — it runs at goal start and on
cycles that never reach RPP, while these throws happen inside RPP mid-goal.

The reset lives in `reset()` and **not** in `setPlan()`, which is the bug this
cost us once: `setPlan()` is also the mid-navigation replan path. Every replan
in `move.xml` (once a second until 2026-10, only when `IsPathValid` reports the
path blocked since)
hands `FollowPath` a fresh path, which reaches the server
as an action preempt and lands in
`updateGlobalPath() → setPlannerPath() → setPlan()`. Zeroing the baseline there
clamped the very next command to a single accel step, so the command re-ramped
from ~0 on every replan. Measured on the real robot over a two-goal run (at the
time the BT replanned every ~3 s and RPP was still configured with
`desired_linear_vel: 0.8` / `rotate_to_heading_angular_vel: 1.3` — hence the
0.8 m/s and 1.3 rad/s plateaus below; today's values are 0.60 / 0.65, see
"Velocity-scale calibration"):

```
t= 3.150  vx +0.650 -> +0.050    (/plan published at 3.058)
t= 6.299  vx +0.300 -> +0.050    (/plan at 6.196)
t=16.158  vx +0.800 -> +0.050    (/plan at 16.056)
t=19.307  wz +1.300 -> +0.160    (/plan at 19.205)
```

`0.050 = max_linear_accel × control_duration` and `0.160 = max_angular_accel ×
control_duration` — i.e. the first step off a zero baseline, every collapse
~100 ms behind a `/plan`. cmd_vel *timing* was fine throughout (50.0 ms median,
p90 50.8 ms), so this only ever showed up as a value discontinuity: about 16% of
the drive spent re-accelerating, most visible during rotate-to-heading, where
the constant 1.3 rad/s plateau was cut to 0.16 and needed 8 cycles to climb back.

### Angular clamping on both branches

The angular clamp is applied twice, once per branch: inside `rotateToHeading()`
for in-place rotation, and again after `angular_vel = linear_vel * curvature` for
path tracking. The second one was missing originally, which made *exiting*
rotation a cliff while *entering* it was smooth (trace measured at the old
`rotate_to_heading_angular_vel: 1.3` plateau — the mechanism is unchanged at
0.65, only the numbers are smaller):

```
entering rotation                    leaving rotation
52.096  vx 0.800  wz 0.195           52.596  vx 0.300  wz 1.300
52.146  vx 0.750  wz 0.497           52.646  vx 0.350  wz 0.235   <- one cycle
52.196  vx 0.700  wz 0.657
  ...   -0.05/cycle  +0.16/cycle     (≈ -21 rad/s², clamp not applied)
```

and it fed a weave: the robot overshot the heading on exit and re-entered
rotate-to-heading the other way one cycle later (`wz 1.300 → 0.024 → -0.064 →
… → -0.815`).

The clamp deliberately does **not** wrap `rotateToHeading()`'s output. That
function ends with a `sqrt(2·α·θ)` deceleration cap; a symmetric acceleration
clamp applied on top of it would pull the command back up and cancel the cap.

## Docking (2026-10 → removed 2026-11-12)

From 2026-10 to 2026-11-12 this node served a second action, `DockToPose`
(`syncai_common`, on `dock_to_pose`): drive to ONE planar pose and stop there,
the last metre onto a charging dock (`dock.xml`) or 1 m straight back off it
(`undock.xml`). Its loop, `dockControl()`, shared the per-cycle body with
FollowPath (`computeVelocityCycle()`, still split out) and one `timed_mutex`
with it, so the two servers refused each other (`CONTROLLER_BUSY` 302). The
plugin was `syncai_controller::DockingController`, a port of Humble's
`nav2_graceful_controller` (Park & Kuipers' smooth control law, the one
`opennav_docking` uses), loaded twice as `Docking` and `Undocking`
(`allow_backward`) with its own `dock_goal_checker`, the target transformed
into odom once at acceptance, and **no costmap** — the dock is itself a lethal
cell, so the last metre was driven blind after a staging leg. It went with the
dock and undock trees, the backend's DOCK / UNDOCK steps and the
`DockToPose` action itself; git history has the code, and the
params file's `Docking` block as it was has the rationale for every value.

What it measured that still holds for anything driving the gait under MPC:
the walking floor under MPC is **0.05 m/s** (2026-10-14; RL's is ~0.3 m/s),
and the in-place turn floor somewhere in 0.09–0.15 rad/s (0.03 held for a
moment but not for a sustained turn: two robot01 undocks tapered to
0.087–0.094 rad/s and the yaw froze until the progress checker aborted;
0.15 was used, unmeasured). Both are pre-scale, and re-measured by stepping a
20 Hz `cmd_vel` down until odom shows no steady motion, holding each step for
several seconds, after any `scale_fwd` / `scale_turn_*` change. The reverse
floor and `scale_back` were never measured.

## Plugins

Three plugin types, all loaded through pluginlib against `syncai_nav_core`
interfaces. Selection is by ID: the parameter names an ID, and `<id>.plugin`
names the C++ type.

**Controllers** (`syncai_nav_core::Controller`, parameter `controller_plugins`) —
two entries (five until 2026-10-28, when the 2D RPP `FollowPathRPP` went with
2D navigation; four until 2026-11-12, when `Docking` / `Undocking` went with
docking):

| ID | Plugin | Used by |
|---|---|---|
| `FollowPath` | `syncai_mppi_controller::MPPIController` | `follow_path` (move.xml, patrol.xml) |
| `FollowPathRPP3D` | `syncai_controller::RegulatedPurePursuitController3D` | `follow_path`, the A/B (`controller_id` in move.xml) |

The IDs are what a goal's `controller_id` field selects.

- **An empty `controller_id` fails a `FollowPath` goal** ("does not exist"). The
  server only falls back to the sole plugin when exactly one is loaded, as it
  did while RPP was alone.
- Both plugins share the one local costmap and its mutex. Only the selected
  one runs a cycle.

**Progress checkers** (`syncai_nav_core::ProgressChecker`, parameter
`progress_checker_plugin` — singular, only one is loaded):

| Plugin | Fails when |
|---|---|
| `SimpleProgressChecker` | The robot has not moved `required_movement_radius` metres within `movement_time_allowance` seconds |

`PoseProgressChecker` (rotation also counts as progress) was deleted on
2026-10-28: ported, built, never loaded.

**Goal checkers** (`syncai_nav_core::GoalChecker`, parameter
`goal_checker_plugins` — a list; a `FollowPath` goal's `goal_checker_id` selects
one):

| Plugin | Checks |
|---|---|
| `SimpleGoalChecker` | xy distance, then yaw. `stateful: true` latches the xy check once satisfied, so the final in-place rotation cannot fail it by drifting. `symmetric_yaw_tolerance` also accepts goal yaw + 180° for symmetric robots. |

`StoppedGoalChecker` (also requires the robot to have stopped) and
`PositionGoalChecker` (position only) were deleted on 2026-10-28: ported,
built, never loaded.

One `SimpleGoalChecker` instance is configured, `goal_checker` (navigation);
`dock_goal_checker` (0.05 m / 0.1 rad) went with docking on 2026-11-12.
**An empty `goal_checker_id` resolves to the first entry of
`goal_checker_plugins`, however many are loaded** (2026-10). Upstream — and this
server until then — only fell back when exactly one was loaded, which was fine
with one entry; adding `dock_goal_checker` (2026-10 → 2026-11-12) would
otherwise have failed every BT-issued `FollowPath` at once, because `move.xml` sets `controller_id` and no
`goal_checker_id` (the BT node defaults it to `""`). The asymmetry with
controllers is deliberate: MPPI vs RPP is a decision a goal must make, while
goal checkers differ only in tolerance and the first listed is the navigation
default by construction — keep it first in the yaml.

Note the coupling to the controllers: `getTolerances()` feeds `goal_dist_tol_`,
which is what decides when RPP switches into rotate-to-goal-heading — so widening
`xy_goal_tolerance` also makes the robot start its final rotation earlier.

## Parameters

Server-level (`/**/controller_server`):

| Parameter | Default | Notes |
|---|---|---|
| `controller_frequency` | `20.0` | Control loop rate; also `1/f` is RPP's `control_duration_`, the `dt` in both accel clamps. Startup only: a runtime set is refused (must stay 1 / MPPI's `model_dt`) |
| `min_x_velocity_threshold` | `0.0001` | Odom twist below this reads as zero, in both places the server hands the twist on — RPP's `computeVelocityCommands()` and the goal checker's `isGoalReached()`. Config sets `0.001`: an order of magnitude above the default but still far below Point-LIO's body-sway noise, so in practice it only decides what `StoppedGoalChecker` accepts as "stopped" |
| `min_y_velocity_threshold` | `0.0001` | Config sets `0.5` — a differential/quadruped base has no meaningful lateral velocity, so this discards it |
| `min_theta_velocity_threshold` | `0.0001` | Config sets `0.001`, same reasoning as `min_x_velocity_threshold` |
| `failure_tolerance` | `0.0` | Seconds to tolerate controller exceptions; `-1.0` = forever. **Config sets `3.0`**, which is materially different from the default: at `0.0` the first `PlannerException` out of the controller (MPPI's "Optimizer fail to compute path" when every candidate collides, RPP's "collision ahead", a transform failure) fails the goal outright; with a positive value the server logs the exception, publishes a **zero** `cmd_vel` for that cycle, and only fails with "Controller patience exceeded" once that long has passed since the last valid command, with the robot braking rather than coasting. It was `0.3` (six control cycles of grace for a TF hiccup or a sway-induced collision flag) until 2026-10, when it was raised so a blocked path **re-routes**: the detour comes from the BT's 1 Hz path-validity check (`IsPathValid`, and the replan it triggers) — then on a 2D global costmap that updated at 1 Hz out of phase with it, on `Octo3D`'s dynamic overlay since 2026-10-21 — up to ~2.2–2.5 s after a blocker appears inside RPP's ~0.6 m projection, and at `0.3` `FollowPath` aborted first — its `RecoveryNode` cleared the local costmap and re-sent the same old path, RPP refused again, and the whole `NavigateToPose` failed in under a second. The timer resets on the first valid command, so the longer window costs nothing once the new path arrives — and deliberately **not** when a replanned path is accepted on preempt (`updateGlobalPath`): a drivable path resets it on its first cycle anyway, and while `is_path_valid` ran RPP's own footprint test on the 2D costmap (until 2026-10-28) a refused path was replanned once a second, so a reset per preempt would leave the progress checker's 30 s as the only abort. The progress checker's 30 s remains the outer bound on standing still. Each attempt still aborts ~3 s after the controller starts refusing (for MPPI, only once no candidate trajectory clears the blocker); how many attempts (and so how long the robot waits for a blocker to leave) is `move.xml`'s `FollowPath` retry count, four since 2026-10-14 (~16 s, to outlast `IsPathValid`'s 10 s `blocked_time`). Covers controller exceptions and robot-pose lookup failures, not the progress checker — see above |
| `goal_reached_max_remaining_path` | `1.0` | The patrol-loop gate above, from the robot's forward-only place on the path to the end of the leg being driven; `<= 0` disables |
| `cusp_min_angle` | `2.0` | A turn sharper than this (rad) is a cusp, where the path is cut into legs (above); `<= 0` hands every path over whole |
| `cusp_window` | `0.4` | Path length (m) read before and after a pose for its turn; four cells, so an A* return leg one cell aside is one turn |
| `body_frame` | `pointlio_body` | The frame whose z, minus `lidar_height`, is the robot's ground height, for paths with heights (above). The launch file prefixes it with `<robot_id>/`. Also read by the controller plugins |
| `lidar_height` | `0.481` | `body_frame`'s height above the feet — the planner's `Octo3D.lidar_height`; change both together |
| `goal_z_tolerance` | `0.5` | Paths with heights: how far the ground under the robot may be from the end pose's height for the goal to count. Dynamically reconfigurable |
| `progress_checker_plugin` / `goal_checker_plugins` / `controller_plugins` | see above | `goal_checker_plugins` is ordered: the first entry is the empty-id default |

The three velocity thresholds, `failure_tolerance` and
`goal_reached_max_remaining_path` are **dynamically reconfigurable** — but the
callback takes the same mutex the control loop holds, so a change is rejected
with a warning while a goal is executing. Plugin parameters (anything with a `.`
in the name) are skipped by that lock and handled by each plugin's own callback.
Every callback on the node sees every set, and rclcpp stops at the first one
that refuses it — after the callbacks before it have already applied their
share. RPP 3D therefore vets the whole batch first and refuse it
with a `reason` before touching a member (2026-11-03: until then their
"rejecting update" was a `continue` that still returned success, so the node
held a value the controller was not using); a caller mixing server and plugin
parameters should send the server's in a set of its own, first.
**`body_frame` and `lidar_height` are refused at runtime too** (2026-11;
accepted and never used before): the plugins copy them at startup. **Every RPP 3D
parameter now applies live** (`approach_velocity_scaling_dist`,
`rotate_to_heading_min_angular_vel`, `max_robot_pose_search_dist`,
`use_collision_detection` and `use_interpolation` were read once), and so do
**StairMode's window and settle time on both controllers**
(`StairMode::setParameter()`: `stair_min_rise`, `stair_enter_dist`,
`stair_standoff`, `stair_exit_behind_dist`, `stair_exit_ahead_dist`,
`stair_policy_settle_time`) — the console's `stair_standoff` was stored and
unused on both until 2026-11. `use_stair_mode` and the two policy ids are
refused: flipped mid-goal they could leave the robot in the stair policy.
**`controller_frequency` is refused at runtime** (2026-11): every controller
reads it once, at startup — MPPI pairs it with `FollowPath.model_dt` (also
refused) for the warm-start shift, and MPPI and RPP 3D take 1 / f as
their acceleration clamps' dt — so a live change left all of them on a dt that
was no longer true. Change both in the params file and restart. A preempt that
**switches controllers** resets the newcomer; one that
keeps the controller still does not.

### The runtime overlay (console settings)

The launch loads two files: the factory `params/controller_server_params.yaml`
(this package, tracked, never written at runtime) and then, when it exists,
**`config/params/<robot_id>/controller_server.yaml`** beside the system INI
(gitignored). The overlay is written by `syncai_backend`'s controller settings
page and holds only the keys the operator changed, under the same
`/**/controller_server: ros__parameters:` key, so a later file wins key by key
and every untouched key keeps following the factory file through a `git pull`.
The backend sets the live value over `set_parameters` / `get_parameters` and
persists what it read back; a parameter the node only reads at startup is
written to the overlay alone and takes effect on the next start of the nav
session. Which parameters the page exposes, with their ranges and
descriptions, is the backend's schema — nothing here declares a descriptor.

Falling back to the factory values: delete the overlay (or one key of it) and
restart the session, or launch with `use_runtime_params:=false`. An overlay
that does not parse is skipped with an error in the controller's log rather
than handed to rcl, which would abort the process on every boot; the log
says how many keys a good one carried. `runtime_params_file:=` points at
another file.

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
- `max_linear_accel: 1.0` — the no-velocity-smoother clamp; 0 → 0.60 m/s in ~0.6 s.

### Velocity-scale calibration

Two RPP speeds were lowered because the gait controller does not deliver the
speed it is commanded, and the controller's geometry silently assumes it does:

- `desired_linear_vel`: **0.8 → 0.60**. At 0.8 the robot actually made
  0.62 m/s (ratio 0.78), so pure pursuit was computing its arcs
  (`w = v · curvature`) for a speed a quarter higher than reality.
- `rotate_to_heading_angular_vel`: **1.3 → 0.65**. At 1.3 it delivered
  0.695 rad/s (ratio 0.53) — the controller believed every in-place turn was
  finishing about twice as fast as it was, exited rotate-to-heading still
  off-heading, and re-entered. That is the weave the angular-clamp fix above
  only partly explained.

The per-direction correction itself does **not** live here. It is
`syncai_driver_manager`'s `scale_fwd` / `scale_back` / `scale_left` /
`scale_right` / `scale_turn_l` / `scale_turn_r` (`driver_manager_params.yaml`,
currently 1.40 / 1.0 / 1.0 / 1.0 / 1.40 / 1.40), multiplied into `cmd_vel`
before it goes over UDP. The YAML's own header carries the two plateau runs the
values come from; the parameter table in
[`syncai_driver_manager/README.md`](../syncai_driver_manager/README.md) lists
them — that package left this repo in 2026-10 for
`chungweeeei/SyncAI-Robot-Driver-Manager` and is vcs-imported back into `src/`,
so the link resolves in a materialised workspace, not in a bare clone. The
two packages' values are **one calibration**: the RPP speeds were chosen to sit
inside the range where the plateaus were measured, and the scale factors fold
the residual error at *those* commands back in. The response is not a fixed
gain — the two measurement runs disagree with each other, and the gait
controller's tracking falls off as the command grows — so raising either speed
here without a fresh plateau measurement at the new command (hold it 3–5 s,
record what odom settles at) reintroduces exactly the mismatch these numbers
were lowered to remove.

### Params

There is a single `controller_server_params.yaml`. It used to be split into a
2D/sim file and a `*_3d_params.yaml` for the real robot; the variants were merged
back once the Isaac Sim fleet was retired, and the values that remain are the
real-robot ones:

| Parameter | Value | Why |
|---|---|---|
| `use_sim_time` | `false` | No `/clock` on the real robot |
| `xy_goal_tolerance` | `0.10` | The robot's floor speed is ~0.3 m/s, so it cannot creep into a 5 cm window (the old sim value). Raised from `0.15` to `0.3` on 2026-10-14 after MPPI's decelerating approach stalled the robot 0.23 m and 0.29 m short, and lowered back the same day because 0.3 ended an ordinary MOVE too far from its goal — nothing has fixed those stalls, so they can return. Lowered `0.15` → `0.10` on 2026-10-29 with MPPI's `approach_velocity_scaling_dist`: robot01 entered 0.15 near `vx_max` and stopped 0.26–0.38 m past the goal (succeeded only because `stateful` had latched the position); at the 0.30 floor the stop is ~0.1 m, so 0.10 holds and 0.05 would not. Estimated — "Reached the goal: X m" in the controller log is the check. Equal to MPPI `GoalAngleCritic.threshold_to_consider` so the final turn starts only once the position is accepted; change the two together |
| `yaw_goal_tolerance` | `0.2` | The gait controller does not turn below ~0.3 rad/s, so the final rotation cannot slow into a narrow window. Raised from `0.1` (2026-10-14): ±0.2 rad takes ~1.3 s to cross at that floor, enough for the command latency and the stop of a heavy body. Estimated, not measured |
| Local costmap sources | `terrain_layer` (`pointlio/body_cloud`) | The only sensor layer since 2026-10-28. From 2026-10-20 it ran beside an `obstacle_layer` (`scan` and `pointlio/body_cloud` in a height band), exactly one of the two picked per map by `map_type` against the planner's `map_available` (`MapTypeGate`; never both, since they disagree on a ramp); the obstacle layer and the gate went with 2D navigation |

The terrain layer takes the cloud's own header frame (`<robot_id>/pointlio_body`)
as its height reference, so it needs no per-robot launch override, but it only
works **after** `syncai_localizer` has established the TF chain
(`/<robot_id>/relocalize`, the INI's `[initial_pose]` or an `initialpose`
message).

## Running

```bash
ros2 launch syncai_controller controller_server.launch.py
ros2 launch syncai_controller controller_server.launch.py \
    system_config:=/path/to/other.ini
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
  `body_frame`) with the `<robot_id>/` prefix, because ROS
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

- **A costmap that is not current stops the robot** (2026-11). `driver_manager`
  has no `cmd_vel` watchdog, so the wait (`waitForCurrentCostmap()`) publishes
  zero, resets the controller as a missing robot pose does, honours a cancel,
  and fails the goal once `failure_tolerance` — never less than 1 s, so the
  one-update gap after a costmap clear cannot — passes without a valid
  command, logging which layer and why ("terrain_layer: newest cloud on … is
  N s old (cloud_timeout 1.0 s)"). Until then it spun with nothing published,
  no cancel and no bound, which was latent only because the terrain layer
  never reported a stale cloud: with `pointlio/body_cloud` gone, MPPI drove
  on an all-free costmap. The robot stops wherever this happens, a flight
  included (stair mode climbs without the costmap, but not without the
  odometry a dead Point-LIO also takes).
- **Nothing smooths `cmd_vel`.** `max_linear_accel` / `max_angular_accel` are the
  only thing between the controller and the gait controller. Raising
  `desired_linear_vel` without checking them gives a step command.
- **The accel clamp baseline is the last command, not measured speed** — so if
  the robot physically fails to track the command, the clamp will not notice and
  will keep ramping. That is intentional (gait noise), but it means the clamp is
  not a safety feature.
- **The footprint's padded inscribed radius is the planner's `robot_radius`.**
  The local costmap's `[[0.35,0.22],…]` with `footprint_padding` 0.01 is
  ~0.23–0.25 m to the side, the 0.25 m that `Octo3D` (`robot_radius`) and the
  terrain layer (`robot_radius`) keep from walls; change them together. Until
  2026-10-28 the global costmap carried the same rectangle with padding 0.03,
  so that the planner's `is_path_valid` perimeter-on-LETHAL test (2D paths)
  was stricter than the one RPP runs here; the global costmap has had no
  layers and no footprint test since, and `Octo3D` judges its own paths.
- **A plugin load failure calls `exit(-1)`.** A typo in a `.plugin` type string
  takes the whole process down at startup rather than degrading.
- **The byobu session launches this server on its default params file** —
  `config/sessions/start_nav.yaml` passes no `params_file:=` override, for either
  the planner or the controller. That is correct now that the separate
  `*_3d_params.yaml` variants have been merged back into the defaults; it was a
  bug while they were separate (the 3D session silently ran with
  `use_sim_time: true` and no pointcloud observation source).
- **Dynamic parameter changes are refused mid-goal** for the server's own
  (undotted) parameters, with `successful: false` and a warning. Set them while
  idle. Plugin parameters apply mid-goal.
- **A value in `config/params/<robot_id>/controller_server.yaml` beats the
  factory file.** When a factory change seems to do nothing on one robot, look
  for the key there first (the controller's log names the overlay at startup).

Upstream references: [`nav2_controller`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_controller)
and [`nav2_regulated_pure_pursuit_controller`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller),
whose README has the full RPP algorithm write-up and diagrams.
