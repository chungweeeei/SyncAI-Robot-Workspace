# syncai_costmap

Named `syncai_costmap_2d` until 2026-10-28: the package, its include directory
(`syncai_costmap/…`), the C++ namespace (`syncai_costmap::`) and the pluginlib
names (`syncai_costmap::TerrainLayer`, base `syncai_costmap::Layer`) were
renamed once 2D navigation had left it, so the name stops suggesting a 2D
stack. The core classes followed: upstream's `Costmap2D`, `Costmap2DROS` and
`Costmap2DPublisher` are `Costmap`, `CostmapROS` and `CostmapPublisher`, in
`costmap.hpp` / `costmap_ros.hpp` / `costmap_publisher.hpp` (the grid is still
a 2D grid; the name just no longer says so, and `nav2_msgs::msg::Costmap`, the
`…/costmap/raw` message, is always written fully qualified beside it). A
workspace built before the rename keeps
`build/syncai_costmap_2d` + `install/syncai_costmap_2d`; delete them.

The occupancy representation the planner and controller both plan against: a
layered 2D costmap with pluginlib layers: a terrain layer judging the lidar
against the ground under each cell, and inflation. Port of `nav2_costmap_2d`,
cut down to what 3D navigation uses: since 2026-10-28 (2D navigation removed)
the static and obstacle layers, the keepout filter and the standalone test
harness are gone (git history has them).

It is used as a **library, not a node**. `syncai_global_planner` and `syncai_controller`
each construct their own `CostmapROS` inside their own process:

| Owner | Costmap | Typical config |
|---|---|---|
| `syncai_global_planner` | `global_costmap` | **No layers**, `global_frame: map`: Octo3D reads no cells; the planner uses it for its frame, TF buffer, robot pose and callback group. Static + obstacle + inflation + keepout filter until 2026-10-28 |
| `syncai_controller` | `local_costmap` | 4×4 m rolling window (3×3 until MPPI, 2026-10), `global_frame: odom`, terrain + inflation |

There is no standalone runner any more: `costmap_2d_node` and its three test
launch files went with the 2D layers they exercised.

## Layout

```
src/costmap.cpp              the raw grid: cells, world↔map conversion, raytracing
src/layered_costmap.cpp         the layer pipeline
src/layer.cpp, costmap_layer.cpp  Layer base classes (+ joinWithParentNamespace)
src/costmap_ros.cpp          the ROS wrapper node — most of the local complexity
src/costmap_publisher.cpp    OccupancyGrid / Costmap / update publishing
src/clear_costmap_service.cpp   the three clear_* services
src/terrain_grid.cpp            the ROS-free 2.5D grid behind TerrainLayer (gtest: test/test_terrain_grid.cpp)
src/footprint.cpp, footprint_collision_checker.cpp, costmap_math.cpp, array_parser.cpp
plugins/terrain_layer.cpp       ┐ pluginlib layers
plugins/inflation_layer.cpp     ┘
include/.../costmap_filters/filter_values.hpp   constants only (NO_SPEED_LIMIT, read by MPPI)
```

No `launch/`, `params/`, `rviz/` or `scripts/`: the costmaps are configured by
their owners' params files (`planner_server_params.yaml`,
`controller_server_params.yaml`).

Everything builds into **one** shared library, `libsyncai_costmap.so`, which
is both the linked library and the pluginlib plugin library — `costmap_plugins.xml`
points at the same `.so`.

## Core objects

**`Costmap`** — the grid itself: an `unsigned char[]` plus resolution and
origin, with world↔cell conversion, Bresenham raytracing, window copy and a
recursive mutex. No ROS.

**`LayeredCostmap`** — owns the master costmap and one ordered list of
**plugins** (layers). Its `updateMap(x, y, yaw)` is the whole pipeline:

1. If `rolling_window`, re-centre the origin on the robot.
2. Ask every layer for its dirty region (`updateBounds`), accumulating a
   bounding box. A layer that *shrinks* the box is logged as an "Illegal
   bounds change".
3. Clamp the box to the grid, `resetMap(window)` on the master, then each
   layer's `updateCosts()` writes into it, in order.

Upstream's second list, the **costmap filters** (with a `primary_costmap_`
the layers wrote into, copied to the master before the filters ran on it, so
filter output never fed back into the layers), went with its only plugin, the
2D keepout filter, on 2026-10-28 (3D zones are Octo3D's own); `filters` is no
longer a parameter. With **no layer** `updateMap()` returns right after step 1,
and `isCurrent()` is true: that is the planner's global costmap.

**`Layer` / `CostmapLayer`** — the plugin interface. `Layer` is the bare
`updateBounds`/`updateCosts`/`activate`/`deactivate`/`reset` contract;
`CostmapLayer` adds a private grid plus the merge helpers
(`updateWithMax`, `updateWithOverwrite`, `updateWithTrueOverwrite`, …).

**`CostmapROS`** — the ROS wrapper. This is where the local design decisions
live, below.

### Cost values

| Constant | Value | Meaning |
|---|---|---|
| `FREE_SPACE` | 0 | |
| `MAX_NON_OBSTACLE` | 252 | Highest cost that is still traversable |
| `INSCRIBED_INFLATED_OBSTACLE` | 253 | The robot's inscribed circle would hit something |
| `LETHAL_OBSTACLE` | 254 | Actual obstacle |
| `NO_INFORMATION` | 255 | Unknown |

`CombinationMethod` (per-layer, the `combination_method` parameter) decides how a
layer merges into the master:

| Value | Name | Used by |
|---|---|---|
| `0` | `Overwrite` — replace, except `NO_INFORMATION` | (the static layer, until 2026-10-28) |
| `1` | `Max` — keep the larger; a known value overrides unknown | `TerrainLayer` (`updateWithMax`) |
| `2` | `MaxWithoutUnknownOverwrite` — as Max, but master's unknown stays unknown | Preserving "unknown means unknown" |

## CostmapROS: no lifecycle, so what changes

**Three-phase construction.** There is no lifecycle manager in this stack, so
nav2's transitions are exposed as ordinary calls the owner makes by hand:

```cpp
auto costmap = std::make_shared<CostmapROS>("local_costmap", "/robot01", "local_costmap");
costmap->init();       // ≈ on_configure : tf, plugins, publishers, services, executor thread
costmap->activate();   // ≈ on_activate  : wait for TF, start the update thread, dyn params
// ... later
costmap->deactivate(); // ≈ on_deactivate: stop layers, join the update thread
```

The constructor only declares parameters. `init()` must run after the object is
held by a `shared_ptr` — it uses `shared_from_this()` for parameter declaration,
the TF listener and plugin initialization. The destructor calls `deactivate()`
itself, because a joinable `std::thread` destroyed without `join()` calls
`std::terminate`, and the loop still touches the objects being torn down.

**`activate()` blocks** until `global_frame → robot_base_frame` is available. On
a cold start that means the whole hosting node waits there — which is exactly why
the byobu scripts stagger the planner and controller behind `sleep`.
A shutdown during that wait (Ctrl-C, or `switch_mode` tearing the session down
before odometry ever arrived) returns from `activate()` **without** starting the
layers or the update thread. Until 2026-10 it fell through to `start()`, whose
first layer `subscribe()` threw `RCLError` "context is invalid" on the dead
context, and `planner_server` / `controller_server` aborted (exit 134) instead of
exiting. A host node must check `rclcpp::ok()` after `activate()` returns, too:
both servers' `main()` do, because constructing an executor on the dead context
throws the same way ("failed to create guard condition").

**The costmap gets its own sub-namespace.** The node is constructed with
`__ns:=<parent_namespace>/<local_namespace>`, so the controller's costmap lives at
`/robot01/local_costmap` and the planner's at `/robot01/global_costmap`. Without
that, two costmaps in one robot namespace would collide on `costmap`,
`published_footprint`, `get_costmap` and the clear services.

That creates a problem for **input** topics: `pointlio/body_cloud` and `plan`
are published in the parent namespace, not the costmap's sub-namespace.
`Layer::joinWithParentNamespace()` solves it by stripping the last segment of
the node namespace before resolving a relative topic — so `pointlio/body_cloud`
in the params becomes `/robot01/pointlio/body_cloud`, not
`/robot01/local_costmap/pointlio/body_cloud`. Absolute topic names are passed
through untouched.

**A dedicated callback group and executor thread.** TF, the footprint
subscription and the layers' sensor subscriptions all run on one
mutually-exclusive callback group spun by `executor_thread_`, separate from the
owner's executor. Two details in there are load-bearing and easy to break:

- `tf_buffer_->setUsingDedicatedThread(true)` — the buffer is filled by
  `executor_thread_` while `canTransform`/`lookupTransform` are called with a
  timeout from another thread. Without this flag the buffer refuses to block and
  every timed lookup times out immediately.
- The `TransformListener` is created with `spin_thread=false` and
  `sub_options` passed for **both** the dynamic and static subscriptions. Miss it
  on `/tf_static` and that subscription lands on the default callback group,
  which nothing spins until `rclcpp::spin()` runs — so `activate()` deadlocks
  waiting for a static transform that is never processed.
- A `tf2_ros::MessageFilter` on `getTfBuffer()` belongs on this group, wherever
  it is created: `getCallbackGroup()` (2026-10-22) hands it out for that.
  `Octo3D`'s dynamic overlay ran its filter on a thread of its own, and on
  robot01 this group's thread then stopped for good: `/tf` stopped reaching
  the buffer, and `getRobotPose()`, which answers the newest transform however
  old, kept returning a pose from 20 m away.

**The map update thread** runs `updateMap()` at `update_frequency` and publishes
at `publish_frequency`, holding `_dynamic_parameter_mutex` for the whole cycle so
a parameter change cannot land mid-update. It also republishes when time moves
backwards, which happens when switching to sim time.

## Layers

Selected by the `plugins` list (each entry an ID whose `<id>.plugin` names the
type). Two layers are left, both in the
controller's local costmap: `TerrainLayer` (the sensor layer) and
`InflationLayer`. The default `plugins` is **empty** (2026-10-28; upstream's is a
static layer), which the planner's global costmap relies on by leaving the key
out — an empty YAML list cannot be typed.

Removed 2026-10-28 with 2D navigation: `StaticLayer` (the gridmap from
`map_server`), `ObstacleLayer` with its `ObservationBuffer` (height-band
marking and raytrace clearing of `scan` / `pointlio/body_cloud`), the
`KeepoutFilter` / `CostmapFilter` (the 2D keepout mask, inflated by the
filter itself), and `MapTypeGate`, which from 2026-10-20 ran the obstacle layer
on a 2D map and the terrain layer on a 3D one in the local costmap — never
both, since they disagree on a ramp (free here, a wall in the band).

### TerrainLayer

The local costmap's only sensor layer, on every map since 2026-10-28 (on a 3D
map only from 2026-10-20, beside the obstacle layer). It replaced the obstacle
layer's height band, which — every return between 0.58 and 1.98 m above the
robot's own ground — could not see a box, a cart's base, a
kerb's wall, a drop-off or a stair tread, and read a ramp as a wall after a
metre, so MPPI had nothing to avoid low. Every
quadruped stack on this kind of hardware (CMU's `terrain_analysis` on the Go2,
SCAN-Planner, RAEM) keeps a rolling 2.5D grid with a ground height per cell and
judges returns against *that* ground; this layer is that grid with
OctomapPlanner3D's rules on top, so the local costmap sees what the global
planner sees. The global side's own memory of what the lidar sees that the
map does not is `Octo3D`'s dynamic overlay (`syncai_global_planner`,
"Dynamic obstacles", 2026-10-21): the same cloud and the same 0.1–0.6 m band
above the surface, in the map frame, raytrace-cleared and forgotten after
30 s, read by the planner's search and its `is_path_valid`. The two are
independent; this layer is the controller's and nothing in it changed for
the overlay. The grid itself, `TerrainGrid` (`terrain_grid.hpp`, ROS-free,
unit-tested on synthetic scenes in `test/test_terrain_grid.cpp`), holds per
cell a ring of the newest returns with their times, in the costmap's frame,
and judges every cell on each update:

1. **Ground** = the 0.25 quantile of the 3×3 neighbourhood's returns younger
   than `ground_memory` (10 s), capped at the lowest + `max_ground_lift`
   (0.15; CMU's `quantileZ` / `maxGroundLift`). Long memory on purpose: the
   MID360's −7° lower FOV never sees the floor within ~3.9 m of the robot, so
   the floor under and ahead of it was seen 5–13 s earlier, from further off.
   A cell with fewer than `min_ground_samples` returns borrows the robot's own
   ground (the lidar frame's z minus `sensor_height`) — except in the obstacle rule below, where a
   cell that has a few returns judges them against the nearest known ground
   within `sparse_ground_radius` (0.4 m) **that the robot can walk onto from
   where it stands** (a second reach flood, from its own cell only), when
   there is one (2026-10-24). Borrowed, the robot's height made the cells the
   lidar has only just begun to see on a slope — a return or two each — a
   lethal band 1–2 m ahead: robot01's dp bag, replayed through the grid at
   dp's arched ramp (0.25 m crown), gave 554 lethal cells on the approach,
   all on borrowed ground, and 1 with the reference. Not a wider ground
   estimate, nor the path's heights: either took a sparse box top or a
   flight's next tread for floor (a box on never-seen floor 78 % → 28 %
   lethal, a flight seen from its foot 70 % → 33 %), while a box or a tread
   is never walked onto and so never the reference; the dp stair's approach,
   replayed the same way, keeps every island, step and slope cell.
   **Overhead** returns — more than
   `body_clearance` + `max_ground_lift` (0.75 m) above the lowest return of
   the cell (on intake) or of the 3×3 neighbourhood (in the estimate) — are
   clearance the robot walks under: above the obstacle band for any ground
   the estimate can return, so neither ground nor obstacle. They are not
   stored at all (2026-10-22), because the ring keeps the newest returns: on
   robot01 the underside of dp's lobby stair, 1.0–1.4 m over the floor and
   inside `max_rel_z`, pushed the floor seen from further off out of the
   rings and became the ground under it — islands and steps round the robot,
   MPPI failing half its cycles for 18 s, the robot standing and twitching.
2. **Obstacle**: a return of the cell itself younger than `obstacle_memory`
   (2 s) between `min_obstacle_height` (0.1) and `body_clearance` (0.6) above
   that ground (for a sparse cell, the reference above) → LETHAL.
   **Ray clearing** (2026-10-23) is what takes a person who walks on out: every return's ray is traced through the cells between
   the lidar and it (`TerrainGrid::clearRay()`, Amanatides–Woo in the plane,
   the ray's height interpolated per cell), and a sample within
   `ray_clear_tolerance` (0.05) of the ray's height in its cell, older than
   the ray, is **removed from the cell** — the ray proves there was nothing at
   that height when it passed. Removed, not only ignored as an obstacle, so
   a departed person's legs leave the ground estimate too (see the ghost
   under "Limits"). A ray proves nothing about the surface it ends on and
   grazes it for its last stretch — a floor return at the −7° edge is within
   0.05 m of the floor for its last 0.41 m — so nothing within
   `ray_clear_end_margin` (0.6) of the end is cleared, which is what keeps
   the floor ahead (seen once, from further off, relied on for 10 s) from
   being eroded as the robot approaches it. Returns outside `max_range` or
   the height band still clear along their rays; a wall at 6 m is the most
   useful ray there is. `obstacle_memory` is the fallback for cells no ray
   crosses: behind an occluder, or a person closer than the margin to the
   wall behind them. Until 2026-10-23 the memory was the only way out: 2 s,
   a costmap cycle and MPPI's start from a stop were the 2–3 s robot01 stood
   after a blocker had left. A tolerance of 0 is that behaviour.
3. **Reach**: cells are flooded from the robot's own over 8-neighbours within
   `max_step_height`; a cell with a ground estimate the flood does not reach
   is an island — a box top, a table, a platform seen without the floor under
   it, which the lower FOV makes the normal view of anything low and near —
   and LETHAL. Octo3D's connected components, locally. The poses of the
   path (`plan_topic`, the planner's `plan`, republished at 1 Hz with Octo3D's
   heights) seed the flood too, so a ramp whose foot the lidar never looked
   at is still the floor the planner says it is; a level path seeds nothing.
4. **Step**: an 8-neighbour whose ground differs by more than
   `max_step_height` (0.17, a riser) → LETHAL: a wall foot, a ledge, a
   drop-off — the negative obstacle the band could never see.
5. **Slope**: the plane fitted under the `robot_radius` disc steeper than
   `max_slope_deg` (20) → LETHAL, judged only when the disc spans more than a
   step (a level floor with a kerb is not fitted) and at least
   `slope_min_known` of it has a ground.
6. Nothing seen → NO_INFORMATION; the rest FREE. Then `updateWithMax`, the
   footprint cleared (its floor is never seen and the robot's legs are in the
   cloud), and the `InflationLayer` inflates it all.

The four geometry values are `SurfaceMapParams`' (Octo3D), one calibration of
the legs; change them together.

**Stairs are lethal, and nothing opens them.** The step and slope rules refuse
a flight, as Octo3D's do, and the global planner crosses one only over a stair
link in its path. The climb is the controllers' stair mode (`syncai_util::
StairMode`, in MPPI and RPP 3D), which does not read the costmap from the hold
at the stair's foot to the hold past its top: **no obstacle avoidance on
stairs**, by decision (2026-10-20). The lethal here is what keeps MPPI's
optimizer off a stair the robot was not routed over. A draft of the same day
opened a corridor along a climbing path (the path's height as the ground, the
obstacle band starting 0.3 m up) so the climb could avoid a box on the stairs;
it was taken out with that decision, and git history has it.

**What a costmap clear does here**: drops the costs, keeps the returns. The
floor the lidar can no longer see is in them, and a stale obstacle mark leaves
with the next ray through it, or after `obstacle_memory` where no ray comes.

**Limits.** A raised surface whose connection to the robot's floor was never
observed is an island even when it is a ramp — at start-up facing a ramp whose
foot is inside the blind 3.9 m and with no path through it, the ramp is lethal
until the robot has seen its foot (a path with heights over it connects it).
Overhead is judged against a floor return younger than `ground_memory`: where
the neighbourhood has none — never seen, or the robot stood under the
overhang longer than that — the underside is taken for a surface and is an
island. An underside between `body_clearance` and 0.75 m up lifts the ground
and reads as an obstacle (conservative, for a few centimetres of clearance).
Unknown cells
are free to MPPI unless the costmap tracks unknown space, so the far side of a
drop the lidar has not looked over is not yet a wall; the drop's edge is.
A person's **ghost**: the floor within 3.9 m is never re-seen while the robot
stands, so a person who blocks it for longer than the floor memory round them
lasts leaves their legs' returns (the lowest ~0.3 m up, the lower FOV edge at
1.5 m) as the cell's only ground evidence — an island 0.45 m up, lethal for
the 10 s of `ground_memory` — unless a ray clears them, which needs a view
through where they stood (open floor or a wall further than the end margin
behind). Ray clearing within a cell is by height, so a ray just over a box top
clears the top returns it passes within the tolerance of; the returns that
end on the box in the same scan are kept, and the cell is an obstacle while
one return a scan lands in its band. Dynamic parameters are not handled: a
change needs a restart. There is no `map_type` / `map_3d_topic` any more
(2026-10-28): the layer runs whatever the map, and a stale entry in a params
file is ignored.

### InflationLayer

Grows a cost gradient outward from every lethal cell so the planner keeps
clearance without a full footprint check.

| Parameter | Default | Notes |
|---|---|---|
| `inflation_radius` | `0.55` | How far the gradient extends |
| `cost_scaling_factor` | `10.0` | Exponential decay rate — **larger means a steeper drop-off**, i.e. less avoidance |
| `inflate_unknown` | `false` | |
| `inflate_around_unknown` | `false` | |

RPP 3D inverts this exact curve to recover a distance-to-obstacle for its cost
regulator, so `cost_scaling_factor` and its `inflation_cost_scaling_factor` in
`syncai_controller`'s params must be kept **the same value**; they are two ends
of one equation (the regulator is off there today,
`use_cost_regulated_linear_velocity_scaling: false`, but the pair still
matches). MPPI's `CostCritic` reads the inflated costs directly. The planner's
global costmap has no inflation layer since 2026-10-28.

## Interfaces

With the costmap node at `/<robot_id>/<costmap_name>`:

| Topic | Type | QoS |
|---|---|---|
| `…/costmap` | `nav_msgs/OccupancyGrid` | transient-local, reliable, depth 1 |
| `…/costmap/raw` | `nav2_msgs/Costmap` | same |
| `…/costmap_updates` | `map_msgs/OccupancyGridUpdate` | same |
| `…/published_footprint` | `geometry_msgs/PolygonStamped` | default |
| `…/footprint` *(in)* | `geometry_msgs/Polygon` | override the footprint at runtime |

The update topic is `<topic>_updates`, not `<topic>/updates` — RViz2's Map display
derives that name itself, and the earlier `/update` spelling left RViz subscribed
to a topic nobody published, so the display simply never refreshed.

| Service | Type |
|---|---|
| `…/get_costmap` | `nav2_msgs/GetCostmap` |
| `…/clear_entirely_<name>` | `nav2_msgs/ClearEntireCostmap` |
| `…/clear_around_<name>` | `nav2_msgs/ClearCostmapAroundRobot` |
| `…/clear_except_<name>` | `nav2_msgs/ClearCostmapExceptRegion` |

The costmap name is part of the service name, so the full path is e.g.
`/robot01/global_costmap/clear_entirely_global_costmap` — which is exactly what
the `ClearEntireCostmap` BT nodes in `syncai_behavior_tree` pass as their
`service_name`.

```bash
ros2 service call /robot01/local_costmap/clear_entirely_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}"
```

Clearing rewrites the grid of every **clearable** layer to the master costmap's
default value — `FREE_SPACE`, or `NO_INFORMATION` when `track_unknown_space` is
set (`TerrainLayer` drops its costs and keeps its returns, see above).
`clear_around_` clears a square of
`reset_distance` centred on the robot; `clear_except_` clears everything outside
that square.

## Key parameters (`CostmapROS` itself)

| Parameter | Default | Notes |
|---|---|---|
| `global_frame` | `map` | `odom` for a rolling local costmap |
| `robot_base_frame` | `base_link` | Both need the `<robot_id>/` prefix from the launch file |
| `update_frequency` | `5.0` | Update thread rate; `0.0` disables the thread entirely |
| `publish_frequency` | `1.0` | `<= 0` disables publishing |
| `width` / `height` | `5` / `5` | **Integers, in metres** — not cells |
| `resolution` | `0.1` | m/cell |
| `origin_x` / `origin_y` | `0.0` | Ignored when rolling |
| `rolling_window` | `false` | Re-centre on the robot every cycle |
| `track_unknown_space` | `false` | |
| `footprint` | `"[]"` | A string of `[[x, y], …]`; **if it parses, it wins over `robot_radius`** |
| `robot_radius` | `0.1` | Circular fallback |
| `footprint_padding` | `0.01` | |
| `transform_tolerance` | `0.3` | |
| `plugins` | `[]` | Empty since 2026-10-28 (was `["static_layer"]`); `filters` is no longer declared |
| `always_send_full_costmap` | `false` | Full grid every publish instead of incremental updates |

`lethal_cost_threshold`, `unknown_cost_value`, `trinary_costmap`,
`use_maximum`, `map_topic` and `observation_sources` are no longer declared
(2026-10-28): they were the static and obstacle layers' parameters. A params
file that still sets one is ignored. `Costmap`'s OccupancyGrid constructor
and `copyWindow()` went too (their users were `CostmapSubscriber` and the
filter pipeline).

An invalid `footprint` string does not fail — it logs an error and silently falls
back to `robot_radius`, which is a much smaller robot. Check the startup log if
paths start passing suspiciously close to walls.

## Tests

`test/test_terrain_grid.cpp` (gtest, `colcon test --packages-select
syncai_costmap`) runs `TerrainGrid` on synthetic scenes — floor, kerb,
step, drop-off, ramp, slope, box, person, wall foot, a stair's underside
overhead (and with no floor seen under it), a stair lethal with and
without a climbing path, the window moving — with no ROS; it also builds on a workstation
with plain `clang++` against `include/` and googletest.

## Removed upstream tools

`CostmapSubscriber`, `FootprintSubscriber` and `CostmapTopicCollisionChecker`
(collision-checking against a costmap received over a topic rather than one
owned — nav2's behavior server's need) were deleted on 2026-10-28: nothing in
the workspace used them, and the checker's `.cpp` was not even compiled (it
was missing from `CMakeLists.txt` and included a header spelled differently
from the file, `costmap_topic_coliision_checker.hpp`). git history has them.

## Gotchas

- **"Missing" warnings are logged once per state change, not repeatedly**
  (2026-10-19): `LayeredCostmap`'s "Robot … is out of bounds of the costmap"
  warns once on entering the state and logs an info on leaving it (the static
  layer's, obstacle layer's and keepout filter's did the same until they went,
  2026-10-28). The planner's layerless global costmap stays at its 5 × 5 m
  default at the map origin, so a robot outside it logs that warning once —
  expected, since nothing reads its cells. So read the log from the top: a
  warning that appeared once still holds until its info says otherwise.

- **Params need `/**/` wildcard keys.** The costmap node's fully-qualified name is
  `/<robot_id>/<costmap_name>`, and the hosting process must declare its `Node`
  with **no `name=`** — a launch-level name remaps both the server and its
  internal costmap to the same name, and the costmap silently loses every
  parameter and runs on defaults.
- **`width`/`height` are integer metres.** Setting `width: 0.5` for a fine local
  costmap gives you 0.
- **`update_frequency: 0.0` disables the update thread** — the costmap is created,
  publishes nothing, and a layered one never becomes current, which reads like
  a TF problem.
- **The footprint lives in the local costmap only** (`[[0.35,0.22],…]`,
  padding 0.01, `controller_server_params.yaml`). Until 2026-10-28 the
  planner's global costmap carried the same rectangle with padding 0.03, so
  its `is_path_valid` perimeter test on 2D paths was stricter than RPP's; that
  test went with 2D navigation. Octo3D's `robot_radius` (0.25) and the terrain
  layer's are the padded rectangle's inscribed radius: change them with it.
- **`inflation_radius` smaller than the circumscribed radius** leaves lethal
  cells MPPI's `CostCritic` cannot see from the centre cell, so a corner clips
  them (the local costmap's 0.45, `controller_server_params.yaml`).
- `package.xml` still carries `TODO: Package description` and
  `TODO: License declaration`, unlike every other package in the workspace.

Upstream reference: [`nav2_costmap_2d`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_costmap_2d)
and the [nav2 costmap configuration guide](https://docs.nav2.org/configuration/packages/configuring-costmaps.html),
which documents the core costmap and inflation parameters in more detail than
is repeated here.
