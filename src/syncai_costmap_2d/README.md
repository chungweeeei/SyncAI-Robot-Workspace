# syncai_costmap_2d

The occupancy representation the planner and controller both plan against: a
layered 2D costmap with pluginlib layers, sensor-driven obstacle marking and
clearing, inflation, and costmap filters. Port of `nav2_costmap_2d`.

It is used as a **library, not a node**. `syncai_global_planner` and `syncai_controller`
each construct their own `Costmap2DROS` inside their own process:

| Owner | Costmap | Typical config |
|---|---|---|
| `syncai_global_planner` | `global_costmap` | Full-map, `global_frame: map`, static + obstacle + inflation (+ keepout filter) |
| `syncai_controller` | `local_costmap` | 4×4 m rolling window (3×3 until MPPI, 2026-10), `global_frame: odom`, obstacle + inflation |

`costmap_2d_node` also exists as a standalone runner, used only by the three test
launch files in this package.

## Layout

```
src/costmap_2d.cpp              the raw grid: cells, world↔map conversion, raytracing
src/layered_costmap.cpp         the layer/filter pipeline
src/layer.cpp, costmap_layer.cpp  Layer base classes (+ joinWithParentNamespace)
src/costmap_2d_ros.cpp          the ROS wrapper node — most of the local complexity
src/costmap_2d_publisher.cpp    OccupancyGrid / Costmap / update publishing
src/clear_costmap_service.cpp   the three clear_* services
src/observation_buffer.cpp      TF-transformed, time-windowed sensor readings
src/terrain_grid.cpp            the ROS-free 2.5D grid behind TerrainLayer (gtest: test/test_terrain_grid.cpp)
src/map_type_gate.cpp           a layer's map_type: obstacle layer on a 2D map, terrain layer on a 3D one
src/footprint.cpp, footprint_collision_checker.cpp, costmap_math.cpp, array_parser.cpp
plugins/static_layer.cpp        ┐
plugins/obstacle_layer.cpp      │
plugins/terrain_layer.cpp       ├ pluginlib layers
plugins/inflation_layer.cpp     ┘
plugins/costmap_filters/{costmap_filter,keepout_filter}.cpp
src/costmap_2d_node.cpp         standalone runner
launch/, params/, rviz/, scripts/fake_scan.py   standalone tests
```

Everything builds into **one** shared library, `libsyncai_costmap_2d.so`, which
is both the linked library and the pluginlib plugin library — `costmap_plugins.xml`
points at the same `.so`.

## Core objects

**`Costmap2D`** — the grid itself: an `unsigned char[]` plus resolution and
origin, with world↔cell conversion, Bresenham raytracing, window copy and a
recursive mutex. No ROS.

**`LayeredCostmap`** — owns the master costmap and two ordered lists: **plugins**
(layers) and **filters**. Its `updateMap(x, y, yaw)` is the whole pipeline:

1. If `rolling_window`, re-centre the origin on the robot.
2. Ask every layer, then every filter, for its dirty region (`updateBounds`),
   accumulating a bounding box. A layer that *shrinks* the box is logged as an
   "Illegal bounds change".
3. Clamp the box to the grid and write costs:
   - **No filters:** `resetMap(window)` on the master, then each layer's
     `updateCosts()` writes into it, in order.
   - **With filters:** layers write into a separate `primary_costmap_`, that
     window is copied into the master, and filters then run on the master. The
     point is that filter output never feeds back into the layers on the next
     cycle — a keepout zone does not become an obstacle the layers can see.

**`Layer` / `CostmapLayer`** — the plugin interface. `Layer` is the bare
`updateBounds`/`updateCosts`/`activate`/`deactivate`/`reset` contract;
`CostmapLayer` adds a private grid plus the merge helpers
(`updateWithMax`, `updateWithOverwrite`, `updateWithTrueOverwrite`, …).

**`Costmap2DROS`** — the ROS wrapper. This is where the local design decisions
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
| `0` | `Overwrite` — replace, except `NO_INFORMATION` | Static layer (the map is ground truth) |
| `1` | `Max` — keep the larger; a known value overrides unknown | Obstacle layer (**default**) |
| `2` | `MaxWithoutUnknownOverwrite` — as Max, but master's unknown stays unknown | Preserving "unknown means unknown" |

## Costmap2DROS: no lifecycle, so what changes

**Three-phase construction.** There is no lifecycle manager in this stack, so
nav2's transitions are exposed as ordinary calls the owner makes by hand:

```cpp
auto costmap = std::make_shared<Costmap2DROS>("local_costmap", "/robot01", "local_costmap");
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

That creates a problem for **input** topics: `map` and `scan` are published in the
parent namespace, not the costmap's sub-namespace. `Layer::joinWithParentNamespace()`
solves it by stripping the last segment of the node namespace before resolving a
relative topic — so `scan` in the params becomes `/robot01/scan`, not
`/robot01/local_costmap/scan`. Absolute topic names are passed through untouched.

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
type); filters by the separate `filters` list.

### StaticLayer

Subscribes to an `OccupancyGrid` and writes it into the costmap, resizing the
master to match the incoming map when not rolling.

| Parameter | Default | Notes |
|---|---|---|
| `map_topic` | `""` | Falls back to the costmap's own `map_topic`; resolved against the parent namespace |
| `map_subscribe_transient_local` | `true` | Must match the latched map server, or the map never arrives |
| `subscribe_to_updates` | `false` | Also subscribe to `<map_topic>_updates` |
| `footprint_clearing_enabled` | `false` | |
| `transform_tolerance` | `0.0` | |

### ObstacleLayer

Marks and clears from sensor data. Each entry in `observation_sources` gets its
own sub-namespace of parameters, its own `ObservationBuffer`, and a
`tf2_ros::MessageFilter` so readings are only processed once their transform is
available.

Per-source parameters: `topic`, `sensor_frame`, `data_type` (`LaserScan` or
`PointCloud2`), `marking`, `clearing`, `min_obstacle_height` /
`max_obstacle_height`, `obstacle_min_range` / `obstacle_max_range` (marking
range), `raytrace_min_range` / `raytrace_max_range` (clearing range),
`observation_persistence`, `expected_update_rate`, `inf_is_valid`, and two that
are not upstream, `height_reference` / `sensor_height` (below).

**`height_reference`** (2026-10): `global` (default, upstream) measures
`min_obstacle_height` / `max_obstacle_height` from the global frame's `z = 0`;
`sensor` measures them from the ground under the sensor — its origin's z in the
global frame minus `sensor_height`. The buffered cloud (and its origin) are
stored with z relative to that ground, so the layer-level
`min_obstacle_height` / `max_obstacle_height` read the same way for that source;
nothing downstream of the buffer reads z in a 2D costmap. It exists because
lio_bridge keeps `odom` planar, so z in `odom` is height in the map, whose origin
is the lidar where mapping started: the 1F floor sits at about -0.48, and on 2F
(~5.5 m up) a fixed band sees nothing. The controller's `pointcloud` source uses
`sensor` with `sensor_height: 0.481` and a band chosen to equal what 1F already
ran with; the global costmap's sources are still `global`.

Two things worth internalising:

- **Marking and clearing use different ranges.** Clearing raytraces from the
  sensor origin out to `raytrace_max_range`; marking only accepts returns within
  `obstacle_max_range`. Clearing range should be the larger of the two, or
  obstacles get marked in a band the layer can never clear again.
- **Leaving `sensor_frame` empty is meaningful,** not a mistake: the observation
  buffer then takes the raytrace origin from the message header's `frame_id`.
  That is how the 3D config consumes `pointlio/body_cloud` without needing a
  per-robot launch-file frame override.

`expected_update_rate` is what drives `isCurrent()`: a source that stops
publishing eventually marks the layer not-current, and both the planner and the
controller wait on `isCurrent()` before using the costmap.

Since 2026-10-20 the **local** costmap runs this layer on a **2D map only**;
on a 3D map `TerrainLayer` (next) runs in its place — see "Which sensor layer
runs". The global costmap keeps it on every map.

### Which sensor layer runs (`map_type`)

The local costmap loads both `obstacle_layer` and `terrain_layer` (2026-10-20,
`feature/octomap-planner-3d`) and exactly one of them runs, picked by the map:
the obstacle layer on a 2D map — `dev`'s local costmap, unchanged — and the
terrain layer on a 3D one. Never both: they disagree on a ramp (free in the
terrain layer, a wall in the band) and the max of the two is the wall. MPPI
(`FollowPath`) drives on either.

Each layer has a `map_type` parameter — `"any"` (the default for the obstacle
layer, so the global costmap and every other use are upstream's), `"2d"` or
`"3d"` — read by `MapTypeGate` (`map_type_gate.hpp`) against `map_3d_topic`,
the planner's latched `planner_server/Octo3D/map_available`. That is the topic
the task_runner picks `move.xml` / `move_3d.xml` by, so the tree and the
costmap switch together, and since `Octo3D/load_map` switches maps without a
restart, the layer switches at the next update too. The map reads 2D until the
first message, as it does for the task_runner. On a switch either layer
empties itself and puts its whole window in the update bounds once, so the
master grid loses what the other layer had marked. A gated-off layer counts as
current (a costmap clear would otherwise leave it not-current, and the
controller waiting on it), and the terrain layer takes no samples while off.
The gate does not touch `enabled`, which stays the operator's.

### TerrainLayer

The local costmap's sensor source on a **3D map** (2026-10-20), in place of
the obstacle layer's height band. The band — every return between 0.58 and
1.98 m above the robot's own ground — could not see a box, a cart's base, a
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
   ground (the lidar frame's z minus `sensor_height`, the obstacle layer's
   `height_reference: sensor`). **Overhead** returns — more than
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
   that ground → LETHAL. Short memory is how a person who walks on is gone
   without raytracing.
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
   footprint cleared as the obstacle layer clears it, and the
   `InflationLayer` inflates it all as before.

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
by itself after `obstacle_memory`.

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
Returns are not raytraced, so a wall seen through a doorway and a person in it
age out at the same rate. Dynamic parameters are not handled: a change needs a
restart.

### InflationLayer

Grows a cost gradient outward from every lethal cell so the planner keeps
clearance without a full footprint check.

| Parameter | Default | Notes |
|---|---|---|
| `inflation_radius` | `0.55` | How far the gradient extends |
| `cost_scaling_factor` | `10.0` | Exponential decay rate — **larger means a steeper drop-off**, i.e. less avoidance |
| `inflate_unknown` | `false` | |
| `inflate_around_unknown` | `false` | |

RPP inverts this exact curve to recover a distance-to-obstacle for its cost
regulator, so `cost_scaling_factor` and `inflation_cost_scaling_factor` in
`syncai_controller`'s params must be kept **the same value**; they are two ends of
one equation.

### KeepoutFilter

A *filter*, not a layer: listed under `filters`, and it runs after the layers on
the combined costmap (see the pipeline above).

It subscribes to a `nav2_msgs/CostmapFilterInfo` on `filter_info_topic`
(transient-local), learns the mask topic from it, then subscribes to that
`OccupancyGrid` mask and writes the mask's occupancy into the costmap. Both
subscriptions are transient-local, matching the latched publishers in
`syncai_map_server`.

Frames are handled two ways: if the mask frame equals the costmap's global frame,
it iterates only the overlap of the mask and the update window; otherwise it
looks up the transform per cell region. `base` and `multiplier` in the filter info
must stay at their defaults (0.0 / 1.0) for keepout semantics — anything else is
logged as an error.

It is enabled on the planner's global costmap (`filters: ["keepout_filter"]` in
`syncai_global_planner`'s `planner_server_params.yaml`, 2026-09) and not on the
controller's local costmap. The mask is `map/<name>/keepout.yaml`, served by
`syncai_map_server`'s `costmap_filter_info.launch.py` from the nav session,
which writes a blank all-unknown mask of the gridmap's geometry when a map has
none yet. Unknown is the right blank: `process()` skips unknown mask cells but
lets a free mask cell overwrite an unknown costmap cell, so an all-*free* mask
would silently turn unexplored space plannable.

**The filter inflates the mask itself** (2026-10). Filters run after the layer
stack, inflation included, so upstream's keepout lands as a bare LETHAL cell:
no INSCRIBED band, no gradient. SmacPlanner2D collision-checks the robot's
centre cell against INSCRIBED, so two zones one free cell apart were a
plannable corridor — `dp2f` had a 0.25 m gap between two zones that the planner
routed a 0.44 m wide robot through. The documented rule used to be "draw the
footprint margin into the mask"; it bakes the robot's size into per-map data,
the editor does not do it, and a hand-drawn band has no gradient to steer the
cost-aware search off the edge. So `inflateMask()` runs on every mask arrival
and on every `onFootprintChanged()` (`LayeredCostmap::setFootprint` calls it
on filters too), never per update cycle:

- Radii come from the costmap the filter is attached to, not from parameters
  of its own: the inscribed radius from `LayeredCostmap::getInscribedRadius()`
  (the padded footprint), `inflation_radius` / `cost_scaling_factor` read off
  the costmap's `InflationLayer` the way `syncai_global_planner`'s
  `findCircumscribedCost()` finds it. A zone therefore carries exactly the band
  and gradient a wall does. No `InflationLayer` → the INSCRIBED disc only, with
  a warning.
- Same curve as `InflationLayer::computeCost()` but in metres, because the
  mask has its own map_server and need not share the costmap's resolution. A
  disc kernel is stamped (max) around each zone's boundary cells only.
- Drawn cells keep upstream semantics (free overwrites unknown). **Cells
  produced by inflation never overwrite an unknown master cell**: a gradient
  is a derived cost, not a rule about the cell, and writing it would turn
  unexplored space into known, plannable cost. `process()` tells the two apart
  by comparing the inflated mask against the raw one.
- `updateBounds()` now grows the window to the mask's own extents once per
  rebuild. Upstream pushed the mask's cell count through the *costmap's*
  `mapToWorld`, which is only right when mask and costmap share origin and
  resolution.

A mask is therefore "the forbidden area", nothing more. A mask that still
carries a hand-drawn margin now gets that margin twice.

The filter/layer distinction exists only at the config level. In
`costmap_plugins.xml` `KeepoutFilter` is registered with
`base_class_type="syncai_costmap_2d::Layer"`, exactly like `StaticLayer` — there
is no separate `CostmapFilter` base class for pluginlib to key on. What makes it
a filter is the `filters:` list it is named in, which decides *when* it runs
(after the layers, on the combined grid) and that `clear*` never touches it.

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
set. Costmap filters are never cleared: a keepout zone is a rule, not an
observation, so it must survive a recovery. `clear_around_` clears a square of
`reset_distance` centred on the robot; `clear_except_` clears everything outside
that square.

## Key parameters (`Costmap2DROS` itself)

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
| `plugins` / `filters` | `["static_layer"]` / `[]` | |
| `map_topic` | `<parent>/map` | Default for the static layer |
| `always_send_full_costmap` | `false` | Full grid every publish instead of incremental updates |
| `lethal_cost_threshold` | `100` | Read by `StaticLayer::interpretValue`: an occupancy value `>=` this becomes `LETHAL_OBSTACLE`. Declared on the costmap node, not the layer, so it is shared by every static layer the node hosts |
| `unknown_cost_value` | `255` (`0xff`) | The occupancy byte the static layer treats as "unknown" — `-1` in `OccupancyGrid` arrives as `255` once cast to `unsigned char`. It becomes `NO_INFORMATION` with `track_unknown_space`, `FREE_SPACE` without |
| `trinary_costmap` | `true` | Static layer collapses everything below the lethal threshold to `FREE_SPACE`. Set `false` to keep a linear ramp (`value / lethal_cost_threshold × 254`) — only useful for maps that already encode a cost gradient, which `syncai_map_server` does not produce |
| `use_maximum` | `false` | Static layer writes its cells with **true overwrite** (the map wins, even over a higher cost already in the master grid). `true` switches to a max-merge so earlier plugins' obstacles survive under the map's free space |
| `observation_sources` | `""` | **Dead at this level.** Carried over from nav2, where the node-level declaration is a legacy; the `ObstacleLayer` declares and reads its own `<layer>.observation_sources`, and nothing reads the node-level one. Set it on the layer |

`observation_sources` aside, the last four all belong to the static layer's map
interpretation, yet are declared on the costmap node rather than under
`static_layer.` — a nav2 inheritance, kept so that copied nav2 params files keep
working. Put them at the costmap level, not under the layer name, or they are
silently ignored.

An invalid `footprint` string does not fail — it logs an error and silently falls
back to `robot_radius`, which is a much smaller robot. Check the startup log if
paths start passing suspiciously close to walls.

## Standalone tests

Three launch files bring the costmap up on its own, without the nav stack:

```bash
# static layer only: map_server + a parked robot TF + the costmap
ros2 launch syncai_costmap_2d static_layer_test.launch.py map_yaml:=map/testmap.yaml

# static + obstacle + inflation, optionally with a synthetic scan and RViz
ros2 launch syncai_costmap_2d three_layer_test.launch.py use_fake_scan:=true rviz:=true

# static layer + keepout filter (mask map server + filter info server)
ros2 launch syncai_costmap_2d keepout_filter_test.launch.py \
    map_yaml:=map/testmap.yaml mask_yaml:=map/keepout_mask_test.yaml
```

`test/test_terrain_grid.cpp` (gtest, `colcon test --packages-select
syncai_costmap_2d`) runs `TerrainGrid` on synthetic scenes — floor, kerb,
step, drop-off, ramp, slope, box, person, wall foot, a stair's underside
overhead (and with no floor seen under it), a stair lethal with and
without a climbing path, the window moving — with no ROS; it also builds on a workstation
with plain `clang++` against `include/` and googletest.

`scripts/fake_scan.py` publishes a 50-beam forward scan at 2 m. Its timestamps
are deliberately current: `tf2_ros::MessageFilter` drops messages older than the
TF cache, so a zero-stamped scan from `ros2 topic pub` never reaches the obstacle
layer — worth remembering when hand-testing.

The default `map_yaml` in `static_layer_test` and `three_layer_test` is an
absolute host path (`/home/syncrobotic/Documents/…/map/testmap.yaml`); pass
`map_yaml:=` explicitly when running elsewhere. `keepout_filter_test` already uses
workspace-relative paths.

## Not wired into the stack

`CostmapSubscriber`, `FootprintSubscriber` and `CostmapTopicCollisionChecker` are
ported and built, but nothing in this workspace uses them. They exist for a
consumer that wants to collision-check against a costmap it receives over a
topic rather than one it owns — nav2's behavior server does this. The header is
misspelled `costmap_topic_coliision_checker.hpp` (the `.cpp` is spelled
correctly); renaming it means touching its two includes.

## Gotchas

- **Four "missing" warnings are logged once per state change, not
  repeatedly** (2026-10-19): `StaticLayer`'s "no map received",
  `LayeredCostmap`'s "Robot … is out of bounds of the costmap",
  `ObstacleLayer`'s "Sensor origin … is out of map bounds" and
  `KeepoutFilter`'s "Filter mask was not received" each warn once on
  entering the state and log an info on leaving it. Until then they
  repeated every 1–10 s (the static layer's counter was a function-local
  `static`, shared by every `StaticLayer` in the process), and on a 3D-only
  map — no `gridmap.yaml`, so the global costmap never gets a map and stays
  at its 5 × 5 m default for good — they were most of the planner log. So
  read the log from the top: a warning that appeared once still holds until
  its info says otherwise.

- **Params need `/**/` wildcard keys.** The costmap node's fully-qualified name is
  `/<robot_id>/<costmap_name>`, and the hosting process must declare its `Node`
  with **no `name=`** — a launch-level name remaps both the server and its
  internal costmap to the same name, and the costmap silently loses every
  parameter and runs on defaults.
- **`width`/`height` are integer metres.** Setting `width: 0.5` for a fine local
  costmap gives you 0.
- **QoS must match latched publishers.** `map_subscribe_transient_local` and the
  keepout filter's transient-local subscriptions have to match `syncai_map_server`,
  or the topic connects and never delivers.
- **`update_frequency: 0.0` disables the update thread** — the costmap is created,
  publishes nothing, and never becomes current, which reads like a TF problem.
- **Footprints must agree across costmaps; paddings deliberately do not.** The
  planner's global costmap (`syncai_global_planner/params/planner_server_params.yaml`)
  and the controller's local costmap
  (`syncai_controller/params/controller_server_params.yaml`) both carry
  `[[0.35,0.22],…]` (reconciled 2026-10; the local one had been left at
  `[[0.28,0.20],…]`). `footprint_padding` is 0.03 global vs 0.01 local: the
  planner's `is_path_valid` walks the padded perimeter against LETHAL exactly
  as RPP does (2026-10), and the padding gap is what keeps the planner's copy
  of that test the stricter of the two, so a path it passes is one RPP will
  drive. (The centre-cell test the planner used alone before then never
  ordered against RPP's perimeter test at all, whatever the padding.) Change
  the rectangle in both files or neither.
- **`inflation_radius` smaller than the inscribed radius** leaves lethal cells the
  planner will happily route the robot's corners through.
- `package.xml` still carries `TODO: Package description` and
  `TODO: License declaration`, unlike every other package in the workspace.

Upstream reference: [`nav2_costmap_2d`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_costmap_2d)
and the [nav2 costmap configuration guide](https://docs.nav2.org/configuration/packages/configuring-costmaps.html),
which documents every layer parameter in more detail than is repeated here.
