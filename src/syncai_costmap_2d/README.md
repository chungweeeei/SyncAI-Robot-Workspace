# syncai_costmap_2d

The occupancy representation the planner and controller both plan against: a
layered 2D costmap with pluginlib layers, sensor-driven obstacle marking and
clearing, inflation, and costmap filters. Port of `nav2_costmap_2d`.

It is used as a **library, not a node**. `syncai_planner` and `syncai_controller`
each construct their own `Costmap2DROS` inside their own process:

| Owner | Costmap | Typical config |
|---|---|---|
| `syncai_planner` | `global_costmap` | Full-map, `global_frame: map`, static + obstacle + inflation (+ keepout filter) |
| `syncai_controller` | `local_costmap` | 3×3 m rolling window, `global_frame: odom`, obstacle + inflation |

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
src/footprint.cpp, footprint_collision_checker.cpp, costmap_math.cpp, array_parser.cpp
plugins/static_layer.cpp        ┐
plugins/obstacle_layer.cpp      ├ pluginlib layers
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
`observation_persistence`, `expected_update_rate`, `inf_is_valid`.

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

Enable it in the planner params by adding a `filters:` line; see
`syncai_planner`'s params.

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
- **Footprints must agree across costmaps.** The planner's global costmap and the
  controller's local costmap use the same rectangle; if they diverge, RPP rejects
  paths the planner considers valid.
- **`inflation_radius` smaller than the inscribed radius** leaves lethal cells the
  planner will happily route the robot's corners through.
- `package.xml` still carries `TODO: Package description` and
  `TODO: License declaration`, unlike every other package in the workspace.

Upstream reference: [`nav2_costmap_2d`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_costmap_2d)
and the [nav2 costmap configuration guide](https://docs.nav2.org/configuration/packages/configuring-costmaps.html),
which documents every layer parameter in more detail than is repeated here.
