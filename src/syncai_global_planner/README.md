# syncai_global_planner

The global planning half of the nav stack: an action server that hosts
`GlobalPlanner` plugins, with an internal **global costmap** node. Port of
`nav2_planner`.

```
syncai_task_runner ──ComputePathToPose (nav2_msgs)──►  planner_server
                                                            │  global_costmap (internal node, no layers:
                                                            │  frame, TF buffer, robot pose, callback group)
                                                            ▼
                                                    OctomapPlanner3D (Octo3D)
                                                    on map/<name>/octomap.bt
                                                            │
                                                            ▼
                                     nav_msgs/Path ──► syncai_controller (FollowPath)
```

**One planner is loaded: `OctomapPlanner3D` as `Octo3D`**, the planner
`move.xml` and `dock.xml`'s staging leg ask for. Both trees check their path
once a second with `is_path_valid`, which the server answers by asking the
planner that made the path; `Octo3D` judges it on its own dynamic-obstacle
overlay (2026-10-21, "Dynamic obstacles" below).

**2D navigation was removed on 2026-10-28.** Until then the package also
carried ports of `nav2_navfn_planner` (`NavfnPlanner`, built, not loaded),
`StraightLinePlanner` (a reference plugin) and `nav2_smac_planner`'s 2D
plugin (`SmacPlanner2D`, loaded as `GridBased`, what a 2D map's `move.xml`
planned with on the gridmap); the task_runner ran the 2D trees or the
`*_3d.xml` ones per goal by `Octo3D`'s `map_available`; and the global
costmap ran static → obstacle → inflation plus the keepout filter, on which
`GridBased` planned and `is_path_valid` judged 2D paths. All of it went
together, as did the nav session's `map_server` window that fed the static
layer and the mask. git history has the 2D planners and their tuning notes.

Two structural notes carry over from the rest of the stack:

1. **`PlannerServer` is a plain `rclcpp::Node`.** Setup is split between the
   constructor (declare parameters, construct the costmap object) and
   `configure()`, which `main.cpp` calls once the node is owned by a
   `shared_ptr`. nav2's `on_configure` **and** `on_activate` are merged there.
2. **The planner plugins and the costmap layers are separate plugin systems.**
   Both load through pluginlib against `syncai_nav_core` / `syncai_costmap`
   respectively, and both are configured from the same params file under two
   different node keys. The global costmap loads no layer today
   (`CostmapROS`'s default plugin list is empty since 2026-10-28, and the
   params leave `plugins` out, since an empty YAML list cannot be typed).

## Layout

```
src/planner_server.cpp   the two action servers + the plan cycle
src/main.cpp             construct → configure() → spin
plugins/
  octomap_planner_3d/              OctomapPlanner3D (planner id Octo3D): the
                                   ROS-free core (surface_map, surface_astar,
                                   path_postprocess, stair_links,
                                   dynamic_overlay, keepout_zones), the
                                   plugin, the offline CLI, and
                                   standalone/CMakeLists.txt to build the CLI
                                   without ROS
global_planner_plugin.xml          pluginlib manifest, exported to syncai_nav_core
params/planner_server_params.yaml
launch/planner_server.launch.py
test/test_surface_map.cpp          OctomapPlanner3D's geometry rules on synthetic scenes
```

Everything builds into one library, `libsyncai_global_planner.so`, which is both
linked by the `planner_server` executable and `dlopen`ed by pluginlib as the
plugin library.

**The package was `syncai_planner` until 2026-10.** It was renamed when
`syncai_mppi_controller` arrived, so the two halves of the stack read as global
planner / controller, and the rename stopped at the package: the node is still
`planner_server`, the executable, the launch file, the
`compute_path_to_pose` action, `is_path_valid`,
`plan`, `global_costmap` and the `planner` log directory all kept their names,
so `move.xml`, rviz and the backend saw nothing change. What did change is
every spelling of the package: the C++ namespace, the include directory, the
library, and the pluginlib lookup names (`syncai_global_planner/OctomapPlanner3D`).
A workspace built before the rename keeps `build/syncai_planner` and
`install/syncai_planner`; delete both. Left in place, the old `install/` stays
on `AMENT_PREFIX_PATH` and keeps registering `syncai_planner/*`, so a params
file still naming the old plugin loads the stale `.so` instead of failing.

## The plan cycle

`computePlan()` runs on the `SimpleActionServer`'s own spin thread
(`spin_thread=true`), so its blocking steps never starve the node's executor —
which in this process handles parameter callbacks, `is_path_valid`,
`clear_plan` and the plan republish timer. The costmap is spun by its own
`NodeThread`; `main.cpp` explicitly warns not to add it to this executor as
well.

```
server inactive / cancel requested?   → bail
accept a preempting goal if pending
start = goal.use_start ? goal.start : costmap.getRobotPose()
transform start and goal into the costmap global frame
path = getPlan(start, goal, goal.planner_id)
path empty?                           → terminate_current()
publish plan, fill planning_time, succeeded_current()
```

There is no wait for the costmap to be current (upstream's
`waitForCostmap()`): no loaded planner reads its cells, and it has no layers
to be current with. Until 2026-10-28 a goal for a planner that read the cells
(`GridBased`; `GlobalPlanner::usesCostmap()`, removed from `syncai_nav_core`
with it) waited there, ending on a cancel, on a static layer that had never
received a map (a 3D-only map: fail at once) or after `costmap_wait_timeout`;
before 2026-10-19 that wait had no way out, and a `GridBased` goal on a
3D-only map hung the action thread for good.

Any exception from a plugin is caught and turned into `terminate_current()` with
a warning — a planner failure fails the goal, never the process.

`expected_planner_frequency` only controls a warning: if a cycle takes longer
than its reciprocal, the server logs "Planner loop missed its desired rate". It
does not throttle or abort anything.

**`getPlan()` selects the plugin by `planner_id`.** An unknown id logs an error
and returns an empty path; an *empty* id with exactly one plugin loaded — the
case today — picks that one and warns once. The BT names `Octo3D` explicitly
in `move.xml` / `dock.xml`.

**No `compute_path_through_poses`** since 2026-10-28: nav2's second action
(plan `start → goals[0] → goals[1] → …` as concatenated segments) was ported
but never called — the trees only use `ComputePathToPose`, and the backend
sequences multi-stop tasks as separate `MOVE` steps. git history has it.

## Planner plugins

### OctomapPlanner3D (`syncai_global_planner/OctomapPlanner3D`) — planner id `Octo3D`, prototype (2026-10)

The quadruped's ground-constrained 3D planner, on `map/<name>/octomap.bt` —
the `octomap::OcTree` that `syncai_mapping`'s `clean_map --octomap 1` writes
beside `map.pcd` (0.1 m; free **and** occupied, so never-seen space is
unknown; hit floor-band voxels kept occupied). It is the only planner loaded
(since 2026-10-28; until then it ran beside `GridBased`, and a 3D map's
`move_3d.xml` / `dock_3d.xml` asked for it), so every navigation goal plans
here, with `controller_id="FollowPath"` — MPPI, which follows a path with
heights in 3D and climbs stairs since 2026-10-20; `FollowPathRPP3D` is the
A/B — `syncai_controller/README.md`. A plan sent while the map is still
being built fails `map_loading`, and one with no map `map_not_loaded`. From
2026-10-19 to 2026-10-28 it latched `planner_server/Octo3D/map_available`
(`std_msgs/Bool`), by which the task_runner picked its trees per goal and the
local costmap its sensor layer; both went with 2D navigation, and the topic
with them.

The global costmap gives this planner its frame id, TF buffer and callback
group and nothing else; it has no layers, so it never "has a map" and stays
at its 5 × 5 m default grid, which is meaningless and harmless. Until
2026-10-28 it carried the 2D layers too, and on a 3D-only map (no
`gridmap.yaml`, `map_server` and the `keepout` pane idle) its static layer
never got a map and logged four warnings about that — no static map, robot
out of bounds, sensor origin out of bounds, no keepout mask — once each
(2026-10-19; until then every 1–10 s).

**Why a surface map and not a free-space search.** The first attempt (the
device workspace's `octo_planner_3d`) ran a 26-neighbour A* over the free
voxels with "something occupied one cell below" as ground support and a
blacklist of wall-foot cells to keep it from climbing. That is a drone
planner with patches: a free cell beside a wall is "supported" at any height,
the body is a 0.3 m half-sphere, and nothing asks whether a step is one the
legs can take. Here the tree is reduced, once at startup, to a **multi-layer
2.5D surface map** — every xy column's floor surfaces, each judged with the
robot's geometry — and the search only ever moves between cells that passed.
So a path cannot leave the ground, climb a wall, cross a stair riser, a kerb
or a drop-off, or jump between floors; it takes ramps, which are runs of
one-cell steps on a plane under the slope limit. The one way between floors
is a stair the robot walked while mapping (see **Stairs** below).

**The build** (`surface_map.cpp`; ~1 s and 355 MB peak for dp1f_1012_v2's
7.3 M nodes, ~30 MB kept once the tree is freed; in a background thread so
`planner_server::configure()` and the other ids are not held up — plans
arriving meanwhile fail with `map_loading`):

1. One pass over the leaves into per-column `known` / `occupied` bitmasks,
   pruned blocks expanded by key range (never `search()` in a loop).
2. A **surface** is an occupied voxel with a non-occupied voxel above and a
   `body_clearance` (0.6 m) band of non-occupied voxels on top. Unknown in
   the band is tolerated (the lidar looks outward from 0.48 m, so the band
   is observed only where the robot passed), but the voxel right above the
   surface must have been observed free (`min_known_free_above: 1`): a floor
   the robot stood on always has, a ceiling seen from below never has — that
   one rule took dp1f_1012_v2 from 748 "components" to 153.
3. **Hole fill.** The MID360's lower field of view is −7°, so the floor
   nearer than ~4 m is never seen directly and the floor under the robot's
   own track is hit only from afar: ~5 % of those columns are empty. A column
   with no surface whose neighbours within 0.3 m agree on a height, from at
   least three of the four quadrants around it and with ≥ 50 % support, and
   with nothing occupied near that height, gets an *inferred* surface. The
   quadrant rule is what keeps a balcony edge from being extended into the
   void (one-sided support); each height group of the neighbours is judged
   on its own, because at a two-storey column the complete upper floor is
   the largest group and would otherwise hide the missing lower one.
4. **Step rule**: every 8-neighbour column that holds surfaces must hold a
   usable one within `max_step_height` (0.17 m, the legs' step capability
   and a standard riser; rounded to two cells at 0.1 m, so a 0.15 m kerb is
   one step and a 0.5 m ledge is not). A neighbour column with no surface at
   all is a *gap* (never observed), left to the footprint rule. A step the
   rule allows is still a step: see **Kerbs** for what the search makes of
   it.
5. **Footprint**: of the columns within `robot_radius` (0.25 m, the 2D
   costmap's inscribed radius), at least `footprint_min_support` (80 %) must
   hold a usable surface inside the slope cone (tolerance grows with
   distance, or a 15° ramp rejects its own neighbours); a column whose
   surfaces are all out of reach or unusable — a wall foot, a drop, an
   obstacle top — blocks the cell outright, a column with no surface is a
   gap, counted (`gap_cnt`) and paid for in cost. If the surfaces span more
   than one step, the plane fitted through them must tilt ≤ `max_slope_deg`
   (20) with residuals ≤ one cell: that is what refuses a flight (two risers
   in a 0.5 m disc, ~30°) while a kerb, one step, is level ground. A fit,
   not max−min: a flat floor read cell by cell looks 11° tilted across a
   0.5 m disc. This is the implicit inflation of walls, drop-offs and
   unfillable holes; the gap share exists because the lidar sees the floor
   at grazing angles only and the observed floor is sparse — with every
   column required, 13 % of dp1f_1012's own keyframes stood on
   "untraversable" ground.
6. 8-connected components (no corner cutting), so a disconnected goal fails
   before the search and a goal snaps to the robot's floor, not the one below
   a balcony edge.
7. A clearance distance field propagated over the surface graph (not the
   2D grid, or a ground-floor wall would cost the floor above it) out to
   `clearance_cost_radius` (0.55 from the traversable boundary = 0.8 from
   the wall, the 2D global costmap's `inflation_radius` until 2026-10-28).

Every rejected surface keeps its reason (`clearance_occupied`,
`no_free_above`, `step`, `footprint_blocked`, `slope`, `residual`), which is
what the CLI dumps and a failed snap reports.

**Search** (`surface_astar.cpp`): A* over the traversable cells, 8 xy
neighbours each resolved to the layer within one step (unique, since two
layers of a column are never that close), diagonals only when both
orthogonal neighbours pass. Edge cost = 3D move length ×
`(1 + clearance_cost_weight · exp(−clearance_cost_scaling · d))` inside the
band (a costmap's inflation curve and the removed SmacPlanner2D's
`cost_travel_multiplier` in one; a multiplier, because a per-node penalty makes diagonals cheaper per
metre and bends paths 45°), × 1.2 on inferred cells, × (1 + `gap_cost_weight`
· gaps / footprint columns) on cells with gaps under the disc. The 3D Euclidean
distance is admissible, so `heuristic_weight: 1.0` is optimal; a 25 m plan on
dp1f_1012_v2 takes 11 k iterations and 1 ms, a 46 m plan upstairs 22 k and
3 ms.

**Snap.** The server's start pose is the planar `map → base_link`, z = 0
always, so the robot's height is read from TF to `body_frame`
(`<robot_id>/pointlio_body`, the LIO chain that keeps z) minus
`lidar_height`; without the TF the request's z is used (right on the ground
floor, wrong upstairs). The goal's z is the request's (`goal_z_mode:
request`; a 2D backend's 0 lands on the ground floor, whose surface is
~0.5 m *below* the map origin — the first lidar pose) or the start's
(`start`). Both snap to the nearest traversable cell within `snap_xy_radius`
whose top face is within `snap_z_window` (1.0 m, far below the 5.7 m floor
spacing) of the hint; the goal only within the start's component. When
nothing is within the window and no column within `snap_xy_radius` holds a
second traversable layer, the one layer there is taken at any height
(`snap_single_layer`, 2026-10-27, logged as a warning): on outdoor_1027 the
ground falls 1.4 m from the map origin to the far end, so a z = 0 goal there
failed `goal_unavailable` with `traversable` printed in the reason. Where
floors overlap the hint still has to pick one. A failed
snap says what *is* there — `goal_unavailable: … surfaces in that column:
z=[-0.40 (untraversable: footprint_blocked), 4.80 (traversable, component
2)]` — or that the nearest surface is on another floor.

**Stairs** (`stair_links.cpp`, 2026-10). The geometry rules refuse every
stair on purpose — a riser is more than one step, a flight is steeper than
20° — and relaxing them map-wide would pass every 0.2 m kerb and drop-off
too. So stairs come from the map's own track instead: `poses.txt` (next to
`octomap.bt`, or `poses_file`) is where the robot stood while mapping, and a
stretch of it that leaves one large component (≥ `stair_anchor_min_area`,
10 m²) and arrives in another at least `stair_min_rise` (0.3 m) higher or
lower, within `stair_max_length` (60 m) of track, is a stair the legs have
taken. The track says *where* the stair is; since 2026-10-19 it no longer
says how to climb it. A single flight (≤ `stair_straight_max_flight`, 8 m of
track) becomes a **row of straight stair links** across the stair's width:
the surfaces beside the walked stretch (within `stair_straight_fit_radius`,
1 m) whose height lies between the two floors — the risers and treads the
rules rejected — are fitted with a plane, its gradient is the uphill
direction, and from every lower-floor cell within `stair_straight_half_width`
(2 m) of the walked foot a line is marched uphill along it, column by column
over surfaces within one step of each other, until it steps onto a cell of
the upper floor; each line that gets there is a link (two points, z as one
ramp). The search then picks the entry, so a route meets the stair where its
own line crosses it, climbs facing the risers and leaves toward the goal —
and the line-of-sight shortcut may span a straight link (`stairCrossing` in
`path_postprocess.cpp`): the vertex before it and the vertex after are
joined by one segment when that segment lies within 15° of the flight's
uphill direction and its columns are lower floor, then step-continuous
surfaces, then upper floor (the clearance rule relaxed only within 0.3 m of
the two edges, which are only as clear as the flight they border). The
segment's floor poses take their height from the cell that walk took in
each column, and the flight's from one ramp between the edges. Until
2026-10-22 they took the surface nearest the height at the flight's edge,
within 0.3 m, which past dp's 2F step (the floor rises 0.3 m in 4 m) found a
rejected layer under the floor in one column — a pose 0.6 m low, which MPPI
read as a stair going down and walked the rest of the route backwards. Without
it a link's cell-centre ends quantise its direction to ~11° steps over
0.5 m, and the path still bent 15° at the stair head. On dp (2F, the 2.6 m
wide flight between z 4.5 and 5.1) this replaced one walked link, 9° off
the stair's axis, with 24 straight ones, and a 6.4 m route across it that
turned 27° at the stair head became one straight line (heading constant to
0.1°). A line
may run level for `stair_straight_exit_reach` (0.6 m) — the blocked floor
the footprint rule leaves before the first riser and after the last — and a
landing longer than that ends the flight. A stretch that is not one straight
flight (longer than 8 m — a switchback stairwell; a plane fit too thin or
more than 60° off the walk; no line reaching the upper floor) keeps the
**walked link**: the keyframes' ground points with every loop cut out (turns
and waits on a landing) and smoothed (`stair_smooth_passes`; a hand-driven
track weaves, and the ground estimate inherits the body's pitch on a
flight), and a startup note says which and why (dp's 1F → 2F stairwell, 30 m
of track, is one). `stair_straight: false` is the 2026-10-14 behaviour.
What could not be linked is logged at startup: a climb on the track longer
than `stair_max_length`, and large components at different heights no link
joins — the floors this map cannot get between. The search may take a link in either direction at its 3D
length × `stair_cost_multiplier` (2.0; ≥ 1 keeps the heuristic admissible),
components joined by links count as one for the snap and the
disconnected-check, and the path follows a walked link's points through the
flight; the line-of-sight shortcut is applied to the floor stretches and,
since 2026-10-19, across straight links.
Only large components end a link because a stairwell's own floor rarely
survives the rules: on dp1f_1012 the three landings of the switchback stair
came out as a few square metres of islands or not at all (small, walled, the
air above them never observed), so the link runs floor to floor over the
whole well. A kerb walked over (< 0.3 m) makes no link. `stair_links: false`
restores separate floors.

**Stairs nobody walked** are detected off the surface map since 2026-10-25
(`stair_detect`), because a map's track holds only the stairs the operator
happened to take while mapping: on dp the U stair in the middle of the
building (1F → 2F, the track passed beside and under it) was refused, and a
route from its foot to its head went 55 m round by the stairwell the track did
climb. The surfaces along each large component's edge within
`stair_detect_floor_band` (0.25 m) of the floor cell they are reached from
belong to that floor (the footprint rule's rim, a stair's first tread); the
rest of the rejected surfaces, bar `clearance_occupied`, and the small
components' cells are cut by the step rule into blobs, and a blob touching two
floors at least `stair_min_rise` apart is a candidate. The shortest way
through it from the lower floor's edge to the upper one's, each move costing
up to 21 times its length within 0.5 m of the blob's edge (a column holding
surfaces but none within 0.35 m — a wall, a drop), stands in for the walk;
without that weight it cut the U landing's inside corner round the newel in
under 0.5 m of level walk, and the stair was read as one 5.5 m flight. The
stair is then laid from the stand-in exactly as a walked one is — straight
links for one straight flight, flights and landing corners otherwise, the
geometry read off the map — and a blob neither reads, or one narrower than
`stair_detect_min_width` (0.6 m), stays unlinked with a startup note. A blob a
track link already crosses is skipped. Detected links are logged as `stair
link detected at (x, y)`, and they work without a `poses.txt` at all. On dp:
the middle stair (2 flights 1.6 / 1.7 m wide, a U landing, 0.29 m to spare at
its turns) and a straight stair with a mid-landing at (12, 25); the route from
the middle stair's foot to its head is 17 m. The stand-in may cross a straight
flight anywhere more than 0.5 m in from its edges, and entries are tried
within `stair_straight_half_width` of it, so a flight wider than ~4 m is used
over part of its width.

**Kerbs** (2026-10-29). A step within `max_step_height` is one piece of
ground, which is right -- the legs take a kerb -- and was also how the G23
fell twice at the same kerb on outdoor_1027 (sim, 2026-10-28): the shortest
line to the goal cut the pavement's corner and ran along the kerb's edge at
~15°, a pair of feet on each side, in the flat gait. Two things change
that. Every traversable cell next to a step is a *step edge*, and a second
distance field (`step_d`, out to `step_edge_cost_radius`, 0.4 m: the
footprint and a foot's reach beyond it) puts a cost band round it like the
clearance band (`step_edge_cost_weight`, the same curve), so running along
a kerb costs more than keeping off it while crossing one costs almost
nothing. At 0.1 m voxels a one-cell step is as often a slope's contour line
as a kerb -- a 20° slope rises 0.036 m per column, so it can never change
two cells between neighbours, while a 12 cm kerb shows as one cell along
most of its length and two where the road is lowest -- so a one-cell edge
gets `step_edge_one_cell` of the weight, and the heights the edges are read
from are the median over each cell's movable neighbours (read raw, the
floor's scatter made 38 000 step cells of outdoor_1027's road; the median
leaves 17 000, the lines). The cells on the low side of a step are then
strung into lines (neighbours whose step faces within ~45° of the same way,
so a contour line that runs into a kerb is a line of its own), and a line
at least `kerb_min_length` (1 m) long with `kerb_min_sure_cells` (3) cells
of a two-cell step and level ground four cells past the step (a stair's
first tread, an island joined to the floor by the step rule, has the next
riser 0.3 m on) is a **kerb**, its cells sure steps in the band whatever
their own voxels say. Each kerb then gets **kerb links** (`kerb_links`,
`stair_links.cpp`): at every cell of it the line's local direction (a fit
over its cells within `kerb_fit_radius`) gives the normal, along which a
straight crossing `kerb_lead` (0.5 m, half the body's length) either side
is marched over traversable ground that stays within a cell of the low
side's height before the kerb and of the high side's after it; each that
gets across is a link, four points (the lead-in, the kerb's two sides, the
lead-out), taken at `kerb_cost_multiplier` (1.5) × its length. Where a link
crosses, the plain step over the kerb is not a move (`kerbCrossing`,
whichever way) and the line-of-sight shortcut refuses to span it, so the
route meets a kerb square, holds the heading for the lead either side and
cannot be straightened back into the diagonal; where none could be laid (a
wall right behind, a second step within the lead) the kerb is crossed as
before, with a startup note. Both sides of a kerb are one component, so the
links join nothing, and a kerb's rise is under the controllers' stair
mode's `min_rise`, so the gait does not change for it. The kerb scene
(`--synthetic kerb`, a 0.15 m step): 74 links, a route from (2, 2) to
(10, 6) crosses at y = 4.25 from x 5.55 to 6.45 in one straight line. What
this cannot do: outdoor_1027's own kerb is one cell along its whole length
in the 0.1 m octomap, so it is a step edge with the one-cell band but not a
kerb, and no link crosses it; telling a 12 cm kerb from a slope's contour
line needs the true floor height, which `map.pcd` holds and the octomap
does not (see the gotchas).

A stretch that is not one straight flight is laid as **flights and landing
corners** since 2026-10-21 (`stair_flights`), because the legs climb a
flight straight and turn on the landing, and a path copied from the walk
curves through both. The walk's height profile (loops cut, raw) is split
into level runs of at least `stair_flights_landing_min_len` (0.5 m; the
first and last are the floors) and the rises between them. For each flight:
its direction is the walk's over the flight, refined to the line through
the points where the cross-sections square to it end on the straighter
side — one side of a flight is the well's wall and its hits line up to a
few centimetres, the other is the open well or the rail and scatters (the
plane fit a straight flight uses was 17° off on dp's second flight, pulled
by the parallel flight inside its radius); the flights of one well are then
held to one orientation, the weighted circular mean of theirs folded to a
quarter turn (dp's four ran 5–13° off the map's y axis and settle on 5.7°).
Its axis is the walk moved by the median centring shift measured square to
that direction; its foot and head are where a march along the axis, down
and up from the middle, finds the surface level for 0.45 m (a tread is
0.3) at about the landings' heights. Consecutive flights are joined with
corners on the landing: ~90° apart, one corner at the axes' crossing (an
L); ~180°, two corners — out onto the landing, across square to the flights,
and in to the next foot (a U); any other angle,
or a flight whose well cannot be read, falls back to the walked link with a
note. A `stair_flights_lead` (0.5 m) straight run before the first foot and
past the last head lines the robot up, re-anchoring the link's ends to the
floor cells nearest those points, else to the first floor cell straight on
along the flight's axis within 2 m (2026-10-24), else keeping the walk's floor
points, lightly smoothed. The straight-on search is for the floor just past a
stair head, which the lidar cannot see from the flight below it nor from on it
(dp's 2F: 1.3 m of `no_free_above`): the walk used to stand in there, and on dp
the path swung 35° off the axis and 56° back right where the robot stops to
switch the policy back; it now runs straight on to the floor, and the nearest
wall along it went from 0.32 m to 0.50 m.
Nothing on the stair itself is smoothed, and `StairLink::turns` lists the
corner vertices for the controllers' stair mode. On dp: 4 flights 1.6 / 1.5
/ 1.4 / 1.3 m wide, three U landings, 6 turns; the
walked route ran 0.08–0.29 m off the axes, the new one under 0.01 m. Two of
dp's three landings are not in the surface map at all (seen only edge-on
from the flights), which is why the corners of a U are not placed against the
surface map.

**Where a U's corners go** (2026-10-24). Both corners lie the same distance
out from the later of the two stair edges, so the leg between them is square
to the flights, and that distance (0.5–1.5 m) is the one that leaves the most
room round the two in-place turns (`stair_flights_turn_radius`, 0.455 m, the
padded footprint's circumscribed radius) and `robot_radius` either side of the
leg — the nearest such when there is room to spare. What blocks is an
occupied voxel in the body's band over the landing (above a step, below
`body_clearance`; a handrail at ~0.9 m is above it), read off the **octree**,
which the plugin and the CLI now keep until the links are built: by surfaces
alone a wall that runs into the slab above and a landing nobody saw the floor
of look the same, and dp's first landing holds faces 0.3 m below and 0.1 m
above its floor side by side. The landing's height is the one most columns of
its near metre hold a face at, not the walk's (0.2 m low there). Until then
the corners came from the walk (0.5–1 m out, each flight's own head / foot),
capped at half a depth measured along one ray: on dp's second landing the ray
went through a gap in the scanned far wall (2.3 m read for 1.05 m), and the two
flights' edges 0.36 m apart slanted the leg 0.22 m past the newel (0.30 m
now). dp's landings are 0.95–1.05 m deep, under the turn's 0.91 m plus the
hind legs' half body, so all three corners stay 0.5 m out and come out
0.02–0.10 m short; a startup note names each such landing.

A walked link that stays walked is **centred in the stairwell** since 2026-10-20
(`stair_centre`), because the operator walks a stair where they like —
along the inner rail, across a landing's inside corner — and the path copied
that line. At each point of the link the cross-section square to the walk is
read off the surface map: column by column to either side, as long as the
column holds a surface within `stair_centre_dz` (0.25 m) of the point's
tread, in any state (on dp the treads and landings are rejected by the
clearance rules as well as the step and slope ones, so the state says
nothing about where the well ends); a wall or the drop to the neighbouring
flight ends a side. Where both sides end within `stair_centre_max_half_width`
(1.5 m) the point moves to the middle, at most `stair_centre_max_shift`
(0.6 m), and the shifts are median-filtered over ±3 points and smoothed,
because one cross-section is noisy (the walk crosses the risers at an angle);
a side that runs out is open floor and the point stays, so the link's ends
are untouched. On dp, 58 of the stairwell link's 66 points moved, mean
0.09 m, at most 0.30 m (the second flight, walked along its inner rail; its
two sides now measure 0.7–0.8 m each). Straight links are not touched — the
search already picks their entry across the flight's width.

**Output** (`path_postprocess.cpp`): a line-of-sight shortcut checked with a
supercover traversal (every cell the segment touches; Bresenham skips the
corner cell a wall foot sits in), step-continuous along the segment, and
never closer to the boundary than the cells it replaces (the rule Smac's
smoother had as "never above 252"); then a resample at `interpolation_resolution` with z read back
from the surface. Poses carry z = the floor's top face (+ `path_z_offset`),
yaw toward the next pose, the goal's orientation last. MPPI
(`FollowPath`), RPP 3D (`FollowPathRPP3D`) and the controller_server's goal
check read z since 2026-10 — a path whose poses do not all share one z is matched to the robot in 3D
(a stair link's spiral is back over the same x / y every 8.1 m on
dp1f_1012), and RPP 3D climbs a stretch rising 0.3 m within 1 m in a stair mode
that switches the gait policy (`syncai_controller/README.md`, "Paths with
heights" and "Stairs").

**Measured on dp1f_1012_v2** (2026-10-13; 1F and 2F, 2 576 keyframes): 1.26 M
columns, 724 k surface cells (24 k inferred), 158 k traversable in 146
components, the two floors being 118 k and 33 k. 86 % of the keyframes stand
on a traversable cell, 90 % within one cell of one, 99.5 % on *a* surface.
What the rest is, by inspection: people's feet and ankles (0.3 m clusters
of occupied voxels 0.1–0.25 m above the floor at the robot's stops — the
operator's, seen once from ~3 m ahead and passed through 30–40 times, kept
by `clean_map`'s floor-band exemption until its `floor_band_dynamic` of
2026-10-14; they read as "the robot's own feet" at first, but the hits come
from a keyframe metres behind), the 1F–2F stairwell (a
switchback stair, four flights and three landings, 5.3 m up in a 4 × 3 m
well; refused by the geometry rules, so the floors are separate components
and only a stair link joins them), and a short ~30° ramp on 2F.

**Stairs on dp1f_1012** (2026-10-14, the map after `syncai_mapping`'s
local-floor and pool fixes): 3 stair links — kf 1677 → 1913 from 1F (z −0.10)
to 2F (5.20), 39 m and 87 points after the loop cut, and two across a 0.4 m
step on 2F. 1F → 2F plans (kf 0 → 2000: 85.6 m; kf 22 → 1913: 42 m, of
which the link is 39) take 15–30 ms and climb the four flights and three
landings in order; 2F → 1F takes the same link backwards.

**Map switch** (2026-10-19). The tree is read at startup, and nothing
watches the INI — the INI changing says nothing about when the rest of a
switch is done. Whoever switches maps calls (the backend's map switch does,
after `relocalize`; it skips `map_server/load_map` when the nav session has
no map_server, which is always since 2026-10-28):

```bash
ros2 service call /<robot_id>/planner_server/Octo3D/load_map nav2_msgs/srv/LoadMap \
    "{map_url: 'map/<name>/octomap.bt'}"
```

- **A result, not a receipt**: the call returns once the new surface map
  and its stair links are in (~1 s on dp1f_1012, a few s on a large site),
  so `RESULT_SUCCESS` means `Octo3D` plans on the new map. A caller's
  timeout must cover the build.
- **An empty `map_url` unloads** (2026-10-19): what the backend sends on a
  switch to a 2D map. The surface map is dropped, the file parameters are emptied and `RESULT_SUCCESS` is answered;
  every goal then fails `map_not_loaded`, since no 2D map navigates since
  2026-10-28 (until then the task_runner ran the 2D trees on it, and
  without the unload a 3D → 2D switch left the old octree loaded and the 3D
  trees running).
- `RESULT_MAP_DOES_NOT_EXIST` is answered **before anything changes**: the current map
  stays.
- Otherwise the current surface map is **dropped first** — plans fail with
  `map_loading` during the build, never on the old site's map, and only one
  tree is ever in memory — and a failed build (`RESULT_INVALID_MAP_DATA`)
  leaves **no map**, not the old one: every `Octo3D` plan then fails with
  `map_not_loaded` and the reason.
- `poses.txt` and `keepout.json` are always taken from beside the new tree
  (an explicit `poses_file` / `keepout.file` applies at boot only), and all
  three paths are written back to `Octo3D.octomap_file` / `poses_file` /
  `keepout.file`, which `ros2 param set` otherwise refuses. An absent
  `keepout.json` is no zones; a broken one loads the map without zones and
  logs an error (see "Keepout zones").
- The planner's own `clear_plan` is called, so `plan` stops re-sending the
  old map's route.
- One reload at a time on its own thread: a second call waits for the
  first, and the last one wins; a call during the boot build waits for it.
  Build-time parameters are not re-read — they still need a restart.

**The CLI.** `ros2 run syncai_global_planner octomap_planner_3d_cli` runs the
same core without the stack, and is the first thing to run on a new map:

```bash
octomap_planner_3d_cli map/<name>/octomap.bt --stats            # cells, rejects, components, floors, timings
   … --self-test                                                 # recompute random columns with OcTree::search()
   … --poses map/<name>/poses.txt --check-poses                  # the map's own track on traversable ground?
   … --poses … --inspect-misses 3 / --inspect-kf 19 / --inspect x y   # ASCII map of the cells around a point
   … --dump-image out.bmp [--image-z 5.1] [--image-region x0 y0 x1 y1 --image-scale 4] [--image-components]
   … --dump-pcd out.pcd                                          # x y z label reject unknown clearance component
   … --synthetic two_floors --write-bt out.bt                    # a test scene as a real .bt (the CI smoke's 3D fixture)
   … --plan sx sy sz gx gy gz --out path.csv / --plan-poses i j  # z are height hints
   … --plan-poses 0 1000 --plan-poses 2000 2250 --dump-html out.html   # 3D viewer: open in a browser
   … --poses … --plan-poses 0 2000 [--no-stair-links]           # with --poses, the track's stairs are links
   … --keepout map/<name>/keepout.json --plan … --dump-image out.bmp   # the zones, as the plugin reads them (see "Keepout zones")
   … --synthetic ramp|stairs|kerb|drop|wall|hole|two_floors      # the test scenes
   … --param max_slope_deg=25 --param heuristic_weight=1.5       # any SurfaceMapParams / SearchParams field
```

**Seeing a route.** Online, the console (or rviz2 from a workstation) shows
`plan` (`nav_msgs/Path`) over the localizer's latched `map_cloud`, and the
poses carry the floor's z, so an
`Octo3D` route lies on the floor of the point cloud. (`Octo3D/raw_plan`, the
un-shortcut chain behind `publish_raw_plan`, was removed on 2026-10-28; the
CLI's `--plan` reports the raw chain's length, and `--no-shortcut` keeps it.) Offline, `--dump-html`
writes a self-contained three.js page (~6 MB for a site map; the same page
style as the OctoMap viewer `clean_map`'s display build makes): the
traversable cells coloured by floor height, hole-filled ones tinted, a
subsample of the rejected cells in grey for the walls, and every `--plan` /
`--plan-poses` of the run as a tube with its start and goal, and, with
`--poses`, every stair link as a thick magenta tube with a cube on each
anchor — drawn on their own because a flight is all rejected cells, so
without them nothing in the page says where the stairs are (2026-10-15).
A "hide ground above z" slider looks at one floor of a multi-storey map;
it hides a route or a link only once its lowest point is above the cut, so
a 1F → 2F route stays whole while 1F is shown.
`--dump-image` draws the same routes on the top-down raster (blue, start
magenta, goal yellow), and `--dump-pcd` + `--out path.csv` are the raw
material for CloudCompare or a script.

On a workstation without ROS, `plugins/octomap_planner_3d/standalone/CMakeLists.txt`
builds the same binary against the system's octomap
(`cmake -S … -B /tmp/octo3d -DCMAKE_PREFIX_PATH=/opt/homebrew`); the core
headers include nothing of ROS on purpose.

**Dynamic obstacles** (2026-10-21). The surface map is static, so on its own
a replan returns the route a person is standing on, and the 2D global
costmap could not judge a 3D path (upstairs it lay over the ground floor's
gridmap; it has no layers at all since 2026-10-28). `Octo3D` therefore keeps
a **dynamic overlay** of its own (`dynamic_overlay.hpp`, ROS-free) — what the
2D global costmap's obstacle layer was to `GridBased`, in the surface map's
terms:

- It subscribes the lidar cloud the costmaps read (`dynamic.topic`,
  `pointlio/body_cloud`), through a TF message filter into the map frame,
  on the global costmap's callback group — the thread that takes `/tf` into
  the buffer the filter waits on, as the costmap layers' filters do. It had
  an executor thread of its own until 2026-10-22, when on robot01 the
  costmap's thread stopped for good mid-goal (every thread in `futex_wait`,
  most likely the filter and tf2 deadlocking across the two threads; no
  backtrace): `/tf` stopped reaching the buffer, `getRobotPose()` kept
  answering the last pose, and every later plan started 20 m away. Every return within
  `obstacle_max_range` (2.5 m) is dropped onto the surface it lies above —
  the highest layer of its column whose top face is at most one cell above
  the return, so two floors never see each other's people — and if it lies
  between `min_obstacle_height` (0.1 m) and `body_clearance` (0.6 m) above
  a **traversable** cell, that cell is marked with the scan's stamp. Before
  that, every return within `raytrace_max_range` (3 m) clears the marks its
  ray crosses: at each column the ray passes, the height it has there is
  checked against the band of the surface under it. A mark the robot
  cannot see (behind it, occluded, out of range) is forgotten after
  `obstacle_memory` (30 s).
- A plan takes a snapshot: per cell, the xy distance to the nearest live
  mark on the same floor (dilated over the footprint disc's own slope cone,
  so a ramp's uphill cells are reached and the floor a storey below is
  not). Cells within `block_radius` (0.35 m — the inscribed radius plus the
  margin the 2D footprint test sweeps) are not expanded by the A*, not in
  sight for the shortcut and refused to a goal snap; beyond the ring the
  clearance the cost factor reads is the smaller of the static one and the
  distance past the ring, so a person is paid for with the same curve as a
  wall and the detour keeps its distance where it has room. The start's own
  disc (`block_radius` round it) is exempt from the dilation — not from a
  mark — so a person beside the robot does not make its first step illegal.
  The components are baked, so a corridor a mark closes is `no_path`, with
  the mark count in the detail, not `disconnected`.
- `is_path_valid` for an `Octo3D` path is answered by the planner
  (`GlobalPlanner::isPathValid()`, which the server calls instead of its
  costmap test for the planner that made the last plan when that planner
  does not use the costmap): the pose nearest the robot in 3D, then every
  pose from there against the snapshot, with the same exempt disc. Two rules
  are the stair's: the check stops where the path leaves traversable ground
  (a stair link), judging only the floor stretch the robot is on, and it
  answers "valid" while the robot itself stands on no traversable cell
  (mid-flight) — a replan from there would snap its start to the wrong
  floor, and a goal aborting on a flight stops the robot on the stair. With
  `blocked_time`'s 10 s between a blocked answer and the replan, judging the
  floor above from the stair foot would fire exactly that.
- `planner_server/Octo3D/clear_overlay` (`nav2_msgs/ClearEntireCostmap`,
  the costmap clear's type, so the trees call it with the stock
  `ClearEntireCostmap` node) drops every mark: at every new goal and when a
  plan fails, with `move.xml`'s semantics.

Three decisions shape it (the header of `dynamic_overlay.hpp` has them in
full): only traversable cells are marked, because a wall's returns land in
the 0.25 m footprint ring and inflating those would narrow every corridor by
0.1–0.2 m a side and close every door under ~0.9 m whenever a wall is in
range — so a person flattened against a wall is the local costmap's to see;
returns within `self_radius` (0.6 m) of the sensor are ignored, because the
robot's own legs sit in the band and marked the cells under the robot on
every scan, so the path's first poses read blocked for good; and marks
decay, because no ray reaches what the robot has walked away from.

`octomap_planner_3d_cli --dynamic <x> <y> <z>` (repeatable) drops a
synthetic person (eighteen returns 0.3 and 0.5 m up on a 0.3 m square) into
the overlay before every `--plan` and prints the plan's mark count: on
`--synthetic flat` a person at (6, 4) turns the 8.00 m line from (2, 4) to
(10, 4) into an 8.34 m detour, and one in the `wall` scene's door makes the
plan through it `no_path`. `test/test_surface_map.cpp`'s `DynamicOverlay`
cases are the promises above, one each.

**Keepout zones** (2026-10-23). What `map/<name>/keepout.yaml` +
`keepout.pgm` were to the 2D global costmap's `KeepoutFilter` (removed with
2D navigation, 2026-10-28) — which this planner never read, so until
2026-10-23 nothing kept the robot out of a zone on a 3D map (decided 2026-10-19, pending a decision on floors). The
zones of a 3D map are **`map/<name>/keepout.json`** beside `octomap.bt`:

```json
{ "version": 1,
  "zones": [ { "id": "zone-7f3a", "name": "server rack 2F",
               "points": [{"x": 12.4, "y": 3.1}, {"x": 15.0, "y": 3.1},
                          {"x": 15.0, "y": 5.6}, {"x": 12.4, "y": 5.6}],
               "z_min": 4.5, "z_max": 5.5 } ] }
```

Vector polygons, not a raster: a 3D map has no gridmap to size a mask from,
a site map has several storeys, and one 2D image cannot say which floor a
pixel forbids — the corridor on 1F and the room above it share an (x, y). So
each zone is a polygon in the map frame (metres, three or more `{"x", "y"}`
points, implicitly closed — a repeated first vertex is dropped — either
winding, even-odd if self-crossing) with a **z band** naming its floor:
`z_min` / `z_max` are optional (absent or `null` = unbounded that side) and a
backend writes `floor_z ± 0.5`.

**One file for 2D and 3D.** This is the file the backend writes for a 2D
map's zones too (beside the `keepout.yaml` / `keepout.pgm` mask it
rasterises from it), in the same `points` shape, with `name`, `z_min` and
`z_max` added as optional keys. A zone without a band covers every floor —
what a 2D zone means — so a file the 2D editor wrote applies unchanged on a
3D map, and the 2D mask ignores the bands (no 2D map navigates since
2026-10-28, so the mask has no reader in this repo). The backend must keep the three
keys when its 2D editor rewrites the file, or a save there drops a 3D zone's
floor. `id` is required, non-empty and unique in the file
(the logs and `error_msg` name zones by it), `name` is for display,
`version` is optional (absent = 1; a larger one is refused as written by a
newer backend), unknown keys are ignored, every number must be finite.
Nothing in the file names the map or holds a path, so a map directory still
renames with one `os.rename`. The rules are enforced by
`keepout_zones.hpp`'s parser, which refuses the file with the zone and key
named; `test/test_surface_map.cpp`'s `KeepoutZones` cases are the contract.

What a zone does, on the surface map: a cell is in the zone's **core** when
its column centre lies inside the polygon and its top face in the band — so
the same polygon with band 4.5..5.5 forbids 2F and leaves 1F alone — and the
core is dilated over the surface by **`keepout.block_radius`** (0.25, the
robot's radius: the 2D costmap's inscribed radius, what the 2D
`KeepoutFilter` inflated its mask with, so a zone closes any gap narrower than the robot
exactly as a wall does and the editor draws the forbidden area only, no
margin), and costed beyond that out to `clearance_cost_radius` with the
walls' own curve. The dilation is the dynamic overlay's (`dilateOnSurface`):
it follows a ramp and stops at a floor edge the way a mark's does, so one
floor's zone never reaches the other. The mask (`KeepoutMask`) rides beside
the overlay's in every `BlockedView`, so the goal snap, the search, the
shortcut and `is_path_valid` keep off a zone exactly as they keep off a
person: a goal inside one fails `goal_unavailable` ("… in keepout zones"), a
zone across the only corridor fails `no_path` with the zones named (the
components are baked, so never `disconnected`), and a **stair link any of
whose points lies in a core is cut** — never expanded by the search, so a
zone over a stair foot forbids that stair (`no_path`, "… cut N stair
links"). A robot **standing inside a core is refused a plan**
(`start_in_keepout`, naming the zone; the 2D planner fails on a lethal start
the same way) and the operator drives it out; the ring round the robot's own
cell is exempt as it is for a mark, so standing next to a zone edge is fine.
`clear_overlay` drops marks and leaves zones alone.

The file is read with the tree — at boot and on every `load_map`, always
from beside the new tree (an explicit `keepout.file` applies at boot only,
and is written back like `octomap_file`). Absent = no zones, one info line.
A **broken file beside a good tree loads the map without zones** and logs an
error saying the zones protect nothing: failing the whole load would leave
*no map* (a reload drops the old one first), which strands the robot worse
than missing zones. Reloading an edited file needs no rebuild:

```bash
ros2 service call /<robot_id>/planner_server/Octo3D/load_keepout nav2_msgs/srv/LoadMap \
    "{map_url: 'map/<name>/keepout.json'}"
```

| `map_url` | Result | Effect |
|---|---|---|
| `""` | `RESULT_SUCCESS` | no zones: the mask dropped, `keepout.file` written back empty |
| a file that does not exist | `RESULT_MAP_DOES_NOT_EXIST` | nothing changes (as `load_map`) |
| any, with no 3D map loaded | `RESULT_UNDEFINED_FAILURE` | nothing to rasterise on; the next `load_map` reads the file beside its tree |
| a file the parser refuses | `RESULT_INVALID_MAP_DATA` | the **current zones are kept** (stale zones are safer than none; unlike `load_map` there is no memory reason to drop first), the error logged |
| a good file | `RESULT_SUCCESS` | parsed and rasterised outside the plan lock, swapped in under it; waits for a boot build and for a plan in flight (≤ `max_planning_time`). **No `clear_plan`**: the current route is judged against the new zones by the next `is_path_valid` (1 Hz), and `move_3d.xml` replans after `blocked_time` |

On `load_map`'s thread, so a reload and a keepout load never interleave. The
backend writes the file and calls this after an edit; a map switch needs no
extra call. `octomap_planner_3d_cli --keepout <file>` reads it for every
`--plan` as the plugin does and draws the cores (crimson) and rings (dark
red) in `--dump-image`: on `--synthetic flat` a 2 × 2 m box across the line
from (2, 4) to (10, 4) turns the 8.00 m route into an 8.97 m detour, a start
inside it is `start_in_keepout`, and on `stairs` a box over the flight cuts
all 41 links and the upper floor is `no_path`.

**Known limits** (prototype):

- The keepout is the planner's alone: the controllers' local costmap has no
  filter (as in 2D), so the planner keeps the robot out and MPPI cannot
  reject a path for grazing a zone edge. `is_path_valid` sees a zone only
  after a `load_keepout` changed it under a running path (a path planned
  with the zones never crosses one).
- `is_path_valid` for an `Octo3D` path judges only the floor stretch the
  robot is on, answers "valid" on a stair, and sees nothing the overlay did
  not mark: a person inside a wall's 0.25 m ring, anything the lidar's −7°
  lower field of view has not reached yet, anything behind the first thing
  the rays hit. After a `clear_overlay` the first plan runs before the next
  scan re-marks a person in view (≤ 0.1 s; the 2D tree's planner waited for
  its costmap to be current instead, until 2026-10-28), and `IsPathValid` replans after its
  10 s. The republish trim on `plan` still uses the server's planar
  nearest-pose search.
- A route over a stair link is only a route: climbing it is the controllers'
  stair mode (`syncai_util::StairMode`, run by MPPI and `FollowPathRPP3D`;
  `syncai_controller/README.md`, "Stairs"), which stops and switches the gait
  controller's RL policy over `set_policy_mode`. Climbing has been run on
  robot01.
- A stair link exists only where the robot walked; a stair blocked since (or
  walked only one way by a robot that cannot descend) is still linked both
  ways. A straight link's direction comes from the stair's surfaces, not the
  walk, but its width is only tried `stair_straight_half_width` either side
  of the walked foot, and a flight that is not one plane (a switchback, a
  turning landing) still follows the walked track, drift and all.
- A 2D backend's z = 0 goal is only right on the ground floor (or on any
  single-layer ground, by `snap_single_layer`); where floors overlap it
  needs a real z or `goal_z_mode: start`.
- A map switch needs its own call: `Octo3D` reloads on
  **`planner_server/Octo3D/load_map`** (see "Map switch" below). The build-time parameters still need a
  planner_server restart.
- Shallow stairs with 0.1 m risers are a 19° ramp at this resolution; the
  tilt limit is the only guard, and a quadruped walks them anyway.
- The failure reason reaches `NavigateToGoal.error_msg` only as
  `PLAN_FAILED`; the text is in the planner log.

## Parameters

**Server** (`/**/planner_server`):

| Parameter | Default | Notes |
|---|---|---|
| `planner_plugins` | `["Octo3D"]` (config: `["Octo3D"]`) | IDs; each needs `<id>.plugin` naming the type. With one loaded, an empty `planner_id` in a goal picks it; with more, an empty id is an error. Until 2026-10-28 the default was NavFn as `GridBased`, and the config loaded `GridBased` (SmacPlanner2D) beside `Octo3D` |
| `expected_planner_frequency` | `1.0` (config: `20.0`) | Warning threshold only — a plan taking longer than 50 ms logs a missed-rate warning. Generous by design: the BT asks for a plan once per goal and again only when the current path is blocked, so the warning is a canary for a pathological search, not a cadence anyone is trying to hit |
| `plan_republish_rate` | `1.0` (config: `1.0`) | Hz at which the last plan is re-sent on `plan`, restamped and trimmed to start at the pose nearest the robot (the whole plan when the robot pose is unavailable); `0` = only when a plan is made. Needed since `move.xml` plans only when the path is blocked: without it a viewer subscribing mid-drive never sees the route. The planner cannot tell when the navigation finished, so the task_runner reports a goal that **succeeded or was cancelled** through `clear_plan`; after a failure the last route keeps showing for `plan_idle_clear_time`. Read once at startup |
| `plan_idle_clear_time` | `30.0` (config: `30.0`) | s (2026-10-25) after which a plan that no navigation has made or checked (`is_path_valid`, which every running tree calls once a second) is dropped, as `clear_plan` drops it; `0` = never. The end of a **failed** navigation's route, which nothing else clears: until then it stayed on `plan` until the next goal (on dp, 1.5 h after the task ended), and the 1 Hz re-send kept the backend's own expiry from firing. Checked before the subscriber test, so a viewer subscribing later gets no stale route. Logs "No navigation has used the plan". Read once at startup |

`expected_planner_frequency` is the only parameter the *server's* dynamic
callback handles, and it takes the same mutex the plan cycle holds.
`Octo3D` registers its own callback (see its table). Swapping the *plugin*
needs a restart. Removed on 2026-10-28 with 2D navigation:
`costmap_wait_timeout` (how long a `GridBased` goal waited for the costmap
to be current) and `is_path_valid.footprint_check` / `heading_margin` (the
padded-footprint perimeter test `is_path_valid` ran on the global costmap for
a 2D path, RPP's own collision test, at the path heading ± 0.35 rad).

**Global costmap** (`/**/global_costmap`) — no layers, no filters (2026-10-28):
`global_frame: map`, `robot_base_frame` (the launch file prefixes it),
`transform_tolerance`, `rolling_window: false`, `track_unknown_space: true`,
`resolution`, and 1 Hz update / publish. `plugins` is left out of the YAML on
purpose: `CostmapROS`'s default is now no layer (upstream's is a static
layer), an empty YAML list cannot be typed, and `LayeredCostmap` skips the
update and reads current with nothing loaded. The node stays because the
server and `Octo3D` take its global frame, TF buffer, `getRobotPose()` /
`transformPoseToGlobalFrame()` and callback group (`Octo3D`'s overlay TF
filter runs on it — see "Dynamic obstacles"). Until then it ran static →
obstacle (`scan` + `pointlio/body_cloud`) → inflation (`inflation_radius`
0.8 / `cost_scaling_factor` 1.5, the 2D planner's wall clearance) plus the
keepout filter on `map/<name>/keepout.yaml`, with a 0.70 × 0.44 m footprint
padded 0.03 m so that `is_path_valid`'s perimeter test was stricter than
RPP's on the local costmap. Several `Octo3D` values were set to agree with
those (`robot_radius` 0.25 = that costmap's inscribed radius,
`clearance_cost_radius` reaching its 0.8 m band, the overlay's 2.5 / 3.0 m
ranges = the obstacle layer's); they are the robot's now, not the costmap's.

**OctomapPlanner3D** (`Octo3D.*`; the YAML carries the reasoning per line):

| Parameter | Default | Notes |
|---|---|---|
| `octomap_file` | `""` (launch: `map/<name>/octomap.bt`) | Derived by the launch file from the directory of `[map] pcd`; empty or missing → every `Octo3D` plan fails with `map_not_loaded`, nothing else is affected |
| `poses_file` | `""` (= `poses.txt` next to `octomap_file`) | The map's track, for the stair links; missing → a warning and no links. Build-time |
| `body_frame`, `lidar_height` | `pointlio_body` (launch: `<robot_id>/pointlio_body`), `0.481` | The frame the robot's real height is read from, and the sensor's height above the ground (`map_clean.recipe.json`'s `lidar_height`). Dynamic |
| `goal_z_mode` | `request` | `request`: the goal's own z picks the floor; `start`: the robot's floor. Dynamic |
| `robot_radius`, `body_clearance` | `0.25`, `0.6` | Footprint disc (= the padded footprint's inscribed radius, and the local costmap's terrain_layer `robot_radius`) and the band above a surface the body needs. Build-time |
| `max_step_height`, `max_slope_deg` | `0.17`, `20.0` | The legs' step (two cells at 0.1 m); tilt of the fitted plane under a disc spanning more than one step. Build-time |
| `footprint_min_support` | `0.8` | Share of the disc's columns that must hold a usable surface; the rest may be gaps (never observed), not walls or drops. `1.0` = every column. Build-time |
| `max_unknown_clearance_cells`, `min_known_free_above` | `-1` (all), `1` | Unknown tolerated in the band; observed-free voxels required right above a surface. Build-time |
| `hole_fill_radius`, `hole_fill_min_support`, `hole_fill_passes` | `0.3`, `0.5`, `2` | See the build, step 3. Build-time |
| `clearance_cost_radius` | `0.55` | Reach of the clearance field from the traversable boundary. Build-time |
| `step_edge_cost_radius`, `kerb_min_length`, `kerb_min_sure_cells` | `0.4`, `1.0`, `3` | The step edges' cost band, and what makes a line of step cells a kerb (see **Kerbs**). Build-time |
| `kerb_links`, `kerb_lead`, `kerb_fit_radius` | `true`, `0.5`, `0.5` | Kerb links: a straight crossing at every cell of a kerb, the lead either side, the fit of the kerb's direction (see **Kerbs**); `false` = a kerb is crossed cell to cell at any angle. Build-time |
| `step_edge_cost_weight`, `step_edge_cost_scaling`, `step_edge_one_cell`, `kerb_cost_multiplier` | `1.0`, `5.0`, `0.5`, `1.5` | The band's cost curve, a one-cell step's share of it, and the factor on a kerb link's length (≥ 1). Dynamic |
| `stair_links`, `stair_min_rise`, `stair_max_length`, `stair_anchor_z_window`, `stair_anchor_min_area`, `stair_smooth_passes` | `true`, `0.3`, `60.0`, `0.3`, `10.0` m², `2` | Stair links from the track (see **Stairs**). Build-time |
| `stair_straight`, `stair_straight_max_flight`, `stair_straight_fit_radius`, `stair_straight_half_width`, `stair_straight_exit_reach` | `true`, `8.0`, `1.0`, `2.0`, `0.6` | Straight links across a single flight's width instead of one along the walk (see **Stairs**); `false` = the walked link only. Build-time |
| `stair_cost_multiplier` | `2.0` | Factor on a link's 3D length (≥ 1). Dynamic |
| `clearance_cost_weight`, `clearance_cost_scaling` | `1.0`, `1.5` | Edge cost multiplier inside the band. Dynamic |
| `inferred_cost_multiplier` | `1.2` | Prefer observed floor. Dynamic |
| `gap_cost_weight` | `1.0` | Cost factor 1 + weight × gaps / disc columns. Dynamic |
| `heuristic_weight`, `max_iterations`, `max_planning_time` | `1.0`, `2000000`, `2.0` s | Weighted A* and the guards. Dynamic |
| `snap_xy_radius`, `snap_z_window` | `0.5`, `1.0` | How far a start / goal may be moved to reach traversable ground. Dynamic |
| `snap_single_layer` | `true` | Nothing within `snap_z_window` and one traversable layer within `snap_xy_radius`: snap to it at any height (sloping ground under a z = 0 goal). `false` = the window is absolute. Dynamic |
| `shortcut`, `interpolation_resolution`, `path_z_offset` | `true`, `0.1`, `0.0` | Post-processing. Dynamic |
| `dynamic.enabled`, `dynamic.topic`, `dynamic.transform_tolerance` | `true`, `pointlio/body_cloud`, `0.3` | The dynamic overlay's input (see **Dynamic obstacles**); `false` = no mark ever, every path valid, no detour (the 2026-10-20 behaviour). Startup only |
| `dynamic.obstacle_max_range`, `dynamic.raytrace_max_range` | `2.5`, `3.0` | Planar range within which a return marks / its ray clears — the 2D obstacle layer's values until 2026-10-28. Dynamic |
| `dynamic.self_radius` | `0.6` | Returns closer to the sensor are neither marks nor rays: the robot's own legs (0.2–0.3 m up, ~0.5 m out) sit in the band. Dynamic |
| `dynamic.min_obstacle_height` | `0.1` | Band bottom above a surface's top face; the top is `body_clearance`. The terrain layer's band. Dynamic |
| `dynamic.obstacle_memory` | `30.0` s | How long an unrefreshed, uncleared mark lives: outlasts `blocked_time` (10 s) plus a plan plus the detour's blind stretch. Dynamic |
| `dynamic.block_radius` | `0.35` | Cells within it of a mark are blocked (search, shortcut, goal snap, `is_path_valid`); also the exempt disc round the start. Between the inscribed 0.25 and the circumscribed 0.455, what the removed 2D perimeter test swept. Dynamic |
| `keepout.file` | `""` (= `keepout.json` next to `octomap_file`) | The zones (see **Keepout zones**); absent = none. Boot only: `load_map` takes the new tree's own and writes it back; `ros2 param set` is refused (call `load_keepout`) |
| `keepout.block_radius` | `0.25` | Ring round a zone's core within which cells are blocked: = `robot_radius` (what the removed 2D `KeepoutFilter` inflated its mask with) — change the two together. Dynamic (re-rasterised) |

A build-time parameter set through `ros2 param set` is refused with the
reason; it needs a planner_server restart.

## Interfaces

With the node at `/<robot_id>`:

| Interface | Name | Type |
|---|---|---|
| Action | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` |
| Service | `is_path_valid` | `nav2_msgs/IsPathValid` — is the part of a path from the pose nearest the robot onward free? Asked of the planner that made the last plan (`GlobalPlanner::isPathValid()`; the request carries no planner id, the server remembers the last plan's): `Octo3D` judges on its dynamic overlay and its keepout zones (see **Dynamic obstacles** and **Keepout zones**). An empty remembered id — before a first plan, or after `clear_plan` — falls back to the only loaded planner, as `getPlan()` does; an id no loaded planner has answers `false` with a throttled warning, so the BT replans. Called once a second by `move.xml` / `dock.xml`'s `IsPathValid`, which replans only once the same spot has answered `false` for 10 s (`blocked_time`, 2026-10). Answers `false` for an empty path or a frame other than the costmap's; answers `true` when there is **no robot pose** (a TF miss must not force a replan whose own start lookup fails). Served on the main executor, so it never queues behind a plan. Until 2026-10-28 a path of a planner on the costmap (`GridBased`) was judged here on the global costmap instead: the centre cell `< INSCRIBED` and, with `is_path_valid.footprint_check`, no LETHAL cell under the padded footprint's perimeter at the path heading ± `heading_margin` (RPP's own collision test) |
| Service | `planner_server/Octo3D/load_map` | `nav2_msgs/LoadMap` (2026-10-19), map_server's own type — reload `Octo3D` from `map_url` (`map/<name>/octomap.bt`; `poses.txt` from beside it) on a map switch. See "Map switch" under `OctomapPlanner3D`. Its own callback group and executor thread, so a multi-second build never stalls `is_path_valid`, `clear_plan` or the republish |
| Service | `planner_server/Octo3D/load_keepout` | `nav2_msgs/LoadMap` (2026-10-23) — replace `Octo3D`'s keepout zones from `map_url` (`map/<name>/keepout.json`; `""` = none) on the loaded map, without rebuilding it. What the backend calls after it writes the file; a map switch needs no call, `load_map` reads the new map's own. Result table under "Keepout zones". On `load_map`'s thread |
| Service | `planner_server/Octo3D/clear_overlay` | `nav2_msgs/ClearEntireCostmap` (2026-10-21) — drop every mark of `Octo3D`'s dynamic overlay (the keepout zones stay); what the trees' planner-side `ClearEntireCostmap` nodes call (at every new goal, and when a plan fails) — the global costmap's own clear is a no-op for `Octo3D` (and the costmap has no layers to clear). Main executor |
| Subscriber | `pointlio/body_cloud` | `sensor_msgs/PointCloud2` (2026-10-21, `Octo3D.dynamic.topic`) — the lidar cloud for the dynamic overlay, through a TF message filter into `map`. Created on the `global_costmap` node and its callback group since 2026-10-22, so the costmap's TF thread runs the filter (see **Dynamic obstacles**); the name is still resolved in the planner's namespace. Dropped unread while `Octo3D` has no map |
| Service | `clear_plan` | `std_srvs/Empty` (2026-10-14) — forget the last plan and send an empty `plan` (frame `map`, not empty: the backend drops any other frame before it looks at the poses), then two more at `plan_republish_rate`, because the backend reads `plan` BEST_EFFORT and one dropped message would leave the console drawing the old route. Called by the task_runner when a navigation is **cancelled** (2026-10-14; until then the cancelled route stayed on `plan` until the next goal) or **succeeds** (2026-10-22; the trimmed re-send left a stub that grew back along the route once the robot moved off the goal), and by `Octo3D` on a map switch. The same drop runs by itself once the plan has gone unused for `plan_idle_clear_time` (2026-10-25: the end of a failed navigation's route). A new plan stops the repeats. Main executor, the republish timer's thread |
| Publisher | `plan` | `nav_msgs/Path` (visualization; skipped when nothing is subscribed). Sent when a plan is made, and the last one again at `plan_republish_rate`, from the robot's nearest pose onward; empty after a `clear_plan` or `plan_idle_clear_time` unused |

Plus everything the internal costmap exposes under
`/<robot_id>/global_costmap/…` — `costmap` (the empty 5 × 5 m default grid),
`costmap_updates`, `published_footprint`, `get_costmap`, and the three
`clear_*_global_costmap` services, which no tree calls since the 2D trees
went (2026-10-28).

## Running

```bash
ros2 launch syncai_global_planner planner_server.launch.py
ros2 launch syncai_global_planner planner_server.launch.py \
    system_config:=/path/to/other.ini
```

A plan without the task runner (the planner the BT asks for), and the same
core offline on the map's own tree (see OctomapPlanner3D above):

```bash
ros2 action send_goal /<robot_id>/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: Octo3D}"
ros2 run syncai_global_planner octomap_planner_3d_cli map/<name>/octomap.bt --stats \
  --poses map/<name>/poses.txt --check-poses
```

The nav session starts it in the `plan_ctrl` window after `sleep 4` —
`CostmapROS::activate()` blocks on TF. (The offset also used to let the
static layer find `map_server`'s latched map; there is no map_server in the
session since 2026-10-28.)

The launch file does the same two namespace-critical things as the controller's:
**no `name=` on the `Node`** (the process hosts `planner_server` *and*
`global_costmap`; a launch-level name would remap both and strip the costmap of
its parameters), and explicit `<robot_id>/` prefixes for TF frame parameters,
which land under the `/**` wildcard and reach the costmap node.

Watching the result and dropping the overlay's marks by hand:

```bash
ros2 topic echo /<robot_id>/plan --once
ros2 service call /<robot_id>/planner_server/Octo3D/clear_overlay \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

## Gotchas

- **A plugin load failure calls `exit(-1)`** — a typo in a `.plugin` string
  takes the whole process down at startup.
- **`robot_radius` must match the controller's footprint.** `Octo3D`'s
  0.25 m is the padded footprint's inscribed radius, and the local
  costmap's terrain layer uses the same value; a planner disc narrower than
  the robot plans paths MPPI then cannot drive. (Until 2026-10-28 the rule
  was the global costmap's footprint against the local one.)
- **Dynamic parameter changes wait for a plan in progress** on the shared
  mutex, and only `expected_planner_frequency` is handled at all — plugin
  parameters go to each plugin's own callback.
- **`is_path_valid` is answered by the planner that made the last plan**
  (`Octo3D`, on its dynamic overlay — "Dynamic obstacles"). The request
  carries no planner id; the server remembers the last plan's, and an empty
  one (before a first plan, after `clear_plan`) falls back to the only
  loaded planner. A person inside a wall's 0.25 m ring is never marked, by
  decision; the local costmap's terrain layer and MPPI deal with them.
  Until 2026-10-21 the check judged every path on the 2D costmap, which
  upstairs read a good path as blocked, and the 3D tree planned once per
  goal with no replan round a blocker.
- **`Octo3D` does not follow a map switch by itself.** Nothing watches the
  INI: the caller of the switch (the backend) must call
  `planner_server/Octo3D/load_map` with the new `octomap.bt` (2026-10-19;
  until then a switch
  needed a planner_server restart, and without one `Octo3D` went on planning
  on the old site's map). A build-time parameter change still needs the
  restart — the session's `plan_ctrl` pane, or `restart_mode`.
- **`Octo3D` trips the `expected_planner_frequency` warning** on a long
  plan only in theory (a 50 m search is a few ms); if it does, it is the
  canary described under Parameters, not a fault.
- **A 10–13 cm kerb is invisible to the kerb links on a 0.1 m octomap.**
  The surface map reads heights off voxel tops, so such a kerb is a
  one-cell step along most of its length -- the same as the contour lines
  a gently sloping road makes every metre or so -- and only a line with
  `kerb_min_sure_cells` two-cell cells becomes a kerb (**Kerbs**).
  outdoor_1027's kerb (12 cm, the two falls of 2026-10-28) is in the
  one-cell band, which charges for running along it, but gets no links and
  no square crossing, and with the road's contour bands charging the same
  the route still runs along it (2026-10-29, `--plan 0 0 -0.4 63.46 29.5
  -2.1`). Telling the two apart needs the true floor height, which
  `map.pcd` holds (the sim's `surface_heights.py` reads the kerb off it at
  0.12 m) and the octomap does not: a per-column median of the cloud's
  floor points at build time would make every step metric and every
  contour line vanish. Until then, a keepout zone over the pavement is the
  way to keep a route off such a kerb.

Upstream reference: [`nav2_planner`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_planner)
(the removed 2D plugins were ports of `nav2_navfn_planner` and
`nav2_smac_planner`).
