# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A ROS 2 Humble software stack for the SyncAI robot (G23 quadruped / AMR chassis),
covering the full vertical: sensor drivers, LIO odometry, a **non-lifecycle port
of Navigation2**, a Temporal-backed task orchestration backend, and a Next.js
operator UI.

Two things shape almost every decision here:

1. **No lifecycle nodes.** The nav2 servers (AMCL, map server, costmap, planner,
   controller, BT navigator) were re-implemented as plain `rclcpp::Node`s. There
   is no lifecycle manager — every node comes up active. This means **startup
   order matters** and is handled by `sleep` offsets in the byobu launch scripts.
2. **Everything is namespaced by `robot_id`.** A single DDS domain can host
   several robots, so node namespaces, topics, and TF frames are all prefixed.
   See "The robot_id convention" below — this is the single most common source of
   mistakes when editing launch files or params.

`README.md` covers clone/build/run from a user's perspective. This file covers
the conventions and gotchas you need to edit the code correctly.

## Package map

### Navigation (C++, nav2 port)

```
NavigateToPose (nav2_msgs) → syncai_task_runner   (BT navigator; ticks behavior_trees/move.xml)
                                   │
                compute_path_to_pose│follow_path
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
             syncai_planner                 syncai_controller
            (NavFn, pluginlib)           (Regulated Pure Pursuit)
                    │                              │
                    └────── syncai_costmap_2d ─────┘
                       (global costmap / local costmap,
                        layered plugins + keepout filter)
```

| Package | Role |
|---|---|
| `syncai_nav_core` | Header-only abstract plugin interfaces (port of `nav2_core`) |
| `syncai_util` | Header-only helpers (geometry, occupancy-grid values) |
| `syncai_common` | Shared msg / srv / action interfaces (`RobotState`, `SetMotionKey`, `ExecuteTask`, …) |
| `syncai_costmap_2d` | Costmaps with layered plugins (static / obstacle / inflation / keepout filter) |
| `syncai_planner` | `ComputePathToPose` action server; NavFn plugin merged in |
| `syncai_controller` | `FollowPath` action server; Regulated Pure Pursuit merged in (clamps linear accel itself — there is no velocity smoother in the stack) |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`) |
| `syncai_task_runner` | The BT navigator. Serves `nav2_msgs/NavigateToPose`, hosts the `Navigator<ActionT>` abstraction and `behavior_trees/*.xml`. (README still calls this `syncai_bt_navigator` — that package does not exist.) |
| `syncai_amcl` | 2D AMCL. Only used by the 2D bringup path; the 3D path replaces it with `syncai_lio_bridge`. |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server |

### Localization & sensing

| Package | Role |
|---|---|
| `syncai_lio_bridge` | **The only odometry source.** Wheel odom is retired. Converts the FAST-LIO2 chain (`map → lio_odom → lio_body`) into `odom → base_link` TF + `/<robot_id>/odom` + the AMCL-style `map → odom` correction, all projected to 2D (x, y, yaw) so the planar nav stack never sees a tilted frame. Angular velocity comes from the lidar IMU gyro because LIO leaves `twist.angular` empty. |
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` (carries the `lidar_top` mount extrinsic the LIO bridge needs) + the Livox driver. The fleet runs **both MID360 and MID360s**; the driver has no ROS parameter for the model, so `[sensor.lidar] type` (`mid360`/`mid360s`) picks the JSON schema. The driver's network JSON is **generated** per `robot_id` and model into `/tmp/syncai_bringup/` from `[sensor.lidar] ip` + `type` (INI) + `host_ip` (params YAML) — the vendor `MID360_config.json` / `MID360s_config.json` in the submodule's share dir is not read. The old 2D/AMCL `bringup_2d.launch.py` (laser scan merger) was removed. |

### Hardware & system

| Package | Role |
|---|---|
| `syncai_driver_manager` | **UDP bridge to the gait controller.** Sends `cmd_vel` (with per-direction velocity-scale correction — the gait controller tracks commands asymmetrically), receives ASCII telemetry, and owns the safe-shutdown path (engages a safety lock and commands MODE X / lie down). |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motor_states / TF into `syncai_common/RobotState` at 10 Hz. Also derives the `state` field: `UNINITIALIZED` (no pose) / `WARNING` (battery <20%, latched with hysteresis) / `IDLE`. Reports only — no threshold here commands the robot. |
| `syncai_sys_manager` | Python: wifi, mDNS, map, and system managers behind ROS services |

### Application layer

| Package | Role |
|---|---|
| `syncai_backend` | Python. FastAPI **and** rclpy in one process (`MultiThreadedExecutor`), port **3000**. Temporal worker for task orchestration. |
| `syncai_frontend` | Next.js + shadcn/ui + raw three.js, port **3001** |
| `syncai_ros_mcp` | Python (`ament_python`). MCP server exposing the ROS 2 graph as MCP tools over HTTP, port **8000**. One process: `rclpy.spin()` owns the main thread, FastMCP runs on a background daemon thread. The topic/service tools read the live graph through the node; the task/map tools are thin REST clients for `syncai_backend` (`SYNCAI_BACKEND_BASE_URL`, default `http://localhost:3000`). |

`syncai_ros_mcp` is **vendored** (source committed here), unlike `syncai_common`
and everything under `src/third-party/`. It started as the submodule
`chungweeeei/SyncAI-ROS-MCP` and was folded in on purpose: it is developed only
against this workspace, and the submodule indirection meant every change needed
two commits and left the workspace pinning a stale pointer. `FastMCP` is a
pip-only dependency (no rosdep key), so `rosdep install` does not cover it —
`python3 -m pip install "fastmcp>=3.4.4"` into the same interpreter `ros2 run`
uses.

### Third-party (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | Submodule, pinned to upstream tag `3.8.8`. Unmodified. |
| `FASTLIO2_ROS2` | Submodule → `chungweeeei/SyncAI-Fast-LIO2` (branch `dev`). Contains LIO + PGO + HBA + `localizer`. |
| `livox_ros_driver2`, `Livox-SDK2` | Submodules (MID360 driver) |
| `small_gicp` | Submodule, pinned to upstream tag `v1.0.1`. Unmodified. The `localizer`'s registration backend (`RegistrationPCL`, a `pcl::Registration` subclass). Ships its own `package.xml` with `<build_type>cmake</build_type>`, so colcon builds it as a plain CMake package and `localizer` finds it with `find_package(small_gicp)`; the ordering comes from `<depend>small_gicp</depend>` in the localizer's manifest. |
| `ros2_laser_scan_merger` | **Vendored** (source committed) — locally modified |

## The robot_id convention

Every launch file reads `[system] robot_id` from **`config/system.ini`** via a
`read_robot_id()` helper, falling back to `default_robot` with a warning. The
resolved value is used as the **node namespace**.

`config/system.ini` is tracked but **empty**. Per-robot identity comes from
`config/instances/robotNN.ini`, which docker-compose bind-mounts over
`config/system.ini` inside the container. That file also carries
`[artifacts]` endpoints, `[initial_pose]`, `[map]` pcd/gridmap paths, and
`[sensor.lidar]` `ip` + `type` (the lidar's address and model, `mid360` or
`mid360s` — `syncai_bringup` renders the livox driver's config JSON from them).
The model is load-bearing: the JSON *section name* is what Livox-SDK2 maps to a
device type, and that decides the command handler, so a wrong `type` yields no
point cloud and no error. A missing `type` warns and falls back to `mid360`, for
back-compat with instance INIs written before the MID360s arrived.

Launch files use the relative path `config/system.ini`, which works because
**processes are expected to run with the workspace root as their cwd** — the
byobu scripts start every pane in the workspace root, and the backend gateways
assume the same.

Three rules follow from namespacing:

- **Topics are written as relative names** in params YAML and in code
  (`map`, `scan`, `pointlio/body_cloud`), so they inherit the namespace
  automatically. Never hardcode `/<robot_id>/…` in a subscriber — a backend
  subscriber that used an absolute topic name is a bug that has already been
  fixed once. There are **no** exceptions: an absolute, fleet-wide `/robot_state`
  was tried and reverted, because a single DDS domain hosts several robots and
  every per-robot consumer (the backend included) is scoped to exactly one.
- **TF frame names are NOT namespaced by ROS.** Launch files therefore override
  frame parameters explicitly (`robot_base_frame: <robot_id>/base_link`,
  `sensor_frame: <robot_id>/laser`). The values in the YAML are only fallbacks
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
docker compose -f docker-compose.robots.yml --profile real up -d robot01
docker compose exec robot01 bash
# inside, workspace is at /home/syncrobotic/robot_ws and is the cwd
colcon build --symlink-install
colcon build --packages-select syncai_planner
source install/setup.bash
```

Recreating a robot container wipes hand-installed build dependencies (the ones
not in the image). Re-run `rosdep install --from-paths src --ignore-src -r -y`
plus any manual deps (Sophus / GTSAM are built from source for `FASTLIO2_ROS2`).

`colcon.meta` at the workspace root carries per-package cmake args. colcon only
finds it because its `--metas` default is the relative `./colcon.meta` — same
cwd-is-the-workspace-root assumption as the launch files. Today its only entry is
`livox_ros_driver2`, which needs `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`: since
the Mid-360s/Jazzy commit, its `CMakeLists.txt` branches on `DISTRO_ROS`, and
without it the pre-Humble branch resolves the message-typesupport include dirs
off a target name that no longer exists and configure dies with `NOTFOUND`. The
flags belong here rather than in the submodule because upstream only ever
configures itself through its own `build.sh`, which passes them; the submodule
stays pinned and unmodified.

**After updating the `livox_ros_driver2` submodule, regenerate its
`package.xml`** — the repo ships only `package_ROS1.xml` / `package_ROS2.xml`,
`build.sh` is what copies one into place, and `package.xml` is in the submodule's
`.gitignore`, so a submodule update deletes it and every ament package in the
workspace fails to configure:

```bash
cp -f src/third-party/livox_ros_driver2/package_ROS2.xml \
      src/third-party/livox_ros_driver2/package.xml
```

`Livox-SDK2` deliberately has **no** `package.xml` — colcon picks it up as a
plain CMake package via `project(livox_sdk2)`. Adding one would turn it into an
ament package and require a build type and dependency list it does not have.

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
| `MAINTENANCE` = 0 | — | none | — |

`MAINTENANCE` is not a mode you switch *into*; it is what `get_mode` reports when
neither session exists. The live mode is never stored — it is derived on demand
from which session byobu actually has, so it stays correct across a `sys_manager`
restart and does not drift when someone builds or kills a session by hand.
`setup_session()` adopts a running session rather than rebuilding it, which is
what makes restarting `sys_manager` mid-mapping-run harmless.

Two services, namespaced under `robot_id`:

```bash
ros2 service call /<robot_id>/get_mode    syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode "{mode: 2}"
```

`switch_mode` kills *every* known session before building the target one (if both
were somehow up, leaving one behind would leave `get_mode` ambiguous), and
refuses to rebuild the mode that is already live — in `MANUAL` that would drop an
unsaved map on the floor, because `pgo_node` accumulates its keyframes in RAM and
`save_maps` is the only thing that serialises them. `sys_manager` itself runs
**outside** both specs — it is the robot container's main process
(`command:` in `docker-compose.robots.yml` runs `ros2 launch syncai_sys_manager
sys_manager.launch.py`), so `docker compose up -d robot01` brings the stack up
on its own: `setup_session()` sees nothing running and builds `AUTO`. Add
`sys_manager` back as a window in a spec and `kill_session` would kill the pane
it is running in.

The window list is **data**: `config/sessions/*.yaml` holds windows / panes /
commands / `sleep` offsets / `multilog` names, and `NodeManager` holds the byobu
plumbing (the two shell scripts it descends from shared ~85 lines of identical
bash). The schema is documented in the `NodeManager` docstring. `sleep` is where
the startup ordering lives, since there is no lifecycle manager: map_server / LIO
→ lio_bridge → planner/controller → task_runner. Sessions are built **detached**
— no `attach-session`, because the caller is a ROS node with no TTY.

The two specs are counterparts, not variants. `start_nav.yaml` *localizes*
against an existing map — map_server and the FAST-LIO2 localizer both load one
during construction and die without it — so it cannot build the map it needs;
`start_mapping.yaml` is the other half of that loop (bringup + pgo + teleop, and
deliberately no planner / controller / backend). Their logs go to separate
subtrees, which is what stops the two from interleaving two multilogs into one
directory. The 2D / AMCL session was retired along with `bringup_2d.launch.py`.

There is no log-reading helper. Read a subsystem back by hand — multilog writes
`current` plus gzipped rotations (16 MiB × 10):

```bash
tail -f log/stack/<robot_id>/planner/current
zcat log/stack/<robot_id>/planner/@*.s | less
```

One pane is **pre-typed but not executed** (`enter: false`): the
`set_motion_key` call in `start_mapping.yaml`'s teleop window. That window's next
pane publishes the namespaced `cmd_vel` that driver_manager consumes, so a stray
keypress there is a real motion command — which is why both specs set `select:`
to `bringup` rather than coming up focused on teleop.

Logging is deliberately split: `ROS_LOG_DIR` points at a tmpfs (ephemeral), while
the byobu `pipe-pane` capture is the persistent, gzip-rotated record.

## Backend architecture (`syncai_backend`)

Layered, and the layering is enforced by convention rather than tooling:

```
interfaces/rest/routers/  → gateways/  → repositories/  → database/
subscribers/  (ROS topics → repositories)
temporal/     (worker, workflows, activities)
```

- `RobotWorkflow` (Temporal) runs a task's steps in order, dispatching by
  `StepType` (`MOVE` → nav action, `ARTIFACT` → REST call to a conveyor). It
  exposes step state via a workflow query and supports cancellation.
- The Temporal task queue is scoped by `robot_id` so one robot's worker never
  picks up another's tasks.
- **Standing decision:** `ARTIFACT` activities call the artifact REST API
  directly. The behavior-tree route is reserved for a future need for tick-level
  parallelism.
- REST vocabulary uses **"vertex"** with a `VertexType` enum
  (`GENERAL`/`ARTIFACT`/`CHARGER`/`HOME`/`WAITING`); the DB model and repository
  still say `MapPoint`. This mismatch is intentional — no migration was done.
- Point clouds are cached in two single-slot repos: the live `body_cloud`
  (drained by a WebSocket stream) and the static localizer map cloud (served over
  REST). The wire format for the WS stream is `[u32 count][f32 xyz…]`, with the
  TF transform to `map` done server-side.
- **There are two pcd → gridmap recipes.** `POST /api/v1/maps` runs the
  **traversability** one: `helpers/traversable.py` segments the floor by lidar
  return intensity plus surface normal, repairs it (coplanar-fragment merge,
  interior hole fill, Delaunay stitching across seams and single steps), then
  `convert_traversable_to_gridmap` projects it and marks every cell the cloud
  does not cover as occupied. Note what that means: the output has **no unknown
  cells** — the padding ring and any floor the segmentation wrongly rejected all
  come out as wall. The older recipe, `convert_pcd_to_gridmap` (z-band slicing,
  trinary occupied/free/unknown), is what every gridmap on the fleet before
  2026-08 was built with; its dp1f bands survive as `GRIDMAP_RECIPE` in
  `routers/map.py` and it is now the **manual** route for a site the
  segmentation cannot handle. It is deliberately *not* an automatic fallback: a
  silent one would leave the catalogue holding maps from two recipes that
  disagree about what an unknown cell means, with nothing on disk saying which
  produced what.
- `helpers/traversable.py` is the only module that needs open3d, and **nothing
  imports it at module scope** — `_start_grid_conversion` imports it inside the
  conversion thread's `try`. Keep it that way, and keep `pcd_to_gridmap.py`
  open3d-free, or every backend start pays a ~100 MB import for a conversion
  that only runs when an operator saves a map. Inside the `try`, not merely
  inside the function: an `ImportError` raised above it escapes the thread and
  lands as a bare traceback that names no map.

Changing a backend ROS parameter requires restarting the backend.

## Frontend (`syncai_frontend`)

Next.js (dev server on port 3001), shadcn/ui components, **raw three.js** for
the 3D point-cloud view (no react-three-fiber). WebRTC streaming was considered
and deferred.

REST reads go through **TanStack Query**: one `QueryClient` in
`components/query-provider.tsx` (retry and focus-refetch are off on purpose —
the poll intervals are the retry policy), and every cache key lives in
`lib/api/query-keys.ts` so cache *sharing* between hooks is a decision visible
in one place (the gridmap editor and the dashboard read the same vertices
entry). The WebSocket streams (telemetry, point cloud) stay outside it — a push
stream has nothing to refetch; see `hooks/use-telemetry.ts`, and the point
cloud additionally bypasses React state entirely (20 Hz × few-hundred-KB frames
go straight into three.js buffers in `PointCloudCanvas`).

`src/syncai_frontend/AGENTS.md` warns that the pinned Next.js version has
breaking changes relative to model training data — read the relevant guide in
`node_modules/next/dist/docs/` before writing Next.js code.

## Infrastructure

- **`docker-compose.yml`** — shared services: `postgres` (5432, bind-mounted to
  `./data/postgres`), `pgadmin` (5050), `temporal` (7233), `temporal_ui` (8081),
  `mediamtx` (RTSP in / WebRTC out for the camera; `network_mode: host` so ICE
  candidates are the real interfaces — capture and Tegra H.264 encoding happen
  outside it and are pushed in over RTSP).
- **`docker-compose.robots.yml`** — robot containers. One service, `robot01`,
  behind the `real` profile (`network_mode: host`, loopback-only unicast DDS);
  the profile only keeps it opt-in, so bringing up the infra stack does not drag
  a robot up with it. The Isaac Sim fleet (`*-sim` services dual-homed on a
  `syncai-lan` macvlan) was **removed** — recover it from git history rather
  than re-deriving it if the simulator comes back. Robot containers also
  bind-mount the host D-Bus socket (so `nmcli` reaches the host NetworkManager —
  needs `apparmor=unconfined` + sudo) and the avahi socket (so `libnss-mdns`
  resolves `*.local`), and pass through `/dev/video0` plus the host `video` gid
  for the camera.
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
- Comments in this codebase explain **why**, often at length, and frequently
  record a past bug or a rejected alternative. Match that density when editing —
  a bare parameter change with no rationale is out of place here.
- `build/`, `install/`, `log/`, `data/`, `.env`, `record/`, and generated maps
  (`map/warehouse01/`, `map/dp2f*`) are gitignored. `.env` holds secrets — never
  commit it.
- `doc/` holds design notes on the FAST-LIO2 work (IESKF update, PGO pipeline,
  sync package, lidar recording).

## Tests

There is no meaningful test suite yet — only the ament linter tests that come
with the Python package templates (`syncai_sys_manager` also has
`test_wifi_manager.py`).

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
