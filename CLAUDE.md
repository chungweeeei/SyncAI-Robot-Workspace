# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A ROS 2 Humble software stack for the SyncAI robot (G23 quadruped / AMR chassis),
covering the full vertical: sensor drivers, LIO odometry, a **non-lifecycle port
of Navigation2**, and a Temporal-backed task orchestration backend whose REST /
WebSocket API is the contract the operator console (out of tree, see "Operator
console") is built against.

Two things shape almost every decision here:

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

`README.md` covers clone/build/run from a user's perspective. This file covers
the conventions and gotchas you need to edit the code correctly. Every package
under `src/` also carries its own `README.md` with its parameter tables, topic
names and package-local gotchas — read it before editing that package, and
update it in the same change when you move a parameter or a topic.

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
| `syncai_planner` | `ComputePathToPose` action server. Three pluginlib planners are built (NavFn, StraightLine, SmacPlanner2D); **SmacPlanner2D is the configured one**, with `cost_travel_multiplier: 1.0` (lowered from 2.0 — at 2.0 paths bowed along the inflation gradient in open space) and an explicit smoother block. |
| `syncai_controller` | `FollowPath` action server; Regulated Pure Pursuit merged in (clamps linear accel itself — there is no velocity smoother in the stack). `desired_linear_vel: 0.60` / `rotate_to_heading_angular_vel: 0.65` are one calibration with `syncai_driver_manager`'s velocity scales — change them together. |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`) |
| `syncai_task_runner` | The BT navigator. Serves `nav2_msgs/NavigateToPose`, hosts the `Navigator<ActionT>` abstraction and `behavior_trees/*.xml` (`move.xml` replans at 1 Hz). `bt_loop_duration: 50` ms ticks the tree at 20 Hz and doubles as every BT node's per-tick spin budget (halved), so it is a latency knob, not just a rate. There is no `syncai_bt_navigator` package. |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server. `costmap_filter_info` is launched by neither session spec — start it by hand when enabling the keepout filter. |

**Known drift:** the global costmap footprint (`planner_server_params.yaml`,
0.35 × 0.22 half-extents) and the local costmap footprint
(`controller_server_params.yaml`, 0.28 × 0.20) currently disagree. Reconcile
them before trusting RPP's collision rejections; the package READMEs flag it.

### Localization & sensing

| Package | Role |
|---|---|
| `syncai_lio_bridge` | **The only odometry source.** Wheel odom is retired. Converts the FAST-LIO2 chain (`map → lio_odom → lio_body`) into `odom → base_link` TF + `/<robot_id>/odom` + the AMCL-style `map → odom` correction, all projected to 2D (x, y, yaw) so the planar nav stack never sees a tilted frame. Angular velocity comes from the lidar IMU gyro because LIO leaves `twist.angular` empty. |
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` (carries the `lidar_top` mount extrinsic the LIO bridge needs) + the Livox driver. The fleet runs **both MID360 and MID360s**; the driver has no ROS parameter for the model, so `[sensor.lidar] type` (`mid360`/`mid360s`) picks the JSON schema. The driver's network JSON is **generated** per `robot_id` and model into `/tmp/syncai_bringup/` from `[sensor.lidar] ip` + `type` (INI) + `host_ip` (params YAML) — the vendor `MID360_config.json` / `MID360s_config.json` in the submodule's share dir is not read. The old 2D/AMCL `bringup_2d.launch.py` (laser scan merger) was removed. Optionally also the TechNexion VCS-AR0234-C camera via `vizionsdk_ros2` (`use_camera:=true`, **default off** — see below). |

**Camera.** The camera has two possible consumers and exactly one may hold the
V4L2 device at a time (the second opener does not fail at `open()`, it dies at
`S_FMT` with "Device or resource busy" long after the pane has scrolled). Today
the owner is the **host-side** `scripts/publish_camera_crop.sh` (GStreamer,
hardware crop/scale/H.264 on the Tegra, `rtspclientsink` to a remote MediaMTX),
which is why `bringup.launch.py` defaults `use_camera` to `false`. When the ROS
node is enabled it is image-only: `publish_imu` must stay `false` (the unit
answers `VxEnableIMUMode` with "not supported", and the node throws out of its
constructor then segfaults on teardown) and no `camera_info` is published (no
usable intrinsics). The camera is configured in `params/bringup.yaml` and
`scripts/publish_camera_crop.env`, not in the instance INI. Crop values in that
`.env` are per-robot (fisheye black-arc widths), and the stream path is
prefixed with the robot name so several robots can publish to one MediaMTX.

### Hardware & system

| Package | Role |
|---|---|
| `syncai_driver_manager` | **UDP bridge to the gait controller.** Sends `cmd_vel` with a per-direction velocity-scale correction (the gait controller tracks commands asymmetrically). The six scales (`scale_fwd` … `scale_turn_r`) are **ROS parameters** loaded from `params/driver_manager_params.yaml` (1.40 fwd / 1.40 turn today) and survive restarts; a runtime `set_speed_scale` override is what does *not* persist. The YAML records the two plateau runs behind them and flags them as a working correction, not a calibration. Receives ASCII telemetry, and owns the safe-shutdown path (`triggerSafeShutdown()`: safety lock + MODE X / lie down) — which still has **zero call sites**. |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motor_states / TF into `syncai_common/RobotState`. The code default is 10 Hz but the shipped params file sets `publish_rate: 1.0`, so it runs at **1 Hz**. Also derives the `state` field: `UNINITIALIZED` (no pose) / `WARNING` (battery <20%, cleared above 25% — latched with hysteresis) / `IDLE`. Reports only — no threshold here commands the robot. |
| `syncai_sys_manager` | Python. Five managers behind ROS services: wifi (`scan_wifi` / `connect_wifi`, `wifi_status` at 1 Hz from a cache refreshed every 5 s), mDNS (`avahi-publish <robot_id>.local`), conf (declares `robot_id`), monitor (host memory / disk to stdout at 1 Hz), and **node** (`NodeManager` — byobu session lifecycle, `switch_mode` / `get_mode`; see "Running the stack"). Also ships the host udev rules (`udev/99-syncai-devices.rules`). |

### Application layer

| Package | Role |
|---|---|
| `syncai_backend` | Python. FastAPI **and** rclpy in one process (`MultiThreadedExecutor`), port **3000**. Temporal worker for task orchestration. Also owns TTS (kokoro-onnx → `aplay`, weights in `models/kokoro/`), task templates + schedules, the wifi bridge to `sys_manager`, and the teleop / telemetry / point-cloud WebSockets. Declares **no** ROS parameters. **The only thing the operator console talks to** — see "Operator console" below. |

`syncai_frontend` (the Next.js operator console, dev server on 3001) **was
removed from this workspace in 2026-09** and lives in its own repository; it
was never an ament package, so nothing in the ROS build changed. The port
`3001` no longer belongs to anything here.

Every `src/syncai_*` package is in-tree; everything under `src/third-party/` is a
submodule.

`syncai_ros_mcp` — a vendored MCP server that exposed the ROS 2 graph and the
backend's REST API as MCP tools over HTTP on port 8000 — **was removed**, because
this version has no use for it: it was in no session spec and not in compose, so
nothing ever started it, and nothing in the stack imported or called it. Recover
it from git history rather than re-deriving it if the agent work resumes. Its
`FastMCP` pip dependency went with it, which leaves Sophus / GTSAM (source builds
for `FASTLIO2_ROS2`) as the only manual dependency `rosdep` does not cover.

### Third-party (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | Submodule, pinned to upstream tag `3.8.8`. Unmodified. |
| `FASTLIO2_ROS2` | Submodule → `chungweeeei/SyncAI-Fast-LIO2` (branch `dev`). Contains LIO + PGO + HBA + `localizer`. |
| `livox_ros_driver2`, `Livox-SDK2` | Submodules (MID360 / MID360s driver) |
| `small_gicp` | Submodule, pinned to upstream tag `v1.0.1`. Unmodified. The `localizer`'s registration backend (`RegistrationPCL`, a `pcl::Registration` subclass). Ships its own `package.xml` with `<build_type>cmake</build_type>`, so colcon builds it as a plain CMake package and `localizer` finds it with `find_package(small_gicp)`; the ordering comes from `<depend>small_gicp</depend>` in the localizer's manifest. |
| `vizionsdk-ros2` | Submodule → `TechNexion-Vision/vizionsdk-ros2` (branch `main`). ROS 2 wrapper (`vizionsdk_ros2/vizionsdk_camera_node`) for the TechNexion camera, started only by `bringup.launch.py use_camera:=true`. Needs the closed-source VizionSDK `.deb`, which the `Dockerfile` downloads from the TechNexion GitHub release (`VIZIONSDK_VERSION`); there is no rosdep key for it. |

All six third-party packages are submodules; the last vendored one,
`ros2_laser_scan_merger`, went away with the 2D/AMCL path (commit `99141a6`).

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
which is what the backend's `set_active_map` writes, and why it rewrites that
one line in place rather than round-tripping the file through configparser),
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
The Python packages (`syncai_backend`, `syncai_sys_manager`) instead default to
the absolute `~/robot_ws/config/system.ini` (`SYNCAI_SYSTEM_INI` /
`system_config:=`), so they do not depend on the cwd.

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
docker compose up -d            # infra + robot01 (docker-compose.yml `include`s the robots file)
docker compose exec robot01 bash
# inside, workspace is at /home/syncrobotic/robot_ws and is the cwd
colcon build --symlink-install
colcon build --packages-select syncai_planner
source install/setup.bash

# or from the host, in a throwaway container made from the same image:
docker compose -f docker-compose.build.yaml run --rm build [colcon args...]
```

`docker-compose.build.yaml` is **standalone** (own project `name:
syncai-build`, not `include`d — a build is not something to `up`) and its one
service runs `scripts/build.sh` as its *entrypoint*, which is why
`run build --packages-select x` appends to `colcon build --symlink-install`
instead of replacing it. Same image as robot01 on purpose: the artefacts land
in the shared bind-mounted `build/` + `install/`, so the toolchain and headers
must be what robot01 has. It therefore only `rosdep check`s by default
(`BUILD_ROSDEP=check`) — an apt install into a throwaway container never
reaches robot01, so a missing dep is a `Dockerfile` change. The script also
restores the `livox_ros_driver2` `package.xml` when a submodule update has
deleted it and refuses to start on empty submodule dirs. It builds **ROS
packages only**, which since the frontend left the workspace is everything
under `src/`. It is equally runnable inside robot01 (`scripts/build.sh`). Its
`build:` block duplicates `x-robot-common` (compose `extends` would drag the
devices / nvidia runtime / X11 mounts along) — keep the two in sync.

Recreating a robot container wipes hand-installed build dependencies (the ones
not in the image). Re-run `rosdep install --from-paths src --ignore-src -r -y`
plus any manual deps (Sophus / GTSAM are built from source for
`FASTLIO2_ROS2`).

The `Dockerfile` is multi-stage: `base` (ros-base + cyclonedds + uid-1000 user)
→ `deps-builder` (GTSAM / Sophus / Livox-SDK2 into `/usr/local`, the slow stage
— keep it free of anything that changes often so its cache survives) → `dev`
(rviz2, colcon, byobu, Node.js, the VizionSDK `.deb`; the workspace is
bind-mounted at `~/robot_ws` and built by hand). Compose builds `target: dev`.
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
`save_maps` is the only thing that serialises them. That refusal is why
**starting a new map is not a mode switch**: `POST /api/v1/mapping/reset` →
`pgo/reset_mapping` does it in place instead, with nothing restarted (see the
backend section). `sys_manager` itself runs
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
nav session's windows, in order: `bringup` → `localization` (map_server +
localizer) → `lio_bridge` → `plan_ctrl` (planner + controller) → `task_runner`
→ `driver_manager` → `backend` (robot_state + backend). There is no `frontend`
window in either spec any more — the console is served from outside the
workspace, so a mode switch no longer interrupts it (it used to be a pane of
the session being killed, which is why each spec carried its own dev server).
Sessions are built **detached** — no `attach-session`, because the caller is a
ROS node with no TTY.

The two specs are counterparts, not variants. `start_nav.yaml` *localizes*
against an existing map — map_server and the FAST-LIO2 localizer both load one
during construction and die without it — so it cannot build the map it needs;
`start_mapping.yaml` is the other half of that loop: `bringup` → `lio` (`pgo`)
→ `driver_manager` → `backend` (robot_state + backend), and deliberately **no**
map_server / localizer / lio_bridge / planner / controller / task_runner / hba.
The backend *is* in the mapping session because the operator console is the
mapping UI: mode switch, teleop over WebSocket, the live `pgo/map_cloud` stream
and save-map all go through the backend. Their logs
go to separate subtrees, which is what stops the two from interleaving two
multilogs into one directory. The 2D / AMCL session was retired along with
`bringup_2d.launch.py`. Neither spec has an rviz2 window (the robot has no
display; `config/rviz2/<robot_id>.rviz` is for running rviz2 from a
workstation) or a camera window (the RTSP publisher runs on the host).

There is no log-reading helper. Read a subsystem back by hand — multilog writes
`current` plus gzipped rotations (16 MiB × 10):

```bash
tail -f log/stack/<robot_id>/planner/current
zcat log/stack/<robot_id>/planner/@*.s | less
```

There is no keyboard-teleop window any more: the console's teleop channel
(backend WebSocket `/api/v1/robot/teleop` → `cmd_vel`) covers it without a pane
where a stray keypress is a motion command. The schema still supports
`enter: false` (pre-type a command without executing it) but no spec uses it
today. Both specs set `select:` to `bringup` because its log is the first thing
to read when the robot comes up — it answers "is the lidar alive?".

Logging is deliberately split: `ROS_LOG_DIR` points at a tmpfs (ephemeral), while
the byobu `pipe-pane` capture is the persistent, gzip-rotated record.

## Backend architecture (`syncai_backend`)

Layered, and the layering is enforced by convention rather than tooling:

```
interfaces/rest/routers/  → gateways/  → repositories/  → database/
subscribers/  (ROS topics → repositories)
temporal/     (worker, workflows, activities)
```

- Routers (`interfaces/rest/routers/`): `task` (`/api/v1/tasks`,
  `/api/v1/active_tasks`), `schedule`, `task_template` (`/api/v1/task_templates`
  — templates replaced "saved tasks"; the prefix was chosen over
  `/api/v1/tasks/templates` deliberately, see `server.py`), `robot` (state,
  `POST /api/v1/robot/mode` → `sys_manager` `switch_mode`, initial pose, motion
  key, policy mode), `network` (wifi scan/connect → `sys_manager`), `map`
  (catalogue, save, rename, grid convert, vertices, image/thumbnail,
  pointcloud), `recording` (`/api/v1/recordings` — `ros2 bag record` as a
  supervised child, one at a time; see below), `tts`
  (`voices` / `synthesize` / `speak`), and the WebSockets `telemetry`,
  `pointcloud` (live + mapping map cloud) and `teleop`.
- `RobotWorkflow` (Temporal) runs a task's steps in order, dispatching by
  `StepType`: `MOVE` (nav action), `STANDUP`, `LIEDOWN`, `SPEAK` (TTS gateway;
  it blocks on `aplay` and **cannot heartbeat**, so it relies on
  `start_to_close` alone). It exposes step state via a workflow query and
  supports cancellation. `ARTIFACT` (conveyor pickup/drop over REST) was removed
  in 2026-08 together with the whole artifact gateway; old saved steps carrying
  it fail validation — purge them before deploying.
- The Temporal task queue is scoped by `robot_id` so one robot's worker never
  picks up another's tasks.
- TTS: one long-lived `TtsGateway` (kokoro-onnx, weights in `models/kokoro/`,
  ~330 MB, downloaded once — URLs in `gateways/tts/tts.py`) whose internal lock
  serialises a scheduled `SPEAK` step against a manual `POST /api/v1/tts/speak`.
  `SpeakParams` mirrors the router's request model on purpose: both drive the
  same gateway, so what one accepts the other must too.
- REST vocabulary uses **"vertex"** with a `VertexType` enum
  (`GENERAL`/`ARTIFACT`/`CHARGER`/`HOME`/`WAITING`); the DB model and repository
  still say `MapPoint`. This mismatch is intentional — no migration was done.
  Vertex routes are nested under `/api/v1/maps/{name}/vertices` and the request
  bodies carry **no** `map_name` — the URL owns the map, so a body cannot name a
  different one. Moving a vertex between maps is a delete and a create.
- **Map rename** (`PATCH /api/v1/maps/{name}`; in the console, the map card's
  inline Rename control) is one `os.rename` of `map/<old>/` plus two bulk `UPDATE`s:
  `MapRepo.move_vertices` re-keys `map_vertices.map` and
  `TaskTemplateRepo.rebind_map` re-keys `task_templates.map_name` — the
  directory name is a foreign key by convention in both tables, with no
  constraint behind it. The single-syscall rename rests on the property that
  **no file inside a map directory names the map or holds an absolute path**
  (`gridmap.yaml` says `image: gridmap.pgm`, `poses.txt` lists bare patch
  basenames); a future sidecar that embeds the path forces `rename_map_dir` to
  start rewriting it. Filesystem first, database second; a DB failure moves the
  directory back. Refusals, all before any mutation: 409 `map_active` for the
  map the stack is running on (map_server and the localizer opened its files at
  launch, so the directory cannot move under them — switch the robot to another
  map first, which is a live call now, not a stack restart), 409
  `conversion_running`, 409 `name_taken`. Temporal schedule memos keep the old
  `map_name` label on purpose (display-only, never re-registered).
- **Map switch** (`POST /api/v1/maps/{name}/activate`; in the console, the map
  card's top-left corner tile, swap arrows in the slot the in-use badge occupies on the
  map the robot is already on) is the verb that
  lifts `map_active` on both of the above, and the only thing in the workspace
  that writes the instance INI
  (`helpers/system_config.py`'s `set_active_map`; the module was read-only by
  design until this). A **live swap, no session restart**: `relocalize`
  takes a `pcd_path`, `map_server/load_map` re-reads a yaml, and the INI write is
  what makes the choice survive a restart. Three things have to agree — the
  localizer's cloud, map_server's grid, and `[map] name` — so the order is chosen
  around failure: the localizer goes first because its refusals happen before it
  mutates anything, and each later step compensates the earlier ones. Two
  non-obvious constraints. `relocalize` returning success is a **receipt, not a
  result** — registration runs async at 5 Hz with no deadline, and
  `relocalize_check` is the only surface that reports it (the backend
  is its first caller; `RobotState.localization_valid` is TF-presence only and
  reads true against a map the robot was never localized in). And `relocCB` takes
  the request's raw 6-DOF, bypassing the `applyPlanarGuess` tilt correction, so
  the swap **must** be followed by an `initialpose` publish or the tilted lidar
  mount freezes the localizer retrying a flat guess forever. `[initial_pose]` is
  zeroed with the switch and the operator re-seeds from the dashboard. Refusals,
  all before any mutation: 409 `grid_missing` / `pointcloud_missing` /
  `conversion_running` / `ini_not_writable` / `task_running` / `tasks_unknown`
  (Temporal unreachable — refuse rather than assume idle) / `stack_not_ready`,
  the last being how "the robot is in mapping mode" is detected, by service
  discoverability rather than the cached mode, which a robot that has lost
  localization does not have. Not guarded: a goal sent straight from the
  dashboard rather than dispatched as a task.
- **Map delete** (`DELETE /api/v1/maps/{name}`; in the console, the X in the
  map card's corner, behind an alert dialog) is `MapRepo.delete_vertices` then
  `MapCatalogRepo.delete_map_dir`'s `shutil.rmtree`, in that order — **the
  inverse of the rename, deliberately.** A rename puts the filesystem first
  because `os.rename` back is a real compensation; `rmtree` has none, so the
  irreversible step goes last, and the residue of the other ordering is worse:
  vertex rows outliving their directory are unreachable (every vertex route
  resolves the map first) until the next map saved under that name silently
  inherits them. Refusals are the rename's first two plus 409 `template_bound`,
  which exists because a rename can re-key `task_templates.map_name` and a
  delete cannot: a template naming a map that is gone will not dispatch, cannot
  be edited (`_require_map` rejects the map), and cannot have its `map_name`
  cleared if it holds MOVE steps — so the map stays and the operator is told
  which templates to unbind. Not covered, and a sentence in the response
  instead: a Temporal schedule registered from a since-deleted template keeps
  firing its frozen steps, and reaching those would mean handing the map router
  `workflow_gw` for a warning.
- Point clouds are cached in single-slot repos: the live `pointlio/body_cloud`
  (drained by WS `/api/v1/robot/pointcloud/stream`) and, during mapping, pgo's
  merged `pgo/map_cloud` (WS `/api/v1/robot/pointcloud/map/stream`). A saved
  map's `map.pcd` is served over REST (`GET /api/v1/maps/{name}/pointcloud`). The
  wire format for the WS streams is `[u32 count][f32 xyz…]`, with the TF
  transform to `map` done server-side. An **empty** `pgo/map_cloud` merge is a
  message, not a non-event — pgo sends one from `reset_mapping` — so
  `MapCloudSubscriber` clears its slot on it rather than skipping it. Clearing
  used to come free with the backend restarting on every mode switch; an
  in-place reset ended that.
- **Starting a new map: `POST /api/v1/mapping/reset` → `pgo/reset_mapping`.**
  Nothing is restarted. pgo orchestrates all of it so the caller makes one call
  and gets one verdict: pause intake, reset the LIO front end over
  `pointlio/reset`, rebuild `SimplePGO`, then drop everything stamped at or
  before the boundary the front end reported. Pausing *first* is the whole
  design — while pointlio's odometry jumps back to the origin pgo is not
  consuming, so the discontinuity has nowhere to land, and no sleep or slack
  window is involved. The only fallible step (the `ResetLIO` round trip) runs
  before anything is destroyed, so a failure leaves the pose graph untouched and
  says so; there is no half-reset. It is deliberately **not** a flag on `POST
  /api/v1/maps`: the common use is abandoning a run that went wrong in its first
  thirty seconds, and a combined route would force a throwaway map directory
  onto exactly that case. The route lives under `/api/v1/mapping/` because it
  touches no file. **The robot must be stationary** — pointlio re-runs a static,
  gravity-aligning IMU init, and one done in motion tilts the new map for its
  whole life with no error anywhere; that is said in the srv, the REST message
  and the console's confirm dialog, and nowhere enforced.
- **There are two pcd → gridmap recipes, and the default is z-band.**
  `POST /api/v1/maps` always converts with `convert_pcd_to_gridmap` (z-band
  slicing, trinary occupied/free/unknown), its bands recentred as offsets from
  the measured floor level, followed by a pose-connectivity filter: free cells
  not connected to the keyframe trajectory in `poses.txt` go back to unknown,
  which is what removes the free space a MID360 paints through glass (5–6% of
  all free cells on the conference maps, in 600–900 speckle components). The
  **traversability** recipe (`helpers/traversable.py`: segment the floor by
  intensity/normal/height, repair it, project it; **no unknown cells** — every
  cell the cloud does not cover comes out as permanent wall) runs only when an
  operator asks for it. There is deliberately **no automatic pick** between the
  two: the bbox-footprint threshold tried in 2026-08 misrouted all three
  conference-hall saves (glass let the lidar see out-of-hall structure, tripling
  the bbox over ~450 m² of real floor) into blobs, and any replacement metric
  would be calibrated on the current fleet's few clouds — while a wrong z-band
  map is recoverable (unknown is hand-editable and drivable-clearable) and a
  wrong traversability map is permanently walled. Every conversion writes
  `gridmap.recipe.json` beside the pgm recording the recipe, both area
  diagnostics and the parameters.
- **A conversion's outcome is disk state, and the catalogue is the only status
  surface.** The thread writes `gridmap.recipe.json` three times — `status:
  converting` before any work, then `ok` (recipe, params, diagnostics) or
  `failed` (the pipeline's `error` plus a `hint`) — and `MapCatalogRepo` reads
  it back so `GET /api/v1/maps` reports `grid_status` (`none` / `converting` /
  `ok` / `failed` / `interrupted`) and `grid_error` per map. There is no job
  resource and no status endpoint: a client that starts a conversion polls the
  catalogue, which is also what makes the answer survive a page reload. The
  record is on disk rather than in the process because it has to outlive both
  the thread and the backend — `switch_mode` tears down the byobu session the
  backend is a pane of, and `_ACTIVE_CONVERSIONS` dies with it; a sidecar left
  saying `converting` with nothing in that registry is what the router reports
  as `interrupted`. The registry still wins while the process is up.
  `grid_converting` is the deprecated boolean this replaced, kept for curl/MCP
  callers; under it a failed conversion was indistinguishable from a map nobody
  had converted and its reason existed only in the backend log.
- **`POST /api/v1/maps/{name}/grid/convert`** is the manual/override route
  (in the console, the map card's Rebuild-grid dropdown): pick the recipe, override
  `gap_fill_size` or the z-band offsets, and pass `debug: true` to get the
  segmentation's intermediate clouds in `<map>/traversable_debug/` — the tuning
  interface for a site the defaults cannot handle (the next outdoor venue). One
  conversion per map at a time (409 `conversion_running`); a hand-edited grid
  (gridmap_raw.pgm differing from gridmap.pgm) refuses with 409
  `gridmap_hand_edited` until `overwrite_edits` — and even then the edited grid
  survives as `gridmap_prev.pgm` (one undo generation, with its yaml, the stale
  raw moved to `gridmap_prev_raw.pgm` and the outgoing recipe record to
  `gridmap_prev.recipe.json`, since the incoming conversion overwrites the live
  sidecar before it does any work). Re-converting the active map reloads
  map_server on success. Note the grid is *copied* aside, not moved, so the
  active map never has a window with no file — which is why a failed
  re-conversion leaves a loadable map and `grid_status: failed` at the same
  time.
- **Bag recording is a subprocess, and the child deliberately shares this
  process's group.** `gateways/recording` spawns `ros2 bag record` (stdout
  inherited, never a pipe nobody drains) and `repositories/recording` is the
  `record/` catalogue, in `MapCatalogRepo`'s shape. Given its own session the
  recorder would outlive a `switch_mode` — which tears down the byobu session
  the backend is a pane of — and be unstoppable through any route, since the
  only handle on it is a slot in memory; sharing the group means the teardown
  takes it too and the half-written directory reports `interrupted`, derived
  the way a map's is. Stopping walks SIGINT → SIGTERM → SIGKILL, and only the
  first lets rosbag2 write `metadata.yaml`; without it the bag needs
  `ros2 bag reindex`. Relative topics in the request are expanded under
  `robot_id`, so a request body is fleet-portable, and nothing checks that a
  topic exists — the recorder waits for one, which is what lets it be armed
  before bringup and also what makes a typo a silent empty bag.
- `helpers/traversable.py` is the only module that needs open3d, and **nothing
  imports it at module scope** — `_start_grid_conversion` imports it inside the
  conversion thread's `try`. Keep it that way, and keep `pcd_to_gridmap.py`
  open3d-free, or every backend start pays a ~100 MB import for a conversion
  that only runs when an operator saves a map. Inside the `try`, not merely
  inside the function: an `ImportError` raised above it escapes the thread and
  lands as a bare traceback that names no map.

The backend declares no ROS parameters; its identity is the namespace it is
launched into plus the instance INI (`SYNCAI_SYSTEM_INI`). Its configuration
comes from the environment (`TEMPORAL_ADDRESS`, `POSTGRES_*`, loaded from the
workspace `.env` via python-dotenv), so changing any of it means restarting the
backend. `src/syncai_backend/test/` holds ~40 pytest files (routers, Temporal
workflows/activities, both gridmap recipes); the conversion tests need open3d
and scipy.

## Operator console (out of tree)

The Next.js operator console (`syncai_frontend`: dashboard with the 3D point
cloud, `/mapping`, `/maps` + the gridmap editor, `/recordings`, `/tasks`,
`/settings`) **was removed from this workspace in 2026-09** and is developed in
its own repository. What that means for this repo:

- **The backend's REST / WebSocket API on port 3000 is the contract.** Every
  console control maps to a route named in the backend section above; changing
  a route, a response shape or a 409 code is a cross-repo change now, not a
  same-commit edit, so say so in the commit message and keep the old shape
  working until the console has moved. `server.py` already serves
  `allow_origins=["*"]`, so a console on another host needs nothing from the
  robot beyond the port.
- **Nothing here serves port 3001 any more.** Neither session spec has a
  `frontend` window (they used to carry one each, so the console survived
  `switch_mode` killing the other session — moot for a console served
  elsewhere), the `Dockerfile`'s Node.js runtime is now only there for
  `scripts/urdf2glb.py`'s `npx gltfpack` step, and `scripts/build.sh` /
  `docker-compose.build.yaml` build every package under `src/`.
- **The robot mesh is still built from here.** `scripts/urdf2glb.py` bakes
  `src/syncai_bringup/description/G23.urdf` into a GLB the console loads as
  `public/models/g23.glb`; regenerate and hand it over after editing the URDF.
  Two invariants are load-bearing on the console side — GLB node names must
  equal URDF link names (the canvas looks links up by name to apply joint
  angles) and coordinates stay ROS Z-up.
- The "in the console, …" hints in the backend section describe the console's
  controls as they were when it left, kept because they say *which* route a
  UI verb hits. Treat them as a pointer to that repo, not as something to
  verify here.

## Infrastructure

- **`docker-compose.yml`** — shared services: `postgres` (5432, bind-mounted to
  `./data/postgres`), `pgadmin` (5050), `temporal` (7233), `temporal_ui` (8081).
  There is **no** `mediamtx` service any more: the camera is pushed over RTSP by
  the **host-side** `scripts/publish_camera_crop.sh` (`start` / `stop` /
  `restart` / `status` / `logs` / `foreground`; reads `/dev/syncai/camera0`) to
  a MediaMTX that already runs on a remote server (`MEDIAMTX_RTSP_HOST` in
  `scripts/publish_camera_crop.env` — deliberately not the compose `.env`, which
  holds secrets the script would hand to gst-launch's environment). The robot is
  a publisher only; nothing in compose or the session specs starts the script.
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
  needs `apparmor=unconfined` + sudo) and the avahi socket (so `libnss-mdns`
  resolves `*.local`), run with `runtime: nvidia`, and pass through the cameras
  as `/dev/syncai/camera0` / `camera1` (stable udev symlinks from
  `src/syncai_sys_manager/udev/99-syncai-devices.rules`, keyed on serial so the
  two cameras cannot swap on reboot) plus `/dev/snd` for TTS, with the host
  `video` and `audio` gids added (`VIDEO_GID` / `AUDIO_GID`, default 44 / 29,
  overridable in `.env`) and `device_cgroup_rules` for the video4linux (81) and
  ALSA (116) majors so a USB replug onto a new minor does not need a container
  recreate. X11 is passed through (`/tmp/.X11-unix` rw, `/tmp/.docker.xauth`
  ro, `DISPLAY=:1`; the file header has the xauth-cookie recipe). Because of
  host networking, `TEMPORAL_ADDRESS` and `POSTGRES_HOST` are `127.0.0.1`, not
  compose service names. `ROS_LOG_DIR` points at a 128 MiB tmpfs.
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
  governs both Python packages; it exists because a ruff upgrade's growing
  defaults produced 471 spurious in-editor warnings overnight).
- Comments and docs are written in **English**. Comments in this codebase
  explain **why**, often at length, and frequently record a past bug or a
  rejected alternative. Match that density when editing — a bare parameter
  change with no rationale is out of place here. The last Chinese remnants
  (BT plugin comments, `ExecuteTask.action`, the camera script's log strings,
  the `doc/` proposals) were translated in 2026-09; anything new in another
  language is a regression, log strings included. The one exception is a
  `*.zh-TW.md` **translation** sitting beside an English original that stays the
  canonical copy and carries the content — `doc/webrtc-worker-proposal.zh-TW.md`
  is the only one today. Edit the English file first; a zh-TW file that has
  drifted is worse than none, so either update both or delete the translation.
- `build/`, `install/`, `log/`, `data/`, `.env`, `record/` (hand-recorded
  rosbags), the whole of `/map/` (LIO output: `map.pcd`, `poses.txt`,
  `patches/`, generated `gridmap.*`) and `/models/` (TTS weights) are
  gitignored. `.env` holds secrets — never commit it.
- `scripts/`: `attach.sh` (host-side: attaches to whichever byobu session is
  live in the container — the session name follows the mode, so a hardcoded
  alias is wrong half the time), `build.sh` (in-image: the build
  steps behind `docker-compose.build.yaml`, see Build), `publish_camera_crop.sh` + `.env` (host-side
  camera publisher, see Infrastructure), `urdf2glb.py` (robot mesh for the
  console's 3D view, see "Operator console"), and `release/` (`build_images.sh` / `save_images.sh` /
  `load_and_up.sh` / `.env.example` — an offline release bundle for the
  customer IPC that is currently **non-functional**: it needs the removed
  production Dockerfile stages and a `docker-compose.prod.yml` that is not in
  the tree, and its `.env.example` says `ROS_DOMAIN_ID=0` while the stack pins
  domain 1). `skills-lock.json` at the root is Claude Code tooling metadata,
  not stack config.
- `doc/` holds six design **proposals** (none implemented). Five are on agent /
  MCP integration: deep-agent wiring, a gridmap-tuning agent, MCP server design,
  RoboNeuron mechanisms, and a task-recovery loop. Three of those target
  `src/syncai_device_agent/`, which was removed in commit `99141a6`, and four
assume `src/syncai_ros_mcp/`, removed later — both are in git history. The
sixth,
  `webrtc-worker-proposal.md`, is unrelated to the other five: it covers the
  camera path (a self-built Go + pion WHEP worker in `src/syncai_webrtc/` that
  owns the capture/crop/encode pipeline as a supervised `gst-launch-1.0` child
  and relays its RTP to browsers; it deliberately ignores the host-side
  `publish_camera_crop.sh` path) and records why the "Go `.so` + zero-copy into
  Python" framing it came from was not adopted. The
  FAST-LIO2 design notes that used to live here (`fastlio2-pgo-pipeline.md`;
  `config/sessions/start_mapping.yaml` still cites its §3.5 / §5 by section
  number, noting that the file is gone) are no longer in the tree — check git
  history. `webrtc-worker-proposal.zh-TW.md` is a translation of the sixth, not
  a seventh proposal; see the English-docs convention above.

## Tests

`syncai_backend` has a substantial pytest suite (~40 files under
`src/syncai_backend/test/`: every REST router, the Temporal workflow /
activities / worker, both pcd → gridmap recipes, the map catalogue, TF and the
telemetry / point-cloud streams). `syncai_sys_manager` has
`test_wifi_manager.py`. The C++ packages have only the ament linter tests that
come with the package templates.

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
# backend only, from src/syncai_backend (open3d + scipy needed for the conversion tests)
python3 -m pytest test/
```
