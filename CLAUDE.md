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
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` (carries the `lidar_top` mount extrinsic the LIO bridge needs) + the Livox driver. The fleet runs **both MID360 and MID360s**; the driver has no ROS parameter for the model, so `[sensor.lidar] type` (`mid360`/`mid360s`) picks the JSON schema. The driver's network JSON is **generated** per `robot_id` and model into `/tmp/syncai_bringup/` from `[sensor.lidar] ip` + `type` (INI) + `host_ip` (params YAML) — the vendor `MID360_config.json` / `MID360s_config.json` in the driver's share dir is not read. The old 2D/AMCL `bringup_2d.launch.py` (laser scan merger) was removed. Optionally also the TechNexion VCS-AR0234-C camera via `vizionsdk_ros2` (`use_camera:=true`, **default off** — see below). |

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
`src/third-party/`. One of those packages is not tracked here — `syncai_common`
moved to `chungweeeei/SyncAI-Robot-Interface` in the same split, so the backend can build
against the message definitions without checking out this workspace, and is
materialised back into `src/` by `vcs import < interface.repos`. Edit the
messages there; a change made in that directory is untracked, and the next
`--force` import overwrites it.

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
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`. Unmodified. |
| `FASTLIO2_ROS2` | `chungweeeei/SyncAI-Fast-LIO2` (branch `dev`, SSH remote). Contains LIO + PGO + HBA + `localizer`. |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver |
| `small_gicp` | Pinned to upstream tag `v1.0.1`. Unmodified. The `localizer`'s registration backend (`RegistrationPCL`, a `pcl::Registration` subclass). Ships its own `package.xml` with `<build_type>cmake</build_type>`, so colcon builds it as a plain CMake package and `localizer` finds it with `find_package(small_gicp)`; the ordering comes from `<depend>small_gicp</depend>` in the localizer's manifest. |
| `vizionsdk-ros2` | `TechNexion-Vision/vizionsdk-ros2` (branch `main`). ROS 2 wrapper (`vizionsdk_ros2/vizionsdk_camera_node`) for the TechNexion camera, started only by `bringup.launch.py use_camera:=true`. Needs the closed-source VizionSDK `.deb`, which the `Dockerfile` downloads from the TechNexion GitHub release (`VIZIONSDK_VERSION`); there is no rosdep key for it. |

All six are checked out by vcstool from `third-party.repos` — they were git
submodules until `fca520b`, and `.gitmodules` is gone; bumping one is a
one-line `version:` edit there plus `vcs import < third-party.repos --force`,
not the old two-commit dance. The import target is the workspace root, not
`src`: the keys already carry `src/`. The last vendored package,
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
restores the `livox_ros_driver2` `package.xml` when a `vcs import` has deleted
it, and refuses to start on an empty vcs checkout — the six `src/third-party/`
dirs or `src/syncai_common`, naming the `.repos` file to import per missing
directory, because "you forgot to import" is not what colcon's own failure
looks like. There are no carve-outs in what it builds any more: since the
frontend and then the backend left, the workspace and what colcon discovers are
the same set. It is equally runnable inside robot01 (`scripts/build.sh`). Its
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
**starting a new map is not a mode switch**: `pgo/reset_mapping` (the backend's
`POST /api/v1/mapping/reset` is one caller) does it in place instead, with
nothing restarted — see "Out of tree". `sys_manager` itself runs
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
→ `driver_manager` → `robot_state`. Neither spec has a `frontend` or a
`backend` window any more — both are served from their own containers, so a
mode switch interrupts neither. That is a real change and not only a tidy-up:
each used to be a pane of the session being killed, which is why every spec
once carried its own dev server, and why a conversion or a recording the
backend was supervising died with a `switch_mode`. Sessions are built
**detached** — no `attach-session`, because the caller is a ROS node with no
TTY.

The two specs are counterparts, not variants. `start_nav.yaml` *localizes*
against an existing map — map_server and the FAST-LIO2 localizer both load one
during construction and die without it — so it cannot build the map it needs;
`start_mapping.yaml` is the other half of that loop: `bringup` → `lio` (`pgo`)
→ `driver_manager` → `robot_state`, and deliberately **no** map_server /
localizer / lio_bridge / planner / controller / task_runner / hba. `robot_state`
is in the mapping session although its pose lookup never succeeds there, for its
`mode` field alone — that is what the console's mode chip reads to know the
switch to `MANUAL` landed. Everything else the console does during a mapping run
(teleop, the live `pgo/map_cloud` stream, save-map) reaches these nodes over DDS
from the backend's container. Their logs go to separate subtrees, which is what
stops the two specs from interleaving two multilogs into one directory. The 2D / AMCL session was retired along with
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
| `syncai_common` msg/srv/action (`SyncAI-Robot-Interface`) | every call below — it builds against that repo, not this checkout |
| `robot_state` topic (`syncai_robot_state`) | the telemetry WebSocket and `GET /api/v1/robot/state` |
| `switch_mode` / `get_mode` (`syncai_sys_manager`) | `POST /api/v1/robot/mode`; its gateway waits 45 s / 70 s for them |
| `scan_wifi` / `connect_wifi` / `wifi_status` (`syncai_sys_manager`) | the network router |
| `cmd_vel`, `set_motion_key` (`syncai_driver_manager`) | teleop WebSocket, STANDUP / LIEDOWN steps |
| `NavigateToPose` on `task_runner` | the MOVE step of a task |
| `localizer/relocalize`, `relocalize_check`, `initialpose` | initial pose, and map switch |
| `map_server/load_map` | map switch, and reload after a re-conversion |
| `pgo/save_maps`, `pgo/reset_mapping`, `pgo/map_cloud` | save a map, start a new one, stream the live merge |
| `pointlio/body_cloud` | the live cloud WebSocket |
| `config/instances/robotNN.ini` (`[map] name`, `[initial_pose]`) | the only file in this repo the backend **writes** |
| `map/<name>/` on disk | the map catalogue: `map.pcd`, `poses.txt`, `patches/`, `gridmap.*` |

**Facts about this stack the backend depends on.** These belong here because
they are properties of the nodes in `src/` and `src/third-party/`, not of the
backend:

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
- **`map_server` and the localizer open their files during construction** and
  die without them. That is why the nav session cannot build the map it needs,
  and why the active map's directory cannot be renamed or deleted under them.
- **No file inside a map directory names the map or holds an absolute path** —
  `gridmap.yaml` says `image: gridmap.pgm`, `poses.txt` lists bare patch
  basenames. That property is what makes a rename one `os.rename`; a future
  sidecar written by anything here that embeds the path breaks it silently.
- **`pgo/reset_mapping` is how a new map is started**, not a mode switch:
  `switch_mode` refuses to rebuild the live mode, and rebuilding `MANUAL` would
  drop an unsaved map, since `pgo_node` accumulates keyframes in RAM. The reset
  pauses intake, resets the LIO front end over `pointlio/reset`, rebuilds the
  pose graph and drops everything at or before the boundary the front end
  reported — pausing first is the whole design, so the odometry discontinuity
  has nowhere to land. The only fallible step runs before anything is
  destroyed: there is no half-reset. **The robot must be stationary**, because
  pointlio re-runs a static, gravity-aligning IMU init and one done in motion
  tilts the new map for its whole life with no error anywhere. Nothing enforces
  that, in either repo.
- **An empty `pgo/map_cloud` merge is a message, not a non-event** — pgo sends
  one from `reset_mapping`, and a consumer that skips empty clouds keeps showing
  the old map.
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
  service any more: the camera is pushed over RTSP by
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
  `patches/`, generated `gridmap.*`) and `/models/` (TTS weights, which nothing
  here downloads any more) are gitignored, as are `src/syncai_common/` and
  `src/syncai_backend/` — the first because it is materialised by `vcs import`,
  the second so a clone of the backend kept there for development can never be
  committed back in. `.env` holds secrets — never commit it.
- `scripts/`: `attach.sh` (host-side: attaches to whichever byobu session is
  live in the container — the session name follows the mode, so a hardcoded
  alias is wrong half the time), `build.sh` (in-image: the build
  steps behind `docker-compose.build.yaml`, see Build), `publish_camera_crop.sh` + `.env` (host-side
  camera publisher, see Infrastructure), `urdf2glb.py` (robot mesh for the
  console's 3D view, see "Out of tree"), and `release/` (`build_images.sh` / `save_images.sh` /
  `load_and_up.sh` / `.env.example` — an offline release bundle for the
  customer IPC that is currently **non-functional**: it needs the removed
  production Dockerfile stages and a `docker-compose.prod.yml` that is not in
  the tree, and its `.env.example` says `ROS_DOMAIN_ID=0` while the stack pins
  domain 1). `skills-lock.json` at the root is Claude Code tooling metadata,
  not stack config.
- `doc/` holds five design **proposals**, none implemented, and all five are on
  agent / MCP integration: deep-agent wiring, a gridmap-tuning agent, MCP server
  design, RoboNeuron mechanisms, and a task-recovery loop. Read them as history,
  not as plans — three target `src/syncai_device_agent/`, removed in `99141a6`,
  and four assume `src/syncai_ros_mcp/`, removed later; both are in git history.
  Two things that used to be in this directory are not:
  `webrtc-worker-proposal.md` and its zh-TW translation (a self-built Go + pion
  WHEP worker owning the capture/crop/encode pipeline, deliberately ignoring the
  host-side `publish_camera_crop.sh` path) were deleted in `3dd5f7c`, and that
  work now lives in its own repository, `chungweeeei/SyncAI-WebRTC-Worker`; and
  the FAST-LIO2 design notes (`fastlio2-pgo-pipeline.md`, which
  `config/sessions/start_mapping.yaml` still cites by section number, noting the
  file is gone). Check git history for either.

## Tests

`syncai_sys_manager` has `test_wifi_manager.py`; the C++ packages have only the
ament linter tests that come with the package templates. That is the whole
suite in this repo. The substantial one — ~40 pytest files covering every REST
router, the Temporal workflow / activities / worker, both pcd → gridmap recipes,
the map catalogue, TF and the telemetry / point-cloud streams — went with
`syncai_backend` in 2026-09 and runs in that repo, against the interfaces from
`SyncAI-Robot-Interface`.

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
