# SyncAI Robot Workspace

A ROS 2 Humble software stack for the SyncAI robot (G23 quadruped / AMR
chassis): Livox lidar + camera drivers, FAST-LIO2 odometry and localization, a
**non-lifecycle port of Navigation2**, and the byobu session manager that brings
all of it up. The two operator-facing halves each live in their own repository
now — `chungweeeei/SyncAI-Robot-Backend` (the Temporal-backed task
orchestration API on port 3000, split out in 2026-09) and
`chungweeeei/SyncAI-Robot-Frontend`, the Next.js console it serves (split out
earlier the same month). This workspace is the robot's ROS 2 side of that line.

The nav2 servers (map server, costmap, planner, controller, BT navigator) were
re-implemented as plain `rclcpp::Node`s instead of lifecycle nodes, so the stack
starts and runs without a lifecycle manager; startup ordering is handled by the
byobu session specs instead. Navigation is driven by a Behavior Tree.

> Built and run inside a Docker container (`ubuntu:22.04` + ROS 2 Humble) on a
> Jetson. RMW is CycloneDDS configured via `config/cyclonedds.xml`.
>
> `CLAUDE.md` holds the conventions and gotchas needed to edit the code (the
> `robot_id` namespacing rules, the session specs, the build). Every
> package under `src/` has its own `README.md` with parameter tables and topic
> names. This file is the user-level clone / build / run guide.

## Architecture

```
                         NavigateToPose (nav2_msgs)
   RobotWorkflow ─────────────────────────────────▶  syncai_task_runner   (BT navigator; ticks behavior_trees/move.xml)
   (SyncAI-Robot-Backend, out of tree)                       │
                                          compute_path_to_pose│follow_path
                                           ┌─────────────────┴─────────────────┐
                                           ▼                                   ▼
                                    syncai_planner                      syncai_controller
                                   (SmacPlanner2D)                   (Regulated Pure Pursuit)
                                           │                                   │
                                           └───────── syncai_costmap_2d ───────┘
                                                (global / local costmaps)          cmd_vel ──▶ syncai_driver_manager ──UDP──▶ gait controller

   livox_ros_driver2 ──▶ FAST-LIO2 (pointlio / localizer) ──▶ syncai_lio_bridge ──▶ odom → base_link TF, map → odom, /odom
   syncai_map_server ──▶ static gridmap (map)
```

- **Localization is 3D.** `syncai_lio_bridge` is the only odometry source: it
  projects the FAST-LIO2 chain to 2D and publishes the `odom → base_link` TF,
  `/<robot_id>/odom`, and the `map → odom` correction. There is no AMCL and no
  laser scan in the stack any more.
- **Everything is namespaced by `robot_id`** (read from `config/system.ini`),
  so several robots can share one DDS domain.

## Packages (`src/`)

| Package | Role |
|---|---|
| `syncai_common` | Shared msg / srv / action interfaces (`RobotState`, `SwitchMode`, `SetMotionKey`, `ExecuteTask`, …) |
| `syncai_util` | Helpers (geometry, odometry window, simple action server, robot utils) |
| `syncai_nav_core` | Header-only abstract interfaces for nav plugins (port of `nav2_core`) |
| `syncai_costmap_2d` | Global / local costmaps with layered plugins (static / obstacle / inflation / keepout filter) |
| `syncai_planner` | `compute_path_to_pose` action server; NavFn, StraightLine and SmacPlanner2D plugins (Smac is configured) |
| `syncai_controller` | `follow_path` action server; Regulated Pure Pursuit plugin, with its own linear-accel clamp (no velocity smoother) |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`) |
| `syncai_task_runner` | BT navigator: serves `NavigateToPose`, ticks `behavior_trees/move.xml` |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server |
| `syncai_lio_bridge` | FAST-LIO2 → planar `odom` / TF bridge (the only odometry source) |
| `syncai_bringup` | `robot_state_publisher` over `description/G23.urdf`, the Livox MID360 / MID360s driver (config JSON generated per robot), optional TechNexion camera node |
| `syncai_driver_manager` | UDP bridge to the gait controller: `cmd_vel` out (with per-direction velocity scales), telemetry in, safety lock |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motors / TF into `syncai_common/RobotState` |
| `syncai_sys_manager` | Python. Wifi, mDNS, host monitoring, and the **byobu session manager** (`switch_mode` / `get_mode`) — the robot container's main process |

Two packages that used to be in this table are not any more, and both moved in
2026-09:

| Was | Now | What it is |
|---|---|---|
| `syncai_frontend` | `SyncAI-Robot-Frontend` | The Next.js operator console, formerly served from port 3001. Nothing in this workspace serves it. The robot mesh it renders is still baked here, by `scripts/urdf2glb.py`. |
| `syncai_backend` | `SyncAI-Robot-Backend` | FastAPI + rclpy in one process on port **3000**: Temporal worker (tasks, templates, schedules), map catalogue + pcd → gridmap conversion, TTS, WebSocket streams. The API the console is built on, and still the only thing it talks to. |

Neither is imported, built or launched from here. The backend runs in its own
container with host networking on DDS domain 1, so it discovers the nodes above
exactly as it did when it was a pane of the byobu session — and `switch_mode` no
longer takes it down. The infra it needs (postgres, temporal) is still in this
repo's `docker-compose.yml`, because the robot is where it is deployed.

`syncai_common` is not in this repo either: it moved to `SyncAI-Robot-Interface`
in the same split, so the backend can build against the message definitions
without checking out the whole workspace. It is imported back into `src/` (see
"Getting started"), because every package here depends on it.

### Third-party (`src/third-party/`)

All six are checked out by vcstool from `third-party.repos` (they were git
submodules until `fca520b`); nothing is vendored there any more.

| Package | Notes |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`, unmodified |
| `FASTLIO2_ROS2` | `chungweeeei/SyncAI-Fast-LIO2`, branch `dev` (**SSH remote** — a recursive clone needs a GitHub key). LIO + PGO + HBA + `localizer` |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver. `colcon.meta` passes the cmake flags the driver needs; see "Build" |
| `small_gicp` | Pinned to `v1.0.1`, unmodified; the localizer's registration backend |
| `vizionsdk-ros2` | TechNexion camera wrapper; needs the VizionSDK `.deb` the `Dockerfile` installs |

To bump one, edit its `version:` in `third-party.repos`, commit that one-line
diff, and re-import:

```bash
vcs import < third-party.repos --force
```

`third-party.repos` has the rest of the rules in its header (no `--shallow`,
and regenerate the livox `package.xml` after every import — see "Build").

## Repository layout

```
.
├── src/                          # colcon packages (see table above)
│   └── third-party/              # six upstream repos, checked out by vcstool
├── config/
│   ├── system.ini                # tracked but EMPTY; the instance INI is bind-mounted over it
│   ├── instances/robot01.ini     # per-robot identity: [system] robot_id, [map], [initial_pose], [sensor.lidar]
│   ├── sessions/                 # byobu session specs: start_nav.yaml (AUTO), start_mapping.yaml (MANUAL)
│   ├── cyclonedds.xml            # CycloneDDS: loopback-only unicast, Domain Id 1
│   └── rviz2/robot01.rviz        # for running rviz2 from a workstation (no display on the robot)
├── scripts/
│   ├── attach.sh                 # host-side: attach to whichever byobu session is live in the container
│   ├── build.sh                  # in-image: colcon build of every ROS package; entrypoint of docker-compose.build.yaml
│   ├── publish_camera_crop.sh    # host-side camera → RTSP publisher (GStreamer → remote MediaMTX)
│   ├── publish_camera_crop.env   # its per-robot settings (crop, stream path, MediaMTX host)
│   ├── urdf2glb.py               # bakes G23.urdf into the GLB the operator console renders (handed over to its repo)
│   └── release/                  # offline release bundle for the IPC (currently non-functional, see CLAUDE.md)
├── doc/                          # design proposals (agent / MCP integration; not implemented)
├── map/                          # LIO map output per map name (map.pcd, poses.txt, gridmap.*) — gitignored
├── log/stack/<robot_id>/         # multilog capture of every byobu pane — gitignored
├── Dockerfile                    # multi-stage: base → deps-builder (GTSAM/Sophus/Livox-SDK2) → dev
├── docker-compose.yml            # infra: postgres (5432) / pgadmin (5050) / temporal (7233) / temporal_ui (8081)
├── docker-compose.robots.yml     # robot01 (host networking, nvidia runtime, cameras, audio, D-Bus, avahi); `include`d above
├── docker-compose.build.yaml     # standalone one-shot `colcon build` service (same image, own project name)
├── colcon.meta                   # per-package cmake args (livox_ros_driver2)
├── ruff.toml                     # Python lint config (syncai_sys_manager, scripts/)
├── third-party.repos             # vcstool: src/third-party/ — upstream, pinned
├── interface.repos               # vcstool: src/syncai_common — SyncAI-Robot-Interface
├── .devcontainer/                # VS Code "Reopen in Container"
└── .env                          # compose env + secrets (gitignored — never commit)
```

## Getting started

### 1. VCS import the source repos

```bash
sudo apt update && sudo apt install python3-vcstool -y
vcs import < third-party.repos     # src/third-party/ — upstream code, pinned
vcs import < interface.repos       # src/syncai_common — our own wire format
```

Both are required before the first `colcon build`: `src/syncai_common` is no
longer tracked here (it moved to `SyncAI-Robot-Interface` in 2026-09, so the
backend can build against the message definitions without the whole workspace)
and every package in this repo depends on it.

There is no third import. `src/syncai_backend` was the operator-facing process
and moved to `SyncAI-Robot-Backend` in the same split; it is not built, run or
imported from here, so nothing in this workspace needs it present. Clone it
next to this repo — or, if you want colcon to build it against your local
interfaces, into `src/syncai_backend`, which is gitignored for exactly that.

**On an existing robot**, the pull that brings this change **deletes both
directories from the working tree** — git removes them as tracked files and
nothing puts them back. Run `vcs import < interface.repos` before the next
build, or it fails on `syncai_common`, which was there yesterday and which
every package needs. `src/syncai_backend` stays gone on purpose;
`scripts/build.sh` refuses to start on an empty checkout and names the `.repos`
file to import, so a forgotten import is one clear error rather than a wall of
CMake output.

### 2. Pick the robot identity

Every launch file reads `[system] robot_id` from `config/system.ini`, which is
tracked **empty**. `docker-compose.robots.yml` bind-mounts
`config/instances/robot01.ini` over it inside the container. That INI also names
the map to localize against (`[map] name`), the initial pose, and the lidar's IP
and model (`[sensor.lidar] type: mid360 | mid360s` — a wrong model yields no
point cloud and no error).

### 3. Start the containers

```bash
# .env supplies UID/GID, postgres / temporal settings, etc.
docker compose up -d            # infra + robot01
docker compose exec robot01 bash
```

The workspace is mounted at `/home/syncrobotic/robot_ws` and is the container's
working directory. ROS 2 and the workspace overlay are sourced by `~/.bashrc`.
Alternatively open the folder in VS Code and **Reopen in Container**
(`.devcontainer/`).

### 4. Build

Two ways, same image, same `build/` + `install/` on disk. **The robot container
is a live robot** — build deliberately, not on every edit.

**From the host**, in a throwaway container (`docker-compose.build.yaml`):

```bash
docker compose -f docker-compose.build.yaml run --rm build                                  # everything
docker compose -f docker-compose.build.yaml run --rm build --packages-select syncai_planner # colcon args pass through
BUILD_ROSDEP=off docker compose -f docker-compose.build.yaml run --rm build                 # skip the rosdep report
```

It runs `scripts/build.sh`: vcs-checkout sanity check (the six
`src/third-party/` dirs and `src/syncai_common`, with the `.repos` file to
import if one is empty), restore the `livox_ros_driver2` `package.xml` if
missing, `rosdep check` (report only — see below), then
`colcon build --symlink-install`. Toggles: `BUILD_COLCON`
(`1`/`0`), `BUILD_ROSDEP` (`check`/`install`/`off`). The container exits when
the build does; robot01 picks the new `install/` up on the next session (re)build
(`switch_mode`).

**By hand**, from the workspace root inside the container:

```bash
rosdep install --from-paths src --ignore-src -r -y   # declared deps
colcon build --symlink-install                       # or: scripts/build.sh
source install/setup.bash

# build a single package
colcon build --packages-select syncai_planner
```

`rosdep install` only makes sense inside robot01: anything it installs into
the throwaway build container is gone when that exits and never reaches the
robot, so the compose route only *reports* unmet keys and a missing
dependency is a `Dockerfile` change. (Keys it reports as "cannot locate" —
GTSAM, livox_sdk2, libgraphicsmagick++1-dev — are satisfied by the image under
names rosdep does not know.)

GTSAM, Sophus and Livox-SDK2 come from the image's `deps-builder` stage. Two
things trip a fresh checkout:

- `colcon.meta` (found only because colcon's default is the relative
  `./colcon.meta`, so build from the workspace root) passes
  `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble` to `livox_ros_driver2`; without it
  configure dies with `NOTFOUND`.
- After any update of the `livox_ros_driver2` submodule its `package.xml` is
  gone (the repo ships `package_ROS2.xml` and gitignores the copy), and every
  ament package then fails to configure:

  ```bash
  cp -f src/third-party/livox_ros_driver2/package_ROS2.xml \
        src/third-party/livox_ros_driver2/package.xml
  ```

### 5. Run the stack

Nothing has to be launched by hand. The robot container's main process is
`ros2 launch syncai_sys_manager sys_manager.launch.py`; on start it builds the
**AUTO** byobu session (`syncai-dev`) from `config/sessions/start_nav.yaml`:
bringup → map_server + localizer → lio_bridge → planner + controller →
task_runner → driver_manager → robot_state, with `sleep` offsets standing in
for the missing lifecycle manager.

```bash
# from the HOST: attach to whichever session is live (syncai-dev in AUTO,
# syncai-mapping in MANUAL — a hardcoded name is wrong half the time)
scripts/attach.sh                    # finds the live session, or a shell if none
scripts/attach.sh robot01 syncai-dev # explicit container / session
alias robot='~/SyncAI-Robot-Workspace/scripts/attach.sh'   # handy in ~/.bashrc

# inside the container: query / switch the operating mode (MAINTENANCE=0 MANUAL=1 AUTO=2)
ros2 service call /<robot_id>/get_mode    syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode "{mode: 1}"
```

The mode is derived from which byobu session exists, never stored, so it stays
correct across a `sys_manager` restart. `switch_mode` kills every session before
building the target one and refuses to rebuild the mode that is already live
(in MANUAL that would drop the unsaved map).

This repo serves the Temporal UI at `:8081` and pgAdmin at `:5050`. The
operator API — REST + WebSockets on `http://<robot>:3000`, interactive docs at
`/docs` — is `SyncAI-Robot-Backend`, deployed as its own container on this host
and pointed at by the console (`SyncAI-Robot-Frontend`; nothing here serves
either). Navigation goals, mode switches, teleop and map saving all go through
that API. A raw `NavigateToPose` goal to `/<robot_id>/task_runner` works as
well, and is the route that needs nothing outside this repo.

**Mapping loop.** Switch to MANUAL (the mapping session runs bringup + PGO +
driver_manager + robot_state, and none of the localization / planning nodes),
drive the robot, then save the map. `pgo/save_maps` is what writes
`map/<name>/`: `map.pcd`, `poses.txt` and `patches/`. Calling it from a shell
gives you exactly that and no gridmap — both pcd → gridmap recipes left with
the backend, so the console's save button (`POST /api/v1/maps`) is what calls
the service and converts in one step. Then point the robot at the new map:
`[map] name` in the instance INI, or the console's map switch, which does the
same thing live. Rebuilding a gridmap with another recipe, hand-editing it,
and renaming a map (the directory moves and the vertices and task templates
bound to it follow, except for the map the stack is running on — 409
`map_active`, because map_server and the localizer loaded its files at launch)
are all backend routes, and all still write into this repo's `map/`.

**Camera.** The camera is published from the **host**, not the container:

```bash
bash scripts/publish_camera_crop.sh            # start (background)
bash scripts/publish_camera_crop.sh status     # state + viewing URLs
bash scripts/publish_camera_crop.sh stop
```

It pushes RTSP to the remote MediaMTX named in `scripts/publish_camera_crop.env`.
The ROS camera node in `bringup.launch.py` is off by default because the two
cannot share the V4L2 device.

**Logs.** Every pane is captured by multilog under
`log/stack/<robot_id>/<window>/` (`mapping/` subtree for the mapping session):

```bash
tail -f log/stack/<robot_id>/planner/current
zcat log/stack/<robot_id>/planner/@*.s | less
```

### 6. Tests

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

The C++ packages carry only the ament linter tests from the package templates;
`syncai_sys_manager` adds `test_wifi_manager.py`. The large pytest suite that
used to live here went with `syncai_backend`.

## Notes

- **Non-lifecycle by design** — there is no lifecycle manager; servers are
  plain nodes and come up active immediately, so ordering lives in the session
  specs' `sleep` fields.
- **`use_sim_time`** is set *only* in the params YAML files, never in launch.
- **DDS is loopback-only.** `config/cyclonedds.xml` disables multicast, binds
  `lo`, and pins `<Domain Id="1">`. Host networking is used for the lidar's UDP
  on the physical NIC and so the containers share the host's loopback, **not**
  for LAN discovery. Compose sets `ROS_DOMAIN_ID: "1"` as a literal; the Jetson
  host shell exports `ROS_DOMAIN_ID=69`, so a `ros2` CLI or rviz2 run on the
  host will not see the containers until that export is changed.
- **Secrets** — `.env` holds local secrets and is gitignored. Do not commit it.
  The camera script deliberately reads its own `publish_camera_crop.env`, not
  the compose `.env`.
