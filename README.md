# SyncAI Robot Workspace

A ROS 2 Humble software stack for the SyncAI robot (G23 quadruped / AMR
chassis), covering the full vertical: Livox lidar + camera drivers, FAST-LIO2
odometry and localization, a **non-lifecycle port of Navigation2**, a
Temporal-backed task orchestration backend, and a Next.js operator console.

The nav2 servers (map server, costmap, planner, controller, BT navigator) were
re-implemented as plain `rclcpp::Node`s instead of lifecycle nodes, so the stack
starts and runs without a lifecycle manager; startup ordering is handled by the
byobu session specs instead. Navigation is driven by a Behavior Tree.

> Built and run inside a Docker container (`ubuntu:22.04` + ROS 2 Humble) on a
> Jetson. RMW is CycloneDDS configured via `config/cyclonedds.xml`.
>
> `CLAUDE.md` holds the conventions and gotchas needed to edit the code (the
> `robot_id` namespacing rules, the session specs, the backend layering). Every
> package under `src/` has its own `README.md` with parameter tables and topic
> names. This file is the user-level clone / build / run guide.

## Architecture

```
                         NavigateToPose (nav2_msgs)
   syncai_backend  ───────────────────────────────▶  syncai_task_runner   (BT navigator; ticks behavior_trees/move.xml)
   (Temporal RobotWorkflow)                                  │
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
| `syncai_backend` | Python. FastAPI + rclpy in one process (port **3000**), Temporal worker (tasks, templates, schedules), map catalogue + pcd → gridmap conversion, TTS, WebSocket streams |
| `syncai_frontend` | Next.js operator console (port **3001**): dashboard with 3D point cloud, mapping, map library (rebuild grid, rename) + gridmap editor, tasks, settings |
| `syncai_ros_mcp` | MCP server as a ROS 2 node (port **8000**): topics / services / tasks / maps (catalogue, image, vertices) as MCP tools over the backend's REST API. Not started by anything — run by hand |

### Third-party (`src/third-party/`)

All six are git submodules; nothing is vendored there any more.

| Package | Notes |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`, unmodified |
| `FASTLIO2_ROS2` | `chungweeeei/SyncAI-Fast-LIO2`, branch `dev` (**SSH remote** — a recursive clone needs a GitHub key). LIO + PGO + HBA + `localizer` |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver. `colcon.meta` passes the cmake flags the driver needs; see "Build" |
| `small_gicp` | Pinned to `v1.0.1`, unmodified; the localizer's registration backend |
| `vizionsdk-ros2` | TechNexion camera wrapper; needs the VizionSDK `.deb` the `Dockerfile` installs |

To bump a submodule, check out the new commit inside it and commit the pointer:

```bash
cd src/third-party/behaviortree_cpp_v3 && git fetch --tags && git checkout <tag>
cd - && git add src/third-party/behaviortree_cpp_v3 && git commit -m "chore: bump behaviortree_cpp_v3 to <tag>"
```

## Repository layout

```
.
├── src/                          # colcon packages (see table above)
│   └── third-party/              # six submodules
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
│   ├── urdf2glb.py               # bakes G23.urdf into the frontend's public/models/g23.glb
│   └── release/                  # offline release bundle for the IPC (currently non-functional, see CLAUDE.md)
├── doc/                          # design proposals (agent / MCP integration; not implemented)
├── map/                          # LIO map output per map name (map.pcd, poses.txt, gridmap.*) — gitignored
├── models/kokoro/                # TTS weights (~330 MB, downloaded once) — gitignored
├── log/stack/<robot_id>/         # multilog capture of every byobu pane — gitignored
├── Dockerfile                    # multi-stage: base → deps-builder (GTSAM/Sophus/Livox-SDK2) → dev
├── docker-compose.yml            # infra: postgres (5432) / pgadmin (5050) / temporal (7233) / temporal_ui (8081)
├── docker-compose.robots.yml     # robot01 (host networking, nvidia runtime, cameras, audio, D-Bus, avahi); `include`d above
├── docker-compose.build.yaml     # standalone one-shot `colcon build` service (same image, own project name; no frontend)
├── colcon.meta                   # per-package cmake args (livox_ros_driver2)
├── ruff.toml                     # Python lint config for all three Python packages
├── .devcontainer/                # VS Code "Reopen in Container"
└── .env                          # compose env + secrets (gitignored — never commit)
```

## Getting started

### 1. Clone (with submodules)

```bash
git clone --recursive <repo-url>
# already cloned without --recursive:
git submodule update --init --recursive
```

`FASTLIO2_ROS2` is a private SSH remote; the clone fails without a GitHub key
that can read it.

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

It runs `scripts/build.sh`: submodule sanity check, restore the
`livox_ros_driver2` `package.xml` if missing, `rosdep check` (report only —
see below), then `colcon build --symlink-install`. Toggles: `BUILD_COLCON`
(`1`/`0`), `BUILD_ROSDEP` (`check`/`install`/`off`). ROS packages only — the
frontend is not an ament package; its build is `npm install` in
`src/syncai_frontend` (see its README). The container exits when the build
does; robot01 picks the new `install/` up on the next session (re)build
(`switch_mode`).

**By hand**, from the workspace root inside the container:

```bash
rosdep install --from-paths src --ignore-src -r -y   # declared deps
python3 -m pip install "fastmcp>=3.4.4"              # pip-only dep of syncai_ros_mcp
colcon build --symlink-install                       # or: scripts/build.sh
source install/setup.bash

# build a single package
colcon build --packages-select syncai_planner
```

`rosdep install` only makes sense inside robot01: anything it installs into
the throwaway build container is gone when that exits and never reaches the
robot, so the compose route only *reports* unmet keys and a missing
dependency is a `Dockerfile` change. (Keys it reports as "cannot locate" —
GTSAM, livox_sdk2, libgraphicsmagick++1-dev, python3-structlog — are
satisfied by the image under names rosdep does not know.)

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
task_runner → driver_manager → robot_state + backend → frontend, with `sleep`
offsets standing in for the missing lifecycle manager.

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

Then open the operator console at `http://<robot>:3001` (backend REST at
`:3000`, Temporal UI at `:8081`, pgAdmin at `:5050`). Navigation goals, mode
switches, teleop, and map saving all go through the console / backend; a raw
`NavigateToPose` goal to `/<robot_id>/task_runner` works too.

**Mapping loop.** Switch to MANUAL (the mapping session runs bringup + PGO +
driver_manager + backend + frontend, and none of the localization / planning
nodes), drive the robot, then save the map from the console
(`POST /api/v1/maps` → `pgo/save_maps` + pcd → gridmap conversion into
`map/<name>/`). Set `[map] name` in the instance INI to the new map and switch
back to AUTO. The gridmap can be rebuilt with a different recipe from the map
card, or hand-edited in the gridmap editor. A map can also be renamed from its
card (`PATCH /api/v1/maps/{name}`): the directory moves and the vertices and
task templates bound to it follow — except for the map the stack is currently
running on, which the backend refuses (409 `map_active`) because map_server and
the localizer loaded its files at launch. Switch maps and restart first.

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

# backend pytest suite (~40 files; conversion tests need open3d + scipy)
cd src/syncai_backend && python3 -m pytest test/
```

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
