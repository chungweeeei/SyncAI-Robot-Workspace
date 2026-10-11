# SyncAI Robot Workspace

A ROS 2 Humble software stack for the SyncAI robot (quadruped robot): Livox lidar + camera drivers, 3D lidar odometry and localization, a
**non-lifecycle port of Navigation2**, and the byobu session manager that brings
all of it up. The two operator-facing halves each live in their own repository
now.

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
                                 syncai_global_planner                         syncai_controller
                                   (SmacPlanner2D)                     (MPPI / RPP plugins)
                                           │                                   │
                                           └───────── syncai_costmap_2d ───────┘
                                                (global / local costmaps)          cmd_vel ──▶ syncai_driver_manager ──UDP──▶ gait controller

   livox_ros_driver2 ──▶ syncai_pointlio ──▶ syncai_localizer ──▶ syncai_lio_bridge ──▶ odom → base_link TF, map → odom, /odom
                                        └──▶ syncai_mapping (pgo_node, mapping mode) ──▶ map/<name>/, pgo/map_cloud_file
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
| `syncai_common` | Shared msg / srv / action interfaces (`RobotState`, `SwitchMode`, `SetMotionKey`, `ExecuteTask`, …). **Not tracked here**, see below |
| `syncai_util` | Helpers (geometry, odometry window, simple action server, robot utils) |
| `syncai_nav_core` | Header-only abstract interfaces for nav plugins (port of `nav2_core`) |
| `syncai_costmap_2d` | Global / local costmaps with layered plugins (static / obstacle / inflation / keepout filter) |
| `syncai_global_planner` | `compute_path_to_pose` action server; NavFn, StraightLine and SmacPlanner2D plugins (Smac is configured) |
| `syncai_controller` | `follow_path` action server and the local costmap; loads MPPI as `FollowPath` and its own Regulated Pure Pursuit as `FollowPathRPP`, each clamping its own acceleration (no velocity smoother) |
| `syncai_mppi_controller` | MPPI controller plugin (port of `nav2_mppi_controller`, Humble): local obstacle avoidance on the local costmap, optimal trajectory on `local_plan` |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`) |
| `syncai_task_runner` | BT navigator: serves `NavigateToPose`, ticks `behavior_trees/move.xml` |
| `syncai_map_server` | Map server, map saver, costmap-filter-info server |
| `syncai_pointlio` | Point-LIO front end (`pointlio_node`): LIO odometry, the body-frame cloud (deskewed to the end of the scan; plus `body_cloud_dense` for mapping), the `pointlio_odom → pointlio_body` TF, and `reset` |
| `syncai_mapping` | Mapping back end (`pgo_node`): keyframes, GTSAM loop closure, the live map-cloud hand-off, and the run lifecycle — `start_mapping` / `save_maps` / `reset_mapping`, state latched on `mapping_status`; plus `clean_map`, the post-save job that removes people from `map.pcd`. **Not tracked here**, see below |
| `syncai_localizer` | Map-based relocalization (`localizer_node`): two-stage GICP of the body cloud against `map.pcd`, the `map → pointlio_odom` correction, `relocalize` / `relocalize_check` and `initialpose` |
| `syncai_lio_bridge` | LIO → planar `odom` / TF bridge (the only odometry source). **Not tracked here**, see below |
| `syncai_bringup` | `robot_state_publisher` over `description/G23.urdf`, the Livox MID360 / MID360s driver (config JSON generated per robot), optional TechNexion camera node |
| `syncai_driver_manager` | UDP bridge to the gait controller: `cmd_vel` out (with per-direction velocity scales), telemetry in, safety lock. **Not tracked here**, see below |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motors / TF into `syncai_common/RobotState`. **Not tracked here**, see below |
| `syncai_sys_manager` | Python. Wifi, mDNS, host monitoring, and the **byobu session manager** (`switch_mode` / `get_mode`) — the robot container's main process |

Five packages in the table are **not tracked in this repo**. Each lives in its
own repository and is imported back into `src/` by vcstool (see "Getting
started"); edit them in their own checkouts.

| Package | Repo | Notes |
|---|---|---|
| `syncai_common` | `SyncAI-Robot-Interface` | The wire format every package here and the backend build against |
| `syncai_mapping` | `SyncAI-Robot-3D-Mapping` | C++ (rclcpp); needs GTSAM / OctoMap from the image and `small_gicp` from `third-party.repos` |
| `syncai_driver_manager` | `SyncAI-Robot-Driver-Manager` | Rust (rclrs, `ament_cargo`) |
| `syncai_robot_state` | `SyncAI-Robot-State` | Rust (rclrs, `ament_cargo`) |
| `syncai_lio_bridge` | `SyncAI-LIO-Bridge` | Rust (rclrs, `ament_cargo`) |

All four that moved out after `syncai_common` keep their package name,
executables and launch file, so the session specs are unchanged. The Rust
message crates the three Rust ones need come from a ros2-rust underlay baked
into the robot image (see "Getting started").

The operator half — `SyncAI-Robot-Backend` (FastAPI + rclpy on port **3000**:
tasks, map catalogue, TTS, WebSockets) and `SyncAI-Robot-Frontend` (the Next.js
console) — is not built, run or imported from here. The backend runs in its own
container with host networking on DDS domain 1 and discovers the nodes above
over DDS; the infra it needs (postgres, temporal) stays in this repo's
`docker-compose.yml` because the robot is where it is deployed.

### Third-party (`src/third-party/`)

All five are checked out by vcstool from `third-party.repos` (they were git
submodules until `fca520b`); nothing is vendored there any more.

| Package | Notes |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`, unmodified |
| `livox_ros_driver2`, `Livox-SDK2` | MID360 / MID360s driver. `colcon.meta` passes the cmake flags the driver needs; see "Build" |
| `small_gicp` | Pinned to `v1.0.1`, unmodified; the registration backend of `syncai_localizer` and of `syncai_mapping`'s loop closure |
| `vizionsdk-ros2` | TechNexion camera wrapper; needs the VizionSDK `.deb` the `Dockerfile` installs |

The FAST-LIO2 fork that used to be imported at `src/third-party/FASTLIO2_ROS2`
is gone: its packages were ported in-tree as `syncai_pointlio`,
`syncai_mapping` and `syncai_localizer`. A checkout left on disk from before
still builds a duplicate `localizer` package; delete it
(`rm -rf src/third-party/FASTLIO2_ROS2 build/localizer install/localizer`).

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
│   └── third-party/              # five upstream repos, checked out by vcstool
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
├── map/                          # LIO map output per map name (map.pcd, poses.txt, gridmap.*, optional keepout.*) — gitignored
├── log/stack/<robot_id>/         # multilog capture of every byobu pane — gitignored
├── Dockerfile                    # multi-stage: base → livox / gtsam / sophus + rust-underlay (parallel) → dev
├── docker-compose.yml            # infra: postgres (5432) / pgadmin (5050) / temporal (7233) / temporal_ui (8081)
├── docker-compose.robots.yml     # robot01 (host networking, nvidia runtime, cameras, audio, D-Bus, avahi); `include`d above
├── docker-compose.build.yaml     # standalone one-shot `colcon build` service (same image, own project name)
├── colcon.meta                   # per-package cmake args (livox_ros_driver2)
├── ruff.toml                     # Python lint config (syncai_sys_manager, scripts/)
├── third-party.repos             # vcstool: src/third-party/ — upstream, pinned
├── dependencies.repos            # vcstool: our own packages in their own repos (src/syncai_common, _mapping, _driver_manager, _robot_state, _lio_bridge)
├── .devcontainer/                # VS Code "Reopen in Container"
└── .env                          # compose env + secrets (gitignored — never commit)
```

## Getting started

### 1. VCS import the source repos

```bash
sudo apt update && sudo apt install python3-vcstool -y
vcs import < third-party.repos     # src/third-party/ — upstream code, pinned
vcs import < dependencies.repos    # src/syncai_{common,mapping,driver_manager,robot_state,lio_bridge} — our own, in their own repos
```

Both are required before the first `colcon build`. The five `src/syncai_*`
directories `dependencies.repos` imports are gitignored: edit them in their own
checkouts, because the next `--force` import overwrites whatever is in them.

Only a missing `src/syncai_common` fails loudly (every package at once). A
missing `src/syncai_mapping`, `src/syncai_driver_manager`,
`src/syncai_robot_state` or `src/syncai_lio_bridge` builds fine and leaves the
robot with nothing to save a map with, no bridge to the gait controller,
nothing publishing `RobotState` and no odometry source — so
`scripts/build.sh` refuses to start on an empty checkout and names the `.repos`
file to import.

**On an existing robot**, the pull that moves a package out deletes its
directory and nothing puts it back: run `vcs import < dependencies.repos`
before the next build. That also fetches and checks out the pin in every
checkout that is already there; add `--skip-existing` to clone only the missing
one and leave the others as they are.

#### The three Rust packages

`syncai_driver_manager`, `syncai_robot_state` and `syncai_lio_bridge` are
`ament_cargo` (rclrs), and need nothing beyond `vcs import`: the robot image
carries the Rust toolchain and a ros2-rust underlay at `/opt/ros2_rust_underlay`
(rclrs plus the Humble interfaces rebuilt with Rust bindings, which the apt
copies lack), sourced between `/opt/ros/humble` and the workspace. An image
built before it has no underlay, and `scripts/build.sh` says so instead of
letting cargo fail. The C++ stack still builds without the Rust packages:

```bash
colcon build --symlink-install --packages-skip syncai_driver_manager syncai_robot_state syncai_lio_bridge
```

If configure fails on a missing `register_rs.cmake`, delete the stale `build/` +
`install/` copies of the interface packages that shadow the underlay. The pins
and the reasoning are in the `Dockerfile`'s underlay stanza and `CLAUDE.md`.

#### Re-importing `src/syncai_common` (stale checkout)

A `src/syncai_common/` left over as a **plain directory** (no `.git` inside)
passes `scripts/build.sh`'s check and builds the old messages; the symptom is a
build failing on a srv it predates (`ResetLIO`, `SaveMaps`, `StartMapping`, …).
`vcs import` will not clone over it, so fix it on the **host**:

```bash
git -C src/syncai_common rev-parse --show-toplevel   # must print .../src/syncai_common
mv src/syncai_common ~/syncai_common.stale           # if it does not
vcs import < dependencies.repos
```

Keep it current with `vcs pull src/syncai_common`, or
`vcs import < dependencies.repos --force` to re-checkout at the pins (drops
local edits — in all five checkouts, not only this one), then rebuild.

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
docker compose -f docker-compose.build.yaml run --rm build --packages-select syncai_global_planner # colcon args pass through
BUILD_ROSDEP=off docker compose -f docker-compose.build.yaml run --rm build                 # skip the rosdep report
```

It runs `scripts/build.sh`: check every vcs checkout (naming the `.repos` file
to import if one is empty), restore the `livox_ros_driver2` `package.xml` if
missing, `rosdep check` (report only — see below), then
`colcon build --symlink-install`. Toggles: `BUILD_COLCON` (`1`/`0`),
`BUILD_ROSDEP` (`check`/`install`/`off`). The container exits when the build
does; robot01 picks the new `install/` up on the next session (re)build
(`switch_mode`).

**By hand**, from the workspace root inside the container:

```bash
rosdep install --from-paths src --ignore-src -r -y   # declared deps
colcon build --symlink-install                       # or: scripts/build.sh
source install/setup.bash

# build a single package
colcon build --packages-select syncai_global_planner
```

`rosdep install` only makes sense inside robot01: anything installed into the
throwaway build container is gone when it exits, so the compose route only
*reports* unmet keys and a missing dependency is a `Dockerfile` change. The keys
it still reports today are either satisfied by the image under another name or
harmless.

GTSAM, Sophus and Livox-SDK2 come from the image's `livox` / `gtsam` /
`sophus` stages, and OctoMap (`ros-humble-octomap`, for `clean_map`) from its `dev` stage. Two
things trip a fresh checkout:

- `colcon.meta` (found only because colcon's default is the relative
  `./colcon.meta`, so build from the workspace root) passes
  `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble` to `livox_ros_driver2`; without it
  configure dies with `NOTFOUND`.
- After any `vcs import` that touches `livox_ros_driver2` its `package.xml` is
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
bringup → map_server + keepout → pointlio + localizer → lio_bridge → planner +
controller → task_runner → driver_manager → robot_state, with `sleep` offsets
standing in for the missing lifecycle manager. The `keepout` pane serves
`map/<name>/keepout.yaml` to the planner's costmap filter, writing a blank
(all-unknown) one of the gridmap's size first when the map has none yet. Draw a
mask as the forbidden area only — the filter inflates it with the robot's
footprint itself, so a hand-drawn margin is applied twice.

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

**Mapping loop.** Switch to MANUAL — the mapping session runs bringup +
pointlio + pgo (`syncai_mapping`) + driver_manager + robot_state, and none of
the localization / planning nodes. A run is bracketed by two service calls:
with the robot **standing still** press *Start mapping* (`pgo/start_mapping`;
pgo comes up idle and banks nothing until then), drive the robot, then save.
`pgo/save_maps` writes `map/<name>/` — `map.pcd`, `poses.txt`, `patches/` — and
ends the run, leaving pgo idle for the next Start; `pgo/reset_mapping` throws a
run away mid-drive and starts over.

`map.pcd` is not final when the save returns: pgo then spawns `clean_map`, a
detached job that survives the mode switch and, a few minutes later, atomically
replaces `map.pcd` with a copy without people and one-off returns.
`map_clean.recipe.json` beside it says `converting` → `ok` / `failed`; a
gridmap converted before `ok` was made from the raw map. `patches/` stay raw,
so `ros2 run syncai_mapping clean_map map/<name>` re-cleans by hand.

From a shell you get those files and no gridmap: both pcd → gridmap
recipes left with the backend, so the console's save button
(`POST /api/v1/maps`) is what calls the service and converts in one step. Then
point the robot at the new map — `[map] name` in the instance INI, or the
console's map switch, which does the same live. Rebuilding a gridmap,
hand-editing it and renaming a map are backend routes too, and all write into
this repo's `map/` (renaming the map the stack is running on is refused with 409
`map_active`, since map_server and the localizer loaded its files at launch).

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
