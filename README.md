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
              NavigateToPose (nav2_msgs) / NavigateToGoal (syncai_common)
   RobotWorkflow ─────────────────────────────────▶  syncai_task_runner   (BT navigator; ticks behavior_trees/move.xml)
   (SyncAI-Robot-Backend, out of tree)                       │
                                          compute_path_to_pose│follow_path
                                           ┌─────────────────┴─────────────────┐
                                           ▼                                   ▼
                                 syncai_global_planner                         syncai_controller
                                       (Octo3D)                        (MPPI / RPP 3D / Docking)
                                           │                                   │
                                           └───────── syncai_costmap_2d ───────┘
                                                (global / local costmaps)          cmd_vel ──▶ syncai_driver_manager ──UDP──▶ gait controller

   livox_ros_driver2 ──▶ syncai_pointlio ──▶ syncai_localizer ──▶ syncai_lio_bridge ──▶ odom → base_link TF, map → odom, /odom
                                        └──▶ syncai_mapping (pgo_node, mapping mode) ──▶ map/<name>/, pgo/map_cloud_file
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
| `syncai_util` | Helpers (geometry, odometry window, simple action server, robot utils); on this branch also `path_height.hpp` (a path's heights, the ground under the robot) and `StairMode`, the stair state machine both 3D-aware controllers run (`test/test_stair_mode.cpp`) |
| `syncai_nav_core` | Header-only abstract interfaces for nav plugins (port of `nav2_core`) |
| `syncai_costmap_2d` | The local costmap's layers: a **terrain layer**, a rolling 2.5D grid of the lidar's returns that marks low obstacles, drop-offs and ledges and judges a stair lethal, and inflation. The planner's global costmap has no layers at all (Octo3D reads no cells). The static and obstacle layers and the keepout filter went with 2D navigation (2026-10-28) |
| `syncai_global_planner` | `compute_path_to_pose` action server with one planner, `Octo3D` (OctomapPlanner3D, the quadruped's ground-constrained 3D planner on `map/<name>/octomap.bt`); it keeps a dynamic-obstacle overlay of the lidar cloud, on which it answers `is_path_valid` and detours when it replans, and reads the map's forbidden zones from `keepout.json`. NavFn, StraightLine and SmacPlanner2D (`GridBased`) went with 2D navigation (2026-10-28) |
| `syncai_controller` | `follow_path` action server and the local costmap; loads MPPI as `FollowPath` and RPP 3D as `FollowPathRPP3D`, each clamping its own acceleration (no velocity smoother). Also `dock_to_pose` (`syncai_common/DockToPose`, 2026-10): the last metre onto or off a dock with its own `DockingController` (`Docking` / `Undocking`), tracked in odom, no costmap. RPP 3D (`RegulatedPurePursuitController3D`; the 2D RPP it was copied from went with 2D navigation) and the goal check follow paths with heights in 3D, and both RPP 3D and MPPI climb stairs in a *stair mode* (`syncai_util::StairMode`) that switches the gait controller's RL policy. The trees name MPPI (`FollowPath`); RPP 3D is the A/B |
| `syncai_mppi_controller` | MPPI controller plugin (port of `nav2_mppi_controller`, Humble): local obstacle avoidance on the local costmap, optimal trajectory on `local_plan`. It also matches a path with heights in 3D and climbs stairs in the shared stair mode — blind, without the optimizer or the costmap |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`), plus the docking nodes `DockToPose`, `SetMotionKey`, `ComputeStagingPose`, `ComputeBackOutPose`, `Wait`, `WaitForMotionState` (2026-10) |
| `syncai_task_runner` | BT navigator: serves `nav2_msgs/NavigateToPose` and `syncai_common/NavigateToGoal` (the same, with a failure reason in the result), ticks `behavior_trees/move.xml` (Octo3D + MPPI) — or `dock.xml` / `undock.xml` when the goal names them |
| `syncai_pointlio` | Point-LIO front end (`pointlio_node`): LIO odometry, the body-frame cloud (deskewed to the end of the scan; plus `body_cloud_dense` for mapping), the `pointlio_odom → pointlio_body` TF, and `reset` |
| `syncai_mapping` | Mapping back end (`pgo_node`): keyframes, GTSAM loop closure, the live map-cloud hand-off, and the run lifecycle — `start_mapping` / `save_maps` / `reset_mapping`, state latched on `mapping_status`; plus `clean_map`, the post-save job that removes people from `map.pcd`. **Not tracked here**, see below |
| `syncai_localizer` | Map-based relocalization (`localizer_node`): two-stage GICP of the body cloud against `map.pcd`, the `map → pointlio_odom` correction, `relocalize` / `relocalize_check` and `initialpose` |
| `syncai_lio_bridge` | LIO → planar `odom` / TF bridge (the only odometry source). **Not tracked here**, see below |
| `syncai_bringup` | `robot_state_publisher` over `description/G23.urdf`, the Livox MID360 / MID360s driver (config JSON generated per robot) |
| `syncai_driver_manager` | UDP bridge to the gait controller: `cmd_vel` out (with per-direction velocity scales), telemetry in, safety lock. **Not tracked here**, see below |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motors / TF into `syncai_common/RobotState`. **Not tracked here**, see below |
| `syncai_sys_manager` | Python. Wifi, mDNS, host monitoring, and the **byobu session manager** (`switch_mode` / `get_mode`) — the robot container's main process |
| `syncai_camera_feeder` | Sole owner of `/dev/syncai/camera0`: its MJPEG stream, undecoded, as RTP/JPEG to the WebRTC worker and as `image_raw/compressed` to ROS. Started by the nav session only. **Not tracked here**, see below |

Six packages in the table are **not tracked in this repo**. Each lives in its
own repository and is imported back into `src/` by vcstool (see "Getting
started"); edit them in their own checkouts.

| Package | Repo | Notes |
|---|---|---|
| `syncai_common` | `SyncAI-Robot-Interface` | The wire format every package here and the backend build against |
| `syncai_mapping` | `SyncAI-Robot-3D-Mapping` | C++ (rclcpp); needs GTSAM / OctoMap from the image and `small_gicp` from `third-party.repos` |
| `syncai_driver_manager` | `SyncAI-Robot-Driver-Manager` | Rust (rclrs, `ament_cargo`) |
| `syncai_robot_state` | `SyncAI-Robot-State` | Rust (rclrs, `ament_cargo`) |
| `syncai_lio_bridge` | `SyncAI-LIO-Bridge` | Rust (rclrs, `ament_cargo`) |
| `syncai_camera_feeder` | `SyncAI-Robot-Camera-Feeder` | Rust (rclrs, `ament_cargo`) + GStreamer; written there, never in this tree |

All four that moved out after `syncai_common` keep their package name,
executables and launch file, so the session specs are unchanged. The Rust
message crates the four Rust ones need come from a ros2-rust underlay baked
into the robot image (see "Getting started"). `syncai_camera_feeder` also
links GStreamer, whose `-dev` packages the image installs, and is the one
opener of `/dev/syncai/camera0`: the device takes only one.

The operator half — `SyncAI-Robot-Backend` (FastAPI + rclpy on port **3000**:
tasks, map catalogue, TTS, WebSockets) and `SyncAI-Robot-Frontend` (the Next.js
console) — is not built, run or imported from here. The backend runs in its own
container with host networking on DDS domain 1 and discovers the nodes above
over DDS; the infra it needs (postgres, temporal) stays in this repo's
`docker-compose.yml` because the robot is where it is deployed.

### Third-party (`src/third-party/`)

All six are checked out by vcstool from `third-party.repos` (they were git
submodules until `fca520b`); nothing is vendored there any more.

| Package | Notes |
|---|---|
| `behaviortree_cpp_v3` | Pinned to upstream tag `3.8.8`, unmodified |
| `direct_visual_lidar_calibration` | Targetless camera → lidar extrinsic calibration, run by hand on bags recorded in CALIBRATION mode. Has git submodules, hence `--recursive`; Ceres / Iridescence / GTSAM's unstable library come from the image |
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
vcs import --recursive < third-party.repos --force
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
│   ├── sessions/                 # byobu session specs: start_nav.yaml (AUTO), start_mapping.yaml (MANUAL), calib_lidar_camera.yaml (CALIBRATION)
│   ├── cyclonedds.xml            # CycloneDDS: loopback-only unicast, Domain Id 1
│   └── rviz2/robot01.rviz        # for running rviz2 from a workstation (no display on the robot)
├── scripts/
│   ├── attach.sh                 # host-side: attach to whichever byobu session is live in the container
│   ├── build.sh                  # in-image: colcon build of every ROS package; entrypoint of docker-compose.build.yml
│   ├── urdf2glb.py               # bakes G23.urdf into the GLB the operator console renders (handed over to its repo)
│   ├── calibration/              # camera intrinsics + lidar-camera extrinsic tools (see docs/lidar-camera-calibration.md)
│   └── release/                  # offline release bundle for the IPC (currently non-functional, see CLAUDE.md)
├── docs/                         # operating procedures: lidar-camera calibration, localizer offline replay
├── .github/                      # CI: lint, arm64 build + smoke test of every session spec
├── map/                          # LIO map output per map name (map.pcd, poses.txt, gridmap.* or octomap.bt, optional keepout.*) — gitignored
├── log/stack/<robot_id>/         # multilog capture of every byobu pane — gitignored
├── Dockerfile                    # multi-stage: base → deps-builder → livox / gtsam / sophus / ceres / iridescence, base → rust-underlay (all parallel) → dev
├── docker-compose.yml            # infra: postgres (5432) / pgadmin (5050) / temporal (7233) / temporal_ui (8081)
├── docker-compose.robots.yml     # robot01 (host networking, nvidia runtime, cameras, audio, D-Bus, avahi); `include`d above
├── docker-compose.build.yml      # standalone one-shot `colcon build` service (same image, own project name)
├── colcon.meta                   # per-package cmake args (livox_ros_driver2)
├── ruff.toml                     # Python lint config (syncai_sys_manager, scripts/)
├── third-party.repos             # vcstool: src/third-party/ — upstream, pinned
├── dependencies.repos            # vcstool: our own packages in their own repos (src/syncai_common, _mapping, _driver_manager, _robot_state, _lio_bridge, _camera_feeder)
├── .devcontainer/                # VS Code "Reopen in Container"
└── .env                          # compose env + secrets (gitignored — never commit)
```

## Getting started

### 1. VCS import the source repos

```bash
sudo apt update && sudo apt install python3-vcstool -y
vcs import --recursive < third-party.repos     # src/third-party/ — upstream code, pinned
vcs import < dependencies.repos    # src/syncai_{common,mapping,driver_manager,robot_state,lio_bridge,camera_feeder} — our own, in their own repos
```

Both are required before the first `colcon build`. The six `src/syncai_*`
directories `dependencies.repos` imports are gitignored: edit them in their own
checkouts, because the next `--force` import overwrites whatever is in them.

Only a missing `src/syncai_common` fails loudly (every package at once). A
missing `src/syncai_mapping`, `src/syncai_driver_manager`,
`src/syncai_robot_state`, `src/syncai_lio_bridge` or `src/syncai_camera_feeder`
builds fine and leaves the robot with nothing to save a map with, no bridge to
the gait controller, nothing publishing `RobotState`, no odometry source and no
camera feed — so
`scripts/build.sh` refuses to start on an empty checkout and names the `.repos`
file to import.

**Keeping them current.** A `git pull` of this workspace never updates those
checkouts — each tracks its repo's `dev`, and no commit here records where it
is. Use `scripts/sync.sh` instead of a bare `git pull`: it pulls the workspace,
fast-forwards every checkout that is on its pinned branch, clones any that is
missing, fetches only (and says so) for one on another branch, and needs only
git on the host.

**On an existing robot**, the pull that moves a package out deletes its
directory and nothing puts it back: run `scripts/sync.sh` (or
`vcs import < dependencies.repos`) before the next build. That also fetches and checks out the pin in every
checkout that is already there; add `--skip-existing` to clone only the missing
one and leave the others as they are.

#### The four Rust packages

`syncai_driver_manager`, `syncai_robot_state`, `syncai_lio_bridge` and
`syncai_camera_feeder` are `ament_cargo` (rclrs), and need nothing beyond `vcs import`: the robot image
carries the Rust toolchain and a ros2-rust underlay at `/opt/ros2_rust_underlay`
(rclrs plus the Humble interfaces rebuilt with Rust bindings, which the apt
copies lack), sourced between `/opt/ros/humble` and the workspace. An image
built before it has no underlay, and `scripts/build.sh` says so instead of
letting cargo fail. The C++ stack still builds without the Rust packages:

```bash
colcon build --symlink-install --packages-skip syncai_driver_manager syncai_robot_state syncai_lio_bridge syncai_camera_feeder
```

If configure fails on a missing `register_rs.cmake`, delete the stale `build/` +
`install/` copies of the interface packages that shadow the underlay. The pins
and the reasoning are in the `Dockerfile`'s underlay stanza and `CLAUDE.md`.

A Rust package that fails to link with `cannot find
-llivox_ros_driver2__rosidl_generator_c` (or another workspace interface
package's typesupport) was built once without `install/setup.bash` sourced and
then with it, on an image older than 2026-10-12: its rclrs build scripts kept
the old link-search paths. Rebuild the image (the underlay now patches them),
or until then `rm -rf build/<pkg>` and build again.

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
local edits — in all six checkouts, not only this one), then rebuild.

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

**From the host**, in a throwaway container (`docker-compose.build.yml`):

```bash
docker compose -f docker-compose.build.yml run --rm build                                  # everything
docker compose -f docker-compose.build.yml run --rm build --packages-select syncai_global_planner # colcon args pass through
BUILD_ROSDEP=off docker compose -f docker-compose.build.yml run --rm build                 # skip the rosdep report
```

It runs `scripts/build.sh`: check every vcs checkout (naming the `.repos` file
to import if one is empty), restore the `livox_ros_driver2` `package.xml` if
missing, `rosdep check` (report only — see below), then
`colcon build --symlink-install`, then delete the copy of the four Rust
nodes' source that cargo-ament-build always installs
(`install/<pkg>/share/<pkg>/rust/` and its `rust_packages` index marker — the
nodes never need it at runtime; a plain `colcon build` leaves it). Toggles: `BUILD_COLCON` (`1`/`0`),
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
bringup → pointlio + localizer → lio_bridge → planner + controller →
task_runner → driver_manager → robot_state → camera_feeder, with `sleep` offsets
standing in for the missing lifecycle manager. The session navigates on 3D maps
(`map/<name>/octomap.bt`) only: it has no map_server since 2026-10-28, so a 2D
gridmap is no longer served. Forbidden zones are `map/<name>/keepout.json`,
read by the 3D planner beside the tree — draw them as the forbidden area only,
the planner keeps the robot's radius off them itself.

```bash
# from the HOST: attach to whichever session is live (syncai-dev in AUTO,
# syncai-mapping in MANUAL — a hardcoded name is wrong half the time)
scripts/attach.sh                    # finds the live session, or a shell if none
scripts/attach.sh robot01 syncai-dev # explicit container / session
alias robot='~/SyncAI-Robot-Workspace/scripts/attach.sh'   # handy in ~/.bashrc

# inside the container: query / switch the operating mode (MAINTENANCE=0 MANUAL=1 AUTO=2 CALIBRATION=3)
ros2 service call /<robot_id>/get_mode    syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode "{mode: 1}"
```

CALIBRATION (`calib_lidar_camera.yaml`, session `syncai-calib`) runs bringup
with the livox driver on `PointCloud2` plus the camera feeder, for recording
lidar-camera calibration bags — no `driver_manager`, so lie the robot down
first. The backend's mode route does not offer it; switch with the service
call above. The procedure is `docs/lidar-camera-calibration.md`.

The mode is derived from which byobu session exists, never stored, so it stays
correct across a `sys_manager` restart. `switch_mode` kills every session before
building the target one and refuses to rebuild the mode that is already live
(in MANUAL that would drop the unsaved map).

This repo serves the Temporal UI at `:8081` and pgAdmin at `:5050`. The
operator API — REST + WebSockets on `http://<robot>:3000`, interactive docs at
`/docs` — is `SyncAI-Robot-Backend`, deployed as its own container on this host
and pointed at by the console (`SyncAI-Robot-Frontend`; nothing here serves
either). Navigation goals, mode switches, teleop and map saving all go through
that API. A raw `nav2_msgs/NavigateToPose` goal to `/<robot_id>/navigate_to_pose`
(or a `syncai_common/NavigateToGoal` one to `/<robot_id>/navigate_to_goal`,
which also says why a failed goal failed) works as well, and is the route that
needs nothing outside this repo.

**3D navigation and stairs.** Every map is 3D (`map/<name>/octomap.bt`) since
2026-10-28, when 2D navigation (a `gridmap.yaml` planned on by SmacPlanner2D)
was removed. `move.xml` / `dock.xml` ask for `planner_id="Octo3D"`
(OctomapPlanner3D on the map's `octomap.bt`) and `controller_id="FollowPath"`
(MPPI, 3D-aware since 2026-10-20; `FollowPathRPP3D` is the A/B), so a MOVE
plans on the octree and can climb the stairs the robot walked while mapping:

- `Octo3D` plans across floors over *stair links* taken from `poses.txt`; a
  goal's z picks the floor (z = 0 lands on 1F). The walk only says where the
  stair is: a single flight is laid as a row of straight links across its
  width (2026-10-19), and a switchback stairwell as straight flights joined
  by explicit 90° landing corners read off the map (2026-10-21). What cannot
  be read that way keeps the walked link, centred in the well.
- MPPI and RPP 3D match the robot to the path in 3D (its height is
  `<robot_id>/pointlio_body`'s z minus `lidar_height`, since `base_link` is
  planar), and a goal only counts within `goal_z_tolerance` (0.5 m) of the end
  pose's height.
- Where the path rises 0.3 m within 1 m, the controller enters **stair mode**
  (`syncai_util::StairMode`, one state machine for both): it stops about
  `stair_standoff` (1.0 m) before the stair, calls
  `set_policy_mode` with `stair_policy_mode` (1, HIMLOCO), stands
  `stair_policy_settle_time` (3 s), climbs at 0.5 m/s (MPPI on a 0.6 m
  carrot, RPP 3D on 0.4 m; MPPI's flat speed is 0.8 m/s, all unmeasured on
  the robot), and once the path `stair_exit_ahead_dist` (3.5 m) ahead is level
  stops again to switch back to the policy the controller reported on
  `<robot_id>/mode`. At a landing corner (2026-10-21) the robot stops 0.15 m
  before it and turns in place onto the next flight; the stair policy is kept
  across a U landing and handed back only on the top floor. A path that goes
  **down** is walked backwards (`stair_descend_backwards`), facing the risers
  as HIMLOCO descends, with a turn onto the path at the top and back round in
  the hold at the bottom. Without `set_policy_mode` it does not climb. The
  short 2F ramp of dp1f_1012 is taken in stair mode too. The state is on
  `<robot_id>/stair_mode`.
- **No obstacle avoidance on stairs**, by decision (2026-10-20): from the hold
  at the stair's foot to the hold past its top the climb reads no costmap.
  On the flat the local costmap runs the terrain layer, which sees low
  obstacles, drop-offs and ledges the old obstacle layer's height band missed,
  clears an obstacle as soon as a later ray passes through it, and judges a
  stair lethal, which also keeps MPPI off a stair it was not routed over.
- **Forbidden zones** are `map/<name>/keepout.json` beside the tree (polygons
  in the map frame, each with a z band naming its floor), read by `Octo3D`
  with the tree and reloaded with `planner_server/Octo3D/load_keepout`; the
  planner keeps the robot's radius off them itself, so draw the forbidden area
  only. The 2D keepout mask (`keepout.yaml` / `.pgm`) is read by nothing here.

Climbing has **not** been run on the robot: whether HIMLOCO takes the
stairwell, its landing turns and the backwards descent, and whether a switch
standing still is enough, are open. Test it attended, ESTOP in hand, from a
short goal near the stair foot. The tree checks its path once a second
(`IsPathValid`, 10 s `blocked_time`), judged by `Octo3D` on
its **dynamic overlay** — the lidar cloud in the map frame, every return
0.1–0.6 m above the surface it lies on a mark on that cell, raytrace-cleared,
forgotten after 30 s (2026-10-21) — so a person standing in the way for 10 s
gets a replan that detours round them; MPPI bends round one inside its local
costmap meanwhile, and at one that closes the way the four `FollowPath`
retries are the wait before the goal fails. The check judges only the floor
stretch the robot is on and never replans mid-stair. `syncai_controller/README.md` ("Paths with
heights", "Stairs"), `syncai_mppi_controller/README.md`,
`syncai_costmap_2d/README.md`, `syncai_global_planner/README.md` and
`syncai_task_runner/README.md` ("The behavior tree") have the details.

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

From a shell you get those files and no `octomap.bt`: `clean_map --octomap 1`
writes it, which is what the console's save button (`POST /api/v1/maps`)
drives; the backend's pcd → gridmap recipes still run for its 2D editor, but
nothing here navigates on a gridmap since 2026-10-28. Then point the robot at
the new map — `[map] name` in the instance INI, or the console's map switch,
which does the same live. Editing zones and renaming a map are backend routes
too, and all write into this repo's `map/` (renaming the map the stack is
running on is refused with 409 `map_active`, since the localizer and `Octo3D`
loaded its files at launch).

**Camera.** The nav session's `camera_feeder` window (`syncai_camera_feeder`)
owns the camera: it opens `/dev/syncai/camera0` and tees the MJPEG, undecoded,
to the WebRTC worker (RTP/JPEG on `127.0.0.1:5008`) and to
`image_raw/compressed`. The mapping session has no camera window. Nothing else
opens the camera: the V4L2 device takes one streaming opener. Both former
owners were removed in 2026-10 — the `vizionsdk_ros2` node `bringup.launch.py`
started under `use_camera:=true`, and the host-side
`scripts/publish_camera_crop.sh` (RTSP to a remote MediaMTX); git history has
both.

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

Four ROS-free gtests: `syncai_global_planner`'s `test_surface_map` (surface
map, search, stair links, dynamic overlay), `syncai_util`'s `test_stair_mode`,
`syncai_costmap_2d`'s `test_terrain_grid` and `syncai_localizer`'s
`test_localizability`; `syncai_sys_manager` adds `test_wifi_manager.py`. The
rest are the template ament linter tests, which are not enforced. The large
pytest suite that used to live here went with `syncai_backend`.

CI (`.github/workflows/ci.yml`) runs ruff and clang-format on changed lines,
then on an arm64 runner builds the robot image and the workspace, runs
`test_wifi_manager.py`, and smoke-tests every pane of the three session specs
against a generated 3D fixture map (`.github/scripts/smoke_test.sh`).

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
