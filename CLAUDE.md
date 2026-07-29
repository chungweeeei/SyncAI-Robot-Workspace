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
| `syncai_bringup` | `bringup.launch.py` — robot_state_publisher over `description/G23.urdf` (carries the `lidar_top` mount extrinsic the LIO bridge needs) + the Livox MID360 driver. The old 2D/AMCL `bringup_2d.launch.py` (laser scan merger) was removed. |

### Hardware & system

| Package | Role |
|---|---|
| `syncai_driver_manager` | **UDP bridge to the gait controller.** Sends `cmd_vel` (with per-direction velocity-scale correction — the gait controller tracks commands asymmetrically), receives ASCII telemetry, and owns the safe-shutdown path (engages a safety lock and commands MODE X / lie down). |
| `syncai_robot_state` | Aggregates odom / battery / wifi / motor_states / TF into `syncai_common/RobotState` at 10 Hz. Also derives the `state` field: `UNINITIALIZED` (no pose) / `WARNING` (battery <20%, latched with hysteresis) / `IDLE`. Reports only — no threshold here commands the robot. |
| `syncai_system_manager` | Python: wifi, mDNS, map, and system managers behind ROS services |

### Application layer

| Package | Role |
|---|---|
| `syncai_backend` | Python. FastAPI **and** rclpy in one process (`MultiThreadedExecutor`), port **3000**. Temporal worker for task orchestration. |
| `syncai_frontend` | Next.js + shadcn/ui + raw three.js, port **3001** |

### Third-party (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | Submodule, pinned to upstream tag `3.8.8`. Unmodified. |
| `FASTLIO2_ROS2` | Submodule → `chungweeeei/SyncAI-Fast-LIO2` (branch `dev`). Contains LIO + PGO + HBA + `localizer`. |
| `livox_ros_driver2`, `Livox-SDK2` | Submodules (MID360 driver) |
| `ros2_laser_scan_merger` | **Vendored** (source committed) — locally modified |

## The robot_id convention

Every launch file reads `[system] robot_id` from **`config/system.ini`** via a
`read_robot_id()` helper, falling back to `default_robot` with a warning. The
resolved value is used as the **node namespace**.

`config/system.ini` is tracked but **empty**. Per-robot identity comes from
`config/instances/robotNN.ini`, which docker-compose bind-mounts over
`config/system.ini` inside the container. That file also carries
`[artifacts]` endpoints, `[initial_pose]`, and `[map]` pcd/gridmap paths.

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

## Running the stack

`scripts/byobu_session.sh` (FAST-LIO2) is the real entrypoint. It builds a byobu
session with one window per subsystem, encodes the required startup ordering as
`sleep` offsets, and taps each pane's output into a size-capped `multilog`
directory under `log/stack/<robot_id>/<name>/`. Read logs back with
`scripts/tailog.sh`. The 2D / AMCL variant of this script was retired along with
`bringup_2d.launch.py`.

Two panes are **pre-typed but not executed** — you hit Enter yourself:
the `localizer/relocalize` service call (3D localization is not active until you
run it, with the robot at its known start pose) and `rviz2`.

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

Changing a backend ROS parameter requires restarting the backend.

## Frontend (`syncai_frontend`)

Next.js (dev server on port 3001), shadcn/ui components, **raw three.js** for
the 3D point-cloud view (no react-three-fiber). WebRTC streaming was considered
and deferred.

`src/syncai_frontend/AGENTS.md` warns that the pinned Next.js version has
breaking changes relative to model training data — read the relevant guide in
`node_modules/next/dist/docs/` before writing Next.js code.

## Infrastructure

- **`docker-compose.yml`** — shared services: `postgres` (5432, bind-mounted to
  `./data/postgres`), `pgadmin` (5050), `temporal` (7233), `temporal_ui` (8081).
- **`docker-compose.robots.yml`** — robot containers under two profiles:
  `real` (`robot01`, `robot02`; `network_mode: host`, loopback-only unicast DDS)
  and `sim` (`robot01-sim` … `robot03-sim`; dual-homed on a `syncai-lan`
  macvlan so they reach Temporal/Postgres while DDS runs on the macvlan).
  Robot containers also bind-mount the host D-Bus socket (so `nmcli` reaches the
  host NetworkManager — needs `apparmor=unconfined` + sudo) and the avahi socket
  (so `libnss-mdns` resolves `*.local`).
- **CycloneDDS** is the RMW, configured by hand: `config/cyclonedds.xml` (LAN /
  macvlan) and `config/cyclonedds_standalone.xml` (loopback + explicit unicast
  peers). Gotchas that have bitten before: a single interface with
  `multicast=false` disables multicast **globally**; peer pings only use the
  top-priority interface; unicast `<Peers>` require the remote side to have
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
with the Python package templates (`syncai_system_manager` also has
`test_wifi_manager.py`).

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
