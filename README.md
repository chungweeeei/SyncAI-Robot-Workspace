# SyncAI Robot Workspace

A ROS 2 Humble navigation stack for the SyncAI robot. The core packages are a
**non-lifecycle port of Navigation2** — the nav2 servers (AMCL, map server,
costmap, planner, controller, BT navigator) have been re-implemented as plain
`rclcpp::Node`s instead of lifecycle nodes, so the stack starts and runs without
a lifecycle manager. Navigation is driven by a Behavior Tree.

> Built and run inside a Docker container (`ubuntu:22.04` + ROS 2 Humble).
> RMW is CycloneDDS configured via `config/cyclonedds.xml`.

## Architecture

The navigation pipeline mirrors nav2's action-server topology:

```
                NavigateToPose
   client  ─────────────────────▶  syncai_bt_navigator   (ticks the behavior tree)
                                          │
                   compute_path_to_pose   │   follow_path
                        ┌─────────────────┴─────────────────┐
                        ▼                                     ▼
                 syncai_planner                        syncai_controller
              (NavFn global plan)                 (Regulated Pure Pursuit)
                        │                                     │
                        └──────────► syncai_costmap_2d ◄──────┘
                                     (global / local costmaps)

   syncai_amcl  ─ pose estimate (map → odom TF)      syncai_map_server ─ static map
```

- **`syncai_nav_core`** defines the abstract plugin interfaces (planner /
  controller / costmap layers); planner and controller load their algorithms as
  `pluginlib` plugins against those interfaces.
- **`syncai_behavior_tree`** provides the BT engine and navigation BT nodes;
  `syncai_bt_navigator` is the action server that loads a tree (see
  `behavior_trees/*.xml`) and ticks it.

## Packages

| Package | Role |
|---|---|
| `syncai_common` | Shared msg / srv / action interfaces for the stack |
| `syncai_util` | Header-only helpers (geometry, occupancy-grid values) |
| `syncai_nav_core` | Header-only abstract interfaces for nav plugins (port of `nav2_core`) |
| `syncai_amcl` | Adaptive Monte Carlo Localization (non-lifecycle) |
| `syncai_map_server` | Map server + map saver nodes (non-lifecycle) |
| `syncai_costmap_2d` | Global / local costmaps with layered plugins |
| `syncai_planner` | Global planner action server (`compute_path_to_pose`) with the NavFn plugin merged in |
| `syncai_controller` | Controller action server (`follow_path`) with the Regulated Pure Pursuit plugin merged in |
| `syncai_behavior_tree` | BT engine + navigation BT nodes (port of `nav2_behavior_tree`) |
| `syncai_bt_navigator` | BT navigator action server (`NavigateToPose`) |
| `syncai_backend` | Robot backend node exposing a RESTful API (FastAPI) bridged to ROS 2 |
| `syncai_task_runner` | Task runner nodes (scaffold) |

### Third-party packages (`src/third-party/`)

| Package | How it is managed |
|---|---|
| `behaviortree_cpp_v3` | **Git submodule**, pinned to upstream tag `3.8.8` (`BehaviorTree/BehaviorTree.CPP`). Not modified locally. |
| `ros2_laser_scan_merger` | **Vendored** (source committed directly) — locally modified, so it is maintained in this repo rather than tracked upstream. |

To upgrade the submodule, check out the new tag inside it and commit the new pointer:

```bash
cd src/third-party/behaviortree_cpp_v3 && git fetch --tags && git checkout <tag>
cd - && git add src/third-party/behaviortree_cpp_v3 && git commit -m "chore: bump behaviortree_cpp_v3 to <tag>"
```

## Repository layout

```
.
├── src/                     # colcon packages (see tables above)
│   └── third-party/         # submodule + vendored deps
├── config/cyclonedds.xml    # CycloneDDS RMW configuration
├── map/                     # test maps (testmap, warehouse) + cartographer pbstream
├── Dockerfile               # ubuntu:22.04 + ROS 2 Humble + nav deps
├── docker-compose.yml       # `robot` service (host networking, X11, workspace mount)
├── .devcontainer/           # VS Code "Reopen in Container" config
└── .env                     # local env vars / secrets (gitignored — never commit)
```

## Getting started

### 1. Clone (with submodules)

`behaviortree_cpp_v3` is a submodule, so clone recursively:

```bash
git clone --recursive <repo-url>
# already cloned without --recursive:
git submodule update --init --recursive
```

### 2. Start the container

```bash
# .env supplies UID/GID, ROS_DOMAIN_ID, DISPLAY, etc.
docker compose up -d robot
docker compose exec robot bash
```

The workspace is mounted at `/home/syncrobotic/robot_ws`. ROS 2 and the workspace
overlay are auto-sourced in every shell via `~/.bashrc`.

> Alternatively, open the folder in VS Code and **Reopen in Container**
> (`.devcontainer/`), which mounts the workspace at `/workspace`.

### 3. Build

Run from the workspace root inside the container:

```bash
rosdep install --from-paths src --ignore-src -r -y   # resolve declared deps
colcon build --symlink-install
source install/setup.bash

# build a single package
colcon build --packages-select syncai_bt_navigator
```

### 4. Run the navigation stack

Each server has its own launch file. Bring them up (each in its own shell, or
compose into one launch file). Launch files default to the `robot01` namespace
and read their parameters from each package's `params/` directory:

```bash
ros2 launch syncai_map_server map_server.launch.py
ros2 launch syncai_amcl amcl.launch.py
ros2 launch syncai_planner planner_server.launch.py
ros2 launch syncai_controller controller_server.launch.py
ros2 launch syncai_bt_navigator bt_navigator.launch.py
```

Common launch arguments:

- `namespace` — node namespace (default `robot01`); prefixes relative topics/actions.
- `params_file` — override the parameters YAML.

Then send a `NavigateToPose` goal (e.g. from RViz2 or a CLI action client) to the
`bt_navigator`.

### 5. Tests

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Notes

- **Non-lifecycle by design** — there is no lifecycle manager; servers are plain
  nodes and come up active immediately.
- **`use_sim_time`** is set *only* in the params YAML files (not overridden in
  launch), so it works correctly when driving the stack from a simulator.
- **CycloneDDS** is the RMW; the container points `CYCLONEDDS_URI` at
  `config/cyclonedds.xml`. Host networking is used so DDS discovery reaches other
  hosts on the LAN.
- **Secrets** — `.env` holds local secrets and is gitignored. Do not commit it.
