# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

**syncai_ros_mcp** is an MCP (Model Context Protocol) server that runs as a
ROS 2 node (`syncai_mcp_server_node`), exposing the live ROS 2 graph and the
`syncai_backend` REST API as MCP tools. Standard `ament_python` package, ROS 2
Humble. `README.md` has the tool inventory, layout and threading model.

It is **vendored** in this workspace — the source is committed here, not a
submodule. It started as `chungweeeei/SyncAI-ROS-MCP`; there is no upstream to
sync with any more, edit it in place.

## Key facts

- **One process, two threads.** `rclpy.spin()` owns the main thread; FastMCP
  runs `mcp.run(transport="http", host="0.0.0.0", port=8000)` on a daemon
  thread started from `Node.__init__` (`server.py`). Clients connect to
  `http://<robot>:8000/mcp`.
- **Two kinds of tool.** `tools/topics.py` and `tools/services.py` read the
  graph through the node. `tools/tasks.py` and `tools/maps.py` never touch ROS:
  they are `requests` clients for the backend via `tools/_backend.py`, base URL
  `SYNCAI_BACKEND_BASE_URL` (default `http://localhost:3000`), `HTTP_TIMEOUT`
  10 s. Backend routes change — check
  `src/syncai_backend/syncai_backend/interfaces/rest/routers/` before touching a
  path string here; the map tools were once left pointing at routes that no
  longer existed.
- **Every tool returns a dict**, with failures as `{'error': ...}` rather than
  exceptions; keep that contract when adding one.
- **Not namespaced.** The node is started by `ros2 run`, not a launch file, so
  it sits in the root namespace and topic/service tools take fully-qualified
  names (`/<robot_id>/cmd_vel`). It is started by nothing automatic — not in a
  session spec, not in compose.

## Environment and build

Builds run **inside the robot container** at `~/robot_ws`
(`/home/syncrobotic/robot_ws`), not on the host — see the root `CLAUDE.md`,
and do not run `colcon build` unless asked. In a non-interactive shell source
`/opt/ros/humble/setup.bash` (and `install/setup.bash` after a build) first.

```bash
colcon build --symlink-install --packages-select syncai_ros_mcp
source install/setup.bash
ros2 run syncai_ros_mcp mcp_server_node

colcon test --packages-select syncai_ros_mcp && colcon test-result --verbose
```

**FastMCP** is pip-only (no rosdep key) — install it into the interpreter
`ros2 run` uses: `python3 -m pip install "fastmcp>=3.4.4"`. `rosdep install`
covers the rest (`rclpy`, `rosidl_runtime_py`, `python3-requests`).

Lint/style are enforced by the ament tests under `test/` (`ament_flake8`,
`ament_pep257`, `ament_copyright`); `python3 -m flake8 --max-line-length=99`
over `syncai_ros_mcp/` is the quick local check.
