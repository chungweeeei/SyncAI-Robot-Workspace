# SyncAI-ROS-MCP

An MCP ([Model Context Protocol](https://modelcontextprotocol.io)) server that
runs as a **ROS 2 node**, exposing ROS 2 introspection and control as MCP tools
via [FastMCP](https://github.com/jlowin/fastmcp).

This is a standard `ament_python` ROS 2 package (ROS 2 Humble). The MCP server
and the ROS node run in one process: `rclpy.spin()` owns the main thread and the
FastMCP server runs on a background daemon thread, so the tool callbacks always
read a graph that is being kept up to date.

## Package layout

```
src/syncai_ros_mcp/             # ROS 2 package "syncai_ros_mcp"
├── package.xml                 # ament manifest (build_type: ament_python)
├── setup.py                    # entry point + install metadata
├── setup.cfg
├── resource/syncai_ros_mcp
├── syncai_ros_mcp/
│   ├── mcp_server_node.py       # ROS node + `main` entry point
│   ├── server.py                # FastMCP instance + background-thread startup
│   └── tools/
│       ├── topics.py            # topic tools    (live ROS graph)
│       ├── services.py          # service tools  (live ROS graph)
│       ├── tasks.py             # task tools     (syncai_backend REST)
│       ├── maps.py              # map tools      (syncai_backend REST)
│       └── _backend.py          # shared HTTP client for the backend API
└── test/                        # ament copyright / flake8 / pep257 tests
```

## Dependencies

- **ROS 2 Humble** (`rclpy`) — declared in `package.xml`.
- **FastMCP** — a pip-only dependency (not in rosdep). Install it into the same
  Python interpreter that runs the node (the system `python3` used by
  `ros2 run`):

  ```bash
  python3 -m pip install "fastmcp>=3.4.4"
  ```

## Build & run

From the workspace root (`/workspace`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select syncai_ros_mcp
source install/setup.bash

ros2 run syncai_ros_mcp mcp_server_node
```

## Test

```bash
colcon test --packages-select syncai_ros_mcp
colcon test-result --verbose
```
