# syncai_ros_mcp

An MCP ([Model Context Protocol](https://modelcontextprotocol.io)) server that
runs as a **ROS 2 node**, exposing the robot's live ROS 2 graph (topics,
services) and the `syncai_backend` REST API (tasks, maps) as MCP tools via
[FastMCP](https://github.com/jlowin/fastmcp). It is what lets an LLM agent
inspect and drive one robot through a single HTTP endpoint instead of a shell.

This is a standard `ament_python` ROS 2 package (ROS 2 Humble). It is
**vendored** in this workspace — the source is committed here, not pulled in as
a submodule. It began life as the submodule `chungweeeei/SyncAI-ROS-MCP` and was
folded in because it is developed only against this workspace, and the
submodule indirection meant every change needed two commits and left the
workspace pinning a stale pointer.

## How it runs

One process, two threads:

- `rclpy.spin()` owns the **main thread**. The node is `syncai_mcp_server_node`
  (`mcp_server_node.py`), and the topic/service tools read the live graph
  through it, so the graph they see is the one `spin()` keeps up to date.
- FastMCP runs on a **background daemon thread**, started from
  `Node.__init__` via `start_mcp_server()` in `server.py` (~31-43):
  `mcp.run(transport="http", host="0.0.0.0", port=8000)`. The server is named
  `syncai-ros-mcp`. Daemon, so a `Ctrl-C` on `spin()` takes the HTTP server
  down with it rather than leaving a listener behind.

`server.py` mirrors `syncai_backend`'s `interfaces/rest/server.py` split:
`init_mcp_server()` builds the `FastMCP` instance and registers the four tool
modules; `start_mcp_server()` runs it.

The task and map tools do **not** touch ROS at all — they are thin `requests`
clients for the backend on port 3000 (`tools/_backend.py`), so they only work
while `syncai_backend` is up (it is a pane of both session specs).

## Connecting

The MCP endpoint is

```
http://<robot>:8000/mcp
```

(streamable-HTTP transport, no auth — the robot container runs with
`network_mode: host`, so this is reachable from anywhere that can reach the
robot). Point an MCP client at it, e.g. for Claude Code:

```bash
claude mcp add --transport http syncai-robot01 http://robot01.local:8000/mcp
```

`robot01.local` resolves because `syncai_sys_manager` publishes
`<robot_id>.local` over mDNS.

## Configuration

| Setting | Where | Default | Meaning |
|---|---|---|---|
| `SYNCAI_BACKEND_BASE_URL` | environment variable | `http://localhost:3000` | Base URL of `syncai_backend` for the task and map tools. Loopback by default because the MCP node and the backend run in the same container. |
| `HTTP_TIMEOUT` | constant in `tools/_backend.py` | `10.0` s | Bound on every backend call so a hung request cannot wedge a tool. |
| host / port | arguments of `start_mcp_server()` | `0.0.0.0` / `8000` | Not exposed as ROS parameters or CLI flags — edit the call in `mcp_server_node.py` if they ever need to change. |

There is no params YAML and no ROS parameter surface. Note that the node is
**not** namespaced by `robot_id`: it is started by hand with `ros2 run`, not by
a launch file, so it lives in the root namespace and its topic tools take
fully-qualified names (`/robot01/cmd_vel`).

## Tools

Every tool returns a plain dict; failures come back as `{'error': ...}` (for
the backend-facing tools also `status_code` and the backend's `detail`), never
as a raised exception. Read-only tools carry `readOnlyHint`; the ones that
change the robot or its data carry `destructiveHint`.

### `tools/topics.py` — live ROS 2 graph

| Tool | What it does |
|---|---|
| `get_topics()` | List every topic on the graph with its type(s). |
| `get_topic_type(topic)` | Message type(s) of one topic. |
| `get_topic_details(topic)` | Type plus the publishers and subscribers (node name, namespace, QoS) of one topic. |
| `get_message_details(message_type)` | Field-by-field structure of a message type, nested types expanded. |
| `publish_once(topic, msg_type, msg)` | Publish a single message from this node. **Destructive** — `/robot01/cmd_vel` is a real motion command. |
| `subscribe_once(topic, msg_type, timeout=…, expects_image=…)` | Subscribe and return the first message received; images are returned as an image content block. |

### `tools/services.py` — live ROS 2 graph

| Tool | What it does |
|---|---|
| `get_services()` | List every service on the graph. |
| `get_service_type(service)` | Service type of one service. |
| `get_service_details(service)` | Request/response structure plus the nodes providing the service. |
| `call_service(service_name, service_type, request, timeout=…)` | Call a service with a request dict (field names as shown by `get_service_details`). **Destructive** — this reaches `switch_mode`, `set_motion_key`, `relocalize`, … |

### `tools/tasks.py` — `syncai_backend` REST (`/api/v1/tasks`)

| Tool | What it does |
|---|---|
| `create_task(task_id, steps)` | `POST /api/v1/tasks`: queue an ordered list of steps. Step types are the backend's `StepType`: `MOVE {x, y, theta°}`, `STANDUP`, `LIEDOWN` (no params), `SPEAK {text, voice, speed}`. |
| `get_task_state(task_id)` | `GET /api/v1/tasks/{id}`: overall status plus per-step status. |
| `cancel_task(task_id)` | `DELETE /api/v1/tasks/{id}`: request cancellation (answers `CANCELING`; poll `get_task_state` for the outcome). **Destructive.** |

`GET /api/v1/active_tasks` (what is running on this robot's queue right now)
has no tool yet.

### `tools/maps.py` — `syncai_backend` REST (`/api/v1/maps`)

All routes are nested under the map's directory name; vertex bodies carry **no**
`map_name` (the URL owns it), and moving a vertex between maps is a delete plus
a create.

| Tool | What it does |
|---|---|
| `list_maps()` | `GET /api/v1/maps`: the map catalogue. The entry with `active: true` is the map the stack was launched with. |
| `get_map_info(map_name)` | `GET /api/v1/maps/{name}`: one map's summary — grid geometry (resolution, origin `{x, y, yaw}`, width, height; `null` until converted), `has_pointcloud`, `grid_converting`, `vertex_count`, … |
| `get_map_image(map_name)` | `GET /api/v1/maps/{name}/image`: the gridmap as a PNG, returned as an image content block. |
| `create_map_vertices(map_name, vertices)` | `POST /api/v1/maps/{name}/vertices`: create one or more `{name, type, x, y, theta}` vertices; `type` is `GENERAL` / `ARTIFACT` / `CHARGER` / `HOME` / `WAITING`. |
| `list_map_vertices(map_name, type=None)` | `GET /api/v1/maps/{name}/vertices[?type=…]`; an unknown map is a 404, not an empty list. |
| `get_map_vertex(map_name, vertex_id)` | `GET /api/v1/maps/{name}/vertices/{id}`. |
| `update_map_vertex(map_name, vertex_id, name=…, type=…, x=…, y=…, theta=…)` | `PUT /api/v1/maps/{name}/vertices/{id}`: change only the fields passed. |
| `delete_map_vertex(map_name, vertex_id)` | `DELETE /api/v1/maps/{name}/vertices/{id}`. **Destructive.** |

## Package layout

```
src/syncai_ros_mcp/             # ROS 2 package "syncai_ros_mcp"
├── package.xml                 # ament manifest (build_type: ament_python)
├── setup.py                    # entry point + InstallNoSource install command
├── setup.cfg
├── resource/syncai_ros_mcp
├── syncai_ros_mcp/
│   ├── mcp_server_node.py      # ROS node `syncai_mcp_server_node` + `main`
│   ├── server.py               # FastMCP instance + background-thread startup
│   └── tools/
│       ├── topics.py           # topic tools    (live ROS graph)
│       ├── services.py         # service tools  (live ROS graph)
│       ├── tasks.py            # task tools     (syncai_backend REST)
│       ├── maps.py             # map tools      (syncai_backend REST)
│       └── _backend.py         # shared HTTP client for the backend API
└── test/                       # ament copyright / flake8 / pep257 tests
```

## Dependencies

Declared in `package.xml` and resolved by `rosdep`:

- `rclpy` — the node and the graph introspection.
- `rosidl_runtime_py` — message/service type lookup and dict ⇄ message
  conversion for `publish_once` / `subscribe_once` / `call_service`.
- `python3-requests` — the backend HTTP client.

**Not** covered by `rosdep`: **FastMCP** is pip-only (no rosdep key). Install
it into the same Python interpreter `ros2 run` uses (the system `python3`):

```bash
python3 -m pip install "fastmcp>=3.4.4"
```

`setup.py`'s `install_requires` lists it too, but colcon does not install
Python dependencies, so the pip step is still yours. Recreating the robot
container wipes it along with every other hand-installed dependency.

## Packaging

`setup.py` uses the same `InstallNoSource` install command as `syncai_backend`
and `syncai_sys_manager` (~12-38): after the normal install it byte-compiles
the package to legacy sourceless `.pyc` and deletes the `.py` files from the
install space, so a deployment ships no source. It self-disables when the
installed modules are symlinks, so `--symlink-install` developer builds keep
their sources.

The entry point is `mcp_server_node = syncai_ros_mcp.mcp_server_node:main`
(~67-69).

## Build & run

Builds run **inside the robot container**, whose workspace is `~/robot_ws`
(`/home/syncrobotic/robot_ws`), not on the host — see the root `CLAUDE.md`.

```bash
docker compose exec robot01 bash
colcon build --symlink-install --packages-select syncai_ros_mcp
source install/setup.bash
ros2 run syncai_ros_mcp mcp_server_node
```

The MCP node is **not started by anything**: it is in neither session spec
(`config/sessions/start_nav.yaml`, `start_mapping.yaml`) nor in
`docker-compose.robots.yml`. Run it by hand when an agent needs it, and expect
it to be gone after a container restart.

## Test

```bash
colcon test --packages-select syncai_ros_mcp
colcon test-result --verbose
```

Only the ament linters (`ament_copyright`, `ament_flake8`, `ament_pep257`).
