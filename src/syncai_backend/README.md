# syncai_backend

The robot's application-layer process: a **FastAPI REST/WebSocket server and an
rclpy ROS 2 node running inside one Python process**, plus a **Temporal worker**
that executes multi-step tasks.

It is the only thing the operator UI (`syncai_frontend`) talks to. Everything the
UI needs — robot state, the map, the point clouds, task submission, wifi setup —
is served from here, and every ROS interaction (nav goals, motion keys, wifi
services) happens on this side of the boundary.

```
                    HTTP :3000 / WebSocket
  syncai_frontend  ────────────────────────►  syncai_backend  ──── ROS 2 ────►  nav stack,
                                                    │                            robot_state,
                                                    │                            LIO / localizer,
                                                    ├──── gRPC ──►  Temporal     system_manager
                                                    └──── SQL  ──►  PostgreSQL
```

## Process model

`main.py` builds one `rclpy` node (`syncai_backend_node`) and starts two extra
threads from inside its constructor:

| Thread | What runs there |
|---|---|
| main | `MultiThreadedExecutor.spin()` — all ROS subscriptions, TF, service/action clients |
| uvicorn (daemon) | The FastAPI app on `0.0.0.0:3000` |
| Temporal worker (daemon) | Polls `<robot_id>.ROBOT_TASK_QUEUE`, runs `RobotWorkflow` + activities |

Two consequences worth remembering:

- The executor is **multi-threaded on purpose**. The point-cloud callback sits in
  its own `MutuallyExclusiveCallbackGroup` so a busy scan frame cannot starve the
  `robot_state` / `map` / TF callbacks.
- REST handlers that block — ROS service calls (up to ~70 s for wifi), psycopg2
  queries, OccupancyGrid→PNG encoding — are declared as **plain `def`, not
  `async def`**, so FastAPI runs them in its worker thread pool instead of
  stalling the event loop. Keep that distinction when adding endpoints.

## Layering

The layering is a convention, not something tooling enforces:

```
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; no business logic
        │
gateways/                  outbound integrations: ROS (robot), Temporal (workflow),
        │                  artifact backends (HTTP)
repositories/              state stores: in-memory caches + PostgreSQL CRUD
        │
database/                  SQLAlchemy engine + ORM models

subscribers/               ROS topics → repositories (the ingest side)
temporal/                  worker, RobotWorkflow, activities
helpers/                   OccupancyGrid→PNG, point-cloud downsample/transform/pack
```

Wiring is explicit: `main.py` constructs every repo/gateway/subscriber and passes
them down as constructor arguments. There is no DI container and no module-level
singleton — if a router needs something, it arrives through
`init_<x>_router(...)`.

`repositories/base.py` and `jobs/base.py` are abstract scaffolding that nothing
currently implements; the live repos are plain classes.

## robot_id, namespaces, and per-robot isolation

The launch file reads `[system] robot_id` from `config/system.ini` (bind-mounted
per robot from `config/instances/robotNN.ini`) and uses it as the **node
namespace**. The node then reads it back out of its own namespace:

```python
robot_id = self.get_namespace().strip("/") or "default_robot"
```

That single value scopes three things:

| Scoped by robot_id | Value |
|---|---|
| ROS topics/services/actions | relative names inherit the `/<robot_id>` namespace |
| PostgreSQL database | `<robot_id>_db` (auto-created on first connect) |
| Temporal task queue | `<robot_id>.ROBOT_TASK_QUEUE` |

**All ROS names in this package are relative** (`map`, `robot_state`,
`navigate_to_pose`, `pointlio/body_cloud`). Never hardcode `/<robot_id>/…` — a
subscriber with an absolute topic name is a bug that has already been fixed once
here.

TF frame names are *not* namespaced by ROS, so the cloud subscriber takes the
source frame from the message header and only pins the target frame (`map`).

## ROS interfaces

**Subscriptions**

| Topic | Type | QoS | Goes to |
|---|---|---|---|
| `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, depth 3 | `RobotRepo` → `GET /api/v1/robot/state` |
| `map` | `nav_msgs/OccupancyGrid` | **RELIABLE + TRANSIENT_LOCAL**, depth 1 | `MapRepo` cache → map info/image |
| `pointlio/body_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | TF→`map`, thinned, packed → WS stream |
| `localizer/map_cloud` | `sensor_msgs/PointCloud2` | **RELIABLE + TRANSIENT_LOCAL**, depth 1 | thinned once → `GET /api/v1/map/pointcloud` |

The two TRANSIENT_LOCAL profiles must match their latched publishers exactly,
otherwise this late-joining subscriber never receives the single retained sample
and the endpoint stays permanently 404.

**`RobotState` carries more than `GET /api/v1/robot/state` exposes.**
`motor_status` (per-joint temperatures, torques, error codes), `motor_timestamp`
and `localization_valid` are there for operators. `routers/robot.py` names its
response fields one by one, and that is the *only* thing keeping them out of a
frozen public payload — adding a field to the message must not add one to the
response.

`RobotStateSubscriber` **drops samples whose `localization_valid` is false**
before they reach `RobotRepo`. The publisher now emits on every tick, including
before the localizer has been relocalized, where `localization_status` is zeroed
rather than a real pose. Without that guard the endpoint would return 200 with
the robot apparently parked on the map origin instead of the 404 the frontend
gates its dashboard on.

**Action client:** `navigate_to_pose` (`nav2_msgs/NavigateToPose`) — served by
`syncai_task_runner`. `RobotGateway` keeps a goal-id → `MoveGoal` table so an
activity can poll status and cancel.

**Service clients:** `scan_wifi`, `connect_wifi` (`syncai_common/srv`, served by
`syncai_system_manager`) and `set_motion_key` (`syncai_common/srv`, served by
`syncai_driver_manager`).

**TF:** a `TransformListener` with `spin_thread=False` (it rides the node's own
executor rather than spawning another GIL-contending thread), used only to bring
`body_cloud` into the `map` frame.

## REST API

Interactive docs are generated by FastAPI at `http://<robot>:3000/docs`.

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | Liveness probe for the container healthcheck |
| POST | `/api/v1/tasks` | Start a `RobotWorkflow`; body is `{id, timestamp, steps[]}` |
| GET | `/api/v1/tasks/{id}` | Overall status + per-step state (workflow query) |
| DELETE | `/api/v1/tasks/{id}` | Request cancellation |
| POST | `/api/v1/schedules` | Create a Temporal schedule (cron **or** interval) |
| GET | `/api/v1/schedules` | List schedules with next run times |
| GET | `/api/v1/schedules/{id}` | Describe one schedule |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| GET | `/api/v1/robot/state` | Latest robot state (pose in degrees, wifi, battery, mode); 404 until localization is valid |
| GET | `/api/v1/network/wifi/scan` | Scan visible networks (blocks up to 45 s) |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s) |
| GET | `/api/v1/map/info` | resolution / width / height / origin |
| GET | `/api/v1/map/image` | Map info **plus** a `data:image/png;base64,…` render |
| GET | `/api/v1/map/pointcloud` | Static localizer map cloud, packed binary |
| POST | `/api/v1/map/vertices` | Batch-create vertices (single transaction) |
| GET | `/api/v1/map/vertices` | List; optional `?map_name=&type=` filters |
| GET · PUT · DELETE | `/api/v1/map/vertices/{id}` | Read / partial update / delete |
| WS | `/api/v1/robot/pointcloud/stream` | Live `body_cloud`, ~10 Hz |

**Errors.** Routers raise domain exceptions from `exceptions.py`; handlers
registered in `server.py` map them to HTTP:

| Exception | Status |
|---|---|
| `NotFoundError` | 404 |
| `BadRequestError` | 400 |
| `UnauthorizedError` | 401 |
| `InternalServerError` | **502 Bad Gateway** (these all mean a downstream — Temporal, a ROS service, an artifact backend — failed) |

**Point-cloud wire format** (both the WS stream and `GET /api/v1/map/pointcloud`):

```
[ uint32 LE point_count ][ float32 LE x, y, z ] * point_count      # map frame
```

The frontend reads this straight into a three.js `BufferGeometry`. The WS loop
polls a single-slot cache at 10 Hz and skips unchanged frames, so a slow client
drops stale frames instead of queuing them.

### Vertex vs. MapPoint

The REST vocabulary is **"vertex"** with a `VertexType` enum
(`GENERAL` / `ARTIFACT` / `CHARGER` / `HOME` / `WAITING`), while the ORM model and
repository still say `MapPoint` (table `map_vertices`). The mismatch is
intentional — no migration was done. `type` is validated at the REST boundary and
stored as a plain string.

## Task orchestration (Temporal)

A task is an ordered list of steps. `RobotWorkflow` walks them one at a time and
dispatches by `StepType`:

| StepType | Activity | What it does |
|---|---|---|
| `MOVE` | `execute_move` | Send a `NavigateToPose` goal, poll to a terminal state, heartbeat each second |
| `ARTIFACT` | `execute_artifact` | POST a command to the artifact backend's REST API, optionally poll its state until `live_info.phase` reaches `wait_for` |

Details that matter when editing this path:

- **Activities are synchronous** and run in a single-worker `ThreadPoolExecutor`,
  matching the one-thing-at-a-time reality of a robot. On cancellation Temporal
  *throws* `CancelledError` into the thread wherever it happens to be (often
  inside `time.sleep`), so cleanup lives in an `except CancelledError:` block, not
  in an `is_cancelled()` poll. `execute_move` wraps the `cancel_move` RPC in
  `activity.shield_thread_cancel_exception()` so the goal is really cancelled
  before the activity dies.
- **Per-step state is a workflow query** (`get_step_states`), not a database
  table. `GET /api/v1/tasks/{id}` degrades to an empty step list if the query
  fails (no worker polling yet), rather than erroring the whole request.
- **Retry semantics are deliberate.** An artifact command is *edge-triggered* —
  retrying re-fires it — so a transient state-poll failure is swallowed and
  polling continues to the deadline instead of failing the activity.
- **Schedules use `SKIP` overlap policy**: a robot can only do one thing at a
  time, so a new run never starts while the previous one is still executing.
- Temporal normalises cron expressions into internal calendar specs, so the
  original trigger is stashed in the schedule **memo** and echoed back verbatim on
  get/list.

**Standing decision:** `ARTIFACT` activities call the artifact REST API directly.
The behavior-tree route is reserved for a future need for tick-level parallelism.

The artifact registry comes from the `[artifacts]` section of `config/system.ini`
(`artifact_id = base_url`), read once at startup. Missing section ⇒ `ARTIFACT`
steps fail with a clear error.

## Configuration

| Env var | Default | Used by |
|---|---|---|
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` | `localhost` | `database/postgres.py` |
| `POSTGRES_PORT` | `5432` | ditto |
| `POSTGRES_USER` | `syncrobotic` | ditto |
| `POSTGRES_PASSWORD` | `syncrobotic` | ditto |
| `SYNCAI_SYSTEM_INI` | `config/system.ini` | Artifact registry lookup |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

Two things are read from **`config/system.ini`** rather than the environment:
`[system] robot_id` (by the launch file) and `[artifacts]` (by
`ArtifactGateway`). Both use the *relative* path `config/system.ini`, which works
because every process in this stack runs with the workspace root as its cwd.

Postgres connection is retried 20× at 5 s intervals on startup, and the
`<robot_id>_db` database is created if absent — the backend can therefore come up
before the `postgres` container is ready.

CORS is currently wide open (`allow_origins=["*"]`).

## Build and run

Builds run **inside the robot container** — see the workspace `CLAUDE.md`.

```bash
colcon build --packages-select syncai_backend --symlink-install
source install/setup.bash
```

Python deps are **not** managed by rosdep (jammy has no reliable key for
fastapi); `requirements.txt` is the single source of truth and both the dev and
`backend-runtime` Docker stages install from it:

```bash
pip install -r src/syncai_backend/requirements.txt
```

Run it:

```bash
ros2 launch syncai_backend backend.launch.py                     # namespaced from config/system.ini
ros2 launch syncai_backend backend.launch.py system_config:=config/instances/robot02.ini
ros2 run syncai_backend backend                                  # no namespace -> default_robot
```

In practice `scripts/byobu_session.sh` starts it in the `state_backend` window.

> `setup.py` installs **compiled bytecode only** (`InstallNoSource`): after a
> normal install it byte-compiles the package and deletes the `.py` sources from
> the install space, so deployments ship no source. The step self-disables when
> the installed modules are symlinks, so `--symlink-install` developer builds are
> unaffected.

## Tests

```bash
colcon test --packages-select syncai_backend
colcon test-result --verbose
# or, inside the container, from src/syncai_backend/:
pytest test/
```

Tests must run where `rclpy` / `nav_msgs` / `syncai_common` / OpenCV are
importable. The database layer is exercised against an **in-memory SQLite**
engine (`StaticPool`, so every session shares one connection), so no PostgreSQL
server is needed. Alongside the unit tests are the standard ament linters
(`test_copyright`, `test_flake8`, `test_pep257`).

## Gotchas

- **Changing a backend ROS parameter requires restarting the backend.** Nothing
  here re-reads parameters at runtime.
- A relative topic name is not optional — see the namespace section above.
- Latched topics (`map`, `localizer/map_cloud`) need exactly-matching QoS.
- `map -> pointlio_odom` only exists **after** you call `/localizer/relocalize`.
  Until then the live cloud stream is silent; the subscriber logs once on the
  first drop and once on recovery rather than per frame, so check the log if the
  3D view is empty.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
