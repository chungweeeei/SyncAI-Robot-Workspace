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
gateways/                  outbound integrations: ROS (robot, map), Temporal (workflow)
        │
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
| `odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 5 | composed with TF `map→odom` → telemetry WS |
| `motor_states` | `syncai_common/MotorStates` | BEST_EFFORT, depth 5 | reduced to `{joint: radians}` → telemetry WS |
| `plan` | `nav_msgs/Path` | **RELIABLE**, depth 1 | thinned to ≤512 xy pairs → telemetry WS |
| `pointlio/body_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | TF→`map`, thinned, packed → WS stream |

`plan` is the only RELIABLE subscription here. The others read 20 Hz feeds where
the next sample is 50 ms behind the one that was dropped; a plan arrives once per
BT replan (~3 s), so dropping one leaves the operator looking at a route the
robot has already left.

Two properties of `syncai_planner`'s publisher are worth knowing before debugging
a missing route: it skips the publish entirely while nothing is subscribed, and
its QoS is VOLATILE, so there is no last-value replay. After a backend restart
mid-run the route is blank until the next replan.

The map itself is *not* subscribed. `map` and `localizer/map_cloud` used to be
(both TRANSIENT_LOCAL, to match their latched publishers), but the map endpoints
read the saved files on request now — see the note at the top of
`routers/map.py`.

**`RobotState` carries more than `GET /api/v1/robot/state` exposes.**
`motor_status`' kinematic half (`q` / `dq` / `ddq` / `tau_est`), its source
`timestamp` and `localization_valid` are there for operators only.
`routers/robot.py` names its response fields one by one, and that is the *only*
thing keeping them out of a frozen public payload.

So that list is a **whitelist, not a mirror**: a field added to the message does
not appear in the response until somebody decides it should. `low_level_mode` is
the one field that decision has been made for — the gait controller's own state
machine, which the console has no other way to read because
`set_motion_key` / `set_policy_mode` are one-way UDP whose 200 only means a
datagram went out. It is decoded to **labels only** (`PPO` / `LOCOMOTION` / …, with `UNKNOWN` for a
code this backend cannot name). The controller's raw integers stay on the
`robot_state` topic, so `ros2 topic echo /<robot_id>/robot_state --field
low_level_mode` is what distinguishes MPC's unknown motion code from the
controller's startup sentinel — over REST they are the same `"UNKNOWN"`.

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
`syncai_sys_manager`) and `set_motion_key` / `set_policy_mode`
(`syncai_common/srv`, served by `syncai_driver_manager`).

**TF:** a `TransformListener` with `spin_thread=False` (it rides the node's own
executor rather than spawning another GIL-contending thread), used only to bring
`body_cloud` into the `map` frame.

## REST API

Interactive docs are generated by FastAPI at `http://<robot>:3000/docs`.

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | Liveness probe (always 200); `status` is `degraded` while `task_server` is not `running` (`connecting`/`dead` + `task_server_error`) |
| POST | `/api/v1/tasks` | Start a `RobotWorkflow`; body is `{id, steps[]}` (a legacy `timestamp` field is ignored). 409 while any task (direct or scheduled) is already running — one robot does one task at a time |
| GET | `/api/v1/tasks/{id}` | Overall status + per-step state (workflow query) |
| DELETE | `/api/v1/tasks/{id}` | Request cancellation; answers `status: CANCELING` — the final state (possibly still `COMPLETED`) comes from GET |
| POST | `/api/v1/schedules` | Create a Temporal schedule (cron **or** interval) |
| GET | `/api/v1/schedules` | List schedules with next run times |
| GET | `/api/v1/schedules/{id}` | Describe one schedule |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| POST | `/api/v1/saved_tasks` | Store a step list so it can be re-dispatched |
| GET | `/api/v1/saved_tasks` | List, optional `?map_name=` (that map's **plus** the map-independent ones) |
| GET | `/api/v1/saved_tasks/{id}` | One saved task, with its vertex references resolved |
| PUT | `/api/v1/saved_tasks/{id}` | Partial update; `steps` replaces the whole list |
| DELETE | `/api/v1/saved_tasks/{id}` | Delete |
| POST | `/api/v1/saved_tasks/{id}/schedule` | Freeze the current resolution into a Temporal schedule |
| GET | `/api/v1/robot/state` | Latest robot state (pose in degrees, wifi, battery, byobu-session `mode`, and `low_level_mode` — what the gait controller reports); 404 until localization is valid |
| WS | `/api/v1/robot/teleop` | Inbound manual-control channel: client sends `{vx, vy, wz}` JSON frames at ~10 Hz; the gateway clamps each axis to [-1, 1] and publishes it as-is (m/s / rad/s — full stick is 1.0, no scale-down below the clamp). Refused (`{"error": ...}` frame, socket stays open) while an autonomous MOVE is executing. A 0.5 s stale-input watchdog and the disconnect path both publish zero velocity — the driver manager has no cmd_vel watchdog of its own |
| POST | `/api/v1/robot/set_initial_pose` | Seed localization with a map-frame pose (degrees in, radians out); fire-and-forget |
| POST | `/api/v1/robot/set_motion_key` | Gait key `"0"`–`"5"`; `"4"` (ESTOP) is accepted but **not** forwarded — 200 with `sent: false` |
| POST | `/api/v1/robot/set_policy_mode` | Gait-controller policy index; only `0` (PPO) and `1` (HIMLOCO) are accepted |
| GET | `/api/v1/network/wifi/scan` | Scan visible networks (blocks up to 45 s) |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s) |
| GET | `/api/v1/maps` | The map directories on disk, with geometry and vertex counts |
| GET | `/api/v1/maps/{name}` | One map's summary |
| GET | `/api/v1/maps/{name}/image` · `/thumbnail` | The gridmap as a full-size / downscaled PNG, content-hash ETag'd |
| PUT | `/api/v1/maps/{name}/grid` | Write edited cells back (raw `application/octet-stream`); reloads map_server when the map is active |
| GET | `/api/v1/maps/{name}/pointcloud` | The saved `map.pcd`, packed binary |
| POST · GET | `/api/v1/maps/{name}/vertices` | Batch-create (single transaction) / list with an optional `?type=` filter |
| GET · PUT · DELETE | `/api/v1/maps/{name}/vertices/{id}` | Read / partial update / delete |
| WS | `/api/v1/robot/pointcloud/stream` | Live `body_cloud`, ~10 Hz |
| WS | `/api/v1/robot/telemetry/stream` | JSON frames keyed by `type`: `pose` (~20 Hz), `joints`, `path` (~0.333 Hz) |

The telemetry stream is the internal visualization channel and deliberately
shares no models with `GET /api/v1/robot/state` — that payload is a frozen
contract, this one may change shape freely. It is a separate socket from the
point cloud so a 360 kB cloud frame cannot head-of-line block pose; `path` rides
this one because a thinned route is ~8 kB every 3 s. An **empty** `path.points`
is a real sample meaning "no route" — the planner never publishes an empty plan,
so a route is cleared by a TTL in `TelemetryRepo` (arrival, cancellation and
abort are indistinguishable silence from the backend's side).

**Errors.** Routers raise domain exceptions from `exceptions.py`; handlers
registered in `server.py` map them to HTTP:

| Exception | Status |
|---|---|
| `NotFoundError` | 404 |
| `BadRequestError` | 400 |
| `UnauthorizedError` | 401 |
| `UpstreamError` | **502 Bad Gateway** (these all mean a downstream — Temporal or a ROS service — failed) |

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

### Saved task definitions

`POST /api/v1/tasks` creates *and dispatches* and persists nothing, and Temporal
is not a library: namespace `default` retains closed workflows for **one day** with
no archival, so a dispatched step list is gone by tomorrow. `saved_tasks` is where
the operator's re-dispatchable step lists live.

- **`steps` is one JSON column, not a child table.** This package has no
  migrations — the schema is whatever `create_all` produced — so a column list is
  a shape that can never be altered again, while a JSON array can grow an optional
  key. It is also the only place the step *order* is recorded, and a child table
  would still be rewritten whole on every edit, because that is what editing a
  step list is. **Forward-compat rule: only ever add optional keys to a stored
  step; never rename, retype, or repurpose one.**
- **A saved MOVE step keeps both a `vertex_id` and a `params` snapshot**, and every
  read reports `resolved_params` — the vertex's *current* pose when it still
  exists (`vertex_status: CURRENT`), the snapshot when it does not (`MISSING`).
  Moving a dock on the map therefore updates every saved route that references it.
  Resolution is server-side so the rule has one implementation; the client
  dispatches by sending `resolved_params` through the ordinary `POST /api/v1/tasks`.
- **Map scoping keys off "does it contain a MOVE", not "does it reference a
  vertex"** — a hand-typed `(x, y, theta)` is in a map's frame just as much as a
  vertex is. Any MOVE step ⇒ `map_name` required; no MOVE step ⇒ `map_name` must be
  absent, and the task runs anywhere. A task whose map is not the active one still
  saves (authoring for a map you are about to load is legitimate) and is reported
  with `map_matches_active: false` for the client to gate on.
- **Cross-field rules answer 400 with a sentence**, not 422 with a validation
  array: the array is unreadable to an operator, and a `PUT` may conflict with the
  *stored* row rather than with its own body, which no request-schema validator can
  see.

## Task orchestration (Temporal)

A task is an ordered list of steps. `RobotWorkflow` walks them one at a time and
dispatches by `StepType`:

| StepType | Activity | What it does |
|---|---|---|
| `MOVE` | `execute_move` | Send a `NavigateToPose` goal, poll to a terminal state, heartbeat each second |
| `STANDUP` / `LIEDOWN` | `execute_stand` / `execute_lie_down` | Send the motion key; fire-and-forget (see the note in `activities.py`) |

(`ARTIFACT` — conveyor pickup/drop over the artifact backend's REST API — was
removed in 2026-08 along with `gateways/artifact/`; saved tasks or schedules
that still carry an ARTIFACT step must be purged before deploying.)

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
- **Schedules use `SKIP` overlap policy**: a robot can only do one thing at a
  time, so a new run never starts while the previous one is still executing.
- Temporal normalises cron expressions into internal calendar specs, so the
  original trigger is stashed in the schedule **memo** and echoed back verbatim on
  get/list. The same memo carries `map_name` / `saved_task_id` / `saved_task_name`,
  because the memo is readable from `list_schedules()` while the start-workflow
  args are not.
- **A schedule's steps are readable from `describe()` but never from `list`.**
  `GET /api/v1/schedules/{id}` decodes them out of `ScheduleActionStartWorkflow.args`
  (raw `Payload` protos plus the description's `data_converter`);
  `GET /api/v1/schedules` always answers `steps: []`, because a schedule *list*
  element carries only the workflow type name and faking it would cost a describe
  RPC per row on first paint. A decode failure degrades to `[]` with a warning,
  never a 502 — same policy as the per-step workflow query.
- **A scheduled run's steps are frozen at registration.** The action args hold a
  concrete `WorkflowTask` and nothing re-reads it, so later vertex edits reach
  saved tasks and immediate dispatches but *not* an already-registered schedule.
  `POST /api/v1/saved_tasks/{id}/schedule` therefore refuses a task whose map is
  not active, and refuses one with a `MISSING` vertex — an unattended run does not
  get the snapshot fallback an operator watching the screen is allowed.

## Configuration

| Env var | Default | Used by |
|---|---|---|
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` | `localhost` | `database/postgres.py` |
| `POSTGRES_PORT` | `5432` | ditto |
| `POSTGRES_USER` | `syncrobotic` | ditto |
| `POSTGRES_PASSWORD` | `syncrobotic` | ditto |
| `SYNCAI_SYSTEM_INI` | `config/system.ini` | `helpers/system_config.py` (per-robot INI reads, e.g. `[map]`) |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

`[system] robot_id` is read from **`config/system.ini`** rather than the
environment (by the launch file), using the *relative* path — which works
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

In practice `NodeManager` starts it from `config/sessions/start_nav.yaml`, in the
`backend` window (pane 2, behind `robot_state`). `start_mapping.yaml` leaves it
out entirely — it is nav-oriented and hard-requires postgres.

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
