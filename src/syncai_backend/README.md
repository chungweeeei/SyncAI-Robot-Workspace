# syncai_backend

The robot's application-layer process: a **FastAPI REST/WebSocket server and an
rclpy ROS 2 node running inside one Python process**, plus a **Temporal worker**
that executes multi-step tasks.

It is the only thing the operator UI (`syncai_frontend`) talks to. Everything the
UI needs — robot state, the map, the point clouds, task submission, mode
switching, wifi setup, speech — is served from here, and every ROS interaction
(nav goals, motion keys, wifi services, `switch_mode`, `save_maps`) happens on
this side of the boundary.

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

- The executor is **multi-threaded on purpose**. Each point-cloud callback
  (`body_cloud`, and the multi-MB `pgo/map_cloud` merges) sits in its own
  `MutuallyExclusiveCallbackGroup` so a busy cloud frame cannot starve the
  `robot_state` / telemetry / TF callbacks — or each other.
- REST handlers that block — ROS service calls (up to ~70 s for wifi), psycopg2
  queries, OccupancyGrid→PNG encoding — are declared as **plain `def`, not
  `async def`**, so FastAPI runs them in its worker thread pool instead of
  stalling the event loop. Keep that distinction when adding endpoints.

## Layering

The layering is a convention, not something tooling enforces:

```
interfaces/rest/routers/   HTTP + WS surface; pydantic schemas; no business logic
        │
gateways/                  outbound integrations: ROS (robot, map), Temporal (workflow),
        │                  speech (tts: kokoro-onnx → aplay — no ROS in it at all)
repositories/              state stores: in-memory caches + PostgreSQL CRUD
        │
database/                  SQLAlchemy engine + ORM models

subscribers/               ROS topics → repositories (the ingest side)
temporal/                  worker, RobotWorkflow, activities
helpers/                   occupancy_grid (OccupancyGrid→PNG), pointcloud (downsample /
                           transform / pack), pgm, pcd_to_gridmap (z-band recipe),
                           traversable (traversability recipe), system_config (INI reader)
```

`gateways/tts` is a gateway like `robot` / `map` even though its downstream is
an inference session plus the speaker rather than a ROS service: the REST router
and the Temporal worker's `SPEAK` activity share **one long-lived owner** for the
lazily-loaded (~310 MB) kokoro model, and that instance's internal lock is what
keeps a scheduled SPEAK step and a manual `POST /api/v1/tts/speak` from talking
over each other. `main.py` constructs exactly one.

There are **two pcd → gridmap recipes** in `helpers/`, and the default is
z-band: `pcd_to_gridmap.py` slices the cloud into floor / obstacle height bands
(trinary occupied / free / unknown) and then reverts free cells not connected to
the keyframe trajectory back to unknown; `traversable.py` segments the floor by
intensity / normal / height and projects it, leaving **no unknown cells** — it
runs only when an operator asks for it via `grid/convert`. There is deliberately
no automatic pick between them; the workspace `CLAUDE.md` ("Backend
architecture") records why and what each conversion writes to disk.
`traversable.py` is the **only** module that imports open3d, and nothing imports
it at module scope — `_start_grid_conversion` imports it inside the conversion
thread's `try`, so a backend start never pays the ~100 MB import and an
`ImportError` lands as a per-map failure instead of a bare thread traceback.
Keep `pcd_to_gridmap.py` open3d-free.

Wiring is explicit: `main.py` constructs every repo/gateway/subscriber and passes
them down as constructor arguments. There is no DI container and no module-level
singleton — if a router needs something, it arrives through
`init_<x>_router(...)`.

`repositories/base.py` and `jobs/base.py` are abstract scaffolding that nothing
currently implements; the live repos are plain classes.

## robot_id, namespaces, and per-robot isolation

The launch file reads `[system] robot_id` from the system INI — by default the
**absolute** `~/robot_ws/config/system.ini` (bind-mounted per robot from
`config/instances/robotNN.ini` inside the container), overridable with the
`system_config:=` launch argument — and uses it as the **node namespace**. The
node then reads it back out of its own namespace:

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
| `pointlio/body_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | TF→`map`, thinned, packed → WS `pointcloud/stream` |
| `pgo/map_cloud` | `sensor_msgs/PointCloud2` | BEST_EFFORT, **depth 1** | already in `map`; stride-capped, packed → WS `pointcloud/map/stream` (mapping mode only) |

`plan` is the only RELIABLE subscription here. The others read 20 Hz feeds where
the next sample is 50 ms behind the one that was dropped; a plan arrives once per
BT replan (~3 s), so dropping one leaves the operator looking at a route the
robot has already left.

Two properties of `syncai_planner`'s publisher are worth knowing before debugging
a missing route: it skips the publish entirely while nothing is subscribed, and
its QoS is VOLATILE, so there is no last-value replay. After a backend restart
mid-run the route is blank until the next replan.

The saved map is *not* subscribed. `map` and `localizer/map_cloud` used to be
(both TRANSIENT_LOCAL, to match their latched publishers), but the map endpoints
read the files on disk on request now — see the note at the top of
`routers/map.py`. The one map-shaped cloud that **is** subscribed is
`pgo/map_cloud`, pgo's merged "map so far" during a MANUAL (mapping) session:
every message is a complete loop-closure-corrected replacement of the last, so
`MapCloudSubscriber` uses depth 1 (a queued older merge is never worth
delivering), skips TF (the points were placed with corrected global poses at
merge time) and skips voxel downsampling (pgo already voxelised at its publish
resolution). pgo publishes subscriber-gated, so this subscription is what
un-gates it. The topic simply does not exist under `AUTO`, which is why the
stream is silent on a navigating robot.

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

**Service clients.** `RobotGateway`: `scan_wifi`, `connect_wifi`, `switch_mode`
(`syncai_common/srv`, served by `syncai_sys_manager`) and `set_motion_key` /
`set_policy_mode` (`syncai_common/srv`, served by `syncai_driver_manager`).
`MapGateway`: `map_server/load_map` (`syncai_map_server`, so an edited or
re-converted gridmap reaches the running map_server) and `pgo/save_maps`
(FAST-LIO2's `pgo_node`, the only thing that serialises a mapping run). The map
gateway is separate on purpose — the map router has no business holding a
handle that can command the robot to move.

**Publishers:** `initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, the
localization seed) and `cmd_vel` (`geometry_msgs/Twist`, the teleop channel's
output — also where the zero-velocity watchdog publishes).

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
| GET | `/api/v1/active_tasks` | What is executing on this robot's Temporal queue right now, *whoever* started it (direct or schedule), with `source` / `schedule_id`. Not `/tasks/active` — that would shadow `/tasks/{id}` |
| POST | `/api/v1/schedules` | Create a Temporal schedule (cron **or** interval) |
| GET | `/api/v1/schedules` | List schedules with next run times |
| GET | `/api/v1/schedules/{id}` | Describe one schedule |
| DELETE | `/api/v1/schedules/{id}` | Delete |
| POST | `/api/v1/schedules/{id}/pause` · `/resume` | Pause / unpause |
| POST | `/api/v1/task_templates` | Store a step list so it can be re-dispatched |
| GET | `/api/v1/task_templates` | List, optional `?map_name=` (that map's **plus** the map-independent ones) |
| GET | `/api/v1/task_templates/{id}` | One template, with its vertex references resolved |
| PUT | `/api/v1/task_templates/{id}` | Partial update; `steps` replaces the whole list |
| DELETE | `/api/v1/task_templates/{id}` | Delete |
| POST | `/api/v1/task_templates/{id}/schedule` | Freeze the current resolution into a Temporal schedule |
| GET | `/api/v1/robot/state` | Latest robot state (pose in degrees, wifi, battery, byobu-session `mode`, and `low_level_mode` — what the gait controller reports); 404 until localization is valid |
| POST | `/api/v1/robot/mode` | `switch_mode` on `syncai_sys_manager`. A real switch kills the byobu session **this backend is a pane of**, so the client usually sees a dropped connection, not a body — treat that as success-in-progress. The body reliably arrives only for the no-op (already in that mode) and a refusal |
| WS | `/api/v1/robot/teleop` | Inbound manual-control channel: client sends `{vx, vy, wz}` JSON frames at ~10 Hz; the gateway clamps each axis to [-1, 1] and publishes it as-is (m/s / rad/s — full stick is 1.0, no scale-down below the clamp). Refused (`{"error": ...}` frame, socket stays open) while an autonomous MOVE is executing. A 0.5 s stale-input watchdog and the disconnect path both publish zero velocity — the driver manager has no cmd_vel watchdog of its own |
| POST | `/api/v1/robot/set_initial_pose` | Seed localization with a map-frame pose (degrees in, radians out); fire-and-forget |
| POST | `/api/v1/robot/set_motion_key` | Gait key `"0"`–`"5"`; `"4"` (ESTOP) is accepted but **not** forwarded — 200 with `sent: false` |
| POST | `/api/v1/robot/set_policy_mode` | Gait-controller policy index; only `0` (PPO) and `1` (HIMLOCO) are accepted |
| GET | `/api/v1/network/wifi/scan` | Scan visible networks (blocks up to 45 s) |
| POST | `/api/v1/network/wifi/connect` | Connect via `nmcli` (blocks up to 70 s) |
| GET | `/api/v1/maps` | The map directories on disk, with geometry and vertex counts |
| GET | `/api/v1/maps/{name}` | One map's summary |
| POST | `/api/v1/maps` | Save the current mapping run: `pgo/save_maps` into a new map dir, then the z-band pcd→gridmap conversion in a background thread (`grid_pending`). Mapping mode only in practice — pgo is the producer |
| PATCH | `/api/v1/maps/{name}` | Rename a map: `{name}` moves `map/<old>/` to `map/<new>/` and re-keys its `map_vertices` and `task_templates` rows. 409 `map_active` for the map the stack is running on (the localizer and map_server loaded its files at launch and nothing rewrites the INI), 409 `conversion_running` while its gridmap is being rebuilt, 409 `name_taken` if the new name exists. Temporal schedule memos keep the old `map_name` label — they are display-only and are not re-registered |
| POST | `/api/v1/maps/{name}/grid/convert` | (Re)build the gridmap from `map.pcd` with a chosen recipe (`z_band` default / `traversability`), optional `z_band_offsets` or `gap_fill_size`, `debug` for intermediate clouds. 409 `conversion_running` (try later) or 409 `gridmap_hand_edited` (confirm with `overwrite_edits`; the edited grid survives as `gridmap_prev.pgm`). Reloads map_server when the map is active |
| GET | `/api/v1/maps/{name}/image` · `/thumbnail` | The gridmap as a full-size / downscaled PNG, content-hash ETag'd |
| PUT | `/api/v1/maps/{name}/grid` | Write edited cells back (raw `application/octet-stream`); reloads map_server when the map is active |
| GET | `/api/v1/maps/{name}/pointcloud` | The saved `map.pcd`, packed binary |
| POST · GET | `/api/v1/maps/{name}/vertices` | Batch-create (single transaction) / list with an optional `?type=` filter |
| GET · PUT · DELETE | `/api/v1/maps/{name}/vertices/{id}` | Read / partial update / delete |
| GET | `/api/v1/tts/voices` | The voice ids the loaded kokoro model carries |
| POST | `/api/v1/tts/synthesize` | Render `{text, voice, speed}` and return the WAV (`audio/wav`) without playing it |
| POST | `/api/v1/tts/speak` | Same body, played on the robot speaker; blocks for the utterance (`duration` in the response). `unknown voice` is a 400, everything else (weights, onnxruntime, aplay) a 502. Text is English only, ≤1000 chars; `speed` 0.5–2.0 |
| WS | `/api/v1/robot/pointcloud/stream` | Live `body_cloud`, ~10 Hz |
| WS | `/api/v1/robot/pointcloud/map/stream` | pgo's merged "map so far" cloud, every few seconds at most, mapping mode only; each frame replaces the whole layer |
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
| `ConflictError` | **409**, with an optional machine-readable `code` next to `detail` (`conversion_running` / `gridmap_hand_edited` on the re-convert route). The frontend confirms-and-retries only the latter, and matching on prose that exists to be reworded was the rejected alternative |
| `UnauthorizedError` | 401 |
| `UpstreamError` | **502 Bad Gateway** (these all mean a downstream — Temporal or a ROS service — failed) |

**Point-cloud wire format** (both WS streams and `GET /api/v1/maps/{name}/pointcloud`):

```
[ uint32 LE point_count ][ float32 LE x, y, z ] * point_count      # map frame
```

The frontend reads this straight into a three.js `BufferGeometry`. Both WS
streams share one `_pump`: it is **frame-driven**, waiting on the single-slot
repo's notification rather than polling (polling cost ~50 ms of queueing latency
and ~5 % dropped frames from two unsynchronised 10 Hz clocks beating), and the
single-slot repo still does the dropping, so a slow client sees the newest frame
instead of a backlog.

### Vertex vs. MapPoint

The REST vocabulary is **"vertex"** with a `VertexType` enum
(`GENERAL` / `ARTIFACT` / `CHARGER` / `HOME` / `WAITING`), while the ORM model and
repository still say `MapPoint` (table `map_vertices`). The mismatch is
intentional — no migration was done. `type` is validated at the REST boundary and
stored as a plain string.

### Task templates

`POST /api/v1/tasks` creates *and dispatches* and persists nothing, and Temporal
is not a library: namespace `default` retains closed workflows for **one day** with
no archival, so a dispatched step list is gone by tomorrow. `task_templates` is
where the operator's re-dispatchable step lists live. The prefix is
`/api/v1/task_templates`, not `/api/v1/tasks/templates`, because the latter
would collide with `/api/v1/tasks/{id}` (see the include order note in
`server.py`).

- **`steps` is one JSON column, not a child table.** This package has no
  migrations — the schema is whatever `create_all` produced — so a column list is
  a shape that can never be altered again, while a JSON array can grow an optional
  key. It is also the only place the step *order* is recorded, and a child table
  would still be rewritten whole on every edit, because that is what editing a
  step list is. **Forward-compat rule: only ever add optional keys to a stored
  step; never rename, retype, or repurpose one.**
- **A template's MOVE step keeps both a `vertex_id` and a `params` snapshot**, and every
  read reports `resolved_params` — the vertex's *current* pose when it still
  exists (`vertex_status: CURRENT`), the snapshot when it does not (`MISSING`).
  Moving a dock on the map therefore updates every template that references it.
  Resolution is server-side so the rule has one implementation; the client
  dispatches by sending `resolved_params` through the ordinary `POST /api/v1/tasks`.
- **Map scoping keys off "does it contain a MOVE", not "does it reference a
  vertex"** — a hand-typed `(x, y, theta)` is in a map's frame just as much as a
  vertex is. Any MOVE step ⇒ `map_name` required; no MOVE step ⇒ `map_name` must be
  absent, and the task runs anywhere. A template whose map is not the active one still
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
| `SPEAK` | `execute_speak` | `TtsGateway.speak()` — synthesise and play on the robot speaker, blocking for the utterance. `SpeakParams`: `text` (1–1000 chars, English only), `voice` (default `af_heart`; list at `GET /api/v1/tts/voices`), `speed` (0.5–2.0) — the same constraints as the REST route, because both drive one gateway |

(`ARTIFACT` — conveyor pickup/drop over the artifact backend's REST API — was
removed in 2026-08 along with `gateways/artifact/`; task templates or schedules
that still carry an ARTIFACT step must be purged before deploying.)

Details that matter when editing this path:

- **Activities are synchronous** and run in a single-worker `ThreadPoolExecutor`,
  matching the one-thing-at-a-time reality of a robot. On cancellation Temporal
  *throws* `CancelledError` into the thread wherever it happens to be (often
  inside `time.sleep`), so cleanup lives in an `except CancelledError:` block, not
  in an `is_cancelled()` poll. `execute_move` wraps the `cancel_move` RPC in
  `activity.shield_thread_cancel_exception()` so the goal is really cancelled
  before the activity dies.
- **`SPEAK` cannot heartbeat.** `execute_speak` sits in one blocking gateway call
  (synthesis, then `aplay` for the whole utterance, plus the one-time model load),
  so the 3 s `heartbeat_timeout` the other activities run under would kill every
  attempt before its first heartbeat. The workflow therefore drops the heartbeat
  for SPEAK and relies on a **5-minute `start_to_close`** alone — much shorter
  than the heartbeating activities' hour, because a dead worker holding a SPEAK
  step would otherwise go unnoticed for that hour. The same fact makes it
  effectively not cancellable mid-utterance: without heartbeats the worker never
  learns of the cancel, so a cancelled task finishes the sentence it is on.
- **Per-step state is a workflow query** (`get_step_states`), not a database
  table. `GET /api/v1/tasks/{id}` degrades to an empty step list if the query
  fails (no worker polling yet), rather than erroring the whole request.
- **Schedules use `SKIP` overlap policy**: a robot can only do one thing at a
  time, so a new run never starts while the previous one is still executing.
- Temporal normalises cron expressions into internal calendar specs, so the
  original trigger is stashed in the schedule **memo** and echoed back verbatim on
  get/list. The same memo carries `map_name` / `task_template_id` / `task_template_name`,
  because the memo is readable from `list_schedules()` while the start-workflow
  args are not. (`saved_task_id` / `saved_task_name` are still accepted on
  *read* for schedules registered before the rename.)
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
  `POST /api/v1/task_templates/{id}/schedule` therefore refuses a template whose map is
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
| `SYNCAI_SYSTEM_INI` | `~/robot_ws/config/system.ini` | `helpers/system_config.py` (per-robot INI reads, e.g. `[map]`) |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

`[system] robot_id` is read from the system INI rather than the environment (by
the launch file). Unlike the rest of the stack, this package resolves that INI
by an **absolute** default path (`~/robot_ws/config/system.ini`, the workspace
inside the robot container), in both `launch/backend.launch.py` and
`helpers/system_config.py`: the old relative `config/system.ini` only worked
because every entrypoint happened to run from the workspace root, and the
backend is also started from tests and shells that do not. Override with the
`system_config:=` launch argument or `SYNCAI_SYSTEM_INI` respectively.

There are **no ROS parameters** in this package — nothing calls
`declare_parameter`. Everything configurable is the INI, the environment, or a
constant with a comment explaining it.

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
ros2 launch syncai_backend backend.launch.py system_config:=config/instances/robot01.ini
ros2 run syncai_backend backend                                  # no namespace -> default_robot
```

In practice `NodeManager` starts it from **both** session specs,
`config/sessions/start_nav.yaml` and `start_mapping.yaml`, in the `backend`
window (pane 2, behind `robot_state`, `sleep: 2`). Earlier revisions left it out
of the mapping spec as nav-oriented; it is there now because the operator
console *is* the mapping UI — mode switching, teleop, the live cloud view and
the save-map call all go through it. It still hard-requires postgres, which
lives in the infra compose stack and is up regardless of which session exists.

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

`test/` holds ~40 files, roughly one per router / gateway / subscriber / repo /
helper (`ls src/syncai_backend/test/` is the index). They must run where `rclpy`
/ `nav_msgs` / `syncai_common` / OpenCV are importable. The database layer is
exercised against an **in-memory SQLite** engine (`StaticPool`, so every session
shares one connection), so no PostgreSQL server is needed. The conversion tests
additionally need scipy (`test_pcd_to_gridmap.py` imports the z-band helper,
which imports `scipy.ndimage`) and open3d (`test_traversable.py`
`importorskip`s `helpers.traversable` per test, so its cases skip rather than
fail where open3d is absent). Alongside the unit tests are the standard ament
linters (`test_copyright`, `test_flake8`, `test_pep257`).

## Gotchas

- **Configuration is read once, at startup.** There are no ROS parameters to
  change, but the INI (`[map]`, `robot_id`), `.env` and the environment are all
  read during construction — a change to any of them needs a backend restart.
- A relative topic name is not optional — see the namespace section above.
- **Nothing here subscribes to a latched topic any more.** `map` and
  `localizer/map_cloud` (both TRANSIENT_LOCAL) went away with the file-based map
  endpoints. `pgo/map_cloud` is *not* latched — it is VOLATILE, BEST_EFFORT,
  subscriber-gated and exists only under `MANUAL`; a silent
  `pointcloud/map/stream` on a navigating robot is the expected state, not a QoS
  mismatch. If a latched subscription ever comes back, remember the durability
  has to match the publisher exactly or nothing arrives.
- `map -> pointlio_odom` only exists **after** you call `/localizer/relocalize`.
  Until then the live cloud stream is silent; the subscriber logs once on the
  first drop and once on recovery rather than per frame, so check the log if the
  3D view is empty.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
