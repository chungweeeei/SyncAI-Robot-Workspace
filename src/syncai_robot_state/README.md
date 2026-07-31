# syncai_robot_state

One node, `syncai_robot_state`, that aggregates the robot's scattered status
sources into a single `syncai_common/RobotState` at **10 Hz** on the relative
topic `robot_state` (so it lands on `<robot_id>/robot_state`). One publisher, one
timer, and `syncai_backend` as its only consumer — which makes this node the thing
that decides what the operator UI can show.

```
        TF: map → <robot_id>/base_link  ─────┐   (from syncai_lio_bridge / syncai_amcl)
        odom          (nav_msgs/Odometry) ───┤
        battery_state (BatteryState)  ───────┼──►  syncai_robot_state
        wifi_status   (WifiStatus)    ───────┤            (10 Hz)
        motor_states  (MotorStates)   ───────┘               │
                                                             │ robot_state
                                                             ▼
                                                      syncai_backend
                                                             │
                                                             ▼
                                              GET /api/v1/robot/state
```

**The message carries more than that REST payload exposes.** `motor_status`
(per-joint temperatures, torques, error codes), `motor_timestamp` and
`localization_valid` exist for operators; `routers/robot.py` names its response
fields one by one, and that is the only thing keeping them out of a frozen
third-party contract.

It aggregates and derives very little: a yaw extraction, a unit conversion, the
localization-validity flag, and one latched threshold (low battery → `WARNING`).
It holds no state beyond the latest sample of each input and that one latch.

> An outward-facing variant on an absolute, fleet-wide `/robot_state` topic was
> built here and then reverted. A single DDS domain hosts several robots, so a
> shared root topic interleaves them, and every per-robot consumer in this
> workspace — `syncai_backend` included, which scopes its DB and Temporal queue by
> `robot_id` — is built for exactly one robot. Re-proposing it needs a consumer
> that genuinely wants the whole fleet on one topic.

## What goes into each field

Exposed through `GET /api/v1/robot/state`:

| Field | Source | Notes |
|---|---|---|
| `timestamp` | `now().seconds()` | **Seconds**, not milliseconds |
| `robot_id` | `robot_id` parameter | Set from the INI by the launch file |
| `map` | `map` parameter | See the gotcha — this is a path, not a name |
| `mode` | — | **Hardcoded to `AUTO`** (`{TODO}` in the source) |
| `localization_status.position` | TF `global_frame → base_frame` | x/y/z plus `yaw` in **radians**. Zeroed when the lookup fails |
| `localization_status.velocity` | `odom.twist.twist.linear.x` | Forward speed only |
| `battery_status.battery_percentage` | `battery_state.percentage × 100` | `BatteryState` is 0–1, this field is 0–100 |
| `network_status.wifi_info` | `wifi_status`, flattened to JSON | See below |

On the topic but **not** in the REST payload:

| Field | Source | Notes |
|---|---|---|
| `state` | TF validity + battery | `UNINITIALIZED` / `WARNING` / `IDLE`, most-severe-first — see below. `RUNNING` and `ERROR` still `{TODO}`; `CHARGING` is not derivable |
| `localization_valid` | TF lookup result | `false` ⇒ `localization_status` is zeroed, not a real pose |
| `motor_status` | `motor_states.states`, verbatim | `MotorState[]`: per-joint `temperature`, `tau_est`, `error` (plus `q`/`dq`). Empty while `syncai_driver_manager` is down |
| `motor_timestamp` | `motor_states.timestamp`, verbatim | **Nanoseconds** — unconverted, unlike `timestamp` above. `0` if no sample ever arrived |

That second table is a convention, not a type guarantee: the router names its
response fields explicitly, and nothing else stops a future field from leaking into
a frozen public contract.

Per-joint temperatures and motor error codes were already on the `motor_states`
wire but reached nothing: the backend's telemetry WebSocket subscribes that topic
and keeps only `q`. This node is what makes them observable.

**Do not build motion out of `motor_status.q`** — 10 Hz is a diagnostic rate, not
an animation rate, and `timestamp` has only whole-second resolution so you cannot
even order the samples. The high-rate joint channel is the backend telemetry
WebSocket, which subscribes `motor_states` directly for exactly that reason.

**`wifi_info` is a JSON string,** not a typed sub-message: the latest
`WifiStatus` is flattened into `{"ssid", "bssid", "rssi", "ip_address",
"mac_address"}` with nlohmann/json. Before the first `wifi_status` arrives the
empty json dumps to the literal string `"null"`, which is why the backend parses
this field defensively and falls back to an empty object.

## The `state` derivation

Three of the six `RobotStatus` values are emitted, evaluated most-severe-first:

| `state` | Condition |
|---|---|
| `UNINITIALIZED` | `map → base_link` TF unavailable |
| `WARNING` | battery below the low-battery threshold (latched) |
| `IDLE` | otherwise |

**`UNINITIALIZED` outranks `WARNING` on purpose.** "We don't know where the robot
is" is the more fundamental fact: a low battery is worth reporting, but not at the
cost of hiding that `localization_status` in the same message is a zero
placeholder. `localization_valid` is the precise answer for consumers that only
care about pose trust; `state` is the coarse rollup, and both come from one TF
lookup so they cannot disagree.

`RUNNING` and `ERROR` are not derived yet. **`CHARGING` cannot be**: the driver
hardcodes `BatteryState.power_supply_status` to `UNKNOWN`
(`syncai_driver_manager.cpp:301`), and the only other candidate is the sign of
`current`, whose convention is undocumented in both this port and the reference
implementation. Deriving it is a hardware-observation task, not a coding one.

### Low battery uses hysteresis

Entering below `low_battery_warn_percentage` (20%), clearing only above
`low_battery_clear_percentage` (25%). Without the gap, a pack sitting on the
threshold would flip the state ten times a second at the 10 Hz publish rate, and
a state that flaps is one nobody can act on. A clear value at or below the warn
value is rejected at startup and both fall back to 20/25.

20% is not a new number — it is the threshold in `syncai_driver_manager`'s
unwired `soc < 20%` safety TODO, the one the reference GaitMPC bridge acts on, and
the one the frontend status strip already hardcodes for its battery colour. This
node is where the *judgement* now lives.

**This node only reports.** Crossing the threshold does not lie the robot down or
block `cmd_vel`; `syncai_driver_manager::triggerSafeShutdown()` still has zero
call sites and who owns that actuation is deliberately still open.

### Two guards against a low battery that isn't one

`updateHealthLatches()` holds the latch at its current value — neither setting nor
clearing — unless it has a battery sample whose percentage is above zero:

1. **No sample ≠ 0%.** `battery_percentage` reports `0.0` when no
   `battery_state` has arrived, so a robot whose `driver_manager` simply has not
   started would otherwise latch `WARNING` immediately.
2. **`percentage == 0` is most likely a bad packet.** The driver parses the BMS
   section with a bare `strtod` lambda instead of the validating
   `parseFloatToken` it uses for every other section, so a non-numeric or empty
   token silently publishes `0.0`. A robot at a genuine 0% is not powered on to be
   asked about.

### Where the latches live, and why not in `buildState()`

`onTimer()` calls `updateHealthLatches()` and then `buildState()`, so the latches
and the message they end up in describe the same tick. `buildState()` stays a pure
read of them.

Keeping the two apart gives any future dwell counter or rate-limited transition
exactly one place it can live, ticking once per publish. It also survives the node
growing a second publisher again: a latch advanced inside `buildState()` would then
step once per *call* rather than once per period.

The latches are not under `mutex_`: they are touched only from the timer, which has
its own `MutuallyExclusive` callback group. `mutex_` guards the sample caches,
which the subscription callbacks write from other threads.

### Known limits

- **No battery-sample staleness detection.** `latest_battery_` is the last sample
  and never expires. If `driver_manager` dies the latch freezes on its last
  verdict: died at 15% → `WARNING` forever (errs safe), died at 80% → `IDLE`
  forever (misleading). `RobotStatus` has no `STALE` value and `UNINITIALIZED` is
  taken by localization, so there is nowhere good to put it.
- **`WARNING` carries no reason.** With one condition, `ros2 topic echo` shows
  `battery_status` alongside it, so the cause is visible. A second `WARNING`
  condition (motor over-temperature, say) will need a reason field or bitmask.
- **The frontend still has its own copy of the 20% rule**
  (`status-strip.tsx:27-31`, `< 20 → warn`, `< 40 → caution`, no hysteresis). It
  cannot read `state` — the REST payload does not expose the field — so this
  duplication is knowingly left in place.

## Missing TF is published, not suppressed

`onTimer()` looks up `global_frame → base_frame` through
`syncai_util::getCurrentPose`. That lookup **fails on the 3D stack until
`/localizer/relocalize` has been called**, because `map → odom` does not exist
before that.

This node used to abort the whole tick on failure, which meant no `robot_state`
topic existed at all in that window — no battery, no wifi, and (once they were
added) no joint temperatures, precisely when an operator is trying to work out
why the robot will not localize. It now publishes regardless, with
`localization_valid = false` and `state = UNINITIALIZED`.

`localization_status` is left **zeroed**, deliberately not held at the last known
pose: a stale pose with no age attached gets read as a live one, whereas the map
origin is an obviously suspicious value.

`GET /api/v1/robot/state` still answers **404** "Robot state is not available
yet" during this window, and the frontend still gates its dashboard on that 404.
That is now enforced one layer up — `RobotStateSubscriber` drops samples whose
`localization_valid` is false instead of writing them to `RobotRepo` — rather
than by this node staying silent.

So: a `robot_state` topic with no data now really does mean the node is dead.
Check for `TF map-><id>/base_link unavailable` in the log to distinguish
"not localized" from "not running".

The other four inputs are not gated either: a missing odom, battery, wifi or
motor message just leaves that field at its zero / `"null"` / empty default.

## Threading

`main.cpp` uses a `MultiThreadedExecutor`, and the 10 Hz timer gets its **own
`MutuallyExclusive` callback group** so the latch update, TF lookup and message
build run independently of the four (lightweight) subscription callbacks. The four
cached samples are guarded by one mutex, taken briefly in each callback and twice
per tick (once in `updateHealthLatches()`, once in `buildState()`).

**The TF lookup is gated by a non-blocking `canTransform()`.** This is load
bearing at 10 Hz: `syncai_util::getCurrentPose` ends in
`tf_buffer.transform(..., transform_tolerance)`, which *blocks for the whole
tolerance* when the transform is absent — which it is for as long as the
localizer has not been relocalized. Ten builds a second each stalling 0.1 s
would saturate the callback group indefinitely, and
`transformPoseInTargetFrame` logs its failure with an **unthrottled**
`RCLCPP_ERROR`, so the pre-relocalize state would flood the byobu multilog
capture at 10 Hz. With the gate, the common failure costs a lock and a map
lookup, and `transform_tolerance` only applies to the rare race where the
transform disappears between the check and the call.

`main.cpp` also holds the node in a named variable rather than passing a
temporary to `add_node()` — the executor only keeps a `weak_ptr`, so a temporary
would be destroyed before `spin()` and the process would exit immediately.

## Interfaces

All names relative, so they inherit the `<robot_id>` namespace.

| Direction | Topic | Type | QoS |
|---|---|---|---|
| Publish | `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `odom` | `nav_msgs/Odometry` | SensorData |
| Subscribe | `battery_state` | `sensor_msgs/BatteryState` | SensorData |
| Subscribe | `wifi_status` | `syncai_common/WifiStatus` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `motor_states` | `syncai_common/MotorStates` | SensorData |

`odom` comes from `syncai_lio_bridge`, `battery_state` and `motor_states` from
`syncai_driver_manager`, `wifi_status` from `syncai_sys_manager`. The
backend's subscriber matches the BEST_EFFORT publisher — and so must any new one:
a best-effort publisher cannot satisfy a RELIABLE subscriber, so subscribing with
default QoS receives nothing at all.

## Parameters

| Parameter | Default | Set by the launch file |
|---|---|---|
| `robot_id` | `""` | `[system] robot_id` from the INI |
| `map` | `""` (yaml: `dp2f_full`) | `[map] map` from the INI, when present |
| `global_frame` | `map` | — (stays unprefixed) |
| `base_frame` | `base_link` | `<robot_id>/base_link` |
| `transform_tolerance` | `0.1` | — |
| `publish_rate` | `10.0` Hz | — (params file only) |
| `low_battery_warn_percentage` | `20.0` % | — (params file only) |
| `low_battery_clear_percentage` | `25.0` % | — (params file only) |

All of these are **load-time only**: this node has no
`add_on_set_parameters_callback`, so `ros2 param set` appears to succeed and
changes nothing.

The launch file reads the same `config/system.ini` as everything else, and
mirrors `map_server.launch.py` in also picking up `[map] map`. It appends its
overrides *after* the params file so they win, and only overrides `map` when the
INI actually provides one.

## Running

```bash
ros2 launch syncai_robot_state robot_state.launch.py
ros2 launch syncai_robot_state robot_state.launch.py \
    system_config:=config/instances/robot02.ini
```

Started in the byobu sessions' `state_backend` window, alongside the backend.

```bash
ros2 topic echo /<robot_id>/robot_state --once   # joint temps, state, localization_valid
ros2 topic hz /<robot_id>/robot_state            # should be a steady 10 Hz
curl http://localhost:3000/api/v1/robot/state    # the public subset, through the backend
```

Before `relocalize`, expect `localization_valid: false` and `state: 0`
(`UNINITIALIZED`) with real joint temperatures alongside them, and the curl to 404.

The low-battery transition can be exercised without a real BMS — note the
`<robot_id>` namespace, and that `ros2 topic pub` defaults to RELIABLE, which a
BEST_EFFORT subscriber accepts:

```bash
ros2 topic echo /<robot_id>/robot_state --field state   # watch in another pane

# 80% -> IDLE (1);  19% -> WARNING (3)
ros2 topic pub -r 2 /<robot_id>/battery_state sensor_msgs/msg/BatteryState \
    '{percentage: 0.19, present: true}'
# 22% is inside the hysteresis band -> still WARNING (3)
# 26% clears it -> IDLE (1)
# 0.0 simulates a corrupt BMS token -> state must NOT change, log warns
```

## Gotchas

- **`mode` is hardcoded.** Every message says `AUTO` regardless of what the
  robot is doing, and the REST layer surfaces it, so the UI always shows AUTO.
  Marked `{TODO}`.
- **`state` distinguishes three values**, `UNINITIALIZED` / `WARNING` / `IDLE` —
  and **the REST layer does not expose the field at all**, so a low-battery
  `WARNING` is visible only via `ros2 topic echo`, not in the UI. `RUNNING` /
  `ERROR` are still `{TODO}` and `CHARGING` is not derivable.
- **`RobotStatus` constants were renumbered** when `UNINITIALIZED` was added at
  `0` (so that an unset `state` no longer decodes as `IDLE`). `state` values in
  bags recorded before that change decode to the wrong constant.
- **Nothing but convention keeps operator fields out of the REST payload.** There
  is no second message type any more: `routers/robot.py` names its response fields
  one by one, and a new field on `RobotState` must not be added there.
- **`map` ends up holding a *path*, not a map name.** The params-file default is
  the friendly `"dp2f_full"`, but the launch file overrides it with `[map] map`
  from the INI, which is the map YAML path (`map/dp2f_full/gridmap.yaml`) — the
  same value `map_server` uses to load the grid. That string is passed straight
  through to `GET /api/v1/robot/state`, so the UI receives a path.
- **`wifi_info` is the literal string `"null"` before the first `wifi_status`.**
  An empty nlohmann json dumps to that, not to `""` or `"N/A"`, which is what the
  backend parses defensively against.
- **`timestamp` is seconds**, unlike `ArtifactState` / `ExecuteTask` which are
  milliseconds. The frontend multiplies by 1000; see `syncai_common`'s README.
  `motor_timestamp` in the same message is **nanoseconds** — two units in one
  message, the latter inherited unconverted from `MotorStates`.
- **At 10 Hz `timestamp` repeats.** Whole seconds means ten consecutive messages
  carry the same value, so it cannot order samples or measure the rate. It stays
  seconds because it is passed verbatim to the frozen REST payload; use
  `motor_timestamp` for sub-second resolution.
- **The battery scaling round-trips.** `driver_manager` divides the BMS's 0–100
  SoC by 100 for `BatteryState`, this node multiplies it back by 100. Changing
  one side without the other gives a battery reading off by 100×.
- **`velocity` is forward speed only** (`linear.x`), not the speed magnitude, so
  it is negative when reversing and ignores lateral motion.
- **Battery voltage, current and pack temperature are still discarded.**
  `driver_manager` publishes them on `battery_state`, but `RobotBatteryStatus`
  carries only the percentage.
