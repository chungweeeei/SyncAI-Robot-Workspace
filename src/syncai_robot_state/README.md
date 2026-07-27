# syncai_robot_state

One node, `syncai_robot_state`, that aggregates the robot's scattered status
sources into a single `syncai_common/RobotState` message at **1 Hz**. It is the
only producer of that topic, and `syncai_backend` is its only consumer — which
makes this node the thing that decides what the operator UI can show.

```
        TF: map → <robot_id>/base_link  ─────┐   (from syncai_lio_bridge / syncai_amcl)
        odom          (nav_msgs/Odometry) ───┤
        battery_state (BatteryState)  ───────┼──►  syncai_robot_state  ──robot_state──►  syncai_backend
        wifi_status   (WifiStatus)    ───────┘            (1 Hz)                              │
                                                                                              ▼
                                                                             GET /api/v1/robot/state
```

It aggregates only — it computes nothing beyond a yaw extraction and a unit
conversion, and it holds no state beyond the latest sample of each input.

## What goes into each field

| `RobotState` field | Source | Notes |
|---|---|---|
| `timestamp` | `now().seconds()` | **Seconds**, not milliseconds |
| `robot_id` | `robot_id` parameter | Set from the INI by the launch file |
| `map` | `map` parameter | See the gotcha — this is a path, not a name |
| `mode` | — | **Hardcoded to `AUTO`** (`{TODO}` in the source) |
| `state` | — | **Hardcoded to `IDLE`** (`{TODO}`); the REST layer does not expose it |
| `localization_status.position` | TF `global_frame → base_frame` | x/y/z plus `yaw` in **radians** |
| `localization_status.velocity` | `odom.twist.twist.linear.x` | Forward speed only |
| `battery_status.battery_percentage` | `battery_state.percentage × 100` | `BatteryState` is 0–1, this field is 0–100 |
| `network_status.wifi_info` | `wifi_status`, flattened to JSON | See below |

**`wifi_info` is a JSON string,** not a typed sub-message: the latest
`WifiStatus` is flattened into `{"ssid", "bssid", "rssi", "ip_address",
"mac_address"}` with nlohmann/json. Before the first `wifi_status` arrives the
empty json dumps to the literal string `"null"`, which is why the backend parses
this field defensively and falls back to an empty object.

## The TF gate

`onTimer()` looks up `global_frame → base_frame` through
`syncai_util::getCurrentPose`, and if that fails it logs a throttled warning and
**returns without publishing anything** — no partial message with a stale or
zeroed pose.

The practical consequence: on the 3D stack there is no `robot_state` topic at all
until `/localizer/relocalize` has been called, because `map → odom` does not
exist before that. The backend then answers `GET /api/v1/robot/state` with
404 "Robot state is not available yet", which is correct but reads like the node
is dead. Check for `TF map-><id>/base_link unavailable` in the log before
suspecting this node.

The other three inputs are not gated: a missing odom, battery or wifi message
just leaves that field at its zero/`"null"` default.

## Threading

`main.cpp` uses a `MultiThreadedExecutor`, and the 1 Hz timer gets its **own
`MutuallyExclusive` callback group** so the TF lookup and message build run
independently of the three (lightweight) subscription callbacks. The three
cached samples are guarded by one mutex, taken briefly in each callback and once
in the timer.

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

`odom` comes from `syncai_lio_bridge`, `battery_state` from
`syncai_driver_manager`, `wifi_status` from `syncai_system_manager`. The
backend's subscriber matches the BEST_EFFORT publisher.

## Parameters

| Parameter | Default | Set by the launch file |
|---|---|---|
| `robot_id` | `""` | `[system] robot_id` from the INI |
| `map` | `""` (yaml: `dp2f_full`) | `[map] map` from the INI, when present |
| `global_frame` | `map` | — (stays unprefixed) |
| `base_frame` | `base_link` | `<robot_id>/base_link` |
| `transform_tolerance` | `0.1` | — |

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
ros2 topic echo /<robot_id>/robot_state --once
ros2 topic hz /<robot_id>/robot_state          # should be a steady 1 Hz
curl http://localhost:3000/api/v1/robot/state  # the same data, through the backend
```

Nothing published at all almost always means the TF gate — see above.

## Gotchas

- **`mode` and `state` are hardcoded.** Every message says `AUTO` / `IDLE`
  regardless of what the robot is doing. The REST layer surfaces `mode` (so the
  UI always shows AUTO) and ignores `state` entirely. Both are marked `{TODO}`.
- **`map` ends up holding a *path*, not a map name.** The params-file default is
  the friendly `"dp2f_full"`, but the launch file overrides it with `[map] map`
  from the INI, which is the map YAML path (`map/dp2f_full/gridmap.yaml`) — the
  same value `map_server` uses to load the grid. That string is passed straight
  through to `GET /api/v1/robot/state`, so the UI receives a path.
- **`odom_topic` in the params YAML is dead.** It is neither declared nor read;
  the subscription hardcodes the relative name `odom`. Undeclared overrides are
  silently ignored by rclcpp, so setting it does nothing.
- **The `"N/A"` in the `wifi_info` comment is wrong.** An empty nlohmann json
  dumps to `"null"`, which is what actually goes on the wire and what the
  backend checks for.
- **`timestamp` is seconds**, unlike `ArtifactState` / `ExecuteTask` which are
  milliseconds. The frontend multiplies by 1000; see `syncai_common`'s README.
- **The battery scaling round-trips.** `driver_manager` divides the BMS's 0–100
  SoC by 100 for `BatteryState`, this node multiplies it back by 100. Changing
  one side without the other gives a battery reading off by 100×.
- **`velocity` is forward speed only** (`linear.x`), not the speed magnitude, so
  it is negative when reversing and ignores lateral motion.
