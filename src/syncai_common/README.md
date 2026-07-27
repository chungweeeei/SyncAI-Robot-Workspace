# syncai_common

The stack's shared ROS 2 interface definitions — 13 messages, 5 services, 1
action. No code, no nodes: `rosidl_generate_interfaces` and nothing else.

Everything here exists because two or more packages need to agree on a wire
format. Interfaces used by exactly one package generally stay in that package;
these crossed a boundary.

```
syncai_driver_manager ──IMUState / MotorStates──►  (telemetry consumers)
        ▲  SetMotionKey / SetPolicyMode / SetSpeedScale
        │
syncai_backend ─────────┼──────────► syncai_robot_state ──RobotState──► syncai_backend
        │  ScanWifiNetworks / ConnectWifiNetwork              ▲
        ▼                                                     │ WifiStatus
syncai_system_manager ────────────────────────────────────────┘
```

## Messages

### Robot state aggregate

`RobotState` is published at 1 Hz by `syncai_robot_state` on the relative topic
`robot_state` (BEST_EFFORT, KeepLast(1)) and consumed by `syncai_backend`, which
re-serialises it for `GET /api/v1/robot/state`. It nests four of the other
messages:

```
RobotState
├─ uint64 timestamp
├─ string robot_id, map
├─ uint8  mode                    ← constants from RobotMode
├─ uint8  state                   ← constants from RobotStatus
├─ RobotLocalizationStatus localization_status
│    ├─ RobotPose position        (x, y, z, yaw — yaw in RADIANS)
│    └─ float64 velocity          (forward linear speed from odom)
├─ RobotNetworkStatus network_status
│    └─ string wifi_info          ← a JSON object, see below
└─ RobotBatteryStatus battery_status
     └─ float64 battery_percentage
```

| Message | Fields | Notes |
|---|---|---|
| `RobotPose` | `x`, `y`, `z`, `yaw` | Plain floats, **not** a `geometry_msgs/Pose`. `yaw` is radians here; the REST layer converts to degrees. |
| `RobotLocalizationStatus` | `position`, `velocity` | |
| `RobotNetworkStatus` | `wifi_info` | Deliberately a JSON string, not a typed field — see below |
| `RobotBatteryStatus` | `battery_percentage` | 0–100, already scaled from `sensor_msgs/BatteryState.percentage` |
| `RobotMode` | `MAINTENANCE=0`, `MANUAL=1`, `AUTO=2` | **Constants only** — no data fields. Never published on its own; it exists so `RobotState.mode` has named values. |
| `RobotStatus` | `IDLE=0`, `RUNNING=1`, `WARNING=2`, `ERROR=3`, `CHARGING=4` | Same pattern, for `RobotState.state`. |

Two fields are placeholders today: `syncai_robot_state` hardcodes `mode = AUTO`
and `state = IDLE` (both marked `{TODO}` in the source). The REST layer surfaces
`mode` and ignores `state` entirely.

**Why `wifi_info` is a JSON string.** `syncai_robot_state` flattens the latest
`WifiStatus` into `{"ssid", "bssid", "rssi", "ip_address", "mac_address"}` and
dumps it into this one field. Before the first `wifi_status` arrives it is the
literal string `"null"`, which is why the backend parses it defensively and falls
back to an empty object. A typed sub-message would be cleaner; the string keeps
the aggregate stable while wifi reporting is still in flux.

### Wifi

| Message | Fields | Used by |
|---|---|---|
| `WifiNetwork` | `bssid`, `ssid`, `rssi` | A scan result. Returned in bulk by `ScanWifiNetworks`. |
| `WifiStatus` | `bssid`, `ssid`, `rssi`, `ip_address`, `mac_address` | Published at 1 Hz on `wifi_status` by `syncai_system_manager`'s wifi manager; consumed only by `syncai_robot_state`. |

`WifiStatus` is `WifiNetwork` plus the two local-interface fields. `rssi` is
`int8` in both (dBm, so roughly −100…0).

### Driver telemetry

Published by `syncai_driver_manager` from the ASCII telemetry it receives over
its UDP link to the gait controller.

| Message | Topic | Fields |
|---|---|---|
| `IMUState` | `imu` (SensorDataQoS) | `timestamp`, `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`, `rpy[3]`, `temperature` |
| `MotorStates` | `motor_states` (SensorDataQoS) | `timestamp` + `MotorState[] states` |
| `MotorState` | — | `name`, `q`, `dq`, `ddq`, `tau_est`, `temperature`, `error` — generalized position / velocity / acceleration / estimated torque |

These are the robot's own formats rather than `sensor_msgs/Imu` and
`sensor_msgs/JointState` because they carry per-motor `temperature` and `error`
fields that the standard messages have nowhere to put, and because `IMUState`
mirrors the field layout the gait controller already sends.

### ArtifactState

`ArtifactState` has **no publisher or subscriber in this workspace**. It is the
robot-side mirror of the interface used by the separate artifact stack
(`SyncAI-Artifact-Workspace`), kept here so a future ROS-side artifact monitor
can be written against it without a second definition.

```
uint64 timestamp     # ms since epoch
string artifact_id   # "conveyor_0", "door_1"
string type          # live_info discriminator: "conveyor", "door", ...
bool   connected     # transport session (e.g. modbus tcp) is up
bool   stale         # connected but the device stopped updating
uint16 error_code    # device-reported, 0 = ok
string live_info     # JSON of DECODED values, never raw registers
```

The split is the point: the four health fields are uniform across artifact types
so a monitor can alarm without parsing anything, while everything type-specific
lives in the `live_info` JSON discriminated by `type`. The backend's
`ConveyorPhase` enum (`belt`/`handoff`/`carried`/`dropped`) is what shows up in
a conveyor's `live_info.phase`.

## Services

| Service | Request | Response | Served by |
|---|---|---|---|
| `ScanWifiNetworks` | *(empty)* | `success`, `message`, `WifiNetwork[] networks` | `syncai_system_manager` on `scan_wifi` |
| `ConnectWifiNetwork` | `ssid`, `password` | `success`, `message` | `syncai_system_manager` on `connect_wifi` |
| `SetMotionKey` | `key` | `success`, `message` | `syncai_driver_manager` on `set_motion_key` |
| `SetPolicyMode` | `uint8 mode` | `success`, `message` | `syncai_driver_manager` on `set_policy_mode` |
| `SetSpeedScale` | six `float64` scales | `success` | `syncai_driver_manager` on `set_speed_scale` |

Notes:

- **`SetMotionKey.key` is a string, not an enum**, because it is forwarded
  verbatim to the gait controller. Values in use: `0` stand, `1` locomotion,
  `2` lie down, `3` damping, `4` emergency stop, `5` MPC. The backend no longer
  exposes this over REST; it is called from the `STANDUP` / `LIEDOWN` task steps
  (`syncai_backend/temporal/activities.py`).
- **`SetSpeedScale` has six independent scales** (`fwd`, `back`, `left`, `right`,
  `turn_l`, `turn_r`) because the gait controller tracks commanded velocity
  asymmetrically per direction; the driver manager applies these as a correction
  to `cmd_vel`. It is also the only service here whose response has no `message`
  field.
- `SetPolicyMode.mode` is a bare `uint8` and does **not** reuse `RobotMode`'s
  constants — it is the gait controller's policy index, a different namespace
  that happens to share the type.

The `success`/`message` pair is the convention for everything here: callers check
`success` and surface `message` verbatim (the backend maps a failed wifi connect
to HTTP 400 with that string as the detail).

## Action

`ExecuteTask` — goal `uuid` / `timestamp` / `behavior_tree` (an **inline BT XML
string**, not a file path), result `success` / `message` / `finished_timestamp`,
feedback `status` / `elapsed_ms`.

**Nothing implements or calls it.** It was defined for a design where the backend
would hand a whole behavior tree to the robot per task. The standing decision
went the other way: `RobotWorkflow` in `syncai_backend` sequences steps itself
and dispatches `MOVE` to `nav2_msgs/NavigateToPose` and `ARTIFACT` to the
artifact REST API, with the behavior-tree route reserved for a future need for
tick-level parallelism. The definition is kept because that need may still
arrive; its comments are in Chinese, unlike the rest of the package.

## Depending on this package

C++ (`CMakeLists.txt` + `package.xml`):

```cmake
find_package(syncai_common REQUIRED)
ament_target_dependencies(${target} syncai_common)   # or list it in `set(dependencies …)`
```

```xml
<depend>syncai_common</depend>
```

```cpp
#include "syncai_common/msg/robot_state.hpp"
#include "syncai_common/srv/set_motion_key.hpp"
```

Python — the generated module is importable once the workspace is sourced:

```python
from syncai_common.msg import RobotState, RobotMode
from syncai_common.srv import ScanWifiNetworks, ConnectWifiNetwork, SetMotionKey
```

Constant-only messages are accessed as class attributes, never instantiated:

```python
if state.mode == RobotMode.AUTO: ...
```

## Build

```bash
colcon build --packages-select syncai_common
source install/setup.bash
```

Then rebuild every dependent package — generated headers and Python modules do
not update in place, and `--symlink-install` does not help here because these are
generated artifacts, not source files:

```bash
colcon build --packages-up-to syncai_robot_state syncai_driver_manager
```

Inspect what actually got generated:

```bash
ros2 interface show syncai_common/msg/RobotState
ros2 interface list | grep syncai_common
```

## Gotchas

- **Changing a field is an ABI break.** Every node that publishes or subscribes
  the message must be rebuilt *and restarted*; a mismatched pair fails at the
  type-hash level with no useful error. On a live robot, rebuild the whole
  workspace rather than one package.
- **Timestamp units are not uniform.** `ArtifactState` and `ExecuteTask` are in
  milliseconds; `RobotState.timestamp` is in **seconds** (`now().seconds()` cast
  to `uint64`), because it is passed through verbatim to
  `GET /api/v1/robot/state` and the frontend already multiplies by 1000. Each
  `.msg` states its unit — check it before doing arithmetic across two of them.
- **No message carries a `std_msgs/Header`.** Timestamps are bare `uint64`
  fields and there is no `frame_id` anywhere — these are status messages, not
  sensor data to be transformed. Anything needing TF uses a `geometry_msgs` type
  instead.
- **`syncai_backend` imports `syncai_common` but does not declare it** in its
  `package.xml`. It works because both live in the same workspace and the install
  space is sourced as a whole, but colcon has no reason to build this package
  first — an explicit `<exec_depend>syncai_common</exec_depend>` there would make
  the ordering real.
- **Constant-only messages (`RobotMode`, `RobotStatus`) generate a publishable
  type with zero fields.** Publishing one is legal and meaningless; they exist
  purely as a constant namespace for `RobotState`'s `uint8` fields.
- `msg/`, `srv/` and `action/` each still carry a `.gitkeep` from when they were
  empty. Harmless, and every new interface must also be listed in
  `CMakeLists.txt` — the directories are not globbed.
