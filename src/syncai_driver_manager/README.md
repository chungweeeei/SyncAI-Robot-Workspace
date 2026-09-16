# syncai_driver_manager

The boundary between ROS 2 and the robot's gait controller. One node,
`driver_manager`, that owns a **bidirectional UDP session** with the controller:
ASCII commands out, ASCII telemetry in.

Everything below this node is not ROS. Everything above it is.

```
syncai_controller ──cmd_vel──┐
                             │        ┌──────────────────┐   AXES / MODE / ESTOP
syncai_backend ──set_motion_key──────► │  driver_manager  │ ═══════════════════════►  gait
                             │        │                  │ ◄═══════════════════════   controller
GUI / CLI ──set_policy_mode──┘        └──────────────────┘   BMS_V2 / IMU_RPY / …      (UDP)
             set_speed_scale                   │
             reset_safety                      ├── imu            (IMUState)
                                               ├── motor_states   (MotorStates)
                                               ├── battery_state  (BatteryState) ──► syncai_robot_state
                                               └── mode           (Int32MultiArray)
```

Ported from `SyncAI-Robot-GaitMPC/src/udp_ros_bridge`, which remains the
reference for any behaviour question the code does not answer.

## The UDP session

Two sockets, both plain `SOCK_DGRAM`, opened in the constructor — **the node
throws if either fails**, so it never comes up half-connected.

| Direction | Parameter | Default | Config value |
|---|---|---|---|
| Inbound (bind) | `telemetry_recv_ip` / `telemetry_recv_port` | `0.0.0.0` / `50012` | `192.168.1.103` / `50010` |
| Outbound (send to) | `command_target_ip` / `command_target_port` | `192.168.1.120` / `50051` | same |

The receive socket is bound to a **specific interface address**, not
`0.0.0.0` — datagrams arriving on any other interface are not delivered. If
telemetry goes silent after a network change, check this first.

`receiveLoop()` runs on its own `std::thread`, blocking in `recvfrom()` with a
100 ms `SO_RCVTIMEO` so it sleeps in the kernel while idle but still notices
`running_` going false at shutdown. The destructor clears `running_`, joins the
thread, then closes both sockets — in that order, because the loop touches the
socket it would otherwise be closing.

### Commands out

| Wire format | Sent by |
|---|---|
| `AXES <vx> <vy> <wz>\n` (`%.6f`) | every `cmd_vel` message |
| `MODE <char>\n` | `set_motion_key` |
| `MODE <uint>\n` | `set_policy_mode` |
| `ESTOP\n` | `set_motion_key` with key `"4"` |

Note that `MODE` is overloaded: `set_motion_key` sends a **character**
(`MODE X`), `set_policy_mode` sends a **number** (`MODE 2`). They are different
commands on the same keyword; the controller disambiguates by the argument.

### Telemetry in

One datagram is a single line of whitespace-separated ASCII sections. Each
section keyword is followed by its values, running until the next known keyword,
and **any subset of sections may appear in a given datagram**:

```
IMU_RPY r p y  ACC ax ay az  OMEGA wx wy wz  JOINT_POS q0 … q11  MODE_STATE pol mot
```

| Keyword | Values | Published as |
|---|---|---|
| `BMS_V2` | voltage, current, soc, …, temps, cells | `battery_state` (`sensor_msgs/BatteryState`) |
| `IMU_RPY` / `ACC` / `OMEGA` | 3 each | `imu` (`syncai_common/IMUState`) |
| `JOINT_POS` / `JOINT_VEL` / `JOINT_TAU` / `JOINT_TEMP` / `JOINT_ERR` | 12 each | `motor_states` (`syncai_common/MotorStates`) |
| `MODE_STATE` | policy state, motion state | `mode` (`std_msgs/Int32MultiArray`) |

`BMS_V2` is handled as a whole-datagram special case (it returns early); the rest
are searched for by keyword within one line.

Parsing is defensive throughout: a section with too few values, or a token that
is not a finite number, is skipped with a throttled warning rather than
publishing garbage. Publication is gated with `||` across the sections of a
message, so a datagram carrying only `IMU_RPY` still publishes an `IMUState`
with the missing fields zeroed.

Three conversions happen on the way in:

- **Battery percentage.** The BMS reports state-of-charge as 0–100;
  `BatteryState.percentage` is defined on 0–1, so it is divided by 100. (The REST
  layer multiplies it back by 100 much later — the scaling round-trips.)
  Temperature is the mean of two reported sensors.
- **IMU orientation.** The telemetry carries no quaternion, so one is derived
  from RPY (ZYX convention) into `quaternion` as **`[w, x, y, z]`**. Without an
  `IMU_RPY` section it falls back to identity.
- **Joint names.** The 12 values per `JOINT_*` section are labelled with the G23
  URDF's actuated joint names, in the order `FL / FR / HL / HR × HipX / HipY /
  Knee`. The `*_Ankle` joints are fixed and not reported.

## ROS interfaces

All names are relative, so they inherit the `<robot_id>` namespace.

**Publishers**

| Topic | Type | QoS |
|---|---|---|
| `imu` | `syncai_common/IMUState` | SensorData |
| `motor_states` | `syncai_common/MotorStates` | SensorData |
| `battery_state` | `sensor_msgs/BatteryState` | depth 10 |
| `mode` | `std_msgs/Int32MultiArray` | depth 10 — `data[0]` policy state, `data[1]` motion state |

**Subscriber:** `cmd_vel` (`geometry_msgs/Twist`, depth 10).

**Services**

| Service | Type | Effect |
|---|---|---|
| `set_motion_key` | `syncai_common/SetMotionKey` | Gait state change; see the key table |
| `set_policy_mode` | `syncai_common/SetPolicyMode` | `MODE <uint>` — the controller's policy index |
| `set_speed_scale` | `syncai_common/SetSpeedScale` | Update the six direction gains at runtime |
| `reset_safety` | `std_srvs/Trigger` | Release the safety lock |

### Motion keys

The service contract is a **numeric string**, `"0"`–`"5"`:

| Key | Wire | Meaning |
|---|---|---|
| `"0"` | `MODE Z` | Stand |
| `"1"` | `MODE C` | Locomotion (RL) |
| `"2"` | `MODE X` | Lie down |
| `"3"` | `MODE R` | Damping |
| `"4"` | `ESTOP` | Emergency stop — **not** a `MODE` character |
| `"5"` | `MODE M` | MPC |

Any GUI that accepts friendlier aliases (`m`, `mpc`, …) does that translation in
its own layer; the ROS contract stays numeric. Two callers in the backend: the
`STANDUP` / `LIEDOWN` task steps (keys `0` / `2`) and
`POST /api/v1/robot/set_motion_key`. That REST endpoint was added, removed in
favour of the task steps, then re-added for manual control; it exposes all six
keys but refuses to forward `"4"`, answering 200 with `sent: false` rather than
emitting `ESTOP`.

### Per-direction speed scaling

`cmdVelCallback` does not forward `cmd_vel` verbatim. The gait controller tracks
commanded velocity **asymmetrically per direction**, so each of the six
directions carries its own empirically tuned gain, picked by the sign of the
command:

```cpp
vx = (linear.x  >= 0) ? linear.x  * scale_fwd    : linear.x  * scale_back;
vy = (linear.y  >= 0) ? linear.y  * scale_left   : linear.y  * scale_right;
wz = (angular.z >= 0) ? angular.z * scale_turn_l : angular.z * scale_turn_r;
```

All six are `std::atomic<double>` because `set_speed_scale` writes them from the
services callback group while `cmd_vel` reads them from its own. They are **ROS
parameters** — `scale_fwd`, `scale_back`, `scale_left`, `scale_right`,
`scale_turn_l`, `scale_turn_r`, each defaulting to `1.0` — declared in
`initParameters()` and loaded from `params/driver_manager_params.yaml` at
startup, so the calibration survives a restart. That was not always so: they
used to be hard-coded `1.0` with `set_speed_scale` as the only way to change
them, and since nothing in the stack calls that service on startup, the
correction was silently never applied. `set_speed_scale` still overrides all
six at runtime — and *that* override is what is **not** persisted. It writes the
atomics, not the parameters, so a restart returns to the YAML values.

### Velocity-scale calibration

The shipped values are `scale_fwd: 1.40`, `scale_turn_l: 1.40`,
`scale_turn_r: 1.40`, and `1.0` for `scale_back` / `scale_left` / `scale_right`
(unmeasured — the nav stack never commands them: `allow_reversing` is `false`
and `min_y_velocity_threshold` gates strafe). They were calibrated on robot01
against `/odom`, where yaw comes from the lidar IMU gyro (a direct measurement)
and forward speed is differentiated from the LIO pose (the softer number). Only
steady-state points count — a bin taken while the command is still ramping
measures the ramp, not the gain — and the two navigation runs that held a
plateau long enough were:

| Run | Scales in force | Command | Measured | Ratio |
|---|---|---|---|---|
| A | `1.00` | `0.80` m/s | `0.624` | `0.78` (n=153) |
| A | `1.00` | `1.30` rad/s | `0.695` | `0.53` (n=125) |
| B | `1.15` fwd / `1.67` turn | `0.60` m/s | `0.495` | `0.82` (n=321) |
| B | `1.15` fwd / `1.67` turn | `0.65` rad/s | `0.776` | `1.19` (n=202) |

The shipped `1.40`s fold run B's residual back into its scales: `1.15 / 0.82`
and `1.67 / 1.19` both land on `1.40`.

**Known weak.** That is one plateau per axis per run, and the two runs do not
agree. Run B reached `0.776` rad/s from an `AXES` value of `1.09`, which is more
output from less input than run A's `0.695` at `AXES 1.30` — so the response is
not a fixed gain, and an earlier reading of it as "gain plus a hard ceiling at
0.69" was wrong (that curve was built from ramp transients). Until someone runs
an open-loop sweep — hold each of `vx` 0.2..0.6 and `wz` 0.2..1.0 for 3–5 s and
record the plateaus — treat these as a **working correction, not a
calibration**.

This is also why `syncai_controller`'s commands are modest: its
`controller_server_params.yaml` lowered `desired_linear_vel` from `0.8` to
`0.60` and `rotate_to_heading_angular_vel` from `1.3` to `0.65`, so that the
commanded rate stays inside the range that has actually been measured (and, for
rotate-to-heading, so its exit test is judged against a rate within ~20% of
reality rather than one the robot delivers at half speed). The two packages'
values are **one calibration**: raise either controller speed only together
with a fresh scale measurement at the new command.

## Threading

`main.cpp` uses a `MultiThreadedExecutor`, and the callback groups are chosen
deliberately:

| Work | Group |
|---|---|
| `cmd_vel` | Its own `MutuallyExclusive` group — the high-rate command stream runs concurrently with service handling |
| All four services | **One shared** `MutuallyExclusive` group, so they are serialized against each other |
| Telemetry receive | A raw `std::thread`, outside the executor entirely |

Serializing the services is intentional: they all read or write `safe_lock_` and
the speed scales, and running them one at a time keeps that state consistent
without additional locking.

## The safety lock — currently inert

The design is: `triggerSafeShutdown(reason)` sets `safe_lock_` via
`exchange(true)` (atomic check-and-set, so concurrent triggers act exactly once),
logs, and sends `MODE X` to lie the robot down. While the lock is held,
`set_motion_key` rejects everything except `"4"` (ESTOP), and only the
`reset_safety` service can clear it — deliberately not a motion key.

**But nothing calls `triggerSafeShutdown()` yet.** The two intended triggers are
still `TODO`s in `parseLine`:

- battery `soc < 20%` — the *judgement* has since moved out of this node:
  `syncai_robot_state` watches `battery_state` and reports
  `RobotStatus::WARNING` below 20% (latched, clearing above 25%). That resolves
  the "monitoring will live somewhere other than this node, where is undecided"
  question this README used to leave open. **The *actuation* is still
  unassigned** — `syncai_robot_state` deliberately only reports, and there is no
  service on this node for anything to call, so lying the robot down on low
  battery still happens nowhere. Wiring it needs a new inbound service here;
  `reset_safety` is the release, not the trigger.
- `JOINT_TEMP` overheat — undecided, and unlike the battery case the judgement
  has not moved anywhere either. The reference implementation's thresholds
  (≥75 °C warn, ≥95 °C lie down, ≥115 °C ESTOP) are the sanctioned numbers when
  someone does it.

So today `safe_lock_` is never set, `set_motion_key` never rejects, and
`reset_safety` always answers "System was not locked."

Two further gaps in the same area, both differences from the reference
implementation:

- **`cmd_vel` is not gated by `safe_lock_`.** Even once a trigger is wired, the
  controller's velocity commands would keep flowing through.
- **`set_policy_mode` is not gated either** — and it is now reachable over REST
  (`POST /api/v1/robot/set_policy_mode`), so once a trigger is wired a policy
  switch will still pass while motion keys are being rejected.

## Parameters

Ten parameters: the four UDP endpoint settings and the six velocity scales.

| Parameter | Default | Config |
|---|---|---|
| `telemetry_recv_ip` | `0.0.0.0` | `192.168.1.103` |
| `telemetry_recv_port` | `50012` | `50010` |
| `command_target_ip` | `192.168.1.120` | `192.168.1.120` |
| `command_target_port` | `50051` | `50051` |
| `scale_fwd` | `1.0` | `1.40` |
| `scale_back` | `1.0` | `1.0` (unmeasured) |
| `scale_left` | `1.0` | `1.0` (unmeasured) |
| `scale_right` | `1.0` | `1.0` (unmeasured) |
| `scale_turn_l` | `1.0` | `1.40` |
| `scale_turn_r` | `1.0` | `1.40` |

None are dynamically reconfigurable through `ros2 param set` — all are read
once in the constructor. The UDP settings are consumed before the sockets are
opened, so a change needs a restart; the scales are copied into their atomics,
and the only runtime path onto them is the `set_speed_scale` service (which
does not write back to the parameters — see "Velocity-scale calibration").

The published `BatteryState` sets only `header.stamp`, never a `frame_id`, so
this node has no frame parameter and the launch file does no `<robot_id>/`
frame rewriting (unlike every other launch file in the stack).

## Running

```bash
ros2 launch syncai_driver_manager driver_manager.launch.py
ros2 launch syncai_driver_manager driver_manager.launch.py \
    system_config:=config/instances/robot02.ini
```

Started in the byobu sessions' `managers` window. The launch file resolves
`robot_id` from `config/system.ini` and uses it as the node namespace; the params
file uses a `/**/driver_manager` wildcard key so it matches at any namespace, and
the `Node` therefore carries no `name=`.

Poking at it by hand:

```bash
ros2 service call /<robot_id>/set_motion_key syncai_common/srv/SetMotionKey "{key: '0'}"   # stand
ros2 service call /<robot_id>/set_motion_key syncai_common/srv/SetMotionKey "{key: '4'}"   # e-stop
ros2 service call /<robot_id>/reset_safety std_srvs/srv/Trigger "{}"

ros2 topic pub -r 10 /<robot_id>/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic hz /<robot_id>/motor_states        # is telemetry arriving at all?
ros2 topic echo /<robot_id>/battery_state
```

The receive loop logs "Received N bytes from ip:port" at 1 Hz (throttled), which
is the quickest confirmation that the UDP link is alive independent of whether
any section parses.

## Gotchas

- **The node throws in its constructor if either socket fails.** A wrong
  `telemetry_recv_ip` for the current interface, or a port already in use, kills
  the process at startup rather than degrading.
- **`cmd_vel` has no watchdog.** If the controller stops publishing, this node
  simply stops sending `AXES` — it does not send a stop command. Whether the
  robot halts is up to the gait controller's own timeout.
- **`set_speed_scale` overrides are not persisted.** The six scales are ROS
  parameters read from `params/driver_manager_params.yaml` at startup, so the
  *calibrated* values survive a restart — but anything written at runtime
  through `set_speed_scale` lives only in the atomics and is gone the next time
  the node comes up. A value worth keeping belongs in the YAML.
- **`IMUState.timestamp` is nanoseconds**, from `now().nanoseconds()` — a third
  convention alongside `RobotState` (seconds) and `ArtifactState` (milliseconds).
  Check the producer before doing arithmetic across messages.
- **UDP is unreliable and unacknowledged.** `udpSend()` ignores the return of
  `sendto()`; a dropped `MODE X` is simply lost. Nothing retries and nothing
  reports it.
- **`angular.z` is not negated, but the comment says it is.** The code reads:

  ```cpp
  // The controller's turning sign convention is opposite to REP 103 (+z =
  // counter-clockwise), so negate before picking the turn gain.
  const double wz_raw = msg->angular.z;      // ← no negation
  ```

  Either the firmware sign convention was fixed and the comment is stale, or the
  negation was dropped and the robot turns the wrong way. Verify on hardware
  before trusting either reading.

## Unverified assumptions

Carried as `TODO`s in the source, listed here so they are not rediscovered:

- Are `IMU_RPY` values in **radians**? The quaternion derivation assumes so; if
  they are degrees, orientation is nonsense.
- Is the joint order really `FL / FR / HL / HR × HipX / HipY / Knee`? The names
  in `motor_states` are assigned positionally.
- Does the firmware always pack `IMU_RPY`, `ACC` and `OMEGA` into one datagram?
  The publish gate is `||`, so a split would emit `IMUState` messages with
  zeroed fields rather than dropping them.
