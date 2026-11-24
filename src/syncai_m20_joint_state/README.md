# syncai_m20_joint_state

The Lynx M20's leg and wheel angles as `sensor_msgs/JointState`, so that
`robot_state_publisher` puts its 16 moving links on `/tf`. One rclrs (0.8) node,
`m20_joint_state_node`, started by `syncai_bringup`'s `bringup.launch.py` for
`robot_model: m20` (`MODEL_JOINT_STATES`), in the `<robot_id>` namespace.

```
/JOINTS_DATA (drdds/msg/JointsData, the vendor's, domain 0, no namespace)
      │  raw × dir + offset (the vendor's own tables)
      ▼
/<robot_id>/joint_states (sensor_msgs/JointState, M20.urdf's joint names)
      │
      ▼
robot_state_publisher → /tf: <robot_id>/base_link → <robot_id>/{fl,fr,hl,hr}_{hipx,hipy,knee,wheel}
```

## Topics and parameters

| | Name | Type | |
|---|---|---|---|
| sub | `/JOINTS_DATA` (`joints_topic`) | `drdds/msg/JointsData` | best effort, keep last; the vendor's writer is reliable / transient local. Absolute: the vendor publishes with no namespace |
| pub | `joint_states` | `sensor_msgs/JointState` | reliable, depth 10; 16 joints, `position` (rad), `velocity` (rad/s), `effort` (N·m) |

| Parameter | Default | |
|---|---|---|
| `joints_topic` | `/JOINTS_DATA` | `/JOINTS_DATA_10HZ` is the same data at 10 Hz |
| `publish_rate` | `50.0` | publications per second at most, 0 = one per reading. `/JOINTS_DATA` arrives at ~180 Hz or more, and each publication is 16 transforms on `/tf`; the leg links are for looking at |

Both are read-only. There is no params file: nothing else is a choice, and the
conversion tables are the vendor's (below), not ours to tune.

## The conversion

`src/conversion.rs`, pure and unit tested (`cargo test`), is DEEP Robotics'
own code transcribed from their RL deployment node,
`src/M20_sdk_deploy/interface/robot/hardware/` in
[sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy) at the SHA
`third-party-sparse.repos` pins:

- the order of the 16 elements: legs fl, fr, hl, hr, each hipx, hipy, knee,
  wheel (`robot_order`); each element's `name` is its index in ASCII and is not
  read;
- `angle = raw × joint_dir + init_pos_offset` (`DdsInterface::Handler`,
  `M20Interface`'s tables); velocity and torque only take the direction;
- the one-off ±360° correction at start-up (`M20Interface::ResetPositionOffset`:
  a hipy past ±140° or a knee past ±164° on the first reading moves its offset a
  full turn, for good). Applied to the first reading only, as there: a hipy's
  range reaches 148°, where a per-reading wrap would flip it a turn.

**Why those angles are `M20.urdf`'s.** The vendor's policy runs on their MJCF
(`M20_description/m20_mjcf/mjcf/M20.xml`), whose link offsets and 16 joint
axes are `syncai_bringup/description/M20.urdf`'s exactly. Checked on robot01
on 2026-11-24 with forward kinematics on `M20.urdf`: lying, every knee folds
under the body; standing, every wheel is 0.48–0.50 m below its hip and under
it. The unit tests hold those two recorded readings. `M20.urdf`'s hipy / knee
`<limit>`s are the MJCF's negated — an exporter artefact that
`robot_state_publisher` does not read.

If the vendor changes their tables, change `conversion.rs` with them and
record a lying and a standing reading for the tests again.

**The stamp is our clock**, not the message's: the vendor stamps
`/JOINTS_DATA` with its own computer's uptime (seconds since its boot), which
`robot_state_publisher` would put on every leg transform, beyond the reach of
any TF lookup on our clock.

## Building

An `ament_cargo` package like the four out-of-tree rclrs ones, but in this tree.
It depends on `drdds`, which is not apt's: `scripts/sparse_import.sh` checks it
out of `sdk_deploy` (`third-party-sparse.repos`), and its Rust bindings exist
because it is built after the image's ros2-rust underlay — a plain
`colcon build` in the robot container is enough. `scripts/build.sh` prunes its
installed `share/syncai_m20_joint_state/rust` copy like the other rclrs nodes'.
A G23 build builds it too (it is in `src/`), but nothing starts it there.

Before committing a change: `cargo fmt --check`, `cargo clippy --all-targets`
and `cargo test` from this directory, after one `colcon build` (it writes the
`.cargo/config.toml` the cargo commands need), all without a warning from
`src/` (rclrs's own are not ours). The image has no clippy component.

## Checked

robot01, 2026-11-24, in an isolated namespace beside the running stack: ~44 Hz
of `joint_states` at the default 50 Hz cap, the first reading needing no
start-up correction, the angles matching the lying pose the robot was in.
`robot_state_publisher` turning them into `/tf` was not yet watched end to end.
