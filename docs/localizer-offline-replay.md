# Offline replay of `syncai_localizer`

How to record a bag on the robot and replay it later through `syncai_localizer`,
to tune its parameters or check a change without driving the robot again.

The localizer consumes two topics, `pointlio/body_cloud` and
`pointlio/lio_odom`, joined by an `ApproximateTime` synchroniser, plus
`initialpose` and its services (`src/syncai_localizer/README.md`, "Inputs").
Both topics are made by `syncai_pointlio`, so there are two ways to feed it:

| | Record | Replay runs | Covers |
|---|---|---|---|
| **A (recommended)** | the raw sensors: `livox/lidar` + `livox/imu` | `pointlio_node` + `localizer_node` | everything, including `reset_odometry` and Point-LIO's own init / deskew |
| **B** | Point-LIO's output: `pointlio/body_cloud` + `pointlio/lio_odom` | `localizer_node` only | registration, the motion gate, `relocalize` / `initialpose` / `auto_localize`; **not** `reset_odometry` |

B cannot test `reset_odometry`, because that service calls `pointlio/reset` and
then waits for odometry newer than the reset — a bag has no Point-LIO to reset.
Record the topics of both in one bag and you can choose at replay time.

## Recording

Record in the nav session (`AUTO`), from inside the robot container, into
`record/`, which is gitignored:

```bash
ros2 bag record -o record/loc_<robot_id>_<yyyymmdd>_<site> \
  /<robot_id>/livox/lidar \
  /<robot_id>/livox/imu \
  /<robot_id>/pointlio/lio_odom \
  /<robot_id>/pointlio/body_cloud \
  /<robot_id>/initialpose \
  /tf /tf_static
```

| Topic | Why |
|---|---|
| `livox/lidar` (`livox_ros_driver2/msg/CustomMsg`), `livox/imu` | Replay A's input. Required. |
| `pointlio/lio_odom`, `pointlio/body_cloud` | Replay B's input, and the live Point-LIO output to compare A against. |
| `initialpose` | The guesses the operator gave on site, if any. Not needed when the robot localized from the INI's `[initial_pose]` at boot. |
| `/tf`, `/tf_static` | **Reference only — never played back.** Holds the live `map → <robot_id>/pointlio_odom` from the localizer on the robot, to compare the replay with. |

What to do while recording:

1. **Start the bag before moving and stand still for a few seconds.** Point-LIO
   runs a static, gravity-aligning IMU init on its first samples. A bag that
   starts in motion tilts replay A's odometry for the whole run, with no
   error anywhere.
2. **Cover what you want to test.** In-place turns; long corridors (few
   geometric constraints); people walking past; a long stationary stretch (it
   exercises the motion gate: `min_update_trans`, `max_update_interval`,
   `static_blend_alpha`); and for `auto_localize` / `reset_odometry` with
   `skip_seed`, a kidnap — lift the robot and put it down somewhere else.
   Record a few minutes of normal driving before the kidnap, so the
   localizer has converged first.
3. **Write down which map was live** (`[map] name` in the instance INI) and
   the INI's `[initial_pose]`. The replay needs both.

The map should come from a **different mapping run** than the bag. A bag
replayed against the map built from that same drive scores better than the
robot ever will. `auto_localize` needs the map's `poses.txt`, which only a
save with `save_patches` writes.

## Replaying

### Where

Not in robot01 while its stack is up. The replay starts a second
`/<robot_id>/pointlio_node` and `/<robot_id>/localizer_node`, which would publish
over the live ones on DDS domain 1. Use the throwaway build container: it has
the same image and `install/`, host networking, and none of robot01's DDS
settings, so put it on a domain of its own:

```bash
# from the host, at the workspace root
docker compose -f docker-compose.build.yml run --rm --entrypoint bash \
  -e ROS_DOMAIN_ID=42 build
# inside
source /opt/ros/humble/setup.bash
source /opt/ros2_rust_underlay/install/setup.bash
source install/setup.bash
```

`livox_ros_driver2` must be built in `install/`, or `ros2 bag play` cannot
deserialise `CustomMsg`. A bag recorded without the `<robot_id>` prefix (older
bags) needs `--remap` on play; pointlio's launch no longer strips it.

### The INI

Both launches read `robot_id`, `[map] pcd` and `[initial_pose]` from an INI,
`~/robot_ws/config/system.ini` by default. That file is empty outside robot01
(robot01 bind-mounts its instance INI over it), so write one for the replay and
pass it with `system_config:=`. Give `pcd` as an **absolute** path: a relative
one is resolved two directories above the INI, which is only the workspace root
when the INI is `config/system.ini` (`src/syncai_localizer/README.md`,
"Running").

```ini
# record/loc_<...>/replay.ini
[system]
robot_id = robot01

[map]
name = <map name>
pcd = /home/syncrobotic/robot_ws/map/<map name>/map.pcd
map = /home/syncrobotic/robot_ws/map/<map name>/gridmap.yaml

[initial_pose]
x = 0.0
y = 0.0
yaw = 0.0
```

Leave out `[initial_pose]` to start unlocalized and give the guess yourself
with `initialpose` or `relocalize`.

### Replay A: raw sensors through Point-LIO

```bash
INI=record/loc_<...>/replay.ini
ros2 launch syncai_pointlio pointlio.launch.py system_config:=$INI &
ros2 launch syncai_localizer localizer.launch.py system_config:=$INI &

ros2 bag play record/loc_<...> \
  --topics /robot01/livox/lidar /robot01/livox/imu /robot01/initialpose
```

### Replay B: Point-LIO's recorded output

```bash
ros2 launch syncai_localizer localizer.launch.py system_config:=$INI &

ros2 bag play record/loc_<...> \
  --topics /robot01/pointlio/lio_odom /robot01/pointlio/body_cloud /robot01/initialpose
```

### Rules for both

- **Play at 1×, no `--rate`.** The localizer paces itself on wall time, not on
  message stamps: `update_hz` and the motion gate's `max_update_interval` /
  `post_reloc_settle` all read `rclcpp::Clock()`
  (`src/syncai_localizer/src/localizer_node.cpp`, `timerCB`). At 2× it runs
  half as many registrations per metre as the robot did, and the result is
  not the robot's. Neither node uses `use_sim_time`, so `--clock` is not needed
  either; the TF is stamped from the messages.
- **Never play `/tf` or `/tf_static`.** The recorded `map → pointlio_odom` and
  `pointlio_odom → pointlio_body` would race the ones the replay broadcasts.
- **Service tests are called during the replay** with the commands in the
  localizer README's "Running" section, at the point of the bag where they
  apply: `auto_localize` after the kidnap, `reset_odometry` (A only) with the
  robot stationary in the bag, `relocalize_check` after a guess.

Check the replay is wired before trusting it: the localizer's log should print
`params: cloud_topic=/robot01/pointlio/body_cloud …` and, once data flows,
`refine: converged=yes … fitness=0.0xx` lines.

## Judging the result

Record what the replay produces, in a second shell:

```bash
ros2 bag record -o record/loc_<...>_replay_<label> /tf /robot01/pointlio/lio_odom
```

Then compare:

- **Against the map's keyframes.** Take the localized body pose
  `map → pointlio_odom → pointlio_body` and measure its distance to the
  nearest keyframe in the map's `poses.txt`, where the replay's path overlaps
  the mapping run's. This is how the dp1f replay was judged in the localizer
  README (median 1.5 cm, p95 4.2 cm).
- **Against the robot's own run.** The same chain from the recorded `/tf`, to
  tell a regression from a bag the localizer always found hard.
- **Accepted rounds.** Count `refine: converged=yes` against all rounds in
  the localizer log. The dp1f numbers (92.5% → 97.9% after the rough-stage
  eps change) are in the README's "Gotchas". Since 2026-10 the node also
  prints `loc stats (since start): run=… accepted=… not_converged=…
  degenerate=… jump_rejected=… projected=…` once a minute, and a round that
  converged can still be dropped by the jump gate or be degenerate — count
  `accepted`, not `converged=yes`.
- **The `loc:` lines** (2026-10, one per accepted round). `t-eig=(a b c)` are
  the per-inlier eigenvalues of the translation information in the map
  frame, weakest first; `lvl=NFF/FFF` the levels (translation / rotation);
  `weak=` the heading of the weakest translation direction; `inl=` the refine
  inlier ratio; `ms=` the round's wall time; `dt=raw->kept` how much of the
  translation correction survived the projection. Plot `a / c` against time:
  it should drop on corridor stretches and sit near 1 in open rooms, and
  `localizability.ratio_none` / `ratio_partial` belong between those two
  populations (the README's "Localizability"). `ms` is what
  `track_accum_scans` costs: rough + refine must stay under half the
  `update_hz` period.
- **A/B on one bag.** `localizability.enabled: false`, `jump_gate.enabled:
  false`, `track_accum_scans: 1` and `guess_timeout_sec: 0` together restore
  the pre-2026-10 node; flip one at a time to attribute a change.

Do **not** judge drift on `map → pointlio_odom` alone. Its translation is
`t_body − R · t_odom`, so 1° of rotation moves it 1 m for a robot 60 m from
where Point-LIO started, while the body pose moves millimetres. The README's
"Gotchas" has the dp1f example (±3 m in that transform, body within 2 cm).

Replay A also re-runs Point-LIO, so a change in its output moves the localizer's
result too. To test a localizer-only change, use replay B, or compare two
replay-A runs made from the same build of `syncai_pointlio`.
