# syncai_localizer

Map-based relocalization for the navigation session. One node,
`localizer_node`, registers `syncai_pointlio`'s body-frame scan against the
saved `map.pcd` with two stages of GICP (rough, 0.25 m voxels, then refine,
0.1 m; [small_gicp](https://github.com/koide3/small_gicp) as the backend) and
broadcasts the result as the `map → <robot_id>/pointlio_odom` correction that
`syncai_lio_bridge` turns into `map → odom`. It serves `relocalize` /
`relocalize_check`, listens on `initialpose`, applies the INI's
`[initial_pose]` as a boot guess, and serves `auto_localize`, a hint-free
global search for a kidnapped robot, and `reset_odometry`, which zeroes a
diverged Point-LIO and carries the map pose across. In the mapping session `syncai_mapping`'s
`pgo_node` owns the same TF instead; this node is never up at the same time.

```
   syncai_pointlio           /<id>/pointlio/{body_cloud, lio_odom}
        │                           │  (ApproximateTime sync)
        │                           ▼
        │                     localizer_node                 /<id>/…  (bare robot_id namespace)
        │          rough GICP → refine GICP, motion-gated, ≤ update_hz
        │             TF  map ──► <id>/pointlio_odom  (frame adopted from lio_odom)
        │       ┌──────────────┬──────────────┬──────────────┬─────────────┐
        │       ▼              ▼              ▼              ▼             ▼
        │  relocalize   relocalize_check   initialpose    map_cloud   [initial_pose]
        │  (backend:    (backend polls     (console /     (latched,   (INI, once, on
        │   map switch)  the outcome)       rviz 2D pose)  rviz)       first odom)
```

Ported into the workspace from `SyncAI-Fast-LIO2`'s `localizer` package in
2026-09, the last node left in that fork (`pointlio`, `pgo`, `hba` and its
`interface` srvs had gone the same way earlier that month), so the fork is no
longer imported at all. The ROS surface did not change with the move — same
executable and node name, namespace, services, topics, TF and parameters — and
the two service types were already `syncai_common`'s. `localizers/` is the
ported registration (`icp_localizer.*`, `commons.h`, upstream lineage, MIT);
`localizer_node.*` is the ROS shell.

## The namespace is the bare `robot_id`

The launch puts the node at **`/<robot_id>/localizer_node`**, not
`/<robot_id>/localizer/localizer_node`, so every relative name resolves one
level up from where most docs used to put it:

| Kind | Resolved name (`robot_id = robot01`) |
|---|---|
| node | `/robot01/localizer_node` |
| services | `/robot01/relocalize`, `/robot01/relocalize_check` |
| subscribed | `/robot01/initialpose` |
| published | `/robot01/map_cloud` |
| TF | `map → robot01/pointlio_odom` |

It was `<robot_id>/localizer` until fork commit `3f5f01b` (2026-07-29) dropped
the segment, and the backend has called the bare `relocalize` /
`relocalize_check` and published the bare `initialpose` from the robot's
namespace ever since — its map gateway records that writing
`localizer/relocalize` cost it a `stack_not_ready` refusal against a perfectly
healthy stack. The fork's README, the srv comments in `SyncAI-Robot-Interface`
and the workspace docs kept the old spelling for two months; the port kept the
*behaviour* (moving the namespace back is a cross-repository change the
backend would have to follow) and corrected the docs in this repo instead. The
srv comments in the interface repo still say `<robot_id>/localizer/…` and are
wrong. Check with `ros2 node info /<robot_id>/localizer_node`, not with
`ros2 service list`, which can keep a stale entry after a participant dies.

## Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `cloud_topic` = `/<robot_id>/pointlio/body_cloud` | `sensor_msgs/PointCloud2`, depth 10 | `syncai_pointlio` | The scan to register: the newest `track_accum_scans` (3) of them, composed through their odom poses into the newest one's body frame, then voxelised twice (rough and refine resolutions) |
| `odom_topic` = `/<robot_id>/pointlio/lio_odom` | `nav_msgs/Odometry`, depth 10 | `syncai_pointlio` | The LIO pose the correction is composed with; its `header.frame_id` becomes `local_frame` |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped`, depth 10 | the console, rviz "2D Pose Estimate", the backend's map switch | A 2D guess through `applyPlanarGuess` (below); `position.z` is the ground's height and taken only when `covariance[14]` (the z variance) is positive |
| `relocalize` | `syncai_common/srv/Relocalize` | the backend (initial pose, map switch), a shell | (Re)load a PCD and take the request's raw 6-DOF pose as the next guess |
| `[map] pcd`, `[initial_pose]` | `config/system.ini` via the launch | | `map_path` (loaded during construction) and the boot guess |

The two topics are joined by a `message_filters` `ApproximateTime` synchroniser
(queue 10, age penalty 0.1). They are **absolute names set as parameters**,
not remappings: they live in pointlio's namespace, which a relative name from
`/<robot_id>` cannot reach, so the launch overrides both per `robot_id` and
they are the contract with `syncai_pointlio/launch/pointlio.launch.py`. The
node logs the resolved names at startup, so a bare run (subscribing to
`/pointlio/…`, which nothing publishes) shows up in the log rather than as
silence.

## Outputs

| Output | Type / QoS | Notes |
|---|---|---|
| TF `map → local_frame` | `<robot_id>/pointlio_odom` | Rebroadcast from a 10 ms timer on every tick, re-solved at most `update_hz` and only when the motion gate says so. `local_frame` is **adopted from the first odom message**; until then it is the launch's `<robot_id>/pointlio_odom`, the same name. |
| `map_cloud` | `PointCloud2`, depth 1, TRANSIENT_LOCAL | The refine-resolution map, published once per `loadMap` (construction, `relocalize`, the `initialpose` fallback). For rviz; set the display's durability to Transient Local. |
| `auto_localize` | `syncai_common/srv/AutoLocalize` | Kidnap recovery, no hint. **Blocks** until a pose is found *and* the timer has registered against it, or fails with localization unchanged. See below. |
| `reset_odometry` | `syncai_common/srv/ResetOdometry` | Zero the odometry, keep the map pose: calls `pointlio/reset` and re-seeds the new run — or, with `skip_seed`, runs `auto_localize`'s search on it (the backend's `POST /api/v1/localization/auto`). **Blocks**, robot **stationary**. See below. |
| `relocalize_check` | `syncai_common/srv/IsValid` | `code: 1` always answers `valid: true` (liveness); anything else answers whether the first registration after the last guess converged. **The stack's only localization-quality signal.** |

Nothing is broadcast until the first cloud/odom pair has arrived; with no map
loaded the TF stays at identity.

### Three ways a guess arrives, two of them planar

1. **`relocalize`** takes the request's **raw 6-DOF** pose and (re)loads
   `pcd_path` as a side effect. Success is a **receipt, not a result**:
   registration runs on the timer with no deadline, and `relocalize_check` is
   the only thing that reports the outcome. It is also the first step of the
   backend's live map switch.
2. **`initialpose`** and 3. **the INI's `[initial_pose]`** (applied once, on
   the first odom) go through `applyPlanarGuess`: only x / y / yaw are taken,
   and roll / pitch / z are filled in from the **current estimate**. This is
   not a nicety. The lidar mount is tilted (~15° pitch) and the map is
   gravity-aligned, so the true `map → body` always carries that pitch; a flat
   guess tilts the whole scan against the map, far points exceed
   `rough_max_corr_dist`, rough GICP never passes its score, and because a
   pending guess also suspends the odom-feedback path the localizer freezes on
   the old offset and retries forever. Measured on the robot: an `initialpose`
   at (5.0, 0.982) hung; with the 14.6° pitch filled in the same spot
   converged on the first round.

   z is the exception an `initialpose` can make: the current estimate's z is
   the storey the localizer thinks it is on, so a guess on another storey
   starts a whole storey (3 m and more) out of the rough stage's reach and
   hangs the same way. A sender that knows the storey puts the ground's height
   in `position.z` and **marks it with a positive z variance**
   (`covariance[14]`); the body then goes `lidar_height` above it. The mark is
   what keeps rviz's 2D Pose Estimate and every older sender — `position.z` 0,
   covariance cell 0 — on the storey they are on, rather than pulled to the
   ground floor. The ground only has to be close: half a metre out (the
   console's "0 is the ground floor") is well inside `rough_max_corr_dist`.
   The INI's `[initial_pose]` has no z and stays on the current estimate's.

The consequence for callers: a `relocalize` from a 2D source (the console's
gridmap click) must be **followed by an `initialpose` publish**, which
overwrites the flat guess with a tilt-correct one before the next round. The
backend's map switch does exactly that, in that order. Keep both paths.

### `auto_localize`: the kidnap recovery

The three paths above all need someone to know roughly where the robot is.
After a kidnap (carried somewhere, booted away from `[initial_pose]`, or
localization simply lost) nobody does, and the timer keeps re-registering from
a wrong offset. `auto_localize` finds the robot with no hint
(`localizers/global_localizer.*`):

1. **Accumulate** the last `auto_loc.accum_scans` body clouds (10 ≈ 1 s; a
   single filter_num-6 scan is too sparse to tell places apart) into the newest
   scan's **gravity-level, yaw-zero** frame through the odom poses. Odom and the
   map are both gravity-aligned, so only x / y / z / yaw are searched — the
   mount tilt is carried, never searched, for the same reason as
   `applyPlanarGuess`.
2. **Candidates** are the map's own mapping trajectory: `poses.txt` beside the
   loaded PCD, thinned to `candidate_spacing`, × a `yaw_step_deg` sweep. Places
   the robot has stood are walkable by construction and carry the right z.
3. **Coarse**: a few hundred scan points per candidate, inlier ratio against
   the rough map's KD-tree (parallel, `num_threads`).
4. **Fine**: the `top_k` best *distinct* candidates (non-max suppressed, or
   they would all be one place at neighbouring yaws) get rough GICP (wider
   `auto_loc.rough_max_corr_dist`) + refine GICP and are scored by inlier ratio
   at refine resolution.
5. **Accept** only if fitness ≤ `refine_score_thresh`, inlier ratio ≥
   `min_inlier_ratio`, and no distinct runner-up (> `ambiguity_trans` or >
   `ambiguity_yaw_deg` away) scores ≥ `ambiguity_ratio` × the winner. A
   symmetric corridor fails as **ambiguous** instead of guessing.
6. **Hand over** as `map → pointlio_odom` re-applied to the newest odom pose,
   through `relocalize`'s path (`initial_guess`, gate bypass, settle window),
   and **wait for the timer's own converged-gated registration**.

Both halves have a deadline, `search_timeout_sec` (10 s, includes building the
GICP targets and KD-trees on the first call after a map load) and
`converge_timeout_sec` (5 s). The second is the important one: a guess the
timer cannot converge on is otherwise retried every round forever — exactly
the failure mode of `relocalize` / `initialpose` — so on that timeout the
candidate is **withdrawn**, the timer falls back to its previous offset +
odometry (`last_offset_*` is only written by a successful align, so it is
still the pre-call value), and the call fails. A client's timeout must exceed
the sum. A search timeout never returns a best-so-far: an unranked or
unchecked pose is not one to hand over.

Limits: only maps saved with `save_patches` have a `poses.txt`; a robot far
(≫ `candidate_spacing`) from the mapping trajectory may not be found; and if
Point-LIO itself has diverged (lifted, shaken, or starved — see below), this
service cannot help, because its scan window is posed by that odometry. Use
`reset_odometry` with `skip_seed: true`, which runs this same search on the
scans of a freshly reset Point-LIO — what the backend's
`POST /api/v1/localization/auto` calls since 2026-10-15, instead of this
service, because it cannot tell a kidnap that diverged the LIO from one that
did not. The acceptance
values (`min_inlier_ratio`, `ambiguity_ratio`) are first guesses, not
measured; tune them on a replay.

### `reset_odometry`: zeroing a diverged Point-LIO

What resetting the encoders is on a wheeled robot. Point-LIO can diverge
silently — on robot01 on 2026-10-13 `lio_odom` read ~1000 m/s with the robot
parked, 4 of 10 boots that week — and nothing above it recovers on its own:

- **`pointlio/reset` alone is not enough.** It restarts the odometry at the
  robot's current spot, but this node's `map → pointlio_odom` was solved
  against the *old* origin: the next guess is that old offset times an
  identity odom pose, unrelated to where the robot is, and GICP never
  converges.
- **`auto_localize` alone is not enough** while `recent_scans` holds scans
  posed by the diverged odometry: the accumulated cloud is smeared over
  kilometres.

`reset_odometry` does the whole sequence, in this order:

1. **Seed** — the request's `x` / `y` / `yaw` with `use_hint`, otherwise the
   **last trusted registration**: the last one that converged while the
   odometry it was chained to moved slower than `reset_odom.max_lio_speed`
   (3 m/s; the robot tops out at 0.6). A diverging LIO can still drag a GICP
   round or two to "converged", and the speed gate keeps those out of the
   seed; such a round logs `LIO moved … m/s … Point-LIO may be diverging`.
   z always comes from the last trusted registration. No seed → it fails
   before resetting anything.
2. **Pause, then reset**: the timer stops registering (it keeps rebroadcasting
   the TF) and `syncCB` drops every pair, then `pointlio/reset`.
3. **Gate**: only pairs stamped strictly after `ResetLIO`'s `last_odom_time`
   are accepted — pgo_node's boundary, same contract — and `recent_scans` is
   cleared. Waits for the first new-run pair (pointlio's static IMU init).
4. **Re-seed the offset directly**, with `applyPlanarGuess`'s construction
   (roll / pitch from the new run's gravity alignment, yaw turned to the
   seed's), so `map → body` is continuous from the first new odometry on —
   `syncai_lio_bridge` composes from the newest TF and holds no state, so it
   needs nothing. Then the seed goes to the timer as a service guess and the
   call waits for its converged registration.
5. **`global_fallback`**: if that times out, `auto_localize`'s search on the
   new-run scans, and another wait.

**`skip_seed`** (2026-10-15) is the kidnap variant: no seed is tried, so it
needs neither a hint nor a trusted registration (right after boot there is
none, and after a carry the last one is wrong by definition — trying it first
cost `converge_timeout_sec` and risked converging on a look-alike spot).
Step 4 still writes the offset from the map pose the TF showed before the
reset, purely to keep `map → body` continuous, but keeps the timer paused;
the call then waits for a full `auto_loc.accum_scans` window of new-run scans
(within `wait_lio_timeout_sec`; fewer is a warning, not a failure), runs the
search, and only then releases the timer onto the winner. Up to 2 + 10 + 10 +
5 = 27 s by default. The response carries the search's `z` / `fitness` /
`inlier_ratio` / `candidates` whenever it found a pose, with `method:
global` — the fields the backend's `POST /api/v1/localization/auto` returns.

On a failure after step 2 the reset is **not** undone (the old odometry was
the problem) and the offset stays at the seed (with `skip_seed`, the pre-reset
pose), so the TF stays continuous;
retry with a hint, or relocalize. `odom → base_link` jumps back to the origin
once — the local costmap's marks shift with it — which is one more reason the
robot has to be stopped first. **The robot must be standing still** for the
whole call: pointlio's re-init is a static, gravity-aligning one, and one done
in motion tilts the new odometry for the rest of the run with no error
anywhere. Nothing here can check that; cancel navigation before calling.

The client is in its own callback group and `main.cpp` runs **three**
threads (two until 2026-10): the handler blocks on `ResetLIO`'s future, and
the response has to be delivered on a thread the handler is not holding —
pgo_node's arrangement, for the same client.

This is the bandage, not the fix: a front end that cannot keep up with the
lidar (136 ms a scan against a 100 ms period on robot01) will diverge again.

### The motion gate

`update_hz` (5) is a ceiling, not a schedule. Parked for 60 s on robot01
(2026-09-21, dp2f map), the correction swung 33.7 / 29.0 mm / 1.07° while
Point-LIO underneath moved 7.8 / 6.9 mm / 0.27°: GICP's own noise floor being
resampled five times a second, which `syncai_robot_state` then sampled at a
random phase and the console showed as teleporting. So when the odom pose has
moved less than `min_update_trans` / `min_update_rot` since the last accepted
registration, the previous offset is rebroadcast and GICP is skipped; a
`max_update_interval` backstop re-registers anyway and blends the result in at
`static_blend_alpha`. Motion-triggered updates always use α = 1, so driving is
unchanged. A guess (any of the three) bypasses the gate and the blend and
opens a `post_reloc_settle` full-rate window, because the pose only converges
over the rounds that follow. The rejected alternatives (plain EMA, plain
deadband) and the numbers are in `params/localizer_params.yaml`; do not
re-propose them without new data. The chosen failure direction is "jitter
partly suppressed", never "frozen on a stale pose".

### Localizability: which directions a round is allowed to correct

Since 2026-10 a registration is not taken whole. GICP reports "converged"
whenever its Gauss-Newton step got small, which says nothing about whether
the solution was *determined*: in a corridor every point lies on the two
walls, nothing pins the pose along the axis, and the optimum slides along it
to wherever this round's sparse sampling favours — and that round passes
every gate the node has (`hasConverged`, the fitness bound, the inlier
count). The same mechanism produced the two-solution bounce in the west-end
corner on 2026-08-28 (`rough_registration_type` in the YAML). The information
about this was always there: small_gicp's final 6×6 Hessian
(`RegistrationResult::H`), whose eigenvector with a small eigenvalue is
exactly a direction the scan did not constrain.

`localizers/localizability.*` turns that into a per-direction decision — the
full / partial / none classification of X-ICP (Tuna et al., T-RO 2024), with
a damping gain in place of their re-solve:

1. `H` is expressed in the map frame (`blockdiag(R,R) · H · blockdiag(R,R)ᵀ`;
   small_gicp's `delta` is a body-frame right perturbation, `[ω; v]`),
   divided by the inlier count, and each block is reduced to its **marginal**
   information (Schur complement), because yaw and lateral translation are
   coupled at range and the plain block overstates the lateral constraint.
2. The eigenvalues of each 3×3 are classified by their ratio to the largest:
   below `localizability.ratio_none` → NONE, below `ratio_partial` → PARTIAL,
   otherwise FULL (plus an optional absolute floor, `min_eig_per_inlier`).
3. The correction — GICP's solution minus the odometry prediction,
   translation and rotation vector separately — is projected: FULL ×1,
   PARTIAL × `partial_gain`, NONE ×0. Along a NONE direction the pose stays
   where Point-LIO put it. All three translation directions NONE is an
   odometry-only round (`round degenerate` in the log; nothing is advanced,
   the next tick registers again).

It is a projection of the *update*, never a rejection of the round: a
corridor round still corrects the two directions it can see. A service guess
and the settle window after it are exempt (a discontinuity by design, and the
prediction may be metres off), and so is the first registration after start.
Every accepted round logs one line:

```
loc: t-eig=(0.021 3.1 4.4) r-eig=(…) lvl=NFF/FFF weak=12deg inl=0.91 ms=38.2 dt=0.112->0.004 dr=0.31->0.30deg
```

eigenvalues weakest first (per inlier, map frame), the levels, the heading of
the weakest translation direction, the refine inlier ratio, the round's wall
time, and how much of the correction survived the projection. **The
thresholds are first guesses.** Set `ratio_none` / `ratio_partial` from the
dp1f replay's corridor stretches against its open-space rounds before
trusting them (`docs/localizer-offline-replay.md`, "Judging the result").
`localizability.enabled: false` keeps the analysis and the log and projects
nothing — the pre-2026-10 behaviour, for an A/B on the same bag.

### The jump gate

A registration whose map-frame body pose is further from the last
**accepted** one than the robot could have moved meanwhile
(`jump_gate.max_speed` 1.5 m/s, `max_rot_speed` 1.5 rad/s, over the two
scans' stamp difference, at least 0.2 s) is dropped, and the TF coasts on
odometry for that round (`jump rejected` in the log). The reference is
deliberately the last accepted body pose and not the odometry prediction: when Point-LIO diverges (robot01,
2026-10-13) the prediction is garbage while GICP's answer is right, and the
offset absorbing that jump is what kept the TF correct — a gate against the
prediction would have thrown those rounds away. What this catches is the
other case, a solution in the wrong basin. After `max_reject_streak` (5)
consecutive rejections the next round is accepted regardless, because by
then it is the reference that is suspect: the gate must never freeze the TF
on a wrong pose for good. Service guesses, their settle window and the round
that straddles `reset_odometry`'s jump are exempt.

The interval was the timer's until 2026-10-23, and that was a bug: the 5 Hz
rounds and the 10 Hz scans do not line up, so two rounds 0.2 s apart often
compared scans 0.3–0.5 s apart, and a robot walking at 0.7–1.1 m/s read as
1.6–3 m/s. The `loc_1022` replay on dp (12 min, 540 m) rejected 8 correct
rounds that way. `reset_odom.max_lio_speed`'s trust check divides by the same
stamp interval now.

### The source window

The tracking round registers the newest `track_accum_scans` (3) body clouds,
composed through their odom poses into the newest scan's body frame, as one
cloud — what `auto_localize` has always done over `auto_loc.accum_scans`,
only shorter. A single filter-num-6 MID360 scan is ~3k points on a
non-repetitive pattern: sparse enough that the rough stage dithers and that a
parked robot's solution resamples GICP's noise floor every round (the reason
the motion gate exists). Three scans is ~0.3 s of coverage; Point-LIO's drift
over that is well under a centimetre and each scan is already de-skewed to
its end. `1` is the single-scan behaviour until 2026-10, bit for bit. The
cost is the refine stage's source size: the `refine:` log line carries the
round's wall time (`ms=`) since 2026-10, and rough + refine has to stay under
half the `update_hz` period. Not yet measured on the robot.

### Guess timeout

A guess from `relocalize`, `initialpose` or the INI that the timer cannot
converge on used to be retried every round forever, with the TF frozen on
the old offset meanwhile (a flat guess on the tilted mount is the classic
case). Since 2026-10 such a guess is withdrawn after `guess_timeout_sec`
(10 s; `guess withdrawn` in the log) and the timer goes back to its previous
offset + odometry; `relocalize_check` keeps answering `false` for it, as
before. `auto_localize` and `reset_odometry` own their guesses and keep their
own deadlines, unchanged. `<= 0` restores the old behaviour.

## Parameters

`params/localizer_params.yaml`, keyed `/**/localizer_node:` so it applies at
any namespace. Every key is a `declare_parameter` whose default repeats the
struct default, so a missing key degrades to the code default instead of
throwing. The launch layers the instance values on top.

| Parameter | Default (YAML) | Launch value | Notes |
|---|---|---|---|
| `cloud_topic` / `odom_topic` | `/pointlio/body_cloud` / `/pointlio/lio_odom` | `/<robot_id>/pointlio/…` | Absolute, into pointlio's namespace |
| `map_frame` | `map` | | Never namespaced |
| `local_frame` | `pointlio_odom` | `<robot_id>/pointlio_odom` | Replaced by the first odom's `frame_id`, but the 10 ms timer broadcasts `map → local_frame` before that, so the launch sets the same name pointlio broadcasts. The YAML said `laser` until 2026-10, and that unprefixed frame was on TF until pointlio's first message |
| `map_path` | `""` | `[map] pcd`, made absolute | Loaded in the constructor. Empty → the node runs but cannot localize until `relocalize` |
| `set_initial_pose`, `initial_pose.{x,y,yaw}` | `false`, `0.0` | from `[initial_pose]` if the section exists | Applied once on the first odom, through `applyPlanarGuess` |
| `lidar_height` | `0.481` m | | Lidar above the feet: how far above a marked `initialpose` z (a ground height) the guess puts the body. Same as the controller's and planner's `lidar_height` |
| `update_hz` | `5.0` | | Ceiling on re-registration; was 1.0 until 2026-08-28 (staircase corrections at 0.7 m/s) |
| `min_update_trans` / `min_update_rot` | `0.05` m / `0.02` rad | | Motion gate thresholds |
| `max_update_interval` | `2.0` s | | Backstop re-registration when parked |
| `static_blend_alpha` | `0.1` | | EMA weight for a backstop refresh; `1.0` keeps the gate and drops the blend |
| `post_reloc_settle` | `3.0` s | | Full-rate window after a guess |
| `track_accum_scans` | `3` | | Body clouds composed into the tracking round's source (above); `1` = single scan, the pre-2026-10 path |
| `min_inlier_ratio` | `0.0` (off) | | Refine inlier-ratio gate; the ratio is logged every round, set it from off-site data |
| `guess_timeout_sec` | `10.0` s | | Withdraw an un-owned guess the timer cannot converge on; `<= 0` retries forever |
| `localizability.{enabled, ratio_none, ratio_partial, min_eig_per_inlier, partial_gain}` | `true`, `0.02`, `0.1`, `0.0`, `0.3` | | Per-direction projection of the correction (above). The thresholds are **first guesses**, to be set from the `loc:` log |
| `jump_gate.{enabled, max_speed, max_rot_speed, max_reject_streak}` | `true`, `1.5` m/s, `1.5` rad/s, `5` | | Against the last accepted body pose (above); the streak cap is what keeps it from freezing the TF |
| `num_threads` | `4` | | GICP covariance / KD-tree / reduction threads. Read **once**: `RegistrationPCL` builds the tree at `setInputTarget`. Shares the CPU with pointlio |
| `num_neighbors` | `20` | | Local-covariance neighbours (pcl::GICP's correspondence randomness) |
| `rough_*` | `0.25` / `0.25` m, `20` iters, score `0.2`, corr `2.0` m, `GICP`, voxel `1.0` | | Absorbs a hand-entered guess. VGICP was tried and reverted (double solutions in degenerate geometry, 2026-08-28) |
| `rough_rotation_eps` / `rough_translation_eps` | `0.0087` rad / `0.005` m | | Convergence tolerance of the rough Gauss-Newton step. Code default is small_gicp's `0.002` / `0.0005`; the YAML loosens it to 0.5° / 5 mm (2026-10-04, see "Gotchas") |
| `refine_*` | `0.1` / `0.1` m, `20` iters, score `0.1`, corr `0.5` m, `GICP`, voxel `0.5` | | The fine stage |
| `refine_rotation_eps` / `refine_translation_eps` | `0.002` rad / `0.0005` m | | Same for refine; kept at small_gicp's defaults, since this solution becomes the TF |

`auto_loc.*` (the global search; struct `GlobalLocConfig`, see the section above):

| Parameter | Default | Notes |
|---|---|---|
| `accum_scans` | `10` | Body clouds accumulated per search |
| `candidate_spacing` | `1.0` m | Keyframe thinning |
| `yaw_step_deg` | `15.0` | Yaw sweep per position |
| `coarse_scan_resolution` / `coarse_max_points` / `coarse_inlier_dist` | `0.5` m / `400` / `1.0` m | Coarse stage |
| `top_k` | `10` | Distinct candidates given GICP |
| `rough_max_corr_dist` / `max_iteration` | `3.0` m / `50` | Fine-stage GICP (starts further off than the timer's) |
| `refine_inlier_dist` | `0.3` m | Final inlier ratio |
| `min_inlier_ratio` / `ambiguity_ratio` | `0.6` / `0.9` | Acceptance — **unmeasured first guesses** |
| `ambiguity_trans` / `ambiguity_yaw_deg` | `1.0` m / `10.0` | What counts as a distinct runner-up |
| `search_timeout_sec` / `converge_timeout_sec` | `10.0` / `5.0` s | `<= 0` disables |

`reset_odom.*` (see the section above):

| Parameter | Default | Notes |
|---|---|---|
| `lio_reset_service` | `pointlio/reset` | Relative → `/<robot_id>/pointlio/reset` |
| `lio_reset_timeout_sec` | `2.0` s | Wait for `ResetLIO`'s response |
| `wait_lio_timeout_sec` | `10.0` s | Wait for the first new-run odometry (IMU init) |
| `converge_timeout_sec` | `5.0` s | Wait for the timer to register against the seed |
| `max_lio_speed` | `3.0` m/s | A registration seeds a later reset only below this; never affects the TF |

A client's timeout must exceed the sum (17 s), plus `auto_loc`'s two (15 s)
with `global_fallback`.

**Write floats** for double parameters (`2.0`, not `2`): a bare integer is an
int64 override and the node dies with `InvalidParameterTypeException`.

## Running

```bash
ros2 launch syncai_localizer localizer.launch.py                        # ~/robot_ws/config/system.ini
ros2 launch syncai_localizer localizer.launch.py system_config:=/path/to/robot01.ini
```

`system_config:=` has one trap the other launches do not: a **relative**
`[map] pcd` (the instance INIs write `map/%(name)s/map.pcd`) is resolved
against *two directories above the INI*, because on the robot the INI is
`config/system.ini` and that is the workspace root. Pointing the argument at
`config/instances/robot01.ini` therefore looks for `config/map/<name>/map.pcd`
and starts nothing (`[map] pcd '…/config/map/dp2f/map.pcd' does not exist`).
For a bare run, use an INI whose `pcd` is absolute, or one that sits at
`<workspace>/config/`.

The nav session (`config/sessions/start_nav.yaml`) runs it as the second pane
of the `localization` window, after `syncai_pointlio` (map_server has its own
window ahead of them since 2026-09-30), logging
to `log/stack/<robot_id>/localizer/`. It is deliberately **absent** from the
mapping session, and could not be present anyway: **the launch starts nothing
when `[map] pcd` is missing or absent on disk** (`nothing to launch` in the
log). That is intentional — the node loads the map in its constructor and a
localizer with no map fails every call silently — so do not make it start
anyway. Launching this alone starts no LIO; run `syncai_pointlio` first.

```bash
# coarse pose in the map frame, radians; loads the PCD again as a side effect
ros2 service call /<robot_id>/relocalize syncai_common/srv/Relocalize \
  "{pcd_path: '/abs/path/map/<name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
# then a planar guess, so roll/pitch/z are filled in from the current estimate
ros2 topic pub --once /<robot_id>/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
# the same on another storey: z is the ground there, marked by the z variance (cell 14)
ros2 topic pub --once /<robot_id>/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 5.2}, orientation: {w: 1.0}}, covariance: [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}}"
# did the first registration after that guess succeed?
ros2 service call /<robot_id>/relocalize_check syncai_common/srv/IsValid "{code: 0}"

# kidnapped: find the robot with no hint (blocks up to ~15 s)
ros2 service call /<robot_id>/auto_localize syncai_common/srv/AutoLocalize "{}"

# Point-LIO diverged (lio_odom velocities in the hundreds of m/s): STOP THE
# ROBOT, then zero the odometry from the last trusted pose (up to ~17 s) ...
ros2 service call /<robot_id>/reset_odometry syncai_common/srv/ResetOdometry "{}"
# ... or from a hint, with the global search as the net (up to ~32 s)
ros2 service call /<robot_id>/reset_odometry syncai_common/srv/ResetOdometry \
  "{use_hint: true, x: 1.0, y: 2.0, yaw: 0.0, global_fallback: true}"
# ... or kidnapped as well: no seed, the global search on the new run (up to ~27 s)
ros2 service call /<robot_id>/reset_odometry syncai_common/srv/ResetOdometry "{skip_seed: true}"
```

Replaying a recorded bag offline (what to record, where to run it, and how to
judge the result) is `docs/localizer-offline-replay.md`.

Checking it:

```bash
ros2 node info /<robot_id>/localizer_node                 # subs /<id>/pointlio/{body_cloud,lio_odom}, initialpose; srvs relocalize, relocalize_check, auto_localize, reset_odometry
ros2 param get /<robot_id>/localizer_node update_hz       # 5.0 → the params file was found
ros2 run tf2_ros tf2_echo map <robot_id>/pointlio_odom    # the correction; identity until a guess converges
```

| Log line | Meaning |
|---|---|
| `params: cloud_topic=/robot01/pointlio/body_cloud …` | The overrides landed; `/pointlio/…` here means the launch was bypassed |
| `map preloaded from …` | Constructor `loadMap` succeeded; `map_cloud` was published |
| `initial pose from config: … (applied on first odom)` | `[initial_pose]` was read; `guess from config applied` follows on the first odom |
| `guess from initialpose applied: … (roll/pitch/z from current estimate)` | An `initialpose` went through `applyPlanarGuess` |
| `guess from initialpose applied: … (z from the given ground, …)` | The same, with a marked z: the storey came from the sender |
| `rough : converged=no iters=20 …` every round | Stuck: the guess is too far off, or a flat guess on the tilted mount — publish an `initialpose` |
| `refine: converged=yes … fitness=0.0xx ms=38.2` | Both stages converged (`ms` is the round's wall time); whether the TF moved is decided by the lines below |
| `loc: t-eig=(…) r-eig=(…) lvl=NFF/FFF weak=12deg inl=0.91 ms=38.2 dt=0.112->0.004 …` | One per accepted round: the localizability eigenvalues (weakest first, per inlier, map frame), the levels, the weakest translation heading, the inlier ratio, and what the projection kept of the correction. `(not projected)` on a guess / settle round |
| `round degenerate: no translation direction constrained` | All three translation directions NONE; odometry only this round |
| `jump rejected (n/5): body moved … m / … deg in … s` | The jump gate dropped the round; `accepting a … jump after 5 rejected rounds` is the streak cap letting the next one through |
| `guess withdrawn: no converged registration within 10.0 s` | `guess_timeout_sec`; back to the previous offset + odometry, `relocalize_check` stays `false` |
| `loc stats (since start): run=… accepted=… not_converged=… degenerate=… jump_rejected=… projected=…` | Once a minute, the round counters — a replay's acceptance rate without counting lines |
| `coarse: N candidates … best 0.xx, K kept for GICP` | `auto_localize` coarse stage; one `fine: …` line per kept candidate follows |
| `search failed …: ambiguous, a distinct pose scores alike` | Symmetric surroundings; move the robot somewhere more distinctive and retry |
| `… the timer did not converge on it within …; withdrawn` | The search's pose did not hold against a fresh scan; localization is as before the call |
| `LIO moved … m/s > reset_odom.max_lio_speed …` | Point-LIO is probably diverging; the round still updated the TF but will not seed `reset_odometry` |
| `reset_odometry: seed from … resetting Point-LIO` | A reset started; `new odometry after … s; seeded` and then `odometry reset; localized from …` (or `reset_odometry failed: …`) follow |
| `initialpose ignored: map not loaded …` | No `map_path` and no `relocalize` yet; the message was dropped, not queued |

## Gotchas

- **The namespace is `/<robot_id>`, not `/<robot_id>/localizer`.** See above.
  The rviz config in `config/rviz2/` reads `/<robot_id>/map_cloud`.
- **`relocalize` success is a receipt.** Poll `relocalize_check`. A guess
  the timer cannot converge on is withdrawn after `guess_timeout_sec` (10 s)
  since 2026-10, so `relocalize_check` answering `false` ten seconds later
  means the guess is gone, not still pending.
  **`auto_localize` and `reset_odometry` success is a result** — the
  opposite — and both block.
  `RobotState.localization_valid` is TF presence only and reads true against a
  map the robot was never localized in; do not turn it into a quality signal
  from anything but `relocalize_check`.
- **`relocCB` takes the raw 6-DOF pose; the other two paths are planar.** A
  `relocalize` from a 2D source must be followed by an `initialpose`, or the
  tilted mount leaves rough GICP retrying a flat guess forever.
- **`small_gicp` reports `converged` only on a small update step.** Hitting
  `max_iteration` is *not converged*, unlike PCL's ICP, and `align()` treats
  `hasConverged()` as a hard condition. Iteration counts below small_gicp's
  default of 20 silently stop the TF from ever updating; the log prints
  `converged=` and `fitness=` before the threshold check so the two failures
  can be told apart. The step tolerance behind that flag is a parameter per
  stage since 2026-10-04 (`*_rotation_eps` / `*_translation_eps`). With the
  library's 0.11° / 0.5 mm on the rough stage, the dp1f replay lost 5.9% of
  its rounds to `rough : converged=no iters=19 … fitness=0.011` — a correct fit
  dithering below the voxel size — in streaks of up to 10 rounds (2 s of
  odometry coasting) while moving; the YAML's 0.5° / 5 mm left 1 of 3013 in
  the same replay (accepted rounds 92.5% → 97.9%), with the pose error against
  the map's keyframes unchanged (median 1.5 cm, p95 4.2 cm). Refine keeps the
  library defaults.
- **The correction's translation is not a drift gauge.** `map →
  pointlio_odom` is `t = t_body − R · t_odom`: a 1° change in its rotation
  moves its translation by 1 m for a robot 60 m from where pointlio started,
  while the body pose itself moves millimetres. In the dp1f replay that
  translation swung ±3 m during the run with the localized body within 2 cm
  (median) of the map's keyframes throughout. Judge drift on `map → base_link`
  (the keyframe comparison in the analysis), never on `tf2_echo map
  pointlio_odom`.
- **`setInput` allocates a fresh cloud every round on purpose.**
  `RegistrationPCL::setInputSource` early-returns on an identical pointer and
  would then register the new scan with the previous scan's covariances.
- **`num_threads` is read once, at construction.** Changing it at runtime does
  not rebuild the KD-tree.
- **Three-thread `MultiThreadedExecutor`, and it is load-bearing.** The timer +
  subscriptions sit on the default group; `relocalize`, `relocalize_check`,
  `auto_localize`, `reset_odometry` and `initialpose` on their own
  MutuallyExclusive group, so a multi-second `loadMap` never gaps the TF
  rebroadcast; and `reset_odometry`'s `ResetLIO` client on a third, because
  the handler blocks on the client's future (two threads until 2026-10).
  `ICPLocalizer::m_target_mutex` exists for the first split (`loadMap` builds
  into locals and swaps under it; `align` holds it for one round). Collapsing
  to `rclcpp::spin()` is safe but reintroduces the gap; adding a fourth group
  needs a look at `NodeState`'s two mutexes first.
- **Lint is partly off.** `ament_cmake_copyright` and `cpplint` are disabled
  as they were upstream: `localizers/` has no per-file licence headers (the
  package `LICENSE` covers it). The node shell follows the workspace
  `.clang-format`.
- **Verified so far in a container, not yet on the robot.** A full build from
  a fresh `syncai-robot-base` image with the fork deleted (22 packages,
  2026-09-28) plus a launch against a synthetic `map.pcd`: node at
  `/robot01/localizer_node`, both pointlio topics overridden, map preloaded,
  `[initial_pose]` read, `relocalize_check` answering `true` for `code: 1` and
  `false` for `code: 0` before any registration, `relocalize` refusing a
  missing PCD and accepting the map. Registration itself (a real map, pointlio
  running, the console's map switch) still needs the robot. The 2026-10
  localizability projection, jump gate and source window are in the same
  state: the analysis has a unit test (`test/test_localizability.cpp`,
  synthetic Hessians, `colcon test --packages-select syncai_localizer`), the
  node's behaviour with them has been checked on neither a replay nor the
  robot yet — the dp1f replay is the first thing to run.
- **A stale fork checkout builds a second `localizer` package.** If
  `src/third-party/FASTLIO2_ROS2` is still on disk from before this port,
  colcon builds its `localizer` too (same executable name, different package).
  Delete it, and `build/localizer` + `install/localizer`, so only one launch
  exists for this node.
