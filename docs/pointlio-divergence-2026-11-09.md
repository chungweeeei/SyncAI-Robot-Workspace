# The map gate closed on a staircase and never reopened (2026-11-09, robot01)

Traditional Chinese translation: `pointlio-divergence-2026-11-09.zh-TW.md` (this file is the
canonical copy).

This failure is not the slow erosion of 2026-10-30 → 11-08 (`pointlio-divergence-2026-11-08.md`).
Point-LIO diverged 7 minutes after a fresh start. It was running the build of
`fix/pointlio-map-gate-gravity` with U2 (9e3afa3: `IVoxMap` holds one point per voxel,
`map_resolution` 0.2) and U3 (d31ef90: `MapInsertGate`, `map_gate.enabled: true`). It was the first
staircase climb in robot01's logs since 10-30.

## Summary

Point-LIO was started 13:51:58 on the dock. After two MOVEs on the ground floor, a MOVE routed up a
switchback staircase: four flights, z 0.15 → 5.6 m, 13:57:41–13:58:33.

1. **Point-LIO tracked the climb well.** All through it the localizer's per-round correction was
   1–3 cm (one round 8.6 cm / 1.1°), with fitness 0.01–0.03.
2. **The gate closed on a healthy pose** (§3.1). On the stairs the share of matched points that fit a
   plane (effective / near) fell from 92–95 % to a per-second worst of 87–89 % while `near` stayed
   at 100 %. At the foot of the fourth flight three scans in a row came in under 85 % (the last
   at 82 %, 610 of 745). The gate closed at 13:58:27.69 and the local map froze at 1436 grids /
   92 053 points.
3. **A closed gate cannot reopen while the robot moves into unmapped space** (§3.2). Reopening needs
   50 passing scans in a row. The fourth flight and the top level were never written, so the further
   the robot walked, the fewer of its points had map neighbours (near 100 → 56 %) and the lower the
   share went (effective / near ≈ 50 %). No scan could pass.
4. **Then the usual run-away** (§3.3). Walking east on the top level at 13:59:01, only 3–4 % of the
   points were still matched. Point-LIO's speed went 2.2 → 4.7 → 8.6 → 20 m/s, so the speed test
   failed as well. From 13:59:10 Point-LIO was on the IMU alone (1640 m/s by 14:02).
5. **The controller stopped the robot by itself.** At 13:59:04.8 the optimizer failed and the
   controller commanded zero, after one 0.12 m back-out. Every retry after that failed ("robot far
   off the path", then a stale local costmap), and the client cancelled the goal at 13:59:20.

**Mitigation in place:** fc5cba2 (14:17) sets `map_gate.enabled: false` in the params file. The run
restarted at 14:18 has every scan inserted again.

**Resolution:** the gate was removed (§7.1). It only ever acted between a run-away and the reset
that clears the local map anyway, and the share it judged cannot tell a wrong pose from a surface
the map has not got yet.

**Not verified:** that the same staircase passes with the gate off. The run after 14:18 stayed on
the top level (§6).

## 1. Timeline (CST)

| Time | What happened |
|---|---|
| 13:51:58 | `pointlio_node` started on the dock, from the U2 + U3 build (`refused` / `unwritten` on the health line) |
| 13:55:52 → 13:56:51 | MOVE dock (−9.15, −4.48) → (−5.24, −15.24), then → (−10.49, −13.61), the stair foot. ~1 m/s; the map grows to 1327 grids / 80 k points |
| 13:56:53 → 13:57:36 | Standing at the stair foot |
| 13:57:35.5 | MOVE → (−5.93, −16.75) on the top level. 13:57:35.68 stair mode; policy settled 13:57:38.73 |
| 13:57:41.7 → 13:57:46.6 | Flight 1, z 0.15 → 1.65, north. The localizer rejects one round at the stair entry: "0.15 m / 25.2 deg in 0.20 s" (the real pitch-up, §5) |
| 13:57:49.50 → 13:58:04.47 | Localizer DEGRADED (1 not converged, 1 jump rejected), then OK |
| 13:57:56.5 → 13:58:01.8 | Flight 2, z 1.6 → 3.0, south |
| 13:58:12.5 → 13:58:17.2 | Flight 3, z 3.0 → 4.3, north |
| **13:58:27.69** | Foot of flight 4: `reportMapGate` "3 scans in a row failed, the last on effective/near 82% (610 of 745)". **Gate closed** |
| 13:58:27.7 → 13:58:29.7 | Localizer correction 8.6 cm / 1.1°, then back to ≤ 3 cm |
| 13:58:27.2 → 13:58:32.6 | Flight 4, z 4.3 → 5.6, south. Nothing written |
| 13:58:32.78 → 13:58:36.58 | Stair mode left, policy settled |
| 13:58:33 → 13:58:49.8 | Top level, walking south to the goal (succeeded). near 100 → 61 %, eff 78 → 29 % |
| 13:58:51 → 13:59:00 | Standing. near 57 %, eff 28 %, Point-LIO still (v 0.06 m/s) |
| 13:58:59.49 | Localizer OK → DEGRADED: mean correction 4.2 cm |
| 13:58:59.8 | MOVE → (10.56, −17.05), 16.7 m east |
| 13:59:01 → 13:59:05 | Walking east: eff 25 → 4 %, the localizer's correction per round 2 → 5 → 9 → 13 → 18 → 27 cm |
| 13:59:04.80 | Controller: optimizer failed, commanding zero; "Every candidate collides: backing out 0.30 m", backed out 0.12 m |
| 13:59:05.66 | Localizer DEGRADED → LOST: "Point-LIO moved at up to 2.35 m/s" |
| 13:59:06.99 → 13:59:19.6 | Controller: "robot far off the path", commanding zero; FollowPath aborted and retried four times |
| 13:59:10 | Point-LIO eff 0 %, v 20 m/s. IMU only |
| 13:59:17.50 | Localizer LOST → NO_ODOM: "no cloud + odom pair for 1.3 s" |
| 13:59:20.09 | Goal cancelled by the client |
| 14:02 | Point-LIO v 1640 m/s, eff 0 %, ~1700 scans not written |
| 14:17 | fc5cba2: `map_gate.enabled: false` |
| 14:18 | Stack restarted with the robot on the top level; relocalized 14:22 |

## 2. Health by segment

Per-second health lines (`lio health`). "eff" is effective / all points; near was 100 % up to the
gate closing, so eff equals effective / near there. "min" is the worst scan of any second in the
segment.

| Segment | s | near | eff mean / min | res mean / max (cm) | sat acc | max \|a\| | v max | Map at end (grids / points) | Not written |
|---|---|---|---|---|---|---|---|---|---|
| Flat walk (two MOVEs) | 58 | 99–100 % | 92.9 / 89 % | 2.55 / 3.19 | 62 | 39.3 | 1.17 | 1321 / 79 506 | 0 |
| Standing at the stair foot | 42 | 100 % | 94.7 / 93 % | 2.03 / 2.11 | 0 | 9.7 | 0.07 | 1332 / 80 604 | 0 |
| Flight 1, z 0.15 → 1.65 | 6 | 100 % | 91.7 / 88 % | 2.60 / 2.91 | 0 | 29.3 | 0.93 | 1364 / 83 316 | 0 |
| Landing, z 1.6 | 9 | 100 % | 91.7 / 89 % | 2.21 / 2.52 | 0 | 22.1 | 0.85 | 1378 / 84 936 | 0 |
| Flight 2, z 1.6 → 3.0 | 6 | 100 % | 92.3 / 89 % | 2.47 / 2.76 | 0 | 26.8 | 0.92 | 1386 / 86 481 | 0 |
| Landing, z 3.0 | 10 | 100 % | 91.2 / 88 % | 2.33 / 2.56 | 6 | 37.5 | 0.81 | 1410 / 88 048 | 0 |
| Flight 3, z 3.0 → 4.3 | 5 | 100 % | 91.2 / 87 % | 2.97 / 3.84 | 2 | 33.3 | 1.05 | 1426 / 90 039 | 0 |
| Landing, z 4.3, to the foot of flight 4 | 10 | 100 % | 89.7 / 84 % | 2.44 / 3.32 | 5 | 39.2 | 0.95 | 1436 / 92 053 | 1 |
| Flight 4, z 4.3 → 5.6 (gate closed) | 5 | 100 % | 87.0 / 80 % | 3.58 / 4.59 | 0 | 27.9 | 0.87 | frozen | 51 |
| Top level, walking south | 18 | 61–100 % | 57.5 / 26 % | 4.01 / 4.82 | 5 | 34.9 | 1.12 | frozen | 232 |
| Top level, standing | 10 | 56–59 % | 27.7 / 26 % | 4.48 / 4.91 | 0 | 12.0 | 0.19 | frozen | 332 |
| Top level, walking east | 5 | 34–56 % | 12.8 / 3 % | 10.26 / 15.99 | 8 | 39.3 | 2.17 | frozen | 382 |

## 3. Mechanism

```
 climbing          ─ the pose is right (localizer correction 1–3 cm)
                                   │
                                   ▼
 each new tread    ─ within 2.24 m of the tread below, already in the map → counted near;
                     its 5 nearest map points lie on the tread below, the riser and the wall
                     → no plane → not effective → effective / near 92–95 % → 87–89 %
                                   │ the foot of flight 4: 3 scans < 85 % (82 %)
                                   ▼
 gate closed       ─ the map freezes; only 50 passing scans in a row reopen it
                                   │
                                   ▼
 dead lock         ─ the robot walks into what was never written → fewer neighbours
                     → lower share → no scan passes → nothing written → ...
                                   │ walking east on the top level, along its weak axis
                                   ▼
 run-away          ─ 3–4 % of the points matched; nothing holds the velocity along the
                     corridor (localizer weak direction 1–5°) → 2 → 20 m/s
                     → the speed test fails too → eff 0 % → IMU only
```

### 3.1 Why the gate closed

How the lidar update classifies a point (`LidarProcessor`, `lidar_processor.cpp`):

- **near:** the 5 nearest map points lie within √5 ≈ 2.24 m.
- **effective:** those 5 fit a plane (`esti_plane`, `plane_thr`) and the point lies on it.

`MapInsertGate` judges effective / near because, as its header puts it, "a scan entering a place
the map has not seen has few near points". That holds in a corridor. It does not hold on a staircase:

- Each new tread is one riser above the tread before it, which is already in the map.
- A point on the new tread therefore has 5 neighbours within 2.24 m, so it counts as near (100 %
  throughout the climb).
- Those neighbours lie on the tread below, the riser and the wall, and fit no plane through the
  point.

So the share falls because the map lacks the surface yet, not because the pose is wrong. Two more
things narrowed the margin:

- **Fewer points per scan in the stairwell.** 500–800 downsampled points, against 950–1700 on the
  flat.
- **Thresholds measured elsewhere.** The 0.85 threshold was measured on the corridor patrol of
  11-08 and on the old leaky map, and was not re-checked on the 0.2 m map (the header and fc5cba2
  say so). Even this run's flat walk reached a worst scan of 89 %, a 4-point margin.

The share drifted down flight by flight (worst scans 88, 89, 88, 87, 84 %). The gate closed on the
first scans of flight 4 (§2).

### 3.2 Why it never reopened

From the header: "Nothing but passing scans reopens it -- a gate that stays closed means Point-LIO
no longer matches its own map, and the way out is a reset". That assumes the map already covers
where the robot goes, as it does in a corridor after the first lap. On a fresh start, or on a level
Point-LIO has never been on, the assumption fails. A closed gate keeps the robot's surroundings out
of the map, the match share falls with every metre, and passing is the only way to reopen.

- 13:58:27–13:59:10: 432 scans in a row not written.
- **No reset came.** The localizer's LOST at 13:59:05 logs "reset_odometry with the robot standing
  still"; nothing calls it (§7.3).

### 3.3 The run-away

- **Standing on the top level** (13:58:51–13:59:00), 28 % of the points were still matched and
  Point-LIO stayed put.
- **Walking east** (13:59:01) along the top level's corridor, the localizer's weak direction was
  1–5° (the corridor's axis) and the share fell to 3–4 %.
- **The localizer's corrections** grew along the walk, 2 → 27 cm per round, and Point-LIO's speed
  rose ~4 m/s every second.
- **From 13:59:10** no point was matched.

The shape is that of the earlier divergences (the IMU constrains acceleration, not velocity). What
is new is that the gate starved the filter of map: there was no slide to start it.

## 4. The gate against the case it was made for

`MapInsertGate` was added for the 11-08 run-away, where Point-LIO ran 37 m north with the robot
standing at P4 and wrote a shifted copy of the corridor (`pointlio-divergence-2026-11-08.md` §3.3).
Any gate has to tell the two cases apart:

| | 11-08 16:16 run-away | 11-09 staircase |
|---|---|---|
| Pose | wrong (37 m off) | right (1–3 cm) |
| Point-LIO speed | 1.8–2.2 m/s | ≤ 1.05 m/s |
| effective / near | 81 % at 16:16:04, 47 % ten seconds later | 87–89 %, 82 % at the foot of flight 4 |
| Should the scans be written | no | yes |

From the numbers in `map_insert_gate.h`'s header (from the 11-08 data):

- **The speed test closed the gate first.** Speed passed 1.5 m/s at 16:16:03 and the gate closed
  then; the share was not under 85 % until 16:16:04. Closing it on 11-08 did not need the share
  test.
- **The share test then held it shut.** The header's replay: reopening after 50 scans rather than
  10 is what kept it shut through the four minutes after.
- **The share test is what closed the gate on the staircase.**

## 5. Ruled out, or not Point-LIO

- **Point-LIO losing the climb by itself:** its pose held to 1–3 cm through flights 1–3 and the
  start of flight 4 with every scan written; it degraded only after the map froze.
- **U2 (one point per 0.2 m voxel):** the climb ran on it, and the pose held while scans were still
  written. It may have lowered the share somewhat (§6.2); it did not lose the track.
- **Scan skipping:** none after startup (one `reportBacklog` at 13:52:03, 3 scans skipped at
  startup); scan cost stayed under 80 ms.
- **Accelerometer saturation:** max |a| reached the 39.3 m/s² clip on the stairs, but more often on
  the flat walk (62 saturated samples in 58 s, against 13 over the four flights and landings).
- **The localizer's jump rejection at 13:57:41.95** (0.15 m / 25.2° in 0.20 s): the real pitch-up
  at the stair foot, the body's pitch going +14° → −12°. One round dropped (flight 1's entry only).
  Harmless here, but `jump_gate.max_rot_speed` 1.5 rad/s can reject a real stair entry.
- **The robot's own motion after the divergence:** the controller commanded zero from 13:59:04.8,
  after one 0.12 m back-out, and kept the robot stopped until the cancel.

## 6. Not verified

1. **The staircase with the gate off.**
   - The operator reports that builds without the gate climb stairs without diverging.
   - These logs (robot01 since 10-30) have no other stair-mode entry.
   - The run with the gate off (from 14:18) stayed on the top level: one split-level stair, 0.75 m
     down at 14:32:53–58 and back up at 14:33:25–29, worst scan 92 %. The gate would not have
     closed there either, so it does not test this staircase.
2. **Whether U2 lowers the share on stairs.** A sparser map fits fewer planes. A climb of the
   pre-U2 build, recorded, would show it.
3. **The localizer's NO_ODOM at 13:59:17.5** and the controller's TF filter dropping
   `pointlio_body` from 13:59:15, while Point-LIO kept processing 10 scans a second. Cause not
   determined.
4. **What a speed-closed gate does after a run-away.** Expected, not replayed: with the copy kept
   out, a Point-LIO 37 m off its frozen map loses its match as it did here, and diverges in
   seconds to minutes rather than running six hours on a copy. Either way it needs a reset.

## 7. Next steps

**Back up first.** There is no bag of this event: `record/lio_ring/` is empty and the recorder
(`scripts/record/lio_ring_record.sh`, `docker-compose.recorder.yml`) was not running. The logs of
§8 are the only record, and multilog rotates them out.

### 7.1 Remove the gate

Done: d31ef90 and fc5cba2 reverted, every scan is written as before d31ef90. Why not the
speed-only gate of §7.2:

- **What it protects is thrown away anyway.** `pointlio/reset` starts an empty `IVoxMap`, so the gate
  only matters between a run-away and the reset. With it or without it the pose is wrong there and
  a reset is the way out (§6.4); the localizer, matching the independent `map.pcd`, already reports
  LOST (seven times on 11-08).
- **It catches what cannot recover and misses what could.** 1.5 m/s catches a full run-away, which
  never comes back by itself; a slide the old map could still pull back (16:05 on 11-08) passes both
  tests.
- **Any reopening judged on the local map dead-locks** once the robot walks somewhere the frozen
  map does not cover, which is §3.2.

What is worth keeping is the detection, not the gate: a run-away's speed raising a signal that
starts a reset (§7.3), and velocity measurements (zero-velocity updates, leg kinematics) that stop
the run-away in the filter. Both are in `todo.md` (1.2–1.4). §7.2 below is the comparison that led
here.

### 7.2 What the gate should judge

Each option against the two cases of §4:

- **Speed only.**
  - *11-08 run-away:* closed at 16:16:03, with the 4 grids written before it.
  - *Staircase:* never closes.
  - *Cost:* once the run-away stops, 50 slow scans reopen the gate with the pose still 37 m off, and
    the copy is written then.
- **Speed closes, the share only gates reopening.**
  - *11-08 run-away:* closed at 16:16:03 by speed. The share, failing at the wrong pose, holds it
    shut until a reset.
  - *Staircase:* speed never fails, so the share is never consulted.
  - *Cost:* while open the share must not drop single scans either, or the staircase becomes a
    slower version of the same dead lock. The hold after a run-away needs the 11-08 replay. It
    depends on a reset nothing triggers (§7.3).
- **near from a smaller radius.**
  - *11-08 run-away:* as today.
  - *Staircase:* a point on a new tread with no neighbour within, e.g., 0.5 m no longer counts as
    near, so it stays out of the share.
  - *Cost:* changes the statistic the update and the health line report. The thresholds on the
    flat and on stairs need measuring again. No data yet on the radius that excludes a tread.
- **Close with a time limit.**
  - *11-08 run-away:* closed, then reopened after N s whatever the pose. If the pose is still wrong,
    the copy is written then.
  - *Staircase:* reopens after N s, so N s of the climb are lost from the map.
  - *Cost:* trades the dead lock for a delay. Neither case is handled by what it judges.

### 7.3 Who calls the reset

Any gate that stays closed until a reset depends on one, and today none comes on its own:

- **`pointlio/reset` (ResetLIO)** builds a fresh `MapBuilder`: empty `IVoxMap`, the gate open, and
  a static IMU init, so the robot must be still.
- **The localizer's `reset_odometry`** seeds it from a hint or the last trusted registration and
  keeps `map → body` continuous. The only caller is the backend's `POST /api/v1/localization/auto`
  (with `skip_seed`), from an operator.
- **The localizer's LOST** only logs the advice (13:59:05.66 here; seven times on 11-08).

`todo.md` 1.4 (reset on the dock after a LOST) covers the dock only. A reset mid-route also needs
the navigation stopped and the robot still first, which only the backend or the task_runner can
arrange.

### 7.4 Record the staircase

Start the recorder (`docker-compose.recorder.yml`), then climb this staircase with the gate off.
That climb:

- answers §6.1;
- gives the replay data every gate threshold needs (the stairwell share);
- with a pre-U2 build as well, answers §6.2.

## 8. Data and method

| Source | Where | Notes |
|---|---|---|
| Point-LIO health, 1 Hz | `log/stack/robot01/pointlio/@400000006af165ca135a5e5c.u` | The 13:51:58–14:18 run (rotated at the 14:18 restart); `reportMapGate` at 13:58:27.69 |
| Localizer rounds | `log/stack/robot01/localizer/@400000006af165ca2e7c0c1c.u` | `guess` (pose, pitch, z), `loc:` (`dt` / `dr`, weak direction), health transitions |
| Controller | `log/stack/robot01/controller/@400000006af165cc0447d534.u` | Stair mode, landing turns, failures after 13:59:04 |
| Task outcomes | `log/stack/robot01/task_runner/@400000006af165cc1e041adc.u` | MOVE goals and their outcomes |
| Gate-off run | `log/stack/robot01/{pointlio,localizer,controller}/current` | From 14:18; split-level stair at 14:32:44 and 14:33:19 |
| Code | `src/syncai_pointlio/` | `map_insert_gate.h` (the gate and its 11-08 numbers), `lidar_processor.cpp` (near / effective), `pointlio_node.cpp` (`resetCB`) |

How the numbers were made:

- **Health lines** parsed into one row per second; segment windows from the localizer's z and the
  controller's stair-mode lines.
- **Flights** are the stretches where the localizer's z rises at constant rate with the body's
  pitch at −3 to −15° (+12 to +17° on the flat; the lidar is mounted pitched).
- **The localizer's correction** is the per-round `dt` / `dr` of its `loc:` lines.
- **Earlier stair climbs** were searched for as "Entering stair mode" in every controller log, and
  as a 1.2 m rise in z within 25 s in every localizer log since 10-30. Only today's runs enter
  stair mode. The other z rises start at the dock or at (0, 0), not at this staircase.
