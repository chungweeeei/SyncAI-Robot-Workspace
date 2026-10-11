# Removing people from `map.pcd` on `dev` — change plan

Status: **implemented on this branch** (2026-10-10; numbers in the
`syncai_mapping` README's "Cleaning map.pcd"). Kept as the record of what was
ported and why. This branch (`feat/mapping-dynamic-removal`,
cut from `dev` at `c2f1d2f`) is where it lands. The work was prototyped and
measured on `feat/mapping-octomap-loop-noise`; that branch also carries the
OctoMap display build, which this one deliberately does **not** take, so the
3D pcd → 2D pgm path (the backend's gridmap conversion from `map.pcd`) can be
maintained on its own.

## Why

On `map/dp1f_1008_2` (1F + 2F, 1519 s bag, 2004 keyframes) people walking
the east corridor stay in `map.pcd` as silhouettes: `pgo_node`'s `saveMapsCB`
concatenates every point of every keyframe with no filtering. Those points end
up in the gridmap the backend derives from `map.pcd`.

What the prototype established (all on `dp1f_1008_2`):

- In the corridor interior at person height (0.3–1.8 m above the floor) 89 %
  of the occupied 0.1 m voxels were hit by exactly one keyframe; real wall
  voxels are hit by a median of 4–5.
- A person is only recognisable as one once a **later** scan sees through the
  voxel (often from another angle, often on a revisit), and the judgement
  needs the **final, loop-closed** poses: before closure the two passes of the
  east corridor were 1.2–2 m apart, so an online check would delete real wall
  at every revisit. Hence: offline, after the save. Decided 2026-10-09 —
  cleanest final map over a clean live preview.
- On the old sparse keyframes (`body_cloud`, 2,640 points each) the rules
  below remove 31 % of all points and 10–24 % of the walls. On the 3× denser,
  deskewed keyframes (`body_cloud_dense`, 7,846 points) they remove 7 % and
  keep 91–94 % of the walls. The dense input is a prerequisite, not an
  option.

## What goes into this branch

### 1. Loop-closure weights and tilt anchor (`syncai_mapping`)

Cherry-pick `21decae` from `feat/mapping-octomap-loop-noise`
("give pgo loop edges real weight, anchor keyframe tilt"):

- `loop_noise_xy_sigma_m` (0.05) and `loop_noise_yaw_sigma_deg` (1.5) replace
  the fork's variance = fitness on the `PlanarLoopFactor`;
- `keyframe_tilt_sigma_deg` (0.5): new `pgos/gravity_prior_factor.h`, one
  `GravityPriorFactor` per keyframe holding roll / pitch to Point-LIO's.

Expect conflicts in `src/syncai_mapping/README.md`, `params/mapping_params.yaml`
and `CLAUDE.md`: the commit was written on top of the OctoMap commits, whose
text is not on `dev`. Code files should apply cleanly. Note
`loop_search_radius` stays 1.0 in the yaml; the validated maps used 3.0.

### 2. Deskewed clouds and `body_cloud_dense` (`syncai_pointlio`)

Cherry-pick `9e7cb2f` ("deskew published clouds, add body_cloud_dense for
mapping"). Touches `syncai_pointlio` only; should apply cleanly.

- Every published cloud is motion-compensated into the body frame at
  `cloud_end_time` (`MapBuilder::deskewToEndBody`). Fast-turning scans: local
  plane residual 1.9 cm → 0.55 cm; straight scans unchanged.
- `body_cloud_dense`: same scan at `dense_filter_num: 2` (3× points),
  x/y/z/intensity only, ~0.12 MB a scan, published only while subscribed. The
  LIO filter sees exactly the points it saw before.

Then point pgo at it (the two hunks from `c327059`, not the whole commit):

- `src/syncai_mapping/launch/mapping.launch.py`: `cloud_topic` →
  `/{robot_id}/pointlio/body_cloud_dense`;
- `src/syncai_mapping/params/mapping_params.yaml`: fallback
  `cloud_topic: /pointlio/body_cloud_dense`.

### 3. New executable `clean_map` (`syncai_mapping`)

A standalone, detached post-save job that rewrites `map.pcd` without people
and one-off returns. It is the cleaning half of the prototype's
`build_octomap` (`octomap_builder.cpp` at `c327059`) with the OctoMap outputs
removed: no `.bt`, no road / occupied layers.

**Input:** `<dir>/poses.txt` + `<dir>/patches/<i>.pcd` (so it needs a save
with `save_patches`). **Output:** `<dir>/map.pcd`, replaced via
`map.pcd.tmp` + rename, and a status sidecar `<dir>/map_clean.recipe.json`.

**Algorithm** (port from `octomap_builder.cpp`; keep the numbers):

1. Per keyframe, world points `w = q·b + t`, dropping `‖b‖ < min_range`
   (0.5 m). Points more than `floor_band − resolution` (0.15 m) below the
   keyframe's floor (`t.z − lidar_height`) are **clipped**: their ray stops at
   that height and their endpoint is not a hit (`worldScan` / `castScan`).
   0.4 % of returns are such floor reflections; their rays would count
   misses under the floor.
2. **Pass 1 — hits.** `octomap::OcTree::computeDiscreteUpdate` (only for its
   key arithmetic and ray traversal; the tree is never updated, so memory is
   just the hash map) gives the occupied keys per scan; `++H` per key, once per
   scan. Clipped returns: `++clipped` on their key.
3. **Pass 2 — misses.** Same scans again; for each free key already in the
   map, `++M`. Keyed on hit voxels only (keying every crossed voxel is every
   free voxel of the site: GBs).
4. **Classify** each counted voxel, with `fz` = local floor (see below):
   - *dynamic*: `M ≥ dynamic_min_miss (3)`, `M ≥ dynamic_miss_ratio (2.0) × H`,
     and `fz + floor_band ≤ z ≤ fz + max_height`;
   - *sparse*: `H + clipped < min_hits (2)`, not dynamic, not within
     `±floor_band` of `fz` (the floor is sampled thinly; applying the rule
     there took 17 % of a corridor floor);
   - otherwise static.
5. **Pass 3 — rewrite.** Concatenate every patch exactly as `saveMapsCB` does
   (`pcl::transformPointCloud(t, q)`, `PointXYZI`, no `min_range` cut, no
   voxel filter), dropping a point only if its voxel is counted and not
   static. Points nobody counted (beyond `max_range`, inside `min_range`) stay.

**Local floor** (`localFloor(x, y, z)` at `c327059`): per 1 m cell, the
keyframes within `floor_radius` (10 m); of those whose floor is at or below
`z + floor_band`, the highest level (within 1 m of the highest), and of that
level the nearest in xy. The xy-only version put 1F under 2F on 2F's floor
(51 of 54 failing 1F path cells), disabling the rules there.

**Process model** — port from `pgo_node.cpp` at `8718221` / `c327059`
(`startOctomapBuild`, `failOctomapBuild`, `reapOctomapBuilds`,
`stopOctomapBuildFor`, `removeOctomapOutputs`), renamed for the cleaner:

- `saveMapsCB`, after a successful save with `save_patches`: write the
  sidecar `converting` synchronously, then `posix_spawn` `clean_map <dir>`
  with `POSIX_SPAWN_SETSID`, default signals, nice 10; the child raises its
  own `oom_score_adj` to 500 and serialises on a `flock` under
  `/dev/shm/syncai_pgo/` (shared with the backend via `ipc: host`).
- The 1 s status timer reaps it (`waitpid WNOHANG`, one log line per outcome).
- A save into a directory whose clean is still running: SIGTERM it first, and
  remove `map_clean.recipe.json` + `map.pcd.tmp`.
- The child fingerprints `poses.txt` (dev, inode, size, mtime) at start and
  re-checks before the rename; on mismatch it fails as `superseded` instead of
  renaming an old save's map over a new one. After taking the lock it re-marks
  the sidecar `converting` (a stopped predecessor may have written `failed`).

**Sidecar** `map_clean.recipe.json` (port `octomap_recipe.cpp`): `status`
`converting` → `ok` / `failed`, `started_at`, `finished_at`, `params`
(resolution, max_range, min_range, lidar_height, floor_band, max_height,
floor_radius, min_hits, dynamic_miss_ratio, dynamic_min_miss,
clip_below_floor), `measurements` (keyframes, skipped_patches, dynamic_voxels,
sparse_voxels, clipped_rays, map_points_removed, map_points_kept,
elapsed_s), `error`. Bare names only: nothing in a map directory may name the
map or hold an absolute path.

**CLI:** `clean_map <dir> [--resolution 0.1] [--max-range 20] [--min-range 0.5]
[--lidar-height 0.481] [--floor-band 0.25] [--max-height 2.0]
[--floor-radius 10] [--min-hits 2] [--dynamic-miss-ratio 2.0]
[--dynamic-min-miss 3] [--clip-below-floor 1] [--nice 0] [--lock <path>|none]
[--started-at <ISO-8601>]`; exit 0 ok · 1 usage · 2 no input · 3 failed ·
4 interrupted. `ros2 run syncai_mapping clean_map map/<name>` is the by-hand
rerun; it always rebuilds from `patches/`, so it is idempotent.

**ROS parameters** (`pgo_node`, `params/mapping_params.yaml`):
`map_clean_enabled` (true), `map_clean_nice` (10, int), and
`map_clean_{resolution, max_range, min_range, lidar_height, floor_band,
max_height, floor_radius, min_hits (int), dynamic_miss_ratio,
dynamic_min_miss (int), clip_below_floor}` with the defaults above.

**Build:**

- `CMakeLists.txt`: new executable `clean_map` (sources: `clean_map_main.cpp`,
  `map_cleaner.cpp`, `map_clean_recipe.cpp`), `find_package(octomap)`, PCL;
  `pgo_node` links neither OctoMap nor the cleaner (keep the header free of
  OctoMap / PCL includes, as `octomap_builder.hpp` is).
- `package.xml`: `<depend>octomap</depend>` (a rosdep key).
- `Dockerfile`, `dev` stage: `ros-humble-octomap`, in its own stanza **below**
  the ros2-rust underlay so that layer's cache survives (take the hunk from
  `a9796ab`).

### 4. Documentation

- `src/syncai_pointlio/README.md`: comes with `9e7cb2f`.
- `src/syncai_mapping/README.md`: inputs (`body_cloud_dense`), the save table
  (`map.pcd` replaced minutes later, the sidecar), a "Cleaning map.pcd"
  section (rules, numbers, why offline), the parameter rows, the by-hand
  command.
- `CLAUDE.md`: the `syncai_pointlio` and `syncai_mapping` rows, the
  `map/<name>/` row of the backend contract, and a bullet under "Facts about
  this stack the backend depends on".

## Cross-repository contract

`map.pcd` changes **after** `save_maps` responds: raw at first, the cleaned
copy once `map_clean.recipe.json` says `ok`. Rename is atomic (an open reader
keeps the old inode). A gridmap conversion started inside that window uses the
raw map and should be rerun when the status flips; the backend does not do
this today. Say so in the commit message. A hard kill leaves the sidecar at
`converting` forever — readers age it out by `started_at`, as with the OctoMap
sidecar on the other branch.

## Verification

Offline in the Mac replay container (`syncai-bagrun`), never on the robot:

1. With `--min-hits 1 --dynamic-min-miss 0` nothing is dropped, and the
   rewritten `map.pcd` must match the one pgo saved point for point (same
   count, same order, every point within 1 mm). Not byte-identical: pass 3 uses
   `poses.txt`, which `saveMapsCB` prints at 6 significant digits, while pgo
   transformed with the full-precision pose.
2. `dp1f_1008_2` (dense keyframes) with the defaults, compared against the
   prototype's numbers in the east corridor (walls x ≈ 15.8 / 18.7, y 10–20):

   | Region | kept |
   |---|---|
   | interior, person height | ≈ 12 % |
   | left / right wall | ≈ 91 % / 94 % |
   | floor | 100 % |
   | stairs treads (kf 1403–1454) | ≈ 99 % |

   and about 1.2 M of 15.4 M points removed overall.
3. `dp1f_1006` as a single-floor control: walls and floor essentially intact.
4. Run time and peak RSS: no tree is built, so expect well under the
   prototype's 11 min / 1.5 GB; measure on the Jetson before relying on it.
5. Convert the cleaned `map.pcd` with the backend's gridmap recipe and check
   the corridor is free of person-shaped obstacles.
