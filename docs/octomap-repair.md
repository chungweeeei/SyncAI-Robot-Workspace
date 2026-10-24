# Repairing a saved map's octomap: stairs, pits, landings, doors

A saved map's `octomap.bt` is what OctomapPlanner3D plans on, and the simulator
(SyncAI-Robot-Simulation) builds the floor the legs stand on from it. A real robot walks over
small faults in it; a planner routes around or through them, and in the sim they are terrain a
gait policy trips on. This page is how to find those faults and fix them in the map, and what dp
taught (2026-10-24, the sim's G23 climbing dp's stairwell 1F -> 2F). The tools are in
`scripts/map_repair/`; the skill `.claude/skills/octomap-repair` drives them.

Everything here edits map data under `map/<name>/`, which is gitignored: there is no history but
the backups the tools make. Never edit by hand without one.

## What goes wrong, and why

The map is built from several passes of the mapping robot. Three things leave faults:

1. **The lidar never sees the ground near the robot.** Its lower edge (about -7 deg) misses the
   floor within ~4 m, so a tread the robot was standing in front of, a landing it crossed once,
   the floor under a turn are holes or are filled by clean_map's floor fill at a guessed height.
2. **Passes drift apart in height.** On dp the whole stairwell, landing 2 up to the stair head on
   2F, sits 0.13-0.15 m under the rest: the mapping robot's own height over landing 2 is 2.48-2.50
   where the map says 2.34, at the stair head 5.11 over a measured 4.97, while landing 1 and the 2F
   corridor agree with it. Where a drifted patch meets the rest there is a step, a trench or a
   ridge that is not in the building -- a 0.21 m step across the middle of landing 2, a 0.25 m
   trench between flight 2's top and landing 2, a 0.16 m "last step" onto 2F that the planner does
   not even see (its heights are the voxels') and that PPO fell on at 0.7 m/s.
3. **Voxels quantise.** A 0.17 m riser on a 0.1 m grid becomes 0.1 and 0.2, a 0.28 m tread two
   or three rows; one tread came out a single row (0.1 m) deep between 3.36 and 3.66, which the
   feet cannot use, so it was a 0.3 m riser and HimLoco tipped over backwards there twice.

And, separately, **things that moved**: a door shut in one pass and open in another is a wall
in the map (dp's 2F corridor door at x -0.6: a one-voxel-thick column of 23 voxels, 5.3-7.6 m,
between its frame's two thicker ones).

## The reference: the mapping track

`map/<name>/poses.txt` is where the mapping robot's IMU was at every keyframe, in the same frame
as the map, and it is one continuous estimate -- it does not drift between passes the way two
passes' points drift against each other. Its ground is `z - 0.481` (clean_map's lidar_height),
give or take ~0.1 m for the gait's body height. Two rules follow, and every repair on dp came from
one of them:

- **Where the track walked level, the floor is level.** A step, pit or trench under a stretch of
  keyframes whose ground stays within a few centimetres is not in the building. (Its side
  of the drift tells which part is right: on landing 2 the track's 2.48-2.50 sits 5 cm over the west
  part (2.55) and 15 cm over the east part (2.34), so the east part is the drifted one.)
- **Where the track climbed, the floor climbed by as much**, over the stretch it climbed in. A
  riser the track rose 0.17 over is one riser; a 0.55 m wall where the track rose 0.1 is missing
  treads.

`inspect_map.py --track` prints the keyframes in a patch with their ground next to the map.

## Kinds of fault, and the tool for each

| Seen in `inspect_map.py` | Is | Fix |
|---|---|---|
| a column or a few 0.15-0.5 m under the tread they sit in, the track over them | a pit: an unseen tread | `find_pits.py` finds them along every flight; `--write` |
| a landing in two levels 0.15-0.25 m apart, the track level across | drift between passes | `fill_box.py` the low part up to the high part's voxel |
| a trench between a flight and its landing, the track level or rising across | drift | `fill_box.py` the trench up to the landing |
| a wall of 0.4-0.6 m at a flight's foot, the track rising ~0.1 there | missing treads | `fill_box.py` row by row, treads as deep as the flight's others (0.2-0.3 m) |
| a tread one row (0.1 m) deep | quantisation | `fill_box.py` the row before it to its height |
| a one-voxel-thick full-height column across a corridor, the track through it | a door, shut in another pass | `erase_box.py` |

Things that are **not** faults: the gap between two flights of a U stair (the floor below seen
through it, 1 m and more down: `find_pits.py --max` keeps it out), a real kerb or threshold the
track stepped over, a newel, a handrail. If the track does not say otherwise, leave it.

### Shape the fix like the building

- Treads as deep as the flight's other treads. On dp the first fix of flight 3's foot put three
  0.1 m treads in 0.3 m; the trunk's nose met the third before the feet reached the first, and the
  robot could not leave the landing (5 of 5 headless runs). Spread over 0.6 m (treads of 0.3 and
  0.2) it climbed.
- When a fix moves an edge, look at what is now beside it. Levelling landing 2 but leaving the
  rows next to flight 2 alone kept a 0.25 m trench there, now beside a raised landing, and the
  robot fell into it coming off flight 2.
- Check the planner's landing report afterwards (planner_server log, `stair links: landing N: no
  room to turn ...`). A fix that shortens a landing shortens its turning room: flight 3's spread
  treads took landing 2 from 0.08 m short to 0.18 m short, until levelling the landing gave it back
  (0.02 m short).

## The records, and the one-key-per-column rule

- `octomap_edits.json` -- the console editor's record, applied by
  `ros2 run syncai_mapping apply_octomap_edits map/<name>`: each key an occupied ground voxel
  with the 0.6 m over it marked free where unknown. **One key per column**: a second key on a
  column is refused ("names a column already patched"). It starts from `octomap.raw.bt` every
  time, so the result is a function of the record alone.
- `octomap_edits_stacked.json` -- what that cannot express, kept by these tools: `fill_key`, fills
  on columns whose record key is another floor (a stairwell's landings and flights stack on the
  same columns: on dp landing 1 at 1.2 m and landing 3 at 3.8 m, landing 2 at 2.6 m and the stair
  head at 5.1 m), and `free_key`, voxels to clear (a door). `apply_stacked.py` writes it into
  `octomap.raw.bt`; `apply_octomap_edits` then carries it into `octomap.bt` on every run.

Replacing a record key with another floor's height silently undoes a repair: on dp levelling
landing 2 replaced twelve stair-head keys (5.1) with landing keys (2.6) and the stair head's pit
came back. `fill_box.py` and `find_pits.py` route such columns to the stacked record instead.

A new save of the map, or a new tree from clean_map, replaces `octomap.raw.bt`: run
`apply_stacked.py` again, then `apply_octomap_edits`.

## Procedure

Run the tools from `scripts/map_repair/` (or with that directory on the path); they need numpy
only. Reading a tree takes 10-60 s.

1. **Find it.** Start from where the robot fails (the sim's `FELL at (x, y)`, a controller that
   stops) or from the planner's warnings. `inspect_map.py map/<name> --box ... --band ... --track`
   around it: tops, then `--count` for walls and doors.
2. **Decide against the track** (above). Write down the rows, the target height and why.
3. **Dry run.** `fill_box.py` / `erase_box.py` / `find_pits.py` without `--write` print every
   column they would change. Check the list against the view.
4. **Write.** `--write` (records backed up), then `apply_stacked.py map/<name>` if the tool said
   so, then `ros2 run syncai_mapping apply_octomap_edits map/<name>`.
5. **Verify the tree.** `inspect_map.py` the patch again; for stacked columns check both floors
   are there (`--band` around each). Re-run `find_pits.py`: what it still lists is either a
   column it routed elsewhere or the next fault.
6. **Verify by walking**, in the sim: restart its session (it rebuilds its world from the tree)
   and navigate across the fixed place, both ways, several times. On dp, ten alternating
   1F <-> 2F navigations through the full stack, 10 of 10, came only after five rounds of
   fault -> fix -> retest; one run proving nothing was usual.

## The sim's side

The simulator stands the feet on `map.pcd`'s measured heights, not the voxel tops, and it ramps
steps between drifted passes that it can find under a level stretch of track
(SyncAI-Robot-Simulation's `surface_heights.py`). So a fault can look different in the sim than in
the tree, and a fix in the tree reaches the sim through those measured heights: a filled voxel with
no point in it takes its neighbours' measured height, its own layer first. When the sim and the
tree disagree, look at both (`inspect_map.py` for the tree; the sim's own terrain dump for the
feet).
