---
name: octomap-repair
description: Repair a saved map's octomap.bt (map/<name>/) where it disagrees with the building -- pits in stair treads, missing or too-narrow treads, landings in two levels, trenches or steps between drifted mapping passes, a door or object that should not be there -- using scripts/map_repair and apply_octomap_edits. Use when the robot or the simulator falls, stalls or trips at a spot on stairs or a floor, when the planner warns about a landing or refuses a route that the real robot walks, when asked to fill holes / pits / steps in the octomap, level a landing, fix stairs in the map, or remove a door / object from the octomap.
---

# Octomap repair

The why, the kinds of fault, the records and the dp examples are in `docs/octomap-repair.md` --
read it before the first edit. This skill is how to run that procedure.

## Ground rules

- `map/<name>/` is gitignored map data: the only history is the tools' backups. Every `--write`
  backs up what it touches; before anything else (a hand edit, a delete), copy the file aside.
- Run the tools from the workspace root as `python3 scripts/map_repair/<tool>.py map/<name> ...`
  (numpy only: host, robot container or sim container). `apply_octomap_edits` needs the ROS
  environment: `source /opt/ros/humble/setup.bash; source install/setup.bash`.
- Tools read a whole tree (10-60 s each). Batch several looks into one command.
- Ask before writing to a map a robot is running on, and before any fix the track does not
  support (see "Decide"). Dry runs and `inspect_map.py` need no asking.
- Never put a second key on a column in `octomap_edits.json`, and never replace a key that is
  another floor's: use the tools, which route those to `octomap_edits_stacked.json`.
- Never commit map files. Never `colcon build` for this.

## 1. Find it

From the failure: the sim log's `[sim] FELL at (x, y) roll .. pitch ..`, the controller's
`Failed to make progress` with the robot's pose, the planner's `stair links: landing N: no room to
turn ...`. Then look, tops and track together:

```bash
python3 scripts/map_repair/inspect_map.py map/dp --box X0 X1 Y0 Y1 --band ZLO ZHI --track
python3 scripts/map_repair/inspect_map.py map/dp --box ... --band ... --count   # walls, doors
python3 scripts/map_repair/find_pits.py map/dp                                  # every flight
```

Pick the band around the floor in question (a stairwell stacks floors on the same columns: a band
of 1 m around one floor shows only it). Show the user the grid and say what you see in it.

## 2. Decide

Against the mapping track (the doc's two rules): level track over a step, pit or trench means the
step is not real; the track climbing means treads, as many as it climbed. Write down for the user:
the rows or box, the target height, which side is right and why, before any `--write`. Shape fixes
like the building (treads 0.2-0.3 m deep, like the flight's others) and look at what a fix leaves
beside it (a trench next to a raised landing).

## 3. Dry run, then write

```bash
python3 scripts/map_repair/fill_box.py map/dp --box X0 X1 Y0 Y1 --top T --band ZLO ZHI [--skip ...]
python3 scripts/map_repair/erase_box.py map/dp --box X0 X1 Y0 Y1 --near FLOOR_Z --zhi TOP_Z
python3 scripts/map_repair/find_pits.py map/dp
```

Check the printed columns against the grid; then the same with `--write`. Then, in this order:

```bash
python3 scripts/map_repair/apply_stacked.py map/dp        # only if a tool said "stacked" / erase
ros2 run syncai_mapping apply_octomap_edits map/dp
```

`apply_stacked.py --roundtrip` first on a map these tools have not touched before (it must print
nothing and exit 0: the writer reproduces the tree byte for byte).

## 4. Verify

- The tree: `inspect_map.py` the patch again; for a stacked column check each floor with its own
  band; `find_pits.py` again; for an erase, `--count` shows the column empty above the floor.
- The planner: restart it (or the session) and read its `buildMap` lines: landing reports should
  not get worse.
- Walking: in the simulator (SyncAI-Robot-Simulation), restart its session so it rebuilds from the
  tree, then navigate across the place both ways, several times, through the full stack
  (`scripts/nav_test.py` there). One success proves little; on dp it took ten alternating runs.
  On a failure: the sim log's `FELL` line, the controller log around it, then back to step 1.

## Report

What was wrong (with the numbers from the grid and the track), what was changed (columns, heights,
which record), what was verified and how, the backups' names, and anything found but left alone
(pits off the route, faults the track does not settle) for the user to decide.
