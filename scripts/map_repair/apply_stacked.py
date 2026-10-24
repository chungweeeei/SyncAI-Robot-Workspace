#!/usr/bin/env python3
"""Write the stacked record (octomap_edits_stacked.json) into the map's raw tree.

    apply_stacked.py map/dp
    apply_stacked.py map/dp --roundtrip     # rewrite with no edits; must come out identical

apply_octomap_edits takes one ground key per column and cannot clear a voxel. The stacked record
holds what it cannot do: fill_key, fills on columns whose record key is another floor (a
stairwell's landings and flights share columns), and free_key, voxels to clear (a door). This
writes them into octomap.raw.bt -- fills occupied with the 0.6 m over each marked free where it
is unknown, as apply_octomap_edits does; clears free -- after a dated backup. apply_octomap_edits
starts from octomap.raw.bt every time, so run it afterwards and octomap.bt has both; later runs
keep them. Writing the same record twice changes nothing.

A new save of the map (or a new tree from clean_map) replaces the raw tree: run this again then.
With no octomap.raw.bt yet (no apply_octomap_edits run on this save), octomap.bt is copied there
first, which is what apply_octomap_edits' own first run does.
"""
import argparse
import os
import shutil

from octomap_bt import CLEARANCE_CELLS, backup, load_stacked, rewrite

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("map_dir")
ap.add_argument("--roundtrip", action="store_true", help="check the writer: no edits, same bytes")
a = ap.parse_args()

raw = os.path.join(a.map_dir, "octomap.raw.bt")
if not os.path.isfile(raw):
    shutil.copy2(os.path.join(a.map_dir, "octomap.bt"), raw)
    print(f"no {raw}: copied octomap.bt there, as apply_octomap_edits' first run does")
if a.roundtrip:
    out = raw + ".roundtrip"
    rewrite(raw, out)
    same = open(raw, "rb").read() == open(out, "rb").read()
    os.remove(out)
    raise SystemExit(0 if same else f"roundtrip DIFFERS from {raw}: do not use the writer")
doc = load_stacked(a.map_dir)
fills = doc["fill_key"]
clear = [[x, y, z + dz] for x, y, z in fills for dz in range(1, CLEARANCE_CELLS + 1)]
if not fills and not doc["free_key"]:
    raise SystemExit("the stacked record is empty: nothing to write")
print(f"backup: {backup(raw)}")
changes = rewrite(raw, raw, occupied=fills, free=doc["free_key"], free_if_unknown=clear)
print(f"{raw}: {len(fills)} fills, {len(doc['free_key'])} clears -> "
      + ", ".join(f"{k} {v}" for k, v in changes.items()))
print("now: ros2 run syncai_mapping apply_octomap_edits", a.map_dir)
