#!/usr/bin/env python3
"""Raise the ground of a box of columns to one height: a landing in two levels, a trench, a
missing or too-narrow tread.

    fill_box.py map/dp --box -10.75 -8.45 -13.85 -12.65 --top 2.6 --band 2.15 2.75
    fill_box.py map/dp --box ... --top ... --band ... --skip -9.75 -8.4 -12.4 -12.0
    fill_box.py map/dp --box ... --top ... --band ... --write

--box takes cell CENTRES (x0 x1 y0 y1, both ends included: -10.75 -8.45 is columns -10.8..-8.4
by their lower edges). --top is the new ground, a voxel top (2.6 = the voxel 2.5-2.6). --band is
the height window that is "this floor": a column is raised when its highest voxel with its centre
in the band tops out under --below (default --top - 0.05) and not under the band's bottom + 0.1
(no ground there at all is left alone: a well, a hole to the floor below). --skip boxes (centres,
repeatable) are left out: a flight's treads beside the landing being levelled.

The tops are read from the tree apply_octomap_edits starts from (octomap.raw.bt, or octomap.bt
before the first apply), since the record is applied to that.

With --write: a column whose record key is on this floor (within 0.5 m of --top), or that has none,
gets --top in octomap_edits.json; one that holds another floor's key (a stairwell's landings share
columns) goes to the stacked record instead, for apply_stacked.py. Both files are backed up first.
Without it the tool only prints what it would do. Then apply_stacked.py if it says so, then
apply_octomap_edits (docs/octomap-repair.md).
"""
import argparse
import os

from octomap_bt import (Columns, backup, centre_m, key, load_record, load_stacked, raw_tree,
                        save_record, save_stacked, top_m)

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("map_dir")
ap.add_argument("--box", nargs=4, type=float, metavar=("X0", "X1", "Y0", "Y1"), required=True)
ap.add_argument("--top", type=float, required=True, help="new ground, a voxel top (m)")
ap.add_argument("--band", nargs=2, type=float, metavar=("ZLO", "ZHI"), required=True)
ap.add_argument("--below", type=float, default=None, help="raise columns topping out under this")
ap.add_argument("--skip", nargs=4, type=float, action="append", default=[],
                metavar=("X0", "X1", "Y0", "Y1"), help="leave these columns (centres) alone")
ap.add_argument("--write", action="store_true")
a = ap.parse_args()

x0, x1, y0, y1 = a.box
below = a.top - 0.05 if a.below is None else a.below
cols = Columns.from_file(raw_tree(a.map_dir), (x0 - 0.1, x1 + 0.1, y0 - 0.1, y1 + 0.1))
res = cols.res
want = key(a.top - res / 2, res)
record = load_record(a.map_dir)
have = {(k[0], k[1]): k[2] for k in record["fill_key"]}
fills, stacked = 0, []
for ix in range(key(x0, res), key(x1, res) + 1):
    for iy in range(key(y0, res), key(y1, res) + 1):
        cx, cy = centre_m(ix, res), centre_m(iy, res)
        if any(s[0] - 1e-6 <= cx <= s[1] + 1e-6 and s[2] - 1e-6 <= cy <= s[3] + 1e-6 for s in a.skip):
            continue
        t = cols.top(ix, iy, *a.band)
        if t is None or top_m(t, res) >= below - 1e-6 or top_m(t, res) < a.band[0] + 0.1 - 1e-6:
            continue
        old = have.get((ix, iy))
        if old == want:
            continue
        if old is not None and abs(top_m(old, res) - a.top) > 0.5:
            stacked.append([ix, iy, want])
            how = f"stacked (record has {top_m(old, res):.1f})"
        else:
            have[(ix, iy)] = want
            fills += 1
            how = "record" if old is None else f"record (was {top_m(old, res):.1f})"
        print(f"  ({cx:6.2f}, {cy:6.2f}) top {top_m(t, res):.2f} -> {a.top:.2f}  {how}")
print(f"{fills} columns for octomap_edits.json, {len(stacked)} for the stacked record"
      + ("" if a.write else "  (dry run: --write to record them)"))
if a.write:
    if fills:
        backup(os.path.join(a.map_dir, "octomap_edits.json"))
        record["fill_key"] = [[x, y, z] for (x, y), z in have.items()]
        save_record(a.map_dir, record)
    if stacked:
        backup(os.path.join(a.map_dir, "octomap_edits_stacked.json"))
        doc = load_stacked(a.map_dir)
        doc["fill_key"] += stacked
        save_stacked(a.map_dir, doc)
        print("stacked fills recorded: run apply_stacked.py, then apply_octomap_edits")
    elif fills:
        print("now: ros2 run syncai_mapping apply_octomap_edits", a.map_dir)
