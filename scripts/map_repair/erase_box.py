#!/usr/bin/env python3
"""Clear what stands on the floor in a box of columns: a door that was shut during one pass of
the mapping, a person, a cart.

    erase_box.py map/dp --box -0.55 -0.55 -16.95 -15.95 --near 5.3 --zhi 7.6
    erase_box.py map/dp --box ... --near 5.3 --zhi 7.6 --write

--box takes cell centres (both ends included). In each column the voxels from just over the floor
up to --zhi are cleared. The floor is the lower of the two side neighbours' tops within 0.3 m of
--near (the floor's height, roughly: floors stack on a column in a stairwell, so the search needs
it), read across the box's thinner direction -- a door frame beside the leaf is no floor -- unless
--floor gives it exactly (m, a voxel top). Cleared voxels are written FREE (observed empty), not
unknown: the planner treats unknown space over a floor as not passable.

apply_octomap_edits cannot clear anything, so with --write the keys go to the stacked record's
free_key for apply_stacked.py, which writes them into octomap.raw.bt (backed up first); then run
apply_octomap_edits so octomap.bt follows. Look at the box with inspect_map.py --count before and
after: a door is a one-voxel-thick column of 15-25 voxels between its frame's two thicker ones.
"""
import argparse
import os

from octomap_bt import (Columns, backup, centre_m, key, load_stacked, raw_tree, save_stacked,
                        top_m)

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("map_dir")
ap.add_argument("--box", nargs=4, type=float, metavar=("X0", "X1", "Y0", "Y1"), required=True)
ap.add_argument("--zhi", type=float, required=True, help="clear up to here (m)")
ap.add_argument("--near", type=float, default=None, help="the floor's height, roughly (m)")
ap.add_argument("--floor", type=float, default=None, help="the floor's top exactly (m)")
ap.add_argument("--write", action="store_true")
a = ap.parse_args()
if a.near is None and a.floor is None:
    ap.error("give --near (the floor's rough height) or --floor")

x0, x1, y0, y1 = a.box
cols = Columns.from_file(raw_tree(a.map_dir), (x0 - 0.2, x1 + 0.2, y0 - 0.2, y1 + 0.2))
res = cols.res
across_x = (x1 - x0) <= (y1 - y0)  # the box's thin direction: the sides to read the floor from
keys = []
for ix in range(key(x0, res), key(x1, res) + 1):
    for iy in range(key(y0, res), key(y1, res) + 1):
        if a.floor is not None:
            floor = key(a.floor - res / 2, res)
        else:
            sides = [(ix - 1, iy), (ix + 1, iy)] if across_x else [(ix, iy - 1), (ix, iy + 1)]
            tops = [cols.top(sx, sy, a.near - 0.3, a.near + 0.3) for sx, sy in sides]
            tops = [t for t in tops if t is not None]
            if not tops:
                continue
            floor = min(tops)
        door = cols.between(ix, iy, floor + 1, key(a.zhi, res))
        keys += [[ix, iy, k] for k in door]
        if door:
            print(f"  ({centre_m(ix, res):6.2f}, {centre_m(iy, res):6.2f}): floor {top_m(floor, res):.2f}, "
                  f"clear {len(door)} voxels {top_m(door[0], res) - res:.2f}..{top_m(door[-1], res):.2f}")
print(f"{len(keys)} voxels to clear" + ("" if a.write else "  (dry run: --write to record them)"))
if a.write and keys:
    backup(os.path.join(a.map_dir, "octomap_edits_stacked.json"))
    doc = load_stacked(a.map_dir)
    doc["free_key"] += keys
    save_stacked(a.map_dir, doc)
    print("recorded: run apply_stacked.py, then apply_octomap_edits")
