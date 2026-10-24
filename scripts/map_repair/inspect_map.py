#!/usr/bin/env python3
"""Look at a patch of a saved map's octomap from above, with the mapping track on it.

    inspect_map.py map/dp --box -11.3 -8.2 -13.9 -11.8 --band 1.9 3.2
    inspect_map.py map/dp --box ... --band 5.45 7.6 --count     # walls, doors, posts
    inspect_map.py map/dp --box ... --band ... --track          # + keyframes there, with ground
    inspect_map.py map/dp --box ... --band ... --tree raw       # octomap.raw.bt instead

Default view: per 0.1 m column, the top (decimetres) of the highest occupied voxel whose centre
is inside the band; `.` none. --count: how many occupied voxels the column has in the band (a
door or a wall is a column of 15-25, a floor 1). `*` marks a column under a keyframe whose
ground (z - 0.481) is within the band, 0.3 m of slack below. Rows print north (+y) at the top;
labels are the cell's lower edge. docs/octomap-repair.md has what to look for.
"""
import argparse
import os

import numpy as np
from octomap_bt import LIDAR_HEIGHT, Columns, key, raw_tree, read_track, top_m

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("map_dir")
ap.add_argument("--box", nargs=4, type=float, metavar=("X0", "X1", "Y0", "Y1"), required=True)
ap.add_argument("--band", nargs=2, type=float, metavar=("ZLO", "ZHI"), required=True)
ap.add_argument("--count", action="store_true", help="voxel count per column instead of the top")
ap.add_argument("--track", action="store_true", help="also list the keyframes in the box")
ap.add_argument("--tree", choices=("bt", "raw"), default="bt",
                help="octomap.bt (what the planner loads, default) or the raw tree")
a = ap.parse_args()

path = raw_tree(a.map_dir) if a.tree == "raw" else os.path.join(a.map_dir, "octomap.bt")
x0, x1, y0, y1 = a.box
zlo, zhi = a.band
cols = Columns.from_file(path, (x0, x1, y0, y1))
res = cols.res
track = read_track(a.map_dir)
ground = track[:, 2] - LIDAR_HEIGHT
inside = ((track[:, 0] >= x0) & (track[:, 0] < x1) & (track[:, 1] >= y0) & (track[:, 1] < y1)
          & (ground >= zlo - 0.3) & (ground < zhi))
marks = {(key(x, res), key(y, res)) for x, y in track[inside, :2]}
ix0, ix1, iy0, iy1 = key(x0, res), key(x1, res), key(y0, res), key(y1, res)
print(f"{path}: {'voxels in' if a.count else 'top (dm) in'} band {zlo}..{zhi} m; * = mapping track")
print("       " + "".join(f"{(ix - 32768) * res:5.1f}" for ix in range(ix0, ix1)))
for iy in range(iy1 - 1, iy0 - 1, -1):
    row = f"{(iy - 32768) * res:6.1f} "
    for ix in range(ix0, ix1):
        if a.count:
            n = len(cols.between(ix, iy, key(zlo, res), key(zhi, res)))
            s = str(n) if n else "."
        else:
            t = cols.top(ix, iy, zlo, zhi)
            s = "." if t is None else f"{top_m(t, res) * 10:.0f}"
        row += f"{s:>4}" + ("*" if (ix, iy) in marks else " ")
    print(row)
if a.track:
    print("keyframes in the box (index, x, y, ground = z - 0.481):")
    for k in np.nonzero(inside)[0]:
        print(f"  {k:5d} ({track[k, 0]:7.2f}, {track[k, 1]:7.2f})  {ground[k]:6.2f}")
