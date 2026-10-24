#!/usr/bin/env python3
"""Find pits in a saved map's stair flights -- treads the lidar never saw -- and fill them.

    find_pits.py map/dp                  # list them (octomap.bt as it is now)
    find_pits.py map/dp --write          # + fill: octomap_edits.json, or the stacked record

The flights are found from the mapping track: runs of keyframes whose ground climbs (or drops)
more than 0.15 m over +-2 keyframes and more than 0.5 m in all. Each flight is assumed to run
along x or along y, whichever its keyframes spread more along. A flight's box is its keyframes'
extent plus --margin across and 0.3 m along, ground +-0.3 m.

Per row across the flight (one voxel along it) the tread is the median column top. A column is
a pit when its top is at least --min under that median, at most --max under it (deeper is the
well between two flights, seen through), at least 0.1 m under both its along-flight neighbours
(a riser row's lower half is the tread before it, not a pit) and within --margin of one of the
flight's keyframes (not the far side of a wall). Pits are filled to the row's median, in up to
three passes: filling one row of a two-row trench exposes the other.

With --write a pit goes into octomap_edits.json (one key per column, apply_octomap_edits' rule)
unless that column already holds another floor's key; then it goes into the stacked record, for
apply_stacked.py. Both files are backed up first. Run apply_stacked.py (if it says so) and then
apply_octomap_edits afterwards; docs/octomap-repair.md has the order.
"""
import argparse
import os

import numpy as np
from octomap_bt import (LIDAR_HEIGHT, Columns, backup, centre_m, key, load_record, load_stacked,
                        read_track, save_record, save_stacked, top_m)

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("map_dir")
ap.add_argument("--min", type=float, default=0.15, help="shallowest pit, m (default 0.15)")
ap.add_argument("--max", type=float, default=0.5, help="deepest pit, m (default 0.5)")
ap.add_argument("--margin", type=float, default=0.65, help="m across the flight from its track")
ap.add_argument("--write", action="store_true", help="fill them (records backed up first)")
a = ap.parse_args()

track = read_track(a.map_dir)
G = track[:, 2] - LIDAR_HEIGHT
n = len(G)
slope = np.array([abs(G[min(k + 2, n - 1)] - G[max(k - 2, 0)]) > 0.15 for k in range(n)])
runs, cur = [], []
for k in range(n):
    if slope[k]:
        cur.append(k)
    elif cur:
        runs.append(cur)
        cur = []
if cur:
    runs.append(cur)
runs = [r for r in runs if np.ptp(G[r]) > 0.5]
if not runs:
    raise SystemExit("no flight on the mapping track")

cols = Columns.from_file(os.path.join(a.map_dir, "octomap.bt"))
res = cols.res
record = load_record(a.map_dir)
have = {(k[0], k[1]): k[2] for k in record["fill_key"]}
fills, stacked = [], []
for i, r in enumerate(runs, 1):
    xy = track[r, :2]
    along_x = np.ptp(xy[:, 0]) > np.ptp(xy[:, 1])
    lo = xy.min(axis=0) - np.where([along_x, not along_x], 0.3, a.margin)
    hi = xy.max(axis=0) + np.where([along_x, not along_x], 0.3, a.margin)
    zlo, zhi = G[r].min() - 0.3, G[r].max() + 0.3
    print(f"flight {i}: keyframes {r[0]}-{r[-1]}, ground {G[r[0]]:.2f} -> {G[r[-1]]:.2f}, "
          f"along {'x' if along_x else 'y'}, box x {lo[0]:.1f}..{hi[0]:.1f} y {lo[1]:.1f}..{hi[1]:.1f}")
    top = {}
    for ix in range(key(lo[0], res), key(hi[0], res) + 1):
        for iy in range(key(lo[1], res), key(hi[1], res) + 1):
            t = cols.top(ix, iy, zlo, zhi)
            if t is not None:
                top[(ix, iy)] = t
    rows = range(key(lo[0], res), key(hi[0], res) + 1) if along_x else range(key(lo[1], res), key(hi[1], res) + 1)
    across = range(key(lo[1], res), key(hi[1], res) + 1) if along_x else range(key(lo[0], res), key(hi[0], res) + 1)
    cell = (lambda r_, c_: (r_, c_)) if along_x else (lambda r_, c_: (c_, r_))
    for npass in range(3):
        found = 0
        for rw in rows:
            row = [(cell(rw, c), top[cell(rw, c)]) for c in across if cell(rw, c) in top]
            if len(row) < 4:
                continue
            med = int(np.median([t for _, t in row]))
            for c, t in row:
                cx, cy = centre_m(c[0], res), centre_m(c[1], res)
                if np.min(np.hypot(xy[:, 0] - cx, xy[:, 1] - cy)) > a.margin:
                    continue
                depth = top_m(med, res) - top_m(t, res)
                if depth < a.min - 1e-6 or depth > a.max + 1e-6:
                    continue
                nb = [top.get(cell(rw - 1, c[1] if along_x else c[0])),
                      top.get(cell(rw + 1, c[1] if along_x else c[0]))]
                if any(b is None or top_m(b, res) - top_m(t, res) < 0.1 - 1e-6 for b in nb):
                    continue
                old = have.get(c)
                where = "record"
                if old is not None and abs(top_m(old, res) - top_m(med, res)) > 0.5:
                    where = "stacked (column holds another floor's key)"
                    stacked.append([c[0], c[1], med])
                else:
                    fills.append((c, med))
                    have[c] = med
                print(f"   pit ({cx:6.2f}, {cy:6.2f}): top {top_m(t, res):.2f}, tread "
                      f"{top_m(med, res):.2f}, depth {depth:.2f} -> {where}")
                top[c] = med
                found += 1
        if not found:
            break
print(f"{len(fills)} pits for octomap_edits.json, {len(stacked)} for the stacked record")
if a.write and (fills or stacked):
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
    else:
        print("now: ros2 run syncai_mapping apply_octomap_edits", a.map_dir)
