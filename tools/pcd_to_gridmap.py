#!/usr/bin/env python3
"""Convert a 3D point-cloud map (.pcd) into a 2D occupancy grid (pgm + yaml)
for the nav2 map server.

The grid is expressed in the SAME frame as the pcd (the SLAM `map` frame),
so localization TF (map -> odom) lines up with the produced map without any
extra alignment: origin in map.yaml is simply the grid's lower-left corner
in pcd coordinates.

Cell classification:
  occupied : >= --min-points points inside the obstacle z-band [--zmin, --zmax]
  free     : an "observed" cell that is not occupied, where observed depends on
             --free-mode:
               floor : cell has points in the floor band [--floor-zmin, --floor-zmax]
               any   : cell has points at ANY z (useful when the lidar cannot see
                       the floor, e.g. MID360 -7..+52 deg FOV: the dense ceiling
                       above walkable space marks the column as observed)
               none  : nothing is marked free (everything not occupied is unknown)
  unknown  : everything else

Typical use:
  # 1. inspect the z distribution to pick slice bands
  python3 tools/pcd_to_gridmap.py map.pcd --stats

  # 2. convert (bands are in map-frame z, remember LIO z=0 is at the lidar
  #    mount height of the mapping start pose, so the floor is negative)
  python3 tools/pcd_to_gridmap.py map.pcd -o maps/warehouse \
      --zmin -0.3 --zmax 1.5 --floor-zmin -0.7 --floor-zmax -0.4
"""

import argparse
import os
import re
import sys

import numpy as np


def read_pcd(path):
    """Minimal PCD v0.7 reader (ascii / binary), returns Nx3 float32 xyz."""
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("#") or not line:
                continue
            key, _, val = line.partition(" ")
            header[key] = val
            if key == "DATA":
                break
        fields = header["FIELDS"].split()
        sizes = list(map(int, header["SIZE"].split()))
        types = header["TYPE"].split()
        counts = list(map(int, header.get("COUNT", " ".join(["1"] * len(fields))).split()))
        points = int(header["POINTS"])
        data_mode = header["DATA"]

        np_types = {("F", 4): "f4", ("F", 8): "f8",
                    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4",
                    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
        dtype = []
        for name, t, s, c in zip(fields, types, sizes, counts):
            base = np_types[(t, s)]
            dtype.append((name, base) if c == 1 else (name, base, (c,)))
        dtype = np.dtype(dtype)

        if data_mode == "binary":
            raw = np.frombuffer(f.read(points * dtype.itemsize), dtype=dtype, count=points)
        elif data_mode == "ascii":
            raw = np.loadtxt(f, dtype=dtype, max_rows=points)
        else:
            sys.exit(f"unsupported DATA mode: {data_mode} (only ascii/binary)")

    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def print_stats(xyz):
    z = xyz[:, 2]
    print(f"points : {len(xyz)}")
    print(f"x range: [{xyz[:,0].min():8.2f}, {xyz[:,0].max():8.2f}] m")
    print(f"y range: [{xyz[:,1].min():8.2f}, {xyz[:,1].max():8.2f}] m")
    print(f"z range: [{z.min():8.2f}, {z.max():8.2f}] m")
    print("\nz histogram (0.2 m bins) — the tallest low bin is usually the floor:")
    lo = np.floor(z.min() * 5) / 5
    hi = np.ceil(z.max() * 5) / 5
    bins = np.arange(lo, hi + 0.2, 0.2)
    hist, edges = np.histogram(z, bins=bins)
    peak = hist.max()
    for h, e in zip(hist, edges):
        bar = "#" * int(60 * h / peak)
        print(f"  z {e:6.2f} .. {e+0.2:6.2f} | {h:8d} {bar}")


def despeckle(grid, min_obstacle_size):
    """Remove occupied blobs smaller than min_obstacle_size cells (sensor noise,
    dynamic-object残影). Real walls/racks form large connected components and
    are untouched. Removed cells become free (they sit in observed space)."""
    from scipy import ndimage
    occ = grid == 0
    labels, n = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    small = sizes < min_obstacle_size
    small[0] = False
    removed = small[labels]
    grid = grid.copy()
    grid[removed] = 254
    print(f"despeckle: removed {int(removed.sum())} cells "
          f"({int(small.sum())} blobs < {min_obstacle_size} cells)")
    return grid


def fill_holes(grid, max_hole_size):
    """Turn small unknown pockets fully enclosed by free space into free
    (lidar ring-gap arcs). Unknown regions touching the border or bounded by
    obstacles (e.g. inside racks) are preserved."""
    from scipy import ndimage
    unk = grid == 205
    labels, n = ndimage.label(unk, structure=np.ones((3, 3)))
    grid = grid.copy()
    border_labels = set(np.unique(np.concatenate([
        labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]]))) - {0}
    filled_cells = 0
    filled_blobs = 0
    for lab in range(1, n + 1):
        if lab in border_labels:
            continue
        mask = labels == lab
        size = int(mask.sum())
        if size > max_hole_size:
            continue
        ring = ndimage.binary_dilation(mask, np.ones((3, 3))) & ~mask
        neigh = grid[ring]
        # fill only if the pocket is (almost) entirely surrounded by free
        if (neigh == 0).sum() <= 0.05 * len(neigh):
            grid[mask] = 254
            filled_cells += size
            filled_blobs += 1
    print(f"fill-holes: filled {filled_cells} cells ({filled_blobs} pockets <= {max_hole_size} cells)")
    return grid


def convert(xyz, args):
    res = args.resolution
    obst = xyz[(xyz[:, 2] >= args.zmin) & (xyz[:, 2] <= args.zmax)]
    if args.free_mode == "floor":
        observed = xyz[(xyz[:, 2] >= args.floor_zmin) & (xyz[:, 2] <= args.floor_zmax)]
    elif args.free_mode == "any":
        observed = xyz
    else:  # none
        observed = xyz[:0]
    if len(obst) == 0:
        sys.exit("no points in obstacle z-band — check --zmin/--zmax (use --stats)")
    print(f"obstacle-band points: {len(obst)}, observed points ({args.free_mode}): {len(observed)}")

    # grid bounds from the union of both slices, small padding
    used = np.vstack([obst[:, :2], observed[:, :2]]) if len(observed) else obst[:, :2]
    min_xy = used.min(axis=0) - res
    max_xy = used.max(axis=0) + res
    width = int(np.ceil((max_xy[0] - min_xy[0]) / res))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / res))
    if width * height > 200_000_000:
        sys.exit(f"grid {width}x{height} too large — wrong bands or resolution?")
    print(f"grid: {width} x {height} @ {res} m/px, origin=({min_xy[0]:.3f}, {min_xy[1]:.3f})")

    def bincount2d(pts):
        ix = ((pts[:, 0] - min_xy[0]) / res).astype(np.int64).clip(0, width - 1)
        iy = ((pts[:, 1] - min_xy[1]) / res).astype(np.int64).clip(0, height - 1)
        return np.bincount(iy * width + ix, minlength=width * height).reshape(height, width)

    obst_cnt = bincount2d(obst)
    obs_cnt = bincount2d(observed[:, :2]) if len(observed) else np.zeros((height, width), np.int64)

    # nav2 pgm convention: 0=occupied(black), 254=free(white), 205=unknown(gray)
    grid = np.full((height, width), 205, dtype=np.uint8)
    free_mask = obs_cnt >= args.min_floor_points
    if args.free_close > 0:
        # lidar samples the floor as sparse rings, so per-cell floor hits are
        # speckled. A morphological closing (dilate then erode by the same disk)
        # bridges those sub-radius gaps into a solid drivable area WITHOUT growing
        # the outer boundary — so unknown outside the walls stays unknown.
        from scipy import ndimage
        r = args.free_close
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        se = (xx * xx + yy * yy) <= r * r
        closed = ndimage.binary_closing(free_mask, structure=se)
        print(f"free-close: r={r} cells, added {int(closed.sum() - free_mask.sum())} free cells")
        free_mask = closed
    grid[free_mask] = 254
    grid[obst_cnt >= args.min_points] = 0

    if args.despeckle:
        grid = despeckle(grid, args.min_obstacle_size)
    if args.fill_holes:
        grid = fill_holes(grid, args.max_hole_size)

    occ = int((grid == 0).sum())
    fre = int((grid == 254).sum())
    print(f"cells: occupied={occ}, free={fre}, unknown={width*height - occ - fre}")

    # row 0 of a pgm is the TOP of the map (max y) -> flip
    img = np.flipud(grid)

    out = args.output
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    pgm_path = out + ".pgm"
    yaml_path = out + ".yaml"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(img.tobytes())
    with open(yaml_path, "w") as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{min_xy[0]:.6f}, {min_xy[1]:.6f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
    print(f"wrote {pgm_path}")
    print(f"wrote {yaml_path}")

    if args.preview:
        try:
            from PIL import Image
            png_path = out + "_preview.png"
            Image.fromarray(img).convert("L").save(png_path)
            print(f"wrote {png_path}")
        except ImportError:
            print("PIL not available, skipping preview png")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pcd", help="input .pcd (map frame)")
    p.add_argument("--stats", action="store_true", help="print z histogram and exit")
    p.add_argument("-o", "--output", default="gridmap", help="output basename (writes .pgm/.yaml)")
    p.add_argument("--resolution", type=float, default=0.05, help="m per pixel (default 0.05)")
    p.add_argument("--zmin", type=float, default=0.0, help="obstacle band lower z (map frame)")
    p.add_argument("--zmax", type=float, default=1.5, help="obstacle band upper z (map frame)")
    p.add_argument("--free-mode", choices=["floor", "any", "none"], default="any",
                   help="how to mark free cells: floor=needs floor-band points, "
                        "any=any point at any z marks the column observed (default), "
                        "none=only occupied/unknown")
    p.add_argument("--floor-zmin", type=float, default=None, help="floor band lower z (free-mode floor)")
    p.add_argument("--floor-zmax", type=float, default=None, help="floor band upper z (free-mode floor)")
    p.add_argument("--min-points", type=int, default=2, help="points/cell to mark occupied (default 2)")
    p.add_argument("--min-floor-points", type=int, default=1, help="observed points/cell to mark free (default 1)")
    p.add_argument("--free-close", type=int, default=0,
                   help="morphological-close the free mask with a disk of this radius (cells) "
                        "to turn speckled floor sampling into solid free space (needs scipy, 0=off)")
    p.add_argument("--despeckle", action="store_true",
                   help="remove occupied blobs smaller than --min-obstacle-size (needs scipy)")
    p.add_argument("--min-obstacle-size", type=int, default=4,
                   help="despeckle: keep occupied blobs >= this many cells (default 4)")
    p.add_argument("--fill-holes", action="store_true",
                   help="fill unknown pockets enclosed by free space (needs scipy)")
    p.add_argument("--max-hole-size", type=int, default=5000,
                   help="fill-holes: only fill pockets <= this many cells (default 5000)")
    p.add_argument("--preview", action="store_true", help="also write a png preview")
    args = p.parse_args()

    xyz = read_pcd(args.pcd)
    if args.stats:
        print_stats(xyz)
        return
    if args.free_mode == "floor" and (args.floor_zmin is None or args.floor_zmax is None):
        sys.exit("free-mode floor needs --floor-zmin/--floor-zmax (run --stats first)")
    convert(xyz, args)


if __name__ == "__main__":
    main()
