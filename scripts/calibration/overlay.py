#!/usr/bin/env python3
"""Render a preprocessed bag's accumulated cloud onto its image with a calib.json pose.

  --mode depth  colours each covered pixel by the nearest point's range (0.5 m blue .. 8 m red)
                over the grey image: near objects show a translation error first.
  --mode side   a 64 px checkerboard of image tiles and lidar-intensity tiles: an edge that runs
                straight across the tile seams is aligned.

usage: overlay.py <preprocessed_dir> <bag> <key> <out.png> [--mode depth|side] [--crop x0,y0,x1,y1]
"""
import argparse

import cv2
import numpy as np

import calib_common as cc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("preprocessed_dir")
    ap.add_argument("bag")
    ap.add_argument("key", help="calib.json results key")
    ap.add_argument("out")
    ap.add_argument("--mode", default="side", choices=["depth", "side"])
    ap.add_argument("--crop", help="x0,y0,x1,y1 of the image to keep")
    a = ap.parse_args()

    cfg = cc.load_calib(a.preprocessed_dir)
    cam = cc.camera_from_calib(cfg)
    t, R = cc.pose_from_result(cfg["results"][a.key])
    pts, img = cc.load_bag(a.preprocessed_dir, a.bag)
    h, w = img.shape
    uv, m = cam.project((pts[:, :3] - t) @ R)
    depth, inten = ((pts[m, :3] - t) @ R)[:, 2], pts[m, 3]
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    uv, depth, inten = uv[ok].astype(int), depth[ok], inten[ok]
    # nearest point wins per pixel, so far walls do not paint over near edges
    order = np.argsort(-depth)
    uv, depth, inten = uv[order], depth[order], inten[order]
    dimg = np.zeros((h, w), np.float32)
    dimg[uv[:, 1], uv[:, 0]] = depth
    valid = dimg > 0
    base = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if a.mode == "depth":
        scaled = 255 * (np.log(np.maximum(dimg, 0.3)) - np.log(0.5)) / (np.log(8) - np.log(0.5))
        col = cv2.applyColorMap(np.clip(scaled, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        out = base.copy()
        out[valid] = (0.35 * base[valid] + 0.65 * col[valid]).astype(np.uint8)
    else:
        iimg = np.zeros((h, w), np.float32)
        iimg[uv[:, 1], uv[:, 0]] = inten
        iimg = cv2.dilate(iimg, np.ones((3, 3), np.uint8))
        v2 = cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        xx, yy = np.meshgrid(np.arange(w), np.arange(h))
        checker = (((xx // 64) + (yy // 64)) % 2 == 1) & v2
        g = img.copy()
        g[checker] = (255 * np.clip(iimg, 0, 1)).astype(np.uint8)[checker]
        out = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if a.crop:
        x0, y0, x1, y1 = (int(v) for v in a.crop.split(","))
        out = out[y0:y1, x0:x1]
    cv2.imwrite(a.out, out)
    print(f"{a.key}: {valid.sum()} pixels covered -> {a.out}")


if __name__ == "__main__":
    main()
