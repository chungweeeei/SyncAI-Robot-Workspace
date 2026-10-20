#!/usr/bin/env python3
"""Compare camera_info YAMLs on the same collect_board.py views.

For each YAML: K, D, the field of view it implies, and how well it explains the board views with
K and D held fixed (each board pose fitted alone), over all views and split into views near the
middle of the image and near its edge, where distortion shows. Then how far apart the first YAML
and each other one put the same ray, in pixels.

A YAML may be suffixed with @<zoom> to divide its K about the image centre: a calibration made on
the feeder's stabilized output (stabilize: true, zoom 1.3) mapped back to the raw stream.

usage: compare_intrinsics.py <collect_dir> <a.yaml> <b.yaml> [...] [--pattern 7x10] [--square 0.021]
                             [--min-square 15]
"""
import argparse

import numpy as np

import calib_common as cc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("collect_dir")
    ap.add_argument("yamls", nargs="+")
    ap.add_argument("--pattern", default="7x10")
    ap.add_argument("--square", type=float, default=0.021)
    ap.add_argument("--min-square", type=float, default=15.0)
    a = ap.parse_args()

    pattern = cc.parse_pattern(a.pattern)
    _, views, shape = cc.load_views(a.collect_dir, pattern)
    views = [c for c in views if cc.square_px(c, pattern) >= a.min_square]
    obj = cc.board_points(pattern, a.square)
    h, w = shape
    centre = np.array([w / 2, h / 2])
    edge = np.array([np.max(np.abs(c.mean(0) - centre) / centre) > 0.5 for c in views])
    print(f"{len(views)} views ({np.count_nonzero(~edge)} centre, {np.count_nonzero(edge)} edge)\n")

    cams = [cc.load_camera_yaml(s) for s in a.yamls]
    for cam, label in cams:
        errs = cam.pose_only_errors(obj, views)

        def rms(m):
            return np.sqrt(np.mean(errs[m] ** 2)) if m.any() else float("nan")

        K = cam.K
        fh, fv, fd = cam.fov_deg()
        print(f"== {label}  ({cam.model})")
        print(f"   fx {K[0, 0]:8.2f}  fy {K[1, 1]:8.2f}  cx {K[0, 2]:7.2f}  cy {K[1, 2]:7.2f}")
        print(f"   D  {np.array2string(cam.D, precision=4, suppress_small=True)}")
        print(f"   FOV h {fh:5.1f}  v {fv:5.1f}  diag {fd:5.1f} deg (nan: that edge has no ray); "
              f"monotonic to {np.degrees(cam.max_angle):.1f} deg off axis")
        print(f"   board RMS with K, D fixed: all {rms(np.ones_like(edge)):.3f} px  centre {rms(~edge):.3f}  "
              f"edge {rms(edge):.3f}  worst view {errs.max():.2f}\n")

    gx, gy = np.meshgrid(np.linspace(0, w, 33), np.linspace(0, h, 19))
    px = np.stack([gx.ravel(), gy.ravel()], 1)
    ref, ref_label = cams[0]
    n = ref.unproject(px)
    ok = ~np.isnan(n).any(1)
    rays = np.hstack([n[ok], np.ones((ok.sum(), 1))])
    r = np.linalg.norm((px[ok] - centre) / centre, axis=1)
    for cam, label in cams[1:]:
        uv, m = cam.project(rays, min_z=0.0)
        diff = np.full(len(rays), np.nan)
        diff[m] = np.linalg.norm(uv - px[ok][m], axis=1)
        parts = [(r < 0.3, "centre"), ((r >= 0.3) & (r < 0.8), "mid"), (r >= 0.8, "edges/corners")]
        print(f"same ray, {ref_label} -> {label}: " + ", ".join(
            f"{name} {np.nanmean(diff[sel]):.1f} px" for sel, name in parts) + f" (max {np.nanmax(diff):.1f})")


if __name__ == "__main__":
    main()
