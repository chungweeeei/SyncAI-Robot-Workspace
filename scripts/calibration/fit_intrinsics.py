#!/usr/bin/env python3
"""Fit camera intrinsics to the views collect_board.py saved, and write the camera_info YAML.

The output is what syncai_camera_feeder publishes as camera_info (camera_calibration's ost.yaml
layout). Three choices are fixed here because each one cost a wrong answer on robot01
(2026-10-20; docs/lidar-camera-calibration.md has the numbers):

  * Views whose squares are under --min-square px are dropped. cornerSubPix's 11x11 window then
    spans more than one square and biases the corners; the far views pulled fx from ~713 to 744.
  * No outlier rejection. The views with the largest residuals were the tilted ones -- the only
    views that pin the focal length -- and rejecting them at 3x the median locked in fx 818.
  * equidistant by default. plumb_bob fits this lens as well inside the image but folds back
    57 deg off axis, so lidar points beyond it project as ghosts in the middle of the image.

--split-half fits the even and the odd views on their own: OpenCV's plumb_bob fit can stop in a
local minimum (one half of robot01's views converged to fx 818, which the full solution explained
better than the half's own fit did), and two halves that disagree for real mean too few tilted
views.

usage: fit_intrinsics.py <collect_dir> --out camera0.yaml [--model equidistant|plumb_bob]
                         [--pattern 7x10] [--square 0.021] [--min-square 15] [--split-half]
"""
import argparse
import datetime
import os

import numpy as np

import calib_common as cc


def describe(views, pattern, shape):
    tilts = np.array([cc.board_tilt(c, pattern) for c in views])
    tilted = np.abs(tilts).max(1) >= cc.TILT
    print(f"  tilted (>= {cc.TILT}): {tilted.sum()}  [tilt_x +{np.sum(tilts[:, 0] >= cc.TILT)} "
          f"-{np.sum(tilts[:, 0] <= -cc.TILT)}, tilt_y +{np.sum(tilts[:, 1] >= cc.TILT)} "
          f"-{np.sum(tilts[:, 1] <= -cc.TILT)}]")
    cov = cc.coverage(views, shape)
    print("  corner coverage, 4x4 grid of the image:")
    for row in cov:
        print("    " + " ".join(f"{v:5d}" for v in row))
    if not (cov > 0).all():
        print("  WARNING: part of the image has no corners; the distortion there is extrapolated")
    return tilted


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("collect_dir")
    ap.add_argument("--out", required=True, help="camera_info YAML to write")
    ap.add_argument("--model", default="equidistant", choices=["equidistant", "plumb_bob"])
    ap.add_argument("--pattern", default="7x10", help="inner corners, per row x rows")
    ap.add_argument("--square", type=float, default=0.021, help="square size in metres (measure the print)")
    ap.add_argument("--min-square", type=float, default=15.0, help="drop views with smaller squares (px)")
    ap.add_argument("--split-half", action="store_true")
    ap.add_argument("--camera-name", default="camera0")
    a = ap.parse_args()

    pattern = cc.parse_pattern(a.pattern)
    names, views, shape = cc.load_views(a.collect_dir, pattern)
    keep = [cc.square_px(c, pattern) >= a.min_square for c in views]
    views = [c for c, k in zip(views, keep) if k]
    size = (shape[1], shape[0])
    print(f"{len(keep)} views in {a.collect_dir}, {len(views)} with squares >= {a.min_square:g} px")
    if len(views) < 10:
        raise SystemExit("too few usable views")
    tilted = describe(views, pattern, shape)
    obj = cc.board_points(pattern, a.square)

    print(f"fitting {a.model} on {len(views)} views (plumb_bob takes minutes on the Orin) ...", flush=True)
    cam, rms, errs = cc.fit_camera(a.model, obj, views, size)
    K, D = cam.K, cam.D
    print(f"RMS {rms:.3f} px; per view median {np.median(errs):.3f}, max {errs.max():.2f}; "
          f"square-on views {np.median(errs[~tilted]):.2f}, tilted {np.median(errs[tilted]) if tilted.any() else np.nan:.2f}")
    print(f"fx {K[0, 0]:.2f}  fy {K[1, 1]:.2f}  cx {K[0, 2]:.2f}  cy {K[1, 2]:.2f}")
    print(f"D  {np.array2string(D, precision=6)}")
    h, v, dg = cam.fov_deg()
    print(f"FOV h {h:.1f}  v {v:.1f}  diag {dg:.1f} deg; model monotonic to {np.degrees(cam.max_angle):.1f} deg off axis")
    lost = cam.unreachable_fraction()
    if lost > 0:
        print(f"WARNING: {lost:.1%} of the image (its corners) has no ray in this model: it folds back "
              f"{np.degrees(cam.max_angle):.0f} deg off axis. Prefer --model equidistant.")
    worst = np.argsort(-errs)[:5]
    print("worst views: " + ", ".join(f"{e:.2f}" for e in errs[worst]))

    if a.split_half:
        print("split-half check ...", flush=True)
        for half, sel in (("even", views[0::2]), ("odd", views[1::2])):
            hc, hrms, _ = cc.fit_camera(a.model, obj, sel, size)
            full_on_half = np.sqrt(np.mean(cam.pose_only_errors(obj, sel) ** 2))
            dev = hc.K[0, 0] / K[0, 0] - 1
            if abs(dev) < 0.015:
                verdict = "agrees"
            elif full_on_half < hrms:
                verdict = "local minimum in the half's fit; the full fit explains these views better"
            else:
                verdict = "DISAGREES: the views do not pin fx down; collect more close, tilted views"
            print(f"  {half:4s} {len(sel)} views: fx {hc.K[0, 0]:.1f} ({dev:+.1%}), own RMS {hrms:.3f}, "
                  f"full fit on them {full_on_half:.3f} -> {verdict}")

    comments = [
        f"{a.camera_name} intrinsics, {datetime.date.today()}, scripts/calibration/fit_intrinsics.py",
        f"{len(views)} views of {os.path.basename(os.path.normpath(a.collect_dir))} (squares >= {a.min_square:g} px, "
        f"{pattern[0]}x{pattern[1]} corners, {a.square * 1000:g} mm), {a.model}, RMS {rms:.2f} px.",
        "See docs/lidar-camera-calibration.md.",
    ]
    cc.write_camera_yaml(a.out, cam, a.camera_name, comments)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
