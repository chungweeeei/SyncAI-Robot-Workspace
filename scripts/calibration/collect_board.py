#!/usr/bin/env python3
"""Live checkerboard collector for camera intrinsics. Runs in the robot container (needs rclpy).

Subscribes <robot_id>/image_raw/compressed -- syncai_camera_feeder's MJPEG, which must be the raw
passthrough (stabilize: false) -- and keeps each still, distinct board view: its JPEG bytes as they
arrived (no re-encode) and the sub-pixel corners found in it, as <out_dir>/vNNN_<sec>_<nsec>.jpg
/ .npy. fit_intrinsics.py reads that directory. Every second it rewrites <out_dir>/status.txt with
what the fit will be able to use, so the person moving the board can be told what is missing:

    watch -n1 cat <out_dir>/status.txt

Running it again on the same out_dir adds to the views already there.

usage: collect_board.py <out_dir> [--pattern 7x10] [--min-square 15] [--robot-id ID]
                        [--topic image_raw/compressed]
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

import calib_common as cc

# Fastest corner motion (px/s) a kept view may have. The feeder publishes every other frame of a
# 60 fps capture; above this the corners smear by a pixel within one exposure.
MAX_SPEED = 25.0
# Distance in descriptor space (centre x / y, size, skew, both tilts) to every kept view: a board
# held still is kept once, not thirty times a second.
MIN_DIST = 0.025
# What fit_intrinsics.py needs, as rough targets for the person collecting (robot01, 2026-10-20:
# 63 usable views with 15 tilted ones gave a split-half-stable fit). Guidance, not a gate.
TARGET_USABLE = 60
TARGET_TILTED_EACH = 3
TARGET_NEAR = 0.3  # board size (sqrt of its share of the image area) of a close view


def descriptor(c, pattern, shape):
    h, w = shape
    q = cc.board_quad(c, pattern)
    v1, v2 = q[1] - q[0], q[3] - q[0]
    skew = abs(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9))
    tx, ty = cc.board_tilt(c, pattern)
    return np.array([c[:, 0].mean() / w, c[:, 1].mean() / h, cc.board_size(c, pattern, shape), skew, tx, ty])


def status_text(views, pattern, shape, min_square, kept, seen, visible):
    usable = [c for c in views if cc.square_px(c, pattern) >= min_square]
    tilts = np.array([cc.board_tilt(c, pattern) for c in usable]).reshape(-1, 2)
    sizes = np.array([cc.board_size(c, pattern, shape) for c in usable])
    cov = cc.coverage(usable, shape)
    tilted = int(np.sum(np.abs(tilts).max(1) >= cc.TILT)) if len(usable) else 0

    def mark(ok):
        return "ok " if ok else "-- "

    counts = {
        "x+": int(np.sum(tilts[:, 0] >= cc.TILT)), "x-": int(np.sum(tilts[:, 0] <= -cc.TILT)),
        "y+": int(np.sum(tilts[:, 1] >= cc.TILT)), "y-": int(np.sum(tilts[:, 1] <= -cc.TILT)),
    }
    near = int(np.sum(sizes >= TARGET_NEAR))
    lines = [
        f"time {time.strftime('%H:%M:%S')}  kept {kept}  detections {seen}  "
        f"board visible {'yes' if visible else 'no'}",
        f"usable views (squares >= {min_square:g} px): {len(usable)}, {tilted} of them tilted",
        "",
        f"{mark(len(usable) >= TARGET_USABLE)}usable views {len(usable)} / {TARGET_USABLE}",
        f"{mark((cov > 0).all())}every cell of the grid below covered",
        f"{mark(min(counts.values()) >= TARGET_TILTED_EACH)}tilted >= {cc.TILT} each way, "
        f"{TARGET_TILTED_EACH}+ each: tilt_x +{counts['x+']} / -{counts['x-']}   "
        f"tilt_y +{counts['y+']} / -{counts['y-']}",
        f"{mark(near >= TARGET_TILTED_EACH)}close views (size >= {TARGET_NEAR}): {near}",
        "",
        "corner coverage of usable views (4x4 grid of the image, rows top -> bottom):",
    ]
    lines += [" ".join(f"{v:5d}" for v in row) for row in cov]
    if len(usable):
        lines.append(f"size {sizes.min():.2f}..{sizes.max():.2f}  tilt_x {tilts[:, 0].min():+.2f}..{tilts[:, 0].max():+.2f}"
                     f"  tilt_y {tilts[:, 1].min():+.2f}..{tilts[:, 1].max():+.2f}")
    far = len(views) - len(usable)
    if far:
        lines.append(f"({far} kept views are too far away to be used: bring the board closer)")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out_dir")
    ap.add_argument("--pattern", default="7x10", help="inner corners, per row x rows")
    ap.add_argument("--min-square", type=float, default=15.0,
                    help="the fit's squares-in-px cut, for the status only")
    ap.add_argument("--robot-id", help="default: [system] robot_id of config/system.ini")
    ap.add_argument("--topic", default="image_raw/compressed", help="relative to the robot namespace")
    a = ap.parse_args()
    pattern = cc.parse_pattern(a.pattern)
    robot_id = a.robot_id or cc.read_robot_id()
    os.makedirs(a.out_dir, exist_ok=True)

    shape = None
    views, descs = [], []
    for f in sorted(glob.glob(os.path.join(a.out_dir, "v*.npy"))):
        c = np.load(f).reshape(-1, 2)
        if shape is None:
            shape = cv2.imread(f[:-4] + ".jpg", cv2.IMREAD_GRAYSCALE).shape
        views.append(c)
        descs.append(descriptor(c, pattern, shape))
    if views:
        print(f"continuing: {len(views)} views already in {a.out_dir}")

    rclpy.init()
    node = rclpy.create_node("board_collector", namespace=robot_id)
    latest = [None]

    def on_image(m):
        latest[0] = m

    sub = node.create_subscription(CompressedImage, a.topic, on_image, qos_profile_sensor_data)
    print(f"subscribed to {sub.topic_name}; pattern {pattern[0]}x{pattern[1]} inner corners")

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
    prev, seen, last_det, last_status = None, 0, 0.0, 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            m, latest[0] = latest[0], None
            now = time.time()
            if m is not None:
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
                shape = img.shape
                ok, c = cv2.findChessboardCorners(img, pattern, flags)
                if ok:
                    seen += 1
                    last_det = now
                    c = cv2.cornerSubPix(img, c, (5, 5), (-1, -1), crit).reshape(-1, 2)
                    speed = np.inf if prev is None else np.abs(c - prev[1]).max() / max(t - prev[0], 1e-3)
                    prev = (t, c)
                    d = descriptor(c, pattern, shape)
                    far = not descs or np.min(np.linalg.norm(np.array(descs) - d, axis=1)) > MIN_DIST
                    if speed < MAX_SPEED and far:
                        name = f"v{len(views):03d}_{m.header.stamp.sec}_{m.header.stamp.nanosec:09d}"
                        with open(os.path.join(a.out_dir, name + ".jpg"), "wb") as f:
                            f.write(bytes(m.data))
                        np.save(os.path.join(a.out_dir, name + ".npy"), c)
                        views.append(c)
                        descs.append(d)
                        sq = cc.square_px(c, pattern)
                        print(f"kept {name[:4]}  squares {sq:4.1f} px{'' if sq >= a.min_square else ' (too far)'}"
                              f"  size {d[2]:.2f}  tilt_x {d[4]:+.2f}  tilt_y {d[5]:+.2f}", flush=True)
                else:
                    prev = None
            if shape is not None and now - last_status > 1.0:
                last_status = now
                text = status_text(views, pattern, shape, a.min_square, len(views), seen, now - last_det < 1.0)
                tmp = os.path.join(a.out_dir, "status.txt.tmp")
                with open(tmp, "w") as f:
                    f.write(text)
                os.replace(tmp, os.path.join(a.out_dir, "status.txt"))
    except KeyboardInterrupt:
        pass
    print(f"{len(views)} views in {a.out_dir}")


if __name__ == "__main__":
    main()
