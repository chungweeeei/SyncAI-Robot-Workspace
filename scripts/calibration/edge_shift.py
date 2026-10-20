#!/usr/bin/env python3
"""Residual misalignment of lidar-camera extrinsics, per image region, in pixels.

Renders the accumulated cloud's intensity into the camera, then for each region finds the
(dx, dy) shift of that render which best correlates its gradient magnitude with the image's. A
good extrinsic gives ~0 everywhere; a shift common to every region is a pan / tilt error, one that
changes across the image is roll (or translation, for near objects). It is a check independent of
the NID cost the fit minimised -- most useful run on a bag the pose was NOT fitted on.

A region is reported as unreliable when its correlation is under 0.3 or the best shift sits on
the search limit (textureless walls and ceiling do that); the summary line is the RMS over the
reliable ones.

usage: edge_shift.py <preprocessed_dir> <bag> <key> [<key> ...] [--rows 0,200,420,600]
                     [--cols 0,430,860,1280]
"""
import argparse

import cv2
import numpy as np

import calib_common as cc

S = 40  # search half-width, px


def render(pts, cam, t, R, shape):
    h, w = shape
    uv, m = cam.project((pts[:, :3] - t) @ R)
    z, inten = ((pts[m, :3] - t) @ R)[:, 2], pts[m, 3]
    uv = np.round(uv).astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    uv, z, inten = uv[ok], z[ok], inten[ok]
    order = np.argsort(-z)  # nearest written last, so it wins
    L = np.full((h, w), np.nan, np.float32)
    L[uv[order, 1], uv[order, 0]] = inten[order]
    valid = ~np.isnan(L)
    # fill the holes between the non-repetitive scan's points from their neighbours
    acc = cv2.blur(np.where(valid, L, 0).astype(np.float32), (5, 5))
    cnt = cv2.blur(valid.astype(np.float32), (5, 5))
    return np.where(cnt > 0.05, acc / np.maximum(cnt, 1e-6), np.nan)


def grad(x):
    x = cv2.GaussianBlur(x, (0, 0), 2.0)
    return np.hypot(cv2.Sobel(x, cv2.CV_32F, 1, 0), cv2.Sobel(x, cv2.CV_32F, 0, 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("preprocessed_dir")
    ap.add_argument("bag")
    ap.add_argument("keys", nargs="+", help="calib.json results keys")
    # robot01's cloud covers the image down to about y 600 (the preprocess drops points under 1 m)
    ap.add_argument("--rows", default="0,200,420,600")
    ap.add_argument("--cols", default="0,430,860,1280")
    a = ap.parse_args()

    cfg = cc.load_calib(a.preprocessed_dir)
    cam = cc.camera_from_calib(cfg)
    pts, img = cc.load_bag(a.preprocessed_dir, a.bag)
    h, w = img.shape
    Gi = grad(img.astype(np.float32) / 255.0)
    ys = [int(v) for v in a.rows.split(",")]
    xs = [int(v) for v in a.cols.split(",")]
    for key in a.keys:
        t, R = cc.pose_from_result(cfg["results"][key])
        Lf = render(pts, cam, t, R, (h, w))
        cover = ~np.isnan(Lf)
        # gradients at the coverage border are the render's, not the scene's
        inner = cv2.erode(cover.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
        Gl = np.where(inner, grad(np.nan_to_num(Lf, nan=0.0).astype(np.float32)), 0)
        print(f"== {key}")
        good = []
        for y0, y1 in zip(ys[:-1], ys[1:]):
            line = []
            for x0, x1 in zip(xs[:-1], xs[1:]):
                msk = inner[y0 + S:y1 - S, x0 + S:x1 - S]
                if msk.mean() < 0.3:
                    line.append(f"x{x0:4d}-{x1:4d} y{y0:3d}-{y1:3d}:    no lidar       ")
                    continue
                ai = np.where(msk, Gi[y0 + S:y1 - S, x0 + S:x1 - S], 0)
                ai = (ai - ai[msk].mean()) * msk
                best = (-2.0, 0, 0)
                for dy in range(-S, S + 1, 2):
                    for dx in range(-S, S + 1, 2):
                        b = Gl[y0 + S - dy:y1 - S - dy, x0 + S - dx:x1 - S - dx]
                        b = (b - b[msk].mean()) * msk
                        ncc = (ai * b).sum() / (np.sqrt((ai * ai).sum() * (b * b).sum()) + 1e-9)
                        if ncc > best[0]:
                            best = (ncc, dx, dy)
                ncc, dx, dy = best
                reliable = ncc >= 0.3 and abs(dx) < S and abs(dy) < S
                if reliable:
                    good.append((dx, dy))
                line.append(f"x{x0:4d}-{x1:4d} y{y0:3d}-{y1:3d}: dx={dx:+3d} dy={dy:+3d} ncc={ncc:.2f}"
                            f"{' ' if reliable else '?'}")
            print("  " + " | ".join(line))
        if good:
            # RMS, not a median: the regions an error shows in are the ones a median outvotes
            g = np.array(good, np.float64)
            print(f"  reliable regions {len(good)}: RMS dx {np.sqrt(np.mean(g[:, 0] ** 2)):.1f} px, "
                  f"dy {np.sqrt(np.mean(g[:, 1] ** 2)):.1f} px   ('?' = unreliable)")


if __name__ == "__main__":
    main()
