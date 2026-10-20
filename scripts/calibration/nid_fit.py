#!/usr/bin/env python3
"""Lidar-camera extrinsic by NID, with any of rotation / translation / focal scale held fixed.

The cost mirrors direct_visual_lidar_calibration's (normalized information distance over a 16x16
joint histogram of lidar intensity vs image intensity, points hidden behind nearer ones culled
once at the starting pose). What that tool cannot do is freeze a parameter, and that is the point
here: on robot01 (2026-10-20) its free 6-DoF fit moved the camera 27 cm down and tilted it 3.2 deg
up to compensate -- for a scene 2-6 m away the two look alike -- while the tape-measured position
is good to 1-2 cm. So the default fits rotation only (--free w) and keeps the translation of the
starting pose; docs/lidar-camera-calibration.md has the evidence.

Parametrisation, around the starting pose (--from):
  w  [deg]  camera rotation about its own axes: tilt up, pan right, roll
  c  [m]    camera centre offset, lidar frame
  s         focal scale applied to fx and fy (a check on the intrinsics, not a calibration)

Several bags of the same preprocessed directory are fitted jointly.

usage: nid_fit.py <preprocessed_dir> <bag[,bag...]> [--free w] [--from init_T_lidar_camera]
                  [--write KEY] [--profile]
"""
import argparse

import numpy as np

import calib_common as cc


def prepare(pts, img, cam, t0, R0, rng, max_points=400000):
    """Points the camera sees from the starting pose (z-buffered on 4 px cells), subsampled."""
    h, w = img.shape
    uv, m = cam.project((pts[:, :3] - t0) @ R0)
    pts, z = pts[m], ((pts[m, :3] - t0) @ R0)[:, 2]
    inside = (uv[:, 0] >= -20) & (uv[:, 0] < w + 20) & (uv[:, 1] >= -20) & (uv[:, 1] < h + 20)
    pts, uv, z = pts[inside], uv[inside], z[inside]
    cw = (w + 40) // 4 + 1
    cell = ((uv[:, 1] + 20) // 4).astype(int) * cw + ((uv[:, 0] + 20) // 4).astype(int)
    zmin = np.full(cell.max() + 1, np.inf)
    np.minimum.at(zmin, cell, z)
    pts = pts[z <= zmin[cell] * 1.05 + 0.05]
    if len(pts) > max_points:
        pts = pts[rng.choice(len(pts), max_points, replace=False)]
    return pts, img.astype(np.float64) / 255.0


def nid(pts, img, cam, t, R, bins=16):
    h, w = img.shape
    uv, m = cam.project((pts[:, :3] - t) @ R)
    inten = pts[m, 3]
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < h - 1)
    uv, inten = uv[ok], inten[ok]
    x0, y0 = uv[:, 0].astype(int), uv[:, 1].astype(int)
    fx, fy = uv[:, 0] - x0, uv[:, 1] - y0
    pix = (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x0 + 1] * fx * (1 - fy)
           + img[y0 + 1, x0] * (1 - fx) * fy + img[y0 + 1, x0 + 1] * fx * fy)
    bl = np.clip((inten * bins).astype(int), 0, bins - 1)
    # soft binning on the image side keeps the cost continuous in the pose
    v = np.clip(pix * bins - 0.5, 0, bins - 1 - 1e-9)
    bi = v.astype(int)
    a = v - bi
    hist = np.zeros((bins, bins))
    np.add.at(hist, (bi, bl), 1 - a)
    np.add.at(hist, (np.minimum(bi + 1, bins - 1), bl), a)
    hist /= hist.sum()

    def entropy(p):
        return -(p * np.log(p + 1e-12)).sum()

    return 2.0 - (entropy(hist.sum(1)) + entropy(hist.sum(0))) / entropy(hist)


def nelder_mead(f, x0, step, iters=400, tol=1e-7):
    n = len(x0)
    sim = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    fs = [f(x) for x in sim]
    for _ in range(iters):
        o = np.argsort(fs)
        sim, fs = [sim[i] for i in o], [fs[i] for i in o]
        if abs(fs[-1] - fs[0]) < tol:
            break
        c = np.mean(sim[:-1], axis=0)
        xr = c + (c - sim[-1])
        fr = f(xr)
        if fr < fs[0]:
            xe = c + 2 * (c - sim[-1])
            fe = f(xe)
            sim[-1], fs[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fs[-2]:
            sim[-1], fs[-1] = xr, fr
        else:
            xc = c + 0.5 * (sim[-1] - c)
            fc = f(xc)
            if fc < fs[-1]:
                sim[-1], fs[-1] = xc, fc
            else:
                sim = [sim[0]] + [sim[0] + 0.5 * (x - sim[0]) for x in sim[1:]]
                fs = [fs[0]] + [f(x) for x in sim[1:]]
    i = int(np.argmin(fs))
    return sim[i], fs[i]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("preprocessed_dir")
    ap.add_argument("bags", help="comma-separated bag names of that directory")
    ap.add_argument("--free", default="w", help="comma-separated subset of w, c, s (default w: rotation)")
    ap.add_argument("--from", dest="start", default="init_T_lidar_camera", help="calib.json results key")
    ap.add_argument("--write", help="store the fitted pose under this calib.json results key")
    ap.add_argument("--profile", action="store_true", help="print the cost around the result, axis by axis")
    a = ap.parse_args()

    free = a.free.split(",")
    cfg = cc.load_calib(a.preprocessed_dir)
    cam0 = cc.camera_from_calib(cfg)
    t0, R0 = cc.pose_from_result(cfg["results"][a.start])
    rng = np.random.default_rng(0)
    data = []
    for b in a.bags.split(","):
        pts, img = cc.load_bag(a.preprocessed_dir, b)
        data.append(prepare(pts, img, cam0, t0, R0, rng))

    def unpack(x):
        x = list(x)
        w = np.radians(x[:3]) if "w" in free else np.zeros(3)
        x = x[3:] if "w" in free else x
        c = np.array(x[:3]) if "c" in free else np.zeros(3)
        x = x[3:] if "c" in free else x
        s = x[0] if "s" in free else 1.0
        return w, c, s

    def pose(w, c):
        return t0 + c, R0 @ cc.expso3(w)

    def cost_at(w, c, s):
        t, R = pose(w, c)
        K = cam0.K.copy()
        K[0, 0] *= s
        K[1, 1] *= s
        cam = cc.Camera(K, cam0.D, cam0.model)
        return sum(nid(p, im, cam, t, R) for p, im in data) / len(data)

    def cost(x):
        return cost_at(*unpack(x))

    x0, step = [], []
    if "w" in free:
        x0, step = x0 + [0, 0, 0], step + [1.0, 1.0, 1.0]
    if "c" in free:
        x0, step = x0 + [0, 0, 0], step + [0.03, 0.03, 0.03]
    if "s" in free:
        x0, step = x0 + [1.0], step + [0.03]
    f0 = cost(x0)
    x, fbest = nelder_mead(cost, x0, step)
    # restart once from the optimum: Nelder-Mead stalls on a collapsed simplex
    x, fbest = nelder_mead(cost, x, [s * 0.3 for s in step])
    w, c, s = unpack(x)
    t, R = pose(w, c)
    print(f"bags {a.bags}, free {free}: NID {f0:.5f} -> {fbest:.5f}")
    print(f"  rotation (deg) tilt up {np.degrees(w[0]):+.3f}  pan right {np.degrees(w[1]):+.3f}  "
          f"roll {np.degrees(w[2]):+.3f};  translation (m) {np.round(c, 4)};  focal scale {s:.4f}")
    print(f"  URDF origin: {cc.urdf_origin(t, R)}")

    if a.profile:
        print("  NID - optimum (x1e-4) at -2 -1 -0.5 0 +0.5 +1 +2 deg:")
        for ax, name in enumerate(["tilt", "pan", "roll"]):
            row = []
            for d in (-2, -1, -0.5, 0, 0.5, 1, 2):
                wd = np.array(w)
                wd[ax] += np.radians(d)
                row.append(cost_at(wd, c, s))
            print(f"    {name:5s}" + "".join(f"{(v - row[3]) * 1e4:+7.1f}" for v in row))

    if a.write:
        cfg = cc.load_calib(a.preprocessed_dir)
        cfg["results"][a.write] = cc.result_from_pose(t, R)
        cc.save_calib(a.preprocessed_dir, cfg)
        print(f"  stored as results.{a.write}")


if __name__ == "__main__":
    main()
