#!/usr/bin/env python3
"""Print calib.json poses as URDF origins, and each one relative to the first.

The rotation difference is given about the first pose's camera axes (tilt up, pan right, roll),
which is how a misalignment looks in the image; at this camera's ~710 px focal length 1 deg is
about 12 px.

usage: compare_extrinsics.py <calib.json> <key> <key> [...]
"""
import argparse
import json

import numpy as np

import calib_common as cc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("calib_json")
    ap.add_argument("keys", nargs="+")
    a = ap.parse_args()

    with open(a.calib_json) as f:
        res = json.load(f)["results"]
    poses = [(k, *cc.pose_from_result(res[k])) for k in a.keys]
    _, t0, R0 = poses[0]
    for k, t, R in poses:
        z = R[:, 2]
        elev = np.degrees(np.arcsin(np.clip(z[2], -1, 1)))
        print(f"{k}: {cc.urdf_origin(t, R)}")
        print(f"    optical axis in the lidar frame {np.round(z, 4)} ({elev:+.2f} deg above the lidar's x-y plane)")
        if k != poses[0][0]:
            d = cc.camera_delta_deg(R0, R)
            print(f"    vs {poses[0][0]}: tilt up {d[0]:+.3f}  pan right {d[1]:+.3f}  roll {d[2]:+.3f} deg; "
                  f"translation {np.round(t - t0, 4)} m (|d| {np.linalg.norm(t - t0):.4f})")


if __name__ == "__main__":
    main()
