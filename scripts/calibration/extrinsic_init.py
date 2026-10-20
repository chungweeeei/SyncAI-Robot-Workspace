#!/usr/bin/env python3
"""Write the URDF's camera joint into a preprocessed calib.json as the extrinsic starting point.

direct_visual_lidar_calibration's preprocess writes calib.json afresh each time it runs, so run
this after every preprocess. The joint must hang the camera off the frame the cloud is stamped
with (lidar_top): its origin is then exactly the T_lidar_camera the tools fit.

usage: extrinsic_init.py <preprocessed_dir> [--urdf src/syncai_bringup/description/G23.urdf]
                         [--joint camera_optical_joint] [--key init_T_lidar_camera]
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np

import calib_common as cc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("preprocessed_dir")
    ap.add_argument("--urdf", default="src/syncai_bringup/description/G23.urdf")
    ap.add_argument("--joint", default="camera_optical_joint")
    ap.add_argument("--key", default="init_T_lidar_camera")
    a = ap.parse_args()

    joint = next((j for j in ET.parse(a.urdf).getroot().iter("joint") if j.get("name") == a.joint), None)
    if joint is None:
        raise SystemExit(f"no joint {a.joint} in {a.urdf}")
    parent = joint.find("parent").get("link")
    origin = joint.find("origin")
    xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
    rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
    if parent != "lidar_top":
        print(f"WARNING: {a.joint} hangs off {parent}, not lidar_top (the frame the cloud is stamped with)")

    t, R = np.array(xyz), cc.rpy_to_R(*rpy)
    cfg = cc.load_calib(a.preprocessed_dir)
    cfg.setdefault("results", {})[a.key] = cc.result_from_pose(t, R)
    cc.save_calib(a.preprocessed_dir, cfg)
    print(f"{a.key} <- {a.joint} ({parent} -> {joint.find('child').get('link')}): {cc.urdf_origin(t, R)}")


if __name__ == "__main__":
    main()
