#!/usr/bin/env python3
"""Save one frame of <robot_id>/image_raw/compressed. Runs in the robot container (needs rclpy).

For looking at what the camera sees without a display: whether a recording spot has structure at
several depths and nobody walking through it, or why collect_board.py stopped keeping views
(usually a board held too far away or square-on).

usage: snapshot.py <out.jpg> [--scale 0.5] [--robot-id ID] [--topic image_raw/compressed]
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

import calib_common as cc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--robot-id", help="default: [system] robot_id of config/system.ini")
    ap.add_argument("--topic", default="image_raw/compressed", help="relative to the robot namespace")
    ap.add_argument("--timeout", type=float, default=5.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("calib_snapshot", namespace=a.robot_id or cc.read_robot_id())
    got = []
    sub = node.create_subscription(CompressedImage, a.topic, got.append, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + a.timeout * 1e9
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        raise SystemExit(f"no image on {sub.topic_name} within {a.timeout:g} s: is the camera feeder up?")
    img = cv2.imdecode(np.frombuffer(bytes(got[0].data), np.uint8), cv2.IMREAD_COLOR)
    if a.scale != 1.0:
        img = cv2.resize(img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
    cv2.imwrite(a.out, img)
    print(f"{sub.topic_name} -> {a.out} ({img.shape[1]}x{img.shape[0]})")


if __name__ == "__main__":
    main()
