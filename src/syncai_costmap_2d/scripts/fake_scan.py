#!/usr/bin/env python3
"""Publish a synthetic, properly-timestamped LaserScan for costmap testing.

A forward-facing 50-beam scan, all returns at 2.0 m, in the base_scan frame.
Used by three_layer_test.launch.py with use_fake_scan:=true. The timestamp MUST
be current — tf2_ros::MessageFilter drops messages older than the tf cache, so a
zero-stamped scan (e.g. from `ros2 topic pub`) would never reach the obstacle layer.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self):
        super().__init__("fake_scan")
        self.declare_parameter("frame_id", "base_scan")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("range", 2.0)
        self.declare_parameter("num_beams", 50)
        self.frame_id = self.get_parameter("frame_id").value
        self.rng = float(self.get_parameter("range").value)
        self.n = int(self.get_parameter("num_beams").value)
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter("topic").value, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.angle_min = -math.pi / 2
        m.angle_max = math.pi / 2
        m.angle_increment = math.pi / (self.n - 1)
        m.range_min = 0.1
        m.range_max = 10.0
        m.ranges = [self.rng] * self.n
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
