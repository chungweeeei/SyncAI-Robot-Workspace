#!/usr/bin/env python3
"""Trigger for lio_ring_record.sh: Point-LIO's own frame moved under a resting robot.

The earliest reliable sign of the Point-LIO slides in
docs/pointlio-divergence-2026-11-04.md (§2.1, §5) is where the robot rests --
the dock, between laps -- in Point-LIO's own odom frame. Over 117 healthy laps
that position moved at most 7 cm from one rest to the next (11 cm in 19 h);
the lap that slid moved it 2.1 m, and the next jump came 70 min later. The
localizer's LOST, the trigger the recorder had, fired 80 min after the slide,
by when the ring had long dropped the lap that slid (11-03: 30 min, 1.4 m).

So: watch `pointlio/lio_odom`; once the robot has stood within REST_RADIUS of
one spot for REST_SEC, take that spot as a rest, and compare it with the last
rest at the same place in the map (within SAME_PLACE_M, through the
localizer's `map -> <robot_id>/pointlio_odom`). Same place in the map,
different place in Point-LIO's frame by more than JUMP_M = Point-LIO slid
since then, and the localizer absorbed it. The map gate is what lets the robot
rest somewhere else between dock visits without a trigger; with no map
transform (no map loaded) the last rest anywhere is compared, and the trigger
says so.

A rest is taken from the lidar odometry only, never cmd_vel: the dock
alignment's fine corrections are motion as far as this is concerned and just
delay the rest. A diverging Point-LIO moves meters while the robot stands, so
it never makes a rest at all -- the jump is seen at the first rest after it
settles, or not by this trigger (the localizer's LOST still is).

Known false trigger: a reset_odometry or a pointlio_node restart starts a new
odom frame where the robot stands, which reads as a jump at the next rest.
Both are rare, and the recorder's COOLDOWN_SEC / MAX_SAVES bound the cost.

Times are the messages' header stamps, so a bag replay drives it the same way.
Fires by appending a line to <ring>/triggers.log and touching <ring>/TRIGGER,
which is all the recorder's main loop reads.
"""

import math
import os
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

JUMP_M = float(os.environ.get('DOCK_JUMP_M', '0.3'))
REST_SEC = float(os.environ.get('DOCK_REST_SEC', '60'))
# A healthy Point-LIO at rest wanders by millimetres; lying down on the dock
# moves the lidar a few centimetres. 0.1 m is above both and far below a lap.
REST_RADIUS = 0.1
SAME_PLACE_M = 1.0


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rotate(q, v):
    """Rotate v by the unit quaternion q (x, y, z, w)."""
    x, y, z, w = q
    # v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
    cx = y * v[2] - z * v[1]
    cy = z * v[0] - x * v[2]
    cz = x * v[1] - y * v[0]
    return (
        v[0] + 2.0 * (w * cx + y * cz - z * cy),
        v[1] + 2.0 * (w * cy + z * cx - x * cz),
        v[2] + 2.0 * (w * cz + x * cy - y * cx),
    )


class DockJumpWatch(Node):
    def __init__(self, robot_id, ring):
        super().__init__('lio_dock_jump_watch', namespace=robot_id)
        self.odom_frame = f'{robot_id}/pointlio_odom'
        self.ring = Path(ring)
        self.map_from_odom = None  # (translation, quaternion)
        self.anchor = None  # (x, y, t) where the current stay began
        self.rest_taken = False
        self.rests = []  # (odom_xy, map_xy or None, t), oldest first

        # Best effort reads both a reliable and a best-effort publisher.
        self.create_subscription(Odometry, 'pointlio/lio_odom', self.on_odom,
                                 qos_profile_sensor_data)
        # /tf is global, not namespaced: TF frames carry the robot_id instead.
        self.create_subscription(
            TFMessage, '/tf', self.on_tf,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(
            f'watching pointlio/lio_odom for a rest that moved > {JUMP_M} m in '
            f'{self.odom_frame} (rest = {REST_SEC:.0f} s within {REST_RADIUS} m)')

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == self.odom_frame:
                tr, r = t.transform.translation, t.transform.rotation
                self.map_from_odom = ((tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = stamp_sec(msg.header.stamp)
        if self.anchor is None or math.hypot(p.x - self.anchor[0],
                                             p.y - self.anchor[1]) > REST_RADIUS:
            self.anchor = (p.x, p.y, t)
            self.rest_taken = False
            return
        if self.rest_taken or t - self.anchor[2] < REST_SEC:
            return
        self.rest_taken = True
        self.on_rest((p.x, p.y, p.z), t)

    def on_rest(self, odom_p, t):
        map_xy = None
        if self.map_from_odom is not None:
            tr, q = self.map_from_odom
            v = rotate(q, odom_p)
            map_xy = (v[0] + tr[0], v[1] + tr[1])

        prev = None
        for rest in reversed(self.rests):
            if map_xy is None or rest[1] is None or math.dist(map_xy, rest[1]) <= SAME_PLACE_M:
                prev = rest
                break
        odom_xy = odom_p[:2]
        self.rests.append((odom_xy, map_xy, t))
        del self.rests[:-50]

        where = (f'map ({map_xy[0]:.2f}, {map_xy[1]:.2f})' if map_xy
                 else 'no map transform, compared with the last rest anywhere')
        if prev is None:
            self.get_logger().info(
                f'rest at odom ({odom_xy[0]:.2f}, {odom_xy[1]:.2f}), {where}: first here')
            return
        jump = math.dist(odom_xy, prev[0])
        line = (f'rest at odom ({odom_xy[0]:.2f}, {odom_xy[1]:.2f}) is '
                f'{jump:.2f} m from the last rest here ({prev[0][0]:.2f}, {prev[0][1]:.2f}), '
                f'{(t - prev[2]) / 60:.0f} min before; {where}')
        if jump <= JUMP_M:
            self.get_logger().info(line)
            return
        self.get_logger().warn(f'{line} -- over DOCK_JUMP_M {JUMP_M}, triggering a save')
        with open(self.ring / 'triggers.log', 'a') as f:
            f.write(f'dock jump: {line}\n')
        (self.ring / 'TRIGGER').touch()


def main():
    if len(sys.argv) != 3:
        sys.exit('usage: dock_jump_watch.py <robot_id> <ring dir>')
    rclpy.init()
    node = DockJumpWatch(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
