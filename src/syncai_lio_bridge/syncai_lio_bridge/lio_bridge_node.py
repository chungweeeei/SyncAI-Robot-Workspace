"""Bridge FAST-LIO2 3D localization onto the wheel-odometry TF chain.

The FASTLIO2 localizer corrects the LIO chain (map -> lio_odom -> lio_body)
but the nav stack consumes the wheel-odom chain
(map -> <robot_id>/odom -> <robot_id>/base_link). This node computes the
AMCL-style correction and broadcasts it:

    map->odom = (map->lio_body)              # 3D pose of the lidar in map
              @ inv(base_link->lidar_top)    # static mount extrinsic
              @ inv(odom->base_link)         # wheel odometry

lio_body is physically the same frame as <robot_id>/lidar_top (the LIO state
is the lidar pose), which is what makes the composition valid.

The result is projected to 2D (x, y, yaw; z/roll/pitch zeroed) before
broadcasting so the planar nav stack never sees a tilted odom frame.

Until the localizer has been given a map (/localizer/relocalize service),
map->lio_odom does not exist and this node just waits — same behavior as
AMCL before an initial pose.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def transform_to_matrix(transform):
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    t = transform.translation
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    mat = np.eye(4)
    mat[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def invert_rigid(mat):
    """Inverse of a rigid 4x4 transform."""
    inv = np.eye(4)
    r_t = mat[:3, :3].T
    inv[:3, :3] = r_t
    inv[:3, 3] = -r_t @ mat[:3, 3]
    return inv


class LioBridgeNode(Node):

    def __init__(self):
        super().__init__('lio_bridge_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lio_body_frame', 'lio_body')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wheel_odom_frame', 'odom')
        self.declare_parameter('lidar_frame', 'lidar_top')
        self.declare_parameter('publish_rate', 20.0)
        # Future-date the stamp (like AMCL's transform_tolerance) so consumers
        # can interpolate between our broadcasts without extrapolation errors.
        self.declare_parameter('transform_tolerance', 0.1)

        self.map_frame = self.get_parameter('map_frame').value
        self.lio_body_frame = self.get_parameter('lio_body_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_odom_frame = self.get_parameter('wheel_odom_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value
        rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.localized = False
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

        self.get_logger().info(
            f'lio_bridge: ({self.map_frame} -> {self.lio_body_frame}) + '
            f'({self.base_frame} -> {self.lidar_frame}) + '
            f'({self.wheel_odom_frame} -> {self.base_frame}) => '
            f'{self.map_frame} -> {self.wheel_odom_frame} @ {rate} Hz')

    def lookup(self, target, source):
        return self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=Duration(seconds=0.0))

    def timer_cb(self):
        try:
            t_map_liobody = self.lookup(self.map_frame, self.lio_body_frame)
            t_base_lidar = self.lookup(self.base_frame, self.lidar_frame)
            t_odom_base = self.lookup(self.wheel_odom_frame, self.base_frame)
        except Exception as ex:  # noqa: BLE001 - tf2 raises several lookup errors
            self.get_logger().info(
                f'waiting for TF (relocalized yet?): {ex}',
                throttle_duration_sec=5.0)
            return

        m_map_lidar = transform_to_matrix(t_map_liobody.transform)
        m_base_lidar = transform_to_matrix(t_base_lidar.transform)
        m_odom_base = transform_to_matrix(t_odom_base.transform)

        # map->base = map->lidar ∘ lidar->base ; map->odom = map->base ∘ base->odom
        m_map_odom = m_map_lidar @ invert_rigid(m_base_lidar) @ invert_rigid(m_odom_base)

        # project to 2D: keep x, y, yaw only
        yaw = math.atan2(m_map_odom[1, 0], m_map_odom[0, 0])

        msg = TransformStamped()
        stamp = self.get_clock().now() + Duration(seconds=self.transform_tolerance)
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.wheel_odom_frame
        msg.transform.translation.x = float(m_map_odom[0, 3])
        msg.transform.translation.y = float(m_map_odom[1, 3])
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(msg)

        if not self.localized:
            self.localized = True
            self.get_logger().info(
                f'localization bridged: {self.map_frame} -> '
                f'{self.wheel_odom_frame} = '
                f'({msg.transform.translation.x:.3f}, '
                f'{msg.transform.translation.y:.3f}, yaw {yaw:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = LioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
