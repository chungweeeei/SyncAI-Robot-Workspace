"""LIO odometry provider — the robot's only odometry source.

Wheel odometry is gone (the Isaac Sim OmniGraph odom publishers are disabled):
the robot relies solely on FAST-LIO2 lidar-inertial odometry. This node turns
the LIO chain (map -> lio_odom -> lio_body) into the planar chain the nav
stack consumes:

    odom -> base_link   from /<robot_id>/fastlio2/lio_odom, projected to 2D
                        (lio_body is physically <robot_id>/lidar_top, so the
                        static base_link->lidar_top extrinsic maps it to base)
    /<robot_id>/odom    nav_msgs/Odometry republished for the twist consumers
                        (controller_server, task_runner, robot_state); linear
                        velocity comes from LIO (body frame), angular.z from
                        the lidar IMU gyro since LIO leaves twist.angular empty
    map -> odom         AMCL-style correction using the localizer's
                        map -> lio_odom TF:
                        map->odom = P2D(map->base) @ inv(P2D(odom->base))

Everything is projected to 2D (x, y, yaw; z/roll/pitch zeroed) before
broadcasting so the planar nav stack never sees a tilted frame.

odom -> base_link and the odom topic are published as soon as LIO produces
odometry — before relocalization, matching AMCL semantics where the odom
chain exists before an initial pose. Until the localizer has been given a map
(/localizer/relocalize service), map -> lio_odom does not exist and the
map -> odom broadcast just waits.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def to_matrix(translation, rotation):
    """Translation + quaternion (geometry_msgs) -> 4x4 homogeneous matrix.

    Works for both Transform (translation/rotation) and Pose
    (position/orientation) components.
    """
    t, q = translation, rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    mat = np.eye(4)
    mat[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def invert_rigid(mat):
    """Inverse of a rigid 4x4 transform."""
    inv = np.eye(4)
    r_t = mat[:3, :3].T
    inv[:3, :3] = r_t
    inv[:3, 3] = -r_t @ mat[:3, 3]
    return inv


def project_2d(mat):
    """4x4 transform -> planar (x, y, yaw) projection."""
    yaw = math.atan2(mat[1, 0], mat[0, 0])
    return mat[0, 3], mat[1, 3], yaw


def pose_2d_matrix(x, y, yaw):
    """Planar (x, y, yaw) -> 4x4 homogeneous matrix (z/roll/pitch zero)."""
    c, s = math.cos(yaw), math.sin(yaw)
    mat = np.eye(4)
    mat[0, 0], mat[0, 1] = c, -s
    mat[1, 0], mat[1, 1] = s, c
    mat[0, 3], mat[1, 3] = x, y
    return mat


class LioBridgeNode(Node):
    def __init__(self):
        super().__init__("lio_bridge_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("lidar_frame", "lidar_top")
        self.declare_parameter("publish_rate", 20.0)
        # Future-date the stamp (like AMCL's transform_tolerance) so consumers
        # can interpolate between our broadcasts without extrapolation errors.
        # Also bridges the gap between the ~10 Hz LIO updates and this node's
        # publish rate.
        self.declare_parameter("transform_tolerance", 0.1)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.transform_tolerance = self.get_parameter("transform_tolerance").value
        rate = self.get_parameter("publish_rate").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Latest LIO state (set in callbacks, consumed by the timer).
        self.lio_pose_mat = None  # 4x4 of lio_odom -> lio_body
        self.lio_odom_frame = None  # taken from the msg header, e.g. robot01/lio_odom
        self.lio_linear = (0.0, 0.0)  # body-frame linear velocity (x, y)
        self.yaw_rate = 0.0  # from the lidar IMU gyro (LIO leaves twist.angular empty)
        # base_link -> lidar_top mount extrinsic; static, cached on first lookup.
        self.base_lidar_mat = None

        # Sensor-data QoS on both subs: best-effort is compatible with either
        # reliable (lio_node) or best-effort publishers.
        self.lio_sub = self.create_subscription(
            Odometry, "fastlio2/lio_odom", self.lio_cb, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Imu, "livox/imu", self.imu_cb, qos_profile_sensor_data
        )
        # Reliable publisher: the SensorDataQoS (best-effort) subscribers of
        # robot_state and the reliable nav odom subscribers both match it.
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        self.localized = False
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

        self.get_logger().info(
            f"lio_bridge: LIO odometry provider — "
            f"{self.odom_frame} -> {self.base_frame} + odom topic from "
            f"fastlio2/lio_odom, {self.map_frame} -> {self.odom_frame} from "
            f"the localizer TF @ {rate} Hz"
        )

    def lio_cb(self, msg):
        pose = msg.pose.pose
        self.lio_pose_mat = to_matrix(pose.position, pose.orientation)
        self.lio_odom_frame = msg.header.frame_id
        self.lio_linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def imu_cb(self, msg):
        # IMU is co-located with the lidar; planar robot -> gyro z is yaw rate.
        self.yaw_rate = msg.angular_velocity.z

    def lookup(self, target, source):
        return self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(), timeout=Duration(seconds=0.0)
        )

    def base_lidar(self):
        """Cached static base_link -> lidar_top mount extrinsic (or None)."""
        if self.base_lidar_mat is None:
            try:
                t = self.lookup(self.base_frame, self.lidar_frame)
            except Exception as ex:  # noqa: BLE001 - tf2 raises several lookup errors
                self.get_logger().info(
                    f"waiting for {self.base_frame} -> {self.lidar_frame}: {ex}",
                    throttle_duration_sec=5.0,
                )
                return None
            tr = t.transform
            self.base_lidar_mat = to_matrix(tr.translation, tr.rotation)
        return self.base_lidar_mat

    def make_tf(self, stamp, frame_id, child_frame_id, x, y, yaw):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        return msg

    def timer_cb(self):
        if self.lio_pose_mat is None:
            self.get_logger().info(
                "waiting for fastlio2/lio_odom (LIO initializing?)",
                throttle_duration_sec=5.0,
            )
            return
        m_base_lidar = self.base_lidar()
        if m_base_lidar is None:
            return

        # lio_body is physically the lidar frame, so
        # lio_odom->base = lio_odom->lio_body @ lidar->base.
        m_lioodom_base = self.lio_pose_mat @ invert_rigid(m_base_lidar)
        ob_x, ob_y, ob_yaw = project_2d(m_lioodom_base)

        now = self.get_clock().now()
        tf_stamp = (now + Duration(seconds=self.transform_tolerance)).to_msg()

        # odom -> base_link: published before relocalization, like AMCL where
        # the odom chain exists before an initial pose.
        self.tf_broadcaster.sendTransform(
            self.make_tf(tf_stamp, self.odom_frame, self.base_frame, ob_x, ob_y, ob_yaw)
        )

        # Odometry topic for the twist consumers (controller_server,
        # task_runner, robot_state — none of them read the pose).
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = ob_x
        odom.pose.pose.position.y = ob_y
        odom.pose.pose.orientation.z = math.sin(ob_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(ob_yaw / 2.0)
        odom.twist.twist.linear.x = self.lio_linear[0]
        odom.twist.twist.linear.y = self.lio_linear[1]
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)

        # map -> odom: needs the localizer's map -> lio_odom, which only
        # exists after /localizer/relocalize has been given a map.
        try:
            t_map_lioodom = self.lookup(self.map_frame, self.lio_odom_frame)
        except Exception as ex:  # noqa: BLE001 - tf2 raises several lookup errors
            self.get_logger().info(
                f"waiting for TF (relocalized yet?): {ex}", throttle_duration_sec=5.0
            )
            return

        # Project map->base to 2D, then subtract the planar odom->base pose;
        # both are planar so the composition stays planar and consistent.
        tr = t_map_lioodom.transform
        m_map_base = to_matrix(tr.translation, tr.rotation) @ m_lioodom_base
        mb_x, mb_y, mb_yaw = project_2d(m_map_base)
        m_map_odom = pose_2d_matrix(mb_x, mb_y, mb_yaw) @ invert_rigid(
            pose_2d_matrix(ob_x, ob_y, ob_yaw)
        )
        mo_x, mo_y, mo_yaw = project_2d(m_map_odom)

        self.tf_broadcaster.sendTransform(
            self.make_tf(tf_stamp, self.map_frame, self.odom_frame, mo_x, mo_y, mo_yaw)
        )

        if not self.localized:
            self.localized = True
            self.get_logger().info(
                f"localization bridged: {self.map_frame} -> {self.odom_frame} = "
                f"({mo_x:.3f}, {mo_y:.3f}, yaw {mo_yaw:.3f})"
            )


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


if __name__ == "__main__":
    main()
