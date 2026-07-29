import math

import rclpy
import structlog

from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from syncai_common.msg import MotorStates

from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (radians) of a quaternion. Exact for the pure-z rotations the
    2D-projected LIO chain produces; for anything else it is the usual ZYX
    yaw component."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TelemetrySubscriber:
    """Feed the internal telemetry WebSocket from the raw high-rate topics.

    This deliberately bypasses the aggregated ``robot_state`` topic (and the
    RobotRepo behind GET /api/v1/robot/state — that REST payload is a frozen
    third-party contract): the 3D viewer needs pose at a rate a gait actually
    moves at. ``robot_state`` runs at 10 Hz now, but its ``timestamp`` has only
    whole-second resolution and it is reached over a polled REST endpoint, so it
    still cannot be smoothed into continuous motion client-side.

    1. Pose: ``odom`` (lio_bridge, 20 Hz, odom frame) composed with the
       map->odom correction from TF into a map-frame planar pose. Both inputs
       are 2D-projected by lio_bridge, so the composition is done as planar
       (x, y, yaw) math rather than full quaternion algebra. Until the
       localizer has been relocalized there is no map->odom, so samples are
       dropped with an edge-triggered log — same behaviour (and reason) as the
       body_cloud subscriber.

    2. Joints: ``motor_states`` (driver_manager's UDP telemetry bridge),
       reduced to {URDF joint name: position} — exactly the vocabulary the
       frontend uses to look up GLB nodes.
    """

    def __init__(
        self, logger: structlog.stdlib.BoundLogger, telemetry_repo: TelemetryRepo
    ):
        self._logger = logger
        self._repo = telemetry_repo

        # Edge-triggered logging for the map->odom lookup, same pattern as
        # PointCloudSubscriber._cloud_tf_available: log once when pose samples
        # start dropping and once when they recover, not per message.
        self._tf_available = None

        self._tf_buffer: Buffer = None
        self._tf_listener: TransformListener = None

    def register(self, node: Node):
        # A second Buffer/TransformListener next to the point-cloud
        # subscriber's own means a duplicate in-process /tf subscription. That
        # duplication is accepted on purpose: sharing one buffer would couple
        # the two subscribers' lifecycles for the sake of a ~20 Hz planar
        # transform topic, which costs nothing to receive twice.
        # spin_thread=False for the same reason as there: no extra
        # GIL-contending thread; the executor spins the /tf subscriptions.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node, spin_thread=False)

        # Both QoS profiles are best-effort keep-last: only the newest sample
        # matters (the repo is single-slot anyway). Compatible with both
        # publishers — odom is reliable (best-effort sub on reliable pub is
        # fine), motor_states is SensorDataQoS (best-effort, must match).
        qos = QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )

        # Relative topic names, so both inherit the robot_id namespace.
        node.create_subscription(
            msg_type=Odometry, topic="odom", callback=self._odom_cb, qos_profile=qos
        )
        node.create_subscription(
            msg_type=MotorStates,
            topic="motor_states",
            callback=self._motor_states_cb,
            qos_profile=qos,
        )

    def _odom_cb(self, msg: Odometry):
        odom_frame = msg.header.frame_id
        try:
            # map->odom is a slowly-varying correction; latest available
            # (Time()) rather than the message stamp, so the 20 Hz odom feed
            # is never dropped by future-extrapolation errors.
            tf = self._tf_buffer.lookup_transform("map", odom_frame, rclpy.time.Time())
        except TransformException as exc:
            if self._tf_available is not False:
                self._logger.warning(
                    "telemetry pose dropping: TF unavailable (relocalized yet? "
                    "map->odom comes from lio_bridge only after the localizer "
                    "is up)",
                    odom_frame=odom_frame,
                    error=str(exc),
                )
                self._tf_available = False
            return

        if self._tf_available is not True:
            self._logger.info("telemetry pose streaming", odom_frame=odom_frame)
            self._tf_available = True

        # Planar compose: map_T_base = map_T_odom * odom_T_base. Everything
        # upstream is projected to (x, y, yaw) by lio_bridge, so 2D math is
        # exact here and much clearer than a full quaternion product.
        t = tf.transform.translation
        q = tf.transform.rotation
        tf_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        p = msg.pose.pose.position
        oq = msg.pose.pose.orientation
        odom_yaw = _yaw_from_quat(oq.x, oq.y, oq.z, oq.w)

        cos_y, sin_y = math.cos(tf_yaw), math.sin(tf_yaw)
        self._repo.update_pose(
            x=t.x + cos_y * p.x - sin_y * p.y,
            y=t.y + sin_y * p.x + cos_y * p.y,
            z=t.z + p.z,
            yaw_deg=math.degrees(tf_yaw + odom_yaw),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _motor_states_cb(self, msg: MotorStates):
        self._repo.update_joints(
            joints={m.name: float(m.q) for m in msg.states},
            # MotorStates.timestamp is nanoseconds (stamped from now() by
            # driver_manager); the wire format uses seconds like the pose.
            stamp=msg.timestamp * 1e-9,
        )


def init_telemetry_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, telemetry_repo: TelemetryRepo
) -> TelemetrySubscriber:
    subscriber = TelemetrySubscriber(logger=logger, telemetry_repo=telemetry_repo)
    subscriber.register(node=node)
    return subscriber
