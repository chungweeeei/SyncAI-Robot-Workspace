import math
import time

import numpy as np
import rclpy
import structlog

from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from syncai_common.msg import MotorStates

from syncai_backend.helpers.pointcloud import quat_to_rotation_matrix
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# How long one `odom` sample keeps ownership of the pose. In AUTO both feeds
# run at ~20 Hz, so any value comfortably above a sample period makes the
# mapping fallback below inert there; a second also covers a lio_bridge hiccup
# without handing the pose back and forth between two sources mid-drive.
PRIMARY_POSE_HOLD_S = 1.0

# The two URDF frames the mapping fallback composes through, under whatever
# namespace prefix the LIO body frame carries. Constants rather than
# parameters: this package declares no ROS parameters, and these names are
# fixed by description/G23.urdf — syncai_lio_bridge hardcodes the same pair as
# its defaults for the same composition.
BASE_FRAME = "base_link"
LIDAR_FRAME = "lidar_top"


def _rigid(translation, rotation) -> tuple[np.ndarray, np.ndarray]:
    """(3x3 rotation, 3-vector translation) from any ROS translation/rotation
    pair — a Transform's or a Pose's, which differ only in field names."""
    return (
        quat_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        np.array([translation.x, translation.y, translation.z]),
    )


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

    1. Pose, from whichever of two feeds the running mode provides:

       * AUTO: ``odom`` (lio_bridge, 20 Hz, odom frame) composed with the
         map->odom correction from TF into a map-frame planar pose. Both inputs
         are 2D-projected by lio_bridge, so the composition is done as planar
         (x, y, yaw) math rather than full quaternion algebra. Until the
         localizer has been relocalized there is no map->odom, so samples are
         dropped with an edge-triggered log — same behaviour (and reason) as the
         body_cloud subscriber.

       * MANUAL: ``pointlio/lio_odom`` (the LIO front end, 20 Hz) composed the
         long way round — see ``_lio_odom_cb``. The mapping session runs neither
         lio_bridge nor the localizer, so ``odom`` does not exist and map->odom
         is never broadcast: without this feed the console has no pose for the
         whole run, which is what left the robot model undrawn and the mapping
         viewport's Focus mode with nothing to follow.

       ``odom`` wins whenever it is live (``PRIMARY_POSE_HOLD_S``): both topics
       are published in AUTO, and lio_bridge's is the one the rest of the stack
       navigates by, so the console must not show a second opinion of where the
       robot is.

    2. Joints: ``motor_states`` (driver_manager's UDP telemetry bridge),
       reduced to {URDF joint name: position} — exactly the vocabulary the
       frontend uses to look up GLB nodes.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        telemetry_repo: TelemetryRepo,
        tf_buffer: Buffer,
    ):
        self._logger = logger
        self._repo = telemetry_repo
        # The backend's one shared TF buffer, injected from main.py — see
        # subscribers/tf.py for why this is no longer built here. Read from the
        # executor threads that run the callbacks below; tf2's Buffer guards
        # itself, so sharing it with the point-cloud subscriber is safe under
        # the MultiThreadedExecutor.
        self._tf_buffer = tf_buffer

        # Edge-triggered logging for the map->odom lookup, same pattern as
        # PointCloudSubscriber._cloud_tf_available: log once when pose samples
        # start dropping and once when they recover, not per message.
        self._tf_available = None
        # The same, for the mapping fallback's own (different) lookups.
        self._lio_tf_available = None

        # When the primary feed last delivered — monotonic, because this is a
        # "has it gone quiet?" test and the wall clock can step. -inf so the
        # fallback is live from the first message in a session that has no
        # primary feed at all, rather than after one hold period.
        self._primary_seen_s = float("-inf")

    def register(self, node: Node):
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
        # The mapping-mode pose feed. Subscribed unconditionally rather than on
        # the reported mode: the backend is started by the session it belongs
        # to, so "which topics exist" already answers the question, and a
        # subscription to a topic nobody publishes costs nothing.
        node.create_subscription(
            msg_type=Odometry,
            topic="pointlio/lio_odom",
            callback=self._lio_odom_cb,
            qos_profile=qos,
        )
        node.create_subscription(
            msg_type=MotorStates,
            topic="motor_states",
            callback=self._motor_states_cb,
            qos_profile=qos,
        )

    def _odom_cb(self, msg: Odometry):
        # Recorded before the TF gate below, not after it: the claim this makes
        # is "lio_bridge is running", which is true of a dropped sample too.
        # Failing over to the LIO front end because the localizer has not been
        # relocalized would gain nothing anyway — map->pointlio_odom comes from
        # that same localizer in AUTO, so the fallback is blind in exactly the
        # window the primary is.
        self._primary_seen_s = time.monotonic()

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

    def _lio_odom_cb(self, msg: Odometry):
        """Map-frame base_link pose during a mapping run.

        The mapping session's TF tree is two disconnected halves: pgo and
        pointlio own ``map -> <ns>/pointlio_odom -> <ns>/pointlio_body``, while
        bringup's robot_state_publisher owns ``<ns>/base_link -> <ns>/lidar_top``
        and everything below it. Nothing joins them — that splice is exactly
        what lio_bridge does in AUTO, and it is not in this session — so tf2
        cannot answer map->base_link and neither can robot_state (which is why
        it reports UNINITIALIZED for the whole run).

        The join is a fact, not a calibration: the LIO body frame IS the lidar,
        so ``lidar_top -> base_link`` is the URDF mount extrinsic read backwards
        and

            map_T_base = map_T_pointlio_odom . odom_T_body . (base_T_lidar)^-1

        closes the tree. Full 3x3 algebra rather than the primary feed's planar
        shortcut, because nothing here is projected yet: the mount is tilted and
        the LIO solution is 6-DOF, so composing yaws would bake the tilt into
        the heading. Only the result is flattened, to the same (x, y, yaw) the
        console draws in AUTO — z is kept, because during mapping the cloud's
        floor sits a lidar height below the origin and dropping it would float
        the robot above its own map.
        """
        if time.monotonic() - self._primary_seen_s < PRIMARY_POSE_HOLD_S:
            return

        body_frame = msg.child_frame_id
        # Frame names are not namespaced by ROS, so the prefix is carried in the
        # frame id itself — taken from the message for the same reason the cloud
        # subscriber takes its source frame from the header: an upstream rename
        # of the LIO frames must not need an edit here.
        ns = body_frame.rsplit("/", 1)[0] + "/" if "/" in body_frame else ""
        try:
            map_tf = self._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time()
            )
            # Static, but looked up per sample: caching it would mean holding a
            # transform from before the URDF publisher was up, and this is one
            # buffer hit at 20 Hz.
            base_lidar = self._tf_buffer.lookup_transform(
                f"{ns}{BASE_FRAME}", f"{ns}{LIDAR_FRAME}", rclpy.time.Time()
            )
        except TransformException as exc:
            if self._lio_tf_available is not False:
                self._logger.warning(
                    "telemetry pose dropping: mapping TF unavailable (is pgo "
                    "broadcasting map->pointlio_odom, and bringup the URDF?)",
                    body_frame=body_frame,
                    error=str(exc),
                )
                self._lio_tf_available = False
            return

        if self._lio_tf_available is not True:
            self._logger.info("telemetry pose streaming from LIO", body_frame=body_frame)
            self._lio_tf_available = True

        map_r, map_t = _rigid(map_tf.transform.translation, map_tf.transform.rotation)
        odom_r, odom_t = _rigid(msg.pose.pose.position, msg.pose.pose.orientation)
        mount_r, mount_t = _rigid(
            base_lidar.transform.translation, base_lidar.transform.rotation
        )

        lidar_base_r = mount_r.T
        lidar_base_t = -mount_r.T @ mount_t

        rot = map_r @ odom_r @ lidar_base_r
        pos = map_t + map_r @ (odom_t + odom_r @ lidar_base_t)

        self._repo.update_pose(
            x=float(pos[0]),
            y=float(pos[1]),
            z=float(pos[2]),
            # Yaw of the composed rotation, i.e. the heading with the mount tilt
            # already taken out by the extrinsic above.
            yaw_deg=math.degrees(math.atan2(rot[1, 0], rot[0, 0])),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _motor_states_cb(self, msg: MotorStates):
        self._repo.update_joints(
            joints={m.name: float(m.q) for m in msg.states},
            # MotorStates.timestamp is nanoseconds ON THIS TOPIC (stamped from
            # now() by driver_manager); the wire format uses seconds like the
            # pose, hence the scale.
            #
            # Do not "fix" this against RobotState.motor_status.timestamp, which
            # is the same message type carrying SECONDS: syncai_robot_state
            # rescales it on the way into that aggregate. The topic deliberately
            # keeps nanoseconds because this is the high-rate joint channel and
            # whole seconds cannot order samples at 20 Hz.
            stamp=msg.timestamp * 1e-9,
        )


def init_telemetry_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    telemetry_repo: TelemetryRepo,
    tf_buffer: Buffer,
) -> TelemetrySubscriber:
    subscriber = TelemetrySubscriber(
        logger=logger, telemetry_repo=telemetry_repo, tf_buffer=tf_buffer
    )
    subscriber.register(node=node)
    return subscriber
