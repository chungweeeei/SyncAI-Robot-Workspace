import numpy as np
import rclpy
import structlog

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    transform_points,
    voxel_downsample,
)


class PointCloudSubscriber:
    """Feed the frontend 3D viewer two clouds from ROS.

    1. Live body_cloud: the LIO node publishes a per-scan cloud in the
       LIO body frame (``<robot_id>/pointlio_body``) at lidar rate. To overlay
       it on the map, each frame is transformed into ``target_frame`` (``map``)
       using the TF branch the localizer (``map -> <robot_id>/pointlio_odom``,
       after relocalize) and the LIO node (``pointlio_odom -> pointlio_body``)
       broadcast — the source frame is taken from the cloud header, so a frame
       rename upstream needs no change here. The result is packed and handed to
       the single-slot repo the WebSocket router drains. This is the decimated
       per-scan cloud (~a few thousand points), so the vectorised numpy
       transform is sub-millisecond.

    2. Static map cloud: the localizer publishes the accumulated map cloud on
       ``localizer/map_cloud`` once (latched, transient_local) in the map frame
       after it loads a map. It is decimated once on receipt and cached in a
       second single-slot repo the REST /api/v1/map/pointcloud endpoint serves.
       No TF transform is needed — it is already in the map frame.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        pointcloud_repo: PointCloudRepo,
        map_cloud_repo: PointCloudRepo,
    ):
        self._logger = logger
        self._pc_repo = pointcloud_repo
        self._map_cloud_repo = map_cloud_repo
        self._voxel_size = 0.0
        self._max_points = 30000
        # The accumulated map cloud is large and static, so it is decimated
        # harder than the live scan and only once, on receipt.
        self._map_voxel_size = 0.3
        self._map_max_points = 300000
        # Frame the live cloud is transformed into before packing, so it
        # overlays the (also map-frame) static map cloud on the frontend.
        self._target_frame = "map"

        # Edge-triggered logging for the body_cloud TF lookup: None until the
        # first frame, then True/False. Lets us log once when the stream starts
        # dropping (map->pointlio_odom missing, i.e. not relocalized) and once
        # when it recovers, instead of a silent per-frame debug that hides why
        # the viewer is empty.
        self._cloud_tf_available = None

        # register tf buffer
        self._tf_buffer: Buffer = None
        self._tf_listener: TransformListener = None

    def register(self, node: Node):
        # spin_thread=False: the listener's /tf(/tf_static) subscriptions run on
        # the node's own executor, so it never spawns an extra GIL-contending
        # thread. The cloud callback lives in its own MutuallyExclusive group so
        # a busy scan frame can't starve the robot_state/map/TF callbacks (the
        # node is spun by a MultiThreadedExecutor in main.py).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node, spin_thread=False)

        node.create_subscription(
            msg_type=PointCloud2,
            topic="pointlio/body_cloud",
            callback=self._cloud_cb,
            qos_profile=QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Static map cloud from the localizer. QoS MUST match the publisher
        # (rclcpp::QoS(1).transient_local(), i.e. RELIABLE + TRANSIENT_LOCAL) or
        # DDS won't replay the latched sample to this late-joining subscriber.
        node.create_subscription(
            msg_type=PointCloud2,
            topic="localizer/map_cloud",
            callback=self._map_cloud_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _cloud_cb(self, msg: PointCloud2):

        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )

        if points.shape[0] == 0:
            return

        # Thin BEFORE transforming so the rigid transform touches the fewest
        # points. voxel_downsample keeps shape (skipped when voxel_size <= 0),
        # cap_points bounds the frame the client has to render.
        points = voxel_downsample(points=points, voxel_size=self._voxel_size)
        points = cap_points(points=points, max_points=self._max_points)

        source_frame = msg.header.frame_id
        if source_frame != self._target_frame:
            try:
                # map->pointlio_odom is a slowly-varying correction; look up the
                # latest available transform (Time()) rather than the cloud
                # stamp so a high-rate cloud isn't dropped by
                # future-extrapolation errors.
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, source_frame, rclpy.time.Time()
                )
            except TransformException as exc:
                if self._cloud_tf_available is not False:
                    self._logger.warning(
                        "body_cloud frames dropping: TF unavailable "
                        "(relocalized yet? map->pointlio_odom comes from the "
                        "localizer only after /localizer/relocalize)",
                        target_frame=self._target_frame,
                        source_frame=source_frame,
                        error=str(exc),
                    )
                    self._cloud_tf_available = False
                return

            t = tf.transform.translation
            q = tf.transform.rotation
            points = transform_points(
                points=points,
                translation=np.array([t.x, t.y, t.z]),
                quat_xyzw=np.array([q.x, q.y, q.z, q.w]),
            )

        if self._cloud_tf_available is not True:
            self._logger.info(
                "body_cloud streaming to frontend",
                target_frame=self._target_frame,
                source_frame=source_frame,
            )
            self._cloud_tf_available = True

        self._pc_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )

    def _map_cloud_cb(self, msg: PointCloud2):
        # Already in the map frame (localizer publishes with map_frame), so no
        # TF transform — just thin and cache. Fires rarely (once per map load).
        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if points.shape[0] == 0:
            return

        points = voxel_downsample(points=points, voxel_size=self._map_voxel_size)
        points = cap_points(points=points, max_points=self._map_max_points)

        self._map_cloud_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )
        self._logger.info("cached localizer map cloud", num_points=int(points.shape[0]))


def init_pointcloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    pointcloud_repo: PointCloudRepo,
    map_cloud_repo: PointCloudRepo,
) -> PointCloudSubscriber:
    subscriber = PointCloudSubscriber(
        logger=logger,
        pointcloud_repo=pointcloud_repo,
        map_cloud_repo=map_cloud_repo,
    )
    subscriber.register(node=node)
    return subscriber
