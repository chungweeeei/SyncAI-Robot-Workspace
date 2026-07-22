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
    """Stream the robot's live lidar scan (body_cloud) to the frontend.

    The LIO node publishes a per-scan cloud in the lidar/body frame at lidar
    rate. To overlay it on the saved map, each frame is transformed into
    ``target_frame`` (``map`` on the real robot) using the TF tree the
    localizer/pgo (``map -> odom``) and the LIO node (``odom -> body``)
    broadcast, then packed and handed to the single-slot repo the WebSocket
    router drains.

    Cost note: this is the decimated per-scan cloud (~a few thousand points),
    so the vectorised numpy transform is sub-millisecond. The large accumulated
    map cloud is deliberately never transformed here — it is served statically
    from a saved .pcd by the REST /api/v1/map/pointcloud endpoint.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        pointcloud_repo: PointCloudRepo,
    ):
        self._logger = logger
        self._pc_repo = pointcloud_repo
        self._voxel_size = 0.0
        self._max_points = 30000
        # Frame the cloud is transformed into before packing, so it overlays the
        # (also map-frame) static map cloud on the frontend.
        self._target_frame = "map"

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
                # map->odom is a slowly-varying correction; look up the latest
                # available transform (Time()) rather than the cloud stamp so a
                # high-rate cloud isn't dropped by future-extrapolation errors.
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, source_frame, rclpy.time.Time()
                )
            except TransformException as exc:
                self._logger.debug(
                    "dropping cloud frame: TF unavailable",
                    target_frame=self._target_frame,
                    source_frame=source_frame,
                    error=str(exc),
                )
                return

            t = tf.transform.translation
            q = tf.transform.rotation
            points = transform_points(
                points=points,
                translation=np.array([t.x, t.y, t.z]),
                quat_xyzw=np.array([q.x, q.y, q.z, q.w]),
            )

        self._pc_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )


def init_pointcloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    pointcloud_repo: PointCloudRepo,
) -> PointCloudSubscriber:
    subscriber = PointCloudSubscriber(logger=logger, pointcloud_repo=pointcloud_repo)
    subscriber.register(node=node)
    return subscriber
