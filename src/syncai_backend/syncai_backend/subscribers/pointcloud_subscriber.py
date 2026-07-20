import numpy as np
import structlog

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformException

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import (
    voxel_downsample,
    cap_points,
    transform_points,
    pack_xyz_f32,
)


class PointCloudSubscriber:
    """Subscribes to the LIO ``body_cloud``, transforms it into the map frame
    and caches the latest packed frame for the WebSocket streamer.

    Defaults target the real-Livox pointlio estimator: it publishes
    ``/pointlio/body_cloud`` in the ``body`` frame and broadcasts TF
    ``lidar -> body``; the localizer adds ``map -> lidar`` once relocalized, so
    the full chain is ``map -> lidar -> body``. We look up ``map -> <frame_id>``
    and apply it here so the browser receives map-frame points it can drop
    straight onto the (also map-frame) grid floor without doing any TF math.

    For the fastlio2/Isaac path set ``pointcloud.topic`` to
    ``fastlio2/body_cloud`` (frame ``<robot_id>/lio_body``); for a raw render
    test with no saved map set ``pointcloud.target_frame`` to the LIO odom
    frame (``lidar`` for pointlio) so only the estimator's own TF is needed.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        pc_repo: PointCloudRepo,
        tf_buffer: Buffer,
    ):
        self._logger = logger
        self._pc_repo = pc_repo
        self._tf_buffer = tf_buffer

    def register(self, node: Node):
        self._target_frame = (
            node.declare_parameter("pointcloud.target_frame", "map")
            .get_parameter_value()
            .string_value
        )
        # Absolute name: pointlio runs under its own /pointlio namespace, not
        # this node's /<robot_id> namespace.
        self._topic = (
            node.declare_parameter("pointcloud.topic", "/pointlio/body_cloud")
            .get_parameter_value()
            .string_value
        )
        # body_cloud is already voxel-filtered (~0.15 m) by LIO, so extra voxel
        # downsampling is off by default; the point cap bounds bandwidth.
        self._voxel_size = (
            node.declare_parameter("pointcloud.voxel_size", 0.0)
            .get_parameter_value()
            .double_value
        )
        self._max_points = (
            node.declare_parameter("pointcloud.max_points", 30000)
            .get_parameter_value()
            .integer_value
        )

        # SensorDataQoS (BEST_EFFORT) matches the lidar/LIO publisher; the
        # subscription also switches body_cloud's lazy publisher on.
        node.create_subscription(
            msg_type=PointCloud2,
            topic=self._topic,
            callback=self._cloud_cb,
            qos_profile=qos_profile_sensor_data,
        )

    def _cloud_cb(self, msg: PointCloud2):
        # Non-blocking lookup of the newest available transform. On a
        # single-threaded executor we must not wait here (it would starve the
        # TF listener on the same thread); just drop the frame if TF isn't
        # ready yet.
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except TransformException as exc:
            self._logger.debug(
                "point cloud dropped: no transform",
                target=self._target_frame,
                source=msg.header.frame_id,
                error=str(exc),
            )
            return

        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if points.shape[0] == 0:
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        points = transform_points(
            points.astype(np.float64),
            translation=np.array([t.x, t.y, t.z]),
            quat_xyzw=np.array([q.x, q.y, q.z, q.w]),
        )

        if self._voxel_size > 0.0:
            points = voxel_downsample(points, self._voxel_size)
        points = cap_points(points, self._max_points)

        self._pc_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )


def init_pointcloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    pc_repo: PointCloudRepo,
    tf_buffer: Buffer,
) -> PointCloudSubscriber:
    subscriber = PointCloudSubscriber(
        logger=logger, pc_repo=pc_repo, tf_buffer=tf_buffer
    )
    subscriber.register(node=node)
    return subscriber
