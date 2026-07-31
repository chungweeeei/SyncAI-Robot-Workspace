import numpy as np
import rclpy
import structlog
import yaml

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import LookupException, TransformException
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
       broadcast — the source frame (and its TF parent, see
       ``_resolve_fixed_frame``) is taken from the cloud header, so a frame
       rename upstream needs no change here. That last part holds only as long
       as the cloud's frame has exactly ONE TF parent: it briefly did not (the
       LIO launches named the body frame ``base_link``, which syncai_lio_bridge
       already parents to ``odom``), and the resulting ambiguity is what the
       lookup-time comment in ``_cloud_cb`` is about. The result is packed and
       handed to the single-slot repo the WS router drains. This is the decimated
       per-scan cloud (~a few thousand points), so the vectorised numpy
       transform is sub-millisecond.

    The localizer's accumulated ``localizer/map_cloud`` used to be cached here
    too, for a REST endpoint. That endpoint reads the saved ``map/<name>/map.pcd``
    directly now — it can answer for any stored map, not just the loaded one —
    so this subscriber is down to the one live topic.
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
        # Frame the live cloud is transformed into before packing, so it
        # overlays the (also map-frame) stored map cloud the REST catalogue
        # serves from map/<name>/map.pcd.
        self._target_frame = "map"

        # Edge-triggered logging for the body_cloud TF lookup: None until the
        # first frame, then True/False. Lets us log once when the stream starts
        # dropping (map->pointlio_odom missing, i.e. not relocalized) and once
        # when it recovers, instead of a silent per-frame debug that hides why
        # the viewer is empty.
        self._cloud_tf_available = None

        # Cache for _resolve_fixed_frame: {cloud source frame: its TF parent}.
        # Keyed by source frame so an upstream frame rename resolves afresh
        # rather than reusing the old tree's answer.
        self._fixed_frames: dict[str, str] = {}

        # register tf buffer
        self._tf_buffer: Buffer = None
        self._tf_listener: TransformListener = None

    def _resolve_fixed_frame(self, source_frame: str) -> str:
        """The TF parent of ``source_frame`` — where _cloud_cb splits the chain.

        Read out of the live TF tree instead of being configured, so this stays
        as frame-rename-proof as taking the source frame from the cloud header:
        whatever pointlio calls its odom frame (``pointlio_odom`` today,
        ``lio_odom`` in the Isaac launches) is picked up automatically. Resolved
        once per source frame — the tree's shape does not change at runtime,
        only the transforms in it.

        Raises LookupException (a TransformException, so _cloud_cb's existing
        handler drops the frame and logs) until the parent is in the buffer.
        """
        cached = self._fixed_frames.get(source_frame)
        if cached is not None:
            return cached

        # all_frames_as_yaml() is tf2's own tree dump; each entry carries the
        # frame's parent. There is no public per-frame parent accessor on the
        # Python Buffer (the C++ _getParent binding is not exposed), so this is
        # the supported way to ask.
        frames = yaml.safe_load(self._tf_buffer.all_frames_as_yaml()) or {}
        entry = frames.get(source_frame) if isinstance(frames, dict) else None
        parent = entry.get("parent") if isinstance(entry, dict) else None
        if not parent:
            raise LookupException(
                f'cloud frame "{source_frame}" has no TF parent yet '
                "(is pointlio broadcasting?)"
            )

        self._fixed_frames[source_frame] = parent
        self._logger.info(
            "resolved body_cloud TF split point",
            source_frame=source_frame,
            fixed_frame=parent,
        )
        return parent

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
                # The two halves of map->body need DIFFERENT lookup times, which
                # is why this is lookup_transform_full and not a plain
                # lookup_transform. Both single-time variants are wrong:
                #
                #  - At the cloud stamp: ALWAYS fails with "extrapolation into
                #    the future". The localizer stamps map->pointlio_odom with
                #    the stamp of the last cloud its own message_filters sync
                #    delivered, and only refreshes it on the next timer tick —
                #    which is necessarily after we have already handled that
                #    cloud. Measured on robot01: pointlio_odom->pointlio_body
                #    lands at the cloud stamp exactly (pointlio broadcasts it
                #    with the scan, same cloud_end_time), map->pointlio_odom a
                #    full 10 Hz cycle (~100 ms) behind it. 100/100 frames
                #    dropped, i.e. an empty viewer.
                #  - At Time() (latest) for the whole chain: places every scan
                #    at the robot's CURRENT pose, so a moving robot's cloud
                #    smears. It also silently resolved through
                #    syncai_lio_bridge's 2D-projected chain back when base_link
                #    had two TF parents, delivering a yaw-only pose at z == 0
                #    (~15 deg of pitch off what rviz2 drew for the same cloud).
                #
                # So split at the odom frame and take each half at the time it
                # is actually valid:
                #   map <- pointlio_odom  @ Time()          — the ICP correction,
                #     genuinely slowly varying, and the only lagging link.
                #   pointlio_odom <- body @ msg.header.stamp — the LIO pose for
                #     THIS scan, published with it, so always available.
                #
                # The timeout is not about the lagging correction — it covers a
                # delivery race inside THIS process. pointlio broadcasts
                # odom->body before publishing the scan, but they are separate
                # topics, so the executor can hand us the cloud before the /tf
                # callback has filed the matching transform; without a timeout
                # ~20% of frames lost that race and were dropped. Waiting half a
                # scan period lets the transform land. Safe to block here: the
                # cloud callback has its own MutuallyExclusiveCallbackGroup, so
                # the /tf subscription (the node's default group) is still
                # serviced by another thread of the MultiThreadedExecutor.
                fixed_frame = self._resolve_fixed_frame(source_frame)
                tf = self._tf_buffer.lookup_transform_full(
                    target_frame=self._target_frame,
                    target_time=rclpy.time.Time(),
                    source_frame=source_frame,
                    source_time=rclpy.time.Time.from_msg(msg.header.stamp),
                    fixed_frame=fixed_frame,
                    timeout=rclpy.duration.Duration(seconds=0.05),
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


def init_pointcloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    pointcloud_repo: PointCloudRepo,
) -> PointCloudSubscriber:
    subscriber = PointCloudSubscriber(
        logger=logger,
        pointcloud_repo=pointcloud_repo,
    )
    subscriber.register(node=node)
    return subscriber
