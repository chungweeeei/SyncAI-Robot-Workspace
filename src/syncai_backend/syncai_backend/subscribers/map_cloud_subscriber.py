import rclpy
import structlog

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import cap_points, pack_xyz_f32


class MapCloudSubscriber:
    """Feed the console's "map so far" layer from pgo's merged keyframe cloud.

    ``pgo/map_cloud`` (relative, so ``/<robot_id>/pgo/map_cloud``) exists only
    while a mapping (MANUAL) session is up: pgo re-merges every keyframe with
    its *current* loop-closure-corrected pose and publishes at most every few
    seconds, subscriber-gated — this subscription is what un-gates it.

    Deliberately NOT a copy of PointCloudSubscriber's pipeline:

    * **No TF.** The cloud arrives already in the ``map`` frame — every point
      was placed with the keyframes' corrected global poses at merge time.
      Transforming it again would be a no-op bought with a lookup that can
      fail.
    * **No voxel_downsample.** pgo already voxelised at its publish resolution
      (``map_cloud_resolution``); re-voxelising ~half a million points through
      ``np.unique(axis=0)`` every few seconds is O(N log N) of pure waste.
      ``cap_points`` (a stride) stays as the one wire-size guard.

    The single-slot repo semantics fit exactly: each message is a complete
    replacement (a loop closure moves the *whole* map, so deltas are
    impossible), and the accumulated state resets for free because the backend
    process is restarted with every mapping session.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_cloud_repo: PointCloudRepo,
    ):
        self._logger = logger
        self._map_cloud_repo = map_cloud_repo
        # ~6 MB per frame at the cap. pgo's 0.2 m voxel keeps real sites well
        # under this; the cap is for a misconfigured resolution, not a budget.
        self._max_points = 500000

        # Edge-triggered: log once when the first merge lands (the "mapping is
        # visibly producing a map" moment), not per multi-MB frame.
        self._streaming = False

    def register(self, node: Node):
        # Own MutuallyExclusive group for the same reason as the live-cloud
        # subscriber: parsing a multi-MB merge must not starve the 10 Hz
        # body_cloud callback or the state/TF callbacks on the executor.
        #
        # Depth 1 where the live cloud uses 5: each message replaces the map
        # wholesale, so a queued older merge is never worth delivering.
        node.create_subscription(
            msg_type=PointCloud2,
            topic="pgo/map_cloud",
            callback=self._cloud_cb,
            qos_profile=QoSProfile(
                depth=1,
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

        points = cap_points(points=points, max_points=self._max_points)

        if not self._streaming:
            self._streaming = True
            self._logger.info(
                "map cloud streaming",
                num_points=int(points.shape[0]),
                frame=msg.header.frame_id,
            )

        self._map_cloud_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )


def init_map_cloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    map_cloud_repo: PointCloudRepo,
) -> MapCloudSubscriber:
    map_cloud_subscriber = MapCloudSubscriber(
        logger=logger, map_cloud_repo=map_cloud_repo
    )
    map_cloud_subscriber.register(node=node)
    return map_cloud_subscriber
