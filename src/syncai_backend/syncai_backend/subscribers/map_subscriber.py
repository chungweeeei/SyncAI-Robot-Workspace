import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import OccupancyGrid

from syncai_backend.repositories.map.map import MapRepo


class MapSubscriber:
    def __init__(self, logger: structlog.stdlib.BoundLogger, map_repo: MapRepo):
        self._logger = logger
        self._map_repo = map_repo

    def register(self, node: Node):

        # QoS must match the map_server's latched publisher (TRANSIENT_LOCAL +
        # RELIABLE, KeepLast(1)); otherwise this late-joining subscriber never
        # receives the single retained OccupancyGrid.
        self._map_sub = node.create_subscription(
            msg_type=OccupancyGrid,
            topic="map",
            callback=self._map_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _map_cb(self, msg: OccupancyGrid):
        self._map_repo.update_map(grid=msg)


def init_map_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, map_repo: MapRepo
) -> MapSubscriber:
    map_subscriber = MapSubscriber(logger=logger, map_repo=map_repo)
    map_subscriber.register(node=node)
