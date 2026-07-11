import threading
import structlog

from typing import Optional

from nav_msgs.msg import OccupancyGrid


class MapRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        # In-Process memory cache for the latest map (OccupancyGrid).
        self._map_lock = threading.Lock()
        self._map: Optional[OccupancyGrid] = None

    def update_map(self, grid: OccupancyGrid):
        with self._map_lock:
            self._map = grid

    def get_map(self) -> Optional[OccupancyGrid]:
        with self._map_lock:
            return self._map


def init_map_repo(logger: structlog.stdlib.BoundLogger) -> MapRepo:
    map_repo = MapRepo(logger)
    return map_repo
