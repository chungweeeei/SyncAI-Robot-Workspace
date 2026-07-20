import threading
import structlog

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PointCloudFrame:
    """A single packed point-cloud frame ready to ship over the wire.

    ``data`` is little-endian float32 xyz triplets (3 * num_points values) in
    the map frame. ``seq`` is a monotonic counter used by the WebSocket loop to
    detect (and skip) frames it has already sent.
    """

    seq: int
    num_points: int
    data: bytes


class PointCloudRepo:
    """In-process single-slot cache holding only the latest point-cloud frame.

    The ROS subscriber thread writes frames while the uvicorn/asyncio thread
    reads them, so access is guarded by a lock (same convention as RobotRepo).
    Only the newest frame is retained: a slow WebSocket client naturally drops
    stale frames instead of queuing them.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        self._lock = threading.Lock()
        self._seq = 0
        self._frame: Optional[PointCloudFrame] = None

    def update_frame(self, num_points: int, data: bytes):
        with self._lock:
            self._seq += 1
            self._frame = PointCloudFrame(
                seq=self._seq, num_points=num_points, data=data
            )

    def get_latest(self, after_seq: int = 0) -> Optional[PointCloudFrame]:
        """Return the cached frame if it is newer than ``after_seq``, else None."""
        with self._lock:
            if self._frame is None or self._frame.seq <= after_seq:
                return None
            return self._frame


def init_pointcloud_repo(logger: structlog.stdlib.BoundLogger) -> PointCloudRepo:
    return PointCloudRepo(logger)
