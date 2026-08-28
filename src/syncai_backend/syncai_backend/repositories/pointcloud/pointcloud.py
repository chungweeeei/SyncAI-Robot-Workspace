import asyncio
import contextlib
import threading
import structlog

from dataclasses import dataclass
from typing import Iterator, Optional


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

    Writers also **wake** readers (``subscribe``), rather than leaving them to
    poll. The WS pump used to sample this slot on a fixed 100 ms timer, which
    cost two things measured on robot01 against the 10 Hz body_cloud: a mean
    ~50 ms of pure queueing latency, and ~5% of frames dropped outright — two
    unsynchronised 10 Hz clocks beat against each other, so every so often a
    tick found nothing new and the frame that landed just after it waited a
    full extra period and was then superseded. The observed stream ran at
    9.5 Hz with a 205 ms worst-case gap for a producer that never missed a
    10 Hz beat. Notifying costs one ``call_soon_threadsafe`` per frame.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        self._lock = threading.Lock()
        self._seq = 0
        self._frame: Optional[PointCloudFrame] = None
        # (loop, event) per live WS pump. A list, not a single Event: two
        # dashboards on one robot are two independent readers, and one of them
        # clearing a shared Event would swallow the other's wakeup.
        self._waiters: list[tuple[asyncio.AbstractEventLoop, asyncio.Event]] = []

    def update_frame(self, num_points: int, data: bytes):
        with self._lock:
            self._seq += 1
            self._frame = PointCloudFrame(
                seq=self._seq, num_points=num_points, data=data
            )
            waiters = list(self._waiters)

        # Outside the lock: this runs on the ROS callback thread, and the lock
        # is also taken by the asyncio thread on every read. Nothing here can
        # block, but keeping a cross-thread hand-off out of the critical
        # section keeps the subscriber's cost independent of the reader count.
        for loop, event in waiters:
            try:
                loop.call_soon_threadsafe(event.set)
            except RuntimeError:
                # The reader's loop closed between our snapshot and here (the
                # backend is shutting down, or a client disconnected mid-frame).
                # Its `subscribe` finally-block will unregister it.
                pass

    @contextlib.contextmanager
    def subscribe(self) -> Iterator[asyncio.Event]:
        """Register the calling coroutine's loop for frame notifications.

        Yields an Event that ``update_frame`` sets. Must be entered from the
        loop that will await it — the loop handle is captured here so the
        writer, on a ROS thread, has something to hand the callback to.
        """
        loop = asyncio.get_running_loop()
        entry = (loop, asyncio.Event())
        with self._lock:
            self._waiters.append(entry)
        try:
            yield entry[1]
        finally:
            with self._lock:
                # discard-not-remove: a double exit must not raise out of a
                # WebSocket teardown path.
                if entry in self._waiters:
                    self._waiters.remove(entry)

    def get_latest(self, after_seq: int = 0) -> Optional[PointCloudFrame]:
        with self._lock:
            if self._frame is None:
                return None
            if self._frame.seq <= after_seq:
                return None
            return self._frame


def init_pointcloud_repo(logger: structlog.stdlib.BoundLogger) -> PointCloudRepo:
    return PointCloudRepo(logger)
