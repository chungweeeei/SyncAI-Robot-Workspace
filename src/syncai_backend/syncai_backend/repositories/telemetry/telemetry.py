import threading
import structlog

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class PoseSample:
    """Map-frame robot pose ready to ship over the telemetry WebSocket.

    ``seq`` is a monotonic counter the WebSocket loop uses to skip samples it
    has already sent (same convention as PointCloudFrame). ``yaw_deg`` is in
    degrees to match the vocabulary of the frontend's RobotPose.theta.
    ``stamp`` is seconds since epoch, from the odom message header.
    """

    seq: int
    x: float
    y: float
    z: float
    yaw_deg: float
    stamp: float


@dataclass(frozen=True)
class JointsSample:
    """Joint positions keyed by URDF joint name (MotorState.name), radians."""

    seq: int
    joints: Dict[str, float]
    stamp: float


class TelemetryRepo:
    """In-process single-slot caches for the internal telemetry stream.

    Pose and joints are cached (and sequence-numbered) independently because
    they arrive from different topics at different rates — a joints frame must
    not force a resend of an unchanged pose and vice versa. Only the newest
    sample of each is retained: a slow WebSocket client drops stale samples
    instead of queuing them (same convention as PointCloudRepo).

    The ROS subscriber thread writes while the uvicorn/asyncio thread reads,
    so access is guarded by a lock (same convention as RobotRepo).
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        self._lock = threading.Lock()
        self._pose_seq = 0
        self._pose: Optional[PoseSample] = None
        self._joints_seq = 0
        self._joints: Optional[JointsSample] = None

    def update_pose(self, x: float, y: float, z: float, yaw_deg: float, stamp: float):
        with self._lock:
            self._pose_seq += 1
            self._pose = PoseSample(
                seq=self._pose_seq, x=x, y=y, z=z, yaw_deg=yaw_deg, stamp=stamp
            )

    def update_joints(self, joints: Dict[str, float], stamp: float):
        with self._lock:
            self._joints_seq += 1
            self._joints = JointsSample(
                seq=self._joints_seq, joints=joints, stamp=stamp
            )

    def get_pose(self, after_seq: int = 0) -> Optional[PoseSample]:
        """Return the cached pose if newer than ``after_seq``, else None."""
        with self._lock:
            if self._pose is None or self._pose.seq <= after_seq:
                return None
            return self._pose

    def get_joints(self, after_seq: int = 0) -> Optional[JointsSample]:
        """Return the cached joints if newer than ``after_seq``, else None."""
        with self._lock:
            if self._joints is None or self._joints.seq <= after_seq:
                return None
            return self._joints


def init_telemetry_repo(logger: structlog.stdlib.BoundLogger) -> TelemetryRepo:
    return TelemetryRepo(logger)
