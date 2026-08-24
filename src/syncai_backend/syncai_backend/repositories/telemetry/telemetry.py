import threading
import time
import structlog

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


# How long a planned route stays live without being republished.
#
# Nothing on the ROS side ever publishes an empty plan: syncai_planner publishes
# only on a successful ComputePathToPose, and the BT's empty-path-on-abort is
# written to the blackboard, never to the topic. So arriving, cancelling and
# aborting all look identical from here — the plans simply stop coming. Expiring
# on a timer is what turns that silence into a clear, and it covers every one of
# those endings without the backend having to work out which one happened.
#
# 6 s is 2x the BT's replan period (RateController hz="0.333" in
# syncai_task_runner/behavior_trees/move.xml). Tighter, and one slow plan blinks
# the route out mid-run; looser, and a finished route lingers on the operator's
# map. This is the number to tune if either shows up.
_PATH_TTL_S = 6.0


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


@dataclass(frozen=True)
class PathSample:
    """The planner's global route, map-frame metres, downsampled for the wire.

    ``points`` is a tuple of (x, y) pairs. Heading is deliberately dropped: the
    viewport draws the route as a band on the floor, not as a series of poses,
    so the per-pose orientation NavFn fills in would be paid for on every frame
    and never read.

    An empty ``points`` is the explicit "no route" sample rather than the absence
    of one — see ``TelemetryRepo.get_path`` for why the difference matters.
    """

    seq: int
    points: Tuple[Tuple[float, float], ...]
    stamp: float


class TelemetryRepo:
    """In-process single-slot caches for the internal telemetry stream.

    Pose, joints and the planned path are cached (and sequence-numbered)
    independently because they arrive from different topics at wildly different
    rates — a joints frame must not force a resend of an unchanged pose, and the
    path changes about once every 3 s against pose's 20 Hz. Only the newest
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
        self._path_seq = 0
        self._path: Optional[PathSample] = None
        # Monotonic, not the message stamp: this drives the TTL below, which is
        # about how long ago we *heard* a plan, and must not be perturbed by a
        # clock step on the publisher's side.
        self._path_recv_at = 0.0

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

    def update_path(self, points: Tuple[Tuple[float, float], ...], stamp: float):
        with self._lock:
            self._path_seq += 1
            self._path = PathSample(seq=self._path_seq, points=points, stamp=stamp)
            self._path_recv_at = time.monotonic()

    def get_pose(self, after_seq: int = 0) -> Optional[PoseSample]:
        """Return the cached pose if newer than ``after_seq``, else None."""
        with self._lock:
            if self._pose is None:
                return None
            if self._pose.seq <= after_seq:
                return None
            return self._pose

    def get_joints(self, after_seq: int = 0) -> Optional[JointsSample]:
        """Return the cached joints if newer than ``after_seq``, else None."""
        with self._lock:
            if self._joints is None:
                return None
            if self._joints.seq <= after_seq:
                return None
            return self._joints

    def get_path(self, after_seq: int = 0) -> Optional[PathSample]:
        """Return the cached path if newer than ``after_seq``, else None.

        Unlike the two getters above this one can *change* the cache: a route
        that has not been republished within _PATH_TTL_S is expired in place.
        Doing it on read rather than from a timer keeps the whole policy in one
        place — the alternative, letting each WebSocket connection decide when a
        route has gone stale, copies the same rule into every client.
        """
        with self._lock:
            if self._path is None:
                return None
            if (
                self._path.points
                and time.monotonic() - self._path_recv_at > _PATH_TTL_S
            ):
                # Expire in place and bump seq, so every connected client gets
                # exactly one empty sample and then goes quiet. Idempotent: once
                # points is empty this branch can no longer be taken.
                self._path_seq += 1
                self._path = PathSample(
                    seq=self._path_seq, points=(), stamp=self._path.stamp
                )
            if self._path.seq <= after_seq:
                return None
            return self._path


def init_telemetry_repo(logger: structlog.stdlib.BoundLogger) -> TelemetryRepo:
    return TelemetryRepo(logger)
