import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, NamedTuple, Optional, Tuple

import structlog


# The LIO inputs, and the reason this API exists at all. pgo accumulates its
# keyframes in RAM and ``save_maps`` is the only thing that serialises them, so
# a mapping run that ends without a save is gone -- and the only remedy after
# the fact is replaying these two topics into the LIO chain. They are the pair
# the (since-deleted) doc/record-lidar.md procedure recorded; recover its full
# text with `git show a1804af^:doc/record-lidar.md`.
#
# Relative, like every topic name in this workspace: see _resolve_topics.
DEFAULT_TOPICS = ("livox/lidar", "livox/imu")

# One split, and the reason the number is not a tuning knob: 2 GB is the size
# rosbag2 is told to cut a file at, so a start with less than that free cannot
# even finish the file it is opening.
#
# Refusing is the recoverable direction. Filling the partition takes postgres,
# the byobu multilogs and the running stack with it, and the operator's remedy
# for the refusal -- deleting an old recording -- is a route this same router
# serves.
MIN_FREE_BYTES = 2 * 1024 * 1024 * 1024

# What each split is cut at. The old hand-pasted procedure's value, kept
# because it is also what makes a long run copyable off the robot in pieces.
MAX_BAG_SIZE_BYTES = 2000000000

# Reserved: GET /api/v1/recordings/active would otherwise shadow a recording
# directory legitimately called that.
RESERVED_NAMES = frozenset({"active"})

# The stop ladder. SIGINT first because it is the only signal rosbag2 handles
# as "finish and flush": a bag killed any other way has no metadata.yaml and is
# unplayable until `ros2 bag reindex`. The rest is MdnsManager.kill_mdns's
# terminate -> kill escalation, with a budget generous enough for the writer to
# close a 2 GB sqlite file.
_SIGINT_TIMEOUT = 15.0
_SIGTERM_TIMEOUT = 5.0
_SIGKILL_TIMEOUT = 2.0

# How long a freshly spawned recorder has to prove it is still alive. Same
# liveness probe as MdnsManager.publish_mdns: a bad topic name, an unwritable
# directory or a missing `ros2` on PATH all kill the child within milliseconds,
# and catching that here is what turns it into the POST's own error instead of
# a 201 followed by a bag that never appears.
_LIVENESS_PROBE = 0.5


class _Active(NamedTuple):
    """The one recording this backend is running."""

    name: str
    path: str
    topics: List[str]
    started_at: datetime
    started_monotonic: float
    compression: bool
    process: subprocess.Popen


class RecordingGateway:
    """``ros2 bag record`` as a supervised child process.

    A subprocess rather than ``rosbag2_py`` in this process, and that is a
    choice rather than a constraint -- both work in this image. A recorder
    subscribing to the 20 Hz lidar cloud would otherwise share the backend's
    MultiThreadedExecutor with the telemetry, point-cloud and TF callbacks, and
    hold the GIL against uvicorn's threadpool and the Temporal worker while it
    serialises every message. Out of process it competes for CPU and nothing
    else, and a recorder that wedges or dies takes nothing down with it.

    Two properties of the child are load-bearing:

      - It is **not** put in its own session (no ``start_new_session``). A
        ``switch_mode`` kills the byobu session this backend is a pane of; a
        recorder in its own session would survive that, keep writing, and be
        unstoppable through any API, because the only handle on it is the slot
        in this object. Sharing the process group means the teardown takes it
        too, and the half-written directory reports itself as ``interrupted``.
      - Its stdout and stderr are **inherited**, not piped. A pipe nobody drains
        deadlocks the recorder at 64 KiB, and inherited they land in the
        backend's byobu multilog next to everything else. (MdnsManager sends
        avahi-publish to DEVNULL; that process has nothing to say and rosbag2
        does.)

    One recording at a time. The slot is claimed under the lock before the
    spawn, so there is no check-then-start race -- the same shape as map.py's
    _ACTIVE_CONVERSIONS and WebRtcGateway's session slots.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_id: str,
        records_dir: str,
    ):
        self._logger = logger
        self._robot_id = robot_id
        self._records_dir = records_dir

        # Never held across the wait in stop(): status() is the route a console
        # polls at 1 Hz and must stay answerable while a 2 GB file is being
        # closed. The lock covers the slot, not the child.
        self._lock = threading.Lock()
        self._active: Optional[_Active] = None

    # --- Helpers ------------------------------------------------------------

    def _resolve_topics(self, topics: List[str]) -> List[str]:
        """Expand relative topic names under this robot's namespace.

        ``livox/lidar`` becomes ``/robot01/livox/lidar``; an absolute name is
        passed through untouched. This is the workspace's namespacing rule
        (CLAUDE.md, "The robot_id convention") pushed out to the API edge: the
        caller writes what the params YAML and every subscriber write, and the
        same request body records the right thing on any robot in the fleet.

        The escape hatch matters as much as the rule: ``/tf``, ``/tf_static``
        and ``/rosout`` are genuinely fleet-wide, and a bag meant for replay
        usually wants them.
        """
        resolved = []
        for topic in topics:
            stripped = topic.strip()
            if stripped.startswith("/"):
                resolved.append(stripped)
            else:
                resolved.append(f"/{self._robot_id}/{stripped}")
        return resolved

    def _reap(self) -> None:
        """Drop the slot if the child is gone. Caller must hold the lock.

        A recorder can die on its own -- the disk filled, the sqlite writer
        failed, somebody killed the pane. Without this the slot would stay
        claimed and every later start would answer 409 against a process that
        does not exist, a wedge no log line explains.
        """
        active = self._active
        if active is None or active.process.poll() is None:
            return

        self._logger.warning(
            "[RecordingGateway] The recorder exited on its own",
            recording=active.name,
            returncode=active.process.returncode,
        )
        self._active = None

    def _describe(self, active: _Active) -> Dict:
        return {
            "name": active.name,
            "path": active.path,
            "topics": list(active.topics),
            "started_at": active.started_at,
            "elapsed_seconds": time.monotonic() - active.started_monotonic,
            "compression": active.compression,
            "size_bytes": _dir_size(active.path),
        }

    # --- API ----------------------------------------------------------------

    def active_name(self) -> Optional[str]:
        """The live recording's name, or None. Cheap: no disk walk."""
        with self._lock:
            self._reap()
            return self._active.name if self._active is not None else None

    def status(self) -> Optional[Dict]:
        """What is being recorded right now, or None."""
        with self._lock:
            self._reap()
            active = self._active
        # Outside the lock: _describe walks the bag directory, and the stop path
        # must not queue behind a poll.
        return self._describe(active) if active is not None else None

    def start(
        self, name: str, topics: List[str], compression: bool
    ) -> Tuple[bool, str, Dict]:
        """Spawn a recorder. Returns (success, message, payload).

        ``name`` and the free-space floor are checked under the same lock that
        claims the slot, so a second request cannot slip between the check and
        the spawn. The directory itself is not created here -- ``-o`` is the
        recorder's own argument and it refuses an existing path, which is the
        backstop behind the name check the router already did.
        """
        resolved = self._resolve_topics(topics)
        path = os.path.join(self._records_dir, name)

        with self._lock:
            self._reap()
            if self._active is not None:
                return (
                    False,
                    f"already recording '{self._active.name}'",
                    {"name": self._active.name},
                )

            free = _free_bytes(self._records_dir)
            if free < MIN_FREE_BYTES:
                return (
                    False,
                    f"only {free / 1e9:.1f} GB free where recordings are written, "
                    f"and one bag file is {MAX_BAG_SIZE_BYTES / 1e9:.0f} GB — "
                    "delete a recording first",
                    {"free_bytes": free},
                )

            command = _build_command(path, resolved, compression)
            try:
                # No start_new_session, no PIPE: see the class docstring. cwd is
                # the records root's parent rather than inherited, so nothing
                # about this depends on where the backend was started from --
                # `-o` is absolute anyway, but a relative fallback landing in
                # someone's shell cwd is the kind of surprise this workspace has
                # already had once with config/system.ini.
                process = subprocess.Popen(command, cwd=os.path.expanduser("~"))
            except FileNotFoundError:
                return (
                    False,
                    "ros2 not found on PATH — the backend was started without a "
                    "sourced ROS 2 environment",
                    {},
                )
            except OSError as exc:
                return False, f"could not start ros2 bag record: {exc}", {}

            # The liveness probe, MdnsManager's shape. A recorder that is going
            # to fail on its arguments does it immediately, and that failure
            # belongs to this POST rather than to whoever next reads the log.
            try:
                process.wait(timeout=_LIVENESS_PROBE)
                return (
                    False,
                    f"ros2 bag record exited immediately (code "
                    f"{process.returncode}) — see the backend log for its own "
                    "error; a bad topic name or an unwritable record/ is the "
                    "usual cause",
                    {},
                )
            except subprocess.TimeoutExpired:
                pass

            active = _Active(
                name=name,
                path=path,
                topics=resolved,
                started_at=datetime.now(timezone.utc),
                started_monotonic=time.monotonic(),
                compression=compression,
                process=process,
            )
            self._active = active

        self._logger.info(
            "[RecordingGateway] Recording started",
            recording=name,
            topics=resolved,
            compression=compression,
            pid=process.pid,
        )
        return True, "", self._describe(active)

    def stop(self) -> Tuple[bool, str, Dict]:
        """Stop the live recorder and wait for it to flush its metadata.

        Blocking, by up to ~22 s in the worst case. That is why the route is a
        plain ``def`` handler: FastAPI runs it on a worker thread, and the
        caller genuinely wants to know whether the bag is playable, which is
        only knowable once the child is gone.

        The slot is released before the signal, not after. A stop that hangs on
        the last rung must not leave the API refusing every later start against
        a child nobody can reach any more.
        """
        with self._lock:
            self._reap()
            active = self._active
            self._active = None

        if active is None:
            return False, "nothing is being recorded", {}

        stopped_by = _terminate(self._logger, active)

        metadata = os.path.join(active.path, "metadata.yaml")
        complete = os.path.isfile(metadata)
        if not complete:
            self._logger.warning(
                "[RecordingGateway] The bag has no metadata.yaml",
                recording=active.name,
                stopped_by=stopped_by,
            )

        self._logger.info(
            "[RecordingGateway] Recording stopped",
            recording=active.name,
            stopped_by=stopped_by,
            duration_seconds=time.monotonic() - active.started_monotonic,
        )
        return (
            True,
            "",
            {
                "name": active.name,
                "path": active.path,
                "topics": list(active.topics),
                "started_at": active.started_at,
                "elapsed_seconds": time.monotonic() - active.started_monotonic,
                "size_bytes": _dir_size(active.path),
                "stopped_by": stopped_by,
                "complete": complete,
            },
        )


def _build_command(path: str, topics: List[str], compression: bool) -> List[str]:
    """The argv, kept out of start() so a test can read it without a spawn."""
    command = [
        "ros2",
        "bag",
        "record",
        "-o",
        path,
        "--max-bag-size",
        str(MAX_BAG_SIZE_BYTES),
    ]
    if compression:
        # File mode, not message mode: message mode compresses on the writer's
        # own path and shows up as dropped messages under load, while file mode
        # only works on a split that is already closed.
        command += [
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ]
    return command + list(topics)


def _terminate(logger: structlog.stdlib.BoundLogger, active: _Active) -> str:
    """Walk the stop ladder; return which rung ended it.

    SIGINT is the one that matters: rosbag2 installs a handler for it and
    answers by closing the storage and writing metadata.yaml. The two rungs
    below it exist for a writer that is wedged rather than slow, and both of
    them cost the bag its metadata -- recoverable with `ros2 bag reindex`, which
    is why the result is reported rather than raised.
    """
    process = active.process
    if process.poll() is not None:
        return "already_exited"

    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=_SIGINT_TIMEOUT)
        return "sigint"
    except subprocess.TimeoutExpired:
        logger.warning(
            "[RecordingGateway] The recorder ignored SIGINT, sending SIGTERM",
            recording=active.name,
        )

    try:
        process.terminate()
        process.wait(timeout=_SIGTERM_TIMEOUT)
        return "sigterm"
    except subprocess.TimeoutExpired:
        logger.warning(
            "[RecordingGateway] The recorder ignored SIGTERM, sending SIGKILL",
            recording=active.name,
        )

    try:
        process.kill()
        process.wait(timeout=_SIGKILL_TIMEOUT)
    except (subprocess.TimeoutExpired, OSError) as exc:
        # Unreachable short of a child stuck in uninterruptible IO. Reported
        # rather than raised: the slot is already released and the operator's
        # next start must not be refused over it.
        logger.error(
            "[RecordingGateway] Could not kill the recorder",
            recording=active.name,
            error=str(exc),
        )
        return "kill_failed"
    return "sigkill"


def _dir_size(path: str) -> int:
    """Bytes under ``path``, or 0 if it is not there yet.

    Duplicated from the catalogue repo's _walk_stats rather than imported: this
    is the live bag and the only number wanted is the total, and a gateway
    reaching into a repository would invert the layering.
    """
    total = 0
    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                total += os.stat(os.path.join(root, filename)).st_size
            except FileNotFoundError:
                continue
    return total


def _free_bytes(path: str) -> int:
    """Free space where recordings land, answered before record/ exists."""
    probe = path
    while not os.path.exists(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    return shutil.disk_usage(probe).free


def init_recording_gateway(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    records_dir: str,
) -> RecordingGateway:
    return RecordingGateway(logger=logger, robot_id=robot_id, records_dir=records_dir)
