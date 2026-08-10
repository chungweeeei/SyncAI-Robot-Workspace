import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import structlog

from temporalio.client import Client
from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.worker import Worker

from syncai_backend.temporal.shared import TEMPORAL_SERVER_URL
from syncai_backend.temporal.workflows import RobotWorkflow
from syncai_backend.temporal.activities import RobotActivities

from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.artifact.artifact import ArtifactGateway

# Mirrors database/postgres.py: same bounded-retry shape for the same reason —
# on a robot boot the shared docker-compose services (postgres, temporal) may
# come up after the backend does. The difference is what happens when the
# budget runs out: postgres raises and takes the process down (a backend
# without its DB is useless), while the worker only marks itself dead. The
# backend runs in a byobu pane with no supervisor to restart it, and with
# Temporal gone the rest of the process (telemetry, maps, manual control) is
# still worth keeping alive — so the failure is surfaced through /health
# instead of a crash.
MAX_RETRIES = 20
RETRY_INTERVAL = 5


class TemporalWorkerHandle:
    """Cross-thread view of the worker's lifecycle for /health.

    The worker lives in a daemon thread; before this handle existed, a failed
    ``Client.connect`` killed that thread silently and the only symptom was
    tasks that queued forever. Every exit path of the thread now lands in one
    of these states, so "is the task server actually polling?" is answerable
    from the REST side.
    """

    STATUS_CONNECTING = "connecting"
    STATUS_RUNNING = "running"
    STATUS_DEAD = "dead"

    def __init__(self):
        self._lock = threading.Lock()
        self._status = self.STATUS_CONNECTING
        self._last_error: Optional[str] = None
        self.thread: Optional[threading.Thread] = None

    def mark_running(self) -> None:
        with self._lock:
            self._status = self.STATUS_RUNNING
            self._last_error = None

    def mark_dead(self, error: str) -> None:
        with self._lock:
            self._status = self.STATUS_DEAD
            self._last_error = error

    def snapshot(self) -> tuple[str, Optional[str]]:
        with self._lock:
            return self._status, self._last_error


async def run_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    activities: RobotActivities,
    handle: TemporalWorkerHandle,
    ready: Optional[threading.Event] = None,
) -> None:
    """Connect to Temporal (with bounded retries), register the
    workflow/activities, and run forever.

    `ready` is set right before the worker starts polling, so a caller running
    this in a background thread can block until the worker is up.
    """

    client = None
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            client = await Client.connect(
                TEMPORAL_SERVER_URL, data_converter=pydantic_data_converter
            )
            break
        except Exception as err:
            logger.warning(
                "Connection attempt failed",
                component="Temporal",
                attempt=attempt,
                max_retries=MAX_RETRIES,
                error=str(err),
            )
            if attempt == MAX_RETRIES:
                handle.mark_dead(str(err))
                logger.error(
                    "Giving up on Temporal; task server is dead until restart",
                    server=TEMPORAL_SERVER_URL,
                )
                return
            await asyncio.sleep(RETRY_INTERVAL)

    worker = Worker(
        client,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
        workflows=[RobotWorkflow],
        activities=[
            activities.execute_move,
            activities.execute_artifact,
            activities.execute_stand,
            activities.execute_lie_down,
        ],
        activity_executor=ThreadPoolExecutor(max_workers=1),
    )

    logger.info(
        "Temporal worker started",
        server=TEMPORAL_SERVER_URL,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
    )

    handle.mark_running()
    if ready is not None:
        ready.set()

    await worker.run()


def start_temporal_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    robot_gw: RobotGateway,
    artifact_gw: ArtifactGateway,
) -> TemporalWorkerHandle:

    ready = threading.Event()
    handle = TemporalWorkerHandle()

    def _thread_target() -> None:
        # Anything that escapes run_worker — including worker.run() dying
        # mid-flight — must land in the handle, or we are back to the silent
        # dead thread this exists to prevent.
        try:
            activities = RobotActivities(
                logger=logger, robot_gw=robot_gw, artifact_gw=artifact_gw
            )
            asyncio.run(
                run_worker(
                    logger,
                    robot_id=robot_id,
                    activities=activities,
                    handle=handle,
                    ready=ready,
                )
            )
        except Exception as err:
            handle.mark_dead(str(err))
            logger.error("Temporal worker thread died", exc_info=True)

    thread = threading.Thread(target=_thread_target, daemon=True)
    thread.start()
    handle.thread = thread
    # Not becoming ready in 10s is expected when Temporal is still booting —
    # the retry budget above runs ~100s — so this is a heads-up, not a failure.
    if not ready.wait(timeout=10.0):
        logger.warning(
            "Temporal worker not ready yet; still connecting in the background",
            server=TEMPORAL_SERVER_URL,
        )

    return handle
