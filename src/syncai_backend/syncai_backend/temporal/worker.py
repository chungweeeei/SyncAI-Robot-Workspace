import asyncio
import threading
from typing import Optional

import structlog

from temporalio.client import Client
from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.worker import Worker

from temporal.shared import TEMPORAL_SERVER_URL, TASK_QUEUE
from temporal.workflows import TaskWorkflow
from temporal.activities import TaskActivities


async def run_worker(
    logger: structlog.stdlib.BoundLogger,
    ready: Optional[threading.Event] = None,
) -> None:
    """Connect to Temporal, register the workflow/activities, and run forever.

    `ready` is set right before the worker starts polling, so a caller running
    this in a background thread can block until the worker is up.
    """

    client = await Client.connect(
        TEMPORAL_SERVER_URL, data_converter=pydantic_data_converter
    )

    activities = TaskActivities(logger=logger)

    worker = Worker(
        client,
        task_queue=TASK_QUEUE,
        workflows=[TaskWorkflow],
        activities=[activities.execute_move],
    )

    logger.info(
        "[TemporalWorker] started",
        task_queue=TASK_QUEUE,
        server=TEMPORAL_SERVER_URL,
    )

    if ready is not None:
        ready.set()

    await worker.run()


def start_temporal_worker(logger: structlog.stdlib.BoundLogger) -> threading.Thread:
    """Start the Temporal worker in a daemon thread (co-exists with rclpy.spin())."""

    ready = threading.Event()

    def _thread_target() -> None:
        asyncio.run(run_worker(logger, ready=ready))

    thread = threading.Thread(target=_thread_target, daemon=True)
    thread.start()
    ready.wait(timeout=10.0)

    logger.info("[TemporalWorker] ready")
    return thread
