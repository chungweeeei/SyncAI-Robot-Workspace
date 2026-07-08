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


async def run_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    activities: RobotActivities,
    ready: Optional[threading.Event] = None,
) -> None:
    """Connect to Temporal, register the workflow/activities, and run forever.

    `ready` is set right before the worker starts polling, so a caller running
    this in a background thread can block until the worker is up.
    """

    client = await Client.connect(
        TEMPORAL_SERVER_URL, data_converter=pydantic_data_converter
    )

    worker = Worker(
        client,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
        workflows=[RobotWorkflow],
        activities=[
            activities.execute_move,
            activities.execute_patrol,
            activities.execute_artifact,
        ],
        activity_executor=ThreadPoolExecutor(max_workers=1),
    )

    logger.info(
        "Temporal worker started",
        server=TEMPORAL_SERVER_URL,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
    )

    if ready is not None:
        ready.set()

    await worker.run()


def start_temporal_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    robot_gw: RobotGateway,
    artifact_gw: ArtifactGateway,
) -> threading.Thread:

    ready = threading.Event()

    def _thread_target() -> None:
        activities = RobotActivities(
            logger=logger, robot_gw=robot_gw, artifact_gw=artifact_gw
        )
        asyncio.run(
            run_worker(logger, robot_id=robot_id, activities=activities, ready=ready)
        )

    thread = threading.Thread(target=_thread_target, daemon=True)
    thread.start()
    ready.wait(timeout=10.0)

    return thread
