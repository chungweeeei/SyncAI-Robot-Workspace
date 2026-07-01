import structlog

from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.client import Client
from temporalio.service import RPCError, RPCStatusCode

from syncai_backend.exceptions import InternalServerError, NotFoundError

from syncai_backend.gateways.workflow.schema import WorkflowTask
from syncai_backend.gateways.workflow.config import (
    WORKFLOW_TASK_QUEUE,
    WORKFLOW_TYPE_NAME,
)


class WorkflowGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger
        self._client: Client | None = None

    async def _get_client(self) -> Client:
        if self._client is not None:
            return self._client

        try:
            self._client = await Client.connect(
                target_host="127.0.0.1:7233", data_converter=pydantic_data_converter
            )
        except Exception as err:
            raise err

        return self._client

    async def start_task(self, request: WorkflowTask):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error("Failed to connect to Temporal server", error=str(err))
            raise InternalServerError("Failed to connect to Temporal server")

        try:
            await client.start_workflow(
                workflow=WORKFLOW_TYPE_NAME,
                id=request.id,
                args=[request],
                task_queue=WORKFLOW_TASK_QUEUE,
            )
        except Exception as err:
            self._logger.error("Failed to start workflow", error=str(err))
            raise InternalServerError("Start workflow failed")

    async def cancel_task(self, task_id: str):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error("Failed to connect to Temporal server", error=str(err))
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_workflow_handle(task_id)

        try:
            await handle.cancel()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Task {task_id} not found")
            self._logger.error("Failed to cancel workflow", error=str(err))
            raise InternalServerError("Cancel workflow failed")


def init_workflow_gateway(
    logger: structlog.stdlib.BoundLogger,
) -> WorkflowGateway:
    workflow_gw = WorkflowGateway(logger=logger)
    return workflow_gw
