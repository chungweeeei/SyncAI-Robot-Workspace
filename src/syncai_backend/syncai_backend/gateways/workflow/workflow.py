import structlog

from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.client import Client

from exceptions import InternalServerError

from gateways.workflow.schema import (
    Step,
    StepRequest,
    TaskRequest,
    WorkflowTask,
    WorkflowTaskDefinition,
)


class WorkflowGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger
        self._client: Client | None = None

    async def _get_client(self) -> Client:
        if self._client is not None:
            return self._client

        # try to connect to the Temporal server
        try:
            self._client = await Client.connect(
                target_host="127.0.0.1:7233", data_converter=pydantic_data_converter
            )
        except Exception as err:
            raise err

        return self._client

    # async def start_task(self, request: TaskRequest):

    #     try:
    #         client = await self._get_client()
    #     except Exception as err:
    #         self._logger.error("Failed to connect to Temporal server", error=str(err))
    #         raise InternalServerError("Failed to connect to Temporal server")

    #     workflow_steps = [
    #         Step(id=s.id, type=s.type, params=s.params) for s in request.steps
    #     ]
    #     workflow_task = WorkflowTask(
    #         id=request.id,
    #         definition=WorkflowTaskDefinition(
    #             steps=workflow_steps, settings={"repeat": 0}
    #         ),
    #     )


def init_workflow_gateway(
    logger: structlog.stdlib.BoundLogger,
) -> WorkflowGateway:
    workflow_gw = WorkflowGateway(logger=logger)
    return workflow_gw
