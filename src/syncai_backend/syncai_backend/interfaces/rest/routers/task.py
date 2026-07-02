import structlog
from typing import List
from fastapi import APIRouter
from pydantic import BaseModel, Field
from enum import Enum

from syncai_backend.gateways.workflow.schema import (
    Step,
    StepType,
    StepStatus,
    StepParams,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class StepRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: StepParams = Field(
        ...,
        description="Parameters for the step, which vary based on the step type",
    )


class TaskRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    timestamp: int = Field(
        ..., description="Timestamp of the task", examples=[1782786519]
    )
    steps: List[StepRequest] = Field(
        ..., description="List of steps to be executed", examples=[]
    )


class TaskResponse(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    status: TaskStatus = Field(
        ..., description="Current status of the task", examples=["PENDING"]
    )
    message: str = Field(
        ...,
        description="Additional information about the task",
        examples=["Task is pending execution."],
    )


class StepState(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    status: StepStatus = Field(
        ..., description="Current status of the step", examples=["IN_PROGRESS"]
    )
    error_msg: str = Field(
        default="",
        description="Error message if the step failed",
    )


class TaskStateResponse(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    status: TaskStatus = Field(
        ..., description="Overall status of the task", examples=["IN_PROGRESS"]
    )
    steps: List[StepState] = Field(..., description="Per-step state of the task")


def init_task_router(
    logger: structlog.stdlib.BoundLogger, workflow_gw: WorkflowGateway
) -> APIRouter:
    task_router = APIRouter(prefix="", tags=["Task"])

    @task_router.post("/api/v1/tasks", response_model=TaskResponse)
    async def trigger_task(req: TaskRequest):

        workflow_task = WorkflowTask(
            id=req.id,
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(
                        id=step.id,
                        type=step.type,
                        params=step.params,
                    )
                    for step in req.steps
                ],
            ),
        )

        await workflow_gw.start_task(request=workflow_task)

        return TaskResponse(
            id=req.id,
            status=TaskStatus.PENDING,
            message=f"Task {req.id} is already in the queue for execution.",
        )

    @task_router.get("/api/v1/tasks/{id}", response_model=TaskStateResponse)
    async def get_task_state(id: str):
        state = await workflow_gw.get_task_state(task_id=id)

        return TaskStateResponse(
            id=state.id,
            status=TaskStatus(state.status),
            steps=[
                StepState(
                    id=step.id,
                    status=step.status,
                    error_msg=step.error_msg or "",
                )
                for step in state.steps
            ],
        )

    @task_router.delete("/api/v1/tasks/{id}", response_model=TaskResponse)
    async def cancel_task(id: str):
        await workflow_gw.cancel_task(task_id=id)

        return TaskResponse(
            id=id,
            status=TaskStatus.CANCELED,
            message=f"Task {id} has been requested to cancel.",
        )

    return task_router
