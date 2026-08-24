import structlog
from datetime import datetime
from typing import List, Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field, model_validator
from enum import Enum

from syncai_backend.gateways.workflow.schema import (
    Step,
    StepType,
    StepStatus,
    StepParams,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
    validate_step_params,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    # Only ever answered by DELETE /tasks/{id}, never by the status mapping:
    # a delete is a cancel *request*, and Temporal is free to let the workflow
    # finish COMPLETED before the cancellation lands. The old response said
    # CANCELED outright — a lie the frontend had already noticed and was
    # discarding (see its task.ts).
    CANCELING = "CANCELING"


class StepRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: Optional[StepParams] = Field(
        default=None,
        description=(
            "Parameters for the step, which vary based on the step type. "
            "Omitted for STANDUP/LIEDOWN, required for MOVE"
        ),
    )

    # Same check as Step, repeated here so a mismatched body is rejected at the
    # request boundary (422) instead of raising a ValidationError inside the
    # handler when the Step is built (500).
    @model_validator(mode="after")
    def _check_params(self) -> "StepRequest":
        validate_step_params(self.type, self.params)
        return self


# No `timestamp` field: the old required one was received and then never read
# by anything, which made every client invent a value to satisfy validation.
# Existing callers (console, MCP) still send it and pydantic ignores unknown
# fields by default, so dropping it is backward compatible.
class TaskRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
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


class ActiveTaskResponse(BaseModel):
    id: str = Field(
        ...,
        description="Workflow id, i.e. the id GET/DELETE /api/v1/tasks/{id} takes",
        examples=["robot01-goal-1782786519-3"],
    )
    run_id: str = Field(..., description="Temporal run id")
    status: TaskStatus = Field(..., examples=["IN_PROGRESS"])
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    source: TaskSource = Field(
        ..., description="DIRECT (someone called POST /api/v1/tasks) or SCHEDULE"
    )
    schedule_id: Optional[str] = Field(
        default=None, description="The schedule that started it, if any"
    )


class ActiveTasksResponse(BaseModel):
    tasks: List[ActiveTaskResponse] = Field(
        ..., description="Executions running on this robot's task queue"
    )
    as_of: datetime = Field(
        ...,
        description=(
            "When the snapshot was read. Elapsed time should be computed "
            "against this, not against the client's clock — they are two "
            "different clocks and the answer is served from a short cache."
        ),
    )


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
            message=f"Task {req.id} accepted and queued for execution.",
        )

    # Not /api/v1/tasks/active. FastAPI matches by declaration order, so that
    # path only works while it is declared above /api/v1/tasks/{id} — the day
    # someone reorders these decorators it silently becomes a lookup for a task
    # literally named "active" and answers 404. The same collision is why
    # /api/v1/task_templates was chosen over /api/v1/tasks/templates; see the
    # note in interfaces/rest/server.py.
    #
    # Plural, and a list, because "one robot does one thing" is not an invariant
    # this endpoint can rely on: ScheduleOverlapPolicy.SKIP constrains a single
    # schedule against itself, and a direct POST /api/v1/tasks bypasses it
    # entirely, so an operator dispatch during a scheduled run leaves two
    # executions Running. Two is not an error, and reporting one of them would
    # be the lie.
    #
    # Never 404s: an empty list is a valid answer. "Nothing is running" is not a
    # missing resource.
    @task_router.get("/api/v1/active_tasks", response_model=ActiveTasksResponse)
    async def list_active_tasks():
        tasks, as_of = await workflow_gw.list_active_tasks()

        return ActiveTasksResponse(
            tasks=[
                ActiveTaskResponse(
                    id=task.id,
                    run_id=task.run_id,
                    status=TaskStatus(task.status),
                    started_at=task.started_at,
                    source=task.source,
                    schedule_id=task.schedule_id,
                )
                for task in tasks
            ],
            as_of=as_of,
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

        # CANCELING, not CANCELED — see the note on the enum member.
        return TaskResponse(
            id=id,
            status=TaskStatus.CANCELING,
            message=(
                f"Cancellation of task {id} requested; poll "
                f"GET /api/v1/tasks/{id} for the final state."
            ),
        )

    return task_router
