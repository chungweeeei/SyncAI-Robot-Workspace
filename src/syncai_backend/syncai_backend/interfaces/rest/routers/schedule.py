import structlog
from datetime import datetime
from typing import List, Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field, model_validator

from syncai_backend.gateways.workflow.schema import (
    Step,
    ScheduleTask,
    ScheduleTrigger,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.interfaces.rest.routers.task import StepRequest


class ScheduleTriggerRequest(BaseModel):
    cron: Optional[str] = Field(
        default=None,
        description="Cron expression. Provide exactly one of cron or interval_seconds.",
        examples=["*/3 * * * *"],
    )
    interval_seconds: Optional[int] = Field(
        default=None,
        gt=0,
        description="Fire every N seconds. Provide exactly one of cron or interval_seconds.",
        examples=[1800],
    )
    timezone: Optional[str] = Field(
        default=None,
        description="IANA timezone applied to cron (ignored for interval).",
        examples=["Asia/Taipei"],
    )

    @model_validator(mode="after")
    def _exactly_one(self) -> "ScheduleTriggerRequest":
        if bool(self.cron) == bool(self.interval_seconds):
            raise ValueError("Provide exactly one of 'cron' or 'interval_seconds'")
        return self


class ScheduleRequest(BaseModel):
    id: str = Field(
        ...,
        description="Unique identifier of the schedule",
        examples=["robot01-daily-move"],
    )
    trigger: ScheduleTriggerRequest = Field(..., description="When the schedule fires")
    steps: List[StepRequest] = Field(
        ..., description="Steps executed on each trigger", examples=[]
    )


class ScheduleResponse(BaseModel):
    id: str = Field(..., description="Unique identifier of the schedule")
    message: str = Field(..., description="Additional information about the schedule")


class ScheduleTriggerResponse(BaseModel):
    cron: Optional[str] = None
    interval_seconds: Optional[int] = None
    timezone: Optional[str] = None


class ScheduleStateResponse(BaseModel):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTriggerResponse = Field(..., description="When the schedule fires")
    paused: bool = Field(..., description="Whether the schedule is paused")
    next_run_times: List[datetime] = Field(
        ..., description="Upcoming trigger times (UTC)"
    )


def init_schedule_router(
    logger: structlog.stdlib.BoundLogger, workflow_gw: WorkflowGateway
) -> APIRouter:
    schedule_router = APIRouter(prefix="", tags=["Schedule"])

    @schedule_router.post("/api/v1/schedules", response_model=ScheduleResponse)
    async def create_schedule(req: ScheduleRequest):
        schedule_task = ScheduleTask(
            id=req.id,
            trigger=ScheduleTrigger(
                cron=req.trigger.cron,
                interval_seconds=req.trigger.interval_seconds,
                timezone=req.trigger.timezone,
            ),
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(id=step.id, type=step.type, params=step.params)
                    for step in req.steps
                ],
            ),
        )

        await workflow_gw.create_schedule(schedule=schedule_task)

        return ScheduleResponse(
            id=req.id,
            message=f"Schedule {req.id} has been created.",
        )

    @schedule_router.get(
        "/api/v1/schedules", response_model=List[ScheduleStateResponse]
    )
    async def list_schedules():
        views = await workflow_gw.list_schedules()

        return [
            ScheduleStateResponse(
                id=view.id,
                trigger=ScheduleTriggerResponse(
                    cron=view.trigger.cron,
                    interval_seconds=view.trigger.interval_seconds,
                    timezone=view.trigger.timezone,
                ),
                paused=view.paused,
                next_run_times=view.next_run_times,
            )
            for view in views
        ]

    @schedule_router.get("/api/v1/schedules/{id}", response_model=ScheduleStateResponse)
    async def get_schedule(id: str):
        view = await workflow_gw.get_schedule(schedule_id=id)

        return ScheduleStateResponse(
            id=view.id,
            trigger=ScheduleTriggerResponse(
                cron=view.trigger.cron,
                interval_seconds=view.trigger.interval_seconds,
                timezone=view.trigger.timezone,
            ),
            paused=view.paused,
            next_run_times=view.next_run_times,
        )

    @schedule_router.delete("/api/v1/schedules/{id}", response_model=ScheduleResponse)
    async def delete_schedule(id: str):
        await workflow_gw.delete_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been deleted.",
        )

    @schedule_router.post(
        "/api/v1/schedules/{id}/pause", response_model=ScheduleResponse
    )
    async def pause_schedule(id: str):
        await workflow_gw.pause_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been paused.",
        )

    @schedule_router.post(
        "/api/v1/schedules/{id}/resume", response_model=ScheduleResponse
    )
    async def resume_schedule(id: str):
        await workflow_gw.resume_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been resumed.",
        )

    return schedule_router
