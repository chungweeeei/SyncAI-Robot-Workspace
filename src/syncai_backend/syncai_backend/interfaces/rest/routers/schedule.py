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
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "Map whose frame the MOVE coordinates are in, carried through to the "
            "schedule memo so a client can tell whether this schedule still "
            "belongs to the loaded map. A display label only, and deliberately "
            "unvalidated: this router has no MapCatalogRepo, and threading one in "
            "for a label is not worth it. POST /api/v1/saved_tasks/{id}/schedule "
            "is the validated path."
        ),
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
    map_name: Optional[str] = Field(
        default=None,
        description="Map whose frame this schedule's MOVE coordinates are in.",
    )
    saved_task_id: Optional[str] = Field(
        default=None,
        description="The saved task this schedule was frozen from, if any.",
    )
    saved_task_name: Optional[str] = Field(
        default=None, description="That saved task's name at registration time."
    )
    # The element is StepRequest rather than a new response model: it is already
    # exactly {id, type, params}, it is already imported here, and a second
    # identical model is a copy that can drift. (StepState is the wrong shape --
    # that one is status/error, not a definition.)
    steps: List[StepRequest] = Field(
        default_factory=list,
        description=(
            "The frozen step list. Populated by GET /api/v1/schedules/{id} only; "
            "the collection endpoint always answers [], because Temporal's "
            "schedule *list* API does not carry the start-workflow arguments. "
            "Frozen at registration: later vertex edits do not reach a scheduled "
            "run, so a client comparing these against their source saved task is "
            "how staleness becomes visible."
        ),
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
            map_name=req.map_name,
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
                map_name=view.map_name,
                saved_task_id=view.saved_task_id,
                saved_task_name=view.saved_task_name,
                # `steps` left at its default here — see the field's description.
            )
            for view in views
        ]

    @schedule_router.get(
        "/api/v1/schedules/{id}",
        response_model=ScheduleStateResponse,
        # The only route here that serialises StepParams outward, and FastAPI
        # defaults this flag to True: MoveParams / ArtifactParams inherit
        # BaseSchema's to_camel generator, so the default would answer
        # `artifactId` / `waitFor` / `waitTimeoutSeconds` while the request side
        # accepts both spellings — a silent asymmetry at a boundary the frontend
        # documents as snake_case-only.
        response_model_by_alias=False,
    )
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
            map_name=view.map_name,
            saved_task_id=view.saved_task_id,
            saved_task_name=view.saved_task_name,
            steps=[
                StepRequest(id=step.id, type=step.type, params=step.params)
                for step in view.steps
            ],
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
