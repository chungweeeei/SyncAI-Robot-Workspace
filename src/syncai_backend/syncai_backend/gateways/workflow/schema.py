from datetime import datetime
from typing import List, Union, Optional
from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel
from enum import Enum


class BaseSchema(BaseModel):
    model_config = ConfigDict(
        alias_generator=to_camel,
        validate_by_name=True,
        validate_by_alias=True,
    )


class StepStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class StepType(str, Enum):
    MOVE = "MOVE"


class MoveParams(BaseSchema):
    x: float = Field(
        ..., description="Target x-coordinate for the move step", examples=[1.0]
    )
    y: float = Field(
        ..., description="Target y-coordinate for the move step", examples=[2.0]
    )
    theta: float = Field(
        ...,
        gt=-180.0,
        le=180.0,
        description="Target orientation (in degrees) for the move step",
        examples=[90.0],
    )


StepParams = Union[MoveParams]


class Step(BaseSchema):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: StepParams = Field(
        ...,
        description="Parameters for the step, which vary based on the step type",
    )
    status: StepStatus = Field(
        StepStatus.PENDING,
        description="Current status of the step",
        examples=["PENDING"],
    )
    error_msg: Optional[str] = Field(
        default="",
        description="Error message if the step failed",
        json_schema_extra={"example": ""},
    )


class WorkflowTaskDefinition(BaseSchema):
    steps: List[Step] = Field(..., description="Ordered list of steps to execute")


class WorkflowTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the workflow task")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Definition of the workflow task"
    )


class TaskState(BaseSchema):
    id: str = Field(..., description="Unique identifier of the task")
    status: str = Field(..., description="Overall task status")
    steps: List[Step] = Field(..., description="Per-step state")


class ScheduleTrigger(BaseSchema):
    cron: Optional[str] = Field(
        default=None,
        description="Cron expression, e.g. '0 9 * * 1-5'. Mutually exclusive with intervalSeconds.",
        examples=["*/3 * * * *"],
    )
    interval_seconds: Optional[int] = Field(
        default=None,
        gt=0,
        description="Fire every N seconds. Mutually exclusive with cron.",
        examples=[1800],
    )
    timezone: Optional[str] = Field(
        default=None,
        description="IANA timezone applied to cron, e.g. 'Asia/Taipei'. Ignored for interval.",
        examples=["Asia/Taipei"],
    )


class ScheduleTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTrigger = Field(..., description="When the schedule fires")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Task definition executed on each trigger"
    )


class ScheduleView(BaseSchema):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTrigger = Field(..., description="When the schedule fires")
    paused: bool = Field(False, description="Whether the schedule is currently paused")
    next_run_times: List[datetime] = Field(
        default_factory=list,
        description="Upcoming trigger times (UTC)",
    )
