from typing import List, Dict, Any, Union, Optional
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
    settings: Dict[str, Any] = Field(
        ...,
        description="Workflow-level settings (e.g., repeat count)",
    )


class WorkflowTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the workflow task")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Definition of the workflow task"
    )
