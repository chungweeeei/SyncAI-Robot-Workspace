from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel


class BaseSchema(BaseModel):
    model_config = ConfigDict(populate_by_name=True, alias_generator=to_camel)


# ---- Enums ----
class StepType(str, Enum):
    MOVE = "MOVE"


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class StepStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


# ---- Step params ----
class MoveParams(BaseSchema):
    x: float = Field(..., description="Target x-coordinate in meters", examples=[-3.5])
    y: float = Field(..., description="Target y-coordinate in meters", examples=[4.5])
    theta: float = Field(
        ..., ge=-180, le=180, description="Target heading in degrees", examples=[90]
    )


# ---- Step（用繼承減少重複）----
# 目前單一 type 故 params 直接是 MoveParams；新增 step type 時改用 discriminated union。
class StepBase(BaseSchema):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: MoveParams = Field(
        ..., description="Parameters required to execute the step"
    )


class StepRequest(StepBase):
    """對外輸入：尚無執行狀態。"""


class Step(StepBase):
    status: StepStatus = Field(
        default=StepStatus.PENDING, description="Execution status of the step"
    )
    error_msg: Optional[str] = Field(
        default=None, description="Error message if the step failed"
    )


# ---- Request ----
class TaskRequest(BaseSchema):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    timestamp: float = Field(
        ...,
        description="Epoch seconds when the task was issued (converted to ms downstream)",
        examples=[1746662400.0],
    )
    steps: List[StepRequest] = Field(
        ..., min_length=1, description="Ordered list of steps that make up the task"
    )


# ---- Response / domain ----
class TaskResponse(BaseSchema):
    """POST / DELETE 的 ack。"""

    id: str = Field(..., examples=["robot01-task-001"])
    status: TaskStatus = Field(..., examples=["IN_PROGRESS"])
    message: str = Field(..., examples=["Task created successfully"])


# ---- Activity 結果（供 Temporal worker 回傳，需為 Pydantic model）----
class StepResult(BaseSchema):
    success: bool = Field(..., description="Whether the step/activity succeeded")
    message: str = Field(default="", description="Human-readable result message")


class TaskPayload(BaseSchema):
    steps: List[Step] = Field(
        ...,
        min_length=1,
        description="Ordered list of steps with their current execution status",
    )


class Task(BaseSchema):
    id: str = Field(..., description="Unique identifier of the task")
    timestamp: float = Field(
        ...,
        description="Epoch seconds when the task was issued or the workflow started",
        examples=[1746662400.0],
    )
    payload: TaskPayload = Field(..., description="Task payload with per-step status")
    status: TaskStatus = Field(
        default=TaskStatus.PENDING, description="Overall execution status of the task"
    )
    current_step_index: int = Field(
        default=0, description="Index of the step currently being executed"
    )
    error_msg: Optional[str] = Field(
        default=None, description="Aggregated error message when the task has failed"
    )
    workflow_id: Optional[str] = Field(
        default=None, description="Temporal workflow identifier"
    )
    completed_at: Optional[float] = Field(
        default=None,
        description="Epoch seconds when the workflow finished (None if still running)",
    )


# ---- Workflow 內部 ----
class WorkflowTaskDefinition(BaseSchema):
    steps: List[StepRequest] = Field(
        ..., min_length=1, description="Ordered list of steps to execute"
    )
    settings: Dict[str, Any] = Field(
        default_factory=dict,
        description="Workflow-level settings (e.g., repeat count). TODO: 改 typed model",
    )


class WorkflowTask(BaseSchema):
    id: str = Field(..., description="Unique identifier of the task")
    definition: WorkflowTaskDefinition = Field(
        ..., description="Task definition consumed by the workflow worker"
    )
