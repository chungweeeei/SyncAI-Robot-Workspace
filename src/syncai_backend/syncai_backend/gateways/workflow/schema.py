from datetime import datetime
from typing import Annotated, List, Literal, Union, Optional
from pydantic import BaseModel, ConfigDict, Field, model_validator
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
    ARTIFACT = "ARTIFACT"
    STANDUP = "STANDUP"
    LIEDOWN = "LIEDOWN"


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


# Mirrors the artifact backend's command API (SyncAI-Artifact-Workspace
# routers/artifact.py). Field names/values must stay in sync with it: the
# command is forwarded verbatim and re-validated there.
class PickupCommand(BaseSchema):
    action: Literal["pickup"]
    robot: Union[Annotated[int, Field(ge=0, le=0xFFFE)], Literal["any"]] = Field(
        "any", description="Robot index, or 'any' = closest robot in the dock"
    )
    box: int = Field(
        0, ge=0, le=0xFFFE, description="0 = unspecified; N = boxNN", examples=[0]
    )


class DropCommand(BaseSchema):
    action: Literal["drop"]
    zone: Union[Annotated[int, Field(ge=0, le=0xFFFE)], Literal["any"]] = Field(
        "any", description="Drop-zone index, or 'any' = debug bypass"
    )
    box: int = Field(
        0, ge=0, le=0xFFFE, description="0 = unspecified; N = boxNN", examples=[0]
    )


ArtifactCommand = Annotated[
    Union[PickupCommand, DropCommand], Field(discriminator="action")
]


# Cargo pipeline phase reported in live_info.phase (GET /state). Mirrors the
# sim's phase codes decoded by syncai_artifact_state: belt -> handoff ->
# carried -> dropped.
class ConveyorPhase(str, Enum):
    BELT = "belt"
    HANDOFF = "handoff"
    CARRIED = "carried"
    DROPPED = "dropped"


class ArtifactParams(BaseSchema):
    artifact_id: str = Field(
        ...,
        description="Registry key resolving to the artifact backend base URL",
        examples=["conveyor01"],
    )
    command: ArtifactCommand = Field(
        ...,
        description=(
            "Command sent as the POST /api/v1/artifact/command body, "
            "discriminated by 'action'"
        ),
        examples=[{"action": "pickup", "robot": "any", "box": 0}],
    )
    wait_for: Optional[ConveyorPhase] = Field(
        default=None,
        description=(
            "If set, poll the artifact state until live_info.phase reaches "
            "this value; if omitted the step completes once the command is "
            "accepted"
        ),
        examples=["handoff"],
    )
    wait_timeout_seconds: int = Field(
        60,
        gt=0,
        description="Fail the step if wait_for is not reached within this time",
        examples=[120],
    )


StepParams = Union[MoveParams, ArtifactParams]


# Which params model each step type expects; None means the step takes no
# params at all (STANDUP/LIEDOWN are a single motion key, there is nothing to
# parameterise). The table exists because StepParams is a plain union with no
# discriminator: without an explicit check a MOVE step carrying an artifact
# body -- or, now that params is optional, no body at all -- would validate
# fine here and only blow up inside the activity, long after the REST call was
# answered with 200.
STEP_PARAMS_TYPE: dict[StepType, Optional[type[BaseModel]]] = {
    StepType.MOVE: MoveParams,
    StepType.ARTIFACT: ArtifactParams,
    StepType.STANDUP: None,
    StepType.LIEDOWN: None,
}


def validate_step_params(
    step_type: StepType, params: Optional[StepParams]
) -> Optional[StepParams]:
    expected = STEP_PARAMS_TYPE[step_type]

    if expected is None:
        if params is not None:
            raise ValueError(f"{step_type.value} step takes no params")
        return params

    if not isinstance(params, expected):
        raise ValueError(f"{step_type.value} step requires {expected.__name__}")

    return params


class Step(BaseSchema):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: Optional[StepParams] = Field(
        default=None,
        description=(
            "Parameters for the step, which vary based on the step type. "
            "Omitted for STANDUP/LIEDOWN, required for MOVE/ARTIFACT"
        ),
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

    @model_validator(mode="after")
    def _check_params(self) -> "Step":
        validate_step_params(self.type, self.params)
        return self


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


class TaskSource(str, Enum):
    """Who started a running task.

    DIRECT, not OPERATOR: `POST /api/v1/tasks` is also how syncai_ros_mcp
    dispatches, so "an operator did this" would be a claim the backend cannot
    make. All it can tell apart is "someone called the endpoint" from "a
    Temporal schedule fired", and the latter only because the schedule
    machinery stamps its own search attribute on the run.
    """

    DIRECT = "DIRECT"
    SCHEDULE = "SCHEDULE"


class ActiveTask(BaseSchema):
    """One execution that is running on this robot's task queue right now.

    Deliberately identity and provenance only -- no step list. The point of this
    shape is that it comes from a single visibility query with no per-task
    follow-up, which is what makes it cheap enough for the whole console to poll
    (see ACTIVE_TASK_CACHE_TTL_S). Per-step detail already has an endpoint:
    GET /api/v1/tasks/{id}, and `id` here is exactly the id it takes.
    """

    id: str = Field(..., description="Workflow id, i.e. the task id")
    run_id: str = Field(
        ...,
        description=(
            "Temporal run id. Distinguishes two runs of the same schedule, "
            "whose workflow ids differ only by nominal trigger time."
        ),
    )
    status: str = Field(..., description="Mapped through _WORKFLOW_STATUS_MAP")
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    source: TaskSource = Field(..., description="Who started it")
    schedule_id: Optional[str] = Field(
        default=None,
        description="The schedule that started it, when source is SCHEDULE.",
    )


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

    # Provenance, stashed in the schedule memo rather than anywhere Temporal
    # interprets. The memo is the established channel for "keep something so it
    # survives a round trip" (see _trigger_to_memo) and, crucially, unlike the
    # start-workflow args it IS readable on the *list* path -- which is what lets
    # the collection endpoint flag a schedule whose map is no longer the active
    # one without an extra describe per row.
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


class ScheduleView(BaseSchema):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTrigger = Field(..., description="When the schedule fires")
    paused: bool = Field(False, description="Whether the schedule is currently paused")
    next_run_times: List[datetime] = Field(
        default_factory=list,
        description="Upcoming trigger times (UTC)",
    )
    map_name: Optional[str] = Field(default=None)
    saved_task_id: Optional[str] = Field(default=None)
    saved_task_name: Optional[str] = Field(default=None)
    steps: List[Step] = Field(
        default_factory=list,
        description=(
            "The frozen step list, decoded from the schedule's start-workflow "
            "args. Populated by describe() only -- always empty from "
            "list_schedules(), whose ScheduleListActionStartWorkflow carries the "
            "workflow type name and nothing else. Frozen at registration: later "
            "vertex edits do not reach a scheduled run."
        ),
    )
