from datetime import datetime
from typing import List, Optional
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


# ARTIFACT (conveyor pickup/drop via the artifact backend's REST API) was a
# member here until 2026-08: the whole integration — gateways/artifact,
# execute_artifact, ArtifactParams and its command union — was removed when the
# conveyor work was shelved, deliberately rather than left to rot. Old saved
# tasks / frozen schedules that still carry an ARTIFACT step fail Step
# validation now; purge them before deploying a build without it.
class StepType(str, Enum):
    MOVE = "MOVE"
    STANDUP = "STANDUP"
    LIEDOWN = "LIEDOWN"
    SPEAK = "SPEAK"


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


# Field constraints mirror the REST router's SynthesizeRequest
# (routers/tts.py): a SPEAK step and a manual POST /api/v1/tts/speak drive the
# same TtsGateway, so what one accepts the other must too. All single-word
# field names, so BaseSchema's camelCase aliasing is a no-op here — same
# property MoveParams relies on (see the task_template router's serialisation
# note).
class SpeakParams(BaseSchema):
    text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The text to speak. English only for now (see TtsGateway).",
        examples=["Delivery arrived"],
    )
    voice: str = Field(
        "af_heart",
        description="Kokoro voice id; the list is at GET /api/v1/tts/voices.",
    )
    speed: float = Field(
        1.0, ge=0.5, le=2.0, description="Playback rate multiplier (0.5–2.0)."
    )


# Re-widened when SPEAK arrived (it was a single-member alias after the
# ARTIFACT removal). Pydantic's smart union tells the members apart by their
# required fields — MoveParams needs x/y/theta, SpeakParams needs text, with
# no overlap — so no discriminator field is necessary.
StepParams = MoveParams | SpeakParams


# Which params model each step type expects; None means the step takes no
# params at all (STANDUP/LIEDOWN are a single motion key, there is nothing to
# parameterise). The table survives StepParams collapsing to one model:
# without it a MOVE step with no body at all -- params is optional -- would
# validate fine here and only blow up inside the activity, long after the REST
# call was answered with 200.
STEP_PARAMS_TYPE: dict[StepType, Optional[type[BaseModel]]] = {
    StepType.MOVE: MoveParams,
    StepType.STANDUP: None,
    StepType.LIEDOWN: None,
    StepType.SPEAK: SpeakParams,
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
            "Omitted for STANDUP/LIEDOWN, required for MOVE and SPEAK"
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
        description=(
            "Cron expression, e.g. '0 9 * * 1-5'. Mutually exclusive with intervalSeconds."
        ),
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
    task_template_id: Optional[str] = Field(
        default=None,
        description="The task template this schedule was frozen from, if any.",
    )
    task_template_name: Optional[str] = Field(
        default=None, description="That template's name at registration time."
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
    task_template_id: Optional[str] = Field(default=None)
    task_template_name: Optional[str] = Field(default=None)
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
