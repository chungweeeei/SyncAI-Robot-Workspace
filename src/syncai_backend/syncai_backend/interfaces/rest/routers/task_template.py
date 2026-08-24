"""REST surface for the operator's library of re-dispatchable task templates.

Why this exists: ``POST /api/v1/tasks`` creates *and dispatches* in one call and
persists nothing, and Temporal is not a fallback -- namespace ``default`` keeps
closed workflows for one day with no archival, so yesterday's step list is gone.
An operator who authored a patrol route had no way to run it again.

Two things about this module are worth knowing before reading it.

**Vertex resolution happens here, on the read.** A template's MOVE step carries
both a ``vertex_id`` and a ``params`` snapshot. Every read reports
``resolved_params`` -- the vertex's *current* pose when it still exists, the
snapshot when it does not -- so moving a dock on the map updates every template
that references it, and a client dispatches by sending ``resolved_params``
without ever reimplementing that rule. The rejected alternative was a
server-side ``POST /api/v1/task_templates/{id}/dispatch``: it would have to
either refuse or silently substitute, where this lets the operator *see* the
resolved numbers and the "vertex was deleted" warning before committing. It also
keeps ``POST /api/v1/tasks`` the single dispatch path, and keeps a template's
provenance out of everything Temporal persists.

**The path is ``/api/v1/task_templates``, not ``/api/v1/tasks/templates``.**
``GET /api/v1/tasks/{id}`` takes an unconstrained ``str`` (it is a Temporal
workflow id), so any static sub-path is either shadowed by it or steals the
workflow id ``"templates"``, depending on include order. Cross-module
route-order dependence is not worth shipping.
"""

import uuid
import structlog

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Tuple

from fastapi import APIRouter
from fastapi.concurrency import run_in_threadpool
from pydantic import BaseModel, Field, field_validator, model_validator

from syncai_backend.database.models import MapPoint, TaskTemplate
from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.gateways.workflow.schema import (
    MoveParams,
    ScheduleTask,
    ScheduleTrigger,
    Step,
    StepParams,
    StepType,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.interfaces.rest.routers.schedule import (
    ScheduleResponse,
    ScheduleTriggerRequest,
)
from syncai_backend.interfaces.rest.routers.task import StepRequest
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo


# --- Request models ---------------------------------------------------------


class TemplateStepRequest(StepRequest):
    """A step as it is *stored*: a StepRequest plus where its numbers came from.

    Subclassed rather than restated, so ``validate_step_params`` stays the only
    place that knows which step types carry which params -- and so a STANDUP with
    params keeps being a boundary 422 rather than a 500 raised while building the
    Step.
    """

    vertex_id: Optional[uuid.UUID] = Field(
        default=None,
        description=(
            "The map vertex this MOVE step's coordinates were taken from. "
            "`params` is kept as a snapshot either way: a dispatch prefers the "
            "vertex's current pose and falls back to the snapshot once the "
            "vertex has been deleted."
        ),
    )

    # A distinct name from StepRequest._check_params on purpose: reusing that name
    # would *override* the inherited validator rather than add to it, and the
    # params check would silently stop running.
    @model_validator(mode="after")
    def _check_vertex_ref(self) -> "TemplateStepRequest":
        if self.vertex_id is not None and self.type is not StepType.MOVE:
            raise ValueError(f"{self.type.value} step cannot reference a vertex")
        return self


class _StoredStep(TemplateStepRequest):
    """The persisted shape: a TemplateStepRequest plus the server-owned label.

    Private because it is not a wire shape. ``vertex_name`` is resolved from the
    vertex table at save time and is part of the *snapshot* -- a name is as much
    "what the operator picked" as a coordinate is, and it is the difference
    between the UI saying "vertex 'dock' was deleted" and quoting a bare UUID. It
    is never used to re-find a vertex; ``vertex_id`` is the only reference.

    It lives on the stored model and not on the request model so a client cannot
    supply a name that disagrees with the vertex it claims to come from -- the
    same habit as the map router omitting ``map_name`` from a vertex body rather
    than accepting and ignoring it.
    """

    vertex_name: Optional[str] = Field(default=None)


class TaskTemplateRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=255)
    description: str = Field(default="", max_length=1000)
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "The map whose frame the MOVE coordinates are in. Required when the "
            "template has any MOVE step; must be omitted when it has none."
        ),
    )
    # min_length=1, unlike TaskRequest which accepts steps: [] and starts a
    # workflow that instantly COMPLETEs. An empty *template* is worse than an
    # empty dispatch: you save it and can then never run it. TaskRequest is left
    # alone -- tightening an endpoint three other call sites use is not this
    # feature's business.
    steps: List[TemplateStepRequest] = Field(..., min_length=1)

    @field_validator("name")
    @classmethod
    def _strip_name(cls, value: str) -> str:
        # min_length=1 alone lets " " through, and a library row whose label is a
        # space can neither be picked nor renamed by the operator who made it.
        stripped = value.strip()
        if not stripped:
            raise ValueError("name must not be blank")
        return stripped


class TaskTemplateUpdateRequest(BaseModel):
    """Fields to change. All optional; omitted ones are left alone.

    ``steps`` replaces the whole list -- there is no per-step patch, because a
    step list is edited by reordering and inserting, not field by field.
    """

    name: Optional[str] = Field(default=None, min_length=1, max_length=255)
    description: Optional[str] = Field(default=None, max_length=1000)
    map_name: Optional[str] = None
    steps: Optional[List[TemplateStepRequest]] = Field(default=None, min_length=1)

    @field_validator("name")
    @classmethod
    def _strip_name(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        stripped = value.strip()
        if not stripped:
            raise ValueError("name must not be blank")
        return stripped


class TaskTemplateScheduleRequest(BaseModel):
    id: str = Field(
        ..., min_length=1, description="Temporal schedule id to register"
    )
    # Reused rather than restated, so the "exactly one of cron / interval_seconds"
    # validator is not duplicated.
    trigger: ScheduleTriggerRequest = Field(..., description="When the schedule fires")


# --- Response models --------------------------------------------------------


class VertexRefStatus(str, Enum):
    NONE = "NONE"
    """The step references no vertex; its coordinates were typed by hand."""

    CURRENT = "CURRENT"
    """The vertex exists; `resolved_params` and `vertex_name` come from it."""

    MISSING = "MISSING"
    """The vertex is gone; `resolved_params` is the snapshot taken at save time."""


class TemplateStepResponse(BaseModel):
    id: str
    type: StepType
    params: Optional[StepParams] = Field(
        default=None, description="The snapshot, exactly as it was saved."
    )
    vertex_id: Optional[uuid.UUID] = None
    vertex_name: Optional[str] = Field(
        default=None,
        description=(
            "The vertex's CURRENT name when vertex_status is CURRENT, the "
            "snapshot label when MISSING, null otherwise. vertex_status is what "
            "says which of those you are reading."
        ),
    )
    vertex_status: VertexRefStatus
    resolved_params: Optional[StepParams] = Field(
        default=None,
        description=(
            "What a dispatch should send right now. Equal to `params` for a "
            "non-MOVE step, a hand-typed MOVE, or a MISSING vertex; the vertex's "
            "live pose when CURRENT."
        ),
    )


class TaskTemplateResponse(BaseModel):
    """One stored template.

    ``id`` is this row's UUID and is **not** a task id -- it cannot be passed to
    ``GET /api/v1/tasks/{id}``, which wants the Temporal workflow id a dispatch
    returns. That family is a *run*; this is the template it was run from.
    """

    id: uuid.UUID
    name: str
    description: str
    map_name: Optional[str]
    steps: List[TemplateStepResponse]
    map_matches_active: bool = Field(
        ...,
        description=(
            "True when this template's map is the one the robot is on, or the "
            "template is map-independent and so has nothing to mismatch. A "
            "client's dispatch guard: coordinates from another map's frame point "
            "somewhere else in the loaded one."
        ),
    )
    missing_vertex_count: int = Field(
        ..., description="How many steps have vertex_status MISSING."
    )
    created_at: datetime
    updated_at: datetime


class DeleteResponse(BaseModel):
    message: str


# --- Validation helpers -----------------------------------------------------
#
# These raise ValueError and the handlers translate to BadRequestError, rather
# than being model_validators, for two reasons.
#
# A 422's `detail` is a validation *array*, not a sentence -- the frontend files
# go out of their way to keep that off an operator's screen -- while
# `{"detail": "a template with MOVE steps must name the map its coordinates are in"}`
# renders as-is.
#
# And a PUT may send `{"steps": [...]}` alone, so the conflict is between the body
# and the *stored row*, which no request-schema validator can see. Checking merged
# state in the handler gives both verbs one status and one sentence.


def _check_step_ids(steps: List[TemplateStepRequest]) -> None:
    """Reject duplicate step ids.

    Nothing enforces this on TaskRequest, where a duplicate is only a confusing
    TaskStateResponse for one run. In a *template* it is a durable ambiguity,
    re-dispatched every time the row is used.
    """
    seen: set[str] = set()
    duplicates: list[str] = []
    for step in steps:
        if step.id in seen and step.id not in duplicates:
            duplicates.append(step.id)
        seen.add(step.id)
    if duplicates:
        raise ValueError(f"duplicate step ids: {duplicates}")


def _check_map_scope(map_name: Optional[str], steps: List[TemplateStepRequest]) -> None:
    """Any MOVE step means a map name is required; none means it is forbidden.

    The rule is "does it contain a MOVE", not "does it reference a vertex": a
    hand-typed (x, y, theta) lives in a map's frame every bit as much as a vertex
    does, so it is equally map-specific.
    """
    has_move = any(step.type is StepType.MOVE for step in steps)
    if has_move and not map_name:
        raise ValueError(
            "a template with MOVE steps must name the map its coordinates are in"
        )
    if not has_move and map_name:
        raise ValueError(
            "a template with no MOVE step is map-independent; omit map_name"
        )


def init_task_template_router(
    logger: structlog.stdlib.BoundLogger,
    task_template_repo: TaskTemplateRepo,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    workflow_gw: WorkflowGateway,
) -> APIRouter:
    task_template_router = APIRouter(prefix="", tags=["Task template"])

    # --- Helpers ------------------------------------------------------------

    def _require(task_id: uuid.UUID) -> TaskTemplate:
        row = task_template_repo.get_task_template(task_id=task_id)
        if row is None:
            raise NotFoundError(f"Task template {task_id} was not found.")
        return row

    def _require_map(map_name: Optional[str]) -> None:
        """400 -- not 404 -- when a body names a map this robot does not have.

        The map router's own ``_require`` raises NotFoundError because there the
        map name *is* the path: the addressed resource genuinely is not there.
        Here the addressed resource is the task-templates collection, which
        exists; the body is what is wrong, and a 404 would tell the client the
        collection is missing.
        """
        if map_name is None:
            return
        if map_catalog_repo.get_map(map_name) is None:
            raise BadRequestError(f"No map named '{map_name}' on this robot.")

    def _vertices(map_name: Optional[str]) -> Dict[uuid.UUID, MapPoint]:
        if map_name is None:
            return {}
        return {v.id: v for v in map_repo.list_vertices(map=map_name)}

    def _require_vertex_refs(
        map_name: Optional[str],
        steps: List[TemplateStepRequest],
        vertices: Dict[uuid.UUID, MapPoint],
    ) -> None:
        """Every referenced vertex must exist and belong to this template's map.

        The two failures are reported *separately*, unlike the map router's
        ``_require_vertex`` which folds "not there" and "belongs elsewhere" into
        one 404. That rule exists because a URL had already claimed the vertex was
        in a given map, so distinguishing them would confirm an id to a caller who
        addressed the wrong map. Here the body named both the map and the vertex,
        there is no scope being probed, and at save time "vertex X is on 'office',
        not 'warehouse'" is the difference between a fixable client bug and a
        mystery.
        """
        for step in steps:
            if step.vertex_id is None:
                continue
            if step.vertex_id in vertices:
                continue
            other = map_repo.get_vertex(vertex_id=step.vertex_id)
            if other is None:
                raise BadRequestError(
                    f"Step '{step.id}' references vertex {step.vertex_id}, "
                    "which does not exist."
                )
            raise BadRequestError(
                f"Step '{step.id}' references vertex {step.vertex_id}, which is "
                f"on map '{other.map}', not '{map_name}'."
            )

    def _check(map_name: Optional[str], steps: List[TemplateStepRequest]) -> None:
        try:
            _check_step_ids(steps)
            _check_map_scope(map_name, steps)
        except ValueError as exc:
            raise BadRequestError(str(exc))

    def _stored_steps(
        steps: List[TemplateStepRequest], vertices: Dict[uuid.UUID, MapPoint]
    ) -> list[dict]:
        """Freeze the request's steps into the JSON column's element shape.

        The snapshot coordinates are stored **verbatim as posted** -- the server
        does not overwrite them with the vertex's current pose even though it has
        the vertex in hand. The composer rounds prefilled values and documents
        that what is in the field is what gets sent; silently rewriting a
        submitted body is what this codebase avoids; and an exact-at-save-time
        snapshot buys nothing, because every read resolves against the live vertex
        anyway.

        ``mode="json"`` is required -- uuid.UUID is not JSON-serialisable and the
        column's serialiser is json.dumps. No ``exclude_none``: explicit nulls are
        what make a *missing* key unambiguously mean "written by an older
        backend", which is the forward-compatibility property that justifies JSON
        storage. No ``by_alias``: MoveParams inherits BaseSchema's camelCase alias
        generator, and dumping without aliases keeps the blob snake_case, matching
        the REST vocabulary and what a human sees in psql.
        """
        stored: list[dict] = []
        for step in steps:
            vertex = vertices.get(step.vertex_id) if step.vertex_id else None
            stored.append(
                _StoredStep(
                    id=step.id,
                    type=step.type,
                    params=step.params,
                    vertex_id=step.vertex_id,
                    vertex_name=vertex.name if vertex else None,
                ).model_dump(mode="json")
            )
        return stored

    def _read_steps(row: TaskTemplate) -> List[_StoredStep]:
        return [_StoredStep.model_validate(entry) for entry in row.steps]

    def _step_response(
        stored: _StoredStep, vertices: Dict[uuid.UUID, MapPoint]
    ) -> TemplateStepResponse:
        if stored.vertex_id is None:
            return TemplateStepResponse(
                id=stored.id,
                type=stored.type,
                params=stored.params,
                vertex_id=None,
                vertex_name=None,
                vertex_status=VertexRefStatus.NONE,
                resolved_params=stored.params,
            )

        vertex = vertices.get(stored.vertex_id)
        if vertex is None:
            return TemplateStepResponse(
                id=stored.id,
                type=stored.type,
                params=stored.params,
                vertex_id=stored.vertex_id,
                # The snapshot label: the only human handle for what is gone.
                vertex_name=stored.vertex_name,
                vertex_status=VertexRefStatus.MISSING,
                resolved_params=stored.params,
            )

        return TemplateStepResponse(
            id=stored.id,
            type=stored.type,
            params=stored.params,
            vertex_id=stored.vertex_id,
            # The vertex's *current* name, so a rename shows through.
            vertex_name=vertex.name,
            vertex_status=VertexRefStatus.CURRENT,
            # This is the feature: move a dock on the map and every template
            # that references it dispatches to the new pose.
            resolved_params=MoveParams(x=vertex.x, y=vertex.y, theta=vertex.theta),
        )

    def _response(
        row: TaskTemplate,
        active_name: Optional[str],
        vertices: Dict[uuid.UUID, MapPoint],
    ) -> TaskTemplateResponse:
        steps = [_step_response(stored, vertices) for stored in _read_steps(row)]
        return TaskTemplateResponse(
            id=row.id,
            name=row.name,
            description=row.description,
            map_name=row.map_name,
            steps=steps,
            # A map-independent template has nothing to mismatch.
            map_matches_active=row.map_name is None or row.map_name == active_name,
            missing_vertex_count=sum(
                1 for step in steps if step.vertex_status is VertexRefStatus.MISSING
            ),
            created_at=row.created_at,
            updated_at=row.updated_at,
        )

    # --- Routes -------------------------------------------------------------
    #
    # `response_model_by_alias=False` on every route returning Step objects, and
    # it is load-bearing: FastAPI defaults that flag to True, and everything
    # under BaseSchema inherits the to_camel generator, so the default would
    # answer `error_msg` as `errorMsg` (and any future multi-word params field
    # likewise) while the request side accepts both spellings -- a silent
    # asymmetry at the one boundary the frontend documents as snake_case-only.
    # MoveParams happens to be unaffected (all single words), which is exactly
    # why it would go unnoticed until the first multi-word field shipped.
    #
    # Plain `def`, not `async def`, for everything except /schedule: these touch
    # psycopg2, so FastAPI must run them in its worker threadpool rather than on
    # the event loop -- same reason the map router's handlers are sync.

    @task_template_router.post(
        "/api/v1/task_templates",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def create_task_template(req: TaskTemplateRequest):
        _check(req.map_name, req.steps)
        _require_map(req.map_name)
        vertices = _vertices(req.map_name)
        _require_vertex_refs(req.map_name, req.steps, vertices)

        row = task_template_repo.create_task_template(
            name=req.name,
            description=req.description,
            map_name=req.map_name,
            steps=_stored_steps(req.steps, vertices),
        )
        return _response(row, map_catalog_repo.active_name(), vertices)

    @task_template_router.get(
        "/api/v1/task_templates",
        response_model=List[TaskTemplateResponse],
        response_model_by_alias=False,
    )
    def list_task_templates(map_name: Optional[str] = None):
        """Every template, or one map's plus the map-independent ones.

        The console deliberately does not pass ``map_name`` -- it fetches
        everything and filters client-side so it can report how many rows it hid,
        because silently hiding another map's templates is how an operator
        concludes their work was lost. The parameter is here for other consumers.
        """
        rows = task_template_repo.list_task_templates(map_name=map_name)
        active_name = map_catalog_repo.active_name()

        # One vertex query per distinct map among the rows, not per row.
        by_map: Dict[Optional[str], Dict[uuid.UUID, MapPoint]] = {}
        for row in rows:
            if row.map_name not in by_map:
                by_map[row.map_name] = _vertices(row.map_name)

        return [_response(row, active_name, by_map[row.map_name]) for row in rows]

    @task_template_router.get(
        "/api/v1/task_templates/{id}",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def get_task_template(id: uuid.UUID):
        row = _require(id)
        return _response(row, map_catalog_repo.active_name(), _vertices(row.map_name))

    @task_template_router.put(
        "/api/v1/task_templates/{id}",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def update_task_template(id: uuid.UUID, req: TaskTemplateUpdateRequest):
        row = _require(id)
        changes = req.model_dump(exclude_unset=True)

        # Merged before checking, because the invariant spans the body and the
        # stored row: a PUT that only drops the last MOVE step must be allowed to
        # clear map_name, and a PUT that only sets map_name has to be checked
        # against the steps already on disk. Re-validating the stored steps
        # against the possibly-new map is also what makes "template moved to
        # another map while its steps still point at the old map's vertices"
        # unrepresentable.
        merged_map = changes["map_name"] if "map_name" in changes else row.map_name
        merged_steps: List[TemplateStepRequest] = (
            req.steps if req.steps is not None else list(_read_steps(row))
        )

        _check(merged_map, merged_steps)
        _require_map(merged_map)
        vertices = _vertices(merged_map)
        _require_vertex_refs(merged_map, merged_steps, vertices)

        if "steps" in changes and req.steps is not None:
            changes["steps"] = _stored_steps(req.steps, vertices)

        updated = task_template_repo.update_task_template(task_id=id, **changes)
        if updated is None:
            raise NotFoundError(f"Task template {id} was not found.")
        return _response(updated, map_catalog_repo.active_name(), vertices)

    @task_template_router.delete(
        "/api/v1/task_templates/{id}", response_model=DeleteResponse
    )
    def delete_task_template(id: uuid.UUID):
        _require(id)
        task_template_repo.delete_task_template(task_id=id)
        return DeleteResponse(message=f"Task template {id} has been deleted.")

    @task_template_router.post(
        "/api/v1/task_templates/{id}/schedule", response_model=ScheduleResponse
    )
    async def schedule_task_template(id: uuid.UUID, req: TaskTemplateScheduleRequest):
        """Freeze this template's CURRENT resolution into a Temporal schedule.

        ``async``, unlike its CRUD siblings, because the dominant cost is the
        create_schedule gRPC. The blocking DB work is pushed to the threadpool in
        one hop for the same reason those siblings are sync: psycopg2 blocks, and
        a wedged Postgres must not stall the event loop and the telemetry
        WebSocket with it.

        A schedule stores **concrete** steps in Temporal's
        ScheduleActionStartWorkflow args and nothing re-reads them, so later
        vertex edits do not reach a scheduled run. Hence the two refusals below
        that immediate dispatch does not make: a scheduled run is unattended, so
        it does not get to silently use another map's frame or a stale snapshot of
        a deleted vertex. An operator dispatching by hand has just been shown the
        warning and may proceed.
        """

        def _freeze() -> Tuple[TaskTemplate, List[Step]]:
            row = _require(id)
            active_name = map_catalog_repo.active_name()

            if row.map_name is not None and row.map_name != active_name:
                raise BadRequestError(
                    f"Template '{row.name}' is for map '{row.map_name}', but the "
                    f"robot has {active_name or 'no map'} loaded. A scheduled run "
                    "is unattended, so it will not be registered against another "
                    "map's coordinate frame."
                )

            vertices = _vertices(row.map_name)
            steps: List[Step] = []
            for stored in _read_steps(row):
                resolved = _step_response(stored, vertices)
                if resolved.vertex_status is VertexRefStatus.MISSING:
                    raise BadRequestError(
                        f"Step '{stored.id}' references vertex "
                        f"'{stored.vertex_name or stored.vertex_id}', which no "
                        "longer exists. A scheduled run is unattended, so it will "
                        "not be registered against a stale snapshot — re-point or "
                        "remove the step first."
                    )
                # Note what is NOT carried over: vertex_id / vertex_name. A
                # template's provenance never enters the Temporal schema.
                steps.append(
                    Step(
                        id=stored.id,
                        type=stored.type,
                        params=resolved.resolved_params,
                    )
                )
            return row, steps

        row, steps = await run_in_threadpool(_freeze)

        await workflow_gw.create_schedule(
            schedule=ScheduleTask(
                id=req.id,
                trigger=ScheduleTrigger(
                    cron=req.trigger.cron,
                    interval_seconds=req.trigger.interval_seconds,
                    timezone=req.trigger.timezone,
                ),
                definition=WorkflowTaskDefinition(steps=steps),
                map_name=row.map_name,
                task_template_id=str(row.id),
                task_template_name=row.name,
            )
        )

        return ScheduleResponse(
            id=req.id,
            message=(
                f"Schedule {req.id} runs '{row.name}' ({len(steps)} steps), "
                "frozen at the vertices' current positions."
            ),
        )

    return task_template_router
