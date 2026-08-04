"""Tests for /api/v1/schedules and the gateway's schedule-step decoding.

Two things are pinned here.

The **list/describe asymmetry**: a schedule's frozen step list is reachable only
through ``describe()``. ``list_schedules()`` yields a ``ScheduleListDescription``
whose action is a ``ScheduleListActionStartWorkflow`` -- one field, the workflow
type name -- so the collection endpoint answers ``steps: []`` by construction,
not by omission. Faking it would cost a describe RPC per row on first paint.

And ``response_model_by_alias=False`` on the get route. ``MoveParams`` /
``ArtifactParams`` inherit ``BaseSchema``'s camelCase alias generator, and
FastAPI defaults that flag to True, so without it the response would spell an
ARTIFACT step's params ``artifactId`` / ``waitFor`` / ``waitTimeoutSeconds``
while the request side accepts both. ``MoveParams`` is all single words and
cannot catch this -- which is why the test uses an ARTIFACT step.
"""

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip("httpx")
pytest.importorskip("temporalio")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    ArtifactParams,
    MoveParams,
    PickupCommand,
    ScheduleTrigger,
    ScheduleView,
    Step,
    StepType,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import _read_steps  # noqa: E402
from syncai_backend.interfaces.rest.routers.schedule import (  # noqa: E402
    init_schedule_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


ARTIFACT_STEP = Step(
    id="1-artifact",
    type=StepType.ARTIFACT,
    params=ArtifactParams(
        artifact_id="conveyor01",
        command=PickupCommand(action="pickup"),
        wait_for="handoff",
        wait_timeout_seconds=120,
    ),
)
MOVE_STEP = Step(
    id="2-move", type=StepType.MOVE, params=MoveParams(x=1.0, y=2.0, theta=90.0)
)


def _view(**overrides):
    body = {
        "id": "sched-1",
        "trigger": ScheduleTrigger(cron="0 9 * * 1-5", timezone="Asia/Taipei"),
        "paused": False,
        "next_run_times": [],
        "map_name": "full",
        "saved_task_id": "0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55",
        "saved_task_name": "Morning patrol",
        "steps": [ARTIFACT_STEP, MOVE_STEP],
    }
    body.update(overrides)
    return ScheduleView(**body)


class _StubWorkflowGateway:
    def __init__(self, view):
        self.view = view
        self.created = []

    async def create_schedule(self, schedule):
        self.created.append(schedule)

    async def get_schedule(self, schedule_id):
        return self.view

    async def list_schedules(self):
        return [self.view]


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway(_view())


@pytest.fixture
def client(logger, workflow_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_schedule_router(logger=logger, workflow_gw=workflow_gw))
    return TestClient(app)


# --- The REST projection ----------------------------------------------------


def test_get_carries_steps_and_provenance(client):
    body = client.get("/api/v1/schedules/sched-1").json()

    assert [s["id"] for s in body["steps"]] == ["1-artifact", "2-move"]
    assert body["map_name"] == "full"
    assert body["saved_task_name"] == "Morning patrol"


def test_get_serialises_step_params_in_snake_case(client):
    """The response_model_by_alias=False regression test."""
    params = client.get("/api/v1/schedules/sched-1").json()["steps"][0]["params"]

    assert "artifact_id" in params and "artifactId" not in params
    assert "wait_for" in params and "waitFor" not in params
    assert "wait_timeout_seconds" in params and "waitTimeoutSeconds" not in params


def test_list_never_carries_steps(client, workflow_gw):
    """Even though the stubbed view holds them: Temporal's schedule *list* API
    cannot reach the start-workflow args, so the collection endpoint reports the
    truth rather than pretending with an extra describe per row."""
    assert workflow_gw.view.steps  # the stub does have steps

    rows = client.get("/api/v1/schedules").json()
    assert rows[0]["steps"] == []
    # Provenance, unlike steps, *is* readable from the list path via the memo.
    assert rows[0]["map_name"] == "full"


def test_create_passes_the_map_label_through(client, workflow_gw):
    res = client.post(
        "/api/v1/schedules",
        json={
            "id": "sched-2",
            "trigger": {"interval_seconds": 900},
            "map_name": "full",
            "steps": [{"id": "1-standup", "type": "STANDUP"}],
        },
    )
    assert res.status_code == 200
    assert workflow_gw.created[-1].map_name == "full"


# --- _read_steps ------------------------------------------------------------
#
# Module-level, and taking a logger, precisely so it can be exercised against a
# hand-built description like these. Driven with asyncio.run rather than
# pytest-asyncio: the container has no such plugin, and taking on a test
# dependency for five assertions against one coroutine is not worth it.


def _desc(action, data_converter=None):
    return SimpleNamespace(
        id="sched-1",
        schedule=SimpleNamespace(action=action),
        data_converter=data_converter,
    )


class _StubConverter:
    """Stands in for the ScheduleDescription's data_converter."""

    def __init__(self, result=None, raises=False):
        self.result = result
        self.raises = raises

    async def decode(self, payloads, type_hints):
        if self.raises:
            raise RuntimeError("cannot map this payload")
        return self.result


def test_read_steps_passes_through_already_decoded_args(logger):
    """The args are Payloads only on the describe path; a ScheduleTask this
    process just built, and a test's stub, still hold python objects."""
    from temporalio.client import ScheduleActionStartWorkflow

    task = WorkflowTask(
        id="sched-1", definition=WorkflowTaskDefinition(steps=[MOVE_STEP])
    )
    action = ScheduleActionStartWorkflow(
        "RobotWorkflow", args=[task], id="sched-1", task_queue="q"
    )

    steps = asyncio.run(_read_steps(logger, _desc(action)))
    assert [s.id for s in steps] == ["2-move"]


def test_read_steps_decodes_payload_args(logger):
    from temporalio.api.common.v1 import Payload
    from temporalio.client import ScheduleActionStartWorkflow

    task = WorkflowTask(
        id="sched-1", definition=WorkflowTaskDefinition(steps=[ARTIFACT_STEP])
    )
    action = ScheduleActionStartWorkflow(
        "RobotWorkflow", args=[Payload()], id="sched-1", task_queue="q"
    )

    steps = asyncio.run(_read_steps(logger, _desc(action, _StubConverter(result=[task]))))
    assert [s.id for s in steps] == ["1-artifact"]


def test_read_steps_is_empty_for_a_foreign_action(logger):
    steps = asyncio.run(_read_steps(logger, _desc(SimpleNamespace(args=["whatever"]))))
    assert steps == []


def test_read_steps_is_empty_for_an_actionless_description(logger):
    steps = asyncio.run(_read_steps(logger, _desc(None)))
    assert steps == []


def test_read_steps_swallows_a_decode_failure(logger):
    """Never fatal: a schedule created outside this API must still report its
    trigger and next run times -- the same policy get_task_state's step query
    uses."""
    from temporalio.api.common.v1 import Payload
    from temporalio.client import ScheduleActionStartWorkflow

    action = ScheduleActionStartWorkflow(
        "RobotWorkflow", args=[Payload()], id="sched-1", task_queue="q"
    )

    steps = asyncio.run(_read_steps(logger, _desc(action, _StubConverter(raises=True))))
    assert steps == []
