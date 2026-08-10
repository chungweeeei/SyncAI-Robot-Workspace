"""Tests for /api/v1/tasks and /api/v1/active_tasks — the REST projection only.

Same pattern as ``test_schedule_router.py``: a stub gateway records what the
router hands it and answers canned views, so these tests pin the boundary
(request validation, response shapes, exception mapping) without any Temporal.
The gateway's own behaviour lives in ``test_workflow_gateway.py``.
"""

from datetime import datetime, timezone

import pytest

pytest.importorskip("httpx")
pytest.importorskip("temporalio")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.exceptions import (  # noqa: E402
    BadRequestError,
    ConflictError,
    NotFoundError,
)
from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    ActiveTask,
    MoveParams,
    Step,
    StepStatus,
    StepType,
    TaskSource,
    TaskState,
)
from syncai_backend.interfaces.rest.routers.task import init_task_router  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubWorkflowGateway:
    def __init__(self):
        self.started = []
        self.cancelled = []
        self.state = TaskState(
            id="robot01-task-001",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="step1",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.IN_PROGRESS,
                    error_msg=None,
                )
            ],
        )
        self.active = (
            [
                ActiveTask(
                    id="robot01-sched-001-2026-08-10T09:00:00Z",
                    run_id="run-1",
                    status="IN_PROGRESS",
                    started_at=datetime(2026, 8, 10, 9, 0, tzinfo=timezone.utc),
                    source=TaskSource.SCHEDULE,
                    schedule_id="robot01-sched-001",
                )
            ],
            datetime(2026, 8, 10, 9, 0, 30, tzinfo=timezone.utc),
        )

    async def start_task(self, request):
        self.started.append(request)

    async def get_task_state(self, task_id):
        return self.state

    async def cancel_task(self, task_id):
        self.cancelled.append(task_id)

    async def list_active_tasks(self):
        return self.active


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def client(logger, workflow_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_task_router(logger=logger, workflow_gw=workflow_gw))
    return TestClient(app)


def _move_task(task_id="robot01-task-001"):
    return {
        "id": task_id,
        "timestamp": 1782786519,
        "steps": [
            {"id": "step1", "type": "MOVE", "params": {"x": 1.0, "y": 2.0, "theta": 90.0}}
        ],
    }


class TestTriggerTask:
    def test_post_builds_the_workflow_task(self, client, workflow_gw):
        body = client.post("/api/v1/tasks", json=_move_task()).json()

        assert body["status"] == "PENDING"
        task = workflow_gw.started[0]
        assert task.id == "robot01-task-001"
        step = task.definition.steps[0]
        assert step.type is StepType.MOVE
        assert step.params == MoveParams(x=1.0, y=2.0, theta=90.0)

    def test_a_mismatched_step_body_is_a_422(self, client, workflow_gw):
        # STANDUP takes no params; StepRequest's validator must reject this at
        # the boundary rather than let it 500 inside the handler.
        task = _move_task()
        task["steps"] = [
            {"id": "s1", "type": "STANDUP", "params": {"x": 0.0, "y": 0.0, "theta": 0.0}}
        ]

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_duplicate_id_surfaces_as_400(self, client, workflow_gw):
        async def _raise(request):
            raise BadRequestError(f"Task {request.id} already exists")

        workflow_gw.start_task = _raise

        response = client.post("/api/v1/tasks", json=_move_task())

        assert response.status_code == 400
        assert "already exists" in response.json()["detail"]

    def test_a_busy_robot_surfaces_as_409(self, client, workflow_gw):
        # The gateway's one-task-at-a-time gate; the router only translates.
        async def _raise(request):
            raise ConflictError("Robot is busy: task robot01-task-000 is running")

        workflow_gw.start_task = _raise

        response = client.post("/api/v1/tasks", json=_move_task())

        assert response.status_code == 409
        assert "robot01-task-000" in response.json()["detail"]


class TestTaskState:
    def test_get_projects_step_status_only(self, client):
        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["status"] == "IN_PROGRESS"
        # StepState is status/error only — the definition (params) stays out.
        assert body["steps"] == [
            {"id": "step1", "status": "IN_PROGRESS", "error_msg": ""}
        ]

    def test_get_unknown_task_is_404(self, client, workflow_gw):
        async def _raise(task_id):
            raise NotFoundError(f"Task {task_id} not found")

        workflow_gw.get_task_state = _raise

        assert client.get("/api/v1/tasks/missing").status_code == 404


class TestActiveTasks:
    def test_list_carries_provenance_and_as_of(self, client):
        body = client.get("/api/v1/active_tasks").json()

        assert body["as_of"] == "2026-08-10T09:00:30Z"
        task = body["tasks"][0]
        assert task["source"] == "SCHEDULE"
        assert task["schedule_id"] == "robot01-sched-001"

    def test_nothing_running_is_an_empty_list_not_a_404(self, client, workflow_gw):
        workflow_gw.active = ([], datetime(2026, 8, 10, tzinfo=timezone.utc))

        response = client.get("/api/v1/active_tasks")

        assert response.status_code == 200
        assert response.json()["tasks"] == []


class TestCancelTask:
    def test_delete_requests_the_cancel(self, client, workflow_gw):
        body = client.delete("/api/v1/tasks/robot01-task-001").json()

        assert workflow_gw.cancelled == ["robot01-task-001"]
        assert body["status"] == "CANCELED"
