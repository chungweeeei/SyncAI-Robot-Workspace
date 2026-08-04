"""Tests for /api/v1/saved_tasks.

Same shape as test_maps_router.py: the router is mounted on a bare FastAPI app
with the production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. The repos are real, over a tmp_path maps
tree and in-memory SQLite; only the Temporal gateway is stubbed.

The heart of the file is test_resolution_*: a saved MOVE step keeps a coordinate
*snapshot* and a *vertex reference*, and a read reports what a dispatch should
send now. Every one of those tests posts a snapshot that deliberately disagrees
with its vertex, so "prefer the vertex's current pose" is pinned rather than
accidentally satisfied by the two happening to match.
"""

import uuid

import pytest

pytest.importorskip("cv2")
pytest.importorskip("httpx")
pytest.importorskip("yaml")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers.saved_task import (  # noqa: E402
    init_saved_task_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubWorkflowGateway:
    """Records create_schedule calls instead of talking to Temporal."""

    def __init__(self):
        self.schedules = []

    async def create_schedule(self, schedule):
        self.schedules.append(schedule)


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def client(logger, saved_task_repo, map_repo, catalog_repo, workflow_gw, tmp_path,
           monkeypatch):
    """A client whose active map is 'full'; 'rawonly' exists but is not active."""
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_saved_task_router(
            logger=logger,
            saved_task_repo=saved_task_repo,
            map_repo=map_repo,
            map_catalog_repo=catalog_repo,
            workflow_gw=workflow_gw,
        )
    )
    return TestClient(app)


@pytest.fixture
def dock(map_repo):
    """A vertex on the active map, at a pose no snapshot in this file uses."""
    return map_repo.create_vertices(
        [{"name": "dock", "type": "GENERAL", "map_name": "full",
          "x": 3.0, "y": -1.5, "theta": 90.0}]
    )[0]


# A snapshot that is deliberately NOT the dock's pose.
SNAPSHOT = {"x": 0.0, "y": 0.0, "theta": 0.0}
STANDUP = {"id": "2-standup", "type": "STANDUP"}


def _move(vertex_id=None, step_id="1-move", params=None):
    step = {"id": step_id, "type": "MOVE", "params": params or dict(SNAPSHOT)}
    if vertex_id is not None:
        step["vertex_id"] = str(vertex_id)
    return step


def _create(client, **overrides):
    body = {"name": "patrol", "map_name": "full", "steps": [_move()]}
    body.update(overrides)
    return client.post("/api/v1/saved_tasks", json=body)


# --- Resolution -------------------------------------------------------------


def test_resolution_prefers_the_vertexs_current_pose(client, dock):
    created = _create(client, steps=[_move(dock.id), STANDUP])
    assert created.status_code == 200

    step = created.json()["steps"][0]
    assert step["vertex_status"] == "CURRENT"
    assert step["resolved_params"] == {"x": 3.0, "y": -1.5, "theta": 90.0}
    # The snapshot is kept verbatim; the server does not overwrite it.
    assert step["params"] == SNAPSHOT
    assert step["vertex_name"] == "dock"
    assert created.json()["missing_vertex_count"] == 0

    posture = created.json()["steps"][1]
    assert posture["vertex_status"] == "NONE"
    assert posture["params"] is None and posture["resolved_params"] is None


def test_resolution_follows_a_moved_vertex(client, map_repo, dock):
    """This is the feature: move a dock and every saved route that references it
    dispatches to the new pose, without the route being re-saved."""
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    map_repo.update_vertex(dock.id, x=9.0)

    step = client.get(f"/api/v1/saved_tasks/{task_id}").json()["steps"][0]
    assert step["resolved_params"]["x"] == 9.0
    assert step["params"] == SNAPSHOT  # untouched


def test_resolution_reports_the_vertexs_current_name(client, map_repo, dock):
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    map_repo.update_vertex(dock.id, name="dock-renamed")

    step = client.get(f"/api/v1/saved_tasks/{task_id}").json()["steps"][0]
    assert step["vertex_name"] == "dock-renamed"


def test_resolution_falls_back_to_the_snapshot_when_the_vertex_is_gone(
    client, map_repo, dock
):
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    map_repo.delete_vertex(dock.id)

    body = client.get(f"/api/v1/saved_tasks/{task_id}").json()
    step = body["steps"][0]
    assert step["vertex_status"] == "MISSING"
    assert step["resolved_params"] == SNAPSHOT
    # The snapshot label is the only human handle for a vertex that is gone.
    assert step["vertex_name"] == "dock"
    assert body["missing_vertex_count"] == 1


def test_hand_typed_move_resolves_to_itself(client):
    step = _create(client).json()["steps"][0]
    assert step["vertex_status"] == "NONE"
    assert step["vertex_id"] is None
    assert step["resolved_params"] == step["params"] == SNAPSHOT


# --- Map scoping ------------------------------------------------------------


def test_move_steps_require_a_map_name(client):
    res = _create(client, map_name=None)
    assert res.status_code == 400
    # A sentence, not a validation array -- that is the whole point of checking
    # this in the handler rather than in a model_validator.
    assert isinstance(res.json()["detail"], str)
    assert "must name the map" in res.json()["detail"]


def test_posture_only_tasks_must_not_name_a_map(client):
    res = _create(client, map_name="full", steps=[STANDUP])
    assert res.status_code == 400
    assert "map-independent" in res.json()["detail"]


def test_posture_only_task_is_map_independent(client):
    res = _create(client, map_name=None, steps=[STANDUP])
    assert res.status_code == 200
    assert res.json()["map_name"] is None
    # Nothing to mismatch, so it is runnable wherever the robot is.
    assert res.json()["map_matches_active"] is True


def test_unknown_map_is_400_not_404(client):
    """The addressed collection exists; the *body* is wrong.

    Deliberately different from the map router's ``_require``, where the map name
    is the path and a 404 is right.
    """
    res = _create(client, map_name="nope")
    assert res.status_code == 400
    assert "No map named 'nope'" in res.json()["detail"]


def test_a_task_for_an_inactive_map_saves_but_is_flagged(client):
    """Authoring for a map you are about to load is legitimate."""
    res = _create(client, map_name="rawonly")
    assert res.status_code == 200
    assert res.json()["map_matches_active"] is False


# --- Vertex references ------------------------------------------------------


def test_unknown_vertex_reference_is_rejected(client):
    res = _create(client, steps=[_move(uuid.uuid4())])
    assert res.status_code == 400
    assert "does not exist" in res.json()["detail"]


def test_vertex_from_another_map_names_both_maps(client, map_repo):
    """Reported separately from "does not exist", unlike the map router's folded
    404: the body named both the map and the vertex, so there is no scope being
    probed, and at save time the distinction is the whole diagnosis."""
    elsewhere = map_repo.create_vertices(
        [{"name": "other", "type": "GENERAL", "map_name": "rawonly",
          "x": 0.0, "y": 0.0, "theta": 0.0}]
    )[0]

    res = _create(client, steps=[_move(elsewhere.id)])
    assert res.status_code == 400
    assert "rawonly" in res.json()["detail"] and "full" in res.json()["detail"]


def test_posture_step_cannot_reference_a_vertex(client, dock):
    res = _create(client, map_name=None,
                  steps=[dict(STANDUP, vertex_id=str(dock.id))])
    assert res.status_code == 422


def test_posture_step_with_params_is_still_rejected(client):
    """Proves the inherited StepRequest validator still runs -- _check_vertex_ref
    is a distinct method name precisely so it adds to it rather than replacing it."""
    res = _create(client, map_name=None,
                  steps=[dict(STANDUP, params={"x": 1, "y": 2, "theta": 0})])
    assert res.status_code == 422


# --- Shape ------------------------------------------------------------------


def test_empty_step_list_is_rejected(client):
    """Diverges from TaskRequest, which accepts steps: [] and starts a workflow
    that instantly COMPLETEs. A saved empty task is worse: you save it and can
    then never run it."""
    assert _create(client, steps=[]).status_code == 422


def test_blank_name_is_rejected(client):
    assert _create(client, name="   ", map_name=None,
                   steps=[STANDUP]).status_code == 422


def test_duplicate_step_ids_are_rejected(client):
    res = _create(client, steps=[_move(), _move()])
    assert res.status_code == 400
    assert "duplicate step ids" in res.json()["detail"]


def test_non_uuid_path_id_is_422(client):
    assert client.get("/api/v1/saved_tasks/not-a-uuid").status_code == 422


def test_unknown_id_is_404(client):
    unknown = uuid.uuid4()
    assert client.get(f"/api/v1/saved_tasks/{unknown}").status_code == 404
    assert client.put(f"/api/v1/saved_tasks/{unknown}",
                      json={"name": "x"}).status_code == 404
    assert client.delete(f"/api/v1/saved_tasks/{unknown}").status_code == 404


# --- CRUD -------------------------------------------------------------------


def test_create_list_get_delete_round_trip(client):
    task_id = _create(client).json()["id"]

    assert [t["id"] for t in client.get("/api/v1/saved_tasks").json()] == [task_id]
    assert client.get(f"/api/v1/saved_tasks/{task_id}").json()["name"] == "patrol"
    assert client.delete(f"/api/v1/saved_tasks/{task_id}").status_code == 200
    assert client.get(f"/api/v1/saved_tasks/{task_id}").status_code == 404


def test_list_filter_returns_the_map_and_the_map_independent(client):
    _create(client, name="on-full")
    _create(client, name="on-rawonly", map_name="rawonly")
    _create(client, name="anywhere", map_name=None, steps=[STANDUP])

    names = [t["name"] for t in
             client.get("/api/v1/saved_tasks?map_name=full").json()]
    assert sorted(names) == ["anywhere", "on-full"]


def test_put_can_clear_the_map_scope_together_with_the_steps(client):
    task_id = _create(client).json()["id"]

    res = client.put(f"/api/v1/saved_tasks/{task_id}",
                     json={"map_name": None, "steps": [STANDUP]})
    assert res.status_code == 200
    assert res.json()["map_name"] is None


def test_put_cannot_clear_the_map_scope_while_move_steps_remain(client):
    """The invariant spans the body and the stored row, which is why it is checked
    against merged state in the handler."""
    task_id = _create(client).json()["id"]

    res = client.put(f"/api/v1/saved_tasks/{task_id}", json={"map_name": None})
    assert res.status_code == 400
    assert "must name the map" in res.json()["detail"]


def test_put_cannot_move_a_task_to_a_map_its_vertices_are_not_on(client, dock):
    """Re-validating the *stored* steps against the new map is what keeps
    "task on map A, vertices on map B" unrepresentable."""
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    res = client.put(f"/api/v1/saved_tasks/{task_id}", json={"map_name": "rawonly"})
    assert res.status_code == 400


def test_put_renames_without_touching_the_steps(client, dock):
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    res = client.put(f"/api/v1/saved_tasks/{task_id}", json={"name": "renamed"})
    assert res.status_code == 200
    assert res.json()["name"] == "renamed"
    assert res.json()["steps"][0]["vertex_status"] == "CURRENT"


# --- Schedule from a saved task ---------------------------------------------


def test_schedule_freezes_the_current_resolution(client, workflow_gw, dock):
    task_id = _create(client, steps=[_move(dock.id), STANDUP]).json()["id"]

    res = client.post(f"/api/v1/saved_tasks/{task_id}/schedule",
                      json={"id": "sched-1", "trigger": {"cron": "0 9 * * 1-5"}})
    assert res.status_code == 200

    registered = workflow_gw.schedules[-1]
    move = registered.definition.steps[0]
    # The vertex's current pose, not the snapshot.
    assert (move.params.x, move.params.y, move.params.theta) == (3.0, -1.5, 90.0)
    # Provenance rides in the memo, never in the Temporal step schema.
    assert registered.map_name == "full"
    assert registered.saved_task_id == task_id
    assert registered.saved_task_name == "patrol"
    assert not hasattr(move, "vertex_id")


def test_schedule_refuses_a_missing_vertex(client, workflow_gw, map_repo, dock):
    """No snapshot fallback for an unattended run -- unlike an immediate dispatch,
    where the operator has just been shown the warning."""
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]
    map_repo.delete_vertex(dock.id)

    res = client.post(f"/api/v1/saved_tasks/{task_id}/schedule",
                      json={"id": "sched-1", "trigger": {"cron": "0 9 * * 1-5"}})
    assert res.status_code == 400
    assert "no longer exists" in res.json()["detail"]
    assert workflow_gw.schedules == []


def test_schedule_refuses_a_task_for_an_inactive_map(client, workflow_gw):
    task_id = _create(client, map_name="rawonly").json()["id"]

    res = client.post(f"/api/v1/saved_tasks/{task_id}/schedule",
                      json={"id": "sched-1", "trigger": {"cron": "0 9 * * 1-5"}})
    assert res.status_code == 400
    assert "rawonly" in res.json()["detail"]
    assert workflow_gw.schedules == []


def test_schedule_reuses_the_exactly_one_trigger_validator(client, dock):
    """Proves ScheduleTriggerRequest is reused rather than restated."""
    task_id = _create(client, steps=[_move(dock.id)]).json()["id"]

    res = client.post(
        f"/api/v1/saved_tasks/{task_id}/schedule",
        json={"id": "sched-1",
              "trigger": {"cron": "0 9 * * 1-5", "interval_seconds": 300}},
    )
    assert res.status_code == 422


def test_schedule_on_a_map_independent_task_is_allowed(client, workflow_gw):
    task_id = _create(client, map_name=None, steps=[STANDUP]).json()["id"]

    res = client.post(f"/api/v1/saved_tasks/{task_id}/schedule",
                      json={"id": "sched-1", "trigger": {"interval_seconds": 900}})
    assert res.status_code == 200
    assert workflow_gw.schedules[-1].map_name is None
