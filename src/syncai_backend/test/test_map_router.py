"""Unit tests for the vertex routes under /api/v1/maps/{name}/vertices.

The rest of the catalogue is covered by test_maps_router.py, which mounts the
same router — the two files are split by fixture, not by router.

The router is mounted on a bare FastAPI app with the same exception handlers as
production; the map_repo fixture is a real MapRepo whose vertex store is the
SQLite-backed session factory from conftest, so no ROS graph or PostgreSQL is
needed.
"""

import uuid

import pytest

pytest.importorskip("cv2")
pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")
# The merged router pulls in the catalogue repo, which parses gridmap.yaml.
pytest.importorskip("yaml")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)
from syncai_backend.interfaces.rest.routers.map import init_map_router  # noqa: E402


# No map_name: the owning map is the URL's path segment now.
_VERTEX = {
    "name": "dock",
    "type": "GENERAL",
    "x": 3.0,
    "y": -1.5,
    "theta": 90.0,
}

# Map directories the conftest maps_dir fixture lays down.
_MAP = "full"
_OTHER_MAP = "rawonly"

# A well-formed UUID that never gets created, for "not found" paths (a
# non-UUID like "999999" would fail path validation with 422, not 404).
_MISSING_ID = "00000000-0000-0000-0000-000000000000"


class _StubMapGateway:
    """Stands in for the LoadMap client, which needs a live map_server.

    Only the vertex routes are exercised here, so it is never called; the three
    lines are duplicated from test_maps_router.py rather than shared because the
    two files are deliberately independent by fixture.
    """

    def reload_map(self, yaml_path):
        raise AssertionError("the vertex routes must not reload a map")


@pytest.fixture
def client(logger, map_repo, catalog_repo):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=catalog_repo,
            map_gw=_StubMapGateway(),
        )
    )
    return TestClient(app)


# --- /api/v1/maps/{name}/vertices ------------------------------------------

def test_create_single_and_get_vertex(client):
    created = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[_VERTEX])
    assert created.status_code == 200
    body = created.json()
    assert isinstance(body, list) and len(body) == 1
    uuid.UUID(body[0]["id"])  # id is a well-formed UUID string
    assert body[0]["name"] == "dock"
    assert body[0]["x"] == 3.0

    fetched = client.get(f"/api/v1/maps/{_MAP}/vertices/{body[0]['id']}")
    assert fetched.status_code == 200
    assert fetched.json()["name"] == "dock"


def test_create_batch_returns_all_in_order(client):
    created = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[
        {**_VERTEX, "name": "a"},
        {**_VERTEX, "name": "b", "type": "ARTIFACT"},
        {**_VERTEX, "name": "c"},
    ])
    assert created.status_code == 200
    body = created.json()
    assert [v["name"] for v in body] == ["a", "b", "c"]
    # All persisted and independently retrievable.
    assert len(client.get(f"/api/v1/maps/{_MAP}/vertices").json()) == 3
    assert len({v["id"] for v in body}) == 3


def test_get_missing_vertex_returns_404(client):
    assert client.get(f"/api/v1/maps/{_MAP}/vertices/{_MISSING_ID}").status_code == 404


def test_get_non_uuid_id_returns_422(client):
    assert client.get(f"/api/v1/maps/{_MAP}/vertices/999999").status_code == 422


def test_create_empty_list_returns_422(client):
    assert client.post(f"/api/v1/maps/{_MAP}/vertices", json=[]).status_code == 422


def test_create_missing_fields_returns_422(client):
    assert client.post(f"/api/v1/maps/{_MAP}/vertices",
                       json=[{"name": "x"}]).status_code == 422


def test_create_invalid_type_returns_422(client):
    assert client.post(f"/api/v1/maps/{_MAP}/vertices",
                       json=[{**_VERTEX, "type": "bogus"}]).status_code == 422


def test_create_batch_is_atomic_on_invalid_item(client):
    # One bad item rejects the whole batch; nothing is persisted.
    resp = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[
        {**_VERTEX, "name": "ok"},
        {**_VERTEX, "name": "bad", "type": "bogus"},
    ])
    assert resp.status_code == 422
    assert client.get(f"/api/v1/maps/{_MAP}/vertices").json() == []


def test_list_vertices_scoped_to_the_map_in_the_path(client):
    client.post(f"/api/v1/maps/{_MAP}/vertices", json=[
        {**_VERTEX, "name": "w", "type": "GENERAL"},
        {**_VERTEX, "name": "p", "type": "ARTIFACT"},
    ])
    client.post(f"/api/v1/maps/{_OTHER_MAP}/vertices", json=[
        {**_VERTEX, "name": "o", "type": "GENERAL"},
    ])

    assert len(client.get(f"/api/v1/maps/{_MAP}/vertices").json()) == 2
    assert len(client.get(f"/api/v1/maps/{_OTHER_MAP}/vertices").json()) == 1
    assert len(client.get(f"/api/v1/maps/{_MAP}/vertices",
                          params={"type": "ARTIFACT"}).json()) == 1


def test_created_vertex_takes_map_name_from_the_path(client):
    body = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[_VERTEX]).json()
    assert body[0]["map_name"] == _MAP


def test_vertices_on_unknown_map_return_404(client):
    assert client.get("/api/v1/maps/nope/vertices").status_code == 404
    assert client.post("/api/v1/maps/nope/vertices", json=[_VERTEX]).status_code == 404


def test_update_vertex(client):
    vertex_id = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[_VERTEX]).json()[0]["id"]

    response = client.put(f"/api/v1/maps/{_MAP}/vertices/{vertex_id}",
                          json={"name": "renamed", "x": 9.0})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "renamed"
    assert body["x"] == 9.0
    assert body["y"] == -1.5  # untouched


def test_update_missing_vertex_returns_404(client):
    response = client.put(f"/api/v1/maps/{_MAP}/vertices/{_MISSING_ID}", json={"name": "x"})
    assert response.status_code == 404


def test_delete_vertex(client):
    vertex_id = client.post(f"/api/v1/maps/{_MAP}/vertices", json=[_VERTEX]).json()[0]["id"]

    response = client.delete(f"/api/v1/maps/{_MAP}/vertices/{vertex_id}")

    assert response.status_code == 200
    assert "deleted" in response.json()["message"].lower()
    assert client.get(f"/api/v1/maps/{_MAP}/vertices/{vertex_id}").status_code == 404


def test_delete_missing_vertex_returns_404(client):
    assert client.delete(f"/api/v1/maps/{_MAP}/vertices/{_MISSING_ID}").status_code == 404


def test_vertex_is_not_reachable_through_another_map(client):
    """The path is checked, not just used to find the row."""
    vertex_id = client.post(
        f"/api/v1/maps/{_MAP}/vertices", json=[_VERTEX]
    ).json()[0]["id"]

    assert client.get(
        f"/api/v1/maps/{_OTHER_MAP}/vertices/{vertex_id}"
    ).status_code == 404
    assert client.put(
        f"/api/v1/maps/{_OTHER_MAP}/vertices/{vertex_id}", json={"name": "x"}
    ).status_code == 404
    assert client.delete(
        f"/api/v1/maps/{_OTHER_MAP}/vertices/{vertex_id}"
    ).status_code == 404

    # And the vertex is untouched where it does live.
    assert client.get(
        f"/api/v1/maps/{_MAP}/vertices/{vertex_id}"
    ).json()["name"] == "dock"


def test_vertex_under_an_unknown_map_returns_404(client):
    assert client.get(f"/api/v1/maps/nope/vertices/{_MISSING_ID}").status_code == 404
