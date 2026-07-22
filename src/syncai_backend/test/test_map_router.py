"""Unit tests for the Map REST router via FastAPI's TestClient.

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

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)
from syncai_backend.interfaces.rest.routers.map import init_map_router  # noqa: E402
from syncai_backend.repositories.pointcloud.pointcloud import (  # noqa: E402
    init_pointcloud_repo,
)


_VERTEX = {
    "name": "dock",
    "type": "GENERAL",
    "map_name": "warehouse",
    "x": 3.0,
    "y": -1.5,
    "theta": 90.0,
}

# A well-formed UUID that never gets created, for "not found" paths (a
# non-UUID like "999999" would fail path validation with 422, not 404).
_MISSING_ID = "00000000-0000-0000-0000-000000000000"


@pytest.fixture
def client(logger, map_repo):
    app = FastAPI()
    register_exception_handlers(app)
    map_cloud_repo = init_pointcloud_repo(logger=logger)
    app.include_router(
        init_map_router(
            logger=logger, map_repo=map_repo, map_cloud_repo=map_cloud_repo
        )
    )
    return TestClient(app)


# --- /api/v1/map/info -------------------------------------------------------

def test_map_info_404_when_no_map(client):
    assert client.get("/api/v1/map/info").status_code == 404


def test_map_info_returns_metadata(client, map_repo, make_occupancy_grid):
    grid = make_occupancy_grid(4, 5, [0] * 20, resolution=0.05,
                               origin=(-1.0, -2.0, 0.0))
    map_repo.update_map(grid)

    response = client.get("/api/v1/map/info")

    assert response.status_code == 200
    body = response.json()
    assert body["width"] == 4
    assert body["height"] == 5
    assert body["resolution"] == pytest.approx(0.05)
    assert body["origin"] == {"x": -1.0, "y": -2.0, "z": 0.0}
    assert body["frame_id"] == "map"


# --- /api/v1/map/image ------------------------------------------------------

def test_map_image_404_when_no_map(client):
    assert client.get("/api/v1/map/image").status_code == 404


def test_map_image_returns_data_uri_with_metadata(client, map_repo,
                                                  make_occupancy_grid):
    grid = make_occupancy_grid(2, 2, [0, 100, -1, 50])
    map_repo.update_map(grid)

    response = client.get("/api/v1/map/image")

    assert response.status_code == 200
    body = response.json()
    assert body["image"].startswith("data:image/png;base64,")
    assert body["width"] == 2
    assert body["height"] == 2


# --- /api/v1/map/vertices ---------------------------------------------------

def test_create_single_and_get_vertex(client):
    created = client.post("/api/v1/map/vertices", json=[_VERTEX])
    assert created.status_code == 200
    body = created.json()
    assert isinstance(body, list) and len(body) == 1
    uuid.UUID(body[0]["id"])  # id is a well-formed UUID string
    assert body[0]["name"] == "dock"
    assert body[0]["x"] == 3.0

    fetched = client.get(f"/api/v1/map/vertices/{body[0]['id']}")
    assert fetched.status_code == 200
    assert fetched.json()["name"] == "dock"


def test_create_batch_returns_all_in_order(client):
    created = client.post("/api/v1/map/vertices", json=[
        {**_VERTEX, "name": "a"},
        {**_VERTEX, "name": "b", "type": "ARTIFACT"},
        {**_VERTEX, "name": "c"},
    ])
    assert created.status_code == 200
    body = created.json()
    assert [v["name"] for v in body] == ["a", "b", "c"]
    # All persisted and independently retrievable.
    assert len(client.get("/api/v1/map/vertices").json()) == 3
    assert len({v["id"] for v in body}) == 3


def test_get_missing_vertex_returns_404(client):
    assert client.get(f"/api/v1/map/vertices/{_MISSING_ID}").status_code == 404


def test_get_non_uuid_id_returns_422(client):
    assert client.get("/api/v1/map/vertices/999999").status_code == 422


def test_create_empty_list_returns_422(client):
    assert client.post("/api/v1/map/vertices", json=[]).status_code == 422


def test_create_missing_fields_returns_422(client):
    assert client.post("/api/v1/map/vertices",
                       json=[{"name": "x"}]).status_code == 422


def test_create_invalid_type_returns_422(client):
    assert client.post("/api/v1/map/vertices",
                       json=[{**_VERTEX, "type": "bogus"}]).status_code == 422


def test_create_batch_is_atomic_on_invalid_item(client):
    # One bad item rejects the whole batch; nothing is persisted.
    resp = client.post("/api/v1/map/vertices", json=[
        {**_VERTEX, "name": "ok"},
        {**_VERTEX, "name": "bad", "type": "bogus"},
    ])
    assert resp.status_code == 422
    assert client.get("/api/v1/map/vertices").json() == []


def test_list_vertices_with_filters(client):
    client.post("/api/v1/map/vertices", json=[
        {**_VERTEX, "name": "w", "type": "GENERAL", "map_name": "warehouse"},
        {**_VERTEX, "name": "p", "type": "ARTIFACT", "map_name": "warehouse"},
        {**_VERTEX, "name": "o", "type": "GENERAL", "map_name": "office"},
    ])

    assert len(client.get("/api/v1/map/vertices").json()) == 3
    assert len(client.get("/api/v1/map/vertices",
                          params={"map_name": "warehouse"}).json()) == 2
    assert len(client.get("/api/v1/map/vertices",
                          params={"type": "ARTIFACT"}).json()) == 1
    assert client.get("/api/v1/map/vertices",
                      params={"map_name": "nope"}).json() == []


def test_update_vertex(client):
    vertex_id = client.post("/api/v1/map/vertices", json=[_VERTEX]).json()[0]["id"]

    response = client.put(f"/api/v1/map/vertices/{vertex_id}",
                          json={"name": "renamed", "x": 9.0})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "renamed"
    assert body["x"] == 9.0
    assert body["y"] == -1.5  # untouched


def test_update_missing_vertex_returns_404(client):
    response = client.put(f"/api/v1/map/vertices/{_MISSING_ID}", json={"name": "x"})
    assert response.status_code == 404


def test_delete_vertex(client):
    vertex_id = client.post("/api/v1/map/vertices", json=[_VERTEX]).json()[0]["id"]

    response = client.delete(f"/api/v1/map/vertices/{vertex_id}")

    assert response.status_code == 200
    assert "deleted" in response.json()["message"].lower()
    assert client.get(f"/api/v1/map/vertices/{vertex_id}").status_code == 404


def test_delete_missing_vertex_returns_404(client):
    assert client.delete(f"/api/v1/map/vertices/{_MISSING_ID}").status_code == 404
