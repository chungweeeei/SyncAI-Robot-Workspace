"""Unit tests for the Map REST router via FastAPI's TestClient.

The router is mounted on a bare FastAPI app with the same exception handlers as
production; the map cache is a real in-memory MapRepo and the point store is the
SQLite-backed MapPointRepo from conftest, so no ROS graph or PostgreSQL is
needed.
"""

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
from syncai_backend.repositories.map.map import init_map_repo  # noqa: E402


_POINT = {
    "name": "dock",
    "type": "waypoint",
    "map_name": "warehouse",
    "x": 3.0,
    "y": -1.5,
    "theta": 90.0,
}


@pytest.fixture
def map_repo(logger):
    return init_map_repo(logger)


@pytest.fixture
def client(logger, map_repo, map_point_repo):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_map_router(
            logger=logger, map_repo=map_repo, map_point_repo=map_point_repo
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


# --- /api/v1/map/points -----------------------------------------------------

def test_create_and_get_point(client):
    created = client.post("/api/v1/map/points", json=_POINT)
    assert created.status_code == 200
    body = created.json()
    assert body["id"] >= 1
    assert body["name"] == "dock"
    assert body["x"] == 3.0

    fetched = client.get(f"/api/v1/map/points/{body['id']}")
    assert fetched.status_code == 200
    assert fetched.json()["name"] == "dock"


def test_get_missing_point_returns_404(client):
    assert client.get("/api/v1/map/points/999999").status_code == 404


def test_create_missing_fields_returns_422(client):
    assert client.post("/api/v1/map/points", json={"name": "x"}).status_code == 422


def test_list_points_with_filters(client):
    client.post("/api/v1/map/points",
                json={**_POINT, "name": "w", "type": "waypoint",
                      "map_name": "warehouse"})
    client.post("/api/v1/map/points",
                json={**_POINT, "name": "p", "type": "patrol",
                      "map_name": "warehouse"})
    client.post("/api/v1/map/points",
                json={**_POINT, "name": "o", "type": "waypoint",
                      "map_name": "office"})

    assert len(client.get("/api/v1/map/points").json()) == 3
    assert len(client.get("/api/v1/map/points",
                          params={"map_name": "warehouse"}).json()) == 2
    assert len(client.get("/api/v1/map/points",
                          params={"type": "patrol"}).json()) == 1
    assert client.get("/api/v1/map/points",
                      params={"map_name": "nope"}).json() == []


def test_update_point(client):
    point_id = client.post("/api/v1/map/points", json=_POINT).json()["id"]

    response = client.put(f"/api/v1/map/points/{point_id}",
                          json={"name": "renamed", "x": 9.0})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "renamed"
    assert body["x"] == 9.0
    assert body["y"] == -1.5  # untouched


def test_update_missing_point_returns_404(client):
    response = client.put("/api/v1/map/points/999999", json={"name": "x"})
    assert response.status_code == 404


def test_delete_point(client):
    point_id = client.post("/api/v1/map/points", json=_POINT).json()["id"]

    response = client.delete(f"/api/v1/map/points/{point_id}")

    assert response.status_code == 200
    assert "deleted" in response.json()["message"].lower()
    assert client.get(f"/api/v1/map/points/{point_id}").status_code == 404


def test_delete_missing_point_returns_404(client):
    assert client.delete("/api/v1/map/points/999999").status_code == 404
