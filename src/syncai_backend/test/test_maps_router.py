"""Tests for the map catalogue routes (/api/v1/maps).

Same shape as test_map_router.py: the router is mounted on a bare FastAPI app
with the production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. Repos are real, over a tmp_path maps tree and
in-memory SQLite.
"""

import pytest

pytest.importorskip("cv2")
pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")
pytest.importorskip("yaml")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers.maps import (  # noqa: E402
    init_maps_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


@pytest.fixture
def client(logger, catalog_repo, map_repo, tmp_path, monkeypatch):
    """A client whose active map is 'full', set through the INI env override."""
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_maps_router(
            logger=logger, catalog_repo=catalog_repo, map_repo=map_repo
        )
    )
    return TestClient(app)


def _by_name(body):
    return {entry["name"]: entry for entry in body}


# --- /api/v1/maps -----------------------------------------------------------


def test_list_returns_both_maps_sorted(client):
    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert [entry["name"] for entry in response.json()] == ["full", "rawonly"]


def test_list_marks_only_the_ini_map_active(client):
    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["active"] is True
    assert body["rawonly"]["active"] is False


def test_list_reports_grid_geometry(client):
    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid"]["width"] == 6
    assert entry["grid"]["height"] == 4
    assert entry["grid"]["resolution"] == pytest.approx(0.05)
    assert entry["grid"]["origin"]["x"] == pytest.approx(-6.94)
    assert entry["grid"]["origin"]["yaw"] == pytest.approx(0.0)
    assert entry["thumbnail"] == "/api/v1/maps/full/thumbnail"
    assert entry["has_pointcloud"] is True
    assert entry["size_bytes"] > 0
    assert entry["modified_at"].endswith("Z")


def test_list_nulls_grid_for_an_unconverted_map(client):
    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid"] is None
    assert entry["thumbnail"] is None


def test_list_counts_vertices_of_that_map_only(client, map_repo):
    map_repo.create_vertices([
        {"name": "a", "type": "GENERAL", "map_name": "full",
         "x": 1.0, "y": 2.0, "theta": 0.0},
        {"name": "b", "type": "CHARGER", "map_name": "full",
         "x": 3.0, "y": 4.0, "theta": 90.0},
        {"name": "c", "type": "GENERAL", "map_name": "rawonly",
         "x": 5.0, "y": 6.0, "theta": 0.0},
    ])

    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["vertex_count"] == 2
    assert body["rawonly"]["vertex_count"] == 1


# --- /api/v1/maps/{name} ----------------------------------------------------


def test_get_returns_one_summary(client):
    response = client.get("/api/v1/maps/full")

    assert response.status_code == 200
    assert response.json()["name"] == "full"


def test_get_missing_map_returns_404(client):
    assert client.get("/api/v1/maps/nosuchmap").status_code == 404


def test_get_unsafe_name_returns_400(client):
    assert client.get("/api/v1/maps/with%20space").status_code == 400


# --- /api/v1/maps/{name}/gridmap --------------------------------------------


def test_gridmap_is_byte_identical_to_the_file(client, maps_dir):
    response = client.get("/api/v1/maps/full/gridmap")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"
    assert response.content == (maps_dir / "full" / "gridmap.pgm").read_bytes()
    assert response.headers["etag"]


def test_gridmap_404_when_the_map_has_none(client):
    assert client.get("/api/v1/maps/rawonly/gridmap").status_code == 404


def test_gridmap_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/gridmap").status_code == 404


def test_gridmap_revalidates_to_304(client):
    tag = client.get("/api/v1/maps/full/gridmap").headers["etag"]

    response = client.get(
        "/api/v1/maps/full/gridmap", headers={"If-None-Match": tag}
    )

    assert response.status_code == 304
    assert response.content == b""


def test_gridmap_etag_follows_content_not_mtime(client, maps_dir, make_pgm):
    """An edited gridmap keeps its dimensions, so it keeps its file size, and
    this filesystem hands out a coarse mtime — the tag has to be content-based
    or the editor would reload the pre-edit grid."""
    before = client.get("/api/v1/maps/full/gridmap").headers["etag"]
    path = maps_dir / "full" / "gridmap.pgm"
    size_before = path.stat().st_size

    make_pgm(path, 6, 4, fill=0)
    after = client.get("/api/v1/maps/full/gridmap")

    assert path.stat().st_size == size_before
    assert after.headers["etag"] != before
    assert after.status_code == 200


# --- /api/v1/maps/{name}/thumbnail ------------------------------------------


def test_thumbnail_returns_png(client):
    response = client.get("/api/v1/maps/full/thumbnail")

    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    # PNG magic; proves an image came back rather than an error body.
    assert response.content[:8] == b"\x89PNG\r\n\x1a\n"


def test_thumbnail_is_cached_until_the_file_changes(client, maps_dir, make_pgm):
    first = client.get("/api/v1/maps/full/thumbnail")
    again = client.get("/api/v1/maps/full/thumbnail")

    assert again.headers["etag"] == first.headers["etag"]
    assert again.content == first.content

    make_pgm(maps_dir / "full" / "gridmap.pgm", 9, 9, fill=0)
    third = client.get("/api/v1/maps/full/thumbnail")

    assert third.headers["etag"] != first.headers["etag"]
    assert third.content != first.content


def test_thumbnail_revalidates_to_304(client):
    tag = client.get("/api/v1/maps/full/thumbnail").headers["etag"]

    response = client.get(
        "/api/v1/maps/full/thumbnail", headers={"If-None-Match": tag}
    )

    assert response.status_code == 304


def test_thumbnail_404_when_the_map_has_none(client):
    assert client.get("/api/v1/maps/rawonly/thumbnail").status_code == 404


def test_thumbnail_404_when_the_gridmap_is_unreadable(client, maps_dir):
    """A torn file must be a 404, not a traceback."""
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"garbage")

    assert client.get("/api/v1/maps/full/thumbnail").status_code == 404
