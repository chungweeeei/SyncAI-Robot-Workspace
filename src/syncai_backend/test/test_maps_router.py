"""Tests for the map catalogue routes (/api/v1/maps).

Same router as test_map_router.py — the two URL families were merged into
routers/map.py — but kept as its own file because the fixture differs: this one
needs a tmp_path maps tree and an INI override pinning the active map.

Same shape otherwise: the router is mounted on a bare FastAPI app with the
production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. Repos are real, over a tmp_path maps tree and
in-memory SQLite.
"""

import os
import struct

import pytest

pytest.importorskip("cv2")
pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")
pytest.importorskip("yaml")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers.map import init_map_router  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubMapGateway:
    """Records reload_map / save_map calls instead of making ROS service calls.

    The one thing this suite cannot make real: a LoadMap client needs a live
    map_server on a DDS graph, a SaveMaps client a live pgo. The repos either
    side stay real. Note save_map writes nothing — a saved map's on-disk files
    are pgo's doing, so tests that need a map.pcd create it themselves.
    """

    def __init__(self):
        self.calls = []
        self.result = (True, "")
        self.save_calls = []
        self.save_result = (True, "")

    def reload_map(self, yaml_path):
        self.calls.append(yaml_path)
        return self.result

    def save_map(self, directory):
        self.save_calls.append(directory)
        return self.save_result


@pytest.fixture
def map_gw():
    return _StubMapGateway()


@pytest.fixture
def client(logger, catalog_repo, map_repo, map_gw, tmp_path, monkeypatch):
    """A client whose active map is 'full', set through the INI env override."""
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=catalog_repo,
            map_gw=map_gw,
        )
    )
    return TestClient(app)


_OCTET = {"Content-Type": "application/octet-stream"}


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


# --- /api/v1/maps/{name}/image ----------------------------------------------


def test_image_is_a_full_size_png(client):
    """Native resolution, unlike the thumbnail: 6x4 in, 6x4 out."""
    response = client.get("/api/v1/maps/full/image")

    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    decoded = cv2.imdecode(
        np.frombuffer(response.content, np.uint8), cv2.IMREAD_GRAYSCALE
    )
    assert decoded.shape == (4, 6)


def test_image_and_thumbnail_share_the_source_etag(client):
    """Both hash the .pgm, so a client can revalidate either against the other."""
    image = client.get("/api/v1/maps/full/image")
    thumbnail = client.get("/api/v1/maps/full/thumbnail")

    assert image.headers["etag"] == thumbnail.headers["etag"]


def test_image_revalidates_to_304(client):
    tag = client.get("/api/v1/maps/full/image").headers["etag"]

    response = client.get("/api/v1/maps/full/image", headers={"If-None-Match": tag})

    assert response.status_code == 304
    assert response.content == b""


def test_image_404_when_the_map_has_no_gridmap(client):
    assert client.get("/api/v1/maps/rawonly/image").status_code == 404


def test_image_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/image").status_code == 404


def test_image_404_when_the_gridmap_is_unreadable(client, maps_dir):
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"garbage")

    assert client.get("/api/v1/maps/full/image").status_code == 404


def test_image_etag_follows_content_not_mtime(client, maps_dir, make_pgm):
    """An edited gridmap keeps its dimensions, so it keeps its file size, and
    this filesystem hands out a coarse mtime — the tag has to be content-based
    or the editor would reload the pre-edit grid."""
    before = client.get("/api/v1/maps/full/image").headers["etag"]
    path = maps_dir / "full" / "gridmap.pgm"
    size_before = path.stat().st_size

    make_pgm(path, 6, 4, fill=0)
    after = client.get("/api/v1/maps/full/image")

    assert path.stat().st_size == size_before
    assert after.headers["etag"] != before
    assert after.status_code == 200


# --- PUT /api/v1/maps/{name}/grid -------------------------------------------


def _put_grid(client, name, body):
    return client.put(f"/api/v1/maps/{name}/grid", content=body, headers=_OCTET)


def _image_cells(client, name):
    response = client.get(f"/api/v1/maps/{name}/image")
    return cv2.imdecode(
        np.frombuffer(response.content, np.uint8), cv2.IMREAD_GRAYSCALE
    )


def test_save_grid_writes_the_cells(client, maps_dir):
    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.status_code == 200
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes().startswith(b"P5\n6 4\n255\n")
    assert not _image_cells(client, "full").any()


def test_save_grid_reloads_the_active_map(client, map_gw):
    body = _put_grid(client, "full", b"\x00" * 24).json()

    assert body["active"] is True
    assert body["reloaded"] is True
    assert len(map_gw.calls) == 1

    # map_server resolves the yaml's relative image key against dirname() of the
    # string it was handed, unexpanded — so this must be absolute and ~-free.
    called = map_gw.calls[0]
    assert called.endswith("full/gridmap.yaml")
    assert called.startswith("/")
    assert "~" not in called


def test_save_grid_does_not_reload_an_inactive_map(
    client, map_gw, maps_dir, make_pgm, make_gridmap_yaml
):
    # Converted here rather than in the maps_dir fixture: a third gridmap there
    # would break the listing tests that assert exactly which maps have one.
    make_pgm(maps_dir / "rawonly" / "gridmap.pgm", 3, 2)
    make_gridmap_yaml(maps_dir / "rawonly" / "gridmap.yaml")

    body = _put_grid(client, "rawonly", b"\x00" * 6).json()

    assert body["active"] is False
    assert body["reloaded"] is False
    assert map_gw.calls == []


def test_save_grid_reports_a_failed_reload_without_failing_the_save(client, map_gw):
    """The bytes are on disk, so a 5xx would be a lie the operator acts on."""
    map_gw.result = (False, "map_server/load_map is not available")

    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.status_code == 200
    body = response.json()
    assert body["active"] is True
    assert body["reloaded"] is False
    assert "map_server/load_map is not available" in body["message"]
    assert not _image_cells(client, "full").any()


def test_save_grid_rejects_a_wrong_length_body(client, maps_dir, map_gw):
    before = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    response = _put_grid(client, "full", b"\x00" * 23)

    assert response.status_code == 400
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() == before
    assert map_gw.calls == []


def test_save_grid_404_for_a_missing_map(client):
    assert _put_grid(client, "nosuchmap", b"\x00" * 24).status_code == 404


def test_save_grid_404_when_the_map_has_no_gridmap(client, maps_dir):
    assert _put_grid(client, "rawonly", b"\x00" * 24).status_code == 404
    assert not (maps_dir / "rawonly" / "gridmap.pgm").exists()


def test_save_grid_400_for_an_unsafe_name(client):
    assert _put_grid(client, "with%20space", b"\x00" * 24).status_code == 400


def test_save_grid_creates_the_raw_backup_once(client, maps_dir):
    pristine = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    raw = maps_dir / "full" / "gridmap_raw.pgm"

    _put_grid(client, "full", b"\x00" * 24)
    assert raw.read_bytes() == pristine

    _put_grid(client, "full", b"\xfe" * 24)
    assert raw.read_bytes() == pristine


def test_save_grid_etag_matches_the_image_etag(client):
    """The tag hashes the whole file, header included, on both sides."""
    body = _put_grid(client, "full", b"\x00" * 24)

    assert body.headers["etag"] == body.json()["etag"]
    assert client.get("/api/v1/maps/full/image").headers["etag"] == body.json()["etag"]


def test_save_grid_updates_the_thumbnail_without_an_eviction(client):
    """The write path deliberately does not touch the caches.

    _png_response re-reads and re-hashes the .pgm before consulting them, so a
    stale entry can never be served — this is the test that keeps that true.
    """
    before = client.get("/api/v1/maps/full/thumbnail")

    _put_grid(client, "full", b"\x00" * 24)
    after = client.get("/api/v1/maps/full/thumbnail")

    assert after.headers["etag"] != before.headers["etag"]
    assert after.content != before.content


def test_save_grid_needs_an_octet_stream_content_type(client, maps_dir):
    """Without the header FastAPI falls back to parsing the body as JSON.

    Documents the contract the frontend has to satisfy: a BufferSource body makes
    fetch() send no Content-Type at all unless it is set explicitly.
    """
    before = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    response = client.put("/api/v1/maps/full/grid", content=b"\x00" * 24)

    assert response.status_code != 200
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() == before


# --- /api/v1/maps/{name}/pointcloud -----------------------------------------


def _unpack_cloud(payload):
    """Undo the wire format: [u32 count][f32 xyz * count]."""
    count = struct.unpack("<I", payload[:4])[0]
    xyz = np.frombuffer(payload[4:], dtype="<f4")
    return count, xyz.reshape(-1, 3)


def test_pointcloud_returns_the_packed_cloud(client):
    response = client.get("/api/v1/maps/full/pointcloud")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"

    count, points = _unpack_cloud(response.content)
    # The fixture's three points are >0.3 m apart, so none are voxel-merged.
    assert count == 3
    assert points.shape == (3, 3)


def test_pointcloud_payload_length_matches_the_count(client):
    """A short body would be read as garbage coordinates by the viewer."""
    payload = client.get("/api/v1/maps/full/pointcloud").content
    count = struct.unpack("<I", payload[:4])[0]

    assert len(payload) == 4 + count * 3 * 4


def test_pointcloud_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/pointcloud").status_code == 404


def test_pointcloud_404_when_the_map_has_no_pcd(client, maps_dir):
    (maps_dir / "full" / "map.pcd").unlink()

    assert client.get("/api/v1/maps/full/pointcloud").status_code == 404


def test_pointcloud_404_when_the_pcd_is_unreadable(client, maps_dir):
    """A torn .pcd must be a 404, not a traceback."""
    (maps_dir / "full" / "map.pcd").write_text("not a pcd at all\n")

    assert client.get("/api/v1/maps/full/pointcloud").status_code == 404


def test_pointcloud_is_recached_when_the_file_changes(client, maps_dir, make_pcd):
    first = client.get("/api/v1/maps/full/pointcloud").content

    make_pcd(
        maps_dir / "full" / "map.pcd",
        points=((0.0, 0.0, 0.0), (5.0, 5.0, 5.0)),
    )
    second = client.get("/api/v1/maps/full/pointcloud").content

    assert struct.unpack("<I", first[:4])[0] == 3
    assert struct.unpack("<I", second[:4])[0] == 2


# --- POST /api/v1/maps --------------------------------------------------------


def _post_map(client, payload):
    return client.post("/api/v1/maps", json=payload)


def test_create_map_saves_through_the_gateway(client, map_gw, catalog_repo):
    """The stub writes no map.pcd, so grid_pending honestly reports false."""
    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "newmap"
    assert body["has_pointcloud"] is True
    assert body["grid_pending"] is False
    # The gateway got the directory this router created, absolute.
    directory = catalog_repo.resolve_dir("newmap")
    assert map_gw.save_calls == [directory]
    assert os.path.isdir(directory)


def test_create_map_lists_afterwards_with_a_null_grid(client, maps_dir, make_pcd):
    _post_map(client, {"name": "newmap"})
    # Stand in for pgo: the stub gateway does not write files.
    make_pcd(maps_dir / "newmap" / "map.pcd")

    entry = _by_name(client.get("/api/v1/maps").json())["newmap"]

    assert entry["grid"] is None
    assert entry["has_pointcloud"] is True


def test_create_map_conflicts_with_an_existing_map(client, map_gw):
    response = _post_map(client, {"name": "full"})

    assert response.status_code == 409
    assert map_gw.save_calls == []


@pytest.mark.parametrize("name", ["../evil", "a/b", "", ".", "x" * 65])
def test_create_map_rejects_bad_names(client, map_gw, name):
    response = _post_map(client, {"name": name})

    # Length/emptiness die in the schema (422), separators in resolve_dir (400);
    # either way nothing reaches the gateway and no directory appears.
    assert response.status_code in (400, 422)
    assert map_gw.save_calls == []


def test_failed_save_unwinds_the_directory(client, map_gw, catalog_repo):
    map_gw.save_result = (False, "NO POSES!")

    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 502
    assert response.json()["detail"] == "NO POSES!"
    assert not os.path.exists(catalog_repo.resolve_dir("newmap"))
