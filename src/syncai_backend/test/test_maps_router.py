"""Tests for the map catalogue routes (/api/v1/maps).

Same router as test_map_router.py — the two URL families were merged into
routers/map.py — but kept as its own file because the fixture differs: this one
needs a tmp_path maps tree and an INI override pinning the active map.

Same shape otherwise: the router is mounted on a bare FastAPI app with the
production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. Repos are real, over a tmp_path maps tree and
in-memory SQLite.
"""

import builtins
import json
import os
import struct
import sys
import threading
import types

import pytest

pytest.importorskip("cv2")
pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")
pytest.importorskip("yaml")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers import map as map_router_module  # noqa: E402
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
        # The map-switch surface. `order` records every ROS step across both
        # clients in sequence, because for a switch the ordering *is* the
        # contract: the localizer moves before map_server so that the likeliest
        # failure leaves nothing changed.
        self.order = []
        self.swap_calls = []
        self.swap_result = (True, "")
        self.services_ready = True
        self.converged = True

    def reload_map(self, yaml_path):
        self.calls.append(yaml_path)
        self.order.append(("reload_map", yaml_path))
        return self.result

    def save_map(self, directory):
        self.save_calls.append(directory)
        return self.save_result

    def nav_services_ready(self, timeout_sec=2.0):
        return self.services_ready

    def swap_localizer_map(self, pcd_path, x, y, yaw):
        self.swap_calls.append((pcd_path, x, y, yaw))
        self.order.append(("swap_localizer_map", pcd_path))
        return self.swap_result

    def localization_converged(self, timeout_s=3.0):
        return self.converged


class _StubWorkflowGateway:
    """Stands in for the Temporal visibility query behind the task_running gate.

    `error` is how the "Temporal is down" refusal is exercised: list_active_tasks
    raises for real in that case rather than returning an empty list, and the
    difference between those two is the whole point of the tasks_unknown code.
    """

    def __init__(self):
        self.tasks = []
        self.error = None

    async def list_active_tasks(self):
        if self.error is not None:
            raise self.error
        return self.tasks, "2026-09-16T00:00:00Z"


@pytest.fixture
def map_gw():
    return _StubMapGateway()


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def client(
    logger,
    catalog_repo,
    map_repo,
    map_gw,
    workflow_gw,
    task_template_repo,
    tmp_path,
    monkeypatch,
):
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
            task_template_repo=task_template_repo,
            workflow_gw=workflow_gw,
        )
    )
    return TestClient(app)


_OCTET = {"Content-Type": "application/octet-stream"}


def _by_name(body):
    return {entry["name"]: entry for entry in body}


def _plant_sidecar(directory, payload):
    """Write a conversion record into a map directory, as a conversion would.

    Planted rather than produced by a real conversion for the status tests: the
    states worth pinning are the ones no single run can reach on demand — a
    record left behind by a process that no longer exists, and one written by a
    version of this code that had no status field.
    """
    path = directory / map_router_module.GRIDMAP_RECIPE_SIDECAR
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


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


# --- grid_status --------------------------------------------------------------
#
# The conversion-status surface. There is no job resource and no status
# endpoint: a client that starts a conversion watches this field on the
# catalogue, so every state a conversion can leave a map in has to be
# reachable through it — including the two that outlive the process.


def test_list_reports_ok_for_a_converted_map(client):
    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid_status"] == "ok"
    assert entry["grid_error"] is None
    assert entry["grid_converting"] is False


def test_list_reports_none_for_a_map_nobody_converted(client):
    """Distinct from `failed`, and that is the point of the enum: this one wants
    Build grid pressed, a failure wants its reason read first."""
    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "none"
    assert entry["grid_error"] is None


def test_list_reports_ok_for_a_sidecar_written_before_the_status_field(
    client, maps_dir
):
    """Every map converted before 2026-09 has a sidecar with no `status` key.
    The grid on disk is then the only evidence, and it says the run worked."""
    _plant_sidecar(maps_dir / "full", {"recipe": "z-band", "footprint_m2": 400.0})

    assert _by_name(client.get("/api/v1/maps").json())["full"]["grid_status"] == "ok"


def test_list_reports_a_failed_conversion_with_its_reason(client, maps_dir):
    """The whole point of the sidecar carrying a status: before it, this map was
    indistinguishable from one nobody had converted and the reason lived only in
    log/stack/<robot_id>/backend/current."""
    _plant_sidecar(
        maps_dir / "rawonly",
        {
            "status": "failed",
            "recipe": "z-band",
            "error": "obstacle band selected no points",
        },
    )

    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "failed"
    assert entry["grid_error"] == "obstacle band selected no points"
    assert entry["grid"] is None


def test_a_failed_reconversion_reports_failed_over_the_grid_it_left_behind(
    client, maps_dir
):
    """archive_gridmap *copies* the live grid aside rather than moving it, so the
    active map never has a window with no file — which means a failed re-convert
    leaves a loadable map whose grid is the old one. Reporting `ok` there would
    present that stale grid as the rebuild the operator asked for."""
    _plant_sidecar(
        maps_dir / "full",
        {"status": "failed", "recipe": "traversability", "error": "no ground"},
    )

    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid_status"] == "failed"
    assert entry["grid_error"] == "no ground"
    # Still loadable, and still listed with its geometry.
    assert entry["grid"] is not None


def test_list_reports_an_abandoned_conversion_as_interrupted(client, maps_dir):
    """A record saying `converting` with nothing in the registry behind it: the
    process running it is gone (a backend restart, or a switch_mode that tore
    down the byobu session the backend is a pane of). Nothing is coming to
    finish it, so the state has to say so rather than read as in-flight."""
    _plant_sidecar(maps_dir / "rawonly", {"status": "converting", "recipe": "z-band"})

    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "interrupted"
    assert entry["grid_converting"] is False
    assert entry["grid_error"] is None


def test_a_running_conversion_outranks_whatever_the_sidecar_says(client, maps_dir):
    """The registry is authoritative while this process is up. The sidecar write
    is best-effort, so a conversion whose record never landed — or landed as the
    previous run's failure — must still report as running."""
    _plant_sidecar(maps_dir / "rawonly", {"status": "failed", "error": "last time"})
    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("rawonly")
    try:
        entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]
        assert entry["grid_status"] == "converting"
        assert entry["grid_error"] is None
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("rawonly")


def test_list_survives_a_half_written_sidecar(client, maps_dir):
    """A conversion writes this file while the console's two-second catalogue
    poll reads it, so a torn read is expected traffic, not a corrupt map."""
    (maps_dir / "rawonly" / map_router_module.GRIDMAP_RECIPE_SIDECAR).write_text(
        '{"status": "conv'
    )

    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert _by_name(response.json())["rawonly"]["grid_status"] == "none"


def test_list_counts_vertices_of_that_map_only(client, map_repo):
    map_repo.create_vertices(map="full", vertices=[
        {"name": "a", "type": "GENERAL", "x": 1.0, "y": 2.0, "theta": 0.0},
        {"name": "b", "type": "CHARGER", "x": 3.0, "y": 4.0, "theta": 90.0},
    ])
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "c", "type": "GENERAL", "x": 5.0, "y": 6.0, "theta": 0.0},
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


def test_save_grid_accepts_a_missing_content_type(client, maps_dir):
    """A bare BufferSource fetch() sends no Content-Type; the save still lands.

    This test used to assert the opposite, on the premise that FastAPI would
    fall back to parsing the body as JSON without the header. The pinned
    FastAPI does no such thing: a ``bytes`` body parameter receives the raw
    payload whatever the Content-Type says, so the ``media_type`` on the Body
    is OpenAPI documentation, not enforcement. The length gate is what actually
    rejects a malformed body (test_save_grid_rejects_a_wrong_length_body), so
    tolerating the missing header loses nothing — and pinning tolerance keeps
    this from silently flipping again on the next FastAPI bump.
    """
    response = client.put("/api/v1/maps/full/grid", content=b"\x00" * 24)

    assert response.status_code == 200
    # The write really happened: the body is the 24 cells just sent.
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes()[-24:] == b"\x00" * 24


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


# --- PATCH /api/v1/maps/{name} ------------------------------------------------
#
# The client fixture pins the active map to 'full', so 'rawonly' is the map a
# rename is allowed to touch.


def _rename(client, name, new_name):
    return client.patch(f"/api/v1/maps/{name}", json={"name": new_name})


def test_rename_moves_the_directory_and_lists_under_the_new_name(
    client, maps_dir, map_repo, task_template_repo
):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "HOME", "x": 1.0, "y": 0.0, "theta": 0.0},
    ])
    task_template_repo.create_task_template(
        name="patrol", description="", map_name="rawonly", steps=[]
    )

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 200
    body = response.json()
    assert body["old_name"] == "rawonly"
    assert body["name"] == "hall"
    assert body["vertices_moved"] == 2
    assert body["templates_moved"] == 1
    assert "Renamed 'rawonly' to 'hall'" in body["message"]
    assert "Schedules already registered" in body["message"]

    assert not (maps_dir / "rawonly").exists()
    assert (maps_dir / "hall" / "map.pcd").is_file()
    listing = _by_name(client.get("/api/v1/maps").json())
    assert "rawonly" not in listing
    assert listing["hall"]["vertex_count"] == 2
    assert listing["hall"]["active"] is False
    assert map_repo.list_vertices(map="rawonly") == []
    assert len(map_repo.list_vertices(map="hall")) == 2


def test_rename_message_without_templates_has_no_schedule_caveat(client):
    body = _rename(client, "rawonly", "hall").json()

    assert body["templates_moved"] == 0
    assert "Schedules" not in body["message"]


def test_rename_refuses_the_active_map(client, maps_dir):
    response = _rename(client, "full", "hall")

    assert response.status_code == 409
    assert response.json()["code"] == "map_active"
    assert (maps_dir / "full" / "gridmap.pgm").is_file()
    assert not (maps_dir / "hall").exists()


def test_rename_refuses_while_a_conversion_is_running(client, maps_dir):
    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("rawonly")
    try:
        response = _rename(client, "rawonly", "hall")
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert (maps_dir / "rawonly").is_dir()


def test_rename_refuses_a_taken_name(client, maps_dir):
    response = _rename(client, "rawonly", "full")

    assert response.status_code == 409
    assert response.json()["code"] == "name_taken"
    assert (maps_dir / "rawonly").is_dir()
    assert (maps_dir / "full" / "gridmap.pgm").is_file()


def test_rename_to_the_same_name_is_a_400(client, maps_dir):
    response = _rename(client, "rawonly", "rawonly")

    assert response.status_code == 400
    assert (maps_dir / "rawonly").is_dir()


@pytest.mark.parametrize("new_name", ["../evil", "a/b", "", ".", "x" * 65])
def test_rename_rejects_bad_new_names(client, maps_dir, new_name):
    response = _rename(client, "rawonly", new_name)

    # Length/emptiness die in the schema (422), separators in resolve_dir (400).
    assert response.status_code in (400, 422)
    assert (maps_dir / "rawonly").is_dir()


def test_rename_of_a_missing_map_is_a_404(client):
    assert _rename(client, "nope", "hall").status_code == 404


def test_rename_moves_the_directory_back_when_the_database_fails(
    client, maps_dir, map_repo, monkeypatch
):
    """The two stores cannot share a transaction, so the filesystem is undone."""
    def _boom(old_map, new_map):
        raise RuntimeError("database is away")

    monkeypatch.setattr(map_repo, "move_vertices", _boom)

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 502
    assert "left under its old name" in response.json()["detail"]
    assert (maps_dir / "rawonly" / "map.pcd").is_file()
    assert not (maps_dir / "hall").exists()


def test_rename_drops_the_cached_renderings_of_the_old_name(client, maps_dir, make_pcd):
    """Warm the cloud cache under the old name, rename, and check nothing is left keyed on it."""
    # 'full' is active; give 'rawonly' a proper cloud and read it so it caches.
    make_pcd(maps_dir / "rawonly" / "map.pcd")
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 200

    assert _rename(client, "rawonly", "hall").status_code == 200

    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 404
    assert client.get("/api/v1/maps/hall/pointcloud").status_code == 200


# --- DELETE /api/v1/maps/{name} ----------------------------------------------
#
# Same fixture geometry as the rename block: 'full' is the active map, so
# 'rawonly' is the one a delete is allowed to touch.


def _delete(client, name):
    return client.delete(f"/api/v1/maps/{name}")


def test_delete_removes_the_directory_and_its_vertices(client, maps_dir, map_repo):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "HOME", "x": 1.0, "y": 0.0, "theta": 0.0},
    ])
    # A vertex on another map must survive: the DELETE is filtered by map.
    map_repo.create_vertices(map="full", vertices=[
        {"name": "charger", "type": "CHARGER", "x": 2.0, "y": 0.0, "theta": 0.0},
    ])

    response = _delete(client, "rawonly")

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "rawonly"
    assert body["vertices_deleted"] == 2
    assert "Deleted 'rawonly' and 2 vertices" in body["message"]
    assert "Schedules already registered" in body["message"]

    assert not (maps_dir / "rawonly").exists()
    assert "rawonly" not in _by_name(client.get("/api/v1/maps").json())
    assert map_repo.list_vertices(map="rawonly") == []
    assert len(map_repo.list_vertices(map="full")) == 1


def test_delete_of_a_missing_map_is_a_404(client):
    assert _delete(client, "nope").status_code == 404


def test_delete_refuses_the_active_map(client, maps_dir):
    response = _delete(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "map_active"
    assert (maps_dir / "full" / "gridmap.pgm").is_file()


def test_delete_refuses_while_a_conversion_is_running(client, maps_dir):
    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("rawonly")
    try:
        response = _delete(client, "rawonly")
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert (maps_dir / "rawonly" / "map.pcd").is_file()


def test_delete_refuses_while_a_task_template_is_bound(
    client, maps_dir, map_repo, task_template_repo
):
    """The one refusal that asks for work rather than a restart."""
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    task_template_repo.create_task_template(
        name="patrol", description="", map_name="rawonly", steps=[]
    )

    response = _delete(client, "rawonly")

    assert response.status_code == 409
    body = response.json()
    assert body["code"] == "template_bound"
    assert "'patrol'" in body["detail"]
    # Nothing was touched -- the refusal comes before either store is written.
    assert (maps_dir / "rawonly" / "map.pcd").is_file()
    assert len(map_repo.list_vertices(map="rawonly")) == 1


def test_delete_ignores_map_independent_templates(client, maps_dir, task_template_repo):
    """map_name IS NULL is a run-anywhere template; no map going away concerns it."""
    task_template_repo.create_task_template(
        name="stand up", description="", map_name=None, steps=[]
    )

    assert _delete(client, "rawonly").status_code == 200
    assert not (maps_dir / "rawonly").exists()


@pytest.mark.parametrize("name", ["../evil", "a/b", ".", "x" * 65])
def test_delete_rejects_bad_names(client, maps_dir, name):
    response = _delete(client, name)

    # Separators never reach the route (FastAPI does not match the path), the
    # rest die in resolve_dir or as a missing map. Nothing is ever a 200.
    assert response.status_code in (400, 404, 405, 422)
    assert (maps_dir / "rawonly").is_dir()


def test_delete_leaves_the_directory_when_the_database_fails(
    client, maps_dir, map_repo, monkeypatch
):
    """Rows first, rmtree last: a failed DELETE must not cost the map."""
    def _boom(map):
        raise RuntimeError("database is away")

    monkeypatch.setattr(map_repo, "delete_vertices", _boom)

    response = _delete(client, "rawonly")

    assert response.status_code == 502
    assert (maps_dir / "rawonly" / "map.pcd").is_file()


def test_delete_drops_the_cached_renderings(client, maps_dir, make_pcd):
    """Warm the cloud cache, delete, and check the entry does not outlive the map."""
    make_pcd(maps_dir / "rawonly" / "map.pcd")
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 200

    assert _delete(client, "rawonly").status_code == 200

    # The caches are closure-local to init_map_router, so this is the only way
    # to observe them: a stale entry would answer 200 for a map that is gone.
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 404


# --- the background gridmap conversion ---------------------------------------
#
# _start_grid_conversion is exercised directly rather than through POST
# /api/v1/maps: the route answers as soon as the pcd is on disk and the
# conversion runs on a daemon thread, so going through the client would mean
# asserting against a race.


@pytest.fixture
def conversion_threads(monkeypatch):
    """Every Thread the code under test starts, plus anything that escaped one.

    Looking the thread up in ``threading.enumerate()`` by name is the obvious
    alternative and it is racy in both directions: a conversion that finishes
    before the lookup is already gone from the list, so "not found" cannot tell
    "done" from "never started".

    The ``excepthook`` half is what makes the failure cases mean anything. Every
    assertion there is about a file *not* appearing, and a thread that died on an
    unhandled exception satisfies that just as well as one that logged and
    returned — which is exactly how an ImportError raised outside the handler's
    try block passed as a green test once already.
    """
    class _Threads(list):
        """A list with room for the escaped-exception log."""

        escaped: list

    started = _Threads()
    escaped = []
    real_thread = threading.Thread

    def _record(*args, **kwargs):
        thread = real_thread(*args, **kwargs)
        started.append(thread)
        return thread

    monkeypatch.setattr(threading, "Thread", _record)
    monkeypatch.setattr(threading, "excepthook", lambda args: escaped.append(args))
    started.escaped = escaped
    return started


def _join(threads, timeout=10.0):
    """Join every recorded thread and assert none of them died on an exception."""
    for thread in threads:
        thread.join(timeout)
        assert not thread.is_alive(), f"{thread.name} did not finish"
    assert not threads.escaped, (
        "an exception escaped the conversion thread: "
        + ", ".join(f"{a.exc_type.__name__}: {a.exc_value}" for a in threads.escaped)
    )


@pytest.fixture
def fake_traversable(monkeypatch):
    """Stand in for helpers.traversable, which needs open3d.

    The route imports it *inside* the thread body, so injecting a module into
    ``sys.modules`` is enough — and that indirection is itself worth pinning: a
    module-level import would pull open3d into every backend start.
    """
    module = types.ModuleType("syncai_backend.helpers.traversable")
    module.calls = []
    module.raises = None

    def build_traversable_cloud(logger, pcd_path, *, segment, repair, debug_dir):
        module.calls.append(
            dict(pcd_path=pcd_path, segment=segment, repair=repair, debug_dir=debug_dir)
        )
        if module.raises is not None:
            raise module.raises
        # A 2 m x 2 m sheet at 5 cm, i.e. a plausible repaired floor.
        axis = np.arange(0.0, 2.0, 0.05)
        xx, yy = np.meshgrid(axis, axis)
        return np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])

    module.build_traversable_cloud = build_traversable_cloud
    monkeypatch.setitem(sys.modules, "syncai_backend.helpers.traversable", module)
    return module


def _sheet(size, step=0.5, z=0.0):
    """A flat square of points ``size`` metres on a side, as a list of xyz."""
    axis = np.arange(0.0, size, step)
    xx, yy = np.meshgrid(axis, axis)
    return [(float(x), float(y), z) for x, y in zip(xx.ravel(), yy.ravel())]


@pytest.fixture
def saved_map(maps_dir, make_pcd):
    """A map directory with a large-site map.pcd, as save_maps would leave it.

    Sized like a warehouse (60 m x 60 m of bbox) because the tests that use it
    exercise the *traversability* path — which since 2026-09 runs only on
    request, never by size, so these tests reach it through
    ``recipe_request="traversability"`` / the re-convert endpoint rather than by
    making the fixture big enough to trip a threshold that no longer exists.
    """
    directory = maps_dir / "newmap"
    os.makedirs(directory, exist_ok=True)
    # A Path, not a str: the make_pcd factory writes through Path.write_text.
    make_pcd(directory / "map.pcd", points=_sheet(60.0, step=2.0))
    return str(directory)


@pytest.fixture
def small_saved_map(maps_dir, make_pcd):
    """A map directory whose footprint puts it on the z-band side of the split.

    Carries a floor sheet at a **non-zero** z plus a block standing on it,
    because the z-band branch is the one under test here and it needs both: a
    floor to call free and something in the obstacle band to call occupied. The
    floor at -0.4 rather than 0 is what makes the recentring assertion mean
    something — held absolute, the shipped bands would land somewhere else.

    A block with real footprint rather than a single column, because
    ``despeckle_min_size`` is 12 and a column puts all its points in one cell:
    the obstacle survives the projection and is then deleted as speckle, which
    reads in the log as a conversion that simply found no obstacles.
    """
    directory = maps_dir / "smallmap"
    os.makedirs(directory, exist_ok=True)
    floor_z = -0.4
    points = _sheet(20.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (5.0 + float(bx), 5.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    return str(directory)


def test_conversion_writes_a_gridmap_from_the_traversable_cloud(
    logger, saved_map, fake_traversable, conversion_threads
):
    started = map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    assert started is True
    assert fake_traversable.calls[0]["pcd_path"] == os.path.join(saved_map, "map.pcd")
    # Both recipes are empty dicts: the pipeline runs at the helper's tuned
    # defaults, and passing {} is what says so rather than silently omitting it.
    assert fake_traversable.calls[0]["segment"] == {}
    assert fake_traversable.calls[0]["repair"] == {}
    # debug_dir off by default: the intermediates would inflate the size the
    # catalogue reports for every map.
    assert fake_traversable.calls[0]["debug_dir"] is None
    assert os.path.isfile(os.path.join(saved_map, "gridmap.pgm"))
    grid = yaml.safe_load(open(os.path.join(saved_map, "gridmap.yaml")))
    assert grid["image"] == "gridmap.pgm"


def test_conversion_passes_a_debug_dir_when_one_is_configured(
    logger, saved_map, fake_traversable, conversion_threads, monkeypatch
):
    monkeypatch.setattr(map_router_module, "TRAVERSABLE_DEBUG_SUBDIR", "traversable_debug")

    map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )


def test_conversion_passes_a_debug_dir_when_the_request_asks(
    logger, saved_map, fake_traversable, conversion_threads
):
    """debug=True is the per-request tuning interface — the module constant
    stays None so ordinary conversions never inflate the catalogue's sizes."""
    map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability", debug=True
    )
    _join(conversion_threads)

    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )


def test_conversion_reports_a_failed_segmentation_instead_of_dying(
    logger, saved_map, fake_traversable, conversion_threads
):
    """A site whose intensity window selects no floor raises ValueError. The map
    ends up without a grid — the route has already answered 200 by then."""
    fake_traversable.raises = ValueError("intensity/normal gate selected no ground points")

    assert (
        map_router_module._start_grid_conversion(
            logger, "newmap", saved_map, recipe_request="traversability"
        )
        is True
    )
    _join(conversion_threads)

    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


# --- what a conversion records about itself -----------------------------------
#
# The sidecar is the only thing a conversion leaves that outlives its thread and
# its process, so it is also the only place an outcome can be read back from.
# These tests pin the three states it writes; the catalogue tests above pin how
# they are reported.


def test_a_conversion_records_itself_before_doing_any_work(
    logger, small_saved_map, conversion_threads, monkeypatch
):
    """The `converting` record has to land before the pipeline runs, not after.

    It is what an abandoned conversion is detected by, and a conversion is
    abandoned precisely when it never reaches its own ending — so a record
    written on the way out would be missing in the one case it exists for.
    Blocking inside measure_cloud is what makes "before" observable at all; the
    real thing is over in tens of seconds and nothing can be asserted mid-run.
    """
    entered = threading.Event()
    release = threading.Event()
    real_measure = map_router_module.measure_cloud

    def _blocking(bound, pcd_path):
        entered.set()
        assert release.wait(10.0), "the test never released the conversion"
        return real_measure(bound, pcd_path)

    monkeypatch.setattr(map_router_module, "measure_cloud", _blocking)

    map_router_module._start_grid_conversion(logger, "smallmap", small_saved_map)
    assert entered.wait(10.0), "the conversion thread never reached measure_cloud"

    mid_run = _sidecar(small_saved_map)
    assert mid_run["status"] == "converting"
    assert mid_run["recipe"] == "z-band"
    assert mid_run["started_at"].endswith("Z")
    # No ending recorded yet, which is exactly what `interrupted` keys off.
    assert "finished_at" not in mid_run

    release.set()
    _join(conversion_threads)

    assert _sidecar(small_saved_map)["status"] == "ok"


def test_a_successful_conversion_records_ok_and_keeps_the_diagnostics(
    logger, small_saved_map, conversion_threads
):
    map_router_module._start_grid_conversion(logger, "smallmap", small_saved_map)
    _join(conversion_threads)

    side = _sidecar(small_saved_map)
    assert side["status"] == "ok"
    assert side["started_at"].endswith("Z")
    assert side["finished_at"] >= side["started_at"]
    assert "error" not in side
    # The status is additive: everything the sidecar carried before it still has
    # to be there, because the recipe record is what tells two maps converted by
    # different recipes apart.
    assert side["recipe"] == "z-band"
    assert side["params"]["floor_z"] == pytest.approx(-0.4, abs=0.1)


def test_a_failed_conversion_records_the_reason(
    logger, saved_map, fake_traversable, conversion_threads
):
    """The same sentence the log line carries, put where a client can read it."""
    fake_traversable.raises = ValueError("intensity/normal gate selected no ground points")

    map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["status"] == "failed"
    assert "selected no ground points" in side["error"]
    assert side["recipe"] == "traversability"
    assert side["finished_at"].endswith("Z")
    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


def test_a_missing_open3d_is_recorded_as_a_failure_naming_the_fix(
    logger, saved_map, conversion_threads, monkeypatch
):
    """An environment fault, not a bad cloud: re-converting this map will fail
    identically until the container is fixed, so the operator has to be told
    which of the two it is rather than being invited to retry."""
    real_import = builtins.__import__

    def _no_open3d(name, *args, **kwargs):
        if name == "syncai_backend.helpers.traversable":
            raise ImportError("No module named 'open3d'")
        return real_import(name, *args, **kwargs)

    monkeypatch.delitem(sys.modules, "syncai_backend.helpers.traversable", raising=False)
    monkeypatch.setattr(builtins, "__import__", _no_open3d)

    map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["status"] == "failed"
    assert "open3d" in side["error"]
    assert "pip3 install" in side["hint"]


def test_a_failed_conversion_releases_the_slot(
    logger, saved_map, fake_traversable, conversion_threads
):
    """The registry entry must not outlive the thread, or the map is stuck
    unconvertible until a backend restart."""
    fake_traversable.raises = ValueError("no ground")
    map_router_module._start_grid_conversion(
        logger, "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    assert not map_router_module._is_converting("newmap")
    # And a second attempt is accepted rather than 409'd.
    fake_traversable.raises = None
    assert (
        map_router_module._start_grid_conversion(
            logger, "newmap", saved_map, recipe_request="traversability"
        )
        is True
    )
    _join(conversion_threads)
    assert os.path.isfile(os.path.join(saved_map, "gridmap.pgm"))


def test_a_running_conversion_conflicts(logger, saved_map):
    """Two threads writing the same gridmap.pgm would interleave outputs."""
    from syncai_backend.exceptions import ConflictError

    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("newmap")
    try:
        with pytest.raises(ConflictError) as exc:
            map_router_module._start_grid_conversion(logger, "newmap", saved_map)
        assert exc.value.code == "conversion_running"
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("newmap")


def test_conversion_survives_open3d_being_absent(
    logger, saved_map, conversion_threads, monkeypatch
):
    """ImportError is caught on its own: uncaught it would kill the thread and
    land as a bare traceback with nothing naming the map being saved."""
    real_import = builtins.__import__

    def _no_open3d(name, *args, **kwargs):
        if name == "syncai_backend.helpers.traversable":
            raise ImportError("No module named 'open3d'")
        return real_import(name, *args, **kwargs)

    monkeypatch.delitem(sys.modules, "syncai_backend.helpers.traversable", raising=False)
    monkeypatch.setattr(builtins, "__import__", _no_open3d)

    assert (
        map_router_module._start_grid_conversion(
            logger, "newmap", saved_map, recipe_request="traversability"
        )
        is True
    )
    _join(conversion_threads)

    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


def _sidecar(directory):
    path = os.path.join(directory, map_router_module.GRIDMAP_RECIPE_SIDECAR)
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


@pytest.fixture
def glassy_saved_map(maps_dir, make_pcd):
    """The conference failure in miniature: a huge bbox over a small floor.

    A dense 20 m floor sheet at -0.4 with a block on it (the real hall), plus a
    handful of points 4 m up and 60 m out — out-of-hall structure seen through
    glass. The bbox comes out ~3600 m² while the floor stays ~400 m², which is
    exactly the shape that used to trip the removed footprint threshold into the
    traversability recipe and produce the 59%-occupied blob.
    """
    directory = maps_dir / "glassy"
    os.makedirs(directory, exist_ok=True)
    floor_z = -0.4
    points = _sheet(20.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (5.0 + float(bx), 5.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    points += [(60.0, 60.0, 4.0), (58.0, 61.0, 4.5), (61.0, 58.0, 5.0)]
    make_pcd(directory / "map.pcd", points=points)
    return str(directory)


def test_the_default_recipe_is_z_band_whatever_the_footprint(
    logger, glassy_saved_map, fake_traversable, conversion_threads
):
    """The conference regression: a glass-inflated bbox must not change the
    recipe, because no recipe is picked by size any more.

    The empty ``calls`` list is the assertion that matters twice over: the
    traversability pipeline did not run, and — since the open3d import sits
    inside that branch — was never even imported.
    """
    map_router_module._start_grid_conversion(logger, "glassy", glassy_saved_map)
    _join(conversion_threads)

    assert fake_traversable.calls == []
    side = _sidecar(glassy_saved_map)
    assert side["recipe"] == "z-band"
    # Both diagnostics are recorded, and their divergence is the fingerprint of
    # a glass-inflated cloud: a big bbox over little floor.
    assert side["footprint_m2"] > 3000.0
    assert side["floor_area_m2"] < 600.0
    assert "threshold_m2" not in side


def test_the_z_band_sidecar_carries_both_area_diagnostics(
    logger, small_saved_map, fake_traversable, conversion_threads
):
    map_router_module._start_grid_conversion(logger, "smallmap", small_saved_map)
    _join(conversion_threads)

    assert fake_traversable.calls == []
    side = _sidecar(small_saved_map)
    assert side["recipe"] == "z-band"
    # A 20 m sheet at 0.2 m spacing: bbox and covered floor agree to within the
    # 0.5 m measuring cell's boundary over-count.
    assert side["footprint_m2"] == pytest.approx(400.0, rel=0.15)
    assert side["floor_area_m2"] == pytest.approx(400.0, rel=0.15)
    assert "recipe_override" not in side
    # No poses.txt in the fixture, so the connectivity filter must not claim to
    # have run.
    assert "pose_filter" not in side["params"]


def test_a_requested_traversability_conversion_records_the_override(
    logger, saved_map, fake_traversable, conversion_threads
):
    override = {"requested": "traversability", "picked_by": "test", "reason": None}
    map_router_module._start_grid_conversion(
        logger,
        "newmap",
        saved_map,
        recipe_request="traversability",
        override=override,
    )
    _join(conversion_threads)

    assert len(fake_traversable.calls) == 1
    side = _sidecar(saved_map)
    assert side["recipe"] == "traversability"
    assert side["recipe_override"] == override


def test_grid_overrides_reach_the_traversable_projection_and_the_sidecar(
    logger, saved_map, fake_traversable, conversion_threads
):
    """gap_fill_size is the acknowledged per-site knob (dp1f's bottom aisle);
    an override must land in the conversion and be readable off the sidecar."""
    map_router_module._start_grid_conversion(
        logger,
        "newmap",
        saved_map,
        recipe_request="traversability",
        grid_overrides={"gap_fill_size": 1.0},
    )
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["params"]["grid"] == {"gap_fill_size": 1.0}


def test_the_z_band_recipe_produces_a_trinary_map(
    logger, small_saved_map, fake_traversable, conversion_threads
):
    """The reason the split exists: unknown survives, and unknown is recoverable.

    A traversability output has no unknown cells at all — everything the cloud
    does not cover is wall, permanently, since costmap_layer.cpp:90 can lower a
    NO_INFORMATION master cell but never a LETHAL one.
    """
    map_router_module._start_grid_conversion(logger, "smallmap", small_saved_map)
    _join(conversion_threads)

    grid = cv2.imread(os.path.join(small_saved_map, "gridmap.pgm"), cv2.IMREAD_UNCHANGED)
    present = set(np.unique(grid).tolist())
    assert 205 in present, "no unknown cells — this is not a trinary map"
    assert 254 in present, "no free cells — the floor band missed the floor"
    assert 0 in present, "no occupied cells — the obstacle band missed the column"


def test_the_z_band_bands_are_recentred_on_the_measured_floor(
    logger, small_saved_map, fake_traversable, conversion_threads
):
    """Absolute bands are a per-site guess; z=0 is only the lidar mount height.

    The fixture's floor is at -0.4, so every band must come back shifted by
    roughly that much from the offsets — not at the constants the fleet's older
    maps were built with.
    """
    map_router_module._start_grid_conversion(logger, "smallmap", small_saved_map)
    _join(conversion_threads)

    params = _sidecar(small_saved_map)["params"]
    floor_z = params["floor_z"]
    assert floor_z == pytest.approx(-0.4, abs=0.1)
    for key, offset in map_router_module.GRIDMAP_BANDS_ABOVE_FLOOR.items():
        assert params[key] == pytest.approx(offset + floor_z, abs=0.01)
    # The floor band actually brackets the fixture's floor, which is the whole
    # point of measuring it rather than trusting the constant.
    assert params["floor_zmin"] < -0.4 < params["floor_zmax"]


def test_conversion_is_skipped_without_a_pcd(logger, maps_dir, conversion_threads):
    """pgo reported success but wrote nothing: report False, start no thread."""
    directory = str(maps_dir / "newmap")
    os.makedirs(directory, exist_ok=True)

    assert map_router_module._start_grid_conversion(logger, "newmap", directory) is False
    assert list(conversion_threads) == []


# --- the pose-connectivity filter through the conversion ----------------------


def test_z_band_conversion_reverts_free_space_the_poses_cannot_reach(
    logger, maps_dir, make_pcd, conversion_threads
):
    """The glass-leak regression in miniature: two floor sheets 8 m apart, poses
    on one. The undriven, unconnected sheet must come back unknown — 205, not 0,
    so real driving could still clear it."""
    directory = maps_dir / "twosheets"
    os.makedirs(directory)
    floor_z = -0.4
    points = _sheet(4.0, step=0.2, z=floor_z)
    points += [(12.0 + x, y, z) for x, y, z in _sheet(4.0, step=0.2, z=floor_z)]
    # A block on the driven sheet: the z-band conversion refuses a cloud with an
    # empty obstacle band.
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    (directory / "poses.txt").write_text(
        "".join(
            f"{i}.pcd {x} {y} {floor_z} 1 0 0 0\n"
            for i, (x, y) in enumerate([(2.5, 2.5), (3.0, 2.0), (2.0, 3.0)])
        )
    )

    map_router_module._start_grid_conversion(logger, "twosheets", str(directory))
    _join(conversion_threads)

    side = _sidecar(str(directory))
    stats = side["params"]["pose_filter"]
    assert stats["applied"] == 1
    assert stats["reverted_free_cells"] > 0
    assert stats["components_kept"] >= 1

    grid = cv2.imread(str(directory / "gridmap.pgm"), cv2.IMREAD_UNCHANGED)
    meta = yaml.safe_load((directory / "gridmap.yaml").read_text())
    res, (ox, oy, _) = meta["resolution"], meta["origin"]

    def cell(x, y):
        # pgm row 0 is max y — the writer's flip.
        return grid[grid.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)]

    assert cell(2.5, 2.5) == 254, "the driven sheet lost its free space"
    assert cell(14.0, 2.0) == 205, "the unreachable sheet is still free"


def test_z_band_conversion_survives_a_malformed_poses_txt(
    logger, maps_dir, make_pcd, conversion_threads
):
    """A broken poses.txt costs the filter, never the gridmap."""
    directory = maps_dir / "badposes"
    os.makedirs(directory)
    floor_z = -0.4
    points = _sheet(4.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    (directory / "poses.txt").write_text("not a pose line\n")

    map_router_module._start_grid_conversion(logger, "badposes", str(directory))
    _join(conversion_threads)

    assert os.path.isfile(directory / "gridmap.pgm")
    assert "pose_filter" not in _sidecar(str(directory))["params"]


# --- POST /api/v1/maps/{name}/grid/convert ------------------------------------


def _post_convert(client, name, payload=None):
    return client.post(f"/api/v1/maps/{name}/grid/convert", json=payload or {})


def test_convert_endpoint_runs_the_requested_traversability_recipe(
    client, saved_map, fake_traversable, conversion_threads, map_gw
):
    response = _post_convert(
        client, "newmap", {"recipe": "traversability", "reason": "big warehouse"}
    )
    _join(conversion_threads)

    assert response.status_code == 200
    body = response.json()
    assert body["started"] is True
    assert body["recipe"] == "traversability"
    assert len(fake_traversable.calls) == 1
    side = _sidecar(saved_map)
    assert side["recipe"] == "traversability"
    assert side["recipe_override"]["requested"] == "traversability"
    assert side["recipe_override"]["reason"] == "big warehouse"
    assert side["recipe_override"]["picked_by"] == (
        "POST /api/v1/maps/newmap/grid/convert"
    )
    # newmap is not the active map ('full' is): no reload.
    assert map_gw.calls == []


def test_convert_endpoint_defaults_to_z_band(
    client, maps_dir, saved_map, make_pcd, fake_traversable, conversion_threads
):
    """An empty body re-converts with the safe default, whatever the size."""
    # The 60 m sheet alone has nothing in the obstacle band, so give it a block.
    points = _sheet(60.0, step=2.0)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(maps_dir / "newmap" / "map.pcd", points=points)

    response = _post_convert(client, "newmap")
    _join(conversion_threads)

    assert response.status_code == 200
    assert response.json()["recipe"] == "z-band"
    assert fake_traversable.calls == []
    assert _sidecar(saved_map)["recipe"] == "z-band"


def test_convert_endpoint_reloads_the_active_map(
    client, map_gw, conversion_threads
):
    """'full' is the active map; a re-convert that map_server never hears about
    would leave it serving the grid the operator just replaced."""
    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    assert len(map_gw.calls) == 1
    assert map_gw.calls[0].endswith("full/gridmap.yaml")


def test_convert_endpoint_archives_the_previous_grid(
    client, maps_dir, conversion_threads
):
    previous = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    previous_yaml = (maps_dir / "full" / "gridmap.yaml").read_bytes()

    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    assert "gridmap_prev.pgm" in response.json()["message"]
    assert (maps_dir / "full" / "gridmap_prev.pgm").read_bytes() == previous
    assert (maps_dir / "full" / "gridmap_prev.yaml").read_bytes() == previous_yaml
    # The new conversion really replaced the grid (the fixture's 6x4 became the
    # pcd's real extent).
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() != previous


def test_convert_endpoint_archives_the_previous_recipe_record(
    client, maps_dir, conversion_threads
):
    """The incoming conversion overwrites the sidecar with its own `converting`
    record before it does any work, so the outgoing one has to move aside with
    the grid it describes — otherwise a re-convert that then fails leaves a map
    serving the archived grid with nothing on disk saying how it was made."""
    _plant_sidecar(
        maps_dir / "full",
        {"status": "ok", "recipe": "z-band", "params": {"floor_z": -0.4}},
    )

    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    prev = json.loads(
        (maps_dir / "full" / "gridmap_prev.recipe.json").read_text(encoding="utf-8")
    )
    assert prev["recipe"] == "z-band"
    assert prev["params"] == {"floor_z": -0.4}
    # ...and the live record is the new run's, not the archived one.
    assert _sidecar(str(maps_dir / "full"))["status"] == "ok"
    assert "params" in _sidecar(str(maps_dir / "full"))


def test_convert_endpoint_refuses_to_discard_hand_edits(
    client, maps_dir, make_pgm, conversion_threads
):
    """A raw snapshot differing from the live grid means operator work; silently
    re-converting over it would destroy it."""
    make_pgm(maps_dir / "full" / "gridmap_raw.pgm", 6, 4, fill=0)

    response = _post_convert(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "gridmap_hand_edited"
    assert list(conversion_threads) == []


def test_convert_endpoint_overwrites_hand_edits_only_on_confirmation(
    client, maps_dir, make_pgm, conversion_threads
):
    make_pgm(maps_dir / "full" / "gridmap_raw.pgm", 6, 4, fill=0)
    edited = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    response = _post_convert(client, "full", {"overwrite_edits": True})
    _join(conversion_threads)

    assert response.status_code == 200
    # The edited grid survives as the prev generation...
    assert (maps_dir / "full" / "gridmap_prev.pgm").read_bytes() == edited
    # ...its stale raw snapshot moved aside with it, so the next editor save
    # snapshots the *new* conversion rather than trusting a wrong-era raw...
    assert not (maps_dir / "full" / "gridmap_raw.pgm").exists()
    assert (maps_dir / "full" / "gridmap_prev_raw.pgm").exists()
    # ...and the live grid is the fresh conversion.
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() != edited


def test_convert_endpoint_conflicts_while_a_conversion_runs(client):
    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("full")
    try:
        response = _post_convert(client, "full")
        assert response.status_code == 409
        assert response.json()["code"] == "conversion_running"
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("full")


def test_grid_status_is_converting_while_the_slot_is_held(client):
    """Both fields, read from one sample of the registry: `grid_converting` is a
    deprecated alias of `grid_status == "converting"`, and an alias that can
    disagree with what it aliases is worse than no alias."""
    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("full")
    try:
        entry = _by_name(client.get("/api/v1/maps").json())["full"]
        assert entry["grid_status"] == "converting"
        assert entry["grid_converting"] is True
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("full")

    entry = _by_name(client.get("/api/v1/maps").json())["full"]
    assert entry["grid_status"] == "ok"
    assert entry["grid_converting"] is False


def test_convert_endpoint_400s_without_a_pcd(client, maps_dir, conversion_threads):
    (maps_dir / "full" / "map.pcd").unlink()

    response = _post_convert(client, "full")

    assert response.status_code == 400
    assert list(conversion_threads) == []


def test_convert_endpoint_404s_for_a_missing_map(client):
    assert _post_convert(client, "nosuchmap").status_code == 404


def test_convert_endpoint_rejects_cross_recipe_parameters(client, conversion_threads):
    z_with_gap = _post_convert(client, "full", {"recipe": "z-band", "gap_fill_size": 1.0})
    trav_with_bands = _post_convert(
        client,
        "full",
        {"recipe": "traversability", "z_band_offsets": {"zmax": 3.0}},
    )

    assert z_with_gap.status_code == 400
    assert trav_with_bands.status_code == 400
    assert list(conversion_threads) == []


def test_convert_endpoint_rejects_inverted_merged_bands(client, conversion_threads):
    """The override merges over the recipe's other end, so one wrong number can
    invert a band — that must die here, not half a minute later in the thread."""
    response = _post_convert(
        client, "full", {"recipe": "z-band", "z_band_offsets": {"floor_zmax": -0.5}}
    )

    assert response.status_code == 400
    assert "floor band is inverted" in response.json()["detail"]
    assert list(conversion_threads) == []


def test_convert_endpoint_422s_an_out_of_range_gap_fill(client, conversion_threads):
    response = _post_convert(
        client, "full", {"recipe": "traversability", "gap_fill_size": 5.0}
    )

    assert response.status_code == 422
    assert list(conversion_threads) == []


def test_convert_endpoint_passes_debug_and_overrides_through(
    client, saved_map, fake_traversable, conversion_threads
):
    response = _post_convert(
        client,
        "newmap",
        {"recipe": "traversability", "gap_fill_size": 1.2, "debug": True},
    )
    _join(conversion_threads)

    assert response.status_code == 200
    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )
    side = _sidecar(saved_map)
    assert side["params"]["grid"] == {"gap_fill_size": 1.2}
    assert side["recipe_override"]["param_overrides"] == {"gap_fill_size": 1.2}


# --- POST /api/v1/maps/{name}/activate --------------------------------------
#
# The fixture INI pins `full` active, and `full` is the only fixture map with a
# gridmap, so most switch tests point the INI at `rawonly` first and switch *to*
# `full`. Writing the file rather than re-parameterising the fixture is
# deliberate: `active_map_name` re-reads the INI on every call, which is the
# property the route depends on and therefore one worth exercising.

_INTERPOLATED_INI = (
    "[system]\nrobot_id: robot01\n\n"
    "# which map the stack loads\n"
    "[map]\n"
    "name: rawonly\n"
    "pcd: map/%(name)s/map.pcd\n"
    "map: map/%(name)s/gridmap.yaml\n\n"
    "[initial_pose]\n"
    "x: 3.25\ny: -1.5\nz: 0.0\nyaw: 1.57\n"
)


def _point_ini_at(monkeypatch, tmp_path, text):
    ini = tmp_path / "system.ini"
    ini.write_text(text)
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    return ini


def _activate(client, name):
    return client.post(f"/api/v1/maps/{name}/activate")


def test_activate_the_active_map_is_a_noop(client, map_gw):
    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is False
    assert body["previous"] == "full"
    # The point of the no-op: a good registration is not thrown away.
    assert map_gw.order == []


def test_activate_unknown_map_is_a_404(client, map_gw):
    assert _activate(client, "nosuchmap").status_code == 404
    assert map_gw.order == []


def test_activate_refuses_a_map_with_no_gridmap(client, map_gw, monkeypatch, tmp_path):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: full\n")

    response = _activate(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "grid_missing"
    assert map_gw.order == []


def test_activate_refuses_a_map_with_no_pointcloud(
    client, map_gw, maps_dir, make_pgm, make_gridmap_yaml, monkeypatch, tmp_path
):
    gridonly = maps_dir / "gridonly"
    gridonly.mkdir()
    make_pgm(gridonly / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(gridonly / "gridmap.yaml", origin=(0.0, 0.0, 0.0))

    response = _activate(client, "gridonly")

    assert response.status_code == 409
    assert response.json()["code"] == "pointcloud_missing"
    assert map_gw.order == []


def test_activate_refuses_while_a_conversion_runs(client, map_gw, monkeypatch, tmp_path):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")

    with map_router_module._ACTIVE_CONVERSIONS_LOCK:
        map_router_module._ACTIVE_CONVERSIONS.add("full")
    try:
        response = _activate(client, "full")
    finally:
        with map_router_module._ACTIVE_CONVERSIONS_LOCK:
            map_router_module._ACTIVE_CONVERSIONS.discard("full")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert map_gw.order == []


def test_activate_refuses_while_a_task_is_running(
    client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    workflow_gw.tasks = [types.SimpleNamespace(id="task-7")]

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "task-7" in response.json()["detail"]
    assert map_gw.order == []


def test_activate_refuses_when_temporal_cannot_be_reached(
    client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    """A Temporal outage must not be read as "nothing is running"."""
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    workflow_gw.error = RuntimeError("connection refused")

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.order == []


def test_activate_refuses_when_the_nav_stack_is_not_up(
    client, map_gw, monkeypatch, tmp_path
):
    """How "the robot is in mapping mode" is detected."""
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    map_gw.services_ready = False

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "stack_not_ready"
    assert map_gw.order == []


def test_activate_moves_the_localizer_before_map_server(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)

    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is True
    assert body["name"] == "full"
    assert body["previous"] == "rawonly"
    assert body["localized"] is True

    # The ordering is the contract: the localizer's failure is the clean one, so
    # it goes first.
    assert [step for step, _ in map_gw.order] == ["swap_localizer_map", "reload_map"]
    assert map_gw.order[0][1] == str(maps_dir / "full" / "map.pcd")
    assert map_gw.order[1][1] == str(maps_dir / "full" / "gridmap.yaml")
    # Zeroed, not carried over from the old map's frame.
    assert map_gw.swap_calls[0][1:] == (0.0, 0.0, 0.0)

    written = ini.read_text()
    assert "name: full" in written
    # The whole reason the writer edits lines instead of round-tripping through
    # ConfigParser: expanding this would pin pcd/map at the *old* map forever.
    assert "pcd: map/%(name)s/map.pcd" in written
    assert "map: map/%(name)s/gridmap.yaml" in written
    assert "# which map the stack loads" in written
    # A pose measured in rawonly's frame means nothing in full's.
    assert "x: 0.0" in written and "yaw: 0.0" in written
    assert "3.25" not in written


def test_activate_puts_the_localizer_back_when_map_server_refuses(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.result = (False, "map_server rejected gridmap.yaml")

    response = _activate(client, "full")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
    ]
    # Back onto the map it came from, not left straddling two.
    assert map_gw.order[2][1] == str(maps_dir / "rawonly" / "map.pcd")
    assert "name: rawonly" in ini.read_text()


def test_activate_rolls_back_both_steps_when_the_ini_cannot_be_written(
    client, map_gw, maps_dir, make_pcd, make_pgm, make_gridmap_yaml,
    monkeypatch, tmp_path,
):
    """The INI write is preflighted, so reaching it and failing must undo the rest.

    `previous` here is a fully converted map, unlike the pcd-only `rawonly` the
    other switch tests come from: a map the stack actually launched on must have
    had a gridmap.yaml, and it is the rollback of *both* ROS steps that this
    covers.
    """
    prior = maps_dir / "prior"
    prior.mkdir()
    make_pcd(prior / "map.pcd")
    make_pgm(prior / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(prior / "gridmap.yaml", origin=(0.0, 0.0, 0.0))
    _point_ini_at(
        monkeypatch, tmp_path, _INTERPOLATED_INI.replace("name: rawonly", "name: prior")
    )

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(map_router_module, "set_active_map", _boom)

    response = _activate(client, "full")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
        "reload_map",
    ]
    assert map_gw.order[2][1] == str(prior / "map.pcd")
    assert map_gw.order[3][1] == str(prior / "gridmap.yaml")


def test_activate_says_so_when_the_old_grid_cannot_be_restored(
    client, map_gw, monkeypatch, tmp_path
):
    """Rolling back onto a map with no gridmap.yaml leaves a mismatch worth naming."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(map_router_module, "set_active_map", _boom)

    response = _activate(client, "full")

    assert response.status_code == 502
    detail = response.json()["detail"]
    assert "no gridmap.yaml to put back" in detail
    # The localizer still went back; only the grid could not.
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
    ]


def test_activate_refuses_when_the_ini_is_not_writable(
    client, map_gw, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    os.chmod(ini, 0o444)
    try:
        response = _activate(client, "full")
    finally:
        os.chmod(ini, 0o644)

    assert response.status_code == 409
    assert response.json()["code"] == "ini_not_writable"
    assert map_gw.order == []


def test_activate_reports_an_unconverged_localizer(
    client, map_gw, monkeypatch, tmp_path
):
    """A swap that lands but does not converge is a success with a warning."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = False

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is False
    assert "set an initial pose" in body["message"]


def test_activate_reports_an_unreachable_localizer_check(
    client, map_gw, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = None

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is None
    assert "could not be asked" in body["message"]
