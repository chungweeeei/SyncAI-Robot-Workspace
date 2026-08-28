"""Tests for the two point-cloud WebSocket streams.

Same shape as test_maps_router.py: the router is mounted on a bare FastAPI app
and the repos are real (a PointCloudRepo is a lock and a slot — nothing to
stub). TestClient's websocket_connect runs the endpoint's own loop, so what is
under test is the actual pump: seq bookkeeping, the wire header, and the two
endpoints draining two independent repos.

Every receive is armed by an update_frame first, so no test depends on a
timeout to make progress. Since the pump became frame-driven that is load
bearing rather than merely tidy: on an empty repo it blocks on the repo's
Event indefinitely, so a receive with nothing seeded would hang instead of
arriving a poll interval late. It also means `test_stream_sends_a_frame_only_
once_per_seq` now covers the cross-thread wakeup itself — TestClient runs the
endpoint on its own loop, so the mid-connection `_seed` reaches the blocked
pump only via `call_soon_threadsafe`.
"""

import struct

import pytest

pytest.importorskip("numpy")
pytest.importorskip("httpx")

import numpy as np  # noqa: E402
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.pointcloud import pack_xyz_f32  # noqa: E402
from syncai_backend.interfaces.rest.routers.pointcloud import (  # noqa: E402
    init_pointcloud_router,
)
from syncai_backend.repositories.pointcloud.pointcloud import (  # noqa: E402
    init_pointcloud_repo,
)

LIVE_PATH = "/api/v1/robot/pointcloud/stream"
MAP_PATH = "/api/v1/robot/pointcloud/map/stream"


@pytest.fixture
def live_repo(logger):
    return init_pointcloud_repo(logger=logger)


@pytest.fixture
def map_repo_slot(logger):
    """The second slot. Not named map_repo — that fixture is the vertex table."""
    return init_pointcloud_repo(logger=logger)


@pytest.fixture
def client(logger, live_repo, map_repo_slot):
    app = FastAPI()
    app.include_router(
        init_pointcloud_router(
            logger=logger,
            pointcloud_repo=live_repo,
            map_cloud_repo=map_repo_slot,
        )
    )
    return TestClient(app)


def _seed(repo, points):
    data = np.asarray(points, dtype=np.float32)
    repo.update_frame(num_points=data.shape[0], data=pack_xyz_f32(data))
    return data


def _expect_frame(ws, expected: np.ndarray):
    payload = ws.receive_bytes()
    (count,) = struct.unpack_from("<I", payload, 0)
    assert count == expected.shape[0]
    received = np.frombuffer(payload, dtype="<f4", offset=4).reshape(-1, 3)
    assert np.array_equal(received, expected)


@pytest.mark.parametrize("path", [LIVE_PATH, MAP_PATH])
def test_stream_sends_the_seeded_frame(client, live_repo, map_repo_slot, path):
    repo = live_repo if path == LIVE_PATH else map_repo_slot
    expected = _seed(repo, [(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)])

    with client.websocket_connect(path) as ws:
        _expect_frame(ws, expected)


def test_stream_sends_a_frame_only_once_per_seq(client, live_repo, map_repo_slot):
    """The pump advances its seq cursor: one update, one send — then silence
    until the next update, whose (different) content proves the cursor moved."""
    first = _seed(live_repo, [(1.0, 1.0, 1.0)])

    with client.websocket_connect(LIVE_PATH) as ws:
        _expect_frame(ws, first)

        second = _seed(live_repo, [(2.0, 2.0, 2.0), (3.0, 3.0, 3.0)])
        _expect_frame(ws, second)


def test_streams_drain_independent_repos(client, live_repo, map_repo_slot):
    """The two endpoints must not share a slot or a cursor."""
    live = _seed(live_repo, [(1.0, 0.0, 0.0)])
    merged = _seed(map_repo_slot, [(0.0, 1.0, 0.0), (0.0, 0.0, 1.0)])

    with client.websocket_connect(LIVE_PATH) as live_ws:
        _expect_frame(live_ws, live)
    with client.websocket_connect(MAP_PATH) as map_ws:
        _expect_frame(map_ws, merged)
