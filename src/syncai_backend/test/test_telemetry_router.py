"""Tests for the telemetry WebSocket stream.

Same shape as test_pointcloud_router.py: the router is mounted on a bare
FastAPI app over a real TelemetryRepo (three slots and a lock — nothing to
stub), and TestClient's websocket_connect runs the endpoint's own loop. What is
under test is the pump: the JSON wire frames, the per-type seq cursors, and the
three types multiplexing over one socket at independent rates. The repo's own
semantics (seq gating, the path TTL) are pinned in test_telemetry_repo.py and
not re-tested here.

No test waits out the 50 ms poll interval on an empty repo — every receive is
armed by an update first, so the pump always has a sample on its next tick.
"""

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.routers.telemetry import (  # noqa: E402
    init_telemetry_router,
)
from syncai_backend.repositories.telemetry.telemetry import (  # noqa: E402
    init_telemetry_repo,
)

STREAM_PATH = "/api/v1/robot/telemetry/stream"


@pytest.fixture
def telemetry_repo(logger):
    return init_telemetry_repo(logger=logger)


@pytest.fixture
def client(logger, telemetry_repo):
    app = FastAPI()
    app.include_router(
        init_telemetry_router(logger=logger, telemetry_repo=telemetry_repo)
    )
    return TestClient(app)


def test_a_pose_arrives_as_a_pose_frame(client, telemetry_repo):
    # Full-dict equality on purpose: it pins that the repo's internal seq does
    # NOT leak onto the wire, and that yaw ships in degrees (the frontend's
    # RobotPose.theta vocabulary) exactly as the repo cached it.
    telemetry_repo.update_pose(x=1.5, y=-2.5, z=0.25, yaw_deg=90.0, stamp=10.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json() == {
            "type": "pose",
            "x": 1.5,
            "y": -2.5,
            "z": 0.25,
            "yaw_deg": 90.0,
            "stamp": 10.0,
        }


def test_joints_arrive_keyed_by_urdf_joint_name(client, telemetry_repo):
    telemetry_repo.update_joints(
        joints={"FL_HipX_joint": 0.5, "FR_Knee_joint": -1.2}, stamp=11.0
    )

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json() == {
            "type": "joints",
            "joints": {"FL_HipX_joint": 0.5, "FR_Knee_joint": -1.2},
            "stamp": 11.0,
        }


def test_a_path_arrives_as_xy_pairs(client, telemetry_repo):
    # The repo stores tuples; JSON has no tuple, so the wire shape is nested
    # lists — which is what the frontend indexes into.
    telemetry_repo.update_path(points=((0.0, 0.0), (1.0, 2.0)), stamp=12.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json() == {
            "type": "path",
            "points": [[0.0, 0.0], [1.0, 2.0]],
            "stamp": 12.0,
        }


def test_an_empty_path_is_a_real_frame(client, telemetry_repo):
    # The "no route" sample. It must reach the wire — it is the only thing
    # that ever tells a client to erase the band it is drawing (the absence of
    # further path frames could not).
    telemetry_repo.update_path(points=(), stamp=13.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json() == {"type": "path", "points": [], "stamp": 13.0}


def test_all_three_types_multiplex_over_one_socket(client, telemetry_repo):
    # Seeded before connect, so the pump's very first tick drains all three
    # slots. The pose → joints → path order is the loop's own and therefore
    # observable wire behavior, so it is pinned rather than sorted away.
    telemetry_repo.update_pose(x=1.0, y=2.0, z=0.0, yaw_deg=0.0, stamp=1.0)
    telemetry_repo.update_joints(joints={"FL_HipX_joint": 0.1}, stamp=2.0)
    telemetry_repo.update_path(points=((0.0, 0.0),), stamp=3.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert [ws.receive_json()["type"] for _ in range(3)] == [
            "pose",
            "joints",
            "path",
        ]


def test_a_sample_is_sent_only_once_per_seq(client, telemetry_repo):
    """The pump advances its cursor: one update, one frame — then silence
    until the next update, whose (different) content proves the cursor moved."""
    telemetry_repo.update_pose(x=1.0, y=0.0, z=0.0, yaw_deg=0.0, stamp=1.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json()["x"] == 1.0

        telemetry_repo.update_pose(x=2.0, y=0.0, z=0.0, yaw_deg=0.0, stamp=1.05)
        assert ws.receive_json()["x"] == 2.0


def test_the_type_cursors_are_independent(client, telemetry_repo):
    # A joints update must not drag a resend of the unchanged pose with it:
    # the very next frame after the pose is the joints frame, nothing between.
    telemetry_repo.update_pose(x=1.0, y=0.0, z=0.0, yaw_deg=0.0, stamp=1.0)

    with client.websocket_connect(STREAM_PATH) as ws:
        assert ws.receive_json()["type"] == "pose"

        telemetry_repo.update_joints(joints={"FL_HipX_joint": 0.5}, stamp=2.0)
        frame = ws.receive_json()
        assert frame["type"] == "joints"
        assert frame["joints"] == {"FL_HipX_joint": 0.5}
