"""Tests for the WS teleop endpoint — the first WebSocket test in this repo.

TestClient.websocket_connect drives the ASGI app in a background portal, so
frames sent here are processed in order before the disconnect message, and the
``with`` block's exit waits for the handler to finish — which is what makes
asserting on the post-disconnect zero-stops deterministic.

What is pinned: frame parsing (floats reach the gateway), the error-frame
contract (refusals and malformed frames answer without killing the socket),
the stop-on-disconnect repeats, and the stale-input watchdog (patched down
from 0.5 s so the test doesn't sleep for real).
"""

import time

import pytest

pytest.importorskip("httpx")
pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
pytest.importorskip("syncai_common")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.routers import teleop as teleop_module  # noqa: E402
from syncai_backend.interfaces.rest.routers.teleop import init_teleop_router  # noqa: E402


class _StubRobotGateway:
    def __init__(self):
        self.commands = []
        self.stops = 0
        self.refusal = None  # set to a message to refuse every command

    def teleop_cmd_vel(self, vx, vy, wz):
        if self.refusal is not None:
            return False, self.refusal
        self.commands.append((vx, vy, wz))
        return True, ""

    def teleop_stop(self):
        self.stops += 1


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def client(logger, robot_gw):
    app = FastAPI()
    app.include_router(init_teleop_router(logger=logger, robot_gw=robot_gw))
    return TestClient(app)


def test_frames_reach_the_gateway_as_floats(client, robot_gw):
    with client.websocket_connect("/api/v1/robot/teleop") as ws:
        ws.send_json({"vx": 0.5, "vy": 0.0, "wz": -0.25})
        ws.send_json({"vx": 1, "vy": 0, "wz": 0})  # ints are fine, float() them

    assert robot_gw.commands == [(0.5, 0.0, -0.25), (1.0, 0.0, 0.0)]


def test_a_refusal_answers_an_error_frame_and_keeps_the_socket(client, robot_gw):
    robot_gw.refusal = "autonomous move in progress"

    with client.websocket_connect("/api/v1/robot/teleop") as ws:
        ws.send_json({"vx": 0.5, "vy": 0.0, "wz": 0.0})
        assert ws.receive_json() == {"error": "autonomous move in progress"}

        # Socket must survive the refusal: cancel-the-task-then-drive is the
        # intended flow, on this same connection.
        robot_gw.refusal = None
        ws.send_json({"vx": 0.1, "vy": 0.0, "wz": 0.0})

    assert robot_gw.commands == [(0.1, 0.0, 0.0)]


def test_malformed_frames_are_skipped_not_fatal(client, robot_gw):
    with client.websocket_connect("/api/v1/robot/teleop") as ws:
        ws.send_text("not json")
        assert "malformed" in ws.receive_json()["error"]

        ws.send_json({"vx": 0.5})  # missing keys
        assert "malformed" in ws.receive_json()["error"]

        ws.send_json({"vx": "fast", "vy": 0, "wz": 0})  # non-numeric
        assert "malformed" in ws.receive_json()["error"]

        ws.send_json({"vx": 0.2, "vy": 0.0, "wz": 0.0})

    assert robot_gw.commands == [(0.2, 0.0, 0.0)]


def test_disconnect_repeats_the_zero_stop(client, robot_gw):
    with client.websocket_connect("/api/v1/robot/teleop") as ws:
        ws.send_json({"vx": 0.5, "vy": 0.0, "wz": 0.0})

    # The driver->gait hop is fire-and-forget UDP; a single zero can be lost.
    assert robot_gw.stops >= teleop_module._STOP_REPEATS


def test_watchdog_zeroes_while_the_client_is_quiet(client, robot_gw, monkeypatch):
    monkeypatch.setattr(teleop_module, "_WATCHDOG_S", 0.05)

    with client.websocket_connect("/api/v1/robot/teleop") as ws:
        ws.send_json({"vx": 0.5, "vy": 0.0, "wz": 0.0})
        # Go quiet; the handler runs concurrently in the portal thread, so
        # real time passing here is real time for its watchdog.
        time.sleep(0.3)

    # At least one watchdog stop beyond the disconnect repeats.
    assert robot_gw.stops > teleop_module._STOP_REPEATS
    assert robot_gw.commands == [(0.5, 0.0, 0.0)]
