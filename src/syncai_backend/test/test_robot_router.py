"""Unit tests for the robot REST router: the pure helpers plus the POST routes.

The router is mounted on a bare FastAPI app with the production exception
handlers registered, so the domain-exception -> status-code mapping under test is
the real one (InternalServerError -> 502). RobotRepo is real — it is a lock and a
slot — and only the gateway is stubbed, because a real one needs a live DDS graph
with syncai_driver_manager on it.
"""

import pytest

pytest.importorskip("syncai_common")
# The router imports RobotGateway, which pulls in rclpy, nav2_msgs and
# action_msgs; without these the module import would be an ImportError rather
# than a skip. httpx is TestClient's transport.
pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_common.msg import RobotMode  # noqa: E402

from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.interfaces.rest.routers.robot import (  # noqa: E402
    RobotNetworkStatus,
    init_robot_router,
    _mode_to_str,
    _parse_wifi_info,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)
from syncai_backend.repositories.robot.robot import init_robot_repo  # noqa: E402


class _StubRobotGateway:
    """Records service calls instead of making them.

    The one thing this suite cannot make real: every method here needs a live
    driver_manager on a DDS graph. ``motion_keys`` / ``policy_modes`` are what
    the "did not reach the gateway" assertions read — an empty list is the claim.
    """

    def __init__(self):
        self.motion_keys = []
        self.policy_modes = []
        self.result = (True, "Motion key sent")

    def set_motion_key(self, key):
        self.motion_keys.append(key)
        return self.result

    def set_policy_mode(self, mode):
        self.policy_modes.append(mode)
        return self.result


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def client(logger, robot_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_robot_router(
            logger=logger,
            robot_repo=init_robot_repo(logger=logger),
            robot_gw=robot_gw,
        )
    )
    return TestClient(app)


def _post_motion_key(client, payload):
    return client.post("/api/v1/robot/set_motion_key", json=payload)


def _post_policy_mode(client, payload):
    return client.post("/api/v1/robot/set_policy_mode", json=payload)


# --- helpers ----------------------------------------------------------------


def test_mode_to_str_known_modes():
    assert _mode_to_str(RobotMode.AUTO) == "AUTO"
    assert _mode_to_str(RobotMode.MANUAL) == "MANUAL"
    assert _mode_to_str(RobotMode.MAINTENANCE) == "MAINTENANCE"


def test_mode_to_str_unknown_mode():
    assert _mode_to_str(255) == "UNKNOWN"


def test_parse_wifi_info_valid_json():
    info = _parse_wifi_info('{"ssid": "net", "rssi": -40, "ip_address": "10.0.0.2"}')

    assert isinstance(info, RobotNetworkStatus)
    assert info.ssid == "net"
    assert info.rssi == -40
    assert info.ip_address == "10.0.0.2"


@pytest.mark.parametrize("payload", ["null", "not-json", "", "[1, 2, 3]"])
def test_parse_wifi_info_falls_back_to_defaults(payload):
    info = _parse_wifi_info(payload)

    assert isinstance(info, RobotNetworkStatus)
    assert info.ssid == ""
    assert info.rssi == 0


# --- POST /api/v1/robot/set_motion_key --------------------------------------


@pytest.mark.parametrize("key", ["0", "1", "2", "3", "5"])
def test_each_forwardable_key_reaches_the_gateway(client, robot_gw, key):
    response = _post_motion_key(client, {"key": key})

    assert response.status_code == 200
    assert response.json()["sent"] is True
    assert robot_gw.motion_keys == [MotionKey(key)]
    # A MotionKey member, not the bare string: the gateway reads `key.value`, so
    # handing it a str would raise inside it. This is what pins that contract.
    assert isinstance(robot_gw.motion_keys[0], MotionKey)


def test_estop_key_is_accepted_but_not_forwarded(client, robot_gw):
    response = _post_motion_key(client, {"key": "4"})

    assert response.status_code == 200
    body = response.json()
    assert body["key"] == "4"
    assert body["sent"] is False
    assert robot_gw.motion_keys == []


def test_estop_response_says_no_estop_was_sent(client):
    """The point is that this 200 cannot be misread as "stop engaged"."""
    message = _post_motion_key(client, {"key": "4"}).json()["message"]

    assert "no ESTOP" in message


@pytest.mark.parametrize("payload", [{"key": "9"}, {"key": "z"}, {"key": ""}, {}])
def test_invalid_motion_key_is_422_before_the_gateway(client, robot_gw, payload):
    assert _post_motion_key(client, payload).status_code == 422
    assert robot_gw.motion_keys == []


def test_motion_key_gateway_failure_is_502_with_the_message_verbatim(client, robot_gw):
    """LOCKED is the driver's own string; the operator has to see it.

    Unreachable on hardware today — nothing calls triggerSafeShutdown(), so
    safe_lock_ is never set. This pins the router's mapping, not an observed
    driver behaviour.
    """
    robot_gw.result = (False, "LOCKED")

    response = _post_motion_key(client, {"key": "0"})

    assert response.status_code == 502
    assert response.json()["detail"] == "LOCKED"


# --- POST /api/v1/robot/set_policy_mode -------------------------------------


@pytest.mark.parametrize("mode", [0, 1])
def test_exposed_policy_modes_reach_the_gateway(client, robot_gw, mode):
    response = _post_policy_mode(client, {"mode": mode})

    assert response.status_code == 200
    assert response.json()["mode"] == mode
    # A plain int, which is the gateway's contract and what keeps the srv field a
    # bare uint8.
    assert robot_gw.policy_modes == [mode]


@pytest.mark.parametrize("mode", [2, 3, 300, -1, "PPO"])
def test_unexposed_or_illegal_policy_mode_is_422(client, robot_gw, mode):
    """2 (CHAMP) and 3 (ISSAC) are legal on the controller but not exposed."""
    assert _post_policy_mode(client, {"mode": mode}).status_code == 422
    assert robot_gw.policy_modes == []


def test_policy_mode_gateway_failure_is_502(client, robot_gw):
    robot_gw.result = (False, "set_policy_mode service is not available")

    response = _post_policy_mode(client, {"mode": 1})

    assert response.status_code == 502
    assert response.json()["detail"] == "set_policy_mode service is not available"
