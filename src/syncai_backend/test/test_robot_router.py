"""Unit tests for pure helpers in the robot REST router."""

import pytest

pytest.importorskip("syncai_common")

from syncai_common.msg import RobotMode  # noqa: E402

from syncai_backend.interfaces.rest.routers.robot import (  # noqa: E402
    RobotNetworkStatus,
    _mode_to_str,
    _parse_wifi_info,
)


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
