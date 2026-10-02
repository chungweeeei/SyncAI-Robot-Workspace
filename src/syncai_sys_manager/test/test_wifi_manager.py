import pytest
import subprocess
import netifaces
from unittest import mock
from assertpy import assert_that
from pytest_mock import MockerFixture

from syncai_common.msg import WifiNetwork

from syncai_sys_manager.managers.wifi_manager import (
    WifiStatus,
    WifiManager,
    WifiNetworkInfo,
)


class TestWifiManager:
    @pytest.fixture
    def mock_node(self) -> mock.MagicMock:
        # WifiManager only touches the node through node.get_logger(); a
        # MagicMock stands in for the rclpy Node and swallows log calls.
        return mock.MagicMock()

    @pytest.fixture
    def mock_popen(self, mocker: MockerFixture) -> mock.MagicMock:
        mock_popen = mocker.patch("subprocess.Popen")
        return mock_popen

    @pytest.fixture
    def mock_wifi_manager(self, mock_node: mock.MagicMock) -> WifiManager:
        return WifiManager(node=mock_node)

    def test_setup_wifi_but_wifi_already_turn_on(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.MagicMock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.return_value = (b"enabled\n", None)
        mock_popen.return_value = mock_process

        mock_wifi_manager.setup_wifi()

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli radio wifi",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

    def test_setup_wifi_failed_caused_nmcli_error(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.MagicMock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.side_effect = subprocess.TimeoutExpired(
            cmd="nmcli radio wifi", timeout=3.0
        )
        mock_popen.return_value = mock_process

        with pytest.raises(TimeoutError) as excinfo:
            mock_wifi_manager.setup_wifi()
            assert_that(str(excinfo.value)).is_equal_to(
                "Failed to check WiFi status within 3 seconds"
            )

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli radio wifi",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        mock_process.kill.assert_has_calls(calls=[mock.call()])

    def test_setup_wifi_turn_on_wifi_successfully(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.MagicMock
    ):
        # First call to check wifi status returns "disabled"
        mock_process_check = mock.Mock()
        mock_process_check.communicate.return_value = (b"disabled\n", None)

        # Second call to turn on wifi
        mock_process_turn_on = mock.Mock()
        mock_process_turn_on.communicate.return_value = (b"", None)

        mock_popen.side_effect = [mock_process_check, mock_process_turn_on]

        mock_wifi_manager.setup_wifi()

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli radio wifi",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                ),
                mock.call(
                    "nmcli radio wifi on",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                ),
            ]
        )

    def test_setup_wifi_turn_on_wifi_failed_caused_timeout(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.MagicMock
    ):
        # First call to check wifi status returns "disabled"
        mock_process_check = mock.Mock()
        mock_process_check.communicate.return_value = (b"disabled\n", None)

        # Second call to turn on wifi raises TimeoutExpired
        mock_process_turn_on = mock.Mock()
        mock_process_turn_on.communicate.side_effect = subprocess.TimeoutExpired(
            cmd="nmcli radio wifi on", timeout=5.0
        )

        mock_popen.side_effect = [mock_process_check, mock_process_turn_on]

        with pytest.raises(TimeoutError) as excinfo:
            mock_wifi_manager.setup_wifi()
            assert_that(str(excinfo.value)).is_equal_to(
                "Failed to enable WiFi within 5 seconds"
            )

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli radio wifi",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                ),
                mock.call(
                    "nmcli radio wifi on",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                ),
            ]
        )

        mock_process_turn_on.kill.assert_has_calls(calls=[mock.call()])

    def test_get_mac_address_prefers_wifi_interface(
        self, mocker: MockerFixture, mock_wifi_manager: WifiManager
    ):
        mocker.patch("netifaces.interfaces", return_value=["lo", "eth0", "wlan0"])

        def fake_ifaddresses(interface: str):
            return {
                "eth0": {netifaces.AF_LINK: [{"addr": "11:11:11:11:11:11"}]},
                "wlan0": {netifaces.AF_LINK: [{"addr": "22:22:22:22:22:22"}]},
                "lo": {netifaces.AF_LINK: [{"addr": "00:00:00:00:00:00"}]},
            }[interface]

        mocker.patch("netifaces.ifaddresses", side_effect=fake_ifaddresses)

        mac = mock_wifi_manager.get_mac_address()

        assert_that(mac).is_equal_to("22:22:22:22:22:22")

    def test_get_mac_address_falls_back_to_non_wifi_interface(
        self, mocker: MockerFixture, mock_wifi_manager: WifiManager
    ):
        mocker.patch("netifaces.interfaces", return_value=["lo", "eth0"])

        def fake_ifaddresses(interface: str):
            return {
                "eth0": {netifaces.AF_LINK: [{"addr": "11:11:11:11:11:11"}]},
                "lo": {netifaces.AF_LINK: [{"addr": "00:00:00:00:00:00"}]},
            }[interface]

        mocker.patch("netifaces.ifaddresses", side_effect=fake_ifaddresses)

        mac = mock_wifi_manager.get_mac_address()

        assert_that(mac).is_equal_to("11:11:11:11:11:11")

    def test_get_mac_address_returns_empty_when_no_valid_interface(
        self, mocker: MockerFixture, mock_wifi_manager: WifiManager
    ):
        mocker.patch("netifaces.interfaces", return_value=["lo"])
        mocker.patch(
            "netifaces.ifaddresses",
            return_value={netifaces.AF_LINK: [{"addr": "00:00:00:00:00:00"}]},
        )

        mac = mock_wifi_manager.get_mac_address()

        assert_that(mac).is_equal_to("")

    def test_get_wifi_info_successfully(
        self, mocker: MockerFixture, mock_wifi_manager: WifiManager
    ):
        mock_netifaces = mocker.patch("netifaces.interfaces")
        mock_netifaces.return_value = ["lo", "eth0", "wlan0"]

        mock_ifaddresses = mocker.patch("netifaces.ifaddresses")
        mock_ifaddresses.return_value = {
            netifaces.AF_LINK: [
                {"addr": "f8:3d:c6:91:72:4c", "broadcast": "ff:ff:ff:ff:ff:ff"}
            ],
            netifaces.AF_INET: [
                {
                    "addr": "192.168.0.100",
                    "netmask": "255.255.254.0",
                    "broadcast": "10.8.141.255",
                }
            ],
            netifaces.AF_INET6: [
                {
                    "addr": "fe80::7c55:fcd2:749e:afe2%wlP1p1s0",
                    "netmask": "ffff:ffff:ffff:ffff::/64",
                }
            ],
        }

        wifi_info = mock_wifi_manager.get_wifi_info()

        assert_that(wifi_info).is_type_of(WifiNetworkInfo)
        assert_that(wifi_info.ip_address).is_equal_to("192.168.0.100")
        assert_that(wifi_info.mac_address).is_equal_to("f8:3d:c6:91:72:4c")

    def test_get_wifi_info_but_no_wifi_interface(
        self, mocker: MockerFixture, mock_wifi_manager: WifiManager
    ):
        mock_netifaces = mocker.patch("netifaces.interfaces")
        mock_netifaces.return_value = ["lo", "eth0"]

        wifi_info = mock_wifi_manager.get_wifi_info()

        assert_that(wifi_info).is_type_of(WifiNetworkInfo)
        assert_that(wifi_info.ip_address).is_equal_to("")
        assert_that(wifi_info.mac_address).is_equal_to("")

    def test_get_wifi_status(self, mock_wifi_manager: WifiManager):
        status = mock_wifi_manager.get_wifi_status()

        assert_that(status).is_not_none().is_type_of(WifiStatus)
        assert_that(status.SSID).is_equal_to("N/A")
        assert_that(status.RSSI).is_equal_to(0)

    def test_update_wifi_status_failed_caused_timeout(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.side_effect = subprocess.TimeoutExpired(
            cmd="nmcli -f IN-USE,SIGNAL,SSID device wifi list --rescan no | grep '*'",
            timeout=10.0,
        )
        mock_popen.return_value = mock_process

        with pytest.raises(TimeoutError) as excinfo:
            mock_wifi_manager.update_wifi_status()
            assert_that(str(excinfo.value)).is_equal_to(
                "Failed to get WiFi status within 10 seconds"
            )

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli -f IN-USE,SIGNAL,SSID device wifi list --rescan no | grep '*'",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        mock_process.kill.assert_has_calls(calls=[mock.call()])

    def test_update_wifi_status_successfully(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.return_value = (b"* 75 MyWiFiNetwork\n", None)
        mock_popen.return_value = mock_process

        mock_wifi_manager.update_wifi_status()

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli -f IN-USE,SIGNAL,SSID device wifi list --rescan no | grep '*'",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        status = mock_wifi_manager.get_wifi_status()

        assert_that(status).is_not_none().is_type_of(WifiStatus)
        assert_that(status.SSID).is_equal_to("MyWiFiNetwork")
        assert_that(status.RSSI).is_equal_to(-62)

    def test_scan_wifi_networks_failed_caused_timeout(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.side_effect = subprocess.TimeoutExpired(
            cmd="nmcli -f BSSID,SIGNAL,SSID device wifi list", timeout=30.0
        )
        mock_popen.return_value = mock_process

        networks = mock_wifi_manager.scan_wifi_networks()

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli -f BSSID,SIGNAL,SSID device wifi list",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        mock_process.kill.assert_has_calls(calls=[mock.call()])

        assert_that(networks).is_empty()

    def test_scan_wifi_networks_successfully(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.return_value = (
            b"""BSSID              SIGNAL  SSID
                                                 34:3A:20:0C:4C:D3  84      Test1
                                                 34:3A:20:0C:4C:D2  82      Test2""",
            None,
        )
        mock_popen.return_value = mock_process

        networks = mock_wifi_manager.scan_wifi_networks()

        assert_that(networks).is_length(2)
        assert_that(networks[0]).is_type_of(WifiNetwork)
        assert_that(networks[0].ssid).is_equal_to("Test1")
        assert_that(networks[0].bssid).is_equal_to("34:3A:20:0C:4C:D3")
        assert_that(networks[0].rssi).is_equal_to(-58)
        assert_that(networks[1]).is_type_of(WifiNetwork)
        assert_that(networks[1].ssid).is_equal_to("Test2")
        assert_that(networks[1].bssid).is_equal_to("34:3A:20:0C:4C:D2")
        assert_that(networks[1].rssi).is_equal_to(-59)

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "nmcli -f BSSID,SIGNAL,SSID device wifi list",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        mock_process.kill.assert_not_called()

    def test_connect_wifi_failed_caused_timeout(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.side_effect = subprocess.TimeoutExpired(
            cmd="sudo nmcli device wifi connect 'MyWiFiNetwork' password 'MyPassword'",
            timeout=30.0,
        )
        mock_popen.return_value = mock_process

        with pytest.raises(TimeoutError) as excinfo:
            mock_wifi_manager.connect_wifi(ssid="MyWiFiNetwork", password="MyPassword")
            assert_that(str(excinfo.value)).is_equal_to(
                "Failed to connect to MyWiFiNetwork network within timeout period"
            )

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "sudo nmcli device wifi connect 'MyWiFiNetwork' password 'MyPassword'",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

        mock_process.kill.assert_has_calls(calls=[mock.call()])

    def test_connect_wifi_failed_caused_nmcli_error(
        self, mock_wifi_manager: WifiManager, mock_popen: mock.Mock
    ):
        mock_process = mock.Mock()
        mock_process.communicate.return_value = (b"Error: Connection failed", None)
        mock_popen.return_value = mock_process

        with pytest.raises(RuntimeError) as excinfo:
            mock_wifi_manager.connect_wifi(ssid="MyWiFiNetwork", password="MyPassword")
            assert_that(str(excinfo.value)).is_equal_to(
                "Failed to connect to MyWiFiNetwork network: Error: Connection failed"
            )

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "sudo nmcli device wifi connect 'MyWiFiNetwork' password 'MyPassword'",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

    def test_connect_wifi_successfully(
        self,
        mock_wifi_manager: WifiManager,
        mock_popen: mock.Mock,
    ):
        mock_process = mock.Mock()
        mock_process.communicate.return_value = (
            b"Successfully activated connection",
            None,
        )
        mock_popen.return_value = mock_process

        mock_wifi_manager.connect_wifi(ssid="MyWiFiNetwork", password="MyPassword")

        mock_popen.assert_has_calls(
            calls=[
                mock.call(
                    "sudo nmcli device wifi connect 'MyWiFiNetwork' password 'MyPassword'",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            ]
        )

    def test_init_wifi_manager(self, mock_node: mock.MagicMock):
        wifi_manager = WifiManager(node=mock_node)
        assert_that(wifi_manager).is_not_none().is_type_of(WifiManager)
