import subprocess
from collections import deque
from unittest import mock

import netifaces
import pytest

from syncai_common.msg import WifiNetwork
from syncai_common.srv import (
    ActivateWifiConnection,
    ConnectWifiNetwork,
    DeleteWifiConnection,
    ListWifiConnections,
    ScanWifiNetworks,
)

from syncai_sys_manager.managers.wifi_manager import (
    WifiManager,
    WifiNetworkInfo,
    WifiStatus,
    split_terse,
)

# Plain pytest + unittest.mock on purpose. This file used to need assertpy and
# pytest-mock, neither of which the image installs (and `python3-assertpy-pip`
# is not even a rosdep key), so it died at import — which hid for months that
# it was written against the Popen + shell-string WifiManager and had drifted
# away from the subprocess.run + argv-list one almost entirely.

NMCLI_STATUS = [
    "nmcli", "-f", "IN-USE,BSSID,SIGNAL,SSID", "device", "wifi", "list", "--rescan", "no",
]
NMCLI_RESCAN = ["sudo", "nmcli", "device", "wifi", "rescan"]
NMCLI_LIST = ["nmcli", "-f", "BSSID,SIGNAL,SSID", "device", "wifi", "list"]


def done(stdout: str = "", returncode: int = 0, stderr: str = ""):
    return subprocess.CompletedProcess(args=[], returncode=returncode, stdout=stdout,
                                       stderr=stderr)


def timeout():
    return subprocess.TimeoutExpired(cmd="nmcli", timeout=1.0)


class FakeRun:
    """Stand-in for subprocess.run: replays queued outcomes, records argv."""

    def __init__(self):
        self.outcomes = deque()
        self.calls = []

    def queue(self, *outcomes):
        self.outcomes.extend(outcomes)

    def __call__(self, args, **kwargs):
        self.calls.append(args)
        outcome = self.outcomes.popleft()
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


@pytest.fixture
def run(monkeypatch) -> FakeRun:
    fake = FakeRun()
    monkeypatch.setattr(subprocess, "run", fake)
    return fake


@pytest.fixture
def ifaces(monkeypatch):
    """Install a fake netifaces view: {interface: (mac, ip)}."""

    def install(table: dict):
        def ifaddresses(interface):
            mac, ip = table[interface]
            addrs = {netifaces.AF_LINK: [{"addr": mac}]}
            if ip:
                addrs[netifaces.AF_INET] = [{"addr": ip}]
            return addrs

        monkeypatch.setattr(netifaces, "interfaces", lambda: list(table))
        monkeypatch.setattr(netifaces, "ifaddresses", ifaddresses)

    return install


@pytest.fixture
def node() -> mock.MagicMock:
    # WifiManager only touches the node through get_logger / create_publisher /
    # create_service / create_timer; a MagicMock swallows all of them.
    return mock.MagicMock()


@pytest.fixture
def manager(node) -> WifiManager:
    return WifiManager(node=node)


def test_init_registers_services_and_timers(node):
    WifiManager(node=node)

    services = [c.kwargs["srv_name"] for c in node.create_service.call_args_list]
    periods = [c.kwargs["timer_period_sec"] for c in node.create_timer.call_args_list]
    assert services == [
        "scan_wifi",
        "connect_wifi",
        "list_wifi_connections",
        "activate_wifi_connection",
        "delete_wifi_connection",
    ]
    assert periods == [1.0, 5.0]


def test_initial_status_is_empty(manager):
    assert manager.get_wifi_status() == WifiStatus(
        bssid="", ssid="", rssi=0, ip_address="", mac_address=""
    )


def test_get_wifi_status_returns_a_copy(manager):
    manager.get_wifi_status().ssid = "mutated"

    assert manager.get_wifi_status().ssid == ""


# --- setup_wifi --------------------------------------------------------------


def test_setup_wifi_already_enabled(manager, run):
    run.queue(done("enabled\n"))

    manager.setup_wifi()

    assert run.calls == [["nmcli", "radio", "wifi"]]


def test_setup_wifi_turns_radio_on(manager, run):
    run.queue(done("disabled\n"), done())

    manager.setup_wifi()

    assert run.calls == [["nmcli", "radio", "wifi"], ["sudo", "nmcli", "radio", "wifi", "on"]]


def test_setup_wifi_check_timeout(manager, run):
    run.queue(timeout())

    with pytest.raises(TimeoutError, match="Failed to check WiFi status within 3 seconds"):
        manager.setup_wifi()


def test_setup_wifi_enable_timeout(manager, run):
    run.queue(done("disabled\n"), timeout())

    with pytest.raises(TimeoutError, match="Failed to enable WiFi within 5 seconds"):
        manager.setup_wifi()


def test_setup_wifi_enable_fails(manager, run):
    run.queue(done("disabled\n"), done(returncode=1, stderr="Error: not authorized"))

    with pytest.raises(RuntimeError, match="Failed to enable WiFi: Error: not authorized"):
        manager.setup_wifi()


# --- get_wifi_info -----------------------------------------------------------


def test_get_wifi_info_reads_wifi_interface(manager, ifaces):
    ifaces({
        "lo": ("00:00:00:00:00:00", "127.0.0.1"),
        "eth0": ("11:11:11:11:11:11", "10.0.0.2"),
        "wlan0": ("22:22:22:22:22:22", "192.168.0.100"),
    })

    assert manager.get_wifi_info() == WifiNetworkInfo(
        ip_address="192.168.0.100", mac_address="22:22:22:22:22:22"
    )


def test_get_wifi_info_ignores_non_wifi_interfaces(manager, ifaces):
    ifaces({"lo": ("00:00:00:00:00:00", "127.0.0.1"), "eth0": ("11:11:11:11:11:11", "10.0.0.2")})

    assert manager.get_wifi_info() == WifiNetworkInfo(ip_address="", mac_address="")


def test_get_wifi_info_skips_zero_mac(manager, ifaces):
    ifaces({
        "wlP1p1s0": ("00:00:00:00:00:00", ""),
        "wlan1": ("33:33:33:33:33:33", ""),
    })

    assert manager.get_wifi_info() == WifiNetworkInfo(
        ip_address="", mac_address="33:33:33:33:33:33"
    )


# --- update_wifi_status ------------------------------------------------------


STATUS_OUTPUT = (
    "IN-USE  BSSID              SIGNAL  SSID\n"
    "        34:3A:20:0C:4C:D2  60      Other\n"
    "*       34:3A:20:0C:4C:D3  75      My WiFi Network\n"
)


def test_update_wifi_status_parses_active_network(manager, run, ifaces):
    ifaces({"wlan0": ("22:22:22:22:22:22", "192.168.0.100")})
    run.queue(done(STATUS_OUTPUT))

    manager.update_wifi_status()

    assert run.calls == [NMCLI_STATUS]
    assert manager.get_wifi_status() == WifiStatus(
        bssid="34:3A:20:0C:4C:D3",
        ssid="My WiFi Network",
        rssi=-62,
        ip_address="192.168.0.100",
        mac_address="22:22:22:22:22:22",
    )


def test_update_wifi_status_clears_when_disconnected(manager, run, ifaces):
    ifaces({"wlan0": ("22:22:22:22:22:22", "192.168.0.100")})
    run.queue(done(STATUS_OUTPUT))
    manager.update_wifi_status()

    ifaces({"wlan0": ("22:22:22:22:22:22", "")})
    run.queue(done("IN-USE  BSSID              SIGNAL  SSID\n"))
    manager.update_wifi_status()

    assert manager.get_wifi_status() == WifiStatus(
        bssid="", ssid="", rssi=0, ip_address="", mac_address="22:22:22:22:22:22"
    )


def test_update_wifi_status_timeout(manager, run):
    run.queue(timeout())

    with pytest.raises(TimeoutError, match="Failed to update WiFi status within 10 seconds"):
        manager.update_wifi_status()


def test_update_wifi_status_nmcli_error(manager, run):
    run.queue(done(returncode=10, stdout="Error: NetworkManager is not running."))

    with pytest.raises(RuntimeError, match="NetworkManager is not running"):
        manager.update_wifi_status()


# --- scan_wifi service -------------------------------------------------------


def scan(manager) -> ScanWifiNetworks.Response:
    return manager._scan_available_wifi_networks(
        ScanWifiNetworks.Request(), ScanWifiNetworks.Response()
    )


def test_scan_lists_networks_deduplicated(manager, run):
    run.queue(
        done(),
        done(
            "BSSID              SIGNAL  SSID\n"
            "34:3A:20:0C:4C:D3  84      Test1\n"
            "34:3A:20:0C:4C:D2  82      Test 2\n"
            "34:3A:20:0C:4C:D4  70      Test1\n"
            "34:3A:20:0C:4C:D5  50      --\n"
        ),
    )

    response = scan(manager)

    assert run.calls == [NMCLI_RESCAN, NMCLI_LIST]
    assert response.success
    assert response.networks == [
        WifiNetwork(bssid="34:3A:20:0C:4C:D3", ssid="Test1", rssi=-58),
        WifiNetwork(bssid="34:3A:20:0C:4C:D2", ssid="Test 2", rssi=-59),
    ]


def test_scan_proceeds_when_rescan_times_out(manager, run):
    run.queue(timeout(), done("BSSID  SIGNAL  SSID\n34:3A:20:0C:4C:D3  84  Test1\n"))

    response = scan(manager)

    assert response.success
    assert [n.ssid for n in response.networks] == ["Test1"]


def test_scan_list_timeout(manager, run):
    run.queue(done(), timeout())

    response = scan(manager)

    assert not response.success
    assert response.message == "Scan WiFi networks timed out"
    assert response.networks == []


def test_scan_list_nmcli_error(manager, run):
    run.queue(done(), done(returncode=8, stderr="Error: no wifi device"))

    response = scan(manager)

    assert not response.success
    assert response.message == "Failed to list WiFi networks: Error: no wifi device"


# --- connect_wifi service ----------------------------------------------------


def connect(manager, ssid: str, password: str = "") -> ConnectWifiNetwork.Response:
    return manager._connect_wifi_network(
        ConnectWifiNetwork.Request(ssid=ssid, password=password),
        ConnectWifiNetwork.Response(),
    )


def test_connect_rejects_empty_ssid(manager, run):
    response = connect(manager, "")

    assert not response.success
    assert response.message == "SSID must not be empty"
    assert run.calls == []


def test_connect_with_password_refreshes_status(manager, run, ifaces):
    ifaces({"wlan0": ("22:22:22:22:22:22", "192.168.0.100")})
    run.queue(done("Device 'wlan0' successfully activated."), done(STATUS_OUTPUT))

    # A shell metacharacter in the SSID must reach nmcli as one argv element.
    response = connect(manager, "My WiFi Network; reboot", "p@ss word")

    assert response.success
    assert run.calls == [
        ["sudo", "nmcli", "device", "wifi", "connect", "My WiFi Network; reboot",
         "password", "p@ss word"],
        NMCLI_STATUS,
    ]
    assert manager.get_wifi_status().ssid == "My WiFi Network"


def test_connect_open_network_omits_password(manager, run, ifaces):
    ifaces({})
    run.queue(done(), done(STATUS_OUTPUT))

    connect(manager, "Open")

    assert run.calls[0] == ["sudo", "nmcli", "device", "wifi", "connect", "Open"]


def test_connect_succeeds_even_if_refresh_fails(manager, run):
    run.queue(done(), timeout())

    response = connect(manager, "MyWiFiNetwork", "MyPassword")

    assert response.success


def test_connect_timeout(manager, run):
    run.queue(timeout())

    response = connect(manager, "MyWiFiNetwork", "MyPassword")

    assert not response.success
    assert response.message == "Timed out connecting to MyWiFiNetwork within 60 seconds"


def test_connect_nmcli_error(manager, run):
    run.queue(done(returncode=4, stderr="Error: Connection activation failed."))

    response = connect(manager, "MyWiFiNetwork", "MyPassword")

    assert not response.success
    assert response.message == (
        "Failed to connect to MyWiFiNetwork: Error: Connection activation failed."
    )
    assert len(run.calls) == 1


# --- saved connections -------------------------------------------------------


NMCLI_CONNECTIONS = [
    "nmcli", "-t", "-f", "NAME,UUID,TYPE,TIMESTAMP,ACTIVE", "connection", "show",
]


def nmcli_ssid(uuid: str) -> list:
    return ["nmcli", "-g", "802-11-wireless.ssid", "connection", "show", "uuid", uuid]


# A wired profile in the middle, and a WiFi name with an escaped ':' in it.
CONNECTIONS_OUTPUT = (
    "site-wifi:aaaa-1:802-11-wireless:1760000000:yes\n"
    "Wired connection 1:bbbb-2:802-3-ethernet:1760000100:yes\n"
    "Lab\\: 5G:cccc-3:802-11-wireless:0:no\n"
)


def list_connections(manager) -> ListWifiConnections.Response:
    return manager._list_wifi_connections(
        ListWifiConnections.Request(), ListWifiConnections.Response()
    )


def queue_connections(run):
    run.queue(done(CONNECTIONS_OUTPUT), done("site-wifi\n"), done("lab\\:5g\n"))


def test_split_terse_honours_escapes():
    assert split_terse("a\\:b:c\\\\d:") == ["a:b", "c\\d", ""]


def test_list_keeps_wifi_profiles_only(manager, run):
    queue_connections(run)

    response = list_connections(manager)

    assert response.success
    assert run.calls == [NMCLI_CONNECTIONS, nmcli_ssid("aaaa-1"), nmcli_ssid("cccc-3")]
    assert [(c.uuid, c.name, c.ssid, c.active, c.last_used) for c in response.connections] == [
        ("aaaa-1", "site-wifi", "site-wifi", True, 1760000000),
        ("cccc-3", "Lab: 5G", "lab:5g", False, 0),
    ]


def test_list_falls_back_to_name_when_ssid_unreadable(manager, run):
    run.queue(done("Office:dddd-4:802-11-wireless:0:no\n"), timeout())

    response = list_connections(manager)

    assert response.connections[0].ssid == "Office"


def test_list_reports_nmcli_failure(manager, run):
    run.queue(done(returncode=8, stderr="Error: NetworkManager is not running."))

    response = list_connections(manager)

    assert not response.success
    assert response.message == (
        "Failed to list saved WiFi networks: Error: NetworkManager is not running."
    )


def activate(manager, uuid: str) -> ActivateWifiConnection.Response:
    return manager._activate_wifi_connection(
        ActivateWifiConnection.Request(uuid=uuid), ActivateWifiConnection.Response()
    )


def test_activate_brings_profile_up_and_refreshes(manager, run, ifaces):
    ifaces({})
    queue_connections(run)
    run.queue(done("Connection successfully activated"), done(STATUS_OUTPUT))

    response = activate(manager, "cccc-3")

    assert response.success
    assert response.ssid == "lab:5g"
    assert run.calls[3:] == [["sudo", "nmcli", "connection", "up", "uuid", "cccc-3"], NMCLI_STATUS]


def test_activate_unknown_uuid(manager, run):
    queue_connections(run)

    response = activate(manager, "gone")

    assert not response.success
    assert response.message == "That saved network no longer exists"
    assert len(run.calls) == 3


def test_activate_nmcli_error_names_the_network(manager, run):
    queue_connections(run)
    run.queue(done(returncode=4, stderr="Error: Secrets were required."))

    response = activate(manager, "cccc-3")

    assert not response.success
    assert response.message == "Failed to connect to lab:5g: Error: Secrets were required."


def delete(manager, uuid: str) -> DeleteWifiConnection.Response:
    return manager._delete_wifi_connection(
        DeleteWifiConnection.Request(uuid=uuid), DeleteWifiConnection.Response()
    )


def test_delete_removes_profile(manager, run):
    queue_connections(run)
    run.queue(done("Connection 'Lab: 5G' successfully deleted."))

    response = delete(manager, "cccc-3")

    assert response.success
    assert run.calls[3] == ["sudo", "nmcli", "connection", "delete", "uuid", "cccc-3"]


def test_delete_refuses_the_active_profile(manager, run):
    queue_connections(run)

    response = delete(manager, "aaaa-1")

    assert not response.success
    assert response.message == "Cannot forget site-wifi while the robot is connected to it"
    assert len(run.calls) == 3
