import math
import netifaces
import subprocess
import threading
from dataclasses import dataclass, replace

from rclpy import qos
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from syncai_common.msg import WifiNetwork
from syncai_common.msg import WifiStatus as WifiStatusMsg
from syncai_common.srv import ConnectWifiNetwork, ScanWifiNetworks


@dataclass
class WifiNetworkInfo:
    ip_address: str
    mac_address: str


@dataclass
class WifiStatus:
    bssid: str
    ssid: str
    rssi: int
    ip_address: str
    mac_address: str


class WifiManager:
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        self.init_pub()
        self.init_services()
        self.init_timer()

        self._wifi_status_lock = threading.Lock()
        self._current_wifi_status = WifiStatus(
            bssid="", ssid="", rssi=0, ip_address="", mac_address=""
        )

    def init_pub(self):

        self._wifi_status_pub = self._node.create_publisher(
            msg_type=WifiStatusMsg,
            topic="wifi_status",
            qos_profile=qos.QoSProfile(
                history=qos.HistoryPolicy.KEEP_LAST,
                depth=3,
                reliability=qos.ReliabilityPolicy.BEST_EFFORT,
                durability=qos.DurabilityPolicy.VOLATILE,
            ),
        )

    def init_services(self):

        self._node.create_service(
            srv_type=ScanWifiNetworks,
            srv_name="scan_wifi",
            callback=self._scan_available_wifi_networks,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._node.create_service(
            srv_type=ConnectWifiNetwork,
            srv_name="connect_wifi",
            callback=self._connect_wifi_network,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def init_timer(self):

        self._wifi_status_timer = self._node.create_timer(
            timer_period_sec=1.0,
            callback=self._publish_wifi_status,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _publish_wifi_status(self):
        status = self.get_wifi_status()
        self._wifi_status_pub.publish(
            WifiStatusMsg(
                ssid=status.ssid,
                bssid=status.bssid,
                rssi=status.rssi,
                ip_address=status.ip_address,
                mac_address=status.mac_address,
            )
        )

    def get_wifi_status(self) -> WifiStatus:
        with self._wifi_status_lock:
            return replace(self._current_wifi_status)

    def _check_wifi(self) -> bool:

        try:
            result = subprocess.run(
                ["nmcli", "radio", "wifi"], capture_output=True, text=True, timeout=3.0
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError("Failed to check WiFi status within 3 seconds")

        if result.stdout.strip().lower() == "enabled":
            self._logger.info(
                "[WifiManager][check_wifi] Currently wifi is already enabled."
            )
            return True

        return False

    def setup_wifi(self) -> None:

        if self._check_wifi():
            return

        try:
            result = subprocess.run(
                ["sudo", "nmcli", "radio", "wifi", "on"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError("Failed to enable WiFi within 5 seconds")

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to enable WiFi: {result.stderr.strip() or result.stdout.strip()}"
            )

    def _scan_available_wifi_networks(
        self,
        _: ScanWifiNetworks.Request,
        response: ScanWifiNetworks.Response,
    ) -> ScanWifiNetworks.Response:
        # set is used to avoid duplicate SSID entries
        seen_networks = set()

        # Trigger a rescan (needs root: polkit denies wifi.scan for
        # session-less processes such as ours inside the container)
        try:
            subprocess.run(
                ["sudo", "nmcli", "device", "wifi", "rescan"],
                capture_output=True,
                text=True,
                timeout=10.0,
            )
        except subprocess.TimeoutExpired:
            self._logger.warn(
                "[WifiManager][scan_wifi_networks] Rescan timed out, proceeding with cached results"
            )

        try:
            result = subprocess.run(
                ["nmcli", "-f", "BSSID,SIGNAL,SSID", "device", "wifi", "list"],
                capture_output=True,
                text=True,
                timeout=30.0,
            )
        except subprocess.TimeoutExpired:
            self._logger.error(
                "[WifiManager][scan_wifi_networks] Scan WiFi networks timeout expired"
            )
            response.success = False
            response.message = "Scan WiFi networks timed out"
            return response

        if result.returncode != 0:
            response.success = False
            response.message = (
                f"Failed to list WiFi networks: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )
            return response

        lines = result.stdout.strip().splitlines()[1:]
        for line in lines:
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            bssid = parts[0]
            signal = int(parts[1])
            ssid = " ".join(parts[2:])
            # "--" is nmcli's placeholder for hidden SSIDs
            if not ssid or ssid == "--" or ssid in seen_networks:
                continue

            seen_networks.add(ssid)
            response.networks.append(
                WifiNetwork(bssid=bssid, ssid=ssid, rssi=math.ceil((signal / 2) - 100))
            )

        response.success = True
        response.message = ""
        return response

    def _connect_wifi_network(
        self,
        request: ConnectWifiNetwork.Request,
        response: ConnectWifiNetwork.Response,
    ) -> ConnectWifiNetwork.Response:
        ssid = request.ssid
        if not ssid:
            response.success = False
            response.message = "SSID must not be empty"
            return response

        # List-form args: SSID/password go through exec directly, so shell
        # metacharacters in user input are harmless. Omit the password args
        # entirely for open networks.
        command = ["sudo", "nmcli", "device", "wifi", "connect", ssid]
        if request.password:
            command += ["password", request.password]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=60.0,
            )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = f"Timed out connecting to {ssid} within 60 seconds"
            return response

        if result.returncode != 0:
            response.success = False
            response.message = (
                f"Failed to connect to {ssid}: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )
            return response

        self._logger.info(
            f"[WifiManager][connect_wifi] Successfully connected to WiFi network {ssid}"
        )
        response.success = True
        response.message = ""
        return response

    def get_wifi_info(self) -> WifiNetworkInfo:
        interfaces = netifaces.interfaces()

        # Prefer wifi interfaces (wl*)
        wifi_interfaces = [i for i in interfaces if i.startswith("wl")]
        for interface in wifi_interfaces:
            info = self._read_addresses(interface)
            if info.mac_address:
                return info

        return WifiNetworkInfo(ip_address="", mac_address="")

    @staticmethod
    def _read_addresses(interface: str) -> WifiNetworkInfo:
        addr_info = netifaces.ifaddresses(interface)

        link_entries = addr_info.get(netifaces.AF_LINK, [])
        mac = link_entries[0].get("addr", "") if link_entries else ""
        if mac == "00:00:00:00:00:00":
            mac = ""

        inet_entries = addr_info.get(netifaces.AF_INET, [])
        ip = inet_entries[0].get("addr", "") if inet_entries else ""

        return WifiNetworkInfo(ip_address=ip, mac_address=mac)

    def update_wifi_status(self):
        try:
            result = subprocess.run(
                [
                    "nmcli",
                    "-f",
                    "IN-USE,BSSID,SIGNAL,SSID",
                    "device",
                    "wifi",
                    "list",
                    "--rescan",
                    "no",
                ],
                capture_output=True,
                text=True,
                timeout=10.0,
            )
        except subprocess.TimeoutExpired:
            raise TimeoutError("Failed to update WiFi status within 10 seconds")

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to update WiFi status: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )

        # The IN-USE column marks the currently connected network with "*"
        active_line = next(
            (
                line.strip()
                for line in result.stdout.splitlines()[1:]
                if line.strip().startswith("*")
            ),
            None,
        )

        wifi_info = self.get_wifi_info()

        with self._wifi_status_lock:
            self._current_wifi_status.ip_address = wifi_info.ip_address
            self._current_wifi_status.mac_address = wifi_info.mac_address

            if active_line is None:
                self._current_wifi_status.ssid = ""
                self._current_wifi_status.bssid = ""
                self._current_wifi_status.rssi = 0
                return

            parts = active_line.split()
            self._current_wifi_status.bssid = parts[1]
            self._current_wifi_status.ssid = " ".join(parts[3:])
            self._current_wifi_status.rssi = math.ceil((int(parts[2]) / 2) - 100)


def init_wifi_manager(node: Node) -> WifiManager:
    node.get_logger().info(
        "[WifiManager][init_wifi_manager] Initializing Wifi Management module"
    )
    wifi_manager = WifiManager(node=node)
    try:
        wifi_manager.setup_wifi()
        wifi_manager.update_wifi_status()
    except Exception as err:
        node.get_logger().error(
            f"[WifiManager][init_wifi_manager] Failed to initialize Wifi Management module: {str(err)}"
        )

    return wifi_manager
