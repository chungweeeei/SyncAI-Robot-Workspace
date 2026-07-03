from typing import List

import math
import netifaces
import subprocess
import threading
from dataclasses import dataclass

from rclpy.node import Node


@dataclass
class WifiStatus:
    SSID: str
    RSSI: int


@dataclass
class AvailableNetwork:
    BSSID: str
    SSID: str
    RSSI: int


@dataclass
class WifiNetworkInfo:
    ip_address: str
    mac_address: str


class WifiManager:
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        self._status_lock = threading.Lock()
        self._wifi_status = WifiStatus(SSID="N/A", RSSI=0)

    def setup_wifi(self):
        # check whether wifi is enabled
        try:
            proc = subprocess.Popen(
                "nmcli radio wifi",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output, _ = proc.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            raise TimeoutError("Failed to check WiFi status within 3 seconds")

        output = output.decode("utf-8").strip()
        if output.lower() == "enabled":
            return

        # turn on wifi
        try:
            proc = subprocess.Popen(
                "nmcli radio wifi on",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            proc.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            raise TimeoutError("Failed to enable WiFi within 5 seconds")

    def get_mac_address(self) -> str:
        interfaces = netifaces.interfaces()

        # Prefer wifi interfaces (wl*)
        wifi_interfaces = [i for i in interfaces if i.startswith("wl")]
        for interface in wifi_interfaces:
            mac = self._read_mac(interface)
            if mac:
                return mac

        # Fallback: any non-loopback interface with a valid MAC
        for interface in sorted(interfaces):
            if interface == "lo":
                continue
            mac = self._read_mac(interface)
            if mac:
                return mac

        return ""

    @staticmethod
    def _read_mac(interface: str) -> str:
        addr_info = netifaces.ifaddresses(interface)
        if netifaces.AF_LINK not in addr_info or not addr_info[netifaces.AF_LINK]:
            return ""
        mac = addr_info[netifaces.AF_LINK][0].get("addr", "")
        if not mac or mac == "00:00:00:00:00:00":
            return ""
        return mac

    def get_wifi_info(self) -> WifiNetworkInfo:
        interfaces = netifaces.interfaces()

        # Filter for interfaces starting with 'wl'
        wifi_interfaces = [i for i in interfaces if i.startswith("wl")]

        for interface in wifi_interfaces:
            addr_info = netifaces.ifaddresses(interface)

            ip_address = ""
            mac_address = ""

            # Check if the interface has an IPv4 address assigned (AF_INET)
            if netifaces.AF_INET in addr_info:
                # Return the first IP found on the first 'wl' interface
                ip_address = addr_info[netifaces.AF_INET][0]["addr"]

            if netifaces.AF_LINK in addr_info and addr_info[netifaces.AF_LINK]:
                mac_address = addr_info[netifaces.AF_LINK][0].get("addr", "")

            if ip_address and mac_address:
                return WifiNetworkInfo(ip_address=ip_address, mac_address=mac_address)

        return WifiNetworkInfo(ip_address="", mac_address="")

    def get_wifi_status(self) -> WifiStatus:
        return self._wifi_status

    def update_wifi_status(self):
        try:
            proc = subprocess.Popen(
                "nmcli -f IN-USE,SIGNAL,SSID device wifi list --rescan no | grep '*'",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output, _ = proc.communicate(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            raise TimeoutError("Failed to update WiFi status within 10 seconds")

        output = output.decode("utf-8").strip()
        if not output:
            return

        parts = output.split()
        signal = int(parts[1])
        ssid = " ".join(parts[2:])

        with self._status_lock:
            self._wifi_status.SSID = ssid
            self._wifi_status.RSSI = math.ceil((signal / 2) - 100)

    def scan_wifi_networks(self) -> List[AvailableNetwork]:
        available_networks: List[AvailableNetwork] = []
        # set is used to avoid duplicate SSID entries
        seen_networks = set()

        # Step 1: Trigger a rescan (requires sudo/root)
        # Requires sudo privilege. Add the following rule to sudoers:
        # Run: sudo visudo -f /etc/sudoers.d/nmcli
        # Add: <username> ALL=(ALL) NOPASSWD: /usr/bin/nmcli device wifi rescan *
        try:
            proc = subprocess.Popen(
                "sudo nmcli device wifi rescan",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            proc.communicate(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            self._logger.warn(
                "[WifiManager][scan_wifi_networks] Rescan timed out, proceeding with cached results"
            )

        try:
            proc = subprocess.Popen(
                "nmcli -f BSSID,SIGNAL,SSID device wifi list",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output, _ = proc.communicate(timeout=30.0)
        except subprocess.TimeoutExpired:
            self._logger.error(
                "[WifiManager][scan_wifi_networks] Scan WiFi networks timeout expired"
            )
            proc.kill()
            return available_networks

        output = output.decode("utf-8").strip()
        lines = output.splitlines()[1:]

        for line in lines:
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            bssid = parts[0]
            signal = int(parts[1])
            ssid = " ".join(parts[2:])
            if not ssid or ssid in seen_networks:
                continue

            seen_networks.add(ssid)
            available_networks.append(
                AvailableNetwork(
                    BSSID=bssid, SSID=ssid, RSSI=math.ceil((signal / 2) - 100)
                )
            )

        return available_networks

    def connect_wifi(self, ssid: str, password: str):
        # Add: <username> ALL=(ALL) NOPASSWD: /usr/bin/nmcli device wifi connect *
        try:
            proc = subprocess.Popen(
                f"sudo nmcli device wifi connect '{ssid}' password '{password}'",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output, _ = proc.communicate(timeout=30.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            raise TimeoutError(
                f"Failed to connect to {ssid} network within timeout period"
            )

        output = output.decode("utf-8").strip()
        if "Error" in output:
            raise RuntimeError(f"Failed to connect to {ssid} network: {output}")

        self._logger.info(
            f"[WifiManager][connect_wifi] Successfully connected to WiFi network {ssid}"
        )


def init_wifi_manager(node: Node) -> WifiManager:
    node.get_logger().info(
        "[WifiManager][init_wifi_manager] Initializing Wifi Management Module"
    )
    wifi_manager = WifiManager(node=node)
    wifi_manager.setup_wifi()
    try:
        wifi_manager.update_wifi_status()
    except Exception as err:
        node.get_logger().error(f"{err}")

    return wifi_manager
