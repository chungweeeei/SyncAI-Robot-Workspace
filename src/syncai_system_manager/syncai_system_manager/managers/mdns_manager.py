import ipaddress
import subprocess
from typing import Optional

import netifaces
from rclpy.node import Node

from syncai_system_manager.managers.sys_manager import SysManager
from syncai_system_manager.managers.wifi_manager import WifiNetworkInfo, WifiManager

# Fallback interface search order when WiFi has no address: wired NICs on the
# host (en*), then container NICs (eth*).
PREFERRED_INTERFACE_PREFIXES = ("en", "eth")

# Docker's default bridge address pool; such an address is unreachable from
# the LAN, so it must never be published (in the robot container eth0 is the
# compose bridge and eth1 is the syncai-lan macvlan).
DOCKER_BRIDGE_NETWORK = ipaddress.ip_network("172.16.0.0/12")


class MdnsManager:
    def __init__(
        self,
        node: Node,
        wifi_manager: WifiManager,
        sys_manager: SysManager,
    ):
        self._logger = node.get_logger()
        self._wifi_manager = wifi_manager
        self._sys_manager = sys_manager
        self._publish_proc: Optional[subprocess.Popen] = None

    def setup_mdns(self):
        self.kill_mdns()
        self.publish_mdns()

    def _resolve_publish_ip(self) -> str:
        wifi_info: WifiNetworkInfo = self._wifi_manager.get_wifi_info()
        if wifi_info.ip_address:
            return wifi_info.ip_address

        self._logger.warning(
            "[MdnsManager][_resolve_publish_ip] No WiFi IP address, falling back to preferred interfaces"
        )
        for prefix in PREFERRED_INTERFACE_PREFIXES:
            for interface in netifaces.interfaces():
                if not interface.startswith(prefix):
                    continue
                inet_entries = netifaces.ifaddresses(interface).get(
                    netifaces.AF_INET, []
                )
                ip = inet_entries[0].get("addr", "") if inet_entries else ""
                if not ip or ipaddress.ip_address(ip) in DOCKER_BRIDGE_NETWORK:
                    continue
                self._logger.info(
                    f"[MdnsManager][_resolve_publish_ip] Using {ip} from interface {interface}"
                )
                return ip

        return ""

    def publish_mdns(self):

        ip_address = self._resolve_publish_ip()
        if not ip_address:
            self._logger.warning(
                "[MdnsManager][publish_mdns] No usable IP address found, skip publishing mDNS domain name."
            )
            return

        mdns_domain = f"{self._sys_manager.get_robot_id()}.local"
        try:
            proc = subprocess.Popen(
                [
                    "avahi-publish",
                    "-a",
                    mdns_domain,
                    "-R",
                    ip_address,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self._logger.error(
                "[MdnsManager][publish_mdns] avahi-publish not found; is avahi-utils installed?"
            )
            return
        except Exception as err:
            self._logger.error(
                f"[MdnsManager][publish_mdns] Failed to spawn avahi-publish: {str(err)}",
            )
            return

        # avahi-publish is a long-running daemon; if it exits within the liveness
        # window it means publishing failed (e.g. name collision, no daemon).
        try:
            proc.wait(timeout=0.5)
            self._logger.error(
                "[MdnsManager][publish_mdns] avahi-publish exited prematurely",
                returncode=proc.returncode,
            )
            return
        except subprocess.TimeoutExpired:
            pass

        self._publish_proc = proc
        self._logger.info(
            f"[MdnsManager][publish_mdns] Publishing mDNS domain {mdns_domain} at {ip_address}"
        )

    def kill_mdns(self):
        proc = self._publish_proc
        self._publish_proc = None

        if proc is None or proc.poll() is not None:
            return

        try:
            proc.terminate()
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._logger.warning(
                "[MdnsManager][kill_mdns] avahi-publish did not terminate gracefully, sending SIGKILL",
            )
            try:
                proc.kill()
                proc.wait(timeout=1.0)
            except Exception as err:
                self._logger.error(
                    f"[MdnsManager][kill_mdns] Failed to SIGKILL avahi-publish: {str(err)}",
                )
                return

        self._logger.info("[MdnsManager][kill_mdns] Stopped publishing mDNS domain")


def init_mdns_manager(
    node: Node,
    wifi_manager: WifiManager,
    sys_manager: SysManager,
) -> MdnsManager:

    mdns_manager = MdnsManager(
        node=node,
        wifi_manager=wifi_manager,
        sys_manager=sys_manager,
    )

    try:
        mdns_manager.setup_mdns()
    except Exception as err:
        node.get_logger().error(
            f"[MdnsManager] Failed to set up mDNS publishing: {str(err)}"
        )

    return mdns_manager
