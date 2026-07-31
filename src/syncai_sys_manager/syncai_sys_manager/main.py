import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from syncai_sys_manager.managers.mdns_manager import init_mdns_manager
from syncai_sys_manager.managers.conf_manager import init_conf_manager
from syncai_sys_manager.managers.monitor_manager import init_monitor_manager
from syncai_sys_manager.managers.node_manager import init_node_manager
from syncai_sys_manager.managers.wifi_manager import init_wifi_manager


class SystemManager(Node):
    def __init__(self):
        super().__init__("syncai_sys_manager")

        self.wifi_manager = init_wifi_manager(node=self)
        self.conf_manager = init_conf_manager(node=self)
        self.mdns_manager = init_mdns_manager(
            node=self,
            wifi_manager=self.wifi_manager,
            conf_manager=self.conf_manager,
        )
        self.monitor_manager = init_monitor_manager(node=self)
        # Last, so the rest of this node is fully constructed before the stack it
        # brings up starts talking to it. init_node_manager brings the byobu
        # session up unless it is already running — see NodeManager.setup_session
        # for why that guard is load-bearing. conf_manager supplies the robot_id
        # that scopes its log tree.
        self.node_manager = init_node_manager(
            node=self,
            conf_manager=self.conf_manager,
        )


def main():
    rclpy.init(args=None)

    node = None
    try:
        node = SystemManager()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as err:
        rclpy.logging.get_logger("syncai_sys_manager").error(f"{str(err)}")
    finally:
        # The avahi-publish child must not outlive the node, or a stale
        # <robot_id>.local record keeps resolving to an old IP.
        mdns_manager = getattr(node, "mdns_manager", None)
        if mdns_manager is not None:
            mdns_manager.kill_mdns()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
