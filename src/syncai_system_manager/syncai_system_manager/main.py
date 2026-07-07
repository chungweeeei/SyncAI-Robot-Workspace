import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from syncai_system_manager.managers.mdns_manager import init_mdns_manager
from syncai_system_manager.managers.sys_manager import init_sys_manager
from syncai_system_manager.managers.wifi_manager import init_wifi_manager


class SystemManager(Node):
    def __init__(self):
        super().__init__("syncai_system_manager")

        self.wifi_manager = init_wifi_manager(node=self)
        self.sys_manager = init_sys_manager(node=self)
        self.mdns_manager = init_mdns_manager(
            node=self,
            wifi_manager=self.wifi_manager,
            sys_manager=self.sys_manager,
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
        rclpy.logging.get_logger("syncai_system_manager").error(f"{str(err)}")
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
