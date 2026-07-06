import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from syncai_system_manager.wifi_manager import init_wifi_manager


class SystemManager(Node):
    def __init__(self):
        super().__init__("syncai_system_manager")

        wifi_manager = init_wifi_manager(node=self)


def main():
    rclpy.init(args=None)

    try:
        node = SystemManager()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as err:
        rclpy.logging.get_logger("syncai_system_manager").error(f"{str(err)}")
    finally:
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
