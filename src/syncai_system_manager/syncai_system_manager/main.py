import sys
import rclpy
from rclpy.node import Node


class SystemManager(Node):
    def __init__(self):
        super().__init__("syncai_system_manager")


def main():
    rclpy.init(args=None)

    try:
        node = SystemManager()
        rclpy.spin(node)
    except Exception as err:
        rclpy.get_logger().error(f"{str(err)}")
    finally:
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
