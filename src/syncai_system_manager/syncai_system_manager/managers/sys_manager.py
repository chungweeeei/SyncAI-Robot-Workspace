from rclpy.node import Node


class SysManager:
    def __init__(self, node: Node):
        self._logger = node.get_logger()

        node.declare_parameter("robot_id", "default_robot")
        self._robot_id = (
            node.get_parameter("robot_id").get_parameter_value().string_value
        )

    def get_robot_id(self) -> str:
        return self._robot_id


def init_sys_manager(node: Node):
    sys_manager = SysManager(node=node)
    return sys_manager
