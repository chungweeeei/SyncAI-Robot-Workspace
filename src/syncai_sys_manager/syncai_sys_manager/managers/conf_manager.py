from rclpy.node import Node


class ConfManager:
    def __init__(self, node: Node):
        self._logger = node.get_logger()

        node.declare_parameter("robot_id", "default_robot")
        self._robot_id = (
            node.get_parameter("robot_id").get_parameter_value().string_value
        )

    def get_robot_id(self) -> str:
        return self._robot_id


def init_conf_manager(node: Node):
    conf_manager = ConfManager(node=node)
    return conf_manager
