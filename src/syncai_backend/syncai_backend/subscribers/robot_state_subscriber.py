import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from syncai_common.msg import RobotState as RobotStateMsg

from syncai_backend.repositories.robot.robot import RobotRepo


class RobotStateSubscriber:
    def __init__(self, logger: structlog.stdlib.BoundLogger, robot_repo: RobotRepo):
        self._logger = logger
        self._robot_repo = robot_repo

    def register(self, node: Node):

        self._robot_state_sub = node.create_subscription(
            msg_type=RobotStateMsg,
            topic="robot_state",
            callback=self._robot_state_cb,
            qos_profile=QoSProfile(
                depth=3,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _robot_state_cb(self, msg: RobotStateMsg):
        self._robot_repo.update_robot_state(state=msg)


def init_robot_state_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, robot_repo: RobotRepo
) -> RobotStateSubscriber:
    robot_state_subscriber = RobotStateSubscriber(logger=logger, robot_repo=robot_repo)
    robot_state_subscriber.register(node=node)
