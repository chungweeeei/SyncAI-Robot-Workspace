import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from syncai_common.msg import RobotState as RobotStateMsg

from syncai_backend.repositories.robot.robot import RobotRepo


class RobotStateSubscriber:
    """Feeds RobotRepo, and therefore GET /api/v1/robot/state, from the
    ``robot_state`` topic.

    ``RobotState`` carries more than that REST payload exposes — ``motor_status``
    reaches it flattened to its ``states`` array and trimmed to
    name/temperature/error, while that field's own ``timestamp`` and
    ``localization_valid`` do not reach it at all. The router's field list is a
    whitelist and must stay one; this subscriber hands the whole message through
    unfiltered.

    Note the consequence of the drop below for ``low_level_mode``, which IS
    exposed: the gait controller's state is unreadable over REST until the
    localizer has been relocalized, because the whole sample is discarded until
    then. That is precisely the window in which an operator wants to know whether
    the robot is standing.
    """

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
        # syncai_robot_state publishes on every tick now, including the window
        # before the localizer has been relocalized, where localization_status
        # is zeroed rather than a real pose. Dropping those keeps
        # GET /api/v1/robot/state answering 404 ("no state yet") instead of
        # 200 with the robot apparently parked on the map origin — the frontend
        # gates its whole dashboard on that 404.
        if not msg.localization_valid:
            return
        self._robot_repo.update_robot_state(state=msg)


def init_robot_state_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, robot_repo: RobotRepo
) -> RobotStateSubscriber:
    robot_state_subscriber = RobotStateSubscriber(logger=logger, robot_repo=robot_repo)
    robot_state_subscriber.register(node=node)
