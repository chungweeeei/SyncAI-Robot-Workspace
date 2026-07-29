import threading
import structlog

from typing import Optional

from syncai_common.msg import RobotState as RobotStateMsg


class RobotRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        # In-Process memory cache for robot state. Only samples with
        # localization_valid ever land here — see RobotStateSubscriber.
        self._robot_state_lock = threading.Lock()
        self._robot_state: Optional[RobotStateMsg] = None

    def update_robot_state(self, state: RobotStateMsg):
        with self._robot_state_lock:
            self._robot_state = state

    def get_robot_state(self) -> Optional[RobotStateMsg]:
        with self._robot_state_lock:
            return self._robot_state


def init_robot_repo(logger: structlog.stdlib.BoundLogger) -> RobotRepo:
    robot_repo = RobotRepo(logger)
    return robot_repo
