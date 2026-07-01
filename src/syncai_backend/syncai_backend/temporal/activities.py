import math
import time
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.exceptions import ApplicationError


from syncai_backend.gateways.workflow.schema import StepParams
from syncai_backend.gateways.robot.robot import RobotGateway


class ActivityResult(BaseModel):
    success: bool
    goal_id: str | None = None
    state: str | None = None


class RobotActivities:
    def __init__(self, logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway):
        self._logger = logger
        self._robot_gw = robot_gw

    @activity.defn
    def execute_move(self, params: StepParams) -> ActivityResult:
        yaw = math.radians(params.theta)
        accepted, msg, goal_id = self._robot_gw.move(x=params.x, y=params.y, yaw=yaw)
        if not accepted:
            raise ApplicationError(f"Move rejected: {msg}", non_retryable=False)

        self._logger.info("[RobotActivity] Move accepted", goal_id=goal_id)

        # polling move state
        while True:
            status = self._robot_gw.get_move_status(goal_id=goal_id)
            state = status["state"] if status else None

            # send heartbeat, tell temporal server worker still alive
            activity.heartbeat(state)

            if activity.is_cancelled():
                self._logger.warn("[RobotActivity] Move activity has been cancelled")
                self._robot_gw.cancel_move(goal_id=goal_id)
                raise activity.CompleteAsyncError

            if state in ["succeeded", "aborted", "canceled"]:
                break

            time.sleep(1.0)

        if state != "succeeded":
            raise ApplicationError(f"move ended in {state}", non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)