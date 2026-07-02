import math
import uuid
import threading
import structlog
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, Any
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped


class MoveState(str, Enum):
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"


@dataclass
class MoveGoal:
    goal_id: str
    goal_handle: ClientGoalHandle
    state: MoveState = MoveState.EXECUTING
    feedback: Optional[NavigateToPose.Feedback] = None
    result: Optional[NavigateToPose.Result] = None


def _wait_for_future(future, timeout: Optional[float] = None) -> bool:
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout)


class RobotGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):

        self._logger = logger

        self._node = node

        self._action_clients: Dict[str, ActionClient] = {}
        self.register_action_clients()

        self._lock = threading.Lock()
        self._goals: Dict[str, MoveGoal] = {}

    def register_action_clients(self):

        move_client = ActionClient(
            node=self._node,
            action_type=NavigateToPose,
            action_name="/robot01/navigate_to_pose",
        )

        self._action_clients.update({"move": move_client})

    def move(self, x: float, y: float, yaw: float) -> Tuple[bool, str, Optional[str]]:
        move_client = self._action_clients.get("move")
        if not move_client.wait_for_server(timeout_sec=30.0):
            return False, "Action server is not available", None

        self._logger.info("[RobotGateway] Sending navigation goal ", x=x, y=y, yaw=yaw)

        goal_msg = NavigateToPose.Goal(
            pose=PoseStamped(
                header=Header(
                    frame_id="map", stamp=self._node.get_clock().now().to_msg()
                ),
                pose=Pose(
                    position=Point(x=x, y=y, z=0.0),
                    orientation=Quaternion(
                        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                    ),
                ),
            )
        )

        future = move_client.send_goal_async(
            goal=goal_msg, feedback_callback=self._move_feedback_cb
        )
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for goal acceptance", None

        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            return False, "Goal rejected by task runner node", None

        goal_id = goal_handle.goal_id
        goal_id = str(uuid.UUID(bytes=bytes(goal_id.uuid)))
        with self._lock:
            self._goals[goal_id] = MoveGoal(goal_id=goal_id, goal_handle=goal_handle)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, gid=goal_id: self._move_result_cb(gid, fut)
        )

        return True, "", goal_id

    def _move_feedback_cb(self, goal_handle):

        goal_id = goal_handle.goal_id
        goal_id = str(uuid.UUID(bytes=bytes(goal_id.uuid)))

        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            goal.feedback = goal_handle.feedback

    def _move_result_cb(self, goal_id: str, future):
        result = future.result()
        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                goal.state = MoveState.SUCCEEDED
            elif result.status == GoalStatus.STATUS_CANCELED:
                goal.state = MoveState.CANCELED
            elif result.status == GoalStatus.STATUS_ABORTED:
                goal.state = MoveState.ABORTED

            goal.result = result.result

    def get_move_status(self, goal_id: str) -> Optional[Dict[str, Any]]:

        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            return {
                "goal_id": goal.goal_id,
                "state": goal.state.value,
            }

    def cancel_move(self, goal_id: str) -> Tuple[bool, str]:
        with self._lock:
            goal = self._goals.get(goal_id)
            if goal is None:
                return False, "Unknown goal id"
            if goal.state != MoveState.EXECUTING:
                return False, f"Goal already finished: {goal.state.value}"
            goal_handle = goal.goal_handle

        self._logger.info("[RobotGateway] Cancelling goal", goal_id=goal_id)

        cancel_future = goal_handle.cancel_goal_async()
        if not _wait_for_future(cancel_future, timeout=10.0):
            return False, "Timeout waiting for cancel response"

        response = cancel_future.result()
        if len(response.goals_canceling) == 0:
            return False, "Cancel rejected by server"

        return True, ""


def init_robot_gateway(
    logger: structlog.stdlib.BoundLogger, node: Node
) -> RobotGateway:
    robot_gw = RobotGateway(logger=logger, node=node)
    return robot_gw
