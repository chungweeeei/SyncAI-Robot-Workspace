import math
import uuid
import threading
import structlog
from enum import Enum
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from syncai_common.msg import WifiNetwork
from syncai_common.srv import ConnectWifiNetwork, ScanWifiNetworks

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped


class MoveState(str, Enum):
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"


# Tracks a NavigateToPose goal through its state machine, status polling and
# cancel path.
@dataclass
class MoveGoal:
    goal_id: str
    goal_handle: ClientGoalHandle
    state: MoveState = MoveState.EXECUTING
    feedback: Optional[Any] = None
    result: Optional[Any] = None


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

        self._service_clients: Dict[str, Any] = {}
        self.register_service_clients()

        self._lock = threading.Lock()
        self._goals: Dict[str, MoveGoal] = {}

    def register_action_clients(self):

        move_client = ActionClient(
            node=self._node,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
        )

        self._action_clients.update({"move": move_client})

    def register_service_clients(self):

        scan_wifi_client = self._node.create_client(
            srv_type=ScanWifiNetworks,
            srv_name="scan_wifi",
        )

        connect_wifi_client = self._node.create_client(
            srv_type=ConnectWifiNetwork,
            srv_name="connect_wifi",
        )

        self._service_clients.update(
            {"scan_wifi": scan_wifi_client, "connect_wifi": connect_wifi_client}
        )

    def scan_wifi_networks(self) -> Tuple[bool, str, List[WifiNetwork]]:
        scan_client = self._service_clients.get("scan_wifi")
        if not scan_client.wait_for_service(timeout_sec=5.0):
            return False, "scan_wifi service is not available", []

        self._logger.info("[RobotGateway] Scanning WiFi networks")

        future = scan_client.call_async(ScanWifiNetworks.Request())
        # The service rescans (10s) then lists (30s); leave headroom on top.
        if not _wait_for_future(future, timeout=45.0):
            return False, "Timeout waiting for scan_wifi response", []

        response = future.result()
        if not response.success:
            return False, response.message, []

        return True, "", list(response.networks)

    def connect_wifi(self, ssid: str, password: str) -> Tuple[bool, str]:
        connect_client = self._service_clients.get("connect_wifi")
        if not connect_client.wait_for_service(timeout_sec=5.0):
            return False, "connect_wifi service is not available"

        self._logger.info("[RobotGateway] Connecting to WiFi network", ssid=ssid)

        future = connect_client.call_async(
            ConnectWifiNetwork.Request(ssid=ssid, password=password)
        )
        # The service itself waits up to 60s for nmcli; leave headroom on top.
        if not _wait_for_future(future, timeout=70.0):
            return False, "Timeout waiting for connect_wifi response"

        response = future.result()
        return response.success, response.message

    def _make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        return PoseStamped(
            header=Header(frame_id="map", stamp=self._node.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            ),
        )

    def _send_nav_goal(
        self, client_name: str, goal_msg
    ) -> Tuple[bool, str, Optional[str]]:
        client = self._action_clients.get(client_name)
        if not client.wait_for_server(timeout_sec=30.0):
            return False, "Action server is not available", None

        future = client.send_goal_async(
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

    def move(self, x: float, y: float, yaw: float) -> Tuple[bool, str, Optional[str]]:
        self._logger.info("[RobotGateway] Sending navigation goal ", x=x, y=y, yaw=yaw)

        goal_msg = NavigateToPose.Goal(pose=self._make_pose_stamped(x=x, y=y, yaw=yaw))

        return self._send_nav_goal("move", goal_msg)

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
