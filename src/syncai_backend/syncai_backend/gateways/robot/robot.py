import math
import uuid
import threading
import structlog
from enum import Enum
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple
from rclpy import qos
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from syncai_common.msg import WifiNetwork
from syncai_common.srv import (
    ConnectWifiNetwork,
    ScanWifiNetworks,
    SetMotionKey,
    SetPolicyMode,
    SwitchMode,
)

from std_msgs.msg import Header
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)


class MotionKey(str, Enum):
    """Keys accepted by the driver manager's `set_motion_key` service.

    The driver maps each one to a gait-controller MODE character (see
    DriverManagerNode::setMotionKeyCallback) -- except ESTOP, which bypasses
    the MODE keymap and goes out as its own datagram. Kept as strings because
    the service field is a string that is forwarded verbatim.
    """

    STAND = "0"  # MODE Z
    LOCOMOTION = "1"  # MODE C
    LIE_DOWN = "2"  # MODE X
    DAMPING = "3"  # MODE R
    ESTOP = "4"  # ESTOP datagram, not a MODE character
    MPC = "5"  # MODE M


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


# How many finished MOVE goals _goals keeps around. The dict used to grow
# without bound — every goal carried its ClientGoalHandle and result forever,
# so a robot on a patrol schedule leaked a few dozen entries a day. Five is
# enough for every real reader: the MOVE activity polls only its own (newest)
# goal and returns on the terminal state, so by the time a goal is old enough
# to evict, nothing will ask about it again. Eviction happens on insert, and
# never touches an EXECUTING goal — get_move_status / cancel_move on a live
# goal must keep working no matter how the book fills up.
MAX_TRACKED_GOALS = 5

# How long switch_mode() waits for sys_manager's answer before reporting the
# switch as merely dispatched. This is NOT sized to how long a switch takes
# (tens of seconds of byobu commands and sleep offsets) and must not be: on a
# real switch sys_manager kills the byobu session THIS PROCESS runs in before
# it would ever respond, so no timeout is long enough to see that answer. The
# window only exists to catch the responses that do come back fast — the no-op
# ("Already in AUTO; nothing to do") and a refusal — where reporting the real
# outcome beats a blind "dispatched". Module-level so tests can shrink it.
SWITCH_MODE_ACK_TIMEOUT = 2.0


def _wait_for_future(future, timeout: Optional[float] = None) -> bool:
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout)


def _clamp_normalized(value: float) -> float:
    """[-1, 1] with NaN/inf collapsed to 0 — the last gate before the wire.

    The frontend clamps too, but this is the boundary a malformed or hostile
    client actually reaches, so the guarantee lives here.
    """
    if not math.isfinite(value):
        return 0.0
    return max(-1.0, min(1.0, value))


class RobotGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):

        self._logger = logger

        self._node = node

        self._action_clients: dict[str, ActionClient] = {}
        self.register_action_clients()

        self._service_clients: dict[str, Client] = {}
        self.register_service_clients()

        self._publishers: dict[str, Publisher] = {}
        self.register_publishers()

        self._lock = threading.Lock()
        self._goals: dict[str, MoveGoal] = {}

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

        set_motion_key_client = self._node.create_client(
            srv_type=SetMotionKey,
            srv_name="set_motion_key",
        )

        # Relative, like every other client here, so it resolves under this
        # node's robot_id namespace and reaches this robot's driver_manager only.
        # An absolute, fleet-wide name has been a bug in this backend before.
        set_policy_mode_client = self._node.create_client(
            srv_type=SetPolicyMode,
            srv_name="set_policy_mode",
        )

        # sys_manager's operating-mode switch (which byobu session is up). On
        # this gateway rather than a new one because sys_manager is already part
        # of its surface (the wifi services above), and because switching modes
        # is very much "a handle that can command the robot" — the reason
        # MapGateway exists is to keep such handles away from the map router.
        switch_mode_client = self._node.create_client(
            srv_type=SwitchMode,
            srv_name="switch_mode",
        )

        self._service_clients.update(
            {
                "scan_wifi": scan_wifi_client,
                "connect_wifi": connect_wifi_client,
                "set_motion_key": set_motion_key_client,
                "set_policy_mode": set_policy_mode_client,
                "switch_mode": switch_mode_client,
            }
        )

    def register_publishers(self):
        initial_pose_pub = self._node.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic="initialpose",
            qos_profile=qos.QoSProfile(depth=5),
        )

        cmd_vel_pub = self._node.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=qos.QoSProfile(depth=5),
        )

        self._publishers.update(
            {
                "initial_pose": initial_pose_pub,
                "cmd_vel": cmd_vel_pub
            }
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

    def set_motion_key(self, key: MotionKey) -> Tuple[bool, str]:
        motion_key_client = self._service_clients.get("set_motion_key")
        if not motion_key_client.wait_for_service(timeout_sec=5.0):
            return False, "set_motion_key service is not available"

        self._logger.info("[RobotGateway] Setting motion key", key=key.value)

        # .value, not the enum member: the srv field is a plain string and
        # rosidl stores whatever it is handed.
        future = motion_key_client.call_async(SetMotionKey.Request(key=key.value))
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for set_motion_key response"

        response = future.result()
        return response.success, response.message

    def set_policy_mode(self, mode: int) -> Tuple[bool, str]:
        """Switch the gait controller's policy (`MODE <uint>` over UDP).

        Takes a plain int rather than an enum: `SetPolicyMode.mode` is a bare
        uint8 whose legal set is the gait controller's policy index -- a
        namespace this workspace does not own (see syncai_common/README.md). The
        REST layer narrows it to the indices we are willing to expose, and
        keeping this signature an int means a caller that needs an unexposed one
        does not have to widen an enum that lives in the REST layer.

        Same fire-and-forget caveat as set_motion_key, and worse: the driver's
        callback formats one string and hands it to sendto(), whose return it
        ignores, and unlike set_motion_key it validates nothing and is not gated
        by the safety lock. A True here means the datagram was written -- not
        that the controller switched policy.
        """
        policy_mode_client = self._service_clients.get("set_policy_mode")
        if not policy_mode_client.wait_for_service(timeout_sec=5.0):
            return False, "set_policy_mode service is not available"

        self._logger.info("[RobotGateway] Setting policy mode", mode=mode)

        future = policy_mode_client.call_async(SetPolicyMode.Request(mode=mode))
        # The same 10.0 as set_motion_key, deliberately rather than a third
        # number: the driver's callback is one snprintf plus one sendto on a
        # datagram socket and cannot block, so the only latency is the DDS round
        # trip plus its turn on the driver's shared services callback group. The
        # 45.0 / 70.0 above are large because those service *implementations*
        # sleep (a rescan, nmcli); nothing here does. Not shrunk to ~1 s either:
        # a discovery hiccup, or an executor thread briefly starved by telemetry
        # parsing, would then surface as a spurious 502.
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for set_policy_mode response"

        response = future.result()
        return response.success, response.message

    def switch_mode(self, mode: int) -> Tuple[Optional[bool], str]:
        """Ask sys_manager to switch the operating mode. Three-valued on purpose.

        Returns ``(True, msg)`` / ``(False, msg)`` when sys_manager answered
        inside ``SWITCH_MODE_ACK_TIMEOUT``, and ``(None, msg)`` when it did not
        — and on a real switch, ``None`` is the EXPECTED outcome, not a fault.
        ``switch_mode`` kills every known byobu session before building the
        target one, and this backend is a pane of whichever session is live, so
        the service response to a genuine switch is addressed to a process that
        is already dying. The caller cannot usually deliver its own HTTP
        response either; the console treats the dropped connection as the
        signal and polls robot_state until the new session's backend answers.

        The quick answers are worth the short wait, though: "Already in X;
        nothing to do" (sys_manager refuses to rebuild the live mode — in
        MANUAL a rebuild would drop an unsaved map, since pgo keeps its
        keyframes in RAM) and refusals both come back in well under a second,
        and reporting them beats a blind "dispatched".

        Takes the RobotMode uint8 (MANUAL=1 / AUTO=2), not a REST enum — same
        int-at-the-boundary contract as set_policy_mode.
        """
        switch_mode_client = self._service_clients.get("switch_mode")
        if not switch_mode_client.wait_for_service(timeout_sec=5.0):
            return False, "switch_mode service is not available"

        self._logger.info("[RobotGateway] Switching operating mode", mode=mode)

        future = switch_mode_client.call_async(SwitchMode.Request(mode=mode))
        if not _wait_for_future(future, timeout=SWITCH_MODE_ACK_TIMEOUT):
            return None, "Mode switch dispatched; the stack is rebuilding"

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

    def set_initial_pose(self, x: float, y: float, yaw: float) -> Tuple[bool, str]:
        """Seed localization with an operator-supplied map-frame pose.

        Fire-and-forget: `initialpose` is a plain topic, so there is no ack and
        no way to learn here whether the guess converged — the localizer applies
        it as an ICP initial guess on its next cycle. Only the consumers that
        are already discovered will see this sample (the topic is volatile, not
        latched), which is why a missing subscriber is worth a log line: an
        initialpose that nobody receives is indistinguishable from one that was
        received and ignored, and that has cost debugging time before.
        """
        initial_pose_pub = self._publishers.get("initial_pose")

        if initial_pose_pub.get_subscription_count() == 0:
            self._logger.warning(
                "[RobotGateway] Publishing initialpose with no subscriber; "
                "is the localizer running?"
            )

        self._logger.info("[RobotGateway] Publishing initial pose", x=x, y=y, yaw=yaw)

        pose_stamped = self._make_pose_stamped(x=x, y=y, yaw=yaw)
        # Covariance is left at zero: the localizer reads x/y/yaw only (it
        # refills roll/pitch/z from the current estimate, see applyPlanarGuess)
        # and a REST caller has no meaningful uncertainty to report.
        msg = PoseWithCovarianceStamped(header=pose_stamped.header)
        msg.pose.pose = pose_stamped.pose
        initial_pose_pub.publish(msg)

        return True, ""

    def teleop_cmd_vel(self, vx: float, vy: float, wz: float) -> Tuple[bool, str]:
        """Publish one teleop velocity command, clamped to [-1, 1] per axis.

        The clamped value goes on the wire as-is (m/s and rad/s): there used
        to be scale-to-ceiling constants here (0.5 m/s / 0.65 rad/s) but the
        artificial cap was dropped on request — full stick now means
        1.0 m/s / 1.0 rad/s. The [-1, 1] clamp is kept as the one hard bound:
        a broken or hostile client can command fast, but never 10 m/s. If a
        ceiling below 1.0 is ever wanted again, this is where it goes.

        Refused outright while a MOVE goal is EXECUTING: the controller owns
        cmd_vel during FollowPath (20 Hz), there is no mux in the stack, and
        the driver is last-message-wins per datagram — interleaving teleop
        would make the robot judder between two command sources. The operator
        cancels the task first, then drives.

        No per-call info log and no subscriber-count warning (the
        set_initial_pose template's two lines): this runs at ~10 Hz.
        """
        if self._autonomous_move_active():
            return False, "autonomous move in progress"

        twist = Twist()
        twist.linear.x = _clamp_normalized(vx)
        twist.linear.y = _clamp_normalized(vy)
        twist.angular.z = _clamp_normalized(wz)

        self._publishers.get("cmd_vel").publish(twist)

        return True, ""

    def teleop_stop(self) -> None:
        """Publish a zero-velocity Twist — the teleop watchdog/disconnect stop.

        This exists because the driver manager has NO cmd_vel watchdog: when a
        publisher goes quiet it just stops sending AXES, it never sends a stop.
        So the backend owns stop-on-disconnect and stop-on-stale-input, and
        this is the primitive both use.

        Gated like teleop_cmd_vel: while an autonomous MOVE is executing the
        teleop path contributed no motion, and injecting zeros would fight the
        controller on its own topic.
        """
        if self._autonomous_move_active():
            return

        self._publishers.get("cmd_vel").publish(Twist())

    def _autonomous_move_active(self) -> bool:
        with self._lock:
            return any(
                goal.state is MoveState.EXECUTING for goal in self._goals.values()
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
            self._evict_finished_goals()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, gid=goal_id: self._move_result_cb(gid, fut)
        )

        return True, "", goal_id

    def _evict_finished_goals(self) -> None:
        """Trim _goals to MAX_TRACKED_GOALS, oldest finished first.

        Caller holds self._lock. Relies on dict insertion order for "oldest";
        EXECUTING goals are skipped, so with more live goals than the cap the
        dict simply stays over it until they finish — correctness over the
        bound, and in practice the task-level mutex keeps live goals at one.
        """
        excess = len(self._goals) - MAX_TRACKED_GOALS
        if excess <= 0:
            return

        for goal_id in list(self._goals):
            if excess <= 0:
                break
            if self._goals[goal_id].state is MoveState.EXECUTING:
                continue
            del self._goals[goal_id]
            excess -= 1

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

    def get_move_status(self, goal_id: str) -> Optional[dict[str, Any]]:

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
