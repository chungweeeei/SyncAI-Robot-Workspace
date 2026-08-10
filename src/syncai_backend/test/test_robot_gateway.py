"""Tests for RobotGateway's NavigateToPose goal tracking and command services.

The rclpy seams are mocked the way CoreManager's robot-gateway tests mock their
socket/gRPC seams: ``ActionClient`` is patched at the module import site and the
node is a MagicMock, so no ROS graph (or rclpy.init) is needed. Futures are
replaced by ``_Future`` below — the gateway's ``_wait_for_future`` only relies
on ``add_done_callback``/``result``, and a controllable future is what lets a
test hold a goal in EXECUTING, then deliver the result callback and watch the
state machine move.

This is the state machine the Temporal MOVE activity polls via
``get_move_status`` and unwinds via ``cancel_move`` on cancellation, so the
terminal-state bookkeeping here is business logic, not plumbing.
"""

import math
import uuid
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

from syncai_backend.gateways.robot.robot import (  # noqa: E402
    MotionKey,
    MoveState,
    RobotGateway,
)


GOAL_UUID = uuid.UUID("12345678-1234-5678-1234-567812345678")


class _Future:
    """A future the test completes by hand; done ones fire callbacks at once."""

    def __init__(self, result=None, completed=True):
        self._result = result
        self._completed = completed
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._completed:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self):
        return self._result

    def complete(self, result):
        self._result = result
        self._completed = True
        for callback in self._callbacks:
            callback(self)


def _goal_handle(accepted: bool = True) -> MagicMock:
    handle = MagicMock()
    handle.accepted = accepted
    handle.goal_id = SimpleNamespace(uuid=GOAL_UUID.bytes)
    return handle


@pytest.fixture
def move_client():
    client = MagicMock()
    client.wait_for_server.return_value = True
    return client


@pytest.fixture
def robot_gw(logger, move_client) -> RobotGateway:
    node = MagicMock()
    # _make_pose_stamped stamps headers from the node's clock, and the real
    # Header message asserts its stamp is a builtin_interfaces/Time — a bare
    # MagicMock does not pass, so the mocked clock must answer a real one.
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    with patch(
        "syncai_backend.gateways.robot.robot.ActionClient", return_value=move_client
    ):
        return RobotGateway(logger=logger, node=node)


def _send_goal(robot_gw, move_client, result_future=None):
    """Drive move() to acceptance; return (goal_id, the pending result future)."""
    handle = _goal_handle()
    if result_future is None:
        result_future = _Future(completed=False)
    handle.get_result_async.return_value = result_future
    move_client.send_goal_async.return_value = _Future(result=handle)

    accepted, message, goal_id = robot_gw.move(x=1.0, y=2.0, yaw=math.pi / 2)
    assert (accepted, message) == (True, "")
    return goal_id, result_future


class TestMoveGoalStateMachine:
    def test_accepted_goal_starts_executing(self, robot_gw, move_client):
        goal_id, _ = _send_goal(robot_gw, move_client)

        assert goal_id == str(GOAL_UUID)
        status = robot_gw.get_move_status(goal_id=goal_id)
        assert status == {"goal_id": goal_id, "state": MoveState.EXECUTING.value}

        # The goal itself: map-frame pose, yaw folded into the quaternion.
        goal_msg = move_client.send_goal_async.call_args[1]["goal"]
        assert goal_msg.pose.header.frame_id == "map"
        assert goal_msg.pose.pose.position.x == 1.0
        assert goal_msg.pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))

    @pytest.mark.parametrize(
        "ros_status,expected",
        [
            (GoalStatus.STATUS_SUCCEEDED, MoveState.SUCCEEDED),
            (GoalStatus.STATUS_ABORTED, MoveState.ABORTED),
            (GoalStatus.STATUS_CANCELED, MoveState.CANCELED),
        ],
    )
    def test_result_callback_lands_the_terminal_state(
        self, robot_gw, move_client, ros_status, expected
    ):
        goal_id, result_future = _send_goal(robot_gw, move_client)

        result_future.complete(SimpleNamespace(status=ros_status, result=None))

        status = robot_gw.get_move_status(goal_id=goal_id)
        assert status["state"] == expected.value

    def test_rejected_goal_is_not_tracked(self, robot_gw, move_client):
        move_client.send_goal_async.return_value = _Future(
            result=_goal_handle(accepted=False)
        )

        accepted, message, goal_id = robot_gw.move(x=0.0, y=0.0, yaw=0.0)

        assert accepted is False
        assert "rejected" in message
        assert goal_id is None

    def test_unavailable_server_fails_fast(self, robot_gw, move_client):
        move_client.wait_for_server.return_value = False

        accepted, message, goal_id = robot_gw.move(x=0.0, y=0.0, yaw=0.0)

        assert (accepted, goal_id) == (False, None)
        assert "not available" in message
        move_client.send_goal_async.assert_not_called()

    def test_unknown_goal_has_no_status(self, robot_gw):
        assert robot_gw.get_move_status(goal_id="not-a-goal") is None


class TestCancelMove:
    def test_cancel_unknown_goal(self, robot_gw):
        success, message = robot_gw.cancel_move(goal_id="not-a-goal")
        assert (success, message) == (False, "Unknown goal id")

    def test_cancel_a_finished_goal_is_refused(self, robot_gw, move_client):
        # The Temporal activity's cleanup path can race the result callback;
        # cancelling a goal that already landed must not go back to the server.
        goal_id, result_future = _send_goal(robot_gw, move_client)
        result_future.complete(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)
        )

        success, message = robot_gw.cancel_move(goal_id=goal_id)

        assert success is False
        assert "already finished" in message

    def test_cancel_an_executing_goal(self, robot_gw, move_client):
        goal_id, _ = _send_goal(robot_gw, move_client)
        handle = move_client.send_goal_async.return_value.result()
        handle.cancel_goal_async.return_value = _Future(
            result=SimpleNamespace(goals_canceling=[object()])
        )

        assert robot_gw.cancel_move(goal_id=goal_id) == (True, "")

    def test_cancel_rejected_by_server(self, robot_gw, move_client):
        goal_id, _ = _send_goal(robot_gw, move_client)
        handle = move_client.send_goal_async.return_value.result()
        handle.cancel_goal_async.return_value = _Future(
            result=SimpleNamespace(goals_canceling=[])
        )

        success, message = robot_gw.cancel_move(goal_id=goal_id)

        assert success is False
        assert "rejected" in message


class TestCommandServices:
    def _service(self, robot_gw, name, response, available=True):
        client = robot_gw._service_clients[name]
        client.wait_for_service.return_value = available
        client.call_async.return_value = _Future(result=response)
        return client

    def test_set_motion_key_sends_the_wire_string(self, robot_gw):
        client = self._service(
            robot_gw, "set_motion_key", SimpleNamespace(success=True, message="sent")
        )

        success, message = robot_gw.set_motion_key(key=MotionKey.LIE_DOWN)

        assert (success, message) == (True, "sent")
        # .value, not the enum member — the srv field is a plain string.
        assert client.call_async.call_args[0][0].key == "2"

    def test_set_motion_key_service_unavailable(self, robot_gw):
        self._service(robot_gw, "set_motion_key", None, available=False)

        success, message = robot_gw.set_motion_key(key=MotionKey.STAND)

        assert success is False
        assert "not available" in message

    def test_set_policy_mode_forwards_the_plain_int(self, robot_gw):
        client = self._service(
            robot_gw, "set_policy_mode", SimpleNamespace(success=True, message="")
        )

        success, _ = robot_gw.set_policy_mode(mode=1)

        assert success is True
        assert client.call_async.call_args[0][0].mode == 1

    def test_set_initial_pose_publishes_even_without_subscribers(self, robot_gw):
        # Fire-and-forget by design: no subscriber is a warning, not a failure
        # — the localizer may simply not have discovered the publisher yet.
        publisher = robot_gw._publishers["initial_pose"]
        publisher.get_subscription_count.return_value = 0

        success, _ = robot_gw.set_initial_pose(x=1.0, y=-2.0, yaw=math.pi)

        assert success is True
        published = publisher.publish.call_args[0][0]
        assert published.header.frame_id == "map"
        assert published.pose.pose.position.x == 1.0
        assert published.pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 2))
