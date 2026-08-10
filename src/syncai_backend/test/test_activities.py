"""Tests for the Temporal activities, run under ``ActivityEnvironment``.

``temporalio.testing.ActivityEnvironment`` supplies the activity context that
``activity.heartbeat`` needs, so the real polling loops run unmodified; the
gateway is a MagicMock (the CoreManager seam-mocking pattern) and the module's
``time`` is patched where a test would otherwise sleep.

What is pinned here is the retryability contract, because Temporal acts on it:
a MOVE that aborts is retryable (the path may clear), a rejected motion key is
not (the driver said no and will keep saying no).

The cancellation paths (thread-cancel shielding in ``_wait_for_nav_goal``) are
deliberately not covered: they need a real threaded worker to inject the
cancel, which is integration-test territory.
"""

from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("temporalio")

from temporalio.exceptions import ApplicationError  # noqa: E402
from temporalio.testing import ActivityEnvironment  # noqa: E402

from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.gateways.workflow.schema import MoveParams  # noqa: E402
from syncai_backend.temporal.activities import RobotActivities  # noqa: E402


SLEEP = "syncai_backend.temporal.activities.time.sleep"


@pytest.fixture
def robot_gw():
    return MagicMock()


@pytest.fixture
def activities(logger, robot_gw) -> RobotActivities:
    return RobotActivities(logger=logger, robot_gw=robot_gw)


@pytest.fixture
def env() -> ActivityEnvironment:
    return ActivityEnvironment()


class TestExecuteMove:
    def test_success_converts_degrees_and_polls_to_terminal(
        self, env, activities, robot_gw
    ):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}

        result = env.run(
            activities.execute_move, MoveParams(x=1.0, y=2.0, theta=90.0)
        )

        assert result.success is True
        assert (result.goal_id, result.state) == ("goal-1", "succeeded")
        # Degrees on the wire, radians at the gateway — this boundary converts.
        kwargs = robot_gw.move.call_args[1]
        assert kwargs["yaw"] == pytest.approx(1.5707963)

    def test_rejection_is_retryable(self, env, activities, robot_gw):
        robot_gw.move.return_value = (False, "server not available", None)

        with pytest.raises(ApplicationError, match="Move rejected") as exc_info:
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        # The task runner coming up a moment later must be given the chance.
        assert exc_info.value.non_retryable is False

    def test_an_aborted_goal_fails_the_attempt(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "aborted"}

        with pytest.raises(ApplicationError, match="move ended in aborted"):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

    def test_polling_heartbeats_until_terminal(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.side_effect = [
            {"goal_id": "goal-1", "state": "executing"},
            {"goal_id": "goal-1", "state": "succeeded"},
        ]
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        with patch(SLEEP):  # the 1 s poll pause, not needed under test
            result = env.run(
                activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0)
            )

        assert result.success is True
        # One heartbeat per poll is what keeps the 3 s heartbeat_timeout fed.
        assert beats == [("executing",), ("succeeded",)]


class TestPostureActivities:
    def test_stand_sends_its_motion_key(self, env, activities, robot_gw):
        robot_gw.set_motion_key.return_value = (True, "")

        result = env.run(activities.execute_stand)

        assert result.success is True
        assert robot_gw.set_motion_key.call_args[1]["key"] is MotionKey.STAND

    def test_a_rejected_key_is_non_retryable(self, env, activities, robot_gw):
        # A False from the driver means unknown key or safety lock — neither
        # goes away on its own, so retrying would just hammer the service.
        robot_gw.set_motion_key.return_value = (False, "LOCKED")

        with pytest.raises(ApplicationError, match="LieDown rejected") as exc_info:
            env.run(activities.execute_lie_down)

        assert exc_info.value.non_retryable is True
