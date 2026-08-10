"""Tests for the Temporal activities, run under ``ActivityEnvironment``.

``temporalio.testing.ActivityEnvironment`` supplies the activity context that
``activity.heartbeat`` needs, so the real polling loops run unmodified; the
gateways are MagicMocks (the CoreManager seam-mocking pattern) and the module's
``time`` is patched where a test would otherwise sleep or wait out a deadline.

What is pinned here is the retryability contract, because Temporal acts on it:
a MOVE that aborts is retryable (the path may clear), a rejected motion key is
not (the driver said no and will keep saying no), and an artifact command's
transient state-poll failure must NOT fail the activity at all — the command is
edge-triggered, and a retry would fire it twice.

The cancellation paths (thread-cancel shielding in ``_wait_for_nav_goal``) are
deliberately not covered: they need a real threaded worker to inject the
cancel, which is integration-test territory.
"""

from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("temporalio")

from temporalio.exceptions import ApplicationError  # noqa: E402
from temporalio.testing import ActivityEnvironment  # noqa: E402

from syncai_backend.gateways.artifact.artifact import (  # noqa: E402
    ArtifactUnavailable,
    UnknownArtifactError,
)
from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    ArtifactParams,
    ConveyorPhase,
    MoveParams,
    PickupCommand,
)
from syncai_backend.temporal.activities import RobotActivities  # noqa: E402


SLEEP = "syncai_backend.temporal.activities.time.sleep"
MONOTONIC = "syncai_backend.temporal.activities.time.monotonic"


@pytest.fixture
def robot_gw():
    return MagicMock()


@pytest.fixture
def artifact_gw():
    return MagicMock()


@pytest.fixture
def activities(logger, robot_gw, artifact_gw) -> RobotActivities:
    return RobotActivities(logger=logger, robot_gw=robot_gw, artifact_gw=artifact_gw)


@pytest.fixture
def env() -> ActivityEnvironment:
    return ActivityEnvironment()


def _artifact_params(**overrides) -> ArtifactParams:
    body = {
        "artifact_id": "conveyor01",
        "command": PickupCommand(action="pickup"),
        "wait_for": None,
    }
    body.update(overrides)
    return ArtifactParams(**body)


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


class TestExecuteArtifact:
    def test_fire_and_forget_completes_on_the_ack(self, env, activities, artifact_gw):
        artifact_gw.send_command.return_value = {"accepted": True}

        result = env.run(activities.execute_artifact, _artifact_params())

        assert result.success is True
        # The command body is the schema's dump, forwarded as a plain dict.
        kwargs = artifact_gw.send_command.call_args[1]
        assert kwargs["artifact_id"] == "conveyor01"
        assert kwargs["command"]["action"] == "pickup"
        artifact_gw.get_state.assert_not_called()

    def test_unknown_artifact_is_non_retryable(self, env, activities, artifact_gw):
        artifact_gw.send_command.side_effect = UnknownArtifactError("no conveyor99")

        with pytest.raises(ApplicationError) as exc_info:
            env.run(activities.execute_artifact, _artifact_params())

        assert exc_info.value.non_retryable is True

    def test_unreachable_artifact_is_retryable(self, env, activities, artifact_gw):
        # The command was NOT delivered, so a retry cannot double-trigger.
        artifact_gw.send_command.side_effect = ArtifactUnavailable("refused")

        with pytest.raises(ApplicationError) as exc_info:
            env.run(activities.execute_artifact, _artifact_params())

        assert exc_info.value.non_retryable is False

    def test_wait_for_polls_until_the_phase(self, env, activities, artifact_gw):
        artifact_gw.send_command.return_value = {"accepted": True}
        artifact_gw.get_state.side_effect = [
            {"error_code": 0, "live_info": {"phase": "belt"}},
            {"error_code": 0, "live_info": {"phase": "handoff"}},
        ]

        with patch(SLEEP):
            result = env.run(
                activities.execute_artifact,
                _artifact_params(wait_for=ConveyorPhase.HANDOFF),
            )

        assert result.success is True

    def test_a_transient_poll_failure_does_not_fail_the_activity(
        self, env, activities, artifact_gw
    ):
        # The edge-triggered command already fired; failing (and retrying) the
        # activity on a GET hiccup would re-trigger it. The poll must absorb
        # the error and keep going.
        artifact_gw.send_command.return_value = {"accepted": True}
        artifact_gw.get_state.side_effect = [
            ArtifactUnavailable("hiccup"),
            {"error_code": 0, "live_info": {"phase": "handoff"}},
        ]

        with patch(SLEEP):
            result = env.run(
                activities.execute_artifact,
                _artifact_params(wait_for=ConveyorPhase.HANDOFF),
            )

        assert result.success is True

    def test_a_reported_error_code_is_non_retryable(self, env, activities, artifact_gw):
        artifact_gw.send_command.return_value = {"accepted": True}
        artifact_gw.get_state.return_value = {"error_code": 7, "live_info": {}}

        with pytest.raises(ApplicationError, match="error_code=7") as exc_info:
            env.run(
                activities.execute_artifact,
                _artifact_params(wait_for=ConveyorPhase.HANDOFF),
            )

        assert exc_info.value.non_retryable is True

    def test_the_deadline_fails_the_step(self, env, activities, artifact_gw):
        artifact_gw.send_command.return_value = {"accepted": True}
        artifact_gw.get_state.return_value = {
            "error_code": 0,
            "live_info": {"phase": "belt"},
        }

        # First monotonic() sets the deadline, the second is the loop's check,
        # already past it — no sleeping through a real 60 s timeout.
        with patch(MONOTONIC, side_effect=[0.0, 120.0]):
            with pytest.raises(ApplicationError, match="did not reach phase") as exc_info:
                env.run(
                    activities.execute_artifact,
                    _artifact_params(wait_for=ConveyorPhase.HANDOFF),
                )

        assert exc_info.value.non_retryable is True
