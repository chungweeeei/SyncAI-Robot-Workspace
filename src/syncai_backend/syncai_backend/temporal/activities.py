import math
import time
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.workflow.schema import StepParams
from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.artifact.artifact import (
    ArtifactGateway,
    ArtifactCommandRejected,
    ArtifactUnavailable,
    UnknownArtifactError,
)


class ActivityResult(BaseModel):
    success: bool
    goal_id: str | None = None
    state: str | None = None


# Motion keys accepted by syncai_driver_manager's set_motion_key service; it
# maps them to the gait controller's MODE characters (see
# DriverManagerNode::setMotionKeyCallback).
MOTION_KEY_STAND = "0"  # MODE Z
MOTION_KEY_LIE_DOWN = "2"  # MODE X


class RobotActivities:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_gw: RobotGateway,
        artifact_gw: ArtifactGateway,
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._artifact_gw = artifact_gw

    def _wait_for_nav_goal(self, goal_id: str, label: str) -> str:
        """Poll a navigation goal until it reaches a terminal state.

        NOTE: this runs inside a synchronous (threaded) activity. On
        cancellation Temporal *throws* a CancelledError into this thread at
        whatever point it is currently executing (e.g. inside time.sleep), so
        we can't rely on polling activity.is_cancelled() at the top of the
        loop -- we must catch the injected exception to run cleanup.
        """
        try:
            while True:
                status = self._robot_gw.get_move_status(goal_id=goal_id)
                state = status["state"] if status else None

                # send heartbeat, tell temporal server worker still alive
                activity.heartbeat(state)

                if state in ["succeeded", "aborted", "canceled"]:
                    return state

                time.sleep(1.0)

        except CancelledError:
            # shield so the cancel_move RPC finishes before the CancelledError
            # is re-raised, then propagate to mark the activity as cancelled.
            with activity.shield_thread_cancel_exception():
                self._robot_gw.cancel_move(goal_id=goal_id)

            self._logger.warning(f"[RobotActivity] {label} activity has been cancelled")
            raise

    @activity.defn
    def execute_move(self, params: StepParams) -> ActivityResult:
        yaw = math.radians(params.theta)
        accepted, msg, goal_id = self._robot_gw.move(x=params.x, y=params.y, yaw=yaw)
        if not accepted:
            raise ApplicationError(f"Move rejected: {msg}", non_retryable=False)

        self._logger.info("[RobotActivity] Move accepted", goal_id=goal_id)

        state = self._wait_for_nav_goal(goal_id=goal_id, label="Move")

        if state != "succeeded":
            raise ApplicationError(f"move ended in {state}", non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)

    def _set_motion_key(self, key: str, label: str) -> ActivityResult:
        """Send a motion key. Fire-and-forget: this does NOT wait for the pose.

        MODE is a one-way UDP command, so a successful service call only means
        the datagram was sent -- the step completes while the robot is still
        moving its legs. A step queued right behind this one (e.g. a MOVE) will
        therefore start against a robot that has not finished standing up.

        We deliberately do not paper over that with a fixed sleep: the driver
        manager already republishes the controller's MODE_STATE telemetry on
        the `mode` topic (data[0] = policy state, data[1] = motion state), so
        the real fix is to subscribe to it and poll the actual motion state
        here. That is pending the value mapping for data[1], which is defined
        on the gait controller side, not in this workspace.

        A False from the service means the driver manager rejected the key
        (unknown key, or the safety lock is engaged) -- retrying will not fix
        either on its own, hence non_retryable.
        """
        accepted, msg = self._robot_gw.set_motion_key(key=key)
        if not accepted:
            raise ApplicationError(f"{label} rejected: {msg}", non_retryable=True)

        self._logger.info(f"[RobotActivity] {label} command sent", key=key)

        return ActivityResult(success=True, state="succeeded")

    @activity.defn
    def execute_stand(self) -> ActivityResult:
        return self._set_motion_key(key=MOTION_KEY_STAND, label="Stand")

    @activity.defn
    def execute_lie_down(self) -> ActivityResult:
        return self._set_motion_key(key=MOTION_KEY_LIE_DOWN, label="LieDown")

    @activity.defn
    def execute_artifact(self, params: StepParams) -> ActivityResult:
        try:
            ack = self._artifact_gw.send_command(
                artifact_id=params.artifact_id, command=params.command.model_dump()
            )
        except (UnknownArtifactError, ArtifactCommandRejected) as err:
            raise ApplicationError(str(err), non_retryable=True) from err
        except ArtifactUnavailable as err:
            raise ApplicationError(str(err), non_retryable=False) from err

        self._logger.info(
            "[RobotActivity] Artifact command accepted",
            artifact_id=params.artifact_id,
            ack=ack,
        )

        if params.wait_for is None:
            return ActivityResult(success=True)

        # The command ack only means the trigger register was written; poll
        # the artifact state until live_info.phase reaches the expected value.
        expected_phase = params.wait_for.value
        deadline = time.monotonic() + params.wait_timeout_seconds

        try:
            while True:
                # A transient GET failure must not fail (and thus retry) the
                # activity: the edge-triggered command already fired and a
                # retry would re-trigger it. Keep polling until the deadline.
                try:
                    state = self._artifact_gw.get_state(params.artifact_id)
                except (ArtifactUnavailable, ArtifactCommandRejected) as err:
                    self._logger.warning(
                        "[RobotActivity] Artifact state poll failed",
                        artifact_id=params.artifact_id,
                        error=str(err),
                    )
                    state = None

                activity.heartbeat(state)

                if state is not None:
                    if state.get("error_code", 0) != 0:
                        raise ApplicationError(
                            f"artifact reported error_code={state['error_code']}",
                            non_retryable=True,
                        )

                    live_info = state.get("live_info") or {}
                    if live_info.get("phase") == expected_phase:
                        return ActivityResult(success=True)

                if time.monotonic() >= deadline:
                    raise ApplicationError(
                        f"artifact did not reach phase '{expected_phase}' within "
                        f"{params.wait_timeout_seconds}s",
                        non_retryable=True,
                    )

                time.sleep(1.0)

        except CancelledError:
            # No cancel API on the artifact side; the command already fired.
            self._logger.warning(
                "[RobotActivity] Artifact activity has been cancelled",
                artifact_id=params.artifact_id,
            )
            raise
