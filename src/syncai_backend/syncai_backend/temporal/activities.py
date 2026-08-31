import math
import time
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.workflow.schema import MoveParams, SpeakParams
from syncai_backend.gateways.robot.robot import MotionKey, RobotGateway
from syncai_backend.gateways.tts.tts import TtsGateway


class ActivityResult(BaseModel):
    success: bool
    goal_id: str | None = None
    state: str | None = None


class RobotActivities:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_gw: RobotGateway,
        tts_gw: TtsGateway,
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._tts_gw = tts_gw

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
    def execute_move(self, params: MoveParams) -> ActivityResult:
        yaw = math.radians(params.theta)
        accepted, msg, goal_id = self._robot_gw.move(x=params.x, y=params.y, yaw=yaw)
        if not accepted:
            raise ApplicationError(f"Move rejected: {msg}", non_retryable=False)

        self._logger.info("[RobotActivity] Move accepted", goal_id=goal_id)

        state = self._wait_for_nav_goal(goal_id=goal_id, label="Move")

        if state != "succeeded":
            raise ApplicationError(f"move ended in {state}", non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)

    def _set_motion_key(self, key: MotionKey, label: str) -> ActivityResult:
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

        self._logger.info(f"[RobotActivity] {label} command sent", key=key.value)

        return ActivityResult(success=True, state="succeeded")

    @activity.defn
    def execute_stand(self) -> ActivityResult:
        return self._set_motion_key(key=MotionKey.STAND, label="Stand")

    @activity.defn
    def execute_lie_down(self) -> ActivityResult:
        return self._set_motion_key(key=MotionKey.LIE_DOWN, label="LieDown")

    @activity.defn
    def execute_speak(self, params: SpeakParams) -> ActivityResult:
        """Speak on the robot speaker, blocking until playback finishes.

        This never heartbeats: TtsGateway.speak() sits in one blocking call
        (synthesis, then aplay for the length of the utterance, plus the
        one-time ~3 s model load on the first call), so there is no loop to
        heartbeat from. The workflow therefore drops the heartbeat_timeout
        for SPEAK steps and relies on a short start_to_close instead — see
        the per-step options in workflows.py. Same reason it is effectively
        not cancellable mid-utterance: without heartbeats the worker never
        learns of a cancel, so a canceled task finishes the sentence it is on
        before the workflow's CancelledError lands. An utterance is bounded
        (text is capped at 1000 chars, aplay at duration+10 s), so that is a
        few seconds of latency, not a hang.

        Only "unknown voice" is the request's fault and non-retryable;
        everything else (model missing, aplay/device trouble) is treated as
        possibly transient, same philosophy as the move rejections — the
        workflow's maximum_attempts=3 bounds the ones that are not.
        """
        success, message, duration = self._tts_gw.speak(
            text=params.text, voice=params.voice, speed=params.speed
        )
        if not success:
            raise ApplicationError(
                f"Speak failed: {message}",
                non_retryable=message.startswith("unknown voice"),
            )

        self._logger.info("[RobotActivity] Speak finished", duration=duration)

        return ActivityResult(success=True, state="succeeded")
