import json
import math
import structlog
from enum import Enum
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_common.msg import RobotMode, RobotState as RobotStateMsg

from syncai_backend.exceptions import InternalServerError, NotFoundError
from syncai_backend.gateways.robot.robot import MotionKey, RobotGateway
from syncai_backend.repositories.robot.robot import RobotRepo


# Reverse lookup: RobotMode uint8 constant -> human-readable name.
_ROBOT_MODE_TO_STR = {
    RobotMode.MAINTENANCE: "MAINTENANCE",
    RobotMode.MANUAL: "MANUAL",
    RobotMode.AUTO: "AUTO",
}


def _mode_to_str(mode: int) -> str:
    return _ROBOT_MODE_TO_STR.get(mode, "UNKNOWN")


def _parse_wifi_info(wifi_info: str) -> "RobotNetworkStatus":
    # wifi_info is a JSON string published by syncai_robot_state; it is
    # "null" until the first wifi_status message arrives.
    try:
        data = json.loads(wifi_info)
    except (json.JSONDecodeError, TypeError):
        data = None

    if not isinstance(data, dict):
        data = {}

    return RobotNetworkStatus(**data)


class RobotPose(BaseModel):
    x: float = Field(..., description="The x-coordinate of the robot's position.")
    y: float = Field(..., description="The y-coordinate of the robot's position.")
    z: float = Field(..., description="The z-coordinate of the robot's position.")
    theta: float = Field(..., description="The yaw angle of the robot, in degrees.")


class RobotLocalizationStatus(BaseModel):
    position: RobotPose = Field(..., description="The position of the robot.")
    velocity: float = Field(..., description="The velocity of the robot.")


class RobotNetworkStatus(BaseModel):
    ssid: str = Field("", description="The SSID of the connected WiFi network.")
    bssid: str = Field("", description="The BSSID of the connected WiFi access point.")
    rssi: int = Field(0, description="The WiFi signal strength of the robot, in dBm.")
    ip_address: str = Field(
        "", description="The IP address of the robot's WiFi interface."
    )
    mac_address: str = Field(
        "", description="The MAC address of the robot's WiFi interface."
    )


class RobotBatteryStatus(BaseModel):
    battery_percentage: int = Field(
        ..., description="The battery percentage of the robot."
    )


class RobotMotorStatus(BaseModel):
    """One joint's health, as exposed outward.

    A deliberate SUBSET of syncai_common/MotorState: only the fields that say
    whether a motor is in trouble. q / dq / ddq / tau_est stay behind, because
    RobotState.motor_status is a 10 Hz diagnostic snapshot whose samples cannot
    even be ordered (see the comment on that field) — publishing kinematics from
    it would invite consumers to derive motion from a channel that cannot carry
    it. The high-rate joint channel is the telemetry WebSocket.
    """

    name: str = Field(..., description="The URDF joint name of the motor.")
    temperature: int = Field(..., description="The motor temperature, in Celsius.")
    error: int = Field(..., description="The motor error code; 0 when healthy.")


class RobotState(BaseModel):
    timestamp: int = Field(..., description="The timestamp of the robot state.")
    robot_id: str = Field(..., description="The unique identifier of the robot.")
    map: str = Field(..., description="The name of the map the robot is on.")
    mode: str = Field(
        ..., description="The mode of the robot (MAINTENANCE/MANUAL/AUTO)."
    )
    localization_status: RobotLocalizationStatus = Field(
        ..., description="The localization status of the robot."
    )
    network_status: RobotNetworkStatus = Field(
        ..., description="The network status of the robot."
    )
    battery_status: RobotBatteryStatus = Field(
        ..., description="The battery status of the robot."
    )
    motor_status: list[RobotMotorStatus] = Field(
        default_factory=list,
        description=(
            "Per-joint motor health (name/temperature/error). Empty while "
            "syncai_driver_manager is not publishing motor_states."
        ),
    )


class SetInitialPoseRequest(BaseModel):
    x: float = Field(..., description="The x-coordinate of the pose, in the map frame.")
    y: float = Field(..., description="The y-coordinate of the pose, in the map frame.")
    theta: float = Field(
        ..., description="The yaw angle of the pose, in degrees (map frame)."
    )


class SetInitialPoseResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the request.")


class SetMotionKeyRequest(BaseModel):
    """A gait-controller motion key, as the driver's numeric string.

    Typed with the gateway's ``MotionKey`` rather than a REST-local copy of it.
    All six keys are in the vocabulary, so a second enum would be a verbatim
    duplicate of the one that already has to track
    ``DriverManagerNode::setMotionKeyCallback``'s keymap — and the gateway's
    ``set_motion_key()`` takes a ``MotionKey`` anyway.

    That last part is a live trap, not a hypothetical. The endpoint this one
    replaces (added in ``daca318``, removed in ``191b484`` when the Temporal
    STANDUP / LIEDOWN steps took over) declared ``key: str`` and passed it
    straight through, which worked because the gateway took a ``str`` then.
    ``191b484`` also changed the gateway to take the enum, so restoring the old
    handler verbatim would raise ``AttributeError: 'str' object has no attribute
    'value'`` inside the gateway — and ``server.py`` has no catch-all handler,
    so it would surface as an unhandled 500.
    """

    key: MotionKey = Field(
        ...,
        description=(
            "'0' stand, '1' locomotion, '2' lie down, '3' damping, "
            "'4' emergency stop (accepted but NOT forwarded — see `sent`), "
            "'5' MPC."
        ),
    )


class SetMotionKeyResponse(BaseModel):
    key: MotionKey = Field(..., description="The key the request asked for.")
    sent: bool = Field(
        ...,
        description=(
            "Whether the key was actually forwarded to the driver. False only "
            "for '4' (emergency stop), which this endpoint accepts but does not "
            "forward — no ESTOP datagram was produced. True means the driver "
            "reported writing the datagram, NOT that the robot moved: the "
            "command is one-way UDP with no acknowledgement."
        ),
    )
    message: str = Field(..., description="Human-readable result of the request.")


class PolicyMode(int, Enum):
    """Gait-controller policy indices this REST surface accepts.

    PROVENANCE, because it is defined nowhere in this workspace: the mapping
    comes from the reference implementation's Readme
    (``SyncAI-Robot-GaitMPC/src/udp_ros_bridge`` — "SetPolicyMode: 0=PPO,
    1=HIMLOCO, 2=CHAMP, 3=ISSAC"). ``syncai_common/srv/SetPolicyMode.srv`` is
    four lines with no comments and no constants, and the driver forwards the
    number verbatim as ``MODE <uint>`` without validating it. So these names
    document a firmware behaviour that is **unverified against the controller
    currently running** — on hardware, trust the index, not the label.

    Only 0 and 1 are listed. 2 (CHAMP) and 3 (ISSAC) exist in the controller's
    enum but are deliberately not exposed: nobody here has run them on this
    robot, and an untested locomotion policy should not be reachable from an
    unauthenticated HTTP POST by accident. 4–255 are not legal at all, and the
    driver would still forward them (``setPolicyModeCallback`` only fails if
    ``snprintf`` fails, which it cannot). Widening this is a code change with a
    reviewer attached, which is the point.

    This constrains the **REST vocabulary only, not the ROS contract**: the srv
    field stays a bare uint8 and ``RobotGateway.set_policy_mode()`` stays
    ``int``, so the note in ``syncai_common/README.md`` still holds and a
    non-REST caller can still send an unexposed index.

    Not to be confused with ``RobotMode`` (MAINTENANCE / MANUAL / AUTO), the
    ``mode`` field of ``GET /api/v1/robot/state``. Two unrelated things called
    "mode" now live in this file; they merely share the uint8 type.
    """

    PPO = 0
    HIMLOCO = 1


class SetPolicyModeRequest(BaseModel):
    mode: PolicyMode = Field(
        ...,
        description=(
            "The gait controller's policy index: 0 PPO, 1 HIMLOCO. Not the "
            "robot mode reported by GET /api/v1/robot/state."
        ),
    )


class SetPolicyModeResponse(BaseModel):
    mode: PolicyMode = Field(..., description="The policy mode that was sent.")
    message: str = Field(..., description="Human-readable result of the request.")
    # No `sent` flag here, unlike SetMotionKeyResponse: every value this enum
    # admits is forwarded, so a field that is always True would only dilute the
    # one place that flag carries information.


def init_robot_router(
    logger: structlog.stdlib.BoundLogger,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
) -> APIRouter:
    robot_router = APIRouter(prefix="", tags=["Robot"])

    @robot_router.get("/api/v1/robot/state", response_model=RobotState)
    async def get_robot_state():
        # This response body is a frozen third-party contract, so the fields are
        # named one by one below rather than serialised wholesale. That is the
        # ONLY thing keeping the internal parts of RobotState — motor_timestamp,
        # localization_valid, and the kinematic half of each MotorState — out of
        # a public payload. Adding a field to the message must not add one here.
        #
        # motor_status IS exposed, but re-projected through RobotMotorStatus:
        # name / temperature / error only. Copy the fields explicitly for the
        # same reason as above — a widened MotorState must not widen this.
        #
        # A None here means no sample with localization_valid has arrived yet;
        # the subscriber drops the invalid ones so this endpoint keeps 404-ing
        # rather than reporting a zeroed pose as real.
        state: RobotStateMsg = robot_repo.get_robot_state()
        if state is None:
            raise NotFoundError("Robot state is not available yet.")

        return RobotState(
            timestamp=state.timestamp,
            robot_id=state.robot_id,
            map=state.map,
            mode=_mode_to_str(state.mode),
            localization_status=RobotLocalizationStatus(
                position=RobotPose(
                    x=state.localization_status.position.x,
                    y=state.localization_status.position.y,
                    z=state.localization_status.position.z,
                    theta=math.degrees(state.localization_status.position.yaw),
                ),
                velocity=state.localization_status.velocity,
            ),
            network_status=_parse_wifi_info(state.network_status.wifi_info),
            battery_status=RobotBatteryStatus(
                battery_percentage=int(state.battery_status.battery_percentage),
            ),
            motor_status=[
                RobotMotorStatus(
                    name=motor.name,
                    # int8 / uint16 on the wire; int() is what keeps numpy
                    # scalars from rclpy's array fields out of the JSON encoder.
                    temperature=int(motor.temperature),
                    error=int(motor.error),
                )
                for motor in state.motor_status
            ],
        )

    @robot_router.post(
        "/api/v1/robot/set_initial_pose", response_model=SetInitialPoseResponse
    )
    def set_initial_pose(request: SetInitialPoseRequest):
        # Plain (non-async) handler for the same reason as the network router:
        # the gateway call touches rclpy, so it belongs on FastAPI's worker
        # thread pool rather than the event loop.
        #
        # Degrees in, radians out — the REST vocabulary is degrees everywhere
        # (RobotPose.theta, the map vertices), the ROS side is radians, and this
        # boundary is where that conversion happens.
        success, message = robot_gw.set_initial_pose(
            x=request.x, y=request.y, yaw=math.radians(request.theta)
        )
        if not success:
            logger.error(
                "Failed to set initial pose",
                x=request.x,
                y=request.y,
                theta=request.theta,
                message=message,
            )
            raise InternalServerError(message)

        return SetInitialPoseResponse(
            message=(
                f"Published initial pose x={request.x}, y={request.y}, "
                f"theta={request.theta}"
            )
        )

    # The two command endpoints below are plain (non-async) for the same reason
    # as set_initial_pose above: the gateway calls touch rclpy and block —
    # wait_for_service alone is up to 5 s before a single byte moves — so
    # FastAPI must run them on its worker thread pool instead of stalling the
    # event loop that serves every other route and the point-cloud WebSocket.
    #
    # NO RATE LIMITING, deliberately. This is the record of why, not an
    # oversight:
    #   * The driver has none either. Its four services share one
    #     MutuallyExclusive callback group, which *orders* them against each
    #     other but does not slow anything down — serialisation is not a rate
    #     limit.
    #   * A 200 from either endpoint does not mean the robot did anything.
    #     udpSend() ignores sendto()'s return and the link is unacknowledged, so
    #     a dropped MODE datagram is simply lost, with nothing retrying and
    #     nothing reporting it.
    #   * A REST-level debounce would buy almost nothing. The Temporal STANDUP /
    #     LIEDOWN activities call RobotGateway.set_motion_key() directly,
    #     in-process, and never pass through this router; `ros2 service call`,
    #     the reference GUI and the teleop cmd_vel producer never reach the
    #     backend at all. It would throttle only the slowest caller while
    #     implying the command stream is regulated — worse than the honest
    #     absence.
    #   * MODE is overloaded on the wire: set_motion_key sends `MODE <char>`,
    #     set_policy_mode sends `MODE <uint>`. These two endpoints therefore feed
    #     different command families into the same keyword on the same socket,
    #     with no cross-endpoint ordering guarantee beyond the driver's callback
    #     group. Flipping policy while a gait key is in flight has a result
    #     defined only by the controller.
    # Any real interlock belongs on the driver or the controller, where every
    # caller passes.

    @robot_router.post(
        "/api/v1/robot/set_motion_key", response_model=SetMotionKeyResponse
    )
    def set_motion_key(request: SetMotionKeyRequest):
        # '4' (ESTOP) is accepted by the schema but NOT forwarded.
        #
        # Accepted, because the driver supports it and hiding it would make a
        # documented key look invalid — a 422 reading "not a valid key" for the
        # one key an operator most expects to work is worse than an explicit
        # refusal.
        #
        # Not forwarded, because an emergency stop reached over unauthenticated
        # HTTP, on a one-way UDP command with no acknowledgement and no way to
        # confirm the robot stopped, is a safety claim this layer cannot honour.
        #
        # `sent=False` is the load-bearing half of the answer, not the prose: a
        # caller must not be able to read this 200 as "emergency stop engaged".
        # Same shape as SaveGridmapResponse.reloaded in routers/map.py — the
        # request completed, one of its effects did not. 4xx/5xx was rejected
        # (the request is not malformed, and a non-2xx invites a retry loop
        # against ESTOP of all things); 202 was rejected (nothing is queued).
        #
        # Known follow-up: the driver's own asymmetry is the opposite of this
        # one. While safe_lock_ is engaged, '4' is the ONLY key that passes and
        # every other is rejected — and reset_safety, the release, has no
        # gateway client at all. So once the lock is finally wired (nothing sets
        # it today), this surface will hold the four keys that get rejected,
        # refuse the one that passes, and offer no way to clear it. Add a
        # reset_safety client before ESTOP is ever forwarded from here.
        if request.key is MotionKey.ESTOP:
            logger.warning(
                "Refused to forward ESTOP motion key; no datagram sent",
                key=request.key.value,
            )
            return SetMotionKeyResponse(
                key=request.key,
                sent=False,
                message=(
                    "Emergency stop is not available over this API: no ESTOP "
                    "was sent to the robot."
                ),
            )

        success, message = robot_gw.set_motion_key(key=request.key)
        if not success:
            # Uniform 502 for every gateway failure, matching set_initial_pose
            # above and the map router. The driver's reasons ("LOCKED",
            # "Unknown motion key 'x'") and the gateway's own ("not available",
            # "Timeout") all mean one thing to a caller: a downstream we do not
            # control did not do the thing. Discriminating on the message string
            # would couple this router to C++ literals that no test pins and the
            # driver is free to reword, and would answer 400 for a request that
            # was perfectly well formed. The message goes up verbatim so the
            # operator still gets the real reason.
            logger.error(
                "Failed to set motion key", key=request.key.value, message=message
            )
            raise InternalServerError(message)

        # No success log: the gateway already logged the attempt at info, and a
        # second line for the same event is noise. The refusal above does log,
        # because that path never reaches the gateway.
        #
        # The sentence is this router's own; the driver's ("Motion key sent") is
        # dropped, following set_initial_pose and connect_wifi — the response
        # text is this API's vocabulary, not a downstream string we would then
        # owe stability to.
        return SetMotionKeyResponse(
            key=request.key,
            sent=True,
            message=f"Sent motion key {request.key.value} ({request.key.name})",
        )

    @robot_router.post(
        "/api/v1/robot/set_policy_mode", response_model=SetPolicyModeResponse
    )
    def set_policy_mode(request: SetPolicyModeRequest):
        # PolicyMode does the rejecting, so an unexposed or illegal index is a
        # 422 from FastAPI before any ROS client is touched. That matters
        # because nothing downstream would catch it: the driver forwards
        # `MODE 200` verbatim and the controller validates it no more than the
        # driver does.
        #
        # .value, not the member: the gateway's contract is a plain int, which is
        # what keeps SetPolicyMode.mode a bare uint8 rather than something this
        # REST enum defines.
        #
        # Note this command is NOT gated by the driver's safety lock, unlike
        # set_motion_key — a line that was dropped when the node was ported from
        # udp_ros_bridge, and which syncai_driver_manager/README.md records as a
        # known gap. Once the lock is wired, a policy switch will still go
        # through while motion keys are being rejected.
        success, message = robot_gw.set_policy_mode(mode=request.mode.value)
        if not success:
            # Same uniform 502, for the same reasons as above. Worth knowing
            # while reading this: setPolicyModeCallback cannot currently fail,
            # so in practice this branch reports the gateway's own "service is
            # not available" / "Timeout" rather than anything the driver said.
            logger.error(
                "Failed to set policy mode", mode=request.mode.value, message=message
            )
            raise InternalServerError(message)

        # 200 means the driver wrote `MODE <n>` to the socket. It does not mean
        # the controller switched policy: there is no acknowledgement, and
        # nothing in this workspace subscribes to the `mode` topic that reports
        # the actual policy state back.
        return SetPolicyModeResponse(
            mode=request.mode,
            message=f"Sent policy mode {request.mode.value} ({request.mode.name})",
        )

    return robot_router
