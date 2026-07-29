import json
import math
import structlog
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_common.msg import RobotMode, RobotState as RobotStateMsg

from syncai_backend.exceptions import InternalServerError, NotFoundError
from syncai_backend.gateways.robot.robot import RobotGateway
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

    return robot_router
