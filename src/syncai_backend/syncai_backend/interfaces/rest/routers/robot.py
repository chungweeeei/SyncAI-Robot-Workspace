import json
import math
import structlog
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_common.msg import RobotMode, RobotState as RobotStateMsg

from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.gateways.robot.robot import RobotGateway


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
    battery_percentage: float = Field(
        ..., description="The battery percentage of the robot."
    )


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


class SetMotionKeyRequest(BaseModel):
    key: str = Field(
        ...,
        min_length=1,
        description=(
            "The motion key to send to the driver: "
            "'0' stand, '1' locomotion, '2' lie down, '3' damping, "
            "'4' emergency stop, '5' MPC."
        ),
    )


class SetMotionKeyResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the motion key.")


def init_robot_router(
    logger: structlog.stdlib.BoundLogger,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
) -> APIRouter:
    robot_router = APIRouter(prefix="", tags=["Robot"])

    @robot_router.get("/api/v1/robot/state", response_model=RobotState)
    async def get_robot_state():
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
                battery_percentage=state.battery_status.battery_percentage,
            ),
        )

    # Plain (non-async) handler: the gateway call blocks on a ROS service, so
    # FastAPI runs it in its worker thread pool instead of on the event loop.
    @robot_router.post(
        "/api/v1/robot/motion_key", response_model=SetMotionKeyResponse
    )
    def set_motion_key(request: SetMotionKeyRequest):
        success, message = robot_gw.set_motion_key(key=request.key)
        if not success:
            logger.error(
                "Failed to set motion key", key=request.key, message=message
            )
            raise BadRequestError(message)

        return SetMotionKeyResponse(message=message)

    return robot_router
