import math
import structlog
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_common.msg import RobotMode, RobotState as RobotStateMsg

from syncai_backend.exceptions import NotFoundError
from syncai_backend.repositories.robot.robot import RobotRepo


# Reverse lookup: RobotMode uint8 constant -> human-readable name.
_ROBOT_MODE_TO_STR = {
    RobotMode.MAINTENANCE: "MAINTENANCE",
    RobotMode.MANUAL: "MANUAL",
    RobotMode.AUTO: "AUTO",
}


def _mode_to_str(mode: int) -> str:
    return _ROBOT_MODE_TO_STR.get(mode, "UNKNOWN")


class RobotPose(BaseModel):
    x: float = Field(..., description="The x-coordinate of the robot's position.")
    y: float = Field(..., description="The y-coordinate of the robot's position.")
    z: float = Field(..., description="The z-coordinate of the robot's position.")
    theta: float = Field(..., description="The yaw angle of the robot, in degrees.")


class RobotLocalizationStatus(BaseModel):
    position: RobotPose = Field(..., description="The position of the robot.")
    velocity: float = Field(..., description="The velocity of the robot.")


class RobotNetworkStatus(BaseModel):
    wifi_info: str = Field(..., description="The Wi-Fi information of the robot.")


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


def init_robot_router(
    logger: structlog.stdlib.BoundLogger, robot_repo: RobotRepo
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
            network_status=RobotNetworkStatus(
                wifi_info=state.network_status.wifi_info,
            ),
            battery_status=RobotBatteryStatus(
                battery_percentage=state.battery_status.battery_percentage,
            ),
        )

    return robot_router
