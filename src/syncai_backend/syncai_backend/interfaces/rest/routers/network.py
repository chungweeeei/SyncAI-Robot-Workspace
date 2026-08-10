import structlog
from typing import List
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, UpstreamError
from syncai_backend.gateways.robot.robot import RobotGateway


class WifiNetwork(BaseModel):
    bssid: str = Field(..., description="The BSSID (access point MAC) of the network.")
    ssid: str = Field(..., description="The SSID of the network.")
    rssi: int = Field(..., description="The signal strength of the network, in dBm.")


class ScanWifiNetworksResponse(BaseModel):
    networks: List[WifiNetwork] = Field(
        ..., description="The WiFi networks visible to the robot."
    )


class ConnectWifiNetworkRequest(BaseModel):
    ssid: str = Field(
        ..., min_length=1, description="The SSID of the network to connect to."
    )
    password: str = Field(
        "", description="The network password; leave empty for open networks."
    )


class ConnectWifiNetworkResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the connection.")


def init_network_router(
    logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway
) -> APIRouter:
    network_router = APIRouter(prefix="", tags=["Network"])

    # Plain (non-async) handlers: the gateway calls block for up to tens of
    # seconds waiting on ROS services, so FastAPI must run them in its worker
    # thread pool instead of on the event loop.

    @network_router.get(
        "/api/v1/network/wifi/scan", response_model=ScanWifiNetworksResponse
    )
    def scan_wifi_networks():
        success, message, networks = robot_gw.scan_wifi_networks()
        if not success:
            logger.error("Failed to scan WiFi networks", message=message)
            raise UpstreamError(message)

        return ScanWifiNetworksResponse(
            networks=[
                WifiNetwork(bssid=network.bssid, ssid=network.ssid, rssi=network.rssi)
                for network in networks
            ]
        )

    @network_router.post(
        "/api/v1/network/wifi/connect", response_model=ConnectWifiNetworkResponse
    )
    def connect_wifi_network(request: ConnectWifiNetworkRequest):
        success, message = robot_gw.connect_wifi(
            ssid=request.ssid, password=request.password
        )
        if not success:
            logger.error(
                "Failed to connect to WiFi network", ssid=request.ssid, message=message
            )
            raise BadRequestError(message)

        return ConnectWifiNetworkResponse(
            message=f"Successfully connected to {request.ssid}"
        )

    return network_router
