import structlog
from typing import List, Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field

from nav_msgs.msg import OccupancyGrid

from syncai_backend.exceptions import NotFoundError
from syncai_backend.database.models import MapPoint
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.map_point.map_point import MapPointRepo
from syncai_backend.helpers.occupancy_grid import occupancy_grid_to_png_base64


class MapOrigin(BaseModel):
    x: float = Field(..., description="World x-coordinate of the map origin (metres).")
    y: float = Field(..., description="World y-coordinate of the map origin (metres).")
    z: float = Field(..., description="World z-coordinate of the map origin (metres).")


class MapInfoResponse(BaseModel):
    resolution: float = Field(..., description="Map resolution in metres per pixel.")
    width: int = Field(..., description="Map width in pixels (cells).")
    height: int = Field(..., description="Map height in pixels (cells).")
    origin: MapOrigin = Field(
        ..., description="World pose of the map's bottom-left cell (map frame)."
    )
    frame_id: str = Field(..., description="TF frame the map is expressed in.")


class MapImageResponse(MapInfoResponse):
    image: str = Field(
        ...,
        description="The map as a base64 PNG data URI (data:image/png;base64,...).",
    )


class MapPointRequest(BaseModel):
    name: str = Field(..., min_length=1, description="Human-readable point name.")
    type: str = Field(
        ..., description="Point classification, e.g. waypoint/task_point/patrol."
    )
    map_name: str = Field(..., description="Name of the map this point belongs to.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class MapPointUpdateRequest(BaseModel):
    name: Optional[str] = Field(None, min_length=1, description="New point name.")
    type: Optional[str] = Field(None, description="New point classification.")
    map_name: Optional[str] = Field(None, description="New owning map name.")
    x: Optional[float] = Field(None, description="New world x-coordinate (metres).")
    y: Optional[float] = Field(None, description="New world y-coordinate (metres).")
    theta: Optional[float] = Field(None, description="New yaw angle in degrees.")


class MapPointResponse(BaseModel):
    id: int = Field(..., description="Unique identifier of the point.")
    name: str = Field(..., description="Human-readable point name.")
    type: str = Field(..., description="Point classification.")
    map_name: str = Field(..., description="Name of the map this point belongs to.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class DeleteResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the deletion.")


def _map_info(grid: OccupancyGrid) -> MapInfoResponse:
    return MapInfoResponse(
        resolution=grid.info.resolution,
        width=grid.info.width,
        height=grid.info.height,
        origin=MapOrigin(
            x=grid.info.origin.position.x,
            y=grid.info.origin.position.y,
            z=grid.info.origin.position.z,
        ),
        frame_id=grid.header.frame_id,
    )


def _point_response(point: MapPoint) -> MapPointResponse:
    return MapPointResponse(
        id=point.id,
        name=point.name,
        type=point.type,
        map_name=point.map_name,
        x=point.x,
        y=point.y,
        theta=point.theta,
    )


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_point_repo: MapPointRepo,
) -> APIRouter:
    map_router = APIRouter(prefix="", tags=["Map"])

    @map_router.get("/api/v1/map/info", response_model=MapInfoResponse)
    async def get_map_info():
        grid = map_repo.get_map()
        if grid is None:
            raise NotFoundError("Map is not available yet.")
        return _map_info(grid)

    # Plain (non-async) handlers below: OccupancyGrid->PNG encoding is
    # CPU-bound and the map-point repo blocks on psycopg2, so FastAPI runs
    # these in its worker thread pool instead of on the event loop.

    @map_router.get("/api/v1/map/image", response_model=MapImageResponse)
    def get_map_image():
        grid = map_repo.get_map()
        if grid is None:
            raise NotFoundError("Map is not available yet.")

        info = _map_info(grid)
        return MapImageResponse(
            **info.model_dump(),
            image=occupancy_grid_to_png_base64(grid),
        )

    @map_router.post("/api/v1/map/points", response_model=MapPointResponse)
    def create_map_point(req: MapPointRequest):
        point = map_point_repo.create(
            name=req.name,
            type=req.type,
            map_name=req.map_name,
            x=req.x,
            y=req.y,
            theta=req.theta,
        )
        return _point_response(point)

    @map_router.get("/api/v1/map/points", response_model=List[MapPointResponse])
    def list_map_points(
        map_name: Optional[str] = None, type: Optional[str] = None
    ):
        points = map_point_repo.list_all(map_name=map_name, type=type)
        return [_point_response(point) for point in points]

    @map_router.get("/api/v1/map/points/{id}", response_model=MapPointResponse)
    def get_map_point(id: int):
        point = map_point_repo.get(point_id=id)
        if point is None:
            raise NotFoundError(f"Map point {id} was not found.")
        return _point_response(point)

    @map_router.put("/api/v1/map/points/{id}", response_model=MapPointResponse)
    def update_map_point(id: int, req: MapPointUpdateRequest):
        point = map_point_repo.update(id, **req.model_dump(exclude_unset=True))
        if point is None:
            raise NotFoundError(f"Map point {id} was not found.")
        return _point_response(point)

    @map_router.delete("/api/v1/map/points/{id}", response_model=DeleteResponse)
    def delete_map_point(id: int):
        deleted = map_point_repo.delete(point_id=id)
        if not deleted:
            raise NotFoundError(f"Map point {id} was not found.")
        return DeleteResponse(message=f"Map point {id} has been deleted.")

    return map_router
