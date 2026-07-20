import os
import struct
import uuid
import structlog
from enum import Enum
from typing import List, Optional
from fastapi import APIRouter, Body, Query
from fastapi.responses import Response
from pydantic import BaseModel, Field

from nav_msgs.msg import OccupancyGrid

from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.database.models import MapPoint
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.helpers.occupancy_grid import occupancy_grid_to_png_base64
from syncai_backend.helpers.pointcloud import (
    read_pcd_xyz,
    voxel_downsample,
    cap_points,
    pack_xyz_f32,
)


# Workspace-relative map root; overridable so the same code works in the
# container (CWD == workspace) and in tests. A saved LIO map lives at
# <MAP_DIR>/<map_name>/map.pcd.
_MAP_DIR = os.environ.get("SYNCAI_MAP_DIR", "map")


class VertexType(str, Enum):
    """Semantic role of a map vertex: what the robot does when it visits.

    Persisted as a plain string in the ``map_vertices`` table; validated at the
    REST boundary only.
    """

    # A plain navigation stop (no pure path-only vertices exist in this
    # system, so every ordinary nav target is GENERAL).
    GENERAL = "GENERAL"
    # An IoT device station (pickup/drop/conveyor, etc.); name mirrors
    # ``StepType.ARTIFACT`` in the workflow schema.
    ARTIFACT = "ARTIFACT"
    # A charging dock.
    CHARGER = "CHARGER"
    # An idle/park base the robot returns to.
    HOME = "HOME"
    # A hold spot for queueing / yielding / waiting on a station to free up.
    WAITING = "WAITING"


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


class MapImageResponse(MapInfoResponse):
    image: str = Field(
        ...,
        description="The map as a base64 PNG data URI (data:image/png;base64,...).",
    )


class MapVertexRequest(BaseModel):
    name: str = Field(..., min_length=1, description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    map_name: str = Field(..., description="Name of the map this vertex belongs to.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class MapVertexUpdateRequest(BaseModel):
    name: Optional[str] = Field(None, min_length=1, description="New vertex name.")
    type: Optional[VertexType] = Field(None, description="New vertex role.")
    map_name: Optional[str] = Field(None, description="New owning map name.")
    x: Optional[float] = Field(None, description="New world x-coordinate (metres).")
    y: Optional[float] = Field(None, description="New world y-coordinate (metres).")
    theta: Optional[float] = Field(None, description="New yaw angle in degrees.")


class MapVertexResponse(BaseModel):
    id: uuid.UUID = Field(..., description="Unique identifier of the vertex.")
    name: str = Field(..., description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    map_name: str = Field(..., description="Name of the map this vertex belongs to.")
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
    )


def _vertex_response(vertex: MapPoint) -> MapVertexResponse:
    return MapVertexResponse(
        id=vertex.id,
        name=vertex.name,
        type=vertex.type,
        map_name=vertex.map_name,
        x=vertex.x,
        y=vertex.y,
        theta=vertex.theta,
    )


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
) -> APIRouter:
    map_router = APIRouter(prefix="", tags=["Map"])

    @map_router.get("/api/v1/map/info", response_model=MapInfoResponse)
    async def get_map_info():
        grid = map_repo.get_map()
        if grid is None:
            raise NotFoundError("Map is not available yet.")
        return _map_info(grid)

    # Plain (non-async) handlers below: OccupancyGrid->PNG encoding is
    # CPU-bound and the vertex repo blocks on psycopg2, so FastAPI runs
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

    @map_router.get("/api/v1/map/pointcloud")
    def get_map_pointcloud(
        map_name: str = Query(..., min_length=1, description="Saved map name."),
        voxel_size: float = Query(
            0.3, ge=0.0, description="Voxel leaf size (m); 0 disables downsampling."
        ),
        max_points: int = Query(
            300000, gt=0, description="Hard cap on returned point count."
        ),
    ):
        """Return the static LIO map cloud as packed binary for the 3D viewer.

        Wire format matches the live stream: a little-endian uint32 point count
        followed by ``3 * count`` little-endian float32 xyz values (map frame).
        The map is static, so the response is cacheable.
        """
        # Guard against path traversal via map_name before touching the FS.
        if "/" in map_name or "\\" in map_name or map_name in (".", ".."):
            raise BadRequestError(f"Invalid map name: {map_name!r}")

        pcd_path = os.path.join(_MAP_DIR, map_name, "map.pcd")
        if not os.path.isfile(pcd_path):
            raise NotFoundError(f"Map point cloud not found for '{map_name}'.")

        points = read_pcd_xyz(pcd_path)
        if voxel_size > 0.0:
            points = voxel_downsample(points, voxel_size)
        points = cap_points(points, max_points)

        payload = struct.pack("<I", points.shape[0]) + pack_xyz_f32(points)
        return Response(
            content=payload,
            media_type="application/octet-stream",
            headers={"Cache-Control": "public, max-age=3600"},
        )

    @map_router.post("/api/v1/map/vertices", response_model=List[MapVertexResponse])
    def create_map_vertices(reqs: List[MapVertexRequest] = Body(..., min_length=1)):
        vertices = map_repo.create_vertices(
            [
                {
                    "name": req.name,
                    "type": req.type.value,
                    "map_name": req.map_name,
                    "x": req.x,
                    "y": req.y,
                    "theta": req.theta,
                }
                for req in reqs
            ]
        )
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.get("/api/v1/map/vertices", response_model=List[MapVertexResponse])
    def list_map_vertices(
        map_name: Optional[str] = None, type: Optional[VertexType] = None
    ):
        vertices = map_repo.list_vertices(
            map_name=map_name, type=type.value if type else None
        )
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.get("/api/v1/map/vertices/{id}", response_model=MapVertexResponse)
    def get_map_vertex(id: uuid.UUID):
        vertex = map_repo.get_vertex(vertex_id=id)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found.")
        return _vertex_response(vertex)

    @map_router.put("/api/v1/map/vertices/{id}", response_model=MapVertexResponse)
    def update_map_vertex(id: uuid.UUID, req: MapVertexUpdateRequest):
        changes = req.model_dump(exclude_unset=True)
        if "type" in changes and changes["type"] is not None:
            changes["type"] = changes["type"].value
        vertex = map_repo.update_vertex(id, **changes)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found.")
        return _vertex_response(vertex)

    @map_router.delete("/api/v1/map/vertices/{id}", response_model=DeleteResponse)
    def delete_map_vertex(id: uuid.UUID):
        deleted = map_repo.delete_vertex(vertex_id=id)
        if not deleted:
            raise NotFoundError(f"Map vertex {id} was not found.")
        return DeleteResponse(message=f"Map vertex {id} has been deleted.")

    return map_router
