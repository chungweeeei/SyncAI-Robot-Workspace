"""The map catalogue: the maps stored on disk.

One character of path separates this from ``routers/map.py``, so to be explicit:

* ``/api/v1/map/...``  (singular) is the map the stack has **loaded** — info,
  image and point cloud read from live ROS topics — plus vertex CRUD.
* ``/api/v1/maps/...`` (plural, here) is what is **on disk** under ``map/``:
  which maps exist, their geometry, a thumbnail, and the raw gridmap bytes the
  frontend's editor loads.

Read-only. Writing an edited gridmap back needs a new endpoint *and* a decision
about the running stack (``nav2_msgs/SaveMap`` takes a topic name, not grid data,
so it cannot persist an edited array at all); that is a separate round.
"""

import hashlib
from typing import Dict, List, Optional, Tuple

import structlog
from fastapi import APIRouter, Request, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import NotFoundError
from syncai_backend.helpers.pgm import render_thumbnail
from syncai_backend.repositories.map.catalog import MapCatalogRepo, StoredMap
from syncai_backend.repositories.map.map import MapRepo


class GridOrigin(BaseModel):
    x: float = Field(..., description="World x of the grid's lower-left corner (m).")
    y: float = Field(..., description="World y of the grid's lower-left corner (m).")
    yaw: float = Field(..., description="Grid rotation in the map frame (radians).")


class GridInfoResponse(BaseModel):
    """Geometry of a stored gridmap.

    ``origin`` is the three-element origin from ``gridmap.yaml``, so its third
    component is a **yaw**. The similarly shaped ``MapOrigin`` in the singular
    map router is the ``{x, y, z}`` *position* of an OccupancyGrid message — the
    two are not interchangeable despite looking it.
    """

    resolution: float = Field(..., description="Metres per cell.")
    origin: GridOrigin = Field(..., description="Pose of the grid's lower-left cell.")
    width: int = Field(..., description="Grid width in cells.")
    height: int = Field(..., description="Grid height in cells.")


class MapSummaryResponse(BaseModel):
    name: str = Field(..., description="Directory name under the maps root.")
    active: bool = Field(
        ..., description="Whether this is the map the stack was launched with."
    )
    grid: Optional[GridInfoResponse] = Field(
        None,
        description=(
            "Gridmap geometry, or null when the map has been saved from LIO but "
            "not yet converted by tools/pcd_to_gridmap.py."
        ),
    )
    thumbnail: Optional[str] = Field(
        None,
        description=(
            "Path of this map's thumbnail endpoint, or null when it has no grid."
        ),
    )
    has_pointcloud: bool = Field(
        ..., description="Whether map.pcd is present (the 3D localizer's source)."
    )
    size_bytes: int = Field(..., description="Total size of the map directory.")
    modified_at: str = Field(
        ..., description="ISO 8601 timestamp of the newest file in the directory."
    )
    vertex_count: int = Field(
        ..., description="Number of stored vertices belonging to this map."
    )


def _summary(
    stored: StoredMap, active_name: Optional[str], vertex_count: int
) -> MapSummaryResponse:
    grid = (
        GridInfoResponse(
            resolution=stored.grid.resolution,
            origin=GridOrigin(
                x=stored.grid.origin[0],
                y=stored.grid.origin[1],
                yaw=stored.grid.origin[2],
            ),
            width=stored.grid.width,
            height=stored.grid.height,
        )
        if stored.grid is not None
        else None
    )

    return MapSummaryResponse(
        name=stored.name,
        active=stored.name == active_name,
        grid=grid,
        thumbnail=(
            f"/api/v1/maps/{stored.name}/thumbnail" if stored.grid is not None else None
        ),
        has_pointcloud=stored.has_pointcloud,
        size_bytes=stored.size_bytes,
        modified_at=stored.modified_at.isoformat().replace("+00:00", "Z"),
        vertex_count=vertex_count,
    )


def _content_tag(data: bytes) -> str:
    """Build a strong ETag over the gridmap's own bytes.

    Deliberately not ``(mtime, size)``, which is the cheap and usual choice and
    is broken here. An edited gridmap has the **same dimensions**, so the same
    file size; and this workspace's filesystem hands out a coarse mtime — six
    consecutive writes measured identical ``st_mtime_ns``. The pair would
    therefore be unchanged across a save, and both this router's thumbnail cache
    and the browser would keep serving the pre-edit image.

    The extra cost is one blake2b pass (a few ms on 2.3 MB) over bytes the
    handler has already read, against re-encoding a PNG it did not need to.
    """
    return f'"{hashlib.blake2b(data, digest_size=16).hexdigest()}"'


def _not_modified(request: Request, tag: str) -> bool:
    """Report whether the client already holds this exact content.

    Both endpoints send ``no-cache``, which means "revalidate", not "do not
    store" — so the browser does come back with If-None-Match and this is what
    turns that into a 304 instead of another 2.3 MB body.
    """
    header = request.headers.get("if-none-match")
    return bool(header) and tag in [value.strip() for value in header.split(",")]


def init_maps_router(
    logger: structlog.stdlib.BoundLogger,
    catalog_repo: MapCatalogRepo,
    map_repo: MapRepo,
) -> APIRouter:
    maps_router = APIRouter(prefix="", tags=["Maps"])

    # Plain (non-async) handlers throughout: these walk directories, read PGM
    # headers, encode PNGs and hit psycopg2 for the vertex counts, so FastAPI
    # must run them in its worker thread pool rather than on the event loop.

    # Thumbnails are re-encoded only when the source file changes. Four cards on
    # the catalogue page would otherwise re-decode and re-scale four multi-
    # megabyte PGMs on every visit.
    thumbnail_cache: Dict[str, Tuple[str, bytes]] = {}

    def _vertex_count(name: str) -> int:
        # map_vertices.map_name holds the bare directory name, the same spelling
        # this catalogue uses. RobotState.map, by contrast, is a path
        # ("map/dp2f/gridmap.yaml") — reconciling the two is exactly why `active`
        # is resolved on this side and not in the UI.
        return len(map_repo.list_vertices(map_name=name))

    def _require(name: str) -> StoredMap:
        stored = catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

    @maps_router.get("/api/v1/maps", response_model=List[MapSummaryResponse])
    def list_maps():
        active_name = catalog_repo.active_name()
        return [
            _summary(stored, active_name, _vertex_count(stored.name))
            for stored in catalog_repo.list_maps()
        ]

    @maps_router.get("/api/v1/maps/{name}", response_model=MapSummaryResponse)
    def get_map(name: str):
        stored = _require(name)
        return _summary(stored, catalog_repo.active_name(), _vertex_count(name))

    def _read_gridmap(name: str) -> bytes:
        """Read the map's gridmap.pgm bytes, 404ing with the reason if absent."""
        _require(name)
        path = catalog_repo.gridmap_path(name)
        if path is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap. Run tools/pcd_to_gridmap.py "
                "over its map.pcd first."
            )
        try:
            with open(path, "rb") as handle:
                return handle.read()
        except OSError as exc:
            logger.error("Failed to read gridmap", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

    @maps_router.get("/api/v1/maps/{name}/gridmap")
    def get_map_gridmap(name: str, request: Request):
        """Return the map's gridmap.pgm, byte for byte.

        Served verbatim rather than as JSON: the editor parses the P5 header
        itself, and a 2.3 MB grid base64'd into a JSON envelope would be a third
        larger for no benefit.
        """
        payload = _read_gridmap(name)
        tag = _content_tag(payload)

        # Revalidate every time rather than letting the browser hold it: an
        # operator may have just saved an edit, and opening the editor on a stale
        # grid would silently discard that work on the next save.
        headers = {"ETag": tag, "Cache-Control": "no-cache"}
        if _not_modified(request, tag):
            return Response(status_code=304, headers=headers)

        return Response(
            content=payload,
            media_type="application/octet-stream",
            headers=headers,
        )

    @maps_router.get("/api/v1/maps/{name}/thumbnail")
    def get_map_thumbnail(name: str, request: Request):
        payload = _read_gridmap(name)
        # The tag is over the *source* bytes, not the PNG: the thumbnail is a
        # deterministic function of the gridmap, so this changes exactly when the
        # image would, and it is what lets the 304 be answered without encoding.
        tag = _content_tag(payload)
        headers = {"ETag": tag, "Cache-Control": "no-cache"}

        # Before rendering, not after: a client holding the current image must not
        # cost us a PNG encode we then throw away.
        if _not_modified(request, tag):
            return Response(status_code=304, headers=headers)

        cached = thumbnail_cache.get(name)
        if cached is None or cached[0] != tag:
            try:
                png = render_thumbnail(payload)
            except ValueError as exc:
                logger.error("Failed to render map thumbnail", map=name,
                             error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable gridmap.")
            cached = (tag, png)
            thumbnail_cache[name] = cached

        return Response(content=cached[1], media_type="image/png", headers=headers)

    return maps_router
