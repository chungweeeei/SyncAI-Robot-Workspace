import hashlib
import json
import os
import struct
import threading
import uuid
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import structlog
from fastapi import APIRouter, Body, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, NotFoundError, UpstreamError
from syncai_backend.database.models import MapPoint
from syncai_backend.gateways.map.map import MapGateway
# Both conversions are on this route's path now — pick_recipe below chooses per
# map. Neither pulls in open3d (that is why stage 3 lives in this module and not
# in helpers.traversable), so importing both at startup costs nothing.
from syncai_backend.helpers.pcd_to_gridmap import (
    convert_pcd_to_gridmap,
    convert_traversable_to_gridmap,
    floor_level,
)
from syncai_backend.helpers.pgm import render_png, render_thumbnail
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    read_pcd_xyz,
    voxel_downsample,
)
from syncai_backend.repositories.map.catalog import MapCatalogRepo, StoredMap
from syncai_backend.repositories.map.map import MapRepo


# Decimation for a stored map.pcd. Same numbers the point-cloud subscriber
# applies to the live localizer/map_cloud topic (see PointCloudSubscriber),
# duplicated rather than imported so this router does not depend on a
# subscriber; if one moves, move the other.
MAP_CLOUD_VOXEL_SIZE = 0.3
MAP_CLOUD_MAX_POINTS = 300000

# POST /api/v1/maps has two recipes and picks between them by site size — see
# pick_recipe. Neither is a fallback for the other; they answer different
# questions and disagree about what an unknown cell means, which is exactly why
# the choice is recorded on disk rather than left implicit.
#
# **Small site -> the z-band recipe** (GRIDMAP_RECIPE below). It produces a
# genuine trinary map: walls where obstacles were observed, unknown where
# nothing was. Two things follow, and both matter more than the recipe's
# weakness. Unknown is *recoverable* — costmap_layer.cpp:90 lets the obstacle
# layer's live observations overwrite a NO_INFORMATION master cell, while a
# LETHAL one can never be lowered, so a map that says "unknown" here can still
# grow as the robot drives and a map that says "wall" cannot. And a small site
# is cheap to finish by hand in the gridmap editor, which is how every gridmap
# on this fleet was actually made: dp2f still carries the pre-edit
# gridmap_raw.pgm next to the edited gridmap.pgm, and the edit lifted its
# largest connected free region from 89.1% to 94.4% of all free cells.
#
# **Large site -> the traversability recipe** (helpers.traversable: segment the
# floor by lidar return intensity, surface normal and height, repair it, then
# project it). Nobody hand-edits 6000 m², so the automatic repair has to carry
# the map, and this is the recipe that has one. It also sidesteps the z-band's
# real weakness — classifying a cell by *where* its points are in z cannot
# distinguish a drivable aisle from a kerb top or a ramp, and LIO's z=0 is the
# lidar mount height at the mapping start pose, so those bands are a per-site
# guess.
#
# What the large-site choice costs, and it is not small: the output has **no
# unknown cells**. The input cloud is taken as the whole of the drivable world,
# so every cell it does not cover comes out occupied — the padding ring
# included, and any real floor the segmentation wrongly rejected included. That
# is the safe direction for the planner, but it is unforgiving, and by the note
# above it is also permanent: unobserved area walled off this way can never be
# cleared by driving there.
#
# The parameters stay empty because the helper's own defaults are now the tuned
# ones (its module docstring carries the measurement that moved them). A site
# that needs them changed needs them changed with the intermediate clouds in
# front of you, not guessed here — dp1f is the live example: it converts, and
# converts well, but its bottom aisle needs a wider TRAVERSABLE_GRID_RECIPE
# gap_fill_size to bridge a doorway the floor sampling missed.
TRAVERSABLE_SEGMENT_RECIPE: Dict[str, object] = {}
TRAVERSABLE_REPAIR_RECIPE: Dict[str, object] = {}
TRAVERSABLE_GRID_RECIPE: Dict[str, object] = {}

# Footprint area, in m², at or above which a save converts with the
# traversability recipe instead of the z-band one. The measured areas on this
# fleet are 814 (warehouse01), 1645 (the map this threshold was set against),
# 4238 (dp2f) and 6338 (dp1f) m², so 3000 is the widest gap in that spread and
# puts the two clouds nobody would hand-edit on the automatic-repair side. It is
# a judgement about hand-editing effort, not a property of either algorithm —
# move it when the fleet's idea of "small enough to finish by hand" moves.
LARGE_SITE_AREA_M2 = 3000.0

# Sidecar naming the recipe a stored gridmap came from. Written next to the pgm
# because the catalogue would otherwise hold maps from two recipes with nothing
# on disk saying which produced what, and the two disagree about the meaning of
# an unknown cell — a map with no unknown cells at all is either a
# traversability output or a z-band output of a fully observed site, and there
# is no way to tell from the pgm. Tiny, so its effect on the size _walk_stats
# reports is noise.
GRIDMAP_RECIPE_SIDECAR = "gridmap.recipe.json"

# Where the segmentation's intermediate clouds go, or None for "do not write
# them". Off by default because MapCatalogRepo._walk_stats recurses, so five
# extra clouds would triple the size every catalogue card reports for a map.
# When a site's grid comes out wrong, that is the moment to want them — set this
# to a subdirectory name, or better, re-run the pipeline by hand with
# build_traversable_cloud(..., debug_dir=...) and leave the route alone.
TRAVERSABLE_DEBUG_SUBDIR: Optional[str] = None


# The z-band recipe's non-band parameters. The bands themselves are below,
# because they are no longer constants.
#
# obstacle_close is 1, not the 2 this recipe shipped with. At 2 the 10 cm
# morphological closing sealed a doorway a robot had demonstrably driven through
# — 16 of that map's 212 keyframe poses landed on occupied cells, all within
# 5-14 cm of free space. At 1 that drops to 9, all of them poses that brushed a
# wall, and the walls and pillars survive intact. Raising it back closes real
# gaps at the cost of closing real doors; lower it before raising it.
GRIDMAP_RECIPE = dict(
    free_mode="floor",
    min_points=2,
    obstacle_close=1,
    free_close=5,
    despeckle_min_size=12,
    fill_holes_max_size=20000,
)

# The z-band recipe's bands, as offsets from the cloud's **measured** floor
# level rather than absolute z. These are the numbers every gridmap on the fleet
# before 2026-08 was built with (-0.95 / -0.25 / -0.3 / 1.5 absolute), minus the
# floor level of dp1f, the site they were picked on: -0.66. On dp1f they
# therefore resolve to exactly what they always were.
#
# They are offsets because absolute they were a per-site guess by construction —
# z=0 in a LIO map is the lidar mount height at the mapping start pose, so a
# different mount, a different chassis or a start pose on a slope moves the
# whole band while the floor stays where it is. The measured floor levels on
# this fleet span -0.36 to -0.73, and 37 cm is more than the floor band is wide.
# Held absolute on the shallowest of those sites, the bands put 26 of 212
# keyframe poses in unknown space and 35 on occupied cells — the robot's own
# path, walled off. Recentred, that is 0 and 10. dp1f is unchanged and dp2f
# moves by 4 cm, which is the raw-cloud estimate disagreeing with the
# flat-points one (see pcd_to_gridmap.floor_level) and well inside the band.
GRIDMAP_BANDS_ABOVE_FLOOR = dict(
    floor_zmin=-0.29,
    floor_zmax=0.41,
    zmin=0.36,
    zmax=2.16,
)


def pick_recipe(
    logger: structlog.stdlib.BoundLogger, pcd_path: str
) -> Tuple[str, float, float]:
    """Choose a conversion recipe from the cloud; report it, the area, the floor.

    Area rather than point count, because the question the threshold is standing
    in for is "would a person finish this map in the gridmap editor?", and that
    scales with floor area, not with how long the lidar dwelled. Point *density*
    would be the natural discriminator for whether the traversability recipe can
    segment a floor at all, and it is deliberately not used here: after the
    density gates were retuned it segments every cloud on this fleet, so density
    no longer decides anything and gating on it would only reintroduce a silent
    failure mode.

    The floor level comes back from the same read because the z-band branch needs
    it to place its bands and the cloud is already in memory. The traversability
    branch measures its own, from flat points rather than from the raw cloud, and
    ignores this one — the two agree to within 4 cm on this fleet, but the flat
    estimate is the better-founded of the two and that branch can afford it.

    Reads the cloud, which the chosen conversion then reads again. That is one
    extra pass over ~20 MB inside a background thread that is about to spend
    tens of seconds in open3d, against the alternative of threading an array
    through two conversions whose input types differ.
    """
    xyz = read_pcd_xyz(pcd_path)
    if len(xyz) == 0:
        raise ValueError(f"point cloud is empty: {pcd_path}")
    extent = xyz[:, :2].max(axis=0) - xyz[:, :2].min(axis=0)
    area = float(extent[0] * extent[1])
    floor_z = floor_level(xyz[:, 2].astype(np.float64))
    recipe = "traversability" if area >= LARGE_SITE_AREA_M2 else "z-band"
    logger.info(
        "picked gridmap recipe",
        recipe=recipe,
        footprint_m2=round(area, 1),
        threshold_m2=LARGE_SITE_AREA_M2,
        floor_z=round(floor_z, 3),
        points=len(xyz),
    )
    return recipe, area, floor_z


def _write_recipe_sidecar(
    logger: structlog.stdlib.BoundLogger, directory: str, payload: Dict[str, object]
) -> None:
    """Record which recipe produced this map's gridmap, next to the gridmap.

    Best-effort: a sidecar that fails to write must not lose a gridmap that
    converted fine, so this logs and returns rather than raising into the
    conversion thread's handler.
    """
    path = os.path.join(directory, GRIDMAP_RECIPE_SIDECAR)
    try:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
    except OSError as exc:
        logger.warning("Could not write gridmap recipe sidecar", path=path, error=str(exc))


# --- Schemas ----------------------------------------------------------------


class VertexType(str, Enum):
    """Semantic role of a map vertex: what the robot does when it visits.

    Persisted as a plain string in the ``map_vertices`` table; validated at the
    REST boundary only.
    """

    # A plain navigation stop (no pure path-only vertices exist in this
    # system, so every ordinary nav target is GENERAL).
    GENERAL = "GENERAL"
    # An IoT device station (pickup/drop/conveyor, etc.). The name used to
    # mirror ``StepType.ARTIFACT``; that step type went away with the conveyor
    # integration (2026-08), but the vertex label stays — it marks a place on
    # the map, existing rows carry it, and the frontend renders it.
    ARTIFACT = "ARTIFACT"
    # A charging dock.
    CHARGER = "CHARGER"
    # An idle/park base the robot returns to.
    HOME = "HOME"
    # A hold spot for queueing / yielding / waiting on a station to free up.
    WAITING = "WAITING"


class MapVertexRequest(BaseModel):
    """A vertex to create. The owning map comes from the URL, not the body.

    Deliberately no ``map_name`` field: the create route is nested under
    ``/api/v1/maps/{name}/vertices``, so accepting one here would let a request
    name a different map than the path it was posted to.
    """

    name: str = Field(..., min_length=1, description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class MapVertexUpdateRequest(BaseModel):
    """Fields to change on an existing vertex. All optional; omitted ones stay.

    No ``map_name``, for the same reason the create model has none: the route is
    nested under the owning map, so a body that renamed it would move the vertex
    out from under the URL that just addressed it. Moving a vertex between maps
    is a delete and a create.
    """

    name: Optional[str] = Field(None, min_length=1, description="New vertex name.")
    type: Optional[VertexType] = Field(None, description="New vertex role.")
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


class GridOrigin(BaseModel):
    x: float = Field(..., description="World x of the grid's lower-left corner (m).")
    y: float = Field(..., description="World y of the grid's lower-left corner (m).")
    yaw: float = Field(..., description="Grid rotation in the map frame (radians).")


class GridInfoResponse(BaseModel):
    """Geometry of a stored gridmap.

    ``origin`` is the three-element origin from ``gridmap.yaml``, so its third
    component is a **yaw**, not a z. An OccupancyGrid message's origin is a full
    pose whose position is ``{x, y, z}``; the two are the same shape and are not
    interchangeable, which is worth remembering if a live-topic endpoint ever
    comes back.
    """

    resolution: float = Field(..., description="Metres per cell.")
    origin: GridOrigin = Field(..., description="Pose of the grid's lower-left cell.")
    width: int = Field(..., description="Grid width in cells.")
    height: int = Field(..., description="Grid height in cells.")


class MapSummaryResponse(BaseModel):
    name: str = Field(..., description="Directory name under the maps root.")
    active: bool = Field(..., description="Whether this is the map the stack was launched with.")
    grid: Optional[GridInfoResponse] = Field(
        None,
        description=(
            "Gridmap geometry, or null when the map has been saved from LIO but "
            "not yet converted to a 2D gridmap."
        ),
    )
    thumbnail: Optional[str] = Field(
        None,
        description=("Path of this map's thumbnail endpoint, or null when it has no grid."),
    )
    has_pointcloud: bool = Field(
        ..., description="Whether map.pcd is present (the 3D localizer's source)."
    )
    size_bytes: int = Field(..., description="Total size of the map directory.")
    modified_at: str = Field(
        ..., description="ISO 8601 timestamp of the newest file in the directory."
    )
    vertex_count: int = Field(..., description="Number of stored vertices belonging to this map.")


class CreateMapRequest(BaseModel):
    name: str = Field(
        ...,
        min_length=1,
        max_length=64,
        description=(
            "Directory name for the new map (letters, digits, dot, dash, "
            "underscore). Becomes map/<name>/ on the robot."
        ),
    )


class CreateMapResponse(BaseModel):
    name: str = Field(..., description="The map that was created.")
    has_pointcloud: bool = Field(..., description="Whether pgo wrote map.pcd (true on any 200).")
    grid_pending: bool = Field(
        ...,
        description=(
            "Whether the pcd -> gridmap conversion was started in the "
            "background. Until it finishes (or if it fails, or if false), the "
            "map lists with grid: null — run "
            "syncai_backend.helpers.traversable.build_traversable_cloud over "
            "its map.pcd by hand in that case."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class SaveGridmapResponse(BaseModel):
    name: str = Field(..., description="The map that was written.")
    etag: str = Field(
        ...,
        description=(
            "Strong ETag of the gridmap now on disk — the same value a "
            "subsequent GET of /image or /thumbnail answers with."
        ),
    )
    active: bool = Field(..., description="Whether this is the map the stack was launched with.")
    reloaded: bool = Field(
        ...,
        description=(
            "Whether the running map_server re-read the map and re-published it. "
            "False for any map that is not the active one, and for an active map "
            "whose reload failed — the save itself succeeded either way."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


# --- Helpers ----------------------------------------------------------------


def _vertex_response(vertex: MapPoint) -> MapVertexResponse:
    return MapVertexResponse(
        id=vertex.id,
        name=vertex.name,
        type=vertex.type,
        map_name=vertex.map,
        x=vertex.x,
        y=vertex.y,
        theta=vertex.theta,
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
        thumbnail=(f"/api/v1/maps/{stored.name}/thumbnail" if stored.grid is not None else None),
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


def _start_grid_conversion(
    logger: structlog.stdlib.BoundLogger, name: str, directory: str
) -> bool:
    """Kick off the pcd -> gridmap conversion in the background; report whether.

    A daemon thread rather than the handler's own thread because the conversion
    takes tens of seconds on a large site and POST /api/v1/maps must answer as
    soon as the pcd is on disk. In-process (not a subprocess) since the pipeline
    moved into helpers: the heavy passes are numpy/scipy/open3d, which release
    the GIL, so the FastAPI threadpool keeps serving while it runs. The old
    600 s subprocess timeout went with it — the helpers bound the grid size
    themselves, so the pipeline cannot run away.

    Which recipe runs is decided in here, not by the caller: pick_recipe needs
    the cloud, and reading a 20 MB pcd is not work for a request handler. The
    consequence is that the route cannot report the choice — it is in the log and
    in the sidecar instead.

    The return value only says the thread started, never that the grid appeared:
    a segmentation that rejects the whole floor fails *after* this has answered
    True and the route has 200'd, and shows up as a map that never grows a
    gridmap plus the error below. That is not new to the traversability recipe —
    bad z-bands failed the same way — but the recipe has more ways to reach it,
    so the message names the stage.
    """
    pcd_path = os.path.join(directory, "map.pcd")
    if not os.path.isfile(pcd_path):
        logger.warning("Skipping gridmap conversion: no map.pcd", map=name)
        return False

    def _run() -> None:
        bound = logger.bind(map=name)
        debug_dir = (
            os.path.join(directory, TRAVERSABLE_DEBUG_SUBDIR)
            if TRAVERSABLE_DEBUG_SUBDIR
            else None
        )
        basename = os.path.join(directory, "gridmap")
        try:
            recipe, area, floor_z = pick_recipe(bound, pcd_path)
            if recipe == "traversability":
                # Imported here, not at module scope, and that is load-bearing:
                # this would be the only module-level import of
                # helpers.traversable in the backend, and it pulls in open3d
                # (~100 MB). At module scope every backend start would pay that
                # for a conversion that runs only when an operator saves a map.
                #
                # Inside the try, not just inside the function: an ImportError
                # raised above it escapes _run entirely, and the handler below
                # never sees it. Inside the branch as well, so a small site never
                # pays the import at all.
                from syncai_backend.helpers.traversable import build_traversable_cloud

                cloud = build_traversable_cloud(
                    bound,
                    pcd_path,
                    segment=TRAVERSABLE_SEGMENT_RECIPE,
                    repair=TRAVERSABLE_REPAIR_RECIPE,
                    debug_dir=debug_dir,
                )
                convert_traversable_to_gridmap(
                    bound, cloud, basename, **TRAVERSABLE_GRID_RECIPE
                )
                params: Dict[str, object] = {
                    "segment": dict(TRAVERSABLE_SEGMENT_RECIPE),
                    "repair": dict(TRAVERSABLE_REPAIR_RECIPE),
                    "grid": dict(TRAVERSABLE_GRID_RECIPE),
                }
            else:
                bands = {
                    key: round(offset + floor_z, 3)
                    for key, offset in GRIDMAP_BANDS_ABOVE_FLOOR.items()
                }
                bound.info("z-band recipe bands", floor_z=round(floor_z, 3), **bands)
                convert_pcd_to_gridmap(bound, pcd_path, basename, **GRIDMAP_RECIPE, **bands)
                params = {**GRIDMAP_RECIPE, **bands, "floor_z": round(floor_z, 3)}
        # ValueError is a helper's own diagnosis: an empty cloud, an intensity
        # window that selected no ground, no cluster large enough to be a floor,
        # an oversized grid. OSError is the pcd or the map directory going away
        # under it. RuntimeError is open3d's channel for a cloud it cannot read.
        except (ValueError, OSError, RuntimeError) as exc:
            logger.error(
                "Gridmap conversion failed",
                map=name,
                error=str(exc),
                hint=(
                    "check the logged 'picked gridmap recipe' and, for the "
                    "traversability one, the 'ground height band' under it; then re-run "
                    "helpers.traversable.build_traversable_cloud by hand with debug_dir "
                    "set to inspect the segmentation, or convert this map with "
                    "helpers.pcd_to_gridmap.convert_pcd_to_gridmap + GRIDMAP_RECIPE "
                    "(the z-band recipe)"
                ),
            )
            return
        # Separate, and not folded into the tuple above: this one is an
        # environment fault, not a bad map. Without it the ImportError would kill
        # the thread and land as a bare traceback on stderr, where nothing
        # correlates it with the map that was being saved.
        except ImportError as exc:
            logger.error(
                "Gridmap conversion unavailable: open3d is missing",
                map=name,
                error=str(exc),
                hint="pip3 install -r src/syncai_backend/requirements.txt in the container",
            )
            return

        _write_recipe_sidecar(
            bound,
            directory,
            {
                "recipe": recipe,
                "footprint_m2": round(area, 1),
                "threshold_m2": LARGE_SITE_AREA_M2,
                "params": params,
            },
        )
        logger.info("Gridmap conversion finished", map=name, recipe=recipe)

    threading.Thread(target=_run, name=f"pcd-to-gridmap-{name}", daemon=True).start()
    return True


# --- Router -----------------------------------------------------------------


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
) -> APIRouter:

    map_router = APIRouter(prefix="", tags=["Map"])

    # --- The loaded map -----------------------------------------------------

    # Plain (non-async) handlers throughout: these walk directories, read PGM
    # headers, encode PNGs and hit psycopg2 for the vertex counts, so FastAPI
    # must run them in its worker thread pool rather than on the event loop.

    # Renderings are re-encoded only when their source file changes. Four cards
    # on the catalogue page would otherwise re-decode and re-scale four multi-
    # megabyte PGMs on every visit; the cloud cache saves a ~20 MB .pcd parse.
    # Keyed by map name, so each holds at most one entry per map on disk.
    thumbnail_cache: Dict[str, Tuple[str, bytes]] = {}
    image_cache: Dict[str, Tuple[str, bytes]] = {}
    cloud_cache: Dict[str, Tuple[Tuple[int, int], bytes]] = {}

    def _vertex_count(name: str) -> int:
        # map_vertices.map holds the bare directory name, the same spelling
        # this catalogue uses. RobotState.map, by contrast, is a path
        # ("map/dp2f/gridmap.yaml") — reconciling the two is exactly why `active`
        # is resolved on this side and not in the UI.
        return len(map_repo.list_vertices(map=name))

    def _require(name: str) -> StoredMap:
        stored = map_catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

    @map_router.get("/api/v1/maps", response_model=List[MapSummaryResponse])
    def list_maps():
        active_name = map_catalog_repo.active_name()
        return [
            _summary(stored, active_name, _vertex_count(stored.name))
            for stored in map_catalog_repo.list_maps()
        ]

    @map_router.get("/api/v1/maps/{name}", response_model=MapSummaryResponse)
    def get_map(name: str):
        stored = _require(name)
        return _summary(stored, map_catalog_repo.active_name(), _vertex_count(name))

    @map_router.post("/api/v1/maps", response_model=CreateMapResponse)
    def create_map(request: CreateMapRequest):

        directory = map_catalog_repo.create_map_dir(request.name)

        saved, detail = map_gw.save_map(directory)
        if not saved:
            map_catalog_repo.discard_empty_map_dir(request.name)
            logger.error("Failed to save map", map=request.name, error=detail)
            raise UpstreamError(detail)

        grid_pending = _start_grid_conversion(logger, request.name, directory)

        return CreateMapResponse(
            name=request.name,
            has_pointcloud=True,
            grid_pending=grid_pending,
            message=(
                f"Saved '{request.name}'."
                + (
                    " Converting to a 2D gridmap in the background."
                    if grid_pending
                    else (
                        " Convert its map.pcd with "
                        "syncai_backend.helpers.traversable to get a 2D gridmap."
                    )
                )
            ),
        )

    @map_router.get("/api/v1/maps/{name}/vertices", response_model=List[MapVertexResponse])
    def list_map_vertices(name: str, type: Optional[VertexType] = None):
        # _require first: without it an unknown map name returns [] — the same
        # answer as a real map with no vertices yet, which is the harder of the
        # two states to debug from the UI side.
        _require(name)
        vertices = map_repo.list_vertices(map=name, type=type.value if type else None)
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.post("/api/v1/maps/{name}/vertices", response_model=List[MapVertexResponse])
    def create_map_vertices(name: str, reqs: List[MapVertexRequest] = Body(..., min_length=1)):
        _require(name)
        vertices = map_repo.create_vertices(
            map=name,
            vertices=[
                {
                    "name": req.name,
                    "type": req.type.value,
                    "x": req.x,
                    "y": req.y,
                    "theta": req.theta,
                }
                for req in reqs
            ],
        )
        return [_vertex_response(vertex) for vertex in vertices]

    def _require_vertex(name: str, vertex_id: uuid.UUID) -> MapPoint:
        """Fetch a vertex, but only as a member of ``name``.

        A vertex id is unique on its own, so the map name in the URL is not
        needed to find the row — it is here to be checked. Without this a client
        could read or delete any vertex through any map's URL, and the response
        would contradict the path it came from.

        A row that exists but belongs elsewhere is a 404 rather than a 403: from
        this URL's point of view the resource genuinely is not there, and saying
        "wrong map" would confirm the id exists to a caller who addressed the
        wrong map.
        """
        _require(name)
        vertex = map_repo.get_vertex(vertex_id=vertex_id)
        if vertex is None or vertex.map != name:
            raise NotFoundError(f"Map vertex {vertex_id} was not found in '{name}'.")
        return vertex

    @map_router.get("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def get_map_vertex(name: str, id: uuid.UUID):
        return _vertex_response(_require_vertex(name, id))

    @map_router.put("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def update_map_vertex(name: str, id: uuid.UUID, req: MapVertexUpdateRequest):
        _require_vertex(name, id)

        changes = req.model_dump(exclude_unset=True)
        if "type" in changes and changes["type"] is not None:
            changes["type"] = changes["type"].value

        vertex = map_repo.update_vertex(id, **changes)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found in '{name}'.")
        return _vertex_response(vertex)

    @map_router.delete("/api/v1/maps/{name}/vertices/{id}", response_model=DeleteResponse)
    def delete_map_vertex(name: str, id: uuid.UUID):
        _require_vertex(name, id)
        map_repo.delete_vertex(vertex_id=id)
        return DeleteResponse(message=f"Map vertex {id} has been deleted.")

    def _read_gridmap(name: str) -> bytes:
        """Read the map's gridmap.pgm bytes, 404ing with the reason if absent.

        No endpoint serves these bytes as-is. /image is the wire format for the
        grid: a lossless full-size PNG of the same cells, which every browser
        can already decode, against a P5 the client would need a parser for —
        one that has to tolerate the ``#`` comment line GIMP writes.
        """
        _require(name)
        path = map_catalog_repo.gridmap_path(name)
        if path is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap. Convert its map.pcd "
                "(syncai_backend.helpers.traversable) first."
            )
        try:
            with open(path, "rb") as handle:
                return handle.read()
        except OSError as exc:
            logger.error("Failed to read gridmap", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

    def _png_response(
        name: str,
        request: Request,
        cache: Dict[str, Tuple[str, bytes]],
        render: Callable[[bytes], bytes],
        what: str,
    ) -> Response:
        """Serve a PNG rendered from the map's gridmap, cached and revalidated.

        Shared by /image and /thumbnail because the only thing that differs
        between them is the render call: both key their cache and their ETag on
        the *source* .pgm bytes rather than the encoded PNG, so a rendering that
        is a deterministic function of the gridmap changes exactly when the
        gridmap does.
        """
        payload = _read_gridmap(name)
        tag = _content_tag(payload)
        headers = {"ETag": tag, "Cache-Control": "no-cache"}

        # Before rendering, not after: a client holding the current image must not
        # cost us a PNG encode we then throw away.
        if _not_modified(request, tag):
            return Response(status_code=304, headers=headers)

        cached = cache.get(name)
        if cached is None or cached[0] != tag:
            try:
                png = render(payload)
            except ValueError as exc:
                logger.error(f"Failed to render map {what}", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable gridmap.")
            cached = (tag, png)
            cache[name] = cached

        return Response(content=cached[1], media_type="image/png", headers=headers)

    @map_router.get("/api/v1/maps/{name}/image")
    def get_map_image_by_name(name: str, request: Request):
        """The map's gridmap as a full-size PNG.

        Raw bytes, not the base64-in-JSON envelope the loaded-map endpoint uses:
        the consumer is an <img>/texture load, and base64 would cost a third
        more bytes for a value nothing reads as a string.
        """
        return _png_response(name, request, image_cache, render_png, "image")

    @map_router.get("/api/v1/maps/{name}/thumbnail")
    def get_map_thumbnail(name: str, request: Request):
        return _png_response(name, request, thumbnail_cache, render_thumbnail, "thumbnail")

    @map_router.put("/api/v1/maps/{name}/grid", response_model=SaveGridmapResponse)
    def save_map_grid(
        name: str,
        response: Response,
        payload: bytes = Body(..., media_type="application/octet-stream"),
    ):
        """Write an edited gridmap back, and reload it if it is the live one.

        The body is the cells themselves: exactly ``width * height`` bytes in
        .pgm row order (row 0 is the top of the map, max y). Raw rather than a
        PNG or base64-in-JSON because it is what the editor already holds — the
        client's buffer goes out as a memcpy and the server writes it into a P5
        body verbatim, so there is no encode, no decode, and no chance of a
        colour-managed round trip shifting 205 to 204. (The GET side had to pass
        ``colorSpaceConversion: "none"`` to stop exactly that.) The cost is ~1.6
        MB on the wire per save, on a robot LAN, once per operator edit.

        A plain ``def``, like everything else here, and that is load-bearing: this
        handler fsyncs a multi-megabyte write and then parks on
        ``MapGateway.reload_map`` for up to 25 s. On the event loop that would
        stall every other request in the process, the telemetry WebSocket
        included. ``bytes = Body(...)`` is what makes it possible — FastAPI reads
        the body in its async layer and hands the finished bytes to the
        threadpool. The alternative (``async def`` + ``await request.body()`` +
        ``run_in_threadpool`` twice) buys nothing — a ``bytes`` body parameter
        already receives the raw payload whatever the Content-Type header says;
        the ``media_type`` here is OpenAPI documentation, not enforcement — and
        needs two threadpool hops a later edit can silently drop.

        Cell *values* are deliberately not validated. map_io.cpp classifies by
        range (occupied <= 89, unknown 90..205, free >= 206) under the
        ``negate: 0 / 0.65 / 0.196`` every gridmap.yaml here carries, and two of
        the real maps already contain 255s from a round of hand-editing in GIMP.
        A ``{0, 205, 254}`` whitelist would refuse to save a map this same round
        trip just handed the client. The length is the only thing that can make a
        file map_server would misread.

        No body-size cap either: the payload is already in memory by the time
        this runs, so rejecting on Content-Length would mean streaming, and this
        whole API is unauthenticated on a robot LAN — a cap is middleware's job.
        """
        stored = _require(name)

        # stored.grid rather than a fresh read: _read_grid has already parsed the
        # .pgm header *and* gridmap.yaml, so one None test covers "no pgm", "no
        # yaml" and "torn pgm" — and it makes "the two agree" a precondition of
        # saving, which matters because map_server re-reads both a few lines down.
        if stored.grid is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap. Convert its map.pcd "
                "(syncai_backend.helpers.traversable) first."
            )

        expected = stored.grid.width * stored.grid.height
        if len(payload) != expected:
            raise BadRequestError(
                f"Gridmap body is {len(payload)} bytes; '{name}' is "
                f"{stored.grid.width}x{stored.grid.height} = {expected} cells."
            )

        written = map_catalog_repo.write_gridmap(name, payload)

        # No cache eviction here, and that is the design rather than an oversight.
        # thumbnail_cache and image_cache are in this same closure, so reaching
        # them is trivial — but _png_response re-reads the file and re-hashes it
        # *before* consulting the cache, so a stale entry can never be served.
        # Evicting would add a second place that has to remember the caches exist,
        # making /image's correctness look like it depends on this handler; and
        # popping before the write would be actively wrong, throwing away a valid
        # rendering if the write then failed. There is no thumbnail file on disk
        # to update either: /thumbnail renders from these bytes on demand.
        tag = _content_tag(written)
        response.headers["ETag"] = tag

        active = name == map_catalog_repo.active_name()
        if not active:
            return SaveGridmapResponse(
                name=name,
                etag=tag,
                active=False,
                reloaded=False,
                message=(f"Saved {name}"),
            )

        # None only if gridmap.yaml vanished since stored.grid was read — the same
        # race the repo's own isfile check covers. Handled as a failed reload
        # rather than left to hand the gateway a None it would abspath().
        yaml_path = map_catalog_repo.gridmap_yaml_path(name)
        if yaml_path is None:
            reloaded, detail = False, "gridmap.yaml is missing"
        else:
            reloaded, detail = map_gw.reload_map(yaml_path)

        if not reloaded:
            # Still a 200. The bytes are on disk and every GET now returns them,
            # so a 5xx would tell the operator the save failed when it did not —
            # and they would either press save again or re-edit a grid they
            # believe was lost. `reloaded` is the machine-readable half of the
            # answer, `message` the human one. 202/207 were considered and
            # dropped: no client here understands them, and nothing is partial or
            # queued — the request completed, one of its two effects did not.
            logger.error("Saved gridmap but map_server did not reload", map=name, error=detail)
            # `detail` rides in the message, not only in the log: the gateway
            # goes out of its way to distinguish "service not there" from
            # "map_server rejected it" from a timeout, and the operator staring
            # at a stale map is the one who needs that distinction.
            return SaveGridmapResponse(
                name=name,
                etag=tag,
                active=True,
                reloaded=False,
                message=(f"Saved '{name}', but map_server did not reload: {detail}"),
            )

        return SaveGridmapResponse(
            name=name,
            etag=tag,
            active=True,
            reloaded=True,
            message=f"Saved '{name}' and reloaded.",
        )

    @map_router.get("/api/v1/maps/{name}/pointcloud")
    def get_map_pointcloud_by_name(name: str):
        """The map's saved map.pcd, packed for the viewer.

        Wire format matches the live endpoint and the WebSocket stream: a
        little-endian uint32 point count followed by ``3 * count`` little-endian
        float32 xyz values. Decimated with the same voxel size and cap the
        point-cloud subscriber applies to ``localizer/map_cloud``, so a client
        gets a comparable cloud whichever endpoint it reads.

        Cached on the file's (size, mtime) rather than a content hash: parsing a
        ~20 MB .pcd is the expensive part and hashing it first would mean
        reading the whole file anyway. The objection that made ``_content_tag``
        hash gridmap bytes does not apply here — nothing edits map.pcd in place,
        it is only ever replaced wholesale by pgo/save_maps, which changes the
        point count and so the size.
        """
        _require(name)
        path = map_catalog_repo.pointcloud_path(name)
        if path is None:
            raise NotFoundError(f"Map '{name}' has no map.pcd.")

        try:
            stats = os.stat(path)
        except OSError as exc:
            logger.error("Failed to stat map cloud", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

        stamp = (stats.st_size, stats.st_mtime_ns)
        cached = cloud_cache.get(name)
        if cached is None or cached[0] != stamp:
            try:
                points = read_pcd_xyz(path)
            except (OSError, ValueError) as exc:
                logger.error("Failed to read map cloud", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

            points = voxel_downsample(points=points, voxel_size=MAP_CLOUD_VOXEL_SIZE)
            points = cap_points(points=points, max_points=MAP_CLOUD_MAX_POINTS)
            payload = struct.pack("<I", points.shape[0]) + pack_xyz_f32(points)
            cached = (stamp, payload)
            cloud_cache[name] = cached
            logger.info("packed stored map cloud", map=name, num_points=int(points.shape[0]))

        return Response(
            content=cached[1],
            media_type="application/octet-stream",
            headers={"Cache-Control": "no-store"},
        )

    return map_router
