import hashlib
import json
import os
import struct
import threading
import uuid
from datetime import datetime, timezone
from enum import Enum
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

import numpy as np
import structlog
from fastapi import APIRouter, Body, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    NotFoundError,
    UpstreamError,
)
from syncai_backend.database.models import MapPoint
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.helpers.pcd_to_gridmap import (
    convert_pcd_to_gridmap,
    convert_traversable_to_gridmap,
    floor_level,
    read_poses_xy,
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
from syncai_backend.repositories.task.task_template import TaskTemplateRepo


# Decimation for a stored map.pcd. Same numbers the point-cloud subscriber
# applies to the live localizer/map_cloud topic (see PointCloudSubscriber),
# duplicated rather than imported so this router does not depend on a
# subscriber; if one moves, move the other.
MAP_CLOUD_VOXEL_SIZE = 0.3
MAP_CLOUD_MAX_POINTS = 300000

# POST /api/v1/maps has two recipes. **Every save converts with the z-band one**
# (GRIDMAP_RECIPE below); the traversability one runs only when an operator
# explicitly asks for it through POST /api/v1/maps/{name}/grid/convert. Neither
# is a fallback for the other; they answer different questions and disagree
# about what an unknown cell means, which is exactly why the choice is recorded
# on disk rather than left implicit.
#
# **The default, z-band**, produces a genuine trinary map: walls where obstacles
# were observed, unknown where nothing was. Unknown is *recoverable* —
# costmap_layer.cpp:90 lets the obstacle layer's live observations overwrite a
# NO_INFORMATION master cell, while a LETHAL one can never be lowered, so a map
# that says "unknown" here can still grow as the robot drives and a map that
# says "wall" cannot. It is also cheap to finish by hand in the gridmap editor,
# which is how every gridmap on this fleet was actually made: dp2f still carries
# the pre-edit gridmap_raw.pgm next to the edited gridmap.pgm, and the edit
# lifted its largest connected free region from 89.1% to 94.4% of all free
# cells.
#
# **The opt-in, traversability** (helpers.traversable: segment the floor by
# lidar return intensity, surface normal and height, repair it, then project
# it), is for the site nobody hand-edits — dp1f is 6338 m² of bounding box — and
# sidesteps the z-band's real weakness: classifying a cell by *where* its points
# are in z cannot distinguish a drivable aisle from a kerb top or a ramp. What
# it costs is not small: the output has **no unknown cells**. The input cloud is
# taken as the whole of the drivable world, so every cell it does not cover
# comes out occupied — the padding ring included, and any real floor the
# segmentation wrongly rejected included. That is the safe direction for the
# planner, but it is unforgiving, and by the note above it is also permanent:
# unobserved area walled off this way can never be cleared by driving there.
#
# There used to be an automatic pick between the two, by bounding-box footprint
# against a 3000 m² threshold (LARGE_SITE_AREA_M2, removed 2026-09), and it is
# gone because it failed in the field three saves out of three: a MID360 sees
# through glass, so a 25x32 m conference hall came in at 3128-3512 m² of bbox —
# out-of-hall structure seen 2.5-6.5 m up — and was routed into the
# traversability recipe, whose no-ground-means-occupied inversion turned a
# furniture-occluded floor into a 55-59%-occupied blob with no wall geometry and
# no unknown cells. Measured floor area was considered as the replacement
# metric and rejected as the *decision* input: on this fleet it splits the six
# clouds 171-572 vs 926-1307 m², so any threshold in that gap is calibrated on
# exactly these six clouds, and the next venue (a different ceiling, glass, an
# outdoor lot) has no reason to land on the same side. What decides instead is
# the asymmetry of the failure directions: a wrongly-chosen z-band map is
# coarse but recoverable (unknown cells, hand-editable, drivable-clearable); a
# wrongly-chosen traversability map is permanently walled. So the recoverable
# recipe is the unconditional default and the unforgiving one is a decision a
# person makes. Both areas are still measured and recorded in the sidecar —
# diagnostics, not policy.
#
# The parameters stay empty because the helper's own defaults are now the tuned
# ones (its module docstring carries the measurement that moved them). A site
# that needs them changed needs them changed with the intermediate clouds in
# front of you, not guessed here — dp1f is the live example: it converts, and
# converts well, but its bottom aisle needs a wider gap_fill_size to bridge a
# doorway the floor sampling missed; the re-convert endpoint takes exactly that
# override.
TRAVERSABLE_SEGMENT_RECIPE: Dict[str, object] = {}
TRAVERSABLE_REPAIR_RECIPE: Dict[str, object] = {}
TRAVERSABLE_GRID_RECIPE: Dict[str, object] = {}

# Cell size for the measured-floor-area diagnostic in measure_cloud. 0.5 m
# because a pgo map.pcd is voxel-downsampled: at finer cells the sparse floor
# sampling under-counts (the same six clouds measure 119-998 m² at 0.2 m vs
# 171-1307 m² at 0.5 m), and a driven cell should count as floor even when only
# one return landed in it. Boundary over-count is bounded by perimeter x cell —
# ~30 m² on a 570 m² hall — noise for a diagnostic.
FLOOR_AREA_CELL_M = 0.5

# Sidecar naming the recipe a stored gridmap came from. Written next to the pgm
# because the catalogue would otherwise hold maps from two recipes with nothing
# on disk saying which produced what, and the two disagree about the meaning of
# an unknown cell — a map with no unknown cells at all is either a
# traversability output or a z-band output of a fully observed site, and there
# is no way to tell from the pgm. Tiny, so its effect on the size _walk_stats
# reports is noise.
GRIDMAP_RECIPE_SIDECAR = "gridmap.recipe.json"

# Where the segmentation's intermediate clouds go when a request asks for them
# (``debug: true`` on the re-convert endpoint), and the process-wide override:
# set the module constant to force them on for every conversion. Off by default
# because MapCatalogRepo._walk_stats recurses, so five extra clouds would triple
# the size every catalogue card reports for a map. When a site's grid comes out
# wrong — the next outdoor lot, say — the per-request flag is the tuning
# interface: convert with debug, read the intermediates out of the map
# directory, adjust, re-convert.
TRAVERSABLE_DEBUG_SUBDIR_NAME = "traversable_debug"
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


class CloudMeasure(NamedTuple):
    """What measure_cloud reads off a map.pcd before a conversion."""

    footprint_m2: float
    floor_area_m2: float
    floor_z: float


def measure_cloud(
    logger: structlog.stdlib.BoundLogger, pcd_path: str
) -> CloudMeasure:
    """Measure the cloud: bbox footprint, covered floor area, floor level.

    This is what remains of pick_recipe (removed 2026-09): the measurements
    survived the decision. The z-band conversion needs ``floor_z`` to recentre
    its bands, and the two areas go into the recipe sidecar as diagnostics —
    the module comment above GRIDMAP_RECIPE records why neither is allowed to
    *choose* the recipe any more. ``floor_area_m2`` counts FLOOR_AREA_CELL_M
    cells covered by points inside the z-band recipe's own floor band, so it
    reads as "the floor area that recipe would call observed"; footprint is the
    raw bbox, kept because comparing the two is what exposes a glass-inflated
    cloud at a glance (conference: 3512 m² of bbox over 572 m² of floor).

    The floor level here is the raw-cloud estimate. The traversability pipeline
    measures its own, from flat points, and ignores this one — the two agree to
    within 4 cm on this fleet, but the flat estimate is the better-founded of
    the two and that pipeline can afford it.

    Reads the cloud, which the conversion then reads again. That is one extra
    pass over ~20 MB inside a background thread that may be about to spend tens
    of seconds in open3d, against the alternative of threading an array through
    two conversions whose input types differ.
    """
    xyz = read_pcd_xyz(pcd_path)
    if len(xyz) == 0:
        raise ValueError(f"point cloud is empty: {pcd_path}")
    extent = xyz[:, :2].max(axis=0) - xyz[:, :2].min(axis=0)
    footprint = float(extent[0] * extent[1])
    floor_z = floor_level(xyz[:, 2].astype(np.float64))

    band = xyz[
        (xyz[:, 2] >= floor_z + GRIDMAP_BANDS_ABOVE_FLOOR["floor_zmin"])
        & (xyz[:, 2] <= floor_z + GRIDMAP_BANDS_ABOVE_FLOOR["floor_zmax"])
    ]
    if len(band):
        cells = np.unique(
            np.stack(
                [
                    np.floor(band[:, 0] / FLOOR_AREA_CELL_M).astype(np.int64),
                    np.floor(band[:, 1] / FLOOR_AREA_CELL_M).astype(np.int64),
                ],
                axis=1,
            ),
            axis=0,
        )
        floor_area = float(len(cells)) * FLOOR_AREA_CELL_M**2
    else:
        floor_area = 0.0

    logger.info(
        "measured map cloud",
        footprint_m2=round(footprint, 1),
        floor_area_m2=round(floor_area, 1),
        floor_z=round(floor_z, 3),
        points=len(xyz),
    )
    return CloudMeasure(footprint, floor_area, floor_z)


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
    grid_converting: bool = Field(
        ...,
        description=(
            "Whether a pcd -> gridmap conversion for this map is running right "
            "now. This is the conversion-status surface: there is no separate "
            "status endpoint, a client that started one polls the catalogue "
            "until this drops back to false and reads `grid` for the outcome."
        ),
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
            "map lists with grid: null — POST "
            "/api/v1/maps/{name}/grid/convert to (re)run the conversion."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class RenameMapRequest(BaseModel):
    name: str = Field(
        ...,
        min_length=1,
        max_length=64,
        description=(
            "The new directory name (letters, digits, dot, dash, underscore). "
            "map/<old>/ becomes map/<name>/ on the robot."
        ),
    )


class RenameMapResponse(BaseModel):
    old_name: str = Field(..., description="The name the map had before the rename.")
    name: str = Field(..., description="The name it has now.")
    vertices_moved: int = Field(
        ..., description="Stored vertices re-keyed from old_name to name."
    )
    templates_moved: int = Field(
        ..., description="Task templates whose map binding was re-keyed."
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class GridRecipe(str, Enum):
    """The two pcd -> gridmap recipes an operator can convert with.

    No "auto" member on purpose: the automatic pick was removed 2026-09 (the
    comment above GRIDMAP_RECIPE records the field failure), so a request either
    takes the default or names the recipe it wants.
    """

    Z_BAND = "z-band"
    TRAVERSABILITY = "traversability"


class ZBandOffsets(BaseModel):
    """Per-request overrides for the z-band recipe's bands.

    Offsets from the **measured** floor level, the same convention as
    GRIDMAP_BANDS_ABOVE_FLOOR — never absolute z, which was the per-site guess
    the offsets exist to remove. Omitted fields keep the recipe's values. The
    ranges are sanity rails, not tuning advice: a floor band 2 m off the floor
    or an obstacle band 8 m tall is a typo, not a site.
    """

    floor_zmin: Optional[float] = Field(None, ge=-2.0, le=2.0)
    floor_zmax: Optional[float] = Field(None, ge=-2.0, le=2.0)
    zmin: Optional[float] = Field(None, ge=-2.0, le=5.0)
    zmax: Optional[float] = Field(None, ge=-2.0, le=8.0)


class ConvertGridRequest(BaseModel):
    """POST /api/v1/maps/{name}/grid/convert — (re)build the 2D gridmap.

    Only ``recipe`` is surfaced in the frontend; the rest are the
    tune-with-the-intermediates-in-front-of-you parameters (curl / MCP), per the
    module comment above GRIDMAP_RECIPE.
    """

    recipe: GridRecipe = Field(
        GridRecipe.Z_BAND,
        description=(
            "Which recipe converts. z-band (the default everywhere) is trinary "
            "and recoverable; traversability is the opt-in for a site too large "
            "to hand-edit, and its output walls off everything it did not "
            "observe, permanently."
        ),
    )
    gap_fill_size: Optional[float] = Field(
        None,
        gt=0.0,
        le=3.0,
        description=(
            "Traversability only: widest hole (m) bridged in the traversable "
            "cloud. The one parameter a site normally needs tuned."
        ),
    )
    z_band_offsets: Optional[ZBandOffsets] = Field(
        None, description="z-band only: band offsets from the measured floor."
    )
    debug: bool = Field(
        False,
        description=(
            "Traversability: also write the segmentation's intermediate clouds "
            "into <map>/traversable_debug/ for tuning. They inflate the size "
            "the catalogue reports; delete the directory when done."
        ),
    )
    overwrite_edits: bool = Field(
        False,
        description=(
            "Confirm discarding hand edits. A map whose gridmap was edited "
            "refuses to re-convert without this; even with it, the edited grid "
            "survives as gridmap_prev.pgm."
        ),
    )
    reason: Optional[str] = Field(
        None,
        max_length=500,
        description="Why this recipe/override, recorded in gridmap.recipe.json.",
    )


class ConvertGridResponse(BaseModel):
    name: str = Field(..., description="The map being converted.")
    started: bool = Field(
        ...,
        description=(
            "The conversion thread started — same contract as grid_pending: the "
            "grid appears later or never. Poll the catalogue's grid_converting."
        ),
    )
    recipe: GridRecipe = Field(..., description="The recipe that is converting.")
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
        grid_converting=_is_converting(stored.name),
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


# Maps with a conversion thread currently running, guarded by the lock. Module
# scope, not the router closure: _start_grid_conversion is module-level (the
# tests drive it directly), and one backend process only ever builds one router,
# so there is nothing a per-router registry would isolate. Membership is what
# refuses a second concurrent conversion of the same map — two threads writing
# the same gridmap.pgm would interleave their outputs — and what the catalogue's
# grid_converting flag reads.
_ACTIVE_CONVERSIONS: set = set()
_ACTIVE_CONVERSIONS_LOCK = threading.Lock()


def _is_converting(name: str) -> bool:
    with _ACTIVE_CONVERSIONS_LOCK:
        return name in _ACTIVE_CONVERSIONS


def _start_grid_conversion(
    logger: structlog.stdlib.BoundLogger,
    name: str,
    directory: str,
    *,
    recipe_request: str = "z-band",
    band_offset_overrides: Optional[Dict[str, float]] = None,
    grid_overrides: Optional[Dict[str, object]] = None,
    debug: bool = False,
    override: Optional[Dict[str, object]] = None,
    archive: Optional[Callable[[], None]] = None,
    on_success: Optional[Callable[[], None]] = None,
) -> bool:
    """Kick off the pcd -> gridmap conversion in the background; report whether.

    A daemon thread rather than the handler's own thread because the conversion
    takes tens of seconds on a large site and POST /api/v1/maps must answer as
    soon as the pcd is on disk. In-process (not a subprocess) since the pipeline
    moved into helpers: the heavy passes are numpy/scipy/open3d, which release
    the GIL, so the FastAPI threadpool keeps serving while it runs. The old
    600 s subprocess timeout went with it — the helpers bound the grid size
    themselves, so the pipeline cannot run away.

    ``recipe_request`` defaults to z-band for every caller — the module comment
    above GRIDMAP_RECIPE records why there is no automatic pick any more. The
    keyword-only extras exist for the re-convert endpoint: ``band_offset_
    overrides`` merges over GRIDMAP_BANDS_ABOVE_FLOOR (still as offsets from the
    measured floor), ``grid_overrides`` over TRAVERSABLE_GRID_RECIPE
    (gap_fill_size), ``debug`` writes the segmentation's intermediate clouds
    into the map directory, ``override`` is recorded verbatim in the sidecar,
    ``archive`` runs synchronously once the slot is held (setting the previous
    grid aside — synchronous so a 409'd concurrent request can never archive a
    half-written grid), and ``on_success`` runs in the thread after the sidecar
    (the active-map reload).

    Raises ConflictError while a conversion for this map is already running.
    Acquisition happens in here, before the thread starts, precisely so there is
    no check-then-start race for the endpoint to lose.

    The return value only says the thread started, never that the grid appeared:
    a segmentation that rejects the whole floor fails *after* this has answered
    True and the route has 200'd, and shows up as a map that never grows a
    gridmap plus the error below.
    """
    pcd_path = os.path.join(directory, "map.pcd")
    if not os.path.isfile(pcd_path):
        logger.warning("Skipping gridmap conversion: no map.pcd", map=name)
        return False

    with _ACTIVE_CONVERSIONS_LOCK:
        if name in _ACTIVE_CONVERSIONS:
            raise ConflictError(
                f"A gridmap conversion for '{name}' is already running.",
                code="conversion_running",
            )
        _ACTIVE_CONVERSIONS.add(name)

    def _release() -> None:
        with _ACTIVE_CONVERSIONS_LOCK:
            _ACTIVE_CONVERSIONS.discard(name)

    if archive is not None:
        try:
            archive()
        except OSError as exc:
            _release()
            logger.error("Could not archive the gridmap", map=name, error=str(exc))
            raise UpstreamError(f"Could not set the previous gridmap aside: {exc}")

    subdir = TRAVERSABLE_DEBUG_SUBDIR or (TRAVERSABLE_DEBUG_SUBDIR_NAME if debug else None)
    debug_dir = os.path.join(directory, subdir) if subdir else None
    bands_offsets = {**GRIDMAP_BANDS_ABOVE_FLOOR, **(band_offset_overrides or {})}
    traversable_grid = {**TRAVERSABLE_GRID_RECIPE, **(grid_overrides or {})}

    def _run() -> None:
        bound = logger.bind(map=name)
        basename = os.path.join(directory, "gridmap")
        try:
            # Measured whichever recipe runs: z-band needs floor_z to place its
            # bands, and both areas go into the sidecar as diagnostics.
            measure = measure_cloud(bound, pcd_path)
            if recipe_request == "traversability":
                # Imported here, not at module scope, and that is load-bearing:
                # this would be the only module-level import of
                # helpers.traversable in the backend, and it pulls in open3d
                # (~100 MB). At module scope every backend start would pay that
                # for a conversion that runs only when an operator asks for it.
                #
                # Inside the try, not just inside the function: an ImportError
                # raised above it escapes _run entirely, and the handler below
                # never sees it. Inside the branch as well, so the default
                # z-band path never pays the import at all.
                from syncai_backend.helpers.traversable import build_traversable_cloud

                cloud = build_traversable_cloud(
                    bound,
                    pcd_path,
                    segment=TRAVERSABLE_SEGMENT_RECIPE,
                    repair=TRAVERSABLE_REPAIR_RECIPE,
                    debug_dir=debug_dir,
                )
                convert_traversable_to_gridmap(bound, cloud, basename, **traversable_grid)
                params: Dict[str, object] = {
                    "segment": dict(TRAVERSABLE_SEGMENT_RECIPE),
                    "repair": dict(TRAVERSABLE_REPAIR_RECIPE),
                    "grid": dict(traversable_grid),
                }
            else:
                bands = {
                    key: round(offset + measure.floor_z, 3)
                    for key, offset in bands_offsets.items()
                }
                bound.info(
                    "z-band recipe bands", floor_z=round(measure.floor_z, 3), **bands
                )
                # The pose-connectivity filter needs the keyframe trajectory pgo
                # writes next to the pcd. Missing or unreadable poses degrade to
                # an unfiltered conversion with a warning, never to a failed one:
                # the filter is a cleanup pass, and losing the whole gridmap to a
                # malformed poses.txt would cost far more than the glass-leak
                # speckle it removes.
                pose_xy = None
                poses_path = os.path.join(directory, "poses.txt")
                try:
                    pose_xy = read_poses_xy(poses_path)
                except (OSError, ValueError) as exc:
                    bound.warning(
                        "converting without the pose-connectivity filter",
                        poses=poses_path,
                        error=str(exc),
                    )
                pose_stats = convert_pcd_to_gridmap(
                    bound,
                    pcd_path,
                    basename,
                    **GRIDMAP_RECIPE,
                    **bands,
                    pose_seed_xy=pose_xy,
                )
                params = {
                    **GRIDMAP_RECIPE,
                    **bands,
                    "floor_z": round(measure.floor_z, 3),
                }
                if pose_stats is not None:
                    params["pose_filter"] = pose_stats
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
                    "for the traversability recipe, re-convert through "
                    "POST /api/v1/maps/{name}/grid/convert with debug: true and "
                    "read the intermediate clouds out of the map directory; the "
                    "z-band recipe is the same endpoint with recipe: 'z-band'"
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

        payload: Dict[str, object] = {
            "recipe": recipe_request,
            "footprint_m2": round(measure.footprint_m2, 1),
            "floor_area_m2": round(measure.floor_area_m2, 1),
            "params": params,
        }
        if override is not None:
            payload["recipe_override"] = override
        _write_recipe_sidecar(bound, directory, payload)
        logger.info("Gridmap conversion finished", map=name, recipe=recipe_request)

        if on_success is not None:
            # Guarded like the conversion itself: a failed active-map reload must
            # not read as a failed conversion — the grid is on disk either way.
            try:
                on_success()
            except (ValueError, OSError, RuntimeError) as exc:
                logger.error(
                    "Gridmap converted but the follow-up failed",
                    map=name,
                    error=str(exc),
                )

    def _run_and_release() -> None:
        # try/finally around the whole body: an exception nothing above caught
        # must still free the slot, or the map is unconvertible until a backend
        # restart — a wedge no log line would explain.
        try:
            _run()
        finally:
            _release()

    try:
        threading.Thread(
            target=_run_and_release, name=f"pcd-to-gridmap-{name}", daemon=True
        ).start()
    except BaseException:
        _release()
        raise
    return True


# --- Router -----------------------------------------------------------------


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    task_template_repo: TaskTemplateRepo,
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
                        " Re-run the conversion through "
                        f"POST /api/v1/maps/{request.name}/grid/convert."
                    )
                )
            ),
        )

    @map_router.patch("/api/v1/maps/{name}", response_model=RenameMapResponse)
    def rename_map(name: str, request: RenameMapRequest):
        """Rename a map: move its directory and re-key the rows that name it.

        Every refusal comes before any mutation, in this order:

        - 404 for a map that is not there.
        - 409 ``map_active`` for the map the stack is running on. This is the
          one that matters. map_server and the FAST-LIO2 localizer were both
          launched against ``map/<name>/…`` from ``[map] name`` in the instance
          INI and loaded their files during construction; the localizer cannot
          be re-pointed at runtime at all, and ``map_gw.reload_map`` would fix
          only map_server. Nothing in this backend writes the INI
          (``helpers/system_config.py`` is read-only by design), so renaming
          the directory would leave the running stack holding a path that no
          longer exists and leave ``active_name()`` naming a map the catalogue
          no longer lists — every card would read ``active: false``. Switching
          maps is an INI edit plus a stack restart, and only after that does
          the old name become renameable. The UI greys the control for the
          active map; this is what makes that more than a suggestion.
        - 409 ``conversion_running`` while a gridmap conversion is in flight.
          The conversion thread closed over the old directory path when it
          started, so a rename under it would make it die with ``OSError`` and
          leave ``grid_converting`` reading true for a name that is gone.
        - 400 / 409 ``name_taken`` from ``rename_map_dir`` for a bad or
          already-used new name.

        Filesystem first, database second. The directory move is the step most
        likely to fail (target exists, permissions), and failing there needs no
        compensation. The two repos run separate sessions, so there is no single
        transaction to lean on for the DB half; if either UPDATE fails the
        directory is moved back, because a renamed map whose vertices still
        answer to the old name is worse than a rename that did not happen.
        """
        _require(name)

        active_name = map_catalog_repo.active_name()
        if name == active_name:
            raise ConflictError(
                f"'{name}' is the map the stack is running on and cannot be "
                "renamed while it is in use. Switch the robot to another map "
                "and restart the stack first.",
                code="map_active",
            )
        if _is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; rename it "
                "once the conversion has finished.",
                code="conversion_running",
            )

        new_dir = map_catalog_repo.rename_map_dir(name, request.name)

        try:
            vertices_moved = map_repo.move_vertices(name, request.name)
            templates_moved = task_template_repo.rebind_map(name, request.name)
        except Exception as exc:
            # Best-effort compensation. A second failure here is logged and
            # reported, not raised over the first: the operator needs the
            # sentence about the database, and the log needs the path state.
            old_dir = os.path.join(os.path.dirname(new_dir), name)
            try:
                os.rename(new_dir, old_dir)
                restored = True
            except OSError as undo_exc:
                restored = False
                logger.error(
                    "Could not move the map directory back after a failed rename",
                    map=name,
                    new_name=request.name,
                    error=str(undo_exc),
                )
            logger.error(
                "Map rename failed while re-keying database rows",
                map=name,
                new_name=request.name,
                directory_restored=restored,
                error=str(exc),
            )
            raise UpstreamError(
                f"Could not re-key the rows that name '{name}': {exc}. "
                + (
                    "The map directory was left under its old name."
                    if restored
                    else f"The directory is now map/{request.name}/ but the "
                    f"database still says '{name}' — fix by hand."
                )
            )

        # The renderings are keyed by name. A stale entry under the old name
        # would never be *served* (_png_response re-hashes the file before
        # consulting the cache) but it would sit there forever; drop it.
        thumbnail_cache.pop(name, None)
        image_cache.pop(name, None)
        cloud_cache.pop(name, None)

        logger.info(
            "Renamed map",
            map=name,
            new_name=request.name,
            vertices_moved=vertices_moved,
            templates_moved=templates_moved,
        )
        message = (
            f"Renamed '{name}' to '{request.name}'. Moved {vertices_moved} "
            f"{'vertex' if vertices_moved == 1 else 'vertices'} and "
            f"{templates_moved} task {'template' if templates_moved == 1 else 'templates'}."
        )
        if templates_moved:
            # The schedule memo is frozen at registration and is a display
            # label only (see routers/schedule.py); nothing here re-registers.
            message += " Schedules already registered keep the old map label."
        return RenameMapResponse(
            old_name=name,
            name=request.name,
            vertices_moved=vertices_moved,
            templates_moved=templates_moved,
            message=message,
        )

    @map_router.post(
        "/api/v1/maps/{name}/grid/convert", response_model=ConvertGridResponse
    )
    def convert_map_grid(name: str, request: ConvertGridRequest):
        """(Re)build a map's 2D gridmap from its map.pcd, in the background.

        This is the formal home of what used to be done with a one-off script
        that never made it into the repo: choosing the recipe when the default
        is wrong for the site (a warehouse too large to hand-edit wants
        traversability), re-converting after a parameter override, and getting
        the traversability pipeline's intermediate clouds for tuning a site the
        defaults cannot handle — the next outdoor venue tunes from those, not
        from constants guessed in advance.

        Two 409s can come back and the client must tell them apart (the `code`
        field): `conversion_running` means try later; `gridmap_hand_edited`
        means the current grid holds operator edits and the caller has to
        confirm with `overwrite_edits` — even then the edited grid survives as
        gridmap_prev.pgm.
        """
        stored = _require(name)
        if not stored.has_pointcloud:
            raise BadRequestError(
                f"Map '{name}' has no map.pcd — there is nothing to convert."
            )

        # Cross-field validation as readable 400s rather than schema 422s: which
        # parameter belongs to which recipe is domain knowledge, and the FastAPI
        # 422 for it would name a field, not the mismatch.
        if request.recipe is GridRecipe.Z_BAND and request.gap_fill_size is not None:
            raise BadRequestError(
                "gap_fill_size tunes the traversability recipe; the z-band "
                "recipe takes z_band_offsets."
            )
        if (
            request.recipe is GridRecipe.TRAVERSABILITY
            and request.z_band_offsets is not None
        ):
            raise BadRequestError(
                "z_band_offsets tune the z-band recipe; the traversability "
                "recipe takes gap_fill_size."
            )

        band_overrides: Optional[Dict[str, float]] = None
        if request.z_band_offsets is not None:
            band_overrides = request.z_band_offsets.model_dump(exclude_none=True)
            merged = {**GRIDMAP_BANDS_ABOVE_FLOOR, **band_overrides}
            # Ordering is checked on the *merged* bands: a request overriding one
            # end of a band can invert it against the recipe's other end, and
            # that inversion selects no points and fails half a minute later in
            # the thread, where nothing answers the operator.
            if merged["floor_zmin"] >= merged["floor_zmax"]:
                raise BadRequestError(
                    f"floor band is inverted: floor_zmin {merged['floor_zmin']} "
                    f">= floor_zmax {merged['floor_zmax']}."
                )
            if merged["zmin"] >= merged["zmax"]:
                raise BadRequestError(
                    f"obstacle band is inverted: zmin {merged['zmin']} >= "
                    f"zmax {merged['zmax']}."
                )

        if map_catalog_repo.gridmap_edited(name) and not request.overwrite_edits:
            raise ConflictError(
                f"'{name}' has a hand-edited gridmap. Re-converting replaces it "
                "(the edited grid is kept as gridmap_prev.pgm) — pass "
                "overwrite_edits to proceed.",
                code="gridmap_hand_edited",
            )

        grid_overrides: Optional[Dict[str, object]] = (
            {"gap_fill_size": request.gap_fill_size}
            if request.gap_fill_size is not None
            else None
        )

        param_overrides: Dict[str, object] = {}
        if band_overrides:
            param_overrides["z_band_offsets"] = band_overrides
        if grid_overrides:
            param_overrides.update(grid_overrides)
        override: Dict[str, object] = {
            "requested": request.recipe.value,
            "picked_by": f"POST /api/v1/maps/{name}/grid/convert",
            "reason": request.reason,
            "at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "param_overrides": param_overrides,
        }

        # An active map that re-converts must reach map_server, or it keeps
        # serving the grid the operator just replaced — the same reload the
        # gridmap-editor save does, deferred into the thread because the grid
        # does not exist yet when this handler answers.
        on_success: Optional[Callable[[], None]] = None
        if name == map_catalog_repo.active_name():

            def _reload() -> None:
                yaml_path = map_catalog_repo.gridmap_yaml_path(name)
                if yaml_path is None:
                    logger.error(
                        "Converted the active map but gridmap.yaml is missing",
                        map=name,
                    )
                    return
                reloaded, detail = map_gw.reload_map(yaml_path)
                if not reloaded:
                    logger.error(
                        "Converted the active map but map_server did not reload",
                        map=name,
                        error=detail,
                    )

            on_success = _reload

        started = _start_grid_conversion(
            logger,
            name,
            map_catalog_repo.resolve_dir(name),
            recipe_request=request.recipe.value,
            band_offset_overrides=band_overrides,
            grid_overrides=grid_overrides,
            debug=request.debug,
            override=override,
            archive=lambda: map_catalog_repo.archive_gridmap(name),
            on_success=on_success,
        )
        # started=False cannot happen past the has_pointcloud gate above short of
        # a race deleting map.pcd; report it honestly rather than asserting.
        return ConvertGridResponse(
            name=name,
            started=started,
            recipe=request.recipe,
            message=(
                (
                    f"Converting '{name}' with the {request.recipe.value} recipe "
                    "in the background."
                    + (
                        " The previous gridmap is kept as gridmap_prev.pgm."
                        if stored.grid is not None
                        else ""
                    )
                )
                if started
                else f"map.pcd for '{name}' disappeared before the conversion started."
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
                f"Map '{name}' has no gridmap. Convert its map.pcd first "
                f"(POST /api/v1/maps/{name}/grid/convert)."
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
                f"Map '{name}' has no gridmap. Convert its map.pcd first "
                f"(POST /api/v1/maps/{name}/grid/convert)."
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
