"""The pcd -> gridmap conversion: a 3D point-cloud map into a 2D occupancy grid.

This is the in-process descendant of the retired ``tools/pcd_to_gridmap.py``
CLI (removed 2026-08; it lives on in git history). The map router used to
shell out to it (``sys.executable`` + ``~/robot_ws/tools/pcd_to_gridmap.py``)
after every map save, which coupled the backend to a file outside its own
package — a path that only resolved because the tool happened to be checked
out next to the install space — and meant the conversion's failure modes
arrived as a captured stderr tail instead of a Python exception.

Two of the CLI's affordances were deliberately not ported. ``--preview`` wrote
a PNG the catalogue ignores anyway (``helpers/pgm.py`` explains why).
``--stats`` printed the z histogram used to pick bands on a new site; when a
site needs that again, recover the tool from git history or eyeball the bands
off the 3D view — the backend itself always converts with the fixed recipe in
the map router.

This module holds two conversions, and they answer opposite questions.
``convert_pcd_to_gridmap`` classifies cells by height band — the recipe every
gridmap on the fleet was built with, described below.
``convert_traversable_to_gridmap`` takes an already-segmented traversable cloud
(from ``helpers/traversable.py``) and marks everything it does not cover as
occupied. They share this module because they share an output contract — one
``.pgm`` plus one ``.yaml`` under the same basename, in the pcd's own frame —
and because both are pure numpy/scipy: the segmentation pipeline that feeds the
second one needs open3d, and it stays out of here so the map router's
module-level import does not pull open3d into every backend start.

Cell classification for ``convert_pcd_to_gridmap`` (unchanged from the tool):
  occupied : >= ``min_points`` points inside the obstacle z-band [zmin, zmax]
  free     : an "observed" cell that is not occupied, where observed depends on
             ``free_mode`` — ``floor`` needs points in [floor_zmin, floor_zmax],
             ``any`` counts a point at any z, ``none`` marks nothing free
  unknown  : everything else

Failures raise ``ValueError`` (bad bands, oversized grid) or propagate the
underlying ``OSError`` — the caller decides how to report them.
"""

import os
import tempfile
from typing import Optional, Union

import numpy as np
import structlog
from scipy import ndimage

from syncai_backend.helpers.pgm import write_pgm
from syncai_backend.helpers.pointcloud import read_pcd_xyz


def _disk(radius: int) -> np.ndarray:
    """A disk-shaped structuring element for the morphological passes."""
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (xx * xx + yy * yy) <= radius * radius


def _despeckle(
    logger: structlog.stdlib.BoundLogger, grid: np.ndarray, min_obstacle_size: int
) -> np.ndarray:
    """Remove occupied blobs smaller than ``min_obstacle_size`` cells (sensor
    noise, dynamic-object残影). Real walls/racks form large connected components
    and are untouched. Removed cells become free (they sit in observed space)."""
    occ = grid == 0
    labels, _ = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    small = sizes < min_obstacle_size
    small[0] = False
    removed = small[labels]
    grid = grid.copy()
    grid[removed] = 254
    logger.info(
        "despeckle",
        removed_cells=int(removed.sum()),
        removed_blobs=int(small.sum()),
        min_obstacle_size=min_obstacle_size,
    )
    return grid


def _fill_holes(
    logger: structlog.stdlib.BoundLogger, grid: np.ndarray, max_hole_size: int
) -> np.ndarray:
    """Turn small unknown pockets fully enclosed by free space into free
    (lidar ring-gap arcs). Unknown regions touching the border or bounded by
    obstacles (e.g. inside racks) are preserved."""
    unk = grid == 205
    labels, n = ndimage.label(unk, structure=np.ones((3, 3)))
    grid = grid.copy()
    border_labels = set(
        np.unique(np.concatenate([labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]]))
    ) - {0}
    filled_cells = 0
    filled_blobs = 0
    for lab in range(1, n + 1):
        if lab in border_labels:
            continue
        mask = labels == lab
        size = int(mask.sum())
        if size > max_hole_size:
            continue
        ring = ndimage.binary_dilation(mask, np.ones((3, 3))) & ~mask
        neigh = grid[ring]
        # Fill only if the pocket is (almost) entirely surrounded by free.
        if (neigh == 0).sum() <= 0.05 * len(neigh):
            grid[mask] = 254
            filled_cells += size
            filled_blobs += 1
    logger.info(
        "fill-holes",
        filled_cells=filled_cells,
        filled_blobs=filled_blobs,
        max_hole_size=max_hole_size,
    )
    return grid


def _write_yaml_atomic(path: str, content: str) -> None:
    """Write gridmap.yaml via a temp file + rename, like ``write_pgm``.

    Same reader-race rationale: the catalogue parses gridmap.yaml on every
    ``GET /api/v1/maps``, and a listing that lands mid-write would see a torn
    file and degrade the map to ``grid: None``. Written *after* the .pgm by the
    caller, so a reader that sees the yaml always finds the pgm it names.
    """
    directory = os.path.dirname(path) or "."
    handle = tempfile.NamedTemporaryFile(
        mode="w", dir=directory, prefix=".gridmap-", suffix=".tmp", delete=False
    )
    try:
        with handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise


def _write_gridmap(
    logger: structlog.stdlib.BoundLogger,
    output_basename: str,
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
) -> None:
    """Write ``grid`` as ``<basename>.pgm`` plus its ``.yaml``, bottom row first.

    Shared by both conversions in this module so a map is indistinguishable
    whichever recipe produced it — same header, same yaml key order, same
    ``origin`` spelling. ``grid`` is indexed ``[row, col]`` with row 0 at
    **min y**; the flip to pgm order (row 0 = top = max y) happens here, in one
    place, because getting it wrong hands nav2 a mirrored map that localizes
    fine near the origin and diverges across the site.
    """
    height, width = grid.shape
    os.makedirs(os.path.dirname(output_basename) or ".", exist_ok=True)
    pgm_path = output_basename + ".pgm"
    yaml_path = output_basename + ".yaml"
    # write_pgm rather than a bare open(.., "wb"): the catalogue and map_server
    # can read a freshly converted map at any moment, and this also keeps the
    # header byte-identical to an edited-and-saved one.
    write_pgm(pgm_path, width, height, np.flipud(grid).tobytes())
    # Hand-formatted, not yaml.dump — same reason MapCatalogRepo.write_gridmap
    # never touches this file: image: must stay the relative basename.
    _write_yaml_atomic(
        yaml_path,
        f"image: {os.path.basename(pgm_path)}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_xy[0]:.6f}, {origin_xy[1]:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n",
    )
    logger.info("wrote gridmap", pgm=pgm_path, yaml=yaml_path)


def convert_pcd_to_gridmap(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    output_basename: str,
    *,
    resolution: float = 0.05,
    zmin: float = 0.0,
    zmax: float = 1.5,
    free_mode: str = "any",
    floor_zmin: Optional[float] = None,
    floor_zmax: Optional[float] = None,
    min_points: int = 2,
    min_floor_points: int = 1,
    obstacle_close: int = 0,
    free_close: int = 0,
    despeckle_min_size: Optional[int] = None,
    fill_holes_max_size: Optional[int] = None,
) -> None:
    """Convert a 3D point-cloud map into a 2D occupancy grid (pgm + yaml).

    Writes ``<output_basename>.pgm`` and ``<output_basename>.yaml``. The grid
    is expressed in the SAME frame as the pcd (the SLAM ``map`` frame), so
    localization TF lines up with the produced map without extra alignment:
    the yaml's origin is simply the grid's lower-left corner in pcd
    coordinates.

    Parameters mirror the retired CLI's flags; defaults are its defaults, not
    the router's recipe — the recipe stays at the call site where its z-band
    rationale lives. ``despeckle_min_size`` / ``fill_holes_max_size`` are None
    for off, replacing the CLI's flag-plus-size pairs.
    """
    if free_mode not in ("floor", "any", "none"):
        raise ValueError(f"unknown free_mode: {free_mode!r}")
    if free_mode == "floor" and (floor_zmin is None or floor_zmax is None):
        raise ValueError("free_mode 'floor' needs floor_zmin/floor_zmax")

    # float32, not read_pcd_xyz's float64: the pcd stores float32 fields, so
    # the wider type adds no information — and the retired CLI computed in
    # float32, so this is what kept the port bit-identical to it on the same
    # input (verified on map/dp1f: float64 shifts the origin by 1e-6 m and a
    # handful of boundary cells with it). Every gridmap on the fleet was
    # produced with float32 arithmetic; keep it that way so a re-conversion
    # reproduces the map it replaces.
    xyz = read_pcd_xyz(pcd_path).astype(np.float32)

    obst = xyz[(xyz[:, 2] >= zmin) & (xyz[:, 2] <= zmax)]
    if free_mode == "floor":
        observed = xyz[(xyz[:, 2] >= floor_zmin) & (xyz[:, 2] <= floor_zmax)]
    elif free_mode == "any":
        observed = xyz
    else:
        observed = xyz[:0]

    if len(obst) == 0:
        raise ValueError(
            "no points in obstacle z-band — check zmin/zmax against the site's floor height"
        )
    logger.info(
        "converting pcd to gridmap",
        pcd=pcd_path,
        obstacle_points=len(obst),
        observed_points=len(observed),
        free_mode=free_mode,
    )

    # Grid bounds from the union of both slices, small padding.
    used = np.vstack([obst[:, :2], observed[:, :2]]) if len(observed) else obst[:, :2]
    min_xy = used.min(axis=0) - resolution
    max_xy = used.max(axis=0) + resolution
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    # Also the reason this is safe to run in-process without the subprocess
    # timeout the router used to set: every pass below is linear-ish in the
    # cell count, and the cell count is bounded right here.
    if width * height > 200_000_000:
        raise ValueError(f"grid {width}x{height} too large — wrong bands or resolution?")
    logger.info(
        "gridmap geometry",
        width=width,
        height=height,
        resolution=resolution,
        origin_x=round(float(min_xy[0]), 3),
        origin_y=round(float(min_xy[1]), 3),
    )

    def bincount2d(pts: np.ndarray) -> np.ndarray:
        ix = ((pts[:, 0] - min_xy[0]) / resolution).astype(np.int64).clip(0, width - 1)
        iy = ((pts[:, 1] - min_xy[1]) / resolution).astype(np.int64).clip(0, height - 1)
        return np.bincount(iy * width + ix, minlength=width * height).reshape(height, width)

    obst_cnt = bincount2d(obst)
    obs_cnt = bincount2d(observed) if len(observed) else np.zeros((height, width), np.int64)

    occ_mask = obst_cnt >= min_points
    if obstacle_close > 0:
        # A pgo map.pcd is voxel-downsampled (LIO scan_resolution), so at 0.05 m
        # cells a wall is sampled as a DASHED line: on the dp1f map the median
        # hit cell holds 2 points and only half the cells hit at all. Dashes are
        # worse than a thick wall — NavFn happily threads a path through a
        # one-cell hole, so the planner returns paths straight through walls.
        # Closing (dilate then erode by the same disk) bridges gaps up to ~2*r
        # cells along the wall without thickening it, because the erode undoes
        # the dilation everywhere the gap was not filled.
        #
        # Keep r small: the same operation also seals real openings narrower
        # than ~2*r cells. At r=2 (0.05 m/px) that is 0.2 m, well below any
        # doorway the robot could drive through anyway.
        closed = ndimage.binary_closing(occ_mask, structure=_disk(obstacle_close))
        logger.info(
            "obstacle-close",
            radius=obstacle_close,
            added_cells=int(closed.sum() - occ_mask.sum()),
        )
        occ_mask = closed

    # nav2 pgm convention: 0=occupied(black), 254=free(white), 205=unknown(gray)
    grid = np.full((height, width), 205, dtype=np.uint8)
    free_mask = obs_cnt >= min_floor_points
    if free_close > 0:
        # The lidar samples the floor as sparse rings, so per-cell floor hits
        # are speckled. A morphological closing bridges those sub-radius gaps
        # into a solid drivable area WITHOUT growing the outer boundary — so
        # unknown outside the walls stays unknown.
        closed = ndimage.binary_closing(free_mask, structure=_disk(free_close))
        logger.info(
            "free-close",
            radius=free_close,
            added_cells=int(closed.sum() - free_mask.sum()),
        )
        free_mask = closed
    grid[free_mask] = 254
    grid[occ_mask] = 0

    if despeckle_min_size is not None:
        grid = _despeckle(logger, grid, despeckle_min_size)
    if fill_holes_max_size is not None:
        grid = _fill_holes(logger, grid, fill_holes_max_size)

    occ = int((grid == 0).sum())
    fre = int((grid == 254).sum())
    logger.info(
        "gridmap cells",
        occupied=occ,
        free=fre,
        unknown=width * height - occ - fre,
    )

    _write_gridmap(logger, output_basename, grid, min_xy, resolution)


def convert_traversable_to_gridmap(
    logger: structlog.stdlib.BoundLogger,
    cloud: Union[str, np.ndarray],
    output_basename: str,
    *,
    resolution: float = 0.05,
    padding: float = 1.0,
    gap_fill_size: float = 0.60,
) -> None:
    """Project an already-segmented traversable cloud into a 2D grid.

    The counterpart to ``convert_pcd_to_gridmap`` and the third stage of the
    pipeline in ``helpers/traversable.py`` (read that module's docstring for why
    there are two recipes). It inverts this one's question: rather than asking
    which cells hold an obstacle, it takes the input cloud as the definitive
    statement of where the robot may drive and marks **everything else
    occupied**.

    That inversion is why the output has no unknown cells. It is the safe
    direction — unobserved area comes out as wall rather than as free space the
    planner will route through — but it is also unforgiving: area the
    segmentation wrongly rejected becomes a wall the robot will not cross, and
    the ``padding`` ring around the map is solid black by construction. Feed
    this a repaired ground cloud, not a raw floor slice.

    ``cloud`` is a path to a pcd or an (N, 3) array — the latter is what
    ``build_traversable_cloud`` returns, and taking it keeps this module free of
    open3d.

    ``gap_fill_size`` (metres) is the one parameter a site normally needs tuned,
    and the offline tool's ReadMe says so. It is the widest hole in the input
    cloud that gets bridged, applied as a morphological closing. Too small and a
    sparsely sampled aisle reads as a field of obstacles; too large and a real
    obstacle standing in the middle of the floor gets closed over and
    disappears — which is the failure to watch for, so lower it if the grid
    loses obstacles the pcd clearly shows.
    """
    xyz = read_pcd_xyz(cloud).astype(np.float32) if isinstance(cloud, str) else cloud
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] < 2:
        raise ValueError(f"expected an (N, 3) cloud, got shape {xyz.shape}")
    if len(xyz) == 0:
        raise ValueError("traversable cloud is empty — the segmentation rejected everything")

    min_xy = xyz[:, :2].min(axis=0) - padding
    max_xy = xyz[:, :2].max(axis=0) + padding
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    if width * height > 200_000_000:
        raise ValueError(f"grid {width}x{height} too large — wrong resolution?")
    logger.info(
        "traversable gridmap geometry",
        points=len(xyz),
        width=width,
        height=height,
        resolution=resolution,
        origin_x=round(float(min_xy[0]), 3),
        origin_y=round(float(min_xy[1]), 3),
    )

    ix = ((xyz[:, 0] - min_xy[0]) / resolution).astype(np.int64).clip(0, width - 1)
    iy = ((xyz[:, 1] - min_xy[1]) / resolution).astype(np.int64).clip(0, height - 1)
    free_mask = np.zeros((height, width), dtype=bool)
    free_mask[iy, ix] = True

    if gap_fill_size > 0:
        # A disk of this radius, not the offline tool's cv2.MORPH_ELLIPSE of
        # diameter ceil(gap_fill_size/resolution): the two are the same
        # structuring element to within a cell, and _disk keeps this module on
        # scipy alone. opencv is in the backend's requirements, but only the PNG
        # encoder in helpers/pgm.py needs it and this pass has no reason to
        # widen that.
        radius = max(1, int(np.ceil(gap_fill_size / resolution)) // 2)
        closed = ndimage.binary_closing(free_mask, structure=_disk(radius))
        logger.info(
            "traversable gap fill",
            gap_fill_size=gap_fill_size,
            radius=radius,
            added_cells=int(closed.sum() - free_mask.sum()),
        )
        free_mask = closed

    # Same nav2 pgm convention as the z-band recipe, so the gridmap editor, the
    # thumbnail renderer and syncai_map_server all read this map unchanged: 0 =
    # occupied, 254 = free. 254 and not the offline tool's 255 — 255 is what
    # helpers/occupancy_grid.py renders a *free* OccupancyGrid cell as, but every
    # .pgm on this stack writes 254, and read-modify-write through the editor
    # would otherwise rewrite the values anyway.
    grid = np.zeros((height, width), dtype=np.uint8)
    grid[free_mask] = 254
    free_cells = int(free_mask.sum())
    logger.info(
        "traversable gridmap cells",
        free=free_cells,
        occupied=width * height - free_cells,
    )

    _write_gridmap(logger, output_basename, grid, min_xy, resolution)
