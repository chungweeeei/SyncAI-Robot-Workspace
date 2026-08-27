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

Cell classification (unchanged from the tool):
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
from typing import Optional

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

    # Row 0 of a pgm is the TOP of the map (max y) -> flip.
    img = np.flipud(grid)

    os.makedirs(os.path.dirname(output_basename) or ".", exist_ok=True)
    pgm_path = output_basename + ".pgm"
    yaml_path = output_basename + ".yaml"
    # write_pgm rather than a bare open(.., "wb"): the catalogue and map_server
    # can read a freshly converted map at any moment, and this also keeps the
    # header byte-identical to an edited-and-saved one.
    write_pgm(pgm_path, width, height, img.tobytes())
    # Hand-formatted, not yaml.dump — same reason MapCatalogRepo.write_gridmap
    # never touches this file: image: must stay the relative basename.
    _write_yaml_atomic(
        yaml_path,
        f"image: {os.path.basename(pgm_path)}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{min_xy[0]:.6f}, {min_xy[1]:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n",
    )
    logger.info("wrote gridmap", pgm=pgm_path, yaml=yaml_path)
