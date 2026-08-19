"""Convert a saved ``map.pcd`` into the 2D gridmap the nav2 map server loads.

This was ``tools/pcd_to_gridmap.py``, a standalone CLI the map router ran as a
subprocess. It is a library function now, and the script is gone. Three reasons,
in the order they matter:

* The router was the only caller left, and it reached the script through
  ``~/robot_ws/tools/pcd_to_gridmap.py`` — a path outside the installed package,
  resolved at runtime, that nothing in the build system guarantees. The backend's
  ``setup.py`` also strips ``.py`` sources on a non-symlink install, so a
  deployment that ships bytecode-only had a ``tools/`` directory that may or may
  not have travelled with it. "Tool missing" was a real, silent, grid-less state.
* ``sys.executable`` + ``subprocess`` existed only to borrow this process's
  numpy/cv2 environment. Importing the code is the direct way to say that.
* The pgm write is now ``helpers.pgm.write_pgm``, so a conversion and an editor
  save produce a byte-identical header and the same atomic replace — the
  catalogue can never see a torn grid mid-conversion.

**The frame.** The grid is expressed in the SAME frame as the pcd (the SLAM
``map`` frame), so localization TF (map -> odom) lines up with the produced map
with no extra alignment: ``origin`` in the yaml is simply the grid's lower-left
corner in pcd coordinates.

**Cell classification.**

* occupied: at least ``min_points`` points inside the obstacle z-band
  [``zmin``, ``zmax``]
* free: an "observed" cell that is not occupied, where observed depends on
  ``free_mode`` — ``floor`` needs points in [``floor_zmin``, ``floor_zmax``],
  ``any`` takes points at any z (for a lidar that cannot see the floor: the
  dense ceiling above walkable space marks the column observed), ``none`` marks
  nothing free.
* unknown: everything else.

``DEFAULT_RECIPE`` is the dp1f recipe (80 x 77 m, 1.3 M points) that produced
``map/dp1f/gridmap.pgm``, verbatim, z-bands included. Two of its choices are
worth keeping: ``free_mode="floor"`` rather than ``any``, because on an open
site ``any`` marks the outdoor ground/roof returns beyond the walls as free and
the planner routes through them; and ``min_points=2`` rather than 1, because
dropping to 1 doubles the occupied cells with noise instead of wall (bridge the
gaps with ``obstacle_close``, do not lower the threshold).

The z-bands are honest defaults, not universals: LIO's z=0 is the lidar mount
height at the mapping start pose, so they hold for this robot standing on flat
ground. A site where they do not is why ``convert_pcd_to_gridmap`` takes a
recipe at all, and why the conversion is re-runnable — nothing is destroyed by
running it again, and ``gridmap_raw.pgm`` (the editor's pre-edit backup) is not
created until the first edit.

**Lost with the CLI:** the ``--stats`` z-histogram that used to be how bands
were picked, and ``--preview`` (``helpers/pgm.py`` explains why the preview png
was already ignored by everything downstream).
"""

import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import structlog
from scipy import ndimage

from syncai_backend.helpers.pgm import write_pgm
from syncai_backend.helpers.pointcloud import read_pcd_xyz


# nav2 pgm convention, and the same three values helpers/pgm.py and the
# frontend's grid editor are built around: black obstacle, white free, grey
# unknown, read back through gridmap.yaml's negate: 0 / 0.65 / 0.196.
_OCCUPIED = 0
_UNKNOWN = 205
_FREE = 254

# A grid this size is a wrong band or a wrong resolution, not a map: 200 M cells
# is 200 MB of uint8 before the two closings, each of which allocates again.
_MAX_CELLS = 200_000_000


@dataclass(frozen=True)
class GridmapRecipe:
    """The knobs the old CLI exposed as flags, with the dp1f values as defaults.

    Frozen because the router holds one module-level instance and hands it to a
    background thread; a mutable default would be a shared, racy singleton.
    """

    # Metres per cell. 0.05 is what the costmaps run at.
    resolution: float = 0.05

    # Obstacle band, map-frame z.
    zmin: float = -0.3
    zmax: float = 1.5

    # "floor" | "any" | "none" — see the module docstring.
    free_mode: str = "floor"
    floor_zmin: float = -0.95
    floor_zmax: float = -0.25

    # Points per cell needed to call it occupied / observed.
    min_points: int = 2
    min_floor_points: int = 1

    # Morphological closing radii, in cells. 0 disables.
    obstacle_close: int = 2
    free_close: int = 5

    despeckle: bool = True
    min_obstacle_size: int = 12

    fill_holes: bool = True
    max_hole_size: int = 20000


DEFAULT_RECIPE = GridmapRecipe()


@dataclass(frozen=True)
class GridmapResult:
    """What a conversion produced, for the caller's log line.

    The router logs this and nothing else — the operator-visible signal is the
    map's card growing a thumbnail — so it carries the numbers that tell a bad
    conversion from a good one at a glance: a grid that is almost all unknown,
    or occupied cells in the hundreds, means the z-bands missed.
    """

    pgm_path: str
    yaml_path: str
    width: int
    height: int
    origin: Tuple[float, float]
    occupied: int
    free: int
    unknown: int


def _disk(radius: int) -> np.ndarray:
    """Return a filled circular structuring element of the given radius (cells)."""
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (xx * xx + yy * yy) <= radius * radius


def _despeckle(grid: np.ndarray, min_obstacle_size: int) -> Tuple[np.ndarray, int]:
    """Drop occupied blobs smaller than ``min_obstacle_size`` cells.

    Sensor noise and the smears a dynamic object leaves behind. Real walls and
    racks form large connected components and are untouched. Removed cells
    become free rather than unknown: they sit inside observed space, and leaving
    them unknown would pepper the costmap with no-go pixels.
    """
    occupied = grid == _OCCUPIED
    labels, _count = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    small = sizes < min_obstacle_size
    # Label 0 is the background; it is always "small enough" and must not be
    # swept up with the speckles.
    small[0] = False

    removed = small[labels]
    grid = grid.copy()
    grid[removed] = _FREE
    return grid, int(removed.sum())


def _fill_holes(grid: np.ndarray, max_hole_size: int) -> Tuple[np.ndarray, int]:
    """Turn small unknown pockets fully enclosed by free space into free.

    These are the arcs a lidar's ring gaps leave in an otherwise swept floor.
    Unknown regions that touch the border, or that are bounded by obstacles (the
    inside of a rack), are preserved — those are genuinely unobserved, and a
    planner is right to refuse them.
    """
    unknown = grid == _UNKNOWN
    labels, count = ndimage.label(unknown, structure=np.ones((3, 3)))
    grid = grid.copy()

    border_labels = {
        int(label)
        for label in np.unique(
            np.concatenate([labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]])
        )
    } - {0}

    filled_cells = 0
    for label in range(1, count + 1):
        if label in border_labels:
            continue
        mask = labels == label
        size = int(mask.sum())
        if size > max_hole_size:
            continue
        ring = ndimage.binary_dilation(mask, np.ones((3, 3))) & ~mask
        neighbours = grid[ring]
        # Fill only if the pocket is (almost) entirely surrounded by free space.
        if (neighbours == _OCCUPIED).sum() <= 0.05 * len(neighbours):
            grid[mask] = _FREE
            filled_cells += size

    return grid, filled_cells


def _write_yaml(path: str, image_name: str, resolution: float, origin_xy) -> None:
    """Write the map_server yaml beside the pgm.

    Hand-formatted rather than ``yaml.safe_dump``, and that spelling is
    load-bearing in two places: ``image:`` must stay a bare basename because
    map_server resolves it against the yaml's own directory, and the editor's
    save path (``MapCatalogRepo.write_gridmap``) deliberately never rewrites this
    file, so whatever shape it is written in here is the shape it keeps for the
    life of the map.

    ``origin``'s third element is a **yaw**, not a z, and is always 0.0: the grid
    is axis-aligned with the pcd's frame by construction.
    """
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(
            f"image: {image_name}\n"
            f"mode: trinary\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin_xy[0]:.6f}, {origin_xy[1]:.6f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )


def convert_pcd_to_gridmap(
    pcd_path: str,
    output_basename: str,
    recipe: GridmapRecipe = DEFAULT_RECIPE,
    logger: Optional[structlog.stdlib.BoundLogger] = None,
) -> GridmapResult:
    """Convert ``pcd_path`` into ``<output_basename>.pgm`` + ``.yaml``.

    ``output_basename`` is a path *without* extension (``map/dp1f/gridmap``),
    matching the old ``-o`` flag and what map_server expects to find.

    Raises ``ValueError`` for a pcd this recipe cannot make a map of (no points
    in the obstacle band, a grid larger than ``_MAX_CELLS``) and ``OSError`` for
    an unreadable pcd or an unwritable output directory. The caller is a
    background thread with nobody waiting on it, so both are its to log.

    **This runs minutes on a large site, in this process.** As a subprocess it
    could not touch the event loop; as a library call the GIL is shared, and
    numpy/scipy release it unevenly (the ndimage loops largely do not). The
    conversion is therefore a real, if brief, source of latency for anything else
    the backend is serving. Accepted rather than pushed into a
    ``ProcessPoolExecutor``: it happens once, immediately after a map save, in
    MANUAL mode, when the robot is parked and nothing is navigating — and a spawn
    context inside a process already running rclpy's MultiThreadedExecutor is a
    considerably worse thing to get wrong than a stuttering telemetry socket.
    """
    log = logger.bind(pcd=pcd_path) if logger is not None else None

    # The catalogue's own PCD reader rather than a second one: it is the same
    # header parse, already tolerant of the fields FAST-LIO writes, and it drops
    # the NaN rows LIO leaves in a saved map. It returns float64 where the old
    # script used float32, which is the *only* difference between this and the
    # script's output: replaying both over the two real maps, dp2f came out
    # byte-identical and dp1f differed in 3 cells of 2.4 M, where the extra
    # precision in the grid origin (~1e-6 m) moved a point across a cell
    # boundary. The wider type is the more correct one; nothing here depends on
    # reproducing the old rounding.
    xyz = read_pcd_xyz(pcd_path)
    z = xyz[:, 2]

    obstacle = xyz[(z >= recipe.zmin) & (z <= recipe.zmax)]
    if recipe.free_mode == "floor":
        observed = xyz[(z >= recipe.floor_zmin) & (z <= recipe.floor_zmax)]
    elif recipe.free_mode == "any":
        observed = xyz
    else:
        observed = xyz[:0]

    if len(obstacle) == 0:
        raise ValueError(
            f"No points in the obstacle z-band [{recipe.zmin}, {recipe.zmax}] "
            f"of {pcd_path} — wrong bands for this site."
        )

    # Grid bounds from the union of both slices, with one cell of padding so a
    # point exactly on the edge still has a cell.
    resolution = recipe.resolution
    used = (
        np.vstack([obstacle[:, :2], observed[:, :2]])
        if len(observed)
        else obstacle[:, :2]
    )
    min_xy = used.min(axis=0) - resolution
    max_xy = used.max(axis=0) + resolution
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    if width * height > _MAX_CELLS:
        raise ValueError(
            f"Grid would be {width}x{height} cells — wrong z-bands or resolution?"
        )

    def _bincount2d(points: np.ndarray) -> np.ndarray:
        ix = ((points[:, 0] - min_xy[0]) / resolution).astype(np.int64)
        iy = ((points[:, 1] - min_xy[1]) / resolution).astype(np.int64)
        ix = ix.clip(0, width - 1)
        iy = iy.clip(0, height - 1)
        return np.bincount(iy * width + ix, minlength=width * height).reshape(
            height, width
        )

    obstacle_count = _bincount2d(obstacle)
    observed_count = (
        _bincount2d(observed)
        if len(observed)
        else np.zeros((height, width), dtype=np.int64)
    )

    occupied_mask = obstacle_count >= recipe.min_points
    if recipe.obstacle_close > 0:
        # A pgo map.pcd is voxel-downsampled (LIO scan_resolution), so at 0.05 m
        # cells a wall is sampled as a DASHED line: on the dp1f map the median
        # hit cell holds 2 points and only half the wall cells are hit at all.
        # Dashes are worse than a thick wall — NavFn happily threads a path
        # through a one-cell hole, so the planner returns paths through walls.
        # Closing (dilate then erode by the same disk) bridges gaps up to ~2*r
        # cells along the wall without thickening it, because the erode undoes
        # the dilation everywhere the gap was not filled.
        #
        # Keep r small: the same operation also seals real openings narrower
        # than ~2*r cells. At r=2 and 0.05 m/px that is 0.2 m, well below any
        # doorway the robot could drive through anyway.
        occupied_mask = ndimage.binary_closing(
            occupied_mask, structure=_disk(recipe.obstacle_close)
        )

    free_mask = observed_count >= recipe.min_floor_points
    if recipe.free_close > 0:
        # The lidar samples the floor as sparse rings, so per-cell floor hits are
        # speckled. The same closing bridges those sub-radius gaps into a solid
        # drivable area WITHOUT growing the outer boundary — so unknown space
        # outside the walls stays unknown.
        free_mask = ndimage.binary_closing(
            free_mask, structure=_disk(recipe.free_close)
        )

    grid = np.full((height, width), _UNKNOWN, dtype=np.uint8)
    grid[free_mask] = _FREE
    # Occupied last: an obstacle cell that also saw floor is an obstacle.
    grid[occupied_mask] = _OCCUPIED

    if recipe.despeckle:
        grid, removed = _despeckle(grid, recipe.min_obstacle_size)
        if log is not None:
            log.info("Despeckled gridmap", removed_cells=removed)
    if recipe.fill_holes:
        grid, filled = _fill_holes(grid, recipe.max_hole_size)
        if log is not None:
            log.info("Filled gridmap holes", filled_cells=filled)

    pgm_path = output_basename + ".pgm"
    yaml_path = output_basename + ".yaml"
    os.makedirs(os.path.dirname(pgm_path) or ".", exist_ok=True)

    # Row 0 of a pgm is the TOP of the map (max y); the counts above index from
    # min y up. The flip is the whole of the difference between the two.
    write_pgm(
        path=pgm_path,
        width=width,
        height=height,
        body=np.flipud(grid).tobytes(),
    )
    # The yaml second, and never mind the window in between: a map with a pgm and
    # no yaml is exactly the ``grid: null`` state the catalogue already reports
    # for every map between save_maps and this conversion.
    _write_yaml(yaml_path, os.path.basename(pgm_path), resolution, min_xy)

    occupied = int((grid == _OCCUPIED).sum())
    free = int((grid == _FREE).sum())
    return GridmapResult(
        pgm_path=pgm_path,
        yaml_path=yaml_path,
        width=width,
        height=height,
        origin=(float(min_xy[0]), float(min_xy[1])),
        occupied=occupied,
        free=free,
        unknown=width * height - occupied - free,
    )
