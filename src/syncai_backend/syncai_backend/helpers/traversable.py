"""Intensity-based traversability segmentation and ground repair.

The second pcd → grid recipe on this stack, and a deliberately different one
from ``helpers/pcd_to_gridmap.py``. That module slices the cloud by height: a
cell is occupied when enough points sit in the obstacle z-band, free when the
floor band was observed. It is cheap, it is what every gridmap on the fleet was
built with, and it fails in exactly one way — it has no idea whether the
"floor" it observed is *drivable*. A ramp, a 25 cm kerb top and the flat aisle
all land in the same band on a site that is not perfectly level, and z=0 in a
LIO map is the lidar mount height at the mapping start pose, so the bands are
a per-site guess to begin with.

This module segments the cloud by what the surface *is* instead of where it is,
using the pipeline developed offline in ``SyncAI-Robot-Pointcloud-
ProcessAndConversion`` (three standalone scripts, ported here so the backend can
run it without a checkout next to the install space — the same motivation that
folded the retired ``tools/pcd_to_gridmap.py`` CLI into a helper):

  1. ``segment_traversable``  — split the cloud into flat ground / ramp /
     obstacle using lidar return intensity plus the surface normal's z.
  2. ``repair_ground``        — a LIO map samples a continuous floor with holes
     and dashes; merge the coplanar fragments back into whole planes, fill
     their interiors, and stitch across gaps and single steps.
  3. ``convert_traversable_to_gridmap`` (in ``pcd_to_gridmap.py``) — project the
     repaired ground and treat "no traversable point here" as occupied.

Stage 3 lives in the other module on purpose: it is pure numpy/scipy, and
keeping it there means ``pcd_to_gridmap`` stays free of open3d. The map router
imports that module at startup, and open3d is a ~100 MB import — paying it on
every backend start, on a robot that mostly never converts a map, is not worth
it. **Nothing imports this module at startup.** Import it inside the function
that needs it.

Not wired into ``POST /api/v1/maps``. That route still converts with
``GRIDMAP_RECIPE``, and switching the fleet's default recipe is a separate
decision from having this available — the trinary z-band map is what
``syncai_map_server``, the gridmap editor and every stored ``gridmap_raw.pgm``
were validated against. Run this by hand on a site where the z-bands do not
hold:

    from syncai_backend.helpers.traversable import build_traversable_cloud
    from syncai_backend.helpers.pcd_to_gridmap import convert_traversable_to_gridmap
    cloud = build_traversable_cloud(logger, "map/dp2f/map.pcd",
                                    debug_dir="map/dp2f/traversable_debug")
    convert_traversable_to_gridmap(logger, cloud, "map/dp2f/gridmap")

Every default below is the value the offline tool shipped with, which for stage
2 means the ``__main__`` block's arguments rather than the function signature's
— the two disagreed (0.02/0.35/0.15/0.22 vs 0.04/0.80/0.07/0.31) and the
ReadMe's tuning guide documents the ``__main__`` set as the recommended one.
Failures raise ``ValueError``; the caller decides how to report them.
"""

import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import open3d as o3d
import structlog
from scipy import ndimage
from scipy.spatial import Delaunay, QhullError, cKDTree


# Field names a PCD may carry the lidar return intensity under. FAST-LIO writes
# "intensity"; the other three are what other exporters on the fleet have used.
_INTENSITY_KEYS = ("intensity", "scalar_intensity", "i", "reflectivity")


@dataclass
class SegmentedCloud:
    """What ``segment_traversable`` split the map into.

    ``ground`` is the only surface stage 2 repairs and stage 3 projects.
    ``slope`` and the two obstacle clouds are diagnostics — write them out with
    ``debug_dir`` and open them in a viewer when a site's segmentation looks
    wrong, which is what the offline tool's four output PCDs were for.
    """

    ground: o3d.geometry.PointCloud
    slope: o3d.geometry.PointCloud
    obstacle: o3d.geometry.PointCloud
    obstacle_raw: o3d.geometry.PointCloud


def _load_cloud(
    logger: structlog.stdlib.BoundLogger, pcd_path: str, normal_radius: float, normal_max_nn: int
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, np.ndarray]:
    """Read a PCD and return it with per-point normals and intensity.

    Normals are estimated when the file has none (FAST-LIO's ``map.pcd`` is
    ``x y z intensity``, so that is the normal case) and are estimated **on the
    returned cloud**, not on a throwaway copy. The offline tool estimated them
    on one ``to_legacy()`` result and returned a second, fresh one; the returned
    cloud therefore had no normals, and the obstacle-cleaning pass — which reads
    ``np.asarray(pcd.normals)`` off a subset of it — silently got a (0, 3) array,
    an all-empty ceiling mask, and produced an **empty obstacle cloud** on every
    intensity-only map. It never raised, so it looked like the map simply had no
    walls.
    """
    tensor_pcd = o3d.t.io.read_point_cloud(pcd_path)
    pcd = tensor_pcd.to_legacy()
    points = np.asarray(pcd.points)
    if len(points) == 0:
        raise ValueError(f"point cloud is empty: {pcd_path}")

    has_normals = pcd.has_normals() and np.any(np.asarray(pcd.normals) != 0)
    if not has_normals:
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn)
        )
        # Normals from PCA are sign-ambiguous, and every threshold below is on
        # the *signed* z component (ground is nz≈+1, a ceiling is nz≈-1), so
        # without this the flat floor would split roughly in half.
        pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))
    normals = np.asarray(pcd.normals)

    intensity = None
    for key in _INTENSITY_KEYS:
        if key in tensor_pcd.point:
            intensity = tensor_pcd.point[key].numpy().reshape(-1).astype(np.float64)
            break
    if intensity is None:
        # Not fatal: with no intensity the percentile gate opens fully and the
        # segmentation degrades to a normals-only one, which still separates
        # floor from wall. Worth a warning because the intensity gate is what
        # rejects painted markings and wet patches.
        logger.warning("pcd carries no intensity field; segmenting on normals only", pcd=pcd_path)
        intensity = np.zeros(len(points))

    logger.info(
        "loaded cloud for segmentation",
        pcd=pcd_path,
        points=len(points),
        normals_estimated=not has_normals,
        has_intensity=bool(np.any(intensity)),
    )
    return pcd, normals, intensity


def _clean_ground(
    logger: structlog.stdlib.BoundLogger,
    ground: o3d.geometry.PointCloud,
    *,
    ror_nb_points: int,
    ror_radius: float,
    dbscan_eps: float,
    dbscan_min_points: int,
    min_cluster_points: int,
) -> o3d.geometry.PointCloud:
    """Drop sparse flyers and disconnected crumbs from the raw ground slice.

    Two passes, and they reject different things. The radius filter is a local
    *density* test — it removes the handful of points a single bad return leaves
    floating in the middle of nowhere. DBSCAN then removes whole islands that
    are dense but too small to be floor (a table top caught by the intensity
    gate, a patch of ceiling that survived).

    ``min_cluster_points`` keeps **every** cluster above the threshold rather
    than only the largest. A site with two aisles joined by a doorway the lidar
    never saw through is two clusters, and keeping only the biggest would delete
    half the map.
    """
    if len(ground.points) == 0:
        raise ValueError(
            "intensity/normal gate selected no ground points — check the intensity "
            "percentile window and ground_nz against this site's cloud"
        )

    _, keep = ground.remove_radius_outlier(nb_points=ror_nb_points, radius=ror_radius)
    dense = ground.select_by_index(keep)
    logger.info(
        "ground radius-outlier filter",
        before=len(ground.points),
        after=len(dense.points),
        nb_points=ror_nb_points,
        radius=ror_radius,
    )
    if len(dense.points) == 0:
        return dense

    labels = np.asarray(
        dense.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_min_points, print_progress=False)
    )
    if labels.size == 0 or labels.max() < 0:
        # Every point is noise to DBSCAN. Returning the density-filtered cloud
        # is the useful failure: the caller still gets a floor, just an
        # unfiltered one, and the log says the clustering did nothing.
        logger.warning("ground DBSCAN found no cluster; keeping the density-filtered cloud")
        return dense

    counts = np.bincount(labels[labels >= 0])
    valid = np.flatnonzero(counts >= min_cluster_points)
    if valid.size == 0:
        raise ValueError(
            f"no ground cluster reaches min_cluster_points={min_cluster_points} "
            f"(largest is {int(counts.max())}) — lower it or widen dbscan_eps"
        )
    final = dense.select_by_index(np.flatnonzero(np.isin(labels, valid)))
    logger.info(
        "ground DBSCAN",
        clusters=int(counts.size),
        kept_clusters=int(valid.size),
        after=len(final.points),
        min_cluster_points=min_cluster_points,
    )
    return final


def _clean_obstacles(
    logger: structlog.stdlib.BoundLogger,
    obstacles: o3d.geometry.PointCloud,
    *,
    ceiling_nz: float,
    scatter_knn: int,
    max_scattering: float,
    ror_nb_points: int,
    ror_radius: float,
) -> o3d.geometry.PointCloud:
    """Reduce everything-that-is-not-floor to the solid parts a robot can hit.

    Three passes, each aimed at one thing the LIO map contains that is not an
    obstacle at the robot's height:

    1. **Ceilings.** ``ceiling_nz`` is itself negative (-0.2), so the cut keeps
       walls (nz≈0) and every upward-facing surface, and drops only the clearly
       downward-facing ones: overhead structure, beams, the underside of racking.
    2. **Foliage.** The local PCA scattering ratio λ3/Σλ is near 0 for a plane
       and large for a volume of scattered returns, so a tree canopy or its
       shadow scores far above a wall or a sign. This is the only pass that
       needs neighbourhood covariances, and it is the expensive one.
    3. **Motion residue.** A person walking through the mapping run leaves a
       sparse ribbon of points; a radius filter with a generous radius removes
       it along with flyers.

    Returned empty rather than raising when nothing survives — a corridor map
    genuinely can be all floor, and this cloud is diagnostic anyway.
    """
    if len(obstacles.points) == 0:
        return obstacles

    normals = np.asarray(obstacles.normals)
    kept = obstacles.select_by_index(np.flatnonzero(normals[:, 2] > ceiling_nz))
    logger.info("obstacle ceiling cut", after=len(kept.points), ceiling_nz=ceiling_nz)
    if len(kept.points) == 0:
        return kept

    kept.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(knn=scatter_knn))
    # eigvalsh returns ascending eigenvalues, so column 0 is the smallest —
    # the plane-normal direction, near zero for a flat surface.
    eigenvalues = np.linalg.eigvalsh(np.asarray(kept.covariances))
    scattering = eigenvalues[:, 0] / (eigenvalues.sum(axis=1) + 1e-8)
    kept = kept.select_by_index(np.flatnonzero(scattering < max_scattering))
    logger.info("obstacle foliage cut", after=len(kept.points), max_scattering=max_scattering)
    if len(kept.points) == 0:
        return kept

    _, keep = kept.remove_radius_outlier(nb_points=ror_nb_points, radius=ror_radius)
    kept = kept.select_by_index(keep)
    logger.info("obstacle radius-outlier filter", after=len(kept.points), radius=ror_radius)
    return kept


def segment_traversable(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    *,
    intensity_percentiles: Tuple[float, float] = (8.0, 60.0),
    ground_nz: float = 0.99,
    slope_nz: Tuple[float, float] = (0.5, 0.75),
    normal_radius: float = 0.5,
    normal_max_nn: int = 30,
    ground_ror_nb_points: int = 3,
    ground_ror_radius: float = 0.05,
    ground_dbscan_eps: float = 0.20,
    ground_dbscan_min_points: int = 20,
    ground_min_cluster_points: int = 1000,
    obstacle_ceiling_nz: float = -0.2,
    obstacle_scatter_knn: int = 25,
    obstacle_max_scattering: float = 0.06,
    obstacle_ror_nb_points: int = 12,
    obstacle_ror_radius: float = 0.4,
) -> SegmentedCloud:
    """Split a LIO map into traversable ground, ramps and obstacles.

    The gate is intensity **and** normal direction, and it needs both. Intensity
    alone cannot tell a floor from a same-material wall; the normal alone
    accepts every flat surface in the building, table tops and pallet tops
    included. Together they select "flat, and made of what the floor is made
    of".

    ``intensity_percentiles`` is a window over the *non-zero* intensities of
    this cloud, not an absolute range: return intensity depends on the lidar
    model, the surface and the range, so a fixed [5, 100] that fits one site
    fits no other. The floor is by far the largest surface in a driven map, so
    it dominates the histogram's bulk — the window's job is only to cut the
    zero-return noise below it and the retroreflective signage above it.

    ``ground_nz`` is deliberately severe (nz ≥ 0.99, i.e. within ~8° of level):
    stage 2 is what recovers the area this rejects, and letting a 20° surface
    into the "flat" class would have stage 2 stitch across a slope as if it were
    a step. Ramps come back separately in ``slope`` (``slope_nz`` covers roughly
    40°–60° off level) and are **not** merged into ``ground`` — the planar nav
    stack has no notion of a gradient, so a ramp projected into the grid reads
    as ordinary free space the controller will drive at full speed.
    """
    pcd, normals, intensity = _load_cloud(logger, pcd_path, normal_radius, normal_max_nn)
    nz = normals[:, 2]

    non_zero = intensity[intensity > 0]
    if non_zero.size:
        i_min, i_max = np.percentile(non_zero, list(intensity_percentiles))
        intensity_mask = (intensity >= i_min) & (intensity <= i_max)
        logger.info(
            "intensity window",
            i_min=round(float(i_min), 2),
            i_max=round(float(i_max), 2),
            percentiles=list(intensity_percentiles),
            passed=int(intensity_mask.sum()),
        )
    else:
        intensity_mask = np.ones(len(nz), dtype=bool)

    ground_mask = intensity_mask & (nz >= ground_nz)
    slope_mask = intensity_mask & (nz >= slope_nz[0]) & (nz < slope_nz[1])
    # Anything the two gates did not claim, *before* either was denoised — the
    # points a filter drops are noise, not obstacles, and must not reappear here.
    other_mask = ~(ground_mask | slope_mask)

    obstacle_raw = pcd.select_by_index(np.flatnonzero(other_mask))
    result = SegmentedCloud(
        ground=_clean_ground(
            logger,
            pcd.select_by_index(np.flatnonzero(ground_mask)),
            ror_nb_points=ground_ror_nb_points,
            ror_radius=ground_ror_radius,
            dbscan_eps=ground_dbscan_eps,
            dbscan_min_points=ground_dbscan_min_points,
            min_cluster_points=ground_min_cluster_points,
        ),
        slope=pcd.select_by_index(np.flatnonzero(slope_mask)),
        obstacle=_clean_obstacles(
            logger,
            obstacle_raw,
            ceiling_nz=obstacle_ceiling_nz,
            scatter_knn=obstacle_scatter_knn,
            max_scattering=obstacle_max_scattering,
            ror_nb_points=obstacle_ror_nb_points,
            ror_radius=obstacle_ror_radius,
        ),
        obstacle_raw=obstacle_raw,
    )
    logger.info(
        "segmentation done",
        ground=len(result.ground.points),
        slope=len(result.slope.points),
        obstacle=len(result.obstacle.points),
    )
    return result


def _find_root(parent: dict, node: int) -> int:
    """Union-find root with path compression, iteratively.

    Iterative because the offline tool's recursive version recurses once per
    link on the path and a fragmented floor produces thousands of fragments —
    deep enough to hit CPython's recursion limit inside a background conversion
    thread, where the RecursionError would surface only as a log line.
    """
    root = node
    while parent[root] != root:
        root = parent[root]
    while parent[node] != root:
        parent[node], node = root, parent[node]
    return root


def _row_stride(points: np.ndarray, origin: np.ndarray, grid_res: float, dilation: int) -> int:
    """The row pitch that packs a (cell x, cell y) pair into one int64 key.

    Derived from the extent of **all** the points being compared, not of one
    fragment: two fragments' keys mean the same cell only if they were packed
    with the same stride, so a per-fragment stride silently makes the overlap
    test below compare unrelated numbers. It reads as working, because two
    fragments that happen to span the same rows do get the same stride.
    """
    rows = int((points[:, 1].max() - origin[1]) / grid_res)
    # A cell's dilated key reaches iy + 2 * dilation (see _footprint_keys).
    return rows + 2 * dilation + 1


def _footprint_keys(
    points: np.ndarray, origin: np.ndarray, grid_res: float, dilation: int, stride: int
) -> np.ndarray:
    """The cluster's 2D cell footprint, dilated, as a sorted array of int64 keys.

    Dilating by a couple of cells is what lets two fragments that merely *touch*
    count as overlapping — LIO splits one physical floor into fragments whose
    edges interleave without sharing a cell.

    Keys rather than the offline tool's ``set`` of ``(x, y)`` tuples: the
    dilation there was a 25-iteration Python loop over every cell of every
    cluster, and the overlap test that follows is O(clusters²). Packing (x, y)
    into one int64 makes the dilation a single broadcast and the test an
    ``intersect1d``.
    """
    ix = ((points[:, 0] - origin[0]) / grid_res).astype(np.int64)
    iy = ((points[:, 1] - origin[1]) / grid_res).astype(np.int64)

    offsets = np.arange(-dilation, dilation + 1, dtype=np.int64)
    dx, dy = (mesh.ravel() for mesh in np.meshgrid(offsets, offsets, indexing="ij"))
    # +dilation keeps both indices non-negative (they start at 0, so the -d
    # offset can only reach -d), which is what makes the packing one-to-one.
    kx = (ix[:, None] + dx[None, :]).ravel() + dilation
    ky = (iy[:, None] + dy[None, :]).ravel() + dilation
    return np.unique(kx * stride + ky)


def _merge_coplanar(
    logger: structlog.stdlib.BoundLogger,
    points: np.ndarray,
    labels: np.ndarray,
    fragments: list,
    *,
    grid_res: float,
    same_plane_z: float,
    dilation: int,
) -> list:
    """Group DBSCAN fragments that are the same physical plane.

    Two fragments belong together when their 2D footprints touch *and* they sit
    at the same height. Both halves matter: footprints alone would merge a step
    with the floor it overhangs, height alone would merge two unconnected aisles
    that happen to be level. Transitivity comes from union-find, so a floor
    broken into a chain of fragments reassembles even though the ends of the
    chain never touch.

    Height is the **median** z, not the mean — a fragment with a few points
    smeared onto the wall it meets would otherwise read as raised.
    """
    footprints = {}
    heights = {}
    origin = points.min(axis=0)
    stride = _row_stride(points, origin, grid_res, dilation)
    for fragment in fragments:
        pts = points[labels == fragment]
        heights[fragment] = float(np.median(pts[:, 2]))
        footprints[fragment] = _footprint_keys(pts, origin, grid_res, dilation, stride)

    parent = {fragment: fragment for fragment in fragments}
    for i, first in enumerate(fragments):
        for second in fragments[i + 1:]:
            if abs(heights[first] - heights[second]) >= same_plane_z:
                continue
            if not np.intersect1d(
                footprints[first], footprints[second], assume_unique=True
            ).size:
                continue
            root_a, root_b = _find_root(parent, first), _find_root(parent, second)
            if root_a != root_b:
                parent[root_b] = root_a

    groups = {}
    for fragment in fragments:
        groups.setdefault(_find_root(parent, fragment), []).append(fragment)
    logger.info("merged coplanar fragments", fragments=len(fragments), planes=len(groups))
    return list(groups.values())


def _fill_and_outline(
    points: np.ndarray, grid_res: float, max_cells: int
) -> Tuple[np.ndarray, np.ndarray]:
    """Fill a plane's interior holes and return (filled points, boundary points).

    A LIO floor is a voxel-downsampled surface, so it arrives as a speckled
    sheet with holes wherever the lidar's rings missed. ``binary_fill_holes``
    closes every pocket fully enclosed by the plane — which is exactly the
    conservative choice here, because a pocket that reaches the plane's edge
    (i.e. is not enclosed) might be a real obstacle footprint and is left alone.

    Both outputs are flattened onto the plane's mean height. That is a
    simplification the offline tool made and it is safe *because* this only ever
    runs on fragments already within ``same_plane_z`` of each other; the
    boundary z is then what the stitching pass compares against, so keeping it
    per-point would make the step test depend on which edge point was sampled.
    """
    min_p = points.min(axis=0)
    span = points.max(axis=0) - min_p
    cols = int(span[0] / grid_res) + 3
    rows = int(span[1] / grid_res) + 3
    if cols * rows > max_cells:
        raise ValueError(
            f"plane rasterises to {cols}x{rows} cells at grid_res={grid_res} — "
            "raise grid_res or check the segmentation for a plane spanning the whole site"
        )

    # +1 leaves a one-cell empty margin, so a plane touching its own bounding
    # box still has background on all four sides: binary_fill_holes treats the
    # array edge as background, and binary_erosion would otherwise clip the
    # boundary ring the stitching pass needs.
    ix = ((points[:, 0] - min_p[0]) / grid_res).astype(np.int64) + 1
    iy = ((points[:, 1] - min_p[1]) / grid_res).astype(np.int64) + 1

    occupied = np.zeros((cols, rows), dtype=bool)
    occupied[ix, iy] = True
    mean_z = float(points[:, 2].mean())

    def to_world(cx: np.ndarray, cy: np.ndarray) -> np.ndarray:
        return np.column_stack(
            [
                min_p[0] + (cx - 1) * grid_res,
                min_p[1] + (cy - 1) * grid_res,
                np.full(cx.size, mean_z),
            ]
        )

    holes = ndimage.binary_fill_holes(occupied) & ~occupied
    filled = to_world(*np.nonzero(holes))
    # Holes count as plane from here on, so the boundary is the outline of the
    # *repaired* plane rather than of its holes as well.
    occupied |= holes
    boundary = to_world(*np.nonzero(occupied & ~ndimage.binary_erosion(occupied)))
    return filled, boundary


def _stitch(
    boundary_a: np.ndarray,
    boundary_b: np.ndarray,
    *,
    max_gap: float,
    density: float,
    rng: np.random.Generator,
) -> Optional[np.ndarray]:
    """Sample points across the gap between two plane boundaries.

    Only the boundary points within ``max_gap`` of the *other* plane take part,
    so two planes that touch along one edge are bridged there and not across
    their full extent. Delaunay over the union then gives triangles, and only
    those with a vertex on each side — a triangle wholly inside one plane's own
    boundary ring would re-fill area that is already there.

    The per-triangle edge cap is the second guard: Delaunay triangulates the
    convex hull, so it happily spans the empty space *around* two nearby planes
    with long thin triangles. Dropping any triangle with an edge over
    ``1.5 * max_gap`` removes those without dropping the legitimately stretched
    ones along the seam.
    """
    tree_b = cKDTree(boundary_b[:, :2])
    near_a = boundary_a[tree_b.query(boundary_a[:, :2], distance_upper_bound=max_gap)[0] < np.inf]
    tree_a = cKDTree(boundary_a[:, :2])
    near_b = boundary_b[tree_a.query(boundary_b[:, :2], distance_upper_bound=max_gap)[0] < np.inf]
    if len(near_a) < 2 or len(near_b) < 2:
        return None

    combined = np.vstack([near_a, near_b])
    try:
        simplices = Delaunay(combined[:, :2]).simplices
    except QhullError:
        # Collinear or duplicate seam points — a single-cell-wide contact strip
        # has no 2D triangulation. The offline tool let this propagate and abort
        # the whole conversion; skipping this one pair loses one seam.
        return None

    corners = combined[simplices]
    from_b = simplices >= len(near_a)
    edges = np.stack(
        [
            np.linalg.norm(corners[:, 1, :2] - corners[:, 0, :2], axis=1),
            np.linalg.norm(corners[:, 2, :2] - corners[:, 1, :2], axis=1),
            np.linalg.norm(corners[:, 0, :2] - corners[:, 2, :2], axis=1),
        ],
        axis=1,
    )
    area = 0.5 * np.linalg.norm(
        np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]), axis=1
    )
    keep = (
        from_b.any(axis=1)
        & (~from_b).any(axis=1)
        & (edges.max(axis=1) <= max_gap * 1.5)
        & (area > 0)
    )
    if not keep.any():
        return None

    corners, area = corners[keep], area[keep]
    counts = np.maximum(1, (area * density).astype(np.int64))
    which = np.repeat(np.arange(len(counts)), counts)
    # Uniform barycentric sampling: draw in the unit square and reflect the half
    # that lands outside the triangle back into it.
    u = rng.random((which.size, 1))
    v = rng.random((which.size, 1))
    outside = (u + v > 1).ravel()
    u[outside] = 1.0 - u[outside]
    v[outside] = 1.0 - v[outside]
    origin_v = corners[which, 0]
    return origin_v + u * (corners[which, 1] - origin_v) + v * (corners[which, 2] - origin_v)


def repair_ground(
    logger: structlog.stdlib.BoundLogger,
    ground: o3d.geometry.PointCloud,
    *,
    grid_res: float = 0.04,
    max_gap: float = 0.80,
    min_step: float = 0.07,
    max_step: float = 0.31,
    cluster_eps: float = 0.08,
    cluster_min_points: int = 60,
    min_fragment_points: int = 70,
    same_plane_z: float = 0.05,
    footprint_dilation: int = 2,
    fill_density: float = 1.5,
    max_plane_cells: int = 200_000_000,
    seed: int = 0,
) -> o3d.geometry.PointCloud:
    """Close the gaps a LIO map leaves in a floor that is continuous in reality.

    Three kinds of missing area, three passes:

    * **Holes inside a plane** — ring gaps and occlusion shadows. Filled by
      rasterising the plane and closing its enclosed pockets.
    * **Seams between coplanar fragments** — the same floor, split by DBSCAN
      because the points thin out along a line the lidar grazed. Stitched after
      the fragments are merged into one plane.
    * **Single steps** — a kerb or a threshold whose vertical face the lidar
      barely saw. Stitched when the height difference falls in
      ``[min_step, max_step]``, so the ramp-like transition the robot can
      actually cross gets a surface and a 60 cm drop does not.

    ``min_step`` is well below any real step height (7 cm) on purpose: the top
    surface of a step is the part the lidar keeps *worst*, so the measured
    height difference between two fragments understates the real one. Widening
    the window downward costs nothing — a difference under ``same_plane_z``
    already merged as one plane in the previous pass.

    Returns a new cloud: the input in grey plus the generated points in **red**,
    which is how a site's repair gets eyeballed in a viewer before the grid is
    trusted. Generated points are voxel-downsampled to just under the grid
    pitch, since the triangle sampling deliberately over-samples.

    Sampling is seeded (``seed``), unlike the offline tool's bare
    ``np.random.rand``, so re-converting a map reproduces it — the same reason
    ``convert_pcd_to_gridmap`` computes in float32.
    """
    points = np.asarray(ground.points)
    if len(points) == 0:
        raise ValueError("ground cloud is empty; nothing to repair")

    original = o3d.geometry.PointCloud(ground)
    if not original.has_colors():
        original.paint_uniform_color([0.6, 0.6, 0.6])

    labels = np.asarray(
        ground.cluster_dbscan(eps=cluster_eps, min_points=cluster_min_points, print_progress=False)
    )
    fragments = [
        label
        for label in range(labels.max() + 1)
        if np.count_nonzero(labels == label) > min_fragment_points
    ]
    if not fragments:
        logger.warning(
            "no plane fragment survived clustering; returning the ground unchanged",
            cluster_eps=cluster_eps,
            cluster_min_points=cluster_min_points,
        )
        return original

    planes = _merge_coplanar(
        logger,
        points,
        labels,
        fragments,
        grid_res=grid_res,
        same_plane_z=same_plane_z,
        dilation=footprint_dilation,
    )

    generated = []
    boundaries = []
    for plane in planes:
        filled, boundary = _fill_and_outline(
            points[np.isin(labels, plane)], grid_res, max_plane_cells
        )
        if len(filled):
            generated.append(filled)
        boundaries.append(boundary)
    logger.info(
        "filled plane interiors",
        planes=len(planes),
        points=int(sum(len(chunk) for chunk in generated)),
    )

    # Points per m² of stitched surface. 1.5/grid_res² lands the seam at roughly
    # the plane's own raster density, so the seam does not read as a denser
    # (or sparser) region than the floor it joins.
    density = fill_density / (grid_res * grid_res)
    rng = np.random.default_rng(seed)
    seams = 0
    for i, boundary_a in enumerate(boundaries):
        if not len(boundary_a):
            continue
        for boundary_b in boundaries[i + 1:]:
            if not len(boundary_b):
                continue
            z_diff = abs(boundary_a[:, 2].mean() - boundary_b[:, 2].mean())
            if not (z_diff < same_plane_z or min_step <= z_diff <= max_step):
                continue
            patch = _stitch(
                boundary_a, boundary_b, max_gap=max_gap, density=density, rng=rng
            )
            if patch is not None:
                generated.append(patch)
                seams += 1
    logger.info("stitched seams", seams=seams)

    if not generated:
        logger.warning("nothing to repair: no interior hole and no stitchable seam")
        return original

    patch_cloud = o3d.geometry.PointCloud()
    patch_cloud.points = o3d.utility.Vector3dVector(np.vstack(generated))
    patch_cloud.paint_uniform_color([1.0, 0.0, 0.0])
    patch_cloud = patch_cloud.voxel_down_sample(voxel_size=grid_res * 0.8)
    logger.info("repair done", generated=len(patch_cloud.points), original=len(original.points))
    return original + patch_cloud


def build_traversable_cloud(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    *,
    segment: Optional[dict] = None,
    repair: Optional[dict] = None,
    debug_dir: Optional[str] = None,
) -> np.ndarray:
    """Run segmentation then repair, and return the traversable cloud as (N, 3).

    An ndarray rather than a ``PointCloud`` because the only consumer is
    ``convert_traversable_to_gridmap``, and handing it plain xyz is what keeps
    ``pcd_to_gridmap`` free of open3d (see this module's docstring).

    ``debug_dir`` writes the intermediates the offline tool wrote as its four
    output PCDs, plus the repaired ground. They are the only way to tell a bad
    intensity window from a bad step range when a site's grid comes out wrong,
    and they are worth ~4× the map's size on disk, so they are opt-in.
    """
    segmented = segment_traversable(logger, pcd_path, **(segment or {}))
    repaired = repair_ground(logger, segmented.ground, **(repair or {}))

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
        for filename, cloud in (
            ("ground.pcd", segmented.ground),
            ("slope.pcd", segmented.slope),
            ("obstacle.pcd", segmented.obstacle),
            ("obstacle_raw.pcd", segmented.obstacle_raw),
            ("ground_repaired.pcd", repaired),
        ):
            if len(cloud.points) == 0:
                continue
            path = os.path.join(debug_dir, filename)
            o3d.io.write_point_cloud(path, cloud)
            logger.info("wrote segmentation debug cloud", path=path, points=len(cloud.points))

    return np.asarray(repaired.points)
