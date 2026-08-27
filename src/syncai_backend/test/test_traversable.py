"""Tests for the traversability pipeline: segmentation, ground repair, grid.

Real geometry on real arrays, no mocking — the units under test are numeric, so
a stubbed neighbour search would test nothing. Each case is built from
``_sheet``: a flat raster of points at a known height, which is what a LIO floor
looks like after ``segment_traversable`` has cut it out.

open3d is only needed to *import* ``helpers.traversable`` (the segmentation and
repair entry points are thin wrappers over its KD-tree searches, and are not
exercised here — they need a real cloud with intensity), so it is skipped
through a fixture rather than at module level: the stage-3 conversion lives in
``helpers.pcd_to_gridmap``, is open3d-free by design, and must still run on a
machine without the wheel.
"""

import numpy as np
import pytest
import yaml

from syncai_backend.helpers.pcd_to_gridmap import convert_traversable_to_gridmap
from syncai_backend.helpers.pgm import read_pgm_size

RES = 0.04


@pytest.fixture
def traversable():
    """``helpers.traversable``, or skip. Per-test, not module-level: an
    ``importorskip`` at import time would take the stage-3 cases down with it."""
    return pytest.importorskip(
        "syncai_backend.helpers.traversable", reason="open3d is not installed"
    )


def _sheet(x0, x1, y0, y1, z, step=RES / 2):
    """A flat raster of points, sampled finer than the repair grid pitch."""
    xx, yy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)])


def _read_grid(basename):
    """The pgm body as an (h, w) array, plus the parsed yaml."""
    width, height = read_pgm_size(basename + ".pgm")
    body = open(basename + ".pgm", "rb").read()[-width * height:]
    grid = np.frombuffer(body, dtype=np.uint8).reshape(height, width)
    with open(basename + ".yaml") as handle:
        return grid, yaml.safe_load(handle)


# --- union-find -------------------------------------------------------------


def test_find_root_walks_a_chain_deeper_than_the_recursion_limit(traversable):
    """The reason ``_find_root`` is iterative: a fragmented floor is this deep."""
    depth = 5000
    parent = {i: max(0, i - 1) for i in range(depth)}

    assert traversable._find_root(parent, depth - 1) == 0
    # And compressed on the way back, or the next lookup pays the walk again.
    assert parent[depth - 1] == 0


# --- footprints -------------------------------------------------------------


def test_dilated_footprints_overlap_for_fragments_that_only_touch(traversable):
    """A 5 cm seam is ~1 cell: within the ±2 dilation, so the two are one plane.

    Without the dilation the two fragments share no cell at all and the merge
    pass would leave the floor split down the seam.
    """
    here = _sheet(0.0, 1.0, 0.0, 1.0, 0.0)
    near = _sheet(1.05, 2.0, 0.0, 1.0, 0.0)
    far = _sheet(3.0, 4.0, 0.0, 1.0, 0.0)
    origin = np.vstack([here, near, far]).min(axis=0)
    stride = traversable._row_stride(np.vstack([here, near, far]), origin, RES, 2)

    keys = [
        traversable._footprint_keys(part, origin, RES, 2, stride)
        for part in (here, near, far)
    ]

    assert np.intersect1d(keys[0], keys[1]).size > 0
    assert np.intersect1d(keys[0], keys[2]).size == 0


def test_footprints_stay_comparable_across_fragments_of_different_extent(traversable):
    """The stride packs (x, y) into one key, so it must come from the *whole*
    cloud. A per-fragment stride reads as working whenever the fragments span
    the same rows, then silently compares unrelated integers when they do not:
    here the tall fragment overlaps the short one and must be found to."""
    short = _sheet(0.0, 1.0, 0.0, 0.5, 0.0)
    tall = _sheet(1.05, 2.0, 0.0, 4.0, 0.0)
    points = np.vstack([short, tall])
    origin = points.min(axis=0)
    stride = traversable._row_stride(points, origin, RES, 2)

    keys_short = traversable._footprint_keys(short, origin, RES, 2, stride)
    keys_tall = traversable._footprint_keys(tall, origin, RES, 2, stride)

    assert np.intersect1d(keys_short, keys_tall).size > 0


# --- coplanar merge ---------------------------------------------------------


def test_merge_needs_both_overlap_and_height(logger, traversable):
    """Fragment 2 sits on fragment 1's footprint but 15 cm up: a step, not floor."""
    parts = [
        _sheet(0.00, 1.0, 0.0, 0.5, 0.00),
        _sheet(1.05, 2.0, 0.0, 2.0, 0.01),
        _sheet(1.05, 2.0, 0.0, 2.0, 0.15),
    ]
    points = np.vstack(parts)
    labels = np.concatenate([np.full(len(p), i) for i, p in enumerate(parts)])

    planes = traversable._merge_coplanar(
        logger, points, labels, [0, 1, 2], grid_res=RES, same_plane_z=0.05, dilation=2
    )

    assert sorted(sorted(plane) for plane in planes) == [[0, 1], [2]]


# --- interior fill ----------------------------------------------------------


def test_fill_closes_an_enclosed_hole_but_not_an_edge_notch(traversable):
    """The conservative half of the repair: a notch open to the edge may be a
    real obstacle footprint, so only fully enclosed pockets are filled."""
    plane = _sheet(0.0, 2.0, 0.0, 2.0, 0.0)
    enclosed = (np.abs(plane[:, 0] - 1.0) < 0.15) & (np.abs(plane[:, 1] - 1.0) < 0.15)
    notch = (plane[:, 0] > 1.80) & (np.abs(plane[:, 1] - 0.5) < 0.15)

    filled, boundary = traversable._fill_and_outline(
        plane[~(enclosed | notch)], RES, 200_000_000
    )

    assert len(filled) > 0
    assert len(boundary) > 0
    # Every generated point is inside the enclosed hole, none in the notch.
    assert np.abs(filled[:, 0] - 1.0).max() < 0.20
    assert np.abs(filled[:, 1] - 1.0).max() < 0.20
    assert np.allclose(filled[:, 2], 0.0)


def test_fill_refuses_a_plane_that_rasterises_too_large(traversable):
    plane = _sheet(0.0, 1.0, 0.0, 1.0, 0.0)

    with pytest.raises(ValueError, match="rasterises"):
        traversable._fill_and_outline(plane, RES, max_cells=100)


# --- seam stitching ---------------------------------------------------------


@pytest.fixture
def step_boundaries(traversable):
    """Two plane outlines 30 cm apart with a 15 cm step between them."""
    lower = traversable._fill_and_outline(_sheet(0.0, 1.0, 0.0, 1.0, 0.0), RES, 10 ** 9)[1]
    upper = traversable._fill_and_outline(_sheet(1.30, 2.3, 0.0, 1.0, 0.15), RES, 10 ** 9)[1]
    return lower, upper


def test_stitch_bridges_within_max_gap_only(traversable, step_boundaries):
    lower, upper = step_boundaries
    density = 1.5 / (RES * RES)

    too_far = traversable._stitch(
        lower, upper, max_gap=0.10, density=density, rng=np.random.default_rng(0)
    )
    bridged = traversable._stitch(
        lower, upper, max_gap=0.80, density=density, rng=np.random.default_rng(0)
    )

    assert too_far is None
    assert bridged is not None and len(bridged) > 100
    # The seam stays in the gap; Delaunay's convex hull must not spill sideways.
    assert bridged[:, 0].min() > 0.9
    assert bridged[:, 0].max() < 1.4


def test_stitch_is_reproducible_under_a_seed(traversable, step_boundaries):
    """Why the sampling takes a Generator instead of calling np.random: a
    re-converted map must reproduce the map it replaces."""
    lower, upper = step_boundaries
    kwargs = dict(max_gap=0.80, density=1.5 / (RES * RES))

    first = traversable._stitch(lower, upper, rng=np.random.default_rng(7), **kwargs)
    second = traversable._stitch(lower, upper, rng=np.random.default_rng(7), **kwargs)

    assert np.array_equal(first, second)


def test_stitch_skips_a_seam_with_no_triangulation(traversable):
    """A one-cell-wide contact strip is collinear in 2D. Qhull raises on it, and
    losing one seam beats aborting the conversion."""
    column = np.linspace(0.0, 1.0, 20)
    first = np.column_stack([np.zeros(20), column, np.zeros(20)])
    second = np.column_stack([np.zeros(20), column, np.full(20, 0.001)])

    result = traversable._stitch(
        first, second, max_gap=0.8, density=100.0, rng=np.random.default_rng(0)
    )

    assert result is None


# --- stage 3: traversable cloud -> gridmap ----------------------------------


def test_conversion_writes_a_binary_grid_and_loadable_yaml(logger, tmp_path):
    """No unknown cells by construction: the cloud is the whole free space.

    The yaml assertion is the point of the test — the offline tool built this
    file from an indented f-string and emitted YAML that ``safe_load`` folds
    into one scalar, so map_server could not read the map it had just written.
    """
    basename = str(tmp_path / "gridmap")

    convert_traversable_to_gridmap(logger, _sheet(0.0, 2.0, 0.0, 3.0, 0.0), basename)

    grid, meta = _read_grid(basename)
    assert set(np.unique(grid).tolist()) <= {0, 254}
    assert meta["image"] == "gridmap.pgm"
    assert meta["resolution"] == 0.05
    assert meta["mode"] == "trinary"
    assert len(meta["origin"]) == 3
    assert (meta["occupied_thresh"], meta["free_thresh"]) == (0.65, 0.196)


def test_conversion_registers_the_grid_against_the_yaml_origin(logger, tmp_path):
    """A point at a known world position must land in the cell the yaml's origin
    and resolution address — the flip in row order is where this goes wrong."""
    basename = str(tmp_path / "gridmap")
    points = np.array([[0.0, 0.0, 0.0], [2.0, 3.0, 0.0]])

    convert_traversable_to_gridmap(logger, points, basename, gap_fill_size=0.0)

    grid, meta = _read_grid(basename)
    height = grid.shape[0]
    for x, y in points[:, :2]:
        col = int((x - meta["origin"][0]) / meta["resolution"])
        row = height - 1 - int((y - meta["origin"][1]) / meta["resolution"])
        assert grid[row, col] == 254


def test_gap_fill_bridges_a_cloud_sparser_than_the_cell_pitch(logger, tmp_path):
    """A floor sampled every 20 cm on a 5 cm grid is 1-in-4 cells. Without the
    closing pass the planner sees a field of obstacles where the aisle is."""
    cloud = _sheet(0.0, 4.0, 0.0, 4.0, 0.0, step=0.20)

    free = []
    for gap in (0.0, 0.60):
        basename = str(tmp_path / f"gridmap_{gap}")
        convert_traversable_to_gridmap(logger, cloud, basename, gap_fill_size=gap)
        free.append(int((_read_grid(basename)[0] == 254).sum()))

    assert free[1] > 8 * free[0]


def test_conversion_rejects_an_empty_cloud(logger, tmp_path):
    with pytest.raises(ValueError, match="empty"):
        convert_traversable_to_gridmap(
            logger, np.zeros((0, 3)), str(tmp_path / "gridmap")
        )


def test_conversion_rejects_a_cloud_of_the_wrong_shape(logger, tmp_path):
    with pytest.raises(ValueError, match=r"\(N, 3\)"):
        convert_traversable_to_gridmap(
            logger, np.zeros(12), str(tmp_path / "gridmap")
        )
