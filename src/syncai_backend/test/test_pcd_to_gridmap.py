"""Tests for the pure grid passes in helpers/pcd_to_gridmap.py.

Only the pose-connectivity filter and its poses.txt reader live here — the full
conversions are exercised through the router in test_maps_router.py, where the
recipe parameters and the sidecar live. These are numpy/scipy-only, so the file
runs without ROS, FastAPI or opencv present.
"""

import numpy as np
import pytest

from syncai_backend.helpers.pcd_to_gridmap import read_poses_xy, revert_unreachable_free


def _grid(width=40, height=20):
    """An all-unknown grid with two free islands, columns 2-9 and 25-32."""
    grid = np.full((height, width), 205, dtype=np.uint8)
    grid[5:15, 2:10] = 254
    grid[5:15, 25:33] = 254
    return grid


ORIGIN = np.array([0.0, 0.0])
RES = 0.05


def test_free_space_the_poses_cannot_reach_becomes_unknown(logger):
    grid = _grid()
    # One pose in the left island (cell col 5, row 10).
    pose_xy = np.array([[5 * RES, 10 * RES]])

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 1
    assert stats["components_total"] == 2
    assert stats["components_kept"] == 1
    assert (out[5:15, 2:10] == 254).all(), "the seeded island lost free space"
    assert (out[5:15, 25:33] == 205).all(), "the unreachable island is still free"
    assert stats["reverted_free_cells"] == 10 * 8


def test_a_pose_on_an_occupied_cell_still_seeds_through_the_radius(logger):
    """The band-recentring measurements found 9-16 keyframe poses per map on
    occupied cells 5-14 cm from free space (wall brushes); a radius-0 seed would
    drop their component."""
    grid = _grid()
    grid[10, 10] = 0  # a wall cell hugging the left island's edge
    pose_xy = np.array([[10 * RES, 10 * RES]])  # the pose sits ON the wall

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES, seed_radius=3)

    assert stats["applied"] == 1
    assert (out[5:15, 2:10] == 254).all()


def test_out_of_bounds_poses_are_skipped_not_fatal(logger):
    grid = _grid()
    pose_xy = np.array([[5 * RES, 10 * RES], [-50.0, -50.0], [999.0, 999.0]])

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 1
    assert stats["poses"] == 3
    assert stats["poses_in_grid"] == 1
    assert (out[5:15, 2:10] == 254).all()


def test_no_seeded_component_leaves_the_grid_unchanged(logger):
    """Poses that miss every free component mean mismatched inputs; reverting
    all free space to satisfy the filter would destroy the map."""
    grid = _grid()
    pose_xy = np.array([[39 * RES, 1 * RES]])  # unknown corner, no free nearby

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 0
    assert stats["reverted_free_cells"] == 0
    assert (out == grid).all()


def test_occupied_and_unknown_cells_are_never_touched(logger):
    grid = _grid()
    grid[0, 0:5] = 0
    before_occ = (grid == 0).copy()
    pose_xy = np.array([[5 * RES, 10 * RES]])

    out, _ = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert ((out == 0) == before_occ).all()
    # Nothing unknown became free: this pass only demotes.
    assert not ((grid == 205) & (out == 254)).any()


def test_the_input_grid_is_not_mutated(logger):
    grid = _grid()
    before = grid.copy()
    pose_xy = np.array([[5 * RES, 10 * RES]])

    revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert (grid == before).all()


def test_diagonal_contact_counts_as_connected(logger):
    """8-connectivity, matching _despeckle: a diagonal touch is one component,
    so free space beyond it survives a seed on the other side."""
    grid = np.full((10, 10), 205, dtype=np.uint8)
    grid[2, 2] = 254
    grid[3, 3] = 254  # touches only diagonally

    out, stats = revert_unreachable_free(
        logger, grid, np.array([[2 * RES, 2 * RES]]), ORIGIN, RES, seed_radius=0
    )

    assert stats["components_total"] == 1
    assert out[3, 3] == 254


# --- read_poses_xy ------------------------------------------------------------


def test_read_poses_xy_parses_the_pgo_format(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text(
        "0.pcd 7.39105e-06 -4.7505e-06 0.00050289 0.990556 -0.00839693 0.136854 1.49619e-05\n"
        "1.pcd 0.253802 0.0762091 0.257022 0.985168 -0.0309628 0.126643 0.111568\n"
        "\n"
    )

    poses = read_poses_xy(str(path))

    assert poses.shape == (2, 2)
    assert poses[1, 0] == pytest.approx(0.253802)
    assert poses[1, 1] == pytest.approx(0.0762091)


def test_read_poses_xy_rejects_a_malformed_line(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd 1.0 2.0 0.0 1 0 0 0\nnot a pose\n")

    with pytest.raises(ValueError, match="poses.txt:2"):
        read_poses_xy(str(path))


def test_read_poses_xy_rejects_unparseable_coordinates(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd one two three 1 0 0 0\n")

    with pytest.raises(ValueError, match="could not parse"):
        read_poses_xy(str(path))


def test_read_poses_xy_rejects_an_empty_file(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("\n\n")

    with pytest.raises(ValueError, match="no poses"):
        read_poses_xy(str(path))
