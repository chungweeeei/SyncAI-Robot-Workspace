"""Unit tests for the in-memory MapRepo cache."""

import pytest

pytest.importorskip("nav_msgs")

from syncai_backend.repositories.map.map import init_map_repo  # noqa: E402


def test_empty_repo_returns_none(logger):
    repo = init_map_repo(logger)
    assert repo.get_map() is None


def test_update_then_get_returns_same_grid(logger, make_occupancy_grid):
    repo = init_map_repo(logger)
    grid = make_occupancy_grid(1, 1, [0])

    repo.update_map(grid)

    assert repo.get_map() is grid


def test_update_overwrites_previous_grid(logger, make_occupancy_grid):
    repo = init_map_repo(logger)
    first = make_occupancy_grid(1, 1, [0])
    second = make_occupancy_grid(2, 1, [0, 100])

    repo.update_map(first)
    repo.update_map(second)

    assert repo.get_map() is second
