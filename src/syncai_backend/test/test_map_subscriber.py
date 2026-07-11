"""Unit test for MapSubscriber's callback forwarding.

Only the message callback is exercised (no live rclpy node / DDS), which is the
subscriber's actual logic: hand each OccupancyGrid to the repo.
"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")

from syncai_backend.repositories.map.map import init_map_repo  # noqa: E402
from syncai_backend.subscribers.map_subscriber import MapSubscriber  # noqa: E402


def test_callback_forwards_grid_to_repo(logger, make_occupancy_grid):
    repo = init_map_repo(logger)
    subscriber = MapSubscriber(logger=logger, map_repo=repo)
    grid = make_occupancy_grid(1, 1, [0])

    subscriber._map_cb(grid)

    assert repo.get_map() is grid
