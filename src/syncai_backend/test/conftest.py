"""Shared pytest fixtures for syncai_backend unit tests.

These tests are meant to run inside the ROS 2 devcontainer (via ``colcon test``
or ``pytest``), where rclpy / nav_msgs / syncai_common / opencv are available.
The database layer is exercised against an in-memory SQLite engine instead of
PostgreSQL, so no database server is required.
"""

import pytest
import structlog

from sqlalchemy import create_engine
from sqlalchemy.pool import StaticPool

from syncai_backend.database.models import Base


@pytest.fixture
def logger():
    return structlog.get_logger()


@pytest.fixture
def engine():
    """An SQLAlchemy engine bound to a shared in-memory SQLite database.

    ``StaticPool`` keeps a single connection alive so every session sees the
    same in-memory schema/data; the DB is torn down after each test.
    """
    engine = create_engine(
        "sqlite://",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )
    Base.metadata.create_all(bind=engine)

    yield engine

    Base.metadata.drop_all(bind=engine)
    engine.dispose()


@pytest.fixture
def map_repo(logger, engine):
    """A MapRepo backed by the in-memory SQLite engine.

    nav_msgs is imported lazily inside the repo module, so tests that need this
    fixture should ``importorskip`` it themselves.
    """
    from syncai_backend.repositories.map.map import init_map_repo

    return init_map_repo(logger=logger, engine=engine)


@pytest.fixture
def make_occupancy_grid():
    """Factory building a nav_msgs/OccupancyGrid from raw cell values.

    nav_msgs is imported lazily so tests that never touch a grid (and the host
    machine without ROS 2) do not require it at collection time.
    """
    from nav_msgs.msg import OccupancyGrid

    def _make(width, height, data, resolution=0.05, origin=(-1.0, -2.0, 0.0),
              frame_id="map"):
        grid = OccupancyGrid()
        grid.header.frame_id = frame_id
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = float(origin[2])
        grid.data = list(data)
        return grid

    return _make
