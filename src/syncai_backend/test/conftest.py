"""Shared pytest fixtures for syncai_backend unit tests.

These tests are meant to run inside the ROS 2 devcontainer (via ``colcon test``
or ``pytest``), where rclpy / nav_msgs / syncai_common / opencv are available.
The database layer is exercised against an in-memory SQLite engine instead of
PostgreSQL, so no database server is required.
"""

import pytest
import structlog

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from syncai_backend.database.models import Base
from syncai_backend.repositories.map_point.map_point import MapPointRepo


@pytest.fixture
def logger():
    return structlog.get_logger()


@pytest.fixture
def session_factory():
    """A sessionmaker bound to a shared in-memory SQLite database.

    ``StaticPool`` keeps a single connection alive so every session sees the
    same in-memory schema/data; the DB is torn down after each test.
    """
    engine = create_engine(
        "sqlite://",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )
    Base.metadata.create_all(bind=engine)
    factory = sessionmaker(bind=engine, expire_on_commit=False)

    yield factory

    Base.metadata.drop_all(bind=engine)
    engine.dispose()


@pytest.fixture
def map_point_repo(logger, session_factory):
    return MapPointRepo(logger=logger, session_factory=session_factory)


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
