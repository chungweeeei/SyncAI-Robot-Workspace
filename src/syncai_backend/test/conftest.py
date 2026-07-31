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
def make_pgm():
    """Factory writing a binary PGM, with hooks for the malformed cases.

    ``comment`` inserts a ``# ...`` line after the magic (what GIMP does);
    ``body_bytes`` overrides the pixel body so a truncated file can be built.
    """
    def _make(path, width, height, fill=254, comment=None, maxval=255,
              magic=b"P5", body_bytes=None):
        header = magic + b"\n"
        if comment is not None:
            header += b"# " + comment.encode("ascii") + b"\n"
        header += f"{width} {height}\n{maxval}\n".encode("ascii")
        body = bytes([fill]) * (width * height) if body_bytes is None else body_bytes
        path.write_bytes(header + body)
        return path

    return _make


@pytest.fixture
def make_gridmap_yaml():
    """Factory writing a map YAML in the shape pcd_to_gridmap.py emits."""
    def _make(path, resolution=0.05, origin=(-1.0, -2.0, 0.0)):
        path.write_text(
            "image: gridmap.pgm\n"
            "mode: trinary\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        return path

    return _make


@pytest.fixture
def make_pcd():
    """Factory writing a minimal ``DATA ascii`` PCD with xyz fields.

    Real enough for read_pcd_xyz: the map catalogue's pointcloud endpoint parses
    the file it serves, so a fixture of filler bytes would only ever exercise
    the failure path.
    """
    def _make(path, points=((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0))):
        rows = "\n".join(f"{x} {y} {z}" for x, y, z in points)
        path.write_text(
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {len(points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\n"
            "DATA ascii\n"
            f"{rows}\n"
        )
        return path

    return _make


@pytest.fixture
def maps_dir(tmp_path, make_pgm, make_gridmap_yaml, make_pcd):
    """A maps root laid out the way the robot's ``map/`` directory is.

    ``full`` is a converted map (pcd + gridmap), ``rawonly`` is one straight out
    of pgo/save_maps with no gridmap yet, and a loose file sits at the root to
    prove only directories are listed.
    """
    root = tmp_path / "map"
    root.mkdir()

    full = root / "full"
    full.mkdir()
    make_pcd(full / "map.pcd")
    make_pgm(full / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(full / "gridmap.yaml", origin=(-6.94, -11.09, 0.0))

    rawonly = root / "rawonly"
    rawonly.mkdir()
    make_pcd(rawonly / "map.pcd", points=((0.0, 0.0, 0.0),))

    (root / "stray.pgm").write_bytes(b"not a map directory")

    return root


@pytest.fixture
def catalog_repo(logger, maps_dir):
    """A MapCatalogRepo rooted at the fake maps tree."""
    from syncai_backend.repositories.map.catalog import init_map_catalog_repo

    return init_map_catalog_repo(logger=logger, maps_dir=str(maps_dir))


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
