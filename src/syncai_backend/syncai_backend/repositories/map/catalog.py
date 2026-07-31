"""The maps stored on the robot's disk, as opposed to the one that is loaded.

``MapRepo`` next door caches the live ``map`` topic and owns vertex CRUD. This
repo never touches ROS: it reads ``map/<name>/`` — the directories
``pgo/save_maps`` and ``tools/pcd_to_gridmap.py`` write — and reports what is
there.
"""

import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List, Optional, Tuple

import structlog
import yaml

from syncai_backend.exceptions import BadRequestError
from syncai_backend.helpers.pgm import read_pgm_size
from syncai_backend.helpers.system_config import active_map_name

# Same convention as the artifact gateway's INI constant: an absolute default
# pointing into the container's workspace, overridable by environment so tests
# (and a differently laid out host) can point somewhere else. This package has no
# ROS parameters at all, so a param would have meant introducing a params file, a
# launch change and a restart-to-reconfigure story for one path.
DEFAULT_MAPS_DIR = os.path.expanduser("~/robot_ws/map")

MAPS_DIR_ENV = "SYNCAI_MAPS_DIR"

# The files a map directory is made of.
GRIDMAP_YAML = "gridmap.yaml"
GRIDMAP_PGM = "gridmap.pgm"
MAP_PCD = "map.pcd"

# Deliberately strict. This is the only thing standing between a URL path
# segment and the filesystem, and it also has to hold when the write endpoint
# lands, so it rejects everything that is not obviously a directory name.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


@dataclass(frozen=True)
class GridInfo:
    """A gridmap's geometry, from gridmap.yaml plus the .pgm header."""

    resolution: float
    # World pose of the grid's lower-left corner: [x, y, yaw] in the map frame.
    # This is the yaml's three-element origin, NOT the {x, y, z} position of an
    # OccupancyGrid message — pcd_to_gridmap.py always writes yaw 0.0.
    origin: Tuple[float, float, float]
    width: int
    height: int


@dataclass(frozen=True)
class StoredMap:
    name: str
    # None when the directory has a point cloud but no gridmap yet, which is the
    # state of every map between pgo/save_maps and pcd_to_gridmap.py.
    grid: Optional[GridInfo]
    has_pointcloud: bool
    size_bytes: int
    modified_at: datetime


class MapCatalogRepo:
    """Lists and reads the map directories under ``maps_dir``.

    Synchronous, like every other repo here: the REST handlers that call it are
    declared with plain ``def`` so FastAPI runs the file I/O on its worker thread
    pool rather than the event loop.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, maps_dir: str):
        self.logger = logger
        self.maps_dir = maps_dir

    # --- Paths --------------------------------------------------------------

    def resolve_dir(self, name: str) -> str:
        """Return the absolute path of a map directory, or raise BadRequestError.

        Two independent checks, because either alone has a hole: the pattern
        rejects separators and ``..`` before they reach the filesystem, and the
        realpath comparison catches a symlink inside ``maps_dir`` that points
        out of it (the pattern cannot see through a link).
        """
        if not _NAME_RE.match(name) or name in (".", ".."):
            raise BadRequestError(f"Invalid map name: {name!r}")

        candidate = os.path.realpath(os.path.join(self.maps_dir, name))
        root = os.path.realpath(self.maps_dir)
        if os.path.dirname(candidate) != root:
            raise BadRequestError(f"Invalid map name: {name!r}")

        return candidate

    def gridmap_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.pgm``, or None if absent."""
        path = os.path.join(self.resolve_dir(name), GRIDMAP_PGM)
        return path if os.path.isfile(path) else None

    def pointcloud_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``map.pcd``, or None if absent.

        The same file ``has_pointcloud`` reports on — this hands back the path
        so the REST layer can parse it, rather than making the caller rebuild it
        from ``resolve_dir`` and re-do the containment checks.
        """
        path = os.path.join(self.resolve_dir(name), MAP_PCD)
        return path if os.path.isfile(path) else None

    # --- Listing ------------------------------------------------------------

    def list_maps(self) -> List[StoredMap]:
        """Return every map directory, by name.

        Directories only: the legacy loose ``warehouse.pgm`` / ``testmap.yaml``
        pairs that used to sit at the root of ``map/`` are not maps under the
        per-directory layout, and neither are the ``gridmap_raw.pgm`` backups.
        """
        try:
            entries = sorted(os.scandir(self.maps_dir), key=lambda e: e.name)
        except FileNotFoundError:
            self.logger.warning(
                "[MapCatalogRepo] Maps directory does not exist", path=self.maps_dir
            )
            return []

        maps: List[StoredMap] = []
        for entry in entries:
            if not entry.is_dir():
                continue
            stored = self._read(entry.name, entry.path)
            if stored is not None:
                maps.append(stored)
        return maps

    def get_map(self, name: str) -> Optional[StoredMap]:
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            return None
        return self._read(name, path)

    def active_name(self) -> Optional[str]:
        return active_map_name(self.logger)

    # --- Internals ----------------------------------------------------------

    def _read(self, name: str, path: str) -> Optional[StoredMap]:
        """Describe one directory; None only if it disappeared mid-scan."""
        try:
            size_bytes, newest_mtime = _walk_stats(path)
        except FileNotFoundError:
            return None

        return StoredMap(
            name=name,
            grid=self._read_grid(name, path),
            has_pointcloud=os.path.isfile(os.path.join(path, MAP_PCD)),
            size_bytes=size_bytes,
            modified_at=datetime.fromtimestamp(newest_mtime, tz=timezone.utc),
        )

    def _read_grid(self, name: str, path: str) -> Optional[GridInfo]:
        """Read geometry from gridmap.yaml + the .pgm header, or None.

        A directory whose gridmap is unreadable is reported as having no grid
        rather than failing: one map caught mid-save must not take the whole
        catalogue listing down with it. The reason is logged, because "the card
        says no 2D grid but the files are right there" is otherwise a mystery.
        """
        yaml_path = os.path.join(path, GRIDMAP_YAML)
        pgm_path = os.path.join(path, GRIDMAP_PGM)
        if not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
            return None

        try:
            with open(yaml_path, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}

            resolution = float(document["resolution"])
            origin = document["origin"]
            width, height = read_pgm_size(pgm_path)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable gridmap; reporting the map without one",
                map=name,
                error=str(exc),
            )
            return None

        return GridInfo(
            resolution=resolution,
            origin=(float(origin[0]), float(origin[1]), float(origin[2])),
            width=width,
            height=height,
        )


def _walk_stats(path: str) -> Tuple[int, float]:
    """Return total bytes and newest mtime under ``path``, dir itself included.

    ``patches/`` holds hundreds of small .pcd files, so this is a real walk
    rather than a stat of the directory entry — but ``map.pcd`` (~20 MB) is what
    the number is actually reporting.
    """
    total = 0
    newest = os.stat(path).st_mtime

    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                stats = os.stat(os.path.join(root, filename))
            except FileNotFoundError:
                # A file removed while we walked (a save in flight); skip it.
                continue
            total += stats.st_size
            newest = max(newest, stats.st_mtime)

    return total, newest


def init_map_catalog_repo(
    logger: structlog.stdlib.BoundLogger, maps_dir: Optional[str] = None
) -> MapCatalogRepo:
    resolved = maps_dir or os.environ.get(MAPS_DIR_ENV) or DEFAULT_MAPS_DIR
    logger.info("[MapCatalogRepo] Serving maps", path=resolved)
    return MapCatalogRepo(logger=logger, maps_dir=resolved)
