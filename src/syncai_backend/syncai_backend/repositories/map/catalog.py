"""The maps stored on the robot's disk, as opposed to the one that is loaded.

``MapRepo`` next door owns vertex CRUD in Postgres. This repo never touches ROS:
it reads — and, since the editor's save path landed, writes — ``map/<name>/``,
the directories ``pgo/save_maps`` and ``tools/pcd_to_gridmap.py`` produce.

Writing is deliberately narrow: ``write_gridmap`` replaces the *cells* of an
existing gridmap and nothing else. It cannot create a map, cannot change a map's
extent, and never rewrites ``gridmap.yaml``. Everything about the grid's
geometry stays a property of the pcd → grid conversion.
"""

import os
import re
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List, Optional, Tuple

import structlog
import yaml

from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.helpers.pgm import read_pgm_size, write_pgm
from syncai_backend.helpers.system_config import active_map_name


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

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.maps_dir = os.path.expanduser("~/robot_ws/map")
        # Logged because the path is neither a parameter nor an env var: an empty
        # catalogue on a robot whose HOME is not what the container expects would
        # otherwise leave no breadcrumb at all about where we looked.
        self.logger.info("[MapCatalogRepo] Serving maps", path=self.maps_dir)

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
        path = os.path.join(self.resolve_dir(name), "gridmap.pgm")
        return path if os.path.isfile(path) else None

    def gridmap_yaml_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.yaml``, or None if absent.

        Absolute and free of ``~`` by construction — ``maps_dir`` is already
        expanded and ``resolve_dir`` returns a realpath — which is exactly what
        the consumer needs. ``syncai_map_server``'s ``loadMapYaml`` expands
        ``~/`` only to open the yaml, then resolves the yaml's *relative*
        ``image:`` key against ``dirname()`` of the string it was handed
        unexpanded, so a ``~``-prefixed url loads the metadata and then fails on
        the image with ``RESULT_INVALID_MAP_DATA`` — which reads like a corrupt
        map rather than a path bug. Deriving the path from here instead of from
        the INI's ``[map] map`` also sidesteps a second trap: that value is
        *relative* to the workspace root.
        """
        path = os.path.join(self.resolve_dir(name), "gridmap.yaml")
        return path if os.path.isfile(path) else None

    def pointcloud_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``map.pcd``, or None if absent.

        The same file ``has_pointcloud`` reports on — this hands back the path
        so the REST layer can parse it, rather than making the caller rebuild it
        from ``resolve_dir`` and re-do the containment checks.
        """
        path = os.path.join(self.resolve_dir(name), "map.pcd")
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

    # --- Writing ------------------------------------------------------------

    def write_gridmap(self, name: str, data: bytes) -> bytes:
        """Replace an existing gridmap's cells with ``data``; return the file.

        ``data`` is one byte per cell in .pgm row order (row 0 is the top of the
        map, max y) and must be exactly ``width * height`` long — the extent is
        read back off the file being replaced, so this cannot resize a map. That
        check lives here rather than in the REST layer so "you cannot change a
        map's extent through this repo" is a property of the repo, true for any
        future caller, and not of one endpoint.

        Returns the whole file as written, for the REST layer's ETag.

        **``gridmap.yaml`` is never touched**, and that is not an omission. The
        body length pins the extent; ``resolution`` and ``origin`` are properties
        of the pcd → grid conversion, not of cell values; ``mode`` and the two
        thresholds are how the loader *interprets* bytes, and the editor writes
        values already in range for the existing ones. Rewriting it would be all
        downside: ``yaml.safe_dump`` reformats, losing the ``image: gridmap.pgm``
        spelling and the inline ``origin: [x, y, 0.0]`` that
        ``tools/pcd_to_gridmap.py`` writes, and ``image:`` *must* stay relative
        because map_server resolves it against the yaml's own directory. A torn
        yaml is also the one failure the .pgm's atomic write cannot rescue — the
        map stops loading entirely.
        """
        directory = self.resolve_dir(name)
        path = os.path.join(directory, "gridmap.pgm")

        # Re-checked here even though the router already 404'd on a map without a
        # grid. Without it a race would *create* a gridmap.pgm in a directory that
        # has none, i.e. a pgm with no yaml — which _read_grid reports as having no
        # grid at all, so the map would look untouched while holding the edit.
        if not os.path.isfile(path):
            raise NotFoundError(f"Map '{name}' has no gridmap to overwrite.")

        # Only when absent, so gridmap_raw.pgm always holds the pristine
        # pcd_to_gridmap.py output. Copying on every save would, on the *second*
        # save, overwrite that with the first save's edit and destroy the only way
        # back to the conversion tool's result. copy2 rather than copy to keep the
        # original mtime: otherwise the backup becomes the newest file under
        # _walk_stats and drags the card's modified_at forward to now.
        raw_path = os.path.join(directory, "gridmap_raw.pgm")
        if not os.path.exists(raw_path):
            shutil.copy2(path, raw_path)
            self.logger.info(
                "[MapCatalogRepo] Kept the pre-edit gridmap", map=name, path=raw_path
            )

        try:
            width, height = read_pgm_size(path)
        except ValueError as exc:
            # Only reachable on a race: the router got its geometry from
            # _read_grid, which calls this same function.
            self.logger.warning(
                "[MapCatalogRepo] Refusing to overwrite an unreadable gridmap",
                map=name,
                error=str(exc),
            )
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

        if len(data) != width * height:
            raise BadRequestError(
                f"Gridmap body is {len(data)} bytes; '{name}' is "
                f"{width}x{height} = {width * height} cells."
            )

        written = write_pgm(path=path, width=width, height=height, body=data)
        self.logger.info(
            "[MapCatalogRepo] Wrote gridmap", map=name, width=width, height=height
        )
        return written

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
            has_pointcloud=os.path.isfile(os.path.join(path, "map.pcd")),
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
        yaml_path = os.path.join(path, "gridmap.yaml")
        pgm_path = os.path.join(path, "gridmap.pgm")
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
    logger: structlog.stdlib.BoundLogger
) -> MapCatalogRepo:
    return MapCatalogRepo(logger=logger)
