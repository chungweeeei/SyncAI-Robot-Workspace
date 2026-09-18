import os
import re
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List, Optional, Tuple

import structlog
import yaml

from syncai_backend.exceptions import BadRequestError, NotFoundError


# Same rule, and the same two reasons, as MapCatalogRepo's: this is the only
# thing standing between a URL path segment and an rmtree, and the name also has
# to survive being handed to ``ros2 bag record -o``.
#
# It doubles as a rosbag2 constraint. The recorder names its split files
# ``<name>_0.db3`` after the directory, so a name with a separator or a space in
# it is a bag whose metadata.yaml points at paths that do not resolve.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# rosbag2 writes this last, on a clean shutdown, and nothing else writes it. Its
# presence is therefore the whole of "this recording finished properly" — see
# the status derivation in the REST layer.
METADATA_FILE = "metadata.yaml"

# The top-level key rosbag2 puts everything under.
_METADATA_ROOT = "rosbag2_bagfile_information"


@dataclass(frozen=True)
class BagInfo:
    """What ``metadata.yaml`` says about a finished recording.

    Only the four fields an operator picks a bag by. rosbag2 also records the
    per-file split list, the serialization format and the full QoS profile of
    every topic; reflecting those through the catalogue would be an API surface
    nobody asked for, and they are in the file for whoever is on the robot.
    """

    duration_seconds: float
    message_count: int
    topics: List[str]
    compression: Optional[str]


@dataclass(frozen=True)
class StoredRecording:
    name: str
    size_bytes: int
    modified_at: datetime
    # None for a recording with no metadata.yaml: one still being written, and
    # one whose process died before it could flush. The two are told apart by
    # the live slot in RecordingGateway, which is process state this repo cannot
    # see — the REST layer does that reconciliation, exactly as _grid_status
    # does it for conversions.
    bag: Optional[BagInfo]


class RecordingCatalogRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.records_dir = os.path.expanduser("~/robot_ws/record")
        # Same breadcrumb rationale as MapCatalogRepo and TtsGateway: the path is
        # neither a ROS parameter nor an INI key, so on a robot whose HOME is not
        # what the container expects this line is the only record of where we
        # looked.
        self.logger.info(
            "[RecordingCatalogRepo] Serving recordings", path=self.records_dir
        )

    # --- Paths --------------------------------------------------------------

    def resolve_dir(self, name: str) -> str:
        """Return the absolute path of a recording directory, or raise.

        Two independent checks, because either alone has a hole: the pattern
        rejects separators and ``..`` before they reach the filesystem, and the
        realpath comparison catches a symlink inside ``records_dir`` that points
        out of it (the pattern cannot see through a link). Lifted from
        MapCatalogRepo.resolve_dir rather than imported — it is fifteen lines,
        and that class is the *map* vocabulary; importing it here would make a
        bag directory a kind of map.
        """
        if not _NAME_RE.match(name) or name in (".", ".."):
            raise BadRequestError(f"Invalid recording name: {name!r}")

        candidate = os.path.realpath(os.path.join(self.records_dir, name))
        root = os.path.realpath(self.records_dir)
        if os.path.dirname(candidate) != root:
            raise BadRequestError(f"Invalid recording name: {name!r}")

        return candidate

    def exists(self, name: str) -> bool:
        """Whether a directory of this name is already there.

        Deliberately not a ``create_dir``: unlike a map save, the directory here
        belongs to ``ros2 bag record -o``, which creates it itself and refuses
        outright if the path exists. This answers the ``name_taken`` question
        early so the caller gets a 409 with a sentence rather than a dead child
        and a CLI error; the recorder's own refusal is the backstop behind it.
        """
        return os.path.exists(self.resolve_dir(name))

    # --- Listing ------------------------------------------------------------

    def list_recordings(self) -> List[StoredRecording]:
        """Return every recording directory, newest first.

        Directories only. A bag *is* a directory under rosbag2 — the loose
        ``*.db3`` of a run whose directory was taken apart by hand is not
        something this can describe, and neither is the stray file.
        """
        try:
            entries = list(os.scandir(self.records_dir))
        except FileNotFoundError:
            # Not a warning: an empty record/ is the state of every robot that
            # has never recorded anything, and the directory is gitignored so a
            # fresh clone does not have it.
            self.logger.info(
                "[RecordingCatalogRepo] Recordings directory does not exist",
                path=self.records_dir,
            )
            return []

        recordings: List[StoredRecording] = []
        for entry in entries:
            if not entry.is_dir():
                continue
            stored = self._read(entry.name, entry.path)
            if stored is not None:
                recordings.append(stored)

        # Newest first: a bag is looked up by "the run I just did", never
        # alphabetically, which is the opposite of the map catalogue's order.
        recordings.sort(key=lambda item: item.modified_at, reverse=True)
        return recordings

    def get_recording(self, name: str) -> Optional[StoredRecording]:
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            return None
        return self._read(name, path)

    # --- Writing ------------------------------------------------------------

    def delete_dir(self, name: str) -> None:
        """Remove ``record/<name>/`` and everything under it. There is no undo.

        ``resolve_dir`` is what makes this safe to hang off a URL path segment:
        the name is regex-checked and the resolved path is confined under the
        records root before an ``rmtree`` ever sees it.

        Refusing to delete the recording currently being *written* is the
        router's job, not this repo's — which recording is live is process state
        held by the gateway, and keeping that 409 next to the others puts every
        refusal for the route in one place.
        """
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            raise NotFoundError(f"No recording named '{name}' on this robot.")

        shutil.rmtree(path)
        self.logger.info("[RecordingCatalogRepo] Deleted recording", recording=name)

    # --- Internals ----------------------------------------------------------

    def _read(self, name: str, path: str) -> Optional[StoredRecording]:
        """Describe one directory; None only if it disappeared mid-scan."""
        try:
            size_bytes, newest_mtime = _walk_stats(path)
        except FileNotFoundError:
            return None

        return StoredRecording(
            name=name,
            size_bytes=size_bytes,
            modified_at=datetime.fromtimestamp(newest_mtime, tz=timezone.utc),
            bag=self._read_metadata(name, path),
        )

    def _read_metadata(self, name: str, path: str) -> Optional[BagInfo]:
        """Read ``metadata.yaml``, or None if it says nothing usable.

        Never raises. This runs once per recording on every listing, and the
        recorder is writing this exact file as it does; a half-written or
        absent metadata file must degrade one entry to "unfinished", not fail
        the listing the console depends on. That is the same rule as
        MapCatalogRepo._read_grid_record, for the same reason.
        """
        metadata_path = os.path.join(path, METADATA_FILE)
        try:
            with open(metadata_path, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.logger.debug(
                "[RecordingCatalogRepo] Unreadable bag metadata",
                recording=name,
                error=str(exc),
            )
            return None

        if not isinstance(document, dict):
            return None
        info = document.get(_METADATA_ROOT)
        if not isinstance(info, dict):
            return None

        topics = []
        for entry in info.get("topics_with_message_count") or []:
            if not isinstance(entry, dict):
                continue
            metadata = entry.get("topic_metadata")
            if isinstance(metadata, dict) and isinstance(metadata.get("name"), str):
                topics.append(metadata["name"])

        # Nanoseconds in the file; seconds on the wire. The key is nested under
        # a one-field mapping (``duration: {nanoseconds: N}``), which is why
        # this is not a plain .get.
        duration = info.get("duration")
        nanoseconds = duration.get("nanoseconds") if isinstance(duration, dict) else None

        compression = info.get("compression_format")
        message_count = info.get("message_count")

        return BagInfo(
            duration_seconds=(
                nanoseconds / 1e9 if isinstance(nanoseconds, (int, float)) else 0.0
            ),
            message_count=message_count if isinstance(message_count, int) else 0,
            topics=topics,
            # rosbag2 writes "" rather than omitting the key for an
            # uncompressed bag; None reads better over the wire than an empty
            # string that looks like a codec nobody named.
            compression=compression or None,
        )


def _walk_stats(path: str) -> Tuple[int, float]:
    """Return total bytes and newest mtime under ``path``, dir itself included.

    A real walk rather than a stat of the directory entry, because a long run
    splits into several ``<name>_N.db3`` files and the interesting number is
    their sum. The same helper as the map catalogue's, kept here rather than
    shared for the reason resolve_dir gives.
    """
    total = 0
    newest = os.stat(path).st_mtime

    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                stats = os.stat(os.path.join(root, filename))
            except FileNotFoundError:
                # A file removed while we walked (a split being rotated); skip it.
                continue
            total += stats.st_size
            newest = max(newest, stats.st_mtime)

    return total, newest


def init_recording_catalog_repo(
    logger: structlog.stdlib.BoundLogger,
) -> RecordingCatalogRepo:
    return RecordingCatalogRepo(logger=logger)
