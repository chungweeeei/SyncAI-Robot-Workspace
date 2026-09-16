"""Read and write the per-robot system INI.

``config/system.ini`` is the workspace's identity file: every launch file reads
``[system] robot_id`` from it, and docker-compose bind-mounts
``config/instances/robotNN.ini`` over it inside the container. ``[map]`` in that
same file is what decides which map the stack loads.

``gateways/artifact/artifact.py`` used to carry its own copy of the path
constant and its own ``configparser`` read; it went away with the artifact
integration (2026-08), leaving this module the only INI reader. If a second
reader ever appears, consolidate it here rather than pasting again.

This module was read-only by design until the maps library grew a Switch-map
control: `set_active_map` is the writer that pairs with it, and it is
deliberately the *only* one. Both halves of "which map is the robot on" now live
here -- read by `active_map_name`, moved by `set_active_map` -- because they have
to agree about the interpolation in ``[map]``, and a second writer elsewhere
would be free not to.
"""

import configparser
import os
import re
from typing import Optional

import structlog

# Absolute path so the INI resolves no matter what cwd the process is started
# from. ~/robot_ws is the workspace inside the robot container.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")

SYSTEM_INI_ENV = "SYNCAI_SYSTEM_INI"


def system_ini_path() -> str:
    return os.environ.get(SYSTEM_INI_ENV, DEFAULT_SYSTEM_INI)


def active_map_name(logger: structlog.stdlib.BoundLogger) -> Optional[str]:
    """Return the bare name of the map the stack was launched with, or None.

    Read from the INI rather than from the cached ``RobotState``, even though
    ``RobotState.map`` carries the same value: ``RobotStateSubscriber`` only
    caches samples whose localization is valid, so that field is unavailable
    until relocalization has run, and a freshly booted robot would then report
    none of its maps as active.

    ``[map] name`` is the canonical key. Older instance files only set the
    derived paths, so fall back to the parent directory of ``[map] map``
    (``map/dp2f/gridmap.yaml`` -> ``dp2f``).

    Never raises. A robot with no INI, or one whose INI has no ``[map]``, must
    still be able to list the maps sitting on its disk.
    """
    path = system_ini_path()
    config = configparser.ConfigParser()

    try:
        if not config.read(path):
            logger.warning("[SystemConfig] System INI not found", path=path)
            return None
    except configparser.Error as exc:
        logger.warning("[SystemConfig] System INI is malformed", path=path,
                       error=str(exc))
        return None

    name = config.get("map", "name", fallback="").strip()
    if name:
        return name

    yaml_path = config.get("map", "map", fallback="").strip()
    if yaml_path:
        return os.path.basename(os.path.dirname(yaml_path)) or None

    logger.warning("[SystemConfig] No [map] name in system INI", path=path)
    return None


# The keys `set_active_map` rewrites, by section. `[map] name` is the switch
# itself; the `[initial_pose]` reset is what stops a pose measured in the old
# map's frame from being applied as a seed in the new one.
#
# localizer_launch.py's read_initial_pose() reads x/y/yaw with fallback=0.0 per
# key, so a key this does not find is already the value being written. z is here
# because the section carries it even though the launch ignores it.
_MAP_SECTION = "map"
_MAP_NAME_KEY = "name"
_INITIAL_POSE_SECTION = "initial_pose"
_INITIAL_POSE_RESET = {"x": "0.0", "y": "0.0", "z": "0.0", "yaw": "0.0"}

_SECTION_RE = re.compile(r"^\s*\[(?P<name>[^]]+)\]\s*$")

# Splits `  name: dp2f  ` into indent / key / separator / value / trailing, so a
# rewrite can replace the value and put every other byte of the line back.
_KEY_RE = re.compile(
    r"^(?P<indent>\s*)(?P<key>[^\s:=#;][^:=]*?)(?P<sep>\s*[:=]\s*)"
    r"(?P<value>.*?)(?P<trail>\s*)$"
)


def set_active_map(name: str, logger: structlog.stdlib.BoundLogger) -> str:
    """Point the INI at ``name`` and zero ``[initial_pose]``. Returns the path.

    The counterpart to `active_map_name`, and the only thing in the backend that
    writes this file. It exists because `POST /api/v1/maps/{name}/activate` swaps
    the running localizer and map_server in place: without the INI write the swap
    would not survive a stack restart, and `active_map_name` would go on naming
    the old map to every caller that asks which one is active.

    Two things here look like mistakes and are not.

    **The file is rewritten in place, never through a temp file and a rename.**
    docker-compose bind-mounts ``config/instances/robotNN.ini`` onto
    ``config/system.ini`` as a *single file*. `os.rename` gives the new content a
    new inode; the bind mount goes on resolving the old one, so the container
    would keep reading the pre-edit file forever while the host file sat
    orphaned. The usual atomic-write idiom is exactly wrong for this path. The
    cost is a torn file if the process dies mid-write, which the re-read below
    turns into a loud failure rather than a silent one.

    **Lines are edited individually rather than round-tripped through
    ConfigParser.write().** ``[map]`` is written with interpolation
    (``pcd: map/%(name)s/map.pcd``), so a plain read expands it and a write-back
    would bake the resolved name in permanently -- destroying the property that
    makes this file editable by hand, and silently pinning pcd/map at the *old*
    map while `name` moved on. Rewriting one line at a time also keeps comments,
    key order and the file's ``:`` delimiters, which a git-tracked per-robot
    identity file deserves.

    Raises OSError if the file cannot be read or written, and ValueError if it
    has no ``[map] name`` to rewrite (the caller turns both into a 5xx; the
    router checks W_OK up front so the common case never reaches here).
    """
    path = system_ini_path()

    with open(path, "r", encoding="utf-8") as handle:
        lines = handle.readlines()

    section = ""
    seen_name = False
    out: list[str] = []

    for line in lines:
        match = _SECTION_RE.match(line)
        if match:
            section = match.group("name").strip().lower()
            out.append(line)
            continue

        replacement = None
        if section == _MAP_SECTION:
            replacement = {_MAP_NAME_KEY: name}
        elif section == _INITIAL_POSE_SECTION:
            replacement = _INITIAL_POSE_RESET

        if replacement is None:
            out.append(line)
            continue

        key_match = _KEY_RE.match(line.rstrip("\n"))
        if key_match is None:
            # Blank line, comment, or a continuation of a multi-line value.
            out.append(line)
            continue

        key = key_match.group("key").strip().lower()
        if key not in replacement:
            out.append(line)
            continue

        if section == _MAP_SECTION:
            seen_name = True

        newline = "\n" if line.endswith("\n") else ""
        out.append(
            key_match.group("indent")
            + key_match.group("key")
            + key_match.group("sep")
            + replacement[key]
            + key_match.group("trail")
            + newline
        )

    if not seen_name:
        raise ValueError(f"{path} has no [{_MAP_SECTION}] {_MAP_NAME_KEY} to rewrite")

    with open(path, "w", encoding="utf-8") as handle:
        handle.writelines(out)

    # Read it back before telling the caller the switch is recorded. interpolation
    # is off on purpose: this asserts the raw `%(name)s` survived, which a parser
    # that expands it could not tell us, and it is the one property that makes the
    # next hand edit a one-line change.
    check = configparser.ConfigParser(interpolation=None)
    try:
        if not check.read(path):
            raise ValueError(f"{path} disappeared while being rewritten")
    except configparser.Error as exc:
        raise ValueError(f"{path} is malformed after the rewrite: {exc}") from exc

    written = check.get(_MAP_SECTION, _MAP_NAME_KEY, fallback="").strip()
    if written != name:
        raise ValueError(
            f"{path} still reads [{_MAP_SECTION}] {_MAP_NAME_KEY} = {written!r} "
            f"after being rewritten to {name!r}"
        )

    logger.info("[SystemConfig] Active map written to the system INI",
                path=path, map=name)
    return path
