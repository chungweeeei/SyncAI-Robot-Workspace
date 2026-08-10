"""Read the per-robot system INI.

``config/system.ini`` is the workspace's identity file: every launch file reads
``[system] robot_id`` from it, and docker-compose bind-mounts
``config/instances/robotNN.ini`` over it inside the container. ``[map]`` in that
same file is what decides which map the stack loads.

``gateways/artifact/artifact.py`` used to carry its own copy of the path
constant and its own ``configparser`` read; it went away with the artifact
integration (2026-08), leaving this module the only INI reader. If a second
reader ever appears, consolidate it here rather than pasting again.
"""

import configparser
import os
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
