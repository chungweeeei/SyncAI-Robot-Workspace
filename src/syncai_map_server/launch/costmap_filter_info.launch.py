# Launch the costmap filter info server together with the filter mask server
# (a second map_server instance publishing the keepout mask OccupancyGrid).
#
# robot_id is read from the system config INI at launch time (same convention
# as map_server.launch.py) and is used as the node namespace, prefixing the
# relative topics (costmap_filter_info -> /<robot_id>/costmap_filter_info,
# keepout_filter_mask -> /<robot_id>/keepout_filter_mask). The published
# mask's frame_id stays "map" (the shared global frame).
#
# WHERE THE MASK COMES FROM. The keepout mask lives with the map it belongs to:
# `map/<name>/keepout.yaml` (+ `keepout.pgm`), an ordinary map-server YAML/image
# pair in the same directory as `gridmap.yaml`. The path is derived from the
# INI's `[map] map` value (its directory + "keepout.yaml"), so switching maps
# switches the keepout with it and the instance INI needs no new key -- the INI
# is a file the backend writes, so adding a key there would be a cross-repo
# contract change. Like `[map] map` itself the path is relative to the workspace
# root, which is every pane's cwd. A `mask_yaml:=` argument overrides the
# derivation for tests and by-hand runs.
#
# A MISSING MASK STARTS NOTHING. map_server throws in its constructor when the
# file is absent, and a keepout is optional per map (no map had one when the
# filter was wired in, 2026-09), so this launch checks the file first and
# returns an empty description when it is not there -- the same pattern as
# syncai_localizer's launch for a missing `[map] pcd`. Neither node starts in
# that case: an info server without a mask server would give the planner's
# KeepoutFilter nothing more useful than no info server at all, and an empty
# pane is the clearer signal. The other half of that signal is the filter's
# throttled "Filter mask was not received" warning in the planner log, every
# 2 s -- expected on a map without a keepout, accepted as noise for now.
#
# Filters run AFTER the layer stack, inflation included, so keepout cells land
# as lethal cost but are never inflated: the mask must carry its own footprint
# margin around each zone (see syncai_costmap_2d's README).

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started
# from — the old relative path only worked because every entrypoint happened
# to run from the workspace root. ~/robot_ws is the workspace inside the robot
# container, where docker-compose bind-mounts the per-robot instance INI over
# config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"
# Basename of the keepout mask YAML inside a map directory (next to
# gridmap.yaml). The image it names (keepout.pgm by convention) is resolved by
# map_io relative to the YAML, so nothing inside the map directory holds a path
# that would break a map rename -- the same property the rest of map/<name>/
# keeps, and one the backend relies on.
KEEPOUT_YAML_BASENAME = "keepout.yaml"

logger = launch_logging.get_logger("costmap_filter_info.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def read_keepout_yaml(config_path: str) -> str:
    """Derive the keepout mask YAML path from the INI's [map] map value.

    Returns `<dirname of [map] map>/keepout.yaml`, or "" when the INI or the
    key is absent (the caller then falls back to the params-file value).
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""

    map_yaml = config.get("map", "map", fallback="").strip()
    if not map_yaml:
        logger.info(
            f"No [map] map in '{config_path}'; "
            "using the yaml_filename from the params file for the keepout mask"
        )
        return ""

    return os.path.join(os.path.dirname(map_yaml), KEEPOUT_YAML_BASENAME)


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the nodes.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

    # Mask path precedence: explicit mask_yaml:= argument > derived from the
    # INI's [map] map > the params file. The first two are checked for
    # existence here; the params-file fallback is left to map_server itself
    # (it throws on a bad path), because that path only applies when the node
    # is being run outside the robot convention and its author knows why.
    keepout_yaml = LaunchConfiguration("mask_yaml").perform(context).strip()
    if not keepout_yaml:
        keepout_yaml = read_keepout_yaml(config_path)

    mask_overrides = []
    if keepout_yaml:
        if not os.path.isfile(keepout_yaml):
            logger.error(
                f"Keepout mask '{keepout_yaml}' does not exist; nothing to launch. "
                "A map without a keepout is fine -- the planner's KeepoutFilter "
                "will warn that no mask was received and otherwise do nothing."
            )
            return []
        logger.info(f"Keepout mask yaml: {keepout_yaml}")
        # Later entries in the parameters list take precedence over the params
        # file, exactly as map_server.launch.py layers [map] map on top of it.
        mask_overrides = [{"yaml_filename": keepout_yaml}]

    costmap_filter_info_server_node = Node(
        package="syncai_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        namespace=robot_id,
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    # Second map_server instance publishing the filter mask OccupancyGrid.
    filter_mask_server_node = Node(
        package="syncai_map_server",
        executable="map_server",
        name="filter_mask_server",
        namespace=robot_id,
        output="screen",
        emulate_tty=True,
        parameters=[params_file, *mask_overrides],
    )

    return [costmap_filter_info_server_node, filter_mask_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_map_server")
    default_params_file = os.path.join(
        pkg_share, "params", "costmap_filter_info_params.yaml"
    )

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file for both nodes",
    )

    declare_mask_yaml = DeclareLaunchArgument(
        "mask_yaml",
        default_value="",
        description=(
            "Keepout mask YAML to serve. Empty (the default) derives "
            "<dirname of [map] map>/keepout.yaml from the system INI; either "
            "way the launch starts nothing when the file does not exist."
        ),
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_mask_yaml,
            OpaqueFunction(function=launch_setup),
        ]
    )
