# Launch the syncai_map_server "map_server" node.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used as the node namespace, prefixing
# the relative map topic (topic_name "map" -> /<robot_id>/map). The published
# OccupancyGrid's frame_id stays "map" (the shared global frame), so no frame
# parameters are rewritten here.
#
# The same INI may provide a [map] section with a "map" key (the map YAML
# path); when present it overrides the params-file yaml_filename so each robot
# instance loads its own map without editing map_server_params.yaml.

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same convention as the backend gateways: processes run with the workspace
# root as their working directory, so a relative path works both inside the
# robot container and when launching from the workspace root.
DEFAULT_SYSTEM_INI = "config/system.ini"
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("map_server.launch")


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


def read_map_yaml(config_path: str):
    """Read the [map] "map" key (the map YAML path) from the INI.

    Returns a dict of map_server parameter overrides, or None when the section
    or key is absent (the params-file yaml_filename then applies unchanged).
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return None

    yaml_filename = config.get("map", "map", fallback="").strip()
    if not yaml_filename:
        logger.info(
            f"No [map] map in '{config_path}'; "
            "using the yaml_filename from the params file"
        )
        return None

    logger.info(f"Map yaml from '{config_path}': {yaml_filename}")
    return {"yaml_filename": yaml_filename}


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    map_yaml_overrides = read_map_yaml(config_path)

    params_file = LaunchConfiguration("params_file")

    # Later entries in this list take precedence over the params file.
    parameters = [params_file]
    if map_yaml_overrides:
        parameters.append(map_yaml_overrides)

    map_server_node = Node(
        package="syncai_map_server",
        executable="map_server",
        name="map_server",
        namespace=robot_id,
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )

    return [map_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_map_server")
    default_params_file = os.path.join(pkg_share, "params", "map_server_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file for the map_server node",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
