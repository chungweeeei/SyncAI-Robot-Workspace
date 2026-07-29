# Launch the syncai_robot_state node — publishes syncai_common/RobotState at 1 Hz,
# with position from TF (map -> base_link) and forward velocity from the odom topic.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used as the node namespace, as the
# `robot_id` node parameter, and to rewrite the base_frame parameter, since TF
# frame names are not namespaced by ROS. The global_frame stays "map".
#
# The same INI may provide a [map] section with a "map" key (the map YAML
# path); when present it overrides the `map` node parameter so the published
# RobotState.map reflects the map this instance loaded, without editing
# robot_state_params.yaml. This mirrors map_server.launch.py.

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

logger = launch_logging.get_logger("robot_state.launch")


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


def read_map(config_path: str) -> str:
    """Read the [map] "map" key (the map YAML path) from the INI.

    Returns the raw value, or "" when the section or key is absent (the
    params-file `map` default then applies unchanged).
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""

    map_value = config.get("map", "map", fallback="").strip()
    if not map_value:
        logger.info(
            f"No [map] map in '{config_path}'; "
            "using the map value from the params file"
        )
        return ""

    logger.info(f"Map from '{config_path}': {map_value}")
    return map_value


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    map_value = read_map(config_path)

    params_file = LaunchConfiguration("params_file")

    # Override the yaml defaults with the INI robot_id. TF frame names are not
    # namespaced by ROS, so base_frame gets the robot_id prefix here. Later
    # entries in this list take precedence over the params file.
    overrides = {
        "robot_id": robot_id,
        "base_frame": f"{robot_id}/base_link",
    }
    # Only override `map` when the INI actually provides one, so an absent
    # [map] map leaves the params-file default in place.
    if map_value:
        overrides["map"] = map_value

    # No `name=`: the params file uses the /**/syncai_robot_state wildcard key
    # so it matches in any namespace.
    robot_state_node = Node(
        package="syncai_robot_state",
        executable="robot_state_node",
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            overrides,
        ],
    )

    return [robot_state_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_robot_state")
    default_params_file = os.path.join(pkg_share, "params", "robot_state_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the robot_state parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
