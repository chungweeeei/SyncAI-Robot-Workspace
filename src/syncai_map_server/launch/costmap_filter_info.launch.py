# Launch the costmap filter info server together with the filter mask server
# (a second map_server instance publishing the keepout mask OccupancyGrid).
#
# robot_id is read from the system config INI at launch time (same convention
# as map_server.launch.py) and is used as the node namespace, prefixing the
# relative topics (costmap_filter_info -> /<robot_id>/costmap_filter_info,
# keepout_filter_mask -> /<robot_id>/keepout_filter_mask). The published
# mask's frame_id stays "map" (the shared global frame).

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


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the nodes.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

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
        parameters=[params_file],
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

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
