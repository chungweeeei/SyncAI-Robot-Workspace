# Top-level launch for bringing up the SyncAI navigation stack.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used as the namespace for the merger
# nodes and as the TF prefix for the static base_link -> scan transform. The
# third-party merger launch file still takes a `namespace` argument, so the
# resolved robot_id is forwarded to it.

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same convention as the backend gateways: processes run with the workspace
# root as their working directory, so a relative path works both inside the
# robot container and when launching from the workspace root.
DEFAULT_SYSTEM_INI = "config/system.ini"
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("bringup_2d.launch")


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
    # need the resolved robot_id here to namespace the nodes and TF frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    scan_height = LaunchConfiguration("scan_height")

    laser_scan_merger_dir = get_package_share_directory("ros2_laser_scan_merger")

    # scan_front + scan_rear -> merged /<robot_id>/scan (LaserScan)
    merge_laser_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_scan_merger_dir, "launch", "merge_2_scan.launch.py")
        ),
        launch_arguments={"namespace": robot_id}.items(),
    )

    # Static TF: <robot_id>/base_link -> <robot_id>/scan at the lidar mount
    # height. No namespace on the node so it publishes to the GLOBAL /tf
    # shared with the sim's odom->base_link tree.
    base_link_to_scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_scan_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            scan_height,
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            f"{robot_id}/base_link",
            "--child-frame-id",
            f"{robot_id}/scan",
        ],
    )

    return [merge_laser_scan, base_link_to_scan_tf]


def generate_launch_description():
    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_scan_height = DeclareLaunchArgument(
        "scan_height",
        default_value="0.1225",
        description="Lidar mount height (m) for the <robot_id>/base_link -> "
        "<robot_id>/scan TF (0.175 * 0.7 = 0.1225)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_scan_height,
            OpaqueFunction(function=launch_setup),
        ]
    )
