# Bringup for the 3D (FAST-LIO2) localization stack.
#
# Only publishes the static TF <robot_id>/base_link -> <robot_id>/lidar_top
# (the 3D lidar mount pose) needed to bridge the LIO pose onto the wheel-odom
# TF chain. robot_id is read from the system config INI at launch time, same
# convention as bringup_2d.launch.py.

import configparser

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

logger = launch_logging.get_logger("bringup_3d.launch")


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
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    lidar_height = LaunchConfiguration("lidar_height")

    # Static TF: <robot_id>/base_link -> <robot_id>/lidar_top at the 3D lidar
    # mount height. No namespace on the node so it publishes to the GLOBAL
    # /tf_static shared with the sim's odom->base_link tree.
    base_link_to_lidar_top_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_top_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            lidar_height,
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
            f"{robot_id}/lidar_top",
        ],
    )

    return [base_link_to_lidar_top_tf]


def generate_launch_description():
    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_lidar_height = DeclareLaunchArgument(
        "lidar_height",
        default_value="0.196",
        description="3D lidar mount height (m) for the <robot_id>/base_link -> "
        "<robot_id>/lidar_top TF (matches the current sim-published value)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_lidar_height,
            OpaqueFunction(function=launch_setup),
        ]
    )
