# Offline map refinement (HBA) launch: starts hba_node and nothing else.
#
# Not part of any session. An operator runs it by hand after a mapping drive:
#   ros2 launch syncai_mapping hba.launch.py
#   ros2 service call /<robot_id>/hba/refine_map syncai_common/srv/RefineMap \
#     "{maps_path: '/abs/path/map/<name>'}"
#   ros2 service call /<robot_id>/hba/save_poses syncai_common/srv/SavePoses \
#     "{file_path: '/abs/path/poses_refined.txt'}"
# The optimisation runs on the node's timer after refine_map returns;
# /<robot_id>/hba/map_points shows it converge.
#
# Until 2026-09 this was SyncAI-Fast-LIO2's hba_launch.py, which ran the node
# at a bare /hba with no robot_id handling and started an rviz2 alongside it.
# The port gave it the workspace's robot_id namespace (a single DDS domain
# hosts several robots) and dropped the rviz2: the robot has no display, and
# workstation rviz configs live in config/rviz2/.
#
# hba_node's settings are plain ROS parameters (params/hba_params.yaml, keyed
# `/**/hba_node:`), none of which depend on robot_id, so the file is passed
# straight through.

import configparser
import os

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Same absolute default as mapping.launch.py / pointlio.launch.py.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("hba.launch")


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

    pkg_mapping = FindPackageShare("syncai_mapping").find("syncai_mapping")
    params_file = os.path.join(pkg_mapping, "params", "hba_params.yaml")

    # `name=` is fine: one node per process.
    return [
        launch_ros.actions.Node(
            package="syncai_mapping",
            namespace=f"{robot_id}/hba",
            executable="hba_node",
            name="hba_node",
            output="screen",
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
