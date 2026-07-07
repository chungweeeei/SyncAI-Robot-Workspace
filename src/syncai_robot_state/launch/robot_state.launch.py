# Launch the syncai_robot_state node — publishes syncai_common/RobotState at 1 Hz,
# with position from TF (map -> base_link) and forward velocity from the odom topic.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used as the node namespace, as the
# `robot_id` node parameter, and to rewrite the base_frame parameter, since TF
# frame names are not namespaced by ROS. The global_frame stays "map".

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


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

    # No `name=`: the params file uses the /**/syncai_robot_state wildcard key
    # so it matches in any namespace.
    robot_state_node = Node(
        package="syncai_robot_state",
        executable="robot_state_node",
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            {
                # Override the yaml defaults with the INI robot_id. TF frame
                # names are not namespaced by ROS, so base_frame gets the
                # robot_id prefix here. Later entries in this list take
                # precedence over the params file.
                "robot_id": robot_id,
                "base_frame": f"{robot_id}/base_link",
            },
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
