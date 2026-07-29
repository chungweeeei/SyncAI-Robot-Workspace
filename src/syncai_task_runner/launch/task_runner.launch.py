# Launch the non-lifecycle task_runner node — the BT action server that
# exposes the navigate_to_pose action and ticks the behavior tree. It talks to
# the planner (compute_path_to_pose) and controller (follow_path) action
# servers, so those must be running for a goal to succeed.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used both as the node namespace and to
# rewrite the base_frame parameter, since TF frame names are not namespaced
# by ROS. The global_frame stays "map".

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

logger = launch_logging.get_logger("task_runner.launch")


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

    # No `name=` here: the params file uses the /**/task_runner wildcard key so
    # it matches in any namespace. use_sim_time comes ONLY from the params file —
    # a launch override placed after the file would silently win over the YAML.
    task_runner_node = Node(
        package="syncai_task_runner",
        executable="task_runner",
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            {
                # TF frame names are not namespaced by ROS, so override the
                # yaml default with the robot_id prefix here. Later entries
                # in this list take precedence over the params file.
                "base_frame": f"{robot_id}/base_link",
            },
        ],
    )

    return [task_runner_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_task_runner")
    default_params_file = os.path.join(pkg_share, "params", "task_runner_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the task_runner parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
