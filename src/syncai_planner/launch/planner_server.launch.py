# Launch the non-lifecycle planner server (ComputePathToPose action) with its
# internal global costmap.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used both as the node namespace and to
# rewrite the global costmap TF frame parameters (robot_base_frame /
# sensor_frame), since TF frame names are not namespaced by ROS. The costmap's
# global_frame stays "map" (the shared global frame) and is not rewritten.

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

logger = launch_logging.get_logger("planner_server.launch")


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

    # NOTE: no `name=` here. The process hosts two nodes (planner_server and
    # its internal global_costmap); a launch-level name would remap BOTH to
    # the same name and the costmap would lose its parameters. The namespace
    # remap is fine: it applies to both nodes, and the params file uses /**/
    # wildcard keys so they match in any namespace.
    # use_sim_time deliberately comes ONLY from the params file — a launch
    # override placed after the file would silently win over the YAML value.
    #
    # Because the Node has no `name`, the dict below is written under a /**
    # wildcard key, so it reaches the internal global_costmap node; the
    # planner_server node ignores these undeclared parameters.
    planner_server_node = Node(
        package="syncai_planner",
        executable="planner_server",
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            {
                # TF frame names are not namespaced by ROS, so override the
                # yaml defaults with the robot_id prefix here. Later entries
                # in this list take precedence over the params file.
                "robot_base_frame": f"{robot_id}/base_link",
                "obstacle_layer.scan.sensor_frame": f"{robot_id}/scan",
            },
        ],
    )

    return [planner_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_planner")
    default_params_file = os.path.join(
        pkg_share, "params", "planner_server_params.yaml"
    )

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the planner server parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
