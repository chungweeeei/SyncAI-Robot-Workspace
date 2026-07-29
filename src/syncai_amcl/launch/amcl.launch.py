# Copyright (c) 2024 SyncAI
#
# Launch the non-lifecycle AMCL localization node.
#
# robot_id is read from the system config INI at launch time (same convention
# as system_manager.launch.py) and is used both as the node namespace and to
# rewrite the TF frame parameters (base_frame_id / odom_frame_id), since TF
# frame names are not namespaced by ROS.
#
# The same INI may provide an [initial_pose] section (x/y/z/yaw, map frame);
# when present it overrides the params-file initial_pose so each robot
# instance starts localized at its own spawn pose without editing amcl.yaml.

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

logger = launch_logging.get_logger("amcl.launch")


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


def read_initial_pose(config_path: str):
    """Read the [initial_pose] section (x/y/z/yaw, map frame) from the INI.

    Returns a dict of AMCL parameter overrides, or None when the section is
    absent or malformed (the params-file initial_pose then applies unchanged).
    """
    config = configparser.ConfigParser()
    if not config.read(config_path) or not config.has_section("initial_pose"):
        logger.info(
            f"No [initial_pose] in '{config_path}'; "
            "using the initial_pose from the params file"
        )
        return None

    try:
        pose = {
            key: config.getfloat("initial_pose", key, fallback=0.0)
            for key in ("x", "y", "z", "yaw")
        }
    except ValueError as err:
        logger.warning(
            f"Malformed [initial_pose] in '{config_path}' ({err}); "
            "using the initial_pose from the params file"
        )
        return None

    logger.info(f"Initial pose from '{config_path}': {pose}")
    return {
        "set_initial_pose": True,
        "initial_pose.x": pose["x"],
        "initial_pose.y": pose["y"],
        "initial_pose.z": pose["z"],
        "initial_pose.yaw": pose["yaw"],
    }


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    initial_pose_overrides = read_initial_pose(config_path)

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    overrides = {
        "use_sim_time": use_sim_time,
        # TF frame names are not namespaced by ROS, so override the
        # yaml defaults with the robot_id prefix here. Later entries
        # in this list take precedence over the params file.
        "base_frame_id": f"{robot_id}/base_link",
        "odom_frame_id": f"{robot_id}/odom",
    }
    if initial_pose_overrides:
        overrides.update(initial_pose_overrides)

    amcl_node = Node(
        package="syncai_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            overrides,
        ],
    )

    return [amcl_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_amcl")
    default_params_file = os.path.join(pkg_share, "params", "amcl.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true (no /clock on the real robot)",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the AMCL parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_use_sim_time,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
