# Launch the non-lifecycle controller server (FollowPath action) with its
# internal local costmap and its controller plugins.
#
# robot_id is read from the system config INI at launch time (same convention
# as sys_manager.launch.py) and is used both as the node namespace and to
# rewrite the local costmap TF frame parameters (global_frame /
# robot_base_frame), since TF frame names are not namespaced by ROS.
#
# Two parameter files are loaded, in order: the factory params in this
# package (tracked, never written at runtime) and the robot's runtime overlay,
# config/params/<robot_id>/controller_server.yaml beside the system INI. The
# overlay holds only the keys the console's settings page changed, written by
# syncai_backend, so a later file wins key by key and an untouched key follows
# the factory file through a git pull. Deleting the overlay, or launching with
# use_runtime_params:=false, falls back to the factory values.

import configparser
import os

import yaml

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

logger = launch_logging.get_logger("controller_server.launch")


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


def count_leaves(node) -> int:
    if isinstance(node, dict):
        return sum(count_leaves(v) for v in node.values())
    return 1


def runtime_params(path: str):
    """Return the overlay file to load after the factory params, or None.

    A file that does not parse is skipped with an error rather than handed to
    rcl: rcl aborts the process on a malformed params file, so one bad write
    by the backend would otherwise keep controller_server down on every boot.
    Skipping it is the fault-back the overlay exists for.
    """
    if not os.path.isfile(path):
        logger.info(f"No runtime params overlay at '{path}'; factory params only")
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError("top level is not a mapping")
        keys = sum(
            count_leaves(section.get("ros__parameters", {}))
            for section in data.values()
            if isinstance(section, dict)
        )
    except (OSError, ValueError, yaml.YAMLError) as e:
        logger.error(
            f"Runtime params overlay '{path}' is unreadable ({e}); "
            "skipping it and loading the factory params only"
        )
        return None
    logger.info(f"Runtime params overlay '{path}' loaded: {keys} key(s) over the factory params")
    return path


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

    parameters = [params_file]
    if LaunchConfiguration("use_runtime_params").perform(context).lower() == "true":
        overlay_path = LaunchConfiguration("runtime_params_file").perform(context)
        if not overlay_path:
            overlay_path = os.path.join(
                os.path.dirname(config_path),
                "params",
                robot_id,
                "controller_server.yaml",
            )
        overlay = runtime_params(overlay_path)
        if overlay:
            parameters.append(overlay)
    else:
        logger.warning("use_runtime_params is false: runtime overlay ignored, factory params only")

    # NOTE: no `name=` here. The process hosts two nodes (controller_server
    # and its internal local_costmap); a launch-level name would remap BOTH
    # to the same name and the costmap would lose its parameters. The
    # namespace remap is fine: it applies to both nodes, and the params file
    # uses /**/ wildcard keys so they match in any namespace.
    # use_sim_time deliberately comes ONLY from the params file — a launch
    # override placed after the file would silently win over the YAML value.
    #
    # Because the Node has no `name`, the dict below is written under a /**
    # wildcard key, so it reaches the internal local_costmap node; the
    # controller_server node ignores these undeclared parameters.
    controller_server_node = Node(
        package="syncai_controller",
        executable="controller_server",
        namespace=robot_id,
        output="screen",
        # Factory params, then the runtime overlay, then the frame overrides:
        # a later entry wins, and the overlay never holds a frame.
        parameters=[
            *parameters,
            {
                # TF frame names are not namespaced by ROS, so override the
                # yaml defaults with the robot_id prefix here. Later entries
                # in this list take precedence over the params file.
                "global_frame": f"{robot_id}/odom",
                "robot_base_frame": f"{robot_id}/base_link",
                # Point-LIO's body (the lidar): the one frame that still
                # climbs, for tracking paths with heights (path_height.hpp).
                "body_frame": f"{robot_id}/pointlio_body",
            },
        ],
    )

    return [controller_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_controller")
    default_params_file = os.path.join(
        pkg_share, "params", "controller_server_params.yaml"
    )

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the controller server parameters YAML file",
    )

    declare_runtime_params_file = DeclareLaunchArgument(
        "runtime_params_file",
        default_value="",
        description=(
            "Runtime overlay YAML loaded after params_file; empty = "
            "<dir of system_config>/params/<robot_id>/controller_server.yaml"
        ),
    )

    declare_use_runtime_params = DeclareLaunchArgument(
        "use_runtime_params",
        default_value="true",
        description="false = ignore the runtime overlay and run on the factory params",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_runtime_params_file,
            declare_use_runtime_params,
            OpaqueFunction(function=launch_setup),
        ]
    )
