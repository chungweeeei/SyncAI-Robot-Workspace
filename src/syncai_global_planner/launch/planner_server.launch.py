# Launch the non-lifecycle planner server (ComputePathToPose action) with its
# internal global costmap.
#
# robot_id is read from the system config INI at launch time (same convention
# as sys_manager.launch.py) and is used both as the node namespace and to
# rewrite the global costmap TF frame parameters (robot_base_frame /
# sensor_frame), since TF frame names are not namespaced by ROS. The costmap's
# global_frame stays "map" (the shared global frame) and is not rewritten.
#
# The same INI's [map] pcd locates the map directory, from which the OctoMap
# planner's octomap.bt is derived (the way costmap_filter_info.launch.py
# derives keepout.yaml from [map] map): no new INI key, so the backend's
# one-line map switch is unaffected. A map without the file still launches;
# the Octo3D plugin then logs one error and fails its own plans only.

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


def read_map_pcd(config_path: str) -> str:
    """[map] pcd from the INI, made absolute against the workspace root.

    The instance INI writes it as `map/%(name)s/map.pcd` (relative to the
    workspace root), and the INI sits at <workspace>/config/system.ini, so two
    dirname()s up from it is the root. Same helper as localizer.launch.py's.
    An empty value is returned as-is so the caller can say "no map".
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    pcd = config.get("map", "pcd", fallback="").strip()
    if not pcd or os.path.isabs(pcd):
        return pcd
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, pcd)


def read_map_octomap(config_path: str) -> str:
    """map/<name>/octomap.bt next to [map] pcd, or "" when there is no map.

    The file is clean_map's (`--octomap 1` / map_clean_octomap) and is not
    written for every save, so its absence is logged, not fatal: the plugin
    is still loaded and reports map_not_loaded on its own plans.
    """
    pcd = read_map_pcd(config_path)
    if not pcd:
        logger.warning(
            f"No [map] pcd in '{config_path}'; the Octo3D planner has no octomap.bt"
        )
        return ""
    octomap = os.path.join(os.path.dirname(pcd), "octomap.bt")
    if not os.path.isfile(octomap):
        logger.warning(
            f"'{octomap}' does not exist; the Octo3D planner will fail every plan "
            "with map_not_loaded until clean_map writes it (ros2 run syncai_mapping "
            "clean_map map/<name> --octomap 1)"
        )
    return octomap


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    octomap_file = read_map_octomap(config_path)

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
        package="syncai_global_planner",
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
                "obstacle_layer.scan.sensor_frame": f"{robot_id}/laser",
                # OctomapPlanner3D (planner id Octo3D): the tree next to the
                # map's pcd, and the LIO body frame it reads the robot's real
                # height from (map -> base_link is planar, z = 0).
                "Octo3D.octomap_file": octomap_file,
                "Octo3D.body_frame": f"{robot_id}/pointlio_body",
            },
        ],
    )

    return [planner_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_global_planner")
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
