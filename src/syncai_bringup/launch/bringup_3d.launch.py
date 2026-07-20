# Bringup for the 3D (FAST-LIO2) localization stack.
#
# Publishes:
#   * The G23 body TF tree via robot_state_publisher (reads the URDF under
#     description/). The URDF root link is base_link, so with frame_prefix the
#     legs hang directly off <robot_id>/base_link (odom->base_link comes from
#     LIO). Fixed joints (the 4 *_Ankle) go to /tf_static; the revolute leg
#     joints only appear on /tf once /joint_states is published by the robot
#     controller.
#   * Static TF <robot_id>/base_link -> <robot_id>/lidar_top (the 3D lidar mount
#     pose) needed to bridge the LIO pose onto the odom TF chain.
#
# robot_id is read from the system config INI at launch time, same convention as
# bringup_2d.launch.py.

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
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Load the URDF. robot_state_publisher only parses the kinematic tree for
    # TF; meshes (referenced with relative ../meshes/ paths) are never loaded
    # here, so the relative paths are harmless for TF publishing.
    urdf_path = os.path.join(
        get_package_share_directory("syncai_bringup"), "description", urdf_file
    )
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Body TF tree from the URDF. No node namespace so /tf and /tf_static stay
    # global (shared with the sim/LIO tree); frame_prefix namespaces the frames
    # to "<robot_id>/..." to match the rest of the stack.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_id,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "frame_prefix": f"{robot_id}/",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # Static TF: <robot_id>/base_link -> <robot_id>/lidar_top at the 3D lidar
    # mount height. No namespace on the node so it publishes to the GLOBAL
    # /tf_static shared with the sim's odom->base_link tree.
    # base_link_to_lidar_top_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="base_link_to_lidar_top_tf",
    #     namespace=robot_id,
    #     arguments=[
    #         "--x",
    #         "0.0",
    #         "--y",
    #         "0.0",
    #         "--z",
    #         lidar_height,
    #         "--qx",
    #         "0.0",
    #         "--qy",
    #         "0.0",
    #         "--qz",
    #         "0.0",
    #         "--qw",
    #         "1.0",
    #         "--frame-id",
    #         f"{robot_id}/base_link",
    #         "--child-frame-id",
    #         f"{robot_id}/lidar_top",
    #     ],
    # )

    return [
        robot_state_publisher,
        # base_link_to_lidar_top_tf,
    ]


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

    declare_urdf_file = DeclareLaunchArgument(
        "urdf_file",
        default_value="G23.urdf",
        description="URDF file name under syncai_bringup/description/ for "
        "robot_state_publisher (capsule collisions swapped to cylinder so "
        "urdfdom can parse it; TF-only, collision fidelity not needed)",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (set true when driving from Isaac Sim)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_lidar_height,
            declare_urdf_file,
            declare_use_sim_time,
            OpaqueFunction(function=launch_setup),
        ]
    )
