# Bringup for the FAST-LIO2 localization stack: the sensing / TF layer every
# other node assumes is already running. This is window 0 of
# scripts/byobu_session.sh — there is no lifecycle manager, so it must be up
# before syncai_lio_bridge.
#
# Publishes:
#   * The G23 body TF tree via robot_state_publisher (reads the URDF under
#     description/). The URDF root link is base_link, so with frame_prefix the
#     legs hang directly off <robot_id>/base_link (odom->base_link comes from
#     LIO). The fixed joints (the 4 *_Ankle plus lidar_top_joint) go to
#     /tf_static; the 12 revolute leg joints would only appear on /tf once
#     /joint_states is published, and nothing does that on purpose — live joint
#     angles reach their only consumer (the frontend 3D model) over
#     syncai_driver_manager's motor_states instead.
#   * lidar_top_joint carries the MID360 mount extrinsic (including the 0.25 rad
#     physical tilt), which syncai_lio_bridge looks up as
#     <robot_id>/base_link -> <robot_id>/lidar_top to map the LIO body pose onto
#     the odom TF chain. It used to be a separate static_transform_publisher
#     here, driven by a lidar_height launch argument; both were dropped when the
#     extrinsic moved into the URDF, because a height-only argument could not
#     carry the pitch.
#   * The Livox MID360 point cloud, via the livox_ros_driver2 node.
#
# robot_id is read from the system config INI at launch time, same convention as
# every other launch file in the stack.

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

# Livox MID360 driver configuration, mirrored from
# livox_ros_driver2/launch_ROS2/msg_MID360_launch.py so the 3D lidar publishes
# alongside the rest of the 3D localization stack.
LIVOX_XFER_FORMAT = 1  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
LIVOX_MULTI_TOPIC = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
LIVOX_DATA_SRC = 0  # 0-lidar, others-Invalid data src
LIVOX_PUBLISH_FREQ = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
LIVOX_OUTPUT_TYPE = 0
LIVOX_FRAME_ID = "laser"
LIVOX_LVX_FILE_PATH = "/home/livox/livox_test.lvx"
LIVOX_CMDLINE_BD_CODE = "livox0000000001"

logger = launch_logging.get_logger("bringup.launch")


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

    # Body TF tree from the URDF. The namespace only moves this node's name and
    # its relative topics (-> /<robot_id>/robot_description); it does NOT move
    # /tf or /tf_static, which tf2_ros always publishes on absolute names, so
    # the frames land in the one global tree shared with LIO. That is exactly
    # why frame_prefix is needed: a namespace does not prefix frame ids, so
    # without it two robots on one DDS domain would both claim "base_link".
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

    # Livox MID360 driver, brought over from
    # livox_ros_driver2/launch_ROS2/msg_MID360_launch.py. The user config JSON
    # lives in the livox_ros_driver2 share directory (installed via
    # INSTALL_TO_SHARE in its CMakeLists).
    livox_config_path = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "config",
        "MID360_config.json",
    )
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        namespace=robot_id,
        output="screen",
        parameters=[
            {"xfer_format": LIVOX_XFER_FORMAT},
            {"multi_topic": LIVOX_MULTI_TOPIC},
            {"data_src": LIVOX_DATA_SRC},
            {"publish_freq": LIVOX_PUBLISH_FREQ},
            {"output_data_type": LIVOX_OUTPUT_TYPE},
            {"frame_id": f"{robot_id}/{LIVOX_FRAME_ID}"},
            {"lvx_file_path": LIVOX_LVX_FILE_PATH},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": LIVOX_CMDLINE_BD_CODE},
        ],
    )

    return [robot_state_publisher, livox_driver]


def generate_launch_description():
    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
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
            declare_urdf_file,
            declare_use_sim_time,
            OpaqueFunction(function=launch_setup),
        ]
    )
