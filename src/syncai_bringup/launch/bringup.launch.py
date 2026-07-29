# Copyright (c) 2026 SyncAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
#
# Node parameters live in params/bringup.yaml under /**/ wildcard keys. Only the
# values that cannot be static are set here, appended after the file so they win:
# the URDF text, frame_prefix, the <robot_id>/-prefixed frame_id, and the path to
# the livox user config. use_sim_time is set ONLY in the YAML, per the workspace
# rule — a launch-level override placed after the file silently beats it.

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

# The lidar's raw sensor frame. Kept here rather than in the params file
# because it is the one livox setting the launch has to rewrite: TF frame names
# are not namespaced by ROS, so it ships as "<robot_id>/laser".
LIVOX_FRAME_ID = "laser"

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
    params_file = LaunchConfiguration("params_file")

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
            params_file,
            {
                # Computed here, so appended after the params file to win over
                # it: the URDF text read off disk, and the frame prefix which
                # needs the resolved robot_id.
                "robot_description": robot_description,
                "frame_prefix": f"{robot_id}/",
            },
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
            params_file,
            {
                # Both need the resolved robot_id / share dir, so they are
                # appended after the params file and take precedence over it.
                "frame_id": f"{robot_id}/{LIVOX_FRAME_ID}",
                "user_config_path": livox_config_path,
            },
        ],
    )

    return [robot_state_publisher, livox_driver]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("syncai_bringup"), "params", "bringup.yaml"
    )

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the parameters YAML for robot_state_publisher "
        "and the MID360 driver",
    )

    declare_urdf_file = DeclareLaunchArgument(
        "urdf_file",
        default_value="G23.urdf",
        description="URDF file name under syncai_bringup/description/ for "
        "robot_state_publisher (capsule collisions swapped to cylinder so "
        "urdfdom can parse it; TF-only, collision fidelity not needed)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_urdf_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
