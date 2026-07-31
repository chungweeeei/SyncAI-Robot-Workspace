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
# config/sessions/stack.yaml — there is no lifecycle manager, so it must be up
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
# every other launch file in the stack. The same INI also supplies the MID360's
# own IP ([sensor.lidar] ip), which is per-robot hardware identity and belongs
# next to robot_id rather than in a shared params file.
#
# Node parameters live in params/bringup.yaml under /**/ wildcard keys. Only the
# values that cannot be static are set here, appended after the file so they win:
# the URDF text, frame_prefix, the <robot_id>/-prefixed frame_id, and the path to
# the livox user config. use_sim_time is set ONLY in the YAML, per the workspace
# rule — a launch-level override placed after the file silently beats it.
#
# The livox driver takes its network config from a JSON file, not from ROS
# params, and that file needs two addresses: the lidar's and the host's. Both are
# per-deployment, so the JSON is generated here (write_livox_config) from the INI
# lidar IP + the params-file host_ip, and user_config_path points at the result.

import configparser
import json
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

# The lidar's raw sensor frame. Kept here rather than in the params file
# because it is the one livox setting the launch has to rewrite: TF frame names
# are not namespaced by ROS, so it ships as "<robot_id>/laser".
LIVOX_FRAME_ID = "laser"

# The node whose params carry the livox settings, and the params-file key the
# host IP is read from. The wildcard prefix matches the params YAML, which uses
# /**/ keys so one file works at any namespace.
LIVOX_NODE_NAME = "livox_lidar_publisher"
LIVOX_PARAMS_KEY = f"/**/{LIVOX_NODE_NAME}"

# Where the rendered livox user config is written. The vendor JSON that ships in
# the livox_ros_driver2 share directory is NOT used directly: it hardcodes one
# lidar IP and one host IP, and livox_ros_driver2 is an unmodified submodule, so
# editing it in place would be a local change that never survives a submodule
# update (and would be wrong for the second robot anyway). Instead the file is
# generated per robot_id at launch time — under /tmp because it is derived state
# with no value after the process exits, and because the workspace mount is
# shared with the host.
LIVOX_CONFIG_DIR = "/tmp/syncai_bringup"

# MID360 UDP ports, taken verbatim from the vendor MID360_config.json. These are
# fixed by the lidar's protocol, not per-deployment settings, which is why they
# live here as constants rather than in the params file: every livox sample
# config for the MID360 uses exactly these numbers. The host ports are the
# lidar-side port + 1.
LIDAR_NET_PORTS = {
    "cmd_data_port": 56100,
    "push_msg_port": 56200,
    "point_data_port": 56300,
    "imu_data_port": 56400,
    "log_data_port": 56500,
}

# 8 = kLivoxLidarType, the only value the MID360 accepts (lds_lidar.cpp gates
# the whole init path on this bit).
LIVOX_LIDAR_TYPE = 8

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


def read_lidar_ip(config_path: str) -> str:
    """Read [sensor.lidar] ip from the system INI.

    Unlike robot_id there is no usable fallback: a wrong lidar IP does not fail
    loudly, the driver just sits there publishing nothing while every downstream
    node waits on a cloud that never arrives. Failing the launch is far cheaper
    to debug than that, so this raises instead of warning.
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        raise RuntimeError(
            f"System config '{config_path}' not found; cannot resolve the "
            "MID360 lidar IP. Point system_config at a config/instances/*.ini."
        )

    lidar_ip = config.get("sensor.lidar", "ip", fallback="").strip()
    if not lidar_ip:
        raise RuntimeError(
            f"No [sensor.lidar] ip in '{config_path}'. Add the MID360's IP "
            "(the one printed on the lidar / set with Livox Viewer), e.g.\n"
            "  [sensor.lidar]\n  ip: 192.168.1.149"
        )

    return lidar_ip


def read_host_ip(params_path: str) -> str:
    """Read host_ip from the livox node's block in the params YAML.

    The host IP is the address of the machine's interface on the lidar's subnet
    — the driver binds its receive sockets to it, so it is per-robot wiring, not
    a property of the lidar. It lives in the params file (rather than the INI)
    because that is where every other livox setting already is; it is read back
    out here because the value is needed to render JSON, not to set a ROS param.
    """
    with open(params_path, "r") as f:
        params = yaml.safe_load(f) or {}

    host_ip = (
        params.get(LIVOX_PARAMS_KEY, {}).get("ros__parameters", {}).get("host_ip", "")
    )
    host_ip = str(host_ip).strip()
    if not host_ip:
        raise RuntimeError(
            f"No host_ip under '{LIVOX_PARAMS_KEY}' in '{params_path}'. Set it "
            "to this machine's IP on the lidar subnet (`ip -4 addr`)."
        )

    return host_ip


def write_livox_config(robot_id: str, lidar_ip: str, host_ip: str) -> str:
    """Render the livox user config JSON and return its path.

    Only the two IPs vary; everything else mirrors the vendor MID360_config.json.
    extrinsic_parameter stays all-zero on purpose — the mount extrinsic (the
    0.25 rad tilt included) lives on the URDF's lidar_top_joint, which is what
    syncai_lio_bridge looks up. Setting it here as well would apply the rotation
    twice.
    """
    config = {
        "lidar_summary_info": {"lidar_type": LIVOX_LIDAR_TYPE},
        "MID360": {
            "lidar_net_info": dict(LIDAR_NET_PORTS),
            "host_net_info": {
                "cmd_data_ip": host_ip,
                "cmd_data_port": LIDAR_NET_PORTS["cmd_data_port"] + 1,
                "push_msg_ip": host_ip,
                "push_msg_port": LIDAR_NET_PORTS["push_msg_port"] + 1,
                "point_data_ip": host_ip,
                "point_data_port": LIDAR_NET_PORTS["point_data_port"] + 1,
                "imu_data_ip": host_ip,
                "imu_data_port": LIDAR_NET_PORTS["imu_data_port"] + 1,
                # Left empty like the vendor file: the SDK's own debug log
                # stream is off unless an IP is given, and we do not want it.
                "log_data_ip": "",
                "log_data_port": LIDAR_NET_PORTS["log_data_port"] + 1,
            },
        },
        "lidar_configs": [
            {
                "ip": lidar_ip,
                # 1 = the 6-byte-per-point "extend" format the driver turns into
                # CustomMsg; pairs with the xfer_format: 1 in the params file.
                "pcl_data_type": 1,
                # 0 = non-repetitive scanning, the MID360's normal mode.
                "pattern_mode": 0,
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }

    # Per robot_id so two robots launched on one host (sim profile) cannot
    # clobber each other's config.
    os.makedirs(LIVOX_CONFIG_DIR, exist_ok=True)
    config_path = os.path.join(LIVOX_CONFIG_DIR, f"{robot_id}_MID360_config.json")
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    logger.info(
        f"Livox config for '{robot_id}': lidar {lidar_ip} -> host {host_ip} "
        f"(rendered to {config_path})"
    )
    return config_path


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    # Resolved as well as passed through: the nodes get the file itself, and
    # write_livox_config needs one value out of it (see read_host_ip).
    params_path = LaunchConfiguration("params_file").perform(context)
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
    # carries the network wiring the ROS params cannot express (lidar IP, host
    # IP, UDP ports); it is rendered here from the two per-robot inputs instead
    # of read out of the vendor share directory — see write_livox_config.
    livox_config_path = write_livox_config(
        robot_id, read_lidar_ip(config_path), read_host_ip(params_path)
    )
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name=LIVOX_NODE_NAME,
        namespace=robot_id,
        output="screen",
        parameters=[
            params_file,
            {
                # Both are derived from the resolved robot_id, so they are
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
