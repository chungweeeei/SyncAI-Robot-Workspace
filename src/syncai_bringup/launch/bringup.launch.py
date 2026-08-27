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
#   * The Livox MID360 / MID360s point cloud, via the livox_ros_driver2 node.
#   * Optionally the TechNexion VCS-AR0234-C camera's IMU / image / camera_info,
#     via vizionsdk_ros2. Off by default — see the use_camera argument.
#
# The camera is the one sensor here that is NOT started by default, and that is
# a deliberate regression guard rather than an oversight. /dev/video0 is a V4L2
# capture device that admits exactly one streaming opener, and it already has
# one: scripts/publish_camera.sh feeds MediaMTX over RTSP, which is what the
# frontend's WebRTC view consumes. bringup.launch.py is window 0 of BOTH session
# specs, so defaulting the camera node on would take that stream away from every
# robot in the fleet the next time it came up, and it would fail the quiet way —
# gstreamer dies at S_FMT with "Device or resource busy" long after the pane has
# scrolled. Turn it on per-run with use_camera:=true once you have decided which
# of the two consumers owns the device.
#
# robot_id is read from the system config INI at launch time, same convention as
# every other launch file in the stack. The same INI also supplies the lidar's
# own IP and model ([sensor.lidar] ip / type), which are per-robot hardware
# identity and belong next to robot_id rather than in a shared params file.
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
#
# That JSON is also the only place the lidar *model* is selected. The fleet runs
# both MID360 and MID360s units and livox_ros_driver2 has no ROS parameter for
# the model — the vendor msg_MID360_launch.py and msg_MID360s_launch.py differ
# in exactly one line, which config JSON they point at. So [sensor.lidar] type
# picks the schema write_livox_config emits; see LIVOX_MODELS.

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

# 8 = kLivoxLidarType, the driver's own lidar-family bitmask; lds_lidar.cpp
# gates the whole init path on this bit. It is NOT the device model — the vendor
# MID360_config.json and MID360s_config.json both carry 8, and the real device
# type comes from the section name below.
LIVOX_LIDAR_TYPE = 8

# INI [sensor.lidar] type -> the JSON section name the Livox SDK matches on.
#
# That section name IS the model selector: Livox-SDK2's parse_cfg_file.cpp maps
# {"HAP", "MID360", "Mid360s"} onto device types 10 / 9 / 35 and looks each one
# up with a case-sensitive doc.HasMember(), so these strings have to be copied
# verbatim from its dev_type_map. device_type is not cosmetic either — it decides
# whether the SDK opens a broadcast socket (device_manager.cpp), which lidar-side
# ports it talks to, and which command handler it drives (mid360s has a separate
# implementation). Getting it wrong raises nothing: the driver negotiates with
# the wrong handler and simply never publishes a cloud.
LIVOX_MODELS = {"mid360": "MID360", "mid360s": "Mid360s"}

# What a missing or unrecognised [sensor.lidar] type falls back to, rather than
# failing the launch — see read_lidar_type for why it is a warning and not a
# raise.
FALLBACK_LIDAR_MODEL = "mid360"

# The VizionSDK camera node's name, which doubles as the key its params block is
# matched on (/**/vizionsdk_camera in params/bringup.yaml). Kept identical to
# the vendor launch file's node_name default so the upstream README's
# `ros2 param set /<ns>/vizionsdk_camera isp.<x> <v>` recipes still address the
# right node — the ISP controls are applied live by an on_set_parameters
# callback, so that is the normal way to tune this camera.
CAMERA_NODE_NAME = "vizionsdk_camera"

# The camera's TF frames. Same story as LIVOX_FRAME_ID: frame names are not
# namespaced by ROS, so these are shipped as "<robot_id>/..." from here and win
# over the fallbacks in the params file.
#
# Nothing parents either frame yet. description/G23.urdf has no camera link, so
# both land in the global tree as roots and anything that tries to transform an
# image or the camera IMU into <robot_id>/base_link fails its lookup. That is
# survivable while the camera is only a video source (the WebRTC path does not
# use TF at all), but it has to be fixed before the camera is used for anything
# metric. The fix belongs on a URDF joint next to lidar_top_joint, not on a
# static_transform_publisher here — that is exactly the mistake the lidar mount
# extrinsic was moved out of this file to undo, because an argument-driven
# transform could not carry the full pose.
CAMERA_OPTICAL_FRAME = "camera_optical_frame"
CAMERA_IMU_FRAME = "camera_imu_link"

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


def read_lidar_type(config_path: str) -> str:
    """Read [sensor.lidar] type from the system INI as a LIVOX_MODELS key.

    Warns and falls back instead of raising, unlike read_lidar_ip: the whole
    fleet was MID360 before the first MID360s arrived, so every already-deployed
    instance INI predates this key and has to keep working untouched. The price
    is that a MID360s whose INI forgot the key fails the quiet way — no cloud,
    no error — so the warning spells out both the value it read and the accepted
    set, to leave a trace in the bringup log for whoever goes looking.

    Spelling is normalised (case, dashes, underscores, spaces) so mid360s /
    Mid360s / MID-360S / "mid 360s" all resolve to the same key; the INI is
    hand-edited per robot and the model is the one field with two very similar
    legal values.
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; falling back to lidar "
            f"model '{FALLBACK_LIDAR_MODEL}'"
        )
        return FALLBACK_LIDAR_MODEL

    raw = config.get("sensor.lidar", "type", fallback="").strip()
    model = raw.lower().replace("-", "").replace("_", "").replace(" ", "")
    if model in LIVOX_MODELS:
        return model

    known = ", ".join(sorted(LIVOX_MODELS))
    if not raw:
        logger.warning(
            f"No [sensor.lidar] type in '{config_path}'; falling back to "
            f"'{FALLBACK_LIDAR_MODEL}'. Add it (one of: {known}) — a MID360s "
            "left on the MID360 default publishes nothing and reports nothing."
        )
    else:
        logger.warning(
            f"Unknown [sensor.lidar] type '{raw}' in '{config_path}'; falling "
            f"back to '{FALLBACK_LIDAR_MODEL}'. Expected one of: {known}."
        )

    return FALLBACK_LIDAR_MODEL


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


def render_host_net_info(model: str, host_ip: str):
    """Build the model's host_net_info block, mirroring its vendor config file.

    The MID360 uses an object with one IP field per stream; the MID360s uses an
    array of host entries keyed by a single host_ip. Both shapes are in fact
    accepted for either model — the SDK's ParseLidarCfg branches on array vs
    object, not on device type, and ParseHostNetInfo takes host_ip or
    cmd_data_ip interchangeably — so this could collapse to one array-shaped
    block. It deliberately does not: the MID360 object form is the one already
    running on real hardware, and there is nothing to win by moving it onto a
    parse path it has never been through.
    """
    # The host binds the lidar-side port + 1 for every stream, same convention
    # as every vendor sample.
    host_ports = {key: port + 1 for key, port in LIDAR_NET_PORTS.items()}

    if model == "mid360s":
        # Array of one. The newer schema allows several host entries per lidar
        # (and an optional per-entry lidar_ip list); we only ever have the one
        # Jetson, so a single element is the whole story. It also drops the
        # per-stream IP fields for a single host_ip, which is what the SDK
        # actually stores either way (see below).
        return [{"host_ip": host_ip, **host_ports}]

    return {
        "cmd_data_ip": host_ip,
        "cmd_data_port": host_ports["cmd_data_port"],
        "push_msg_ip": host_ip,
        "push_msg_port": host_ports["push_msg_port"],
        "point_data_ip": host_ip,
        "point_data_port": host_ports["point_data_port"],
        "imu_data_ip": host_ip,
        "imu_data_port": host_ports["imu_data_port"],
        # Left empty only to mirror the vendor file byte for byte; it has no
        # effect. The SDK's HostNetInfo holds a single host_ip, taken from
        # cmd_data_ip when the per-stream form is used (ParseHostNetInfo), and
        # the log socket is opened against that same host_ip regardless
        # (device_manager.cpp). What actually keeps the SDK's debug log stream
        # off is the absent top-level lidar_log_enable, which parse_cfg_file.cpp
        # defaults to false — so these four per-stream IPs all have to agree,
        # and there is no way to opt out of just the log one.
        "log_data_ip": "",
        "log_data_port": host_ports["log_data_port"],
    }


def write_livox_config(robot_id: str, model: str, lidar_ip: str, host_ip: str) -> str:
    """Render the livox user config JSON and return its path.

    Only the two IPs and the model vary; everything else mirrors the vendor
    MID360_config.json / MID360s_config.json. extrinsic_parameter stays all-zero
    on purpose — the mount extrinsic (the 0.25 rad tilt included) lives on the
    URDF's lidar_top_joint, which is what syncai_lio_bridge looks up. Setting it
    here as well would apply the rotation twice.

    lidar_net_info and lidar_configs are shared across models rather than
    branched: the SDK's two port sets (kMid360* and kMid360s* in
    comm/define.h) hold identical numbers, and params_check.cpp overwrites them
    from device_type anyway, so a per-model copy would only be able to disagree.
    """
    config = {
        "lidar_summary_info": {"lidar_type": LIVOX_LIDAR_TYPE},
        LIVOX_MODELS[model]: {
            "lidar_net_info": dict(LIDAR_NET_PORTS),
            "host_net_info": render_host_net_info(model, host_ip),
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
    # clobber each other's config, and per model so re-lidaring a robot does not
    # leave a stale file whose name claims the other one.
    os.makedirs(LIVOX_CONFIG_DIR, exist_ok=True)
    config_path = os.path.join(LIVOX_CONFIG_DIR, f"{robot_id}_{model}_config.json")
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    logger.info(
        f"Livox config for '{robot_id}': {model} at {lidar_ip} -> host "
        f"{host_ip} (rendered to {config_path})"
    )
    return config_path


def camera_enabled(value: str) -> bool:
    """Resolve the use_camera argument, which arrives as a string.

    LaunchConfiguration.perform always yields text, and Python's truthiness
    would make the string "false" enable the camera — the exact bug that makes
    a boolean launch argument look like it is being ignored.
    """
    return value.strip().lower() in ("1", "true", "yes", "on")


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

    # Livox driver, brought over from
    # livox_ros_driver2/launch_ROS2/msg_MID360_launch.py. The user config JSON
    # carries everything the ROS params cannot express: the network wiring
    # (lidar IP, host IP, UDP ports) and the lidar model. It is rendered here
    # from the per-robot inputs instead of read out of the vendor share
    # directory — see write_livox_config.
    livox_config_path = write_livox_config(
        robot_id,
        read_lidar_type(config_path),
        read_lidar_ip(config_path),
        read_host_ip(params_path),
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

    nodes = [robot_state_publisher, livox_driver]

    # TechNexion VCS-AR0234-C over VizionSDK, opt-in (see the header). A plain
    # Node rather than an IncludeLaunchDescription of the vendor's
    # vizionsdk_camera.launch.py, for the same reason the livox driver above is:
    # that file is a Node wrapped in 50 DeclareLaunchArguments, every one of
    # which would have to be threaded back through launch_arguments as a string,
    # and it would put the settings somewhere other than params/bringup.yaml
    # where the rest of this package's parameters live.
    if camera_enabled(LaunchConfiguration("use_camera").perform(context)):
        nodes.append(
            Node(
                package="vizionsdk_ros2",
                executable="vizionsdk_camera_node",
                name=CAMERA_NODE_NAME,
                namespace=robot_id,
                output="screen",
                parameters=[
                    params_file,
                    {
                        # Appended after the params file so they win over it.
                        # The frames need the resolved robot_id; device_index is
                        # an argument because the Jetson enumerates two
                        # VCS-AR0234-C units (usb-3.1 and usb-3.4) and only the
                        # first is passed into the container today.
                        "imu_frame_id": f"{robot_id}/{CAMERA_IMU_FRAME}",
                        "image_frame_id": f"{robot_id}/{CAMERA_OPTICAL_FRAME}",
                        "device_index": int(
                            LaunchConfiguration("camera_device_index").perform(context)
                        ),
                    },
                ],
            )
        )

    return nodes


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
        description="Full path to the parameters YAML for robot_state_publisher, "
        "the MID360 driver, and the VizionSDK camera",
    )

    declare_urdf_file = DeclareLaunchArgument(
        "urdf_file",
        default_value="G23.urdf",
        description="URDF file name under syncai_bringup/description/ for "
        "robot_state_publisher (capsule collisions swapped to cylinder so "
        "urdfdom can parse it; TF-only, collision fidelity not needed)",
    )

    declare_use_camera = DeclareLaunchArgument(
        "use_camera",
        default_value="false",
        description="Start the VizionSDK camera node. Default false: "
        "/dev/video0 takes one streaming opener and scripts/publish_camera.sh "
        "(RTSP -> MediaMTX -> the frontend's WebRTC view) already holds it. "
        "Set true only when this node, not that script, owns the camera",
    )

    declare_camera_device_index = DeclareLaunchArgument(
        "camera_device_index",
        default_value="0",
        description="VizionSDK camera index, as reported by "
        "`ros2 run vizionsdk_ros2 list_devices`. The Jetson has two "
        "VCS-AR0234-C units but docker-compose.robots.yml passes only "
        "/dev/video0 through, so 0 is the only index reachable in-container",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_urdf_file,
            declare_use_camera,
            declare_camera_device_index,
            OpaqueFunction(function=launch_setup),
        ]
    )
