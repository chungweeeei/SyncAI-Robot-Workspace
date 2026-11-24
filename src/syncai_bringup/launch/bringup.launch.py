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
# config/sessions/start_nav.yaml — there is no lifecycle manager, so it must be up
# before syncai_lio_bridge.
#
# Publishes:
#   * The robot's body TF tree via robot_state_publisher (reads the model's
#     URDF under description/; what follows describes G23.urdf, the one every
#     robot ran until 2026-11). The URDF root link is base_link, so with frame_prefix the
#     legs hang directly off <robot_id>/base_link (odom->base_link comes from
#     LIO). The fixed joints (the 4 *_Ankle, lidar_top_joint,
#     lidar_top_imu_joint and camera_optical_joint) go to /tf_static; the 12
#     revolute leg joints would only appear on /tf once /joint_states is
#     published, and nothing does that on purpose — live joint angles reach
#     their only consumer (the frontend 3D model) over syncai_driver_manager's
#     motor_states instead.
#   * lidar_top_joint carries the MID360 mount extrinsic (including the 0.25 rad
#     physical tilt) to <robot_id>/lidar_top, the lidar itself and the frame its
#     cloud is stamped with; lidar_top_imu_joint hangs the lidar's IMU off it,
#     which is what Point-LIO's body frame is. syncai_lio_bridge maps the LIO
#     body pose onto the odom TF chain through one of the two (lidar_top until
#     its own repository moves to lidar_top_imu). The mount used to be a
#     separate static_transform_publisher here, driven by a lidar_height launch
#     argument; both were dropped when the extrinsic moved into the URDF,
#     because a height-only argument could not carry the pitch.
#   * The Livox MID360 / MID360s point cloud, via the livox_ros_driver2 node,
#     stamped <robot_id>/lidar_top -- only for a robot model whose params carry
#     the /**/livox_lidar_publisher block (the G23's g23.yaml; has_livox_block).
#     A model without one (the M20, whose lidars are the vendor stack's) gets
#     no driver.
#   * <robot_id>/joint_states, for a model MODEL_JOINT_STATES names (the M20:
#     syncai_m20_joint_state, from the vendor's /JOINTS_DATA), so its leg and
#     wheel links reach /tf through robot_state_publisher.
#
# No camera. This file could start the TechNexion camera through vizionsdk_ros2
# (use_camera:=true, off by default) until 2026-10-17, when that node was
# removed: the V4L2 device admits exactly one streaming opener, and
# syncai_camera_feeder — the sessions' camera_feeder window — is it, teeing
# the MJPEG to the WebRTC worker and to image_raw/compressed. A second opener
# here could only ever crash-loop the feeder. git history has the node, its
# arguments and its params block. The camera_optical_frame joint stays in the
# URDF, because the feeder stamps its images with that frame.
#
# robot_id is read from the system config INI at launch time, same convention as
# every other launch file in the stack. The same INI also supplies the lidar's
# own IP and model ([sensor.lidar] ip / type), which are per-robot hardware
# identity and belong next to robot_id rather than in a shared params file.
#
# The INI's [system] robot_model (g23 when absent) picks the URDF (MODEL_URDF)
# and the model's own values, params/models/<robot_model>.yaml in this
# package, loaded after params/bringup.yaml: on the G23, the whole Livox
# driver block, host_ip included.
# A model with no such file fails the launch rather than come up on another
# robot's values; the helpers are copied from controller_server.launch.py, not
# shared, as every launch's are.
#
# Node parameters live in params/bringup.yaml and the model's file under /**/
# wildcard keys. Only the values that cannot be static are set here, appended
# after the files so they win:
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

# The frame the driver stamps its clouds with. Kept here rather than in the
# params file because it is the one livox setting the launch has to rewrite: TF
# frame names are not namespaced by ROS, so it ships as "<robot_id>/lidar_top".
#
# It is the URDF's lidar_top link on purpose (2026-10). It used to be a separate
# "laser" frame that nothing parented, so a raw livox/lidar message could not be
# transformed anywhere — harmless while nothing did, but camera <-> lidar
# calibration and an overlay of the raw cloud on the image need the header to
# be a frame in the robot tree. A first attempt at this was reverted because
# lidar_top then meant both the cloud's origin and, to syncai_lio_bridge, the
# Point-LIO body (the lidar's IMU), which are t_il (~5 cm) apart; the IMU has
# its own link under lidar_top now (lidar_top_imu), so lidar_top is the lidar
# alone. Point-LIO ignores the header, so LIO is unaffected. Bags recorded
# before this change carry "<robot_id>/laser".
LIVOX_FRAME_ID = "lidar_top"

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

# The URDF robot_state_publisher reads for each [system] robot_model, under
# description/. A table rather than "<MODEL>.urdf" by convention, so that a
# model is listed here only once its URDF carries the three links the stack
# hangs things on -- lidar_top (the driver's cloud), lidar_top_imu (Point-LIO's
# body) and camera_optical_frame (the feeder's images). M20.urdf does not yet
# (its lidar mount is not measured), and is listed anyway: the body tree is
# what the mount will be measured against, and a missing lidar_top shows as an
# unparented cloud, not as another robot's legs.
MODEL_URDF = {"g23": "G23.urdf", "m20": "M20.urdf"}

# The node that publishes <robot_id>/joint_states for a model, as (package,
# executable): robot_state_publisher puts a URDF's revolute / continuous joints
# on /tf only from that topic. The M20's is syncai_m20_joint_state, which
# converts the vendor's /JOINTS_DATA (2026-11-24). The G23 has none on purpose:
# its leg angles reach their only consumer (the console's 3D model) over
# syncai_driver_manager's motor_states, so its legs stay off /tf. Named here
# and not in package.xml, so a G23 build needs neither the package nor the
# vendor's drdds it depends on.
MODEL_JOINT_STATES = {"m20": ("syncai_m20_joint_state", "m20_joint_state_node")}

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


def count_leaves(node) -> int:
    if isinstance(node, dict):
        return sum(count_leaves(v) for v in node.values())
    return 1


# The model an INI that names none is taken to be: the one every factory params
# file in this workspace was written for (config/models/README.md).
FALLBACK_ROBOT_MODEL = "g23"


def read_robot_model(config_path: str) -> str:
    """[system] robot_model from the INI: which kind of robot this is.

    Lower-cased, because it names a file (params/models/<model>.yaml). A
    missing INI or key falls back to FALLBACK_ROBOT_MODEL with a warning, the
    way robot_id falls back: an INI written before 2026-11 has no such key and
    is a G23's.
    """
    config = configparser.ConfigParser()
    config.read(config_path)
    model = config.get("system", "robot_model", fallback="").strip().lower()
    if not model:
        logger.warning(
            f"No [system] robot_model in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_MODEL}'"
        )
        return FALLBACK_ROBOT_MODEL
    return model


def model_params_path(pkg_share: str, model: str) -> str:
    """params/models/<model>.yaml in this package's share: the model's file.

    It sits beside the factory params file it lays over, as
    syncai_driver_manager's does, so each package owns its per-model values
    and a key renamed in the factory file is renamed in the model files in the
    same commit. Being installed, a new model's file needs a colcon build of
    the package before a launch finds it (--symlink-install links each file,
    not the directory); model_params_file:= names any file without one.
    """
    return os.path.join(pkg_share, "params", "models", f"{model}.yaml")


def model_params(path: str, model: str):
    """Return the robot model's params file for this launch, or None when it
    holds no key.

    Loaded after the factory params: what this *model* of robot changes from
    them (body size, gait limits, sensor mount). Nothing here is skipped: a
    missing or unreadable file fails the launch. Falling back to the factory
    values would start another robot on the G23's footprint and gait floors
    and look healthy doing it, so a model that changes nothing here says so
    with a file that holds no key (each package's params/models/g23.yaml) -- and
    that file is not handed to rcl, which is never given an empty block.
    """
    if not os.path.isfile(path):
        raise RuntimeError(
            f"robot_model '{model}': '{path}' does not exist, so this launch has no "
            f"{model} parameters and will not start on another model's. Write the file "
            "in this package's params/models/ and rebuild it (a block with no key means "
            "no difference), or fix [system] robot_model."
        )
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError("top level is not a mapping")
        keys = 0
        for node, section in data.items():
            if not isinstance(section, dict) or "ros__parameters" not in section:
                raise ValueError(f"'{node}' has no ros__parameters block")
            keys += count_leaves(section["ros__parameters"] or {})
    except (OSError, ValueError, yaml.YAMLError) as e:
        raise RuntimeError(
            f"robot_model '{model}': '{path}' is unreadable ({e}); not starting on "
            "the factory params alone"
        ) from e
    if keys == 0:
        logger.info(f"robot_model '{model}': '{path}' holds no key; factory params as they are")
        return None
    logger.info(f"robot_model '{model}': '{path}' loaded, {keys} key(s) over the factory params")
    return path


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


def has_livox_block(*params_paths: str) -> bool:
    """Whether any of the params files has a /**/livox_lidar_publisher block.

    That block is what says this model's lidar is a Livox of ours: since
    2026-11-24 it lives only in the model's file (params/models/g23.yaml), not
    in the factory params, so a model whose file leaves it out -- the M20, whose
    lidars are the vendor stack's on /LIDAR/POINTS_UDP -- gets no driver at all,
    rather than one started on code defaults with no host IP. Decided by the
    files, not by a table of models here, so that adding a Livox to a model is a
    params change only. A block with no key still counts: it says "a Livox, on
    the defaults".
    """
    for params_path in params_paths:
        with open(params_path, "r") as f:
            params = yaml.safe_load(f) or {}
        if LIVOX_PARAMS_KEY in params:
            return True
    return False


def read_host_ip(*params_paths: str) -> str:
    """Read host_ip from the livox node's block in the params YAML(s).

    The host IP is the address of the machine's interface on the lidar's subnet
    — the driver binds its receive sockets to it, so it is per-robot wiring, not
    a property of the lidar. It lives in the params files (rather than the INI)
    because that is where every other livox setting already is -- since
    2026-11-24 the robot model's file (params/models/g23.yaml), which holds the
    whole livox block; the factory file holds none. It is read back out here
    because the value is needed to render JSON, not to set a ROS param.

    Given the files in the order the node loads them (the factory params, then
    the robot model's when it holds a key), the last one that sets host_ip
    wins, as it does for the ROS parameter: the JSON the driver binds to must
    not disagree with the parameter it reports.
    """
    host_ip = ""
    for params_path in params_paths:
        with open(params_path, "r") as f:
            params = yaml.safe_load(f) or {}

        block = (params.get(LIVOX_PARAMS_KEY) or {}).get("ros__parameters") or {}
        value = str(block.get("host_ip", "")).strip()
        if value:
            host_ip = value

    if not host_ip:
        raise RuntimeError(
            f"No host_ip under '{LIVOX_PARAMS_KEY}' in {list(params_paths)}. Set it "
            "in the robot model's params/models/<robot_model>.yaml to this machine's "
            "IP on the lidar subnet (`ip -4 addr`)."
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
    URDF's lidar_top_joint, the frame the clouds are stamped with. Setting it
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
                # 1 = high-precision cartesian points on the wire. This is the
                # lidar-side format only: the driver decodes it into the same
                # point queue whichever ROS message xfer_format then builds,
                # so livox_xfer_format:=0 needs no change here.
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


def resolve_xfer_format(value: str):
    """Resolve the livox_xfer_format argument: None (keep the YAML's) or 0 / 1.

    Only the two formats the driver supports on ROS 2 are accepted (2, PCL
    PointXYZI, is ROS 1 only), and anything else fails the launch: a typo here
    would otherwise reach the driver as a format it silently does not publish.
    """
    value = value.strip()
    if not value:
        return None

    if value not in ("0", "1"):
        raise RuntimeError(
            f"livox_xfer_format must be 0 (PointCloud2) or 1 (CustomMsg), got '{value}'"
        )

    return int(value)


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    # The robot model picks two things here: the URDF, and the model's own
    # values over the factory params (params/models/<robot_model>.yaml in
    # this package; a model without that file fails the launch in
    # model_params).
    robot_model = read_robot_model(config_path)

    # An explicit urdf_file:= wins; otherwise the model's. A model with no
    # URDF listed fails here rather than publish the G23's legs under another
    # robot's base_link.
    urdf_file = LaunchConfiguration("urdf_file").perform(context).strip()
    if not urdf_file:
        if robot_model not in MODEL_URDF:
            raise RuntimeError(
                f"robot_model '{robot_model}' has no URDF in MODEL_URDF "
                f"({sorted(MODEL_URDF)}). Add it to bringup.launch.py, or pass urdf_file:=."
            )
        urdf_file = MODEL_URDF[robot_model]
    logger.info(f"robot_model '{robot_model}': robot_state_publisher on '{urdf_file}'")

    # Resolved as well as passed through: the nodes get the file itself, and
    # write_livox_config needs one value out of it (see read_host_ip).
    params_path = LaunchConfiguration("params_file").perform(context)
    params_file = LaunchConfiguration("params_file")

    model_path = LaunchConfiguration("model_params_file").perform(context)
    if not model_path:
        model_path = model_params_path(get_package_share_directory("syncai_bringup"), robot_model)
    model_overlay = model_params(model_path, robot_model)

    # What both nodes load before their computed overrides, and what
    # read_host_ip reads back in the same order.
    params_paths = [params_path]
    parameters = [params_file]
    if model_overlay:
        params_paths.append(model_overlay)
        parameters.append(model_overlay)

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
            *parameters,
            {
                # Computed here, so appended after the params files to win over
                # them: the URDF text read off disk, and the frame prefix which
                # needs the resolved robot_id.
                "robot_description": robot_description,
                "frame_prefix": f"{robot_id}/",
            },
        ],
    )

    nodes = [robot_state_publisher]

    # The model's joint_states source, in the same namespace, so it publishes
    # the /<robot_id>/joint_states this robot_state_publisher subscribes to.
    if robot_model in MODEL_JOINT_STATES:
        package, executable = MODEL_JOINT_STATES[robot_model]
        logger.info(f"robot_model '{robot_model}': joint_states from {package}")
        nodes.append(
            Node(
                package=package,
                executable=executable,
                namespace=robot_id,
                output="screen",
            )
        )

    # Only for a model whose params carry the Livox block (has_livox_block).
    # Without it this launch is robot_state_publisher alone: the URDF's fixed
    # joints on /tf_static, and its moving ones once something publishes
    # /joint_states. model_params drops a model file that holds no key, so
    # params_paths is checked rather than the file itself -- an empty livox
    # block there would not reach the node either.
    xfer_format_arg = LaunchConfiguration("livox_xfer_format").perform(context)
    if not has_livox_block(*params_paths):
        logger.info(
            f"robot_model '{robot_model}': no '{LIVOX_PARAMS_KEY}' block in "
            f"{params_paths}, so no Livox driver"
        )
        if xfer_format_arg.strip():
            # The calibration session asks for PointCloud2 from a driver this
            # model does not run: its bag would have no lidar in it. Said here
            # because nothing else would say it.
            logger.warning(
                f"livox_xfer_format:={xfer_format_arg} ignored: robot_model "
                f"'{robot_model}' runs no Livox driver"
            )
        return nodes

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
        read_host_ip(*params_paths),
    )
    livox_overrides = {
        # Both are derived from the resolved robot_id, so they are appended
        # after the params file and take precedence over it.
        "frame_id": f"{robot_id}/{LIVOX_FRAME_ID}",
        "user_config_path": livox_config_path,
    }

    # Unset (the default) keeps the YAML's xfer_format: 1, the CustomMsg that
    # Point-LIO consumes. 0 switches livox/lidar to PointCloud2 for a session
    # with no LIO in it at all -- config/sessions/calib_lidar_camera.yaml,
    # whose calibration tool reads PointCloud2 only. The topic name stays the
    # same and only its type changes, so this must never be set in a session
    # that also runs pointlio.
    xfer_format = resolve_xfer_format(xfer_format_arg)
    if xfer_format is not None:
        livox_overrides["xfer_format"] = xfer_format

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name=LIVOX_NODE_NAME,
        namespace=robot_id,
        output="screen",
        parameters=[*parameters, livox_overrides],
    )

    return [*nodes, livox_driver]


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
        default_value="",
        description="URDF file name under syncai_bringup/description/ for "
        "robot_state_publisher (capsule collisions swapped to cylinder so "
        "urdfdom can parse it; TF-only, collision fidelity not needed). Empty "
        "(default) = the one MODEL_URDF lists for the INI's [system] robot_model",
    )

    declare_model_params_file = DeclareLaunchArgument(
        "model_params_file",
        default_value="",
        description="Robot-model params YAML loaded after params_file; empty = "
        "params/models/<robot_model>.yaml in this package",
    )

    declare_livox_xfer_format = DeclareLaunchArgument(
        "livox_xfer_format",
        default_value="",
        description="Override the livox driver's xfer_format: 0 = PointCloud2, "
        "1 = CustomMsg. Empty (default) keeps the model params' 1, which "
        "Point-LIO needs; 0 is for config/sessions/calib_lidar_camera.yaml only",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_model_params_file,
            declare_urdf_file,
            declare_livox_xfer_format,
            OpaqueFunction(function=launch_setup),
        ]
    )
