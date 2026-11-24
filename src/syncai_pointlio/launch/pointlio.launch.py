# Point-LIO front end launch, for the robot's own lidar and IMU (the G23's
# Livox, the M20's vendor lidars; params/models/<robot_model>.yaml says which).
#
# This is THE definition of how pointlio_node is configured, and the only one:
# the session specs (config/sessions/start_nav.yaml, start_mapping.yaml) run it
# in a pane of its own right before localizer / pgo. Until 2026-09 it lived in
# SyncAI-Fast-LIO2 as pointlio_launch.py and was include()d by those two
# launches; the include existed because a second, drifted copy of the Node had
# once passed a `config_path` nobody read, so the LIO fell back to struct
# defaults and subscribed to topics nothing published. The absolute
# /<robot_id>/pointlio/... names those launches still inject, and the
# <robot_id>/pointlio_odom frame pgo's config is rewritten to, are the contract
# with what is set below.
#
# robot_id is read from the system config INI at launch time (same convention as
# every other launch file in this workspace) and is used as the node namespace,
# as the prefix for the Livox topics, and as the prefix for the TF frames:
#   topics: /<robot_id>/livox/lidar, /<robot_id>/livox/imu (unless the model's
#           file names its own, below)
#   frames: <robot_id>/pointlio_odom -> <robot_id>/pointlio_body
#
# pointlio_node's settings are plain ROS parameters (params/pointlio_params.yaml,
# keyed by the `/**/pointlio_node:` wildcard so the file works at any namespace),
# so this launch passes the installed file straight through and layers only the
# robot_id-dependent bits on top. It used to instead rewrite the whole YAML into
# a /tmp file, because the node parsed that YAML itself with yaml-cpp and knew
# nothing about ROS parameters.
#
# The parameters:
#   1. the installed params/pointlio_params.yaml (what every model shares);
#   2. the robot model's file, params/models/<robot_model>.yaml in this package:
#      its sensors (lidar type, IMU units and range, extrinsic), and for a
#      robot whose sensors are not a Livox of ours its own input topics (the
#      M20's absolute /LIDAR/POINTS_UDP and /IMU). A model with no such file
#      fails the launch rather than run on another robot's values; the helpers
#      are copied from controller_server.launch.py, not shared;
#   3. the Livox input topics, /<robot_id>/livox/{lidar,imu}, as the node's
#      lidar_topic / imu_topic parameters -- computed here, because they carry
#      the robot_id and a params file cannot -- for whichever the model's file
#      leaves out. Remappings of the relative "lidar" / "imu" did this until
#      2026-11-24; `ros2 param get` now shows the topic the node reads. Only
#      the keys the model does not set, because a dict here would override the
#      model's file wherever it sat in the list (launch_setup has why);
#   4. the frame names, prefixed by hand since TF frame ids are not namespaced
#      by ROS (same as the nav params).

import configparser
import os

import yaml

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Absolute rather than the relative `config/system.ini` the C++ packages' launch
# files use: identical to lio_bridge.launch.py, so the two LIO-side launches
# resolve the INI the same way whether or not the cwd is the workspace root.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("pointlio.launch")


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


def model_param_keys(path: str) -> set:
    """The top-level parameter names the model's file sets for pointlio_node.

    Read after model_params(), which has already refused a missing or
    unreadable file, so a failure here is not expected and is not caught.
    """
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    keys = set()
    for section in data.values():
        keys.update((section or {}).get("ros__parameters") or {})
    return keys


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


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    pkg_pointlio = FindPackageShare("syncai_pointlio").find("syncai_pointlio")
    params_file = os.path.join(pkg_pointlio, "params", "pointlio_params.yaml")

    # The robot model's own values, after the installed params and the Livox
    # topics, before the frames: params/models/<robot_model>.yaml in this
    # package. A model without that file fails the launch here (model_params).
    robot_model = read_robot_model(config_path)
    model_path = LaunchConfiguration("model_params_file").perform(context)
    if not model_path:
        model_path = model_params_path(pkg_pointlio, robot_model)
    model_overlay = model_params(model_path, robot_model)

    # The Livox driver's topics (syncai_bringup, in the robot_id namespace),
    # for each input the model's file does not name itself (the M20's names
    # the vendor's). A single set of names covers both the real robot and bag
    # replay: every rosbag is now recorded with the robot_id prefix the driver
    # publishes, so the `bag_topics` argument that used to strip the prefix is
    # gone. Bags predating the prefix need `ros2 bag play --remap` onto these.
    #
    # Left out rather than layered under the model's file: the order of the
    # files does NOT decide between them. A dict here becomes a params file
    # keyed `/**`, and rclcpp merges every file first, then applies the node
    # keys in the order each first appeared -- `/**/pointlio_node` (the factory
    # file's, the model's merged into it) before `/**` -- so the dict won over
    # m20.yaml wherever it sat in the list (robot01, 2026-11-24: subscribed to
    # /robot01/livox/lidar and received nothing). The frame overrides below are
    # dicts for the same reason they win: nothing else sets those keys.
    model_keys = model_param_keys(model_path)
    livox_topics = {
        key: topic
        for key, topic in (
            ("lidar_topic", f"/{robot_id}/livox/lidar"),
            ("imu_topic", f"/{robot_id}/livox/imu"),
        )
        if key not in model_keys
    }
    for key in ("lidar_topic", "imu_topic"):
        if key in model_keys:
            logger.info(f"robot_model '{robot_model}': {key} from {model_path}")

    parameters = [params_file]
    if model_overlay:
        parameters.append(model_overlay)
    if livox_topics:
        parameters.append(livox_topics)

    # Standalone frame names, NOT <robot_id>/laser -> <robot_id>/base_link,
    # which is what they used to be. base_link is already claimed as a TF child
    # by syncai_lio_bridge (odom -> base_link), so naming pointlio's body frame
    # base_link gave that frame two parents. tf2 keys its cache by CHILD frame,
    # so the two broadcasters' samples interleaved in one cache and the parent
    # you got back depended on the lookup time: latest (Time()) hit lio_bridge's
    # 20 Hz samples, the cloud's own stamp hit pointlio's 10 Hz ones. That
    # silently sent the backend's body_cloud transform through lio_bridge's
    # 2D-projected chain (z == 0, roll == pitch == 0) while rviz2, which looks up
    # at the message stamp, went through the full 6DOF chain — a ~15 deg pitch
    # disagreement between rviz2 and the operator UI.
    #
    # The rename also stops body_frame from lying: Point-LIO's body frame is the
    # lidar's built-in IMU, not base_link (and not the lidar either, which is
    # t_il away). Its static counterpart in the robot tree is the URDF's
    # lidar_top_imu, which is what syncai_lio_bridge's lidar->base correction
    # should go through.
    frame_overrides = {
        "world_frame": f"{robot_id}/pointlio_odom",
        "body_frame": f"{robot_id}/pointlio_body",
    }

    # `name=` is fine here, unlike the planner / controller launches: this
    # process hosts exactly one node, so there is no second node to be remapped
    # onto the same name and lose its parameters.
    return [
        launch_ros.actions.Node(
            package="syncai_pointlio",
            namespace=f"{robot_id}/pointlio",
            executable="pointlio_node",
            name="pointlio_node",
            output="screen",
            # installed params, the robot model's, the Livox topics it does not
            # name, then the frames
            parameters=[*parameters, frame_overrides],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id "
                "and robot_model",
            ),
            DeclareLaunchArgument(
                "model_params_file",
                default_value="",
                description="Robot-model params YAML loaded after the installed params; "
                "empty = params/models/<robot_model>.yaml in this package",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
