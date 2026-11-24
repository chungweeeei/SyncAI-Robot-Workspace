# Launch the non-lifecycle controller server (FollowPath action) with its
# internal local costmap and its controller plugins.
#
# robot_id is read from the system config INI at launch time (same convention
# as sys_manager.launch.py) and is used both as the node namespace and to
# rewrite the local costmap TF frame parameters (global_frame /
# robot_base_frame), since TF frame names are not namespaced by ROS.
#
# Three parameter files are loaded, in order, a later one winning key by key:
#   1. the factory params in this package (tracked, never written at runtime);
#   2. the robot model's, params/models/<robot_model>.yaml in this package
#      (tracked): what a G23 or an M20 changes from the
#      factory file -- footprint, speed limits, gait floors, stair policy.
#      [system] robot_model names the model; a model with no such file fails
#      the launch rather than start on another robot's body (model_params);
#   3. the robot's runtime overlay, config/params/<robot_id>/controller_server.yaml
#      (gitignored). It holds only the keys the console's settings page
#      changed, written by syncai_backend, so an untouched key follows the
#      files above through a git pull. Deleting it, or launching with
#      use_runtime_params:=false, falls back to them.

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

    # The robot model's own values, between the factory params and anything
    # per-robot: params/models/<robot_model>.yaml in this package. A model without that file fails the launch here (model_params).
    robot_model = read_robot_model(config_path)
    model_path = LaunchConfiguration("model_params_file").perform(context)
    if not model_path:
        model_path = model_params_path(get_package_share_directory("syncai_controller"), robot_model)
    model_overlay = model_params(model_path, robot_model)

    parameters = [params_file]
    if model_overlay:
        parameters.append(model_overlay)
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
        # Factory params, then the robot model's, then the runtime overlay,
        # then the frame overrides: a later entry wins, and neither overlay
        # holds a frame.
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

    declare_model_params_file = DeclareLaunchArgument(
        "model_params_file",
        default_value="",
        description=(
            "Robot-model params YAML loaded after the factory params; empty = "
            "params/models/<robot_model>.yaml in this package"
        ),
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
            declare_model_params_file,
            declare_runtime_params_file,
            declare_use_runtime_params,
            OpaqueFunction(function=launch_setup),
        ]
    )
