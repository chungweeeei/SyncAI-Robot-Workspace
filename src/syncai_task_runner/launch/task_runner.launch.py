# Launch the non-lifecycle task_runner node — the BT action server that
# exposes navigate_to_pose (nav2_msgs), navigate_to_goal (syncai_common, with
# a failure reason in the result) and the patrol route's two,
# navigate_through_poses (nav2_msgs) and navigate_through_goals
# (syncai_common), and ticks the behavior tree for whichever one a goal
# arrives on. It talks to the planner (compute_path_to_pose /
# compute_path_through_poses) and controller (follow_path) action servers, so
# those must be running for a goal to succeed.
#
# robot_id is read from the system config INI at launch time (same convention
# as sys_manager.launch.py) and is used both as the node namespace and to
# rewrite the base_frame and body_frame parameters, since TF frame names are
# not namespaced by ROS. The global_frame stays "map".
#
# Two parameter files are loaded, in order: the factory params in this package
# and the robot model's, params/models/<robot_model>.yaml in this package
# (the lidar's height, for the model [system] robot_model
# names). A model with no such file fails the launch; the helpers are copied
# from controller_server.launch.py, not shared, as every launch's are.

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

logger = launch_logging.get_logger("task_runner.launch")


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
        model_path = model_params_path(get_package_share_directory("syncai_task_runner"), robot_model)
    model_overlay = model_params(model_path, robot_model)

    parameters = [params_file]
    if model_overlay:
        parameters.append(model_overlay)

    # No `name=` here: the params file uses the /**/task_runner wildcard key so
    # it matches in any namespace. use_sim_time comes ONLY from the params file —
    # a launch override placed after the file would silently win over the YAML.
    task_runner_node = Node(
        package="syncai_task_runner",
        executable="task_runner",
        namespace=robot_id,
        output="screen",
        # Factory params, then the robot model's, then the frame
        # overrides: a later entry wins.
        parameters=[
            *parameters,
            {
                # TF frame names are not namespaced by ROS, so override the
                # yaml default with the robot_id prefix here. Later entries
                # in this list take precedence over the params file.
                "base_frame": f"{robot_id}/base_link",
                "body_frame": f"{robot_id}/pointlio_body",
            },
        ],
    )

    return [task_runner_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_task_runner")
    default_params_file = os.path.join(pkg_share, "params", "task_runner_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the task_runner parameters YAML file",
    )

    declare_model_params_file = DeclareLaunchArgument(
        "model_params_file",
        default_value="",
        description=(
            "Robot-model params YAML loaded after the factory params; empty = "
            "params/models/<robot_model>.yaml in this package"
        ),
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_model_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
