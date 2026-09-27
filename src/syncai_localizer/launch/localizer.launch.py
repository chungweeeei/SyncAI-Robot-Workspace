# Localizer launch: starts localizer_node (syncai_localizer) and nothing else.
#
# This is THE definition of how localizer_node is configured. The nav session
# (config/sessions/start_nav.yaml) runs it in the `localization` window right
# after `ros2 launch syncai_pointlio pointlio.launch.py`, the front end it
# registers against. Until 2026-09 it lived in SyncAI-Fast-LIO2 as
# localizer_launch.py; until earlier that month it also include()d pointlio's
# launch, and before that it spawned a pointlio_node of its own with a
# `config_path` nobody read, so the LIO fell back to struct defaults and
# subscribed to topics nothing published — no input for the localizer at all.
# Do not grow a pointlio Node definition here: the two absolute
# /<robot_id>/pointlio/... names injected below are the only coupling with
# pointlio.launch.py, and they have to match it.
#
# robot_id is read from the system config INI at launch time (the workspace-
# wide convention) and is used as the namespace. THE NAMESPACE IS THE BARE
# ROBOT_ID, NOT <robot_id>/localizer:
#   /<robot_id>/localizer_node                    (the node)
#   /<robot_id>/{relocalize,relocalize_check}     (services)
#   /<robot_id>/initialpose                       (subscribed)
#   /<robot_id>/map_cloud                         (latched, rviz)
# It was <robot_id>/localizer until fork commit 3f5f01b (2026-07-29) dropped
# the segment, and the backend has called the bare names from the robot's
# namespace ever since (its map gateway records that `localizer/relocalize`
# cost it a stack_not_ready refusal against a healthy stack). Moving it back
# would be a cross-repository change; the docs that still said /localizer/
# were corrected instead. `initialpose` in the robot namespace also happens to
# be where rviz's "2D Pose Estimate" and AMCL-era tooling expect it.
#
# localizer_node takes standard ROS 2 parameters — params/localizer_params.yaml
# is a /**/localizer_node params file, and the robot_id-dependent values
# (pointlio's topics, the map PCD path, the boot pose) are passed as a second
# parameters dict that overrides the file. No generated files involved.
#
# The robot_id-dependent values are parameter overrides, not remappings: the
# node builds its message_filters subscribers from parameter values (absolute
# names into pointlio's namespace, which a relative name from /<robot_id> could
# not reach). local_frame needs no override — the node adopts the frame_id of
# the first odom message.
#
# The [map] pcd from the same INI is mandatory: the localizer loads it during
# construction, so a missing file means the node is not started at all. The
# optional [initial_pose] section (the one the retired syncai_amcl read too)
# becomes the localizer's boot guess, so a robot standing at its known start
# pose localizes itself without a relocalize call.

import configparser
import os

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Absolute rather than the relative `config/system.ini` the nav packages'
# launch files use: identical to pointlio.launch.py, mapping.launch.py and
# lio_bridge.launch.py, so the LIO-side launches resolve the INI the same way
# whether or not the cwd is the workspace root.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("localizer.launch")


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

    The instance INI writes it as `map/%(name)s/map.pcd` (configparser
    interpolation, relative to the workspace root), and the INI itself sits at
    <workspace>/config/system.ini, so two dirname()s up from it is the root.
    An absolute value is passed through untouched; an empty one is returned
    as-is so the caller can say "no map" rather than "file not found".
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    pcd = config.get("map", "pcd", fallback="").strip()
    if not pcd or os.path.isabs(pcd):
        return pcd
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, pcd)


def read_initial_pose(config_path: str):
    config = configparser.ConfigParser()
    if not config.read(config_path) or not config.has_section("initial_pose"):
        logger.info(
            f"No [initial_pose] in '{config_path}'; the localizer will wait for "
            "a relocalize call or an initialpose message"
        )
        return None

    try:
        pose = {
            key: config.getfloat("initial_pose", key, fallback=0.0)
            for key in ("x", "y", "yaw")
        }
    except ValueError as err:
        logger.warning(
            f"Malformed [initial_pose] in '{config_path}' ({err}); the localizer "
            "will wait for a relocalize call or an initialpose message"
        )
        return None

    logger.info(f"Initial pose from '{config_path}': {pose}")
    return {
        "set_initial_pose": True,
        "initial_pose.x": pose["x"],
        "initial_pose.y": pose["y"],
        "initial_pose.yaw": pose["yaw"],
    }


def localizer_params_file() -> str:
    """The installed /**/localizer_node params file — holds every localizer
    parameter that does not depend on robot_id."""
    pkg_localizer = FindPackageShare("syncai_localizer").find("syncai_localizer")
    return os.path.join(pkg_localizer, "params", "localizer_params.yaml")


def localizer_overrides(robot_id: str, map_pcd: str, initial_pose: dict) -> dict:
    """The instance-dependent localizer parameters, layered on top of the params
    file. cloud_topic / odom_topic live under pointlio's namespace, not the
    localizer's, so relative names cannot reach them; they are the contract with
    syncai_pointlio/launch/pointlio.launch.py. local_frame needs no override —
    the node adopts the frame_id of the first odom message. map_pcd is already
    absolute and verified to exist (see launch_setup); initial_pose is
    read_initial_pose()'s dict, or None to leave set_initial_pose at the
    params-file default (false)."""
    overrides = {
        "cloud_topic": f"/{robot_id}/pointlio/body_cloud",
        "odom_topic": f"/{robot_id}/pointlio/lio_odom",
        "map_path": map_pcd,
    }
    if initial_pose:
        overrides.update(initial_pose)
    return overrides


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    # The map is a hard requirement: the localizer calls loadMap during
    # construction (see loadInitialMap in localizer_node.cpp), and without a map
    # the whole 3D localization chain can do nothing. So when the file is missing
    # no node is started at all (equivalent to returning an empty
    # LaunchDescription) — easier to diagnose than a localizer that comes up and
    # then fails every relocalize / initialpose. This is also what keeps the
    # mapping session honest: start_mapping.yaml lists the localizer among the
    # nodes deliberately absent there, and this check is why it *could* not be
    # present anyway. The check lives here rather than in
    # generate_launch_description() because the INI path comes from the
    # system_config launch argument, whose value can only be resolved inside the
    # context.
    map_pcd = read_map_pcd(config_path)
    if not map_pcd:
        logger.error(f"No [map] pcd in '{config_path}'; nothing to launch")
        return []
    if not os.path.isfile(map_pcd):
        logger.error(f"[map] pcd '{map_pcd}' does not exist; nothing to launch")
        return []

    initial_pose = read_initial_pose(config_path)

    # `name=` is fine here, unlike the planner / controller launches: this
    # process hosts exactly one node, so there is no second node to be remapped
    # onto the same name and lose its parameters.
    return [
        launch_ros.actions.Node(
            package="syncai_localizer",
            # The bare robot_id, on purpose -- see the header.
            namespace=f"{robot_id}",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            # params file first, robot_id overrides second — later entries win
            parameters=[
                localizer_params_file(),
                localizer_overrides(robot_id, map_pcd, initial_pose),
            ],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id, "
                "[map] pcd and the optional [initial_pose]",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
