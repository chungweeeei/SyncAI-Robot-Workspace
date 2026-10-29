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
# not reach). local_frame is overridden too, with the frame pointlio.launch.py
# names: the node adopts the frame_id of the first odom message, but its 10 ms
# timer broadcasts map -> local_frame from startup, so until then the value is
# what lands on TF.
#
# The [map] pcd from the same INI is loaded during construction when it
# exists. When it does not -- `[map] name:` left empty, which is how a robot
# with no map yet is configured (first deployment), or a name whose directory
# is gone -- the node is started anyway with an empty map_path: it broadcasts
# an identity map -> local_frame, latches an empty map_cloud, and waits for the
# backend's map switch to load one through `relocalize` (see launch_setup). The
# optional [initial_pose] section (the one the retired syncai_amcl read too)
# becomes the localizer's boot guess, so a robot standing at its known start
# pose localizes itself without a relocalize call. Since 2026-10-23 the last
# pose the node saved (config/state/<robot_id>/last_pose.json) comes first
# when it is for the same map, so a robot that shut down anywhere starts where
# it stopped; [initial_pose] is the fallback (read_last_pose).

import configparser
import json
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


def read_map_name(config_path: str) -> str:
    """[map] name from the INI, "" when it is absent or left empty.

    Read on its own because an empty name still interpolates [map] pcd into a
    non-empty `map//map.pcd`, which would be reported as a missing file rather
    than as what it is: no map configured."""
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    return config.get("map", "name", fallback="").strip()


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


def last_pose_path(config_path: str, robot_id: str) -> str:
    """<workspace>/config/state/<robot_id>/last_pose.json, the workspace root
    found the way read_map_pcd finds it (two directories above the INI).
    Per-robot state like config/calibration/, so gitignored; the node creates
    the directory on its first write."""
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, "config", "state", robot_id, "last_pose.json")


def read_last_pose(path: str, map_pcd: str):
    """The boot guess from the last pose file the node writes
    (localizers/last_pose_store.h), or None to fall back to [initial_pose].

    Taken only when it is complete and for the map being loaded -- compared
    by the map directory's name, which is what the node writes (map/dp/map.pcd
    -> "dp"), so a map switch since the last run falls back to the INI rather
    than starting the new map from a pose on the old one. Unlike the INI it
    carries the ground's z, so the guess starts on the storey the robot was
    on. Whether the robot is still there is not knowable here: if it was
    carried off while down, the guess fails like any wrong one and is
    withdrawn after guess_timeout_sec, and the file is not rewritten until the
    robot is localized again."""
    if not os.path.isfile(path):
        logger.info(f"No last pose file at '{path}'; using the INI's [initial_pose]")
        return None
    try:
        with open(path, encoding="utf-8") as f:
            fields = json.load(f)
        # bool is an int subclass; a number has to be a real number here, the
        # same rule the node's own reader applies (loadLastPose).
        pose = {}
        for key in ("x", "y", "ground_z", "yaw"):
            value = fields[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} is not a number: {value!r}")
            pose[key] = float(value)
        saved_map = fields["map"]
        if not isinstance(saved_map, str) or not saved_map:
            raise ValueError(f"map is not a name: {saved_map!r}")
    except (OSError, KeyError, TypeError, ValueError) as err:
        logger.warning(
            f"Last pose file '{path}' is unreadable or incomplete ({err!r}); "
            "using the INI's [initial_pose]"
        )
        return None
    map_name = os.path.basename(os.path.dirname(map_pcd))
    if saved_map != map_name:
        logger.info(
            f"Last pose file '{path}' is for map '{saved_map}', not '{map_name}'; "
            "using the INI's [initial_pose]"
        )
        return None
    logger.info(
        f"Initial pose from the last pose file '{path}' (saved "
        f"{fields.get('saved_at', 'at an unknown time')}): {pose}"
    )
    return {
        "set_initial_pose": True,
        "initial_pose.x": pose["x"],
        "initial_pose.y": pose["y"],
        "initial_pose.yaw": pose["yaw"],
        "initial_pose.use_ground_z": True,
        "initial_pose.ground_z": pose["ground_z"],
    }


def localizer_params_file() -> str:
    """The installed /**/localizer_node params file — holds every localizer
    parameter that does not depend on robot_id."""
    pkg_localizer = FindPackageShare("syncai_localizer").find("syncai_localizer")
    return os.path.join(pkg_localizer, "params", "localizer_params.yaml")


def localizer_overrides(
    robot_id: str, map_pcd: str, initial_pose: dict, last_pose_file: str
) -> dict:
    """The instance-dependent localizer parameters, layered on top of the params
    file. cloud_topic / odom_topic live under pointlio's namespace, not the
    localizer's, so relative names cannot reach them; they are the contract with
    syncai_pointlio/launch/pointlio.launch.py, and so is local_frame (its
    world_frame). The node adopts the frame_id of the first odom message, but
    broadcasts map -> local_frame from startup; without the override that was
    the YAML's unprefixed placeholder until pointlio's first message. map_pcd is
    absolute and verified to exist, or "" for no map (see launch_setup); initial_pose is
    read_last_pose()'s or read_initial_pose()'s dict, or None to leave
    set_initial_pose at the params-file default (false). last_pose_file is
    where the node keeps the pose for the next start, whichever source this
    start's guess came from."""
    overrides = {
        "cloud_topic": f"/{robot_id}/pointlio/body_cloud",
        "odom_topic": f"/{robot_id}/pointlio/lio_odom",
        "local_frame": f"{robot_id}/pointlio_odom",
        "map_path": map_pcd,
        "last_pose.file": last_pose_file,
    }
    if initial_pose:
        overrides.update(initial_pose)
    return overrides


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    # No map is a valid state, not a reason to start nothing. Until 2026-10-29
    # a missing [map] pcd returned an empty LaunchDescription, which left a
    # robot with no map yet -- every first deployment -- with no
    # map -> <robot_id>/pointlio_odom at all: lio_bridge had no map -> odom and
    # the console's 3D view, which lives in the map frame, could place neither
    # the live scan nor the robot. The node itself has always handled an empty
    # map_path (loadInitialMap() logs and returns): it broadcasts the identity
    # offset, the way pgo_node does while IDLE, latches an empty map_cloud, and
    # loads a map on the backend's map switch (`relocalize`) without a session
    # restart. The mapping session is unaffected: start_mapping.yaml does not
    # run this launch. The check lives here rather than in
    # generate_launch_description() because the INI path comes from the
    # system_config launch argument, whose value can only be resolved inside the
    # context.
    map_name = read_map_name(config_path)
    map_pcd = read_map_pcd(config_path)
    if not map_name:
        logger.warning(
            f"No [map] name in '{config_path}'; starting the localizer with no map "
            "(identity map -> odom, empty map_cloud) until a map switch loads one"
        )
        map_pcd = ""
    elif not map_pcd:
        logger.warning(
            f"[map] name '{map_name}' but no [map] pcd in '{config_path}'; starting "
            "the localizer with no map until a map switch loads one"
        )
    elif not os.path.isfile(map_pcd):
        logger.warning(
            f"[map] pcd '{map_pcd}' (map '{map_name}') does not exist; starting the "
            "localizer with no map until a map switch loads one"
        )
        map_pcd = ""

    # The last pose file first, the INI's [initial_pose] when there is none for
    # this map: the robot was last seen at the former, the latter is wherever
    # someone last wrote it (2026-10-23; see read_last_pose). Neither with no
    # map: there is nothing to apply a guess to, and the node would only log
    # that it skipped it.
    last_pose_file = last_pose_path(config_path, robot_id)
    initial_pose = None
    if map_pcd:
        initial_pose = read_last_pose(last_pose_file, map_pcd) or read_initial_pose(
            config_path
        )

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
                localizer_overrides(robot_id, map_pcd, initial_pose, last_pose_file),
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
