# Point-LIO front end launch, for the real Livox sensor.
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
# as the prefix for the sensor topics, and as the prefix for the TF frames:
#   topics: /<robot_id>/livox/lidar, /<robot_id>/livox/imu
#   frames: <robot_id>/pointlio_odom -> <robot_id>/pointlio_body
#
# pointlio_node's settings are plain ROS parameters (params/pointlio_params.yaml,
# keyed by the `/**/pointlio_node:` wildcard so the file works at any namespace),
# so this launch passes the installed file straight through and layers only the
# robot_id-dependent bits on top. It used to instead rewrite the whole YAML into
# a /tmp file, because the node parsed that YAML itself with yaml-cpp and knew
# nothing about ROS parameters.
#
# The two robot_id-dependent bits are handled with the mechanism ROS provides
# for each:
#   * topics -> remappings. The node subscribes to the relative names "lidar" /
#     "imu", which would resolve inside its own /<robot_id>/pointlio namespace.
#   * frames -> parameter overrides. TF frame ids are not namespaced by ROS, so
#     they have to be prefixed by hand (same as the nav params).

import configparser
import os

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


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    pkg_pointlio = FindPackageShare("syncai_pointlio").find("syncai_pointlio")
    params_file = os.path.join(pkg_pointlio, "params", "pointlio_params.yaml")

    # A single set of topic names covers both the real robot and bag replay:
    # every rosbag is now recorded with the robot_id prefix the driver publishes,
    # so the `bag_topics` argument that used to strip the prefix (pointing the
    # remappings at the raw /livox/{lidar,imu}) is gone. Bags predating the
    # prefix need `ros2 bag play --remap` instead of a launch argument.
    remappings = [
        ("lidar", f"/{robot_id}/livox/lidar"),
        ("imu", f"/{robot_id}/livox/imu"),
    ]

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
    # The rename also stops body_frame from lying: Point-LIO's body frame is
    # physically the lidar, not base_link (syncai_lio_bridge relies on exactly
    # that — see the lidar->base correction in its timer_cb).
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
            remappings=remappings,
            parameters=[params_file, frame_overrides],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
