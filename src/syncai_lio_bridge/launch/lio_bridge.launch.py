# Launch the syncai_lio_bridge node — the robot's only odometry source.
# From Point-LIO (pointlio/lio_odom + livox/imu) it publishes the planar
# odom -> <robot_id>/base_link TF, the <robot_id>/odom Odometry topic, and
# the map -> <robot_id>/odom correction (replacing AMCL). There is no wheel
# odometry: the Isaac Sim OmniGraph odom publishers are disabled.
#
# robot_id is read from the system config INI at launch time and is used as
# the node namespace and as the TF prefix for the odom chain frames. The map
# frame stays "map".

import configparser

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

logger = launch_logging.get_logger("lio_bridge.launch")


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

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    lio_bridge_node = Node(
        package="syncai_lio_bridge",
        executable="lio_bridge_node",
        name="lio_bridge_node",
        namespace=robot_id,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "base_frame": f"{robot_id}/base_link",
                "odom_frame": f"{robot_id}/odom",
                "lidar_frame": f"{robot_id}/lidar_top",
                "publish_rate": 20.0,
                "transform_tolerance": 0.1,
            }
        ],
    )

    return [lio_bridge_node]


def generate_launch_description():
    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock (Isaac Sim publishes /clock)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_use_sim_time,
            OpaqueFunction(function=launch_setup),
        ]
    )
