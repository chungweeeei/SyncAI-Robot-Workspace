# Launch the syncai_robot_state node — publishes syncai_common/RobotState at 1 Hz,
# with position from TF (map -> base_link) and forward velocity from the odom topic.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_robot_state")
    default_params_file = os.path.join(pkg_share, "params", "robot_state_params.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot01",
        description="Namespace applied to the node (prefixes all relative topics)",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the robot_state parameters YAML file",
    )

    # No `name=`: the params file uses the /**/syncai_robot_state wildcard key so it
    # matches in any namespace. The {'map': ...} override is placed AFTER the params
    # file so a launch-supplied map wins over the YAML default.
    robot_state_node = Node(
        package="syncai_robot_state",
        executable="robot_state_node",
        namespace=namespace,
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_params_file,
            robot_state_node,
        ]
    )
