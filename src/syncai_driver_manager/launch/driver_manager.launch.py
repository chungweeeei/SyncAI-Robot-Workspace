# Launch the syncai_driver_manager node — monitors the robot's hardware drivers.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_driver_manager")
    default_params_file = os.path.join(pkg_share, "params", "driver_manager_params.yaml")

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
        description="Full path to the driver_manager parameters YAML file",
    )

    # No `name=`: the params file uses the /**/driver_manager wildcard key so it
    # matches the node (constructed as "driver_manager") in any namespace.
    driver_manager_node = Node(
        package="syncai_driver_manager",
        executable="driver_manager_node",
        namespace=namespace,
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_params_file,
            driver_manager_node,
        ]
    )
