"""Launch the syncai_backend REST API node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot01",
        description="Namespace applied to the node (prefixes all relative topics)",
    )

    # No `name=`: the params file uses the /**/syncai_backend_node wildcard key so it
    # matches the node (constructed as "syncai_backend_node") in any namespace.
    backend_node = Node(
        package="syncai_backend",
        executable="backend",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            declare_namespace,
            backend_node,
        ]
    )
