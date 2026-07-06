# Launch the syncai_system_manager node — hosts the WiFi management services
# (scan_wifi / connect_wifi) that the backend gateway calls on demand.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot01",
        description="Namespace applied to the node (prefixes all relative services)",
    )

    system_manager_node = Node(
        package="syncai_system_manager",
        executable="system_manager_node",
        namespace=namespace,
        output="screen",
    )

    return LaunchDescription(
        [
            declare_namespace,
            system_manager_node,
        ]
    )
