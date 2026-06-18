"""Launch the syncai_backend REST API node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host', default_value='0.0.0.0',
            description='Interface the REST API binds to'),
        DeclareLaunchArgument(
            'port', default_value='8080',
            description='Port the REST API listens on'),
        Node(
            package='syncai_backend',
            executable='backend',
            name='syncai_backend',
            output='screen',
            parameters=[{'host': host, 'port': port}],
        ),
    ])
