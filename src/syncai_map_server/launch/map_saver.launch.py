import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_map_server')
    default_params_file = os.path.join(pkg_share, 'params', 'map_saver_params.yaml')

    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for the map_saver node',
    )

    map_saver_node = Node(
        package='syncai_map_server',
        executable='map_saver',
        name='map_saver_server',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_params_file,
        map_saver_node,
    ])
