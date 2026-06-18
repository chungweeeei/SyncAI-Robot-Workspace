# Copyright (c) 2024 SyncAI
#
# Launch the non-lifecycle AMCL localization node.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_amcl')
    default_params_file = os.path.join(pkg_share, 'params', 'amcl.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot01',
        description='Namespace applied to the node; prefixes relative topics '
                    '(scan, map, amcl_pose, initialpose, particle_cloud).',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the AMCL parameters YAML file',
    )

    amcl_node = Node(
        package='syncai_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        amcl_node,
    ])
