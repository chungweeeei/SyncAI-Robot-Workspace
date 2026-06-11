# Launch the navigator node that chains planner_server and controller_server
# behind a single goal_pose topic (RViz "2D Goal Pose").

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_navigator')
    default_params_file = os.path.join(pkg_share, 'params', 'navigator_params.yaml')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot01',
        description='Namespace applied to the node (prefixes all relative topics/actions)',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the navigator parameters YAML file',
    )

    # use_sim_time deliberately comes ONLY from the params file — a launch
    # override placed after the file would silently win over the YAML value.
    navigator_node = Node(
        package='syncai_navigator',
        executable='navigator',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params_file,
        navigator_node,
    ])
