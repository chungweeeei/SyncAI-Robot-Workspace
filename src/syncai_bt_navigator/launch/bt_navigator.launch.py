# Launch the non-lifecycle bt_navigator node — the BT action server that
# exposes the NavigateToPose action and ticks the behavior tree. It talks to
# the planner (compute_path_to_pose) and controller (follow_path) action
# servers, so those must be running for a NavigateToPose goal to succeed.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_bt_navigator')
    default_params_file = os.path.join(pkg_share, 'params', 'bt_navigator_params.yaml')

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
        description='Full path to the bt_navigator parameters YAML file',
    )

    # No `name=` here: the params file uses the /**/bt_navigator wildcard key so
    # it matches in any namespace. use_sim_time comes ONLY from the params file —
    # a launch override placed after the file would silently win over the YAML.
    bt_navigator_node = Node(
        package='syncai_bt_navigator',
        executable='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params_file,
        bt_navigator_node,
    ])
