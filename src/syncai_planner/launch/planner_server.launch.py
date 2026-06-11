# Launch the non-lifecycle planner server (ComputePathToPose action) with its
# internal global costmap.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_planner')
    default_params_file = os.path.join(pkg_share, 'params', 'planner_server_params.yaml')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot01',
        description='Namespace applied to the nodes (prefixes all relative topics/actions)',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the planner server parameters YAML file',
    )

    # NOTE: no `name=` here. The process hosts two nodes (planner_server and
    # its internal global_costmap); a launch-level name would remap BOTH to
    # the same name and the costmap would lose its parameters. The namespace
    # remap is fine: it applies to both nodes, and the params file uses /**/
    # wildcard keys so they match in any namespace.
    # use_sim_time deliberately comes ONLY from the params file — a launch
    # override placed after the file would silently win over the YAML value.
    planner_server_node = Node(
        package='syncai_planner',
        executable='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params_file,
        planner_server_node,
    ])
