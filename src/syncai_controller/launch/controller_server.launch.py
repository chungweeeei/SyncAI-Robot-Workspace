# Launch the non-lifecycle controller server (FollowPath action) with its
# internal local costmap and the DWB local planner plugin.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('syncai_controller')
    default_params_file = os.path.join(pkg_share, 'params', 'controller_server_params.yaml')

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
        description='Full path to the controller server parameters YAML file',
    )

    # NOTE: no `name=` here. The process hosts two nodes (controller_server
    # and its internal local_costmap); a launch-level name would remap BOTH
    # to the same name and the costmap would lose its parameters. The
    # namespace remap is fine: it applies to both nodes, and the params file
    # uses /**/ wildcard keys so they match in any namespace.
    # use_sim_time deliberately comes ONLY from the params file — a launch
    # override placed after the file would silently win over the YAML value.
    controller_server_node = Node(
        package='syncai_controller',
        executable='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params_file,
        controller_server_node,
    ])
