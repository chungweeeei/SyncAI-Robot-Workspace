"""Standalone test for the static layer costmap.

Brings up:
  * map_server  (syncai_map_server) publishing a latched OccupancyGrid on /map
  * a static map -> base_link transform (robot parked at the map origin)
  * the costmap node with only the static layer enabled

Then inspect /costmap/costmap (see the package notes / chat for verification steps).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    costmap_share = get_package_share_directory("syncai_costmap_2d")
    default_params = os.path.join(costmap_share, "params", "costmap_params.yaml")

    # Absolute path to a map yaml. Defaults to the workspace map/ folder; override
    # with map_yaml:=/path/to/your.yaml
    default_map = "/home/syncrobotic/Documents/SyncAI-Robot-Workspace/map/testmap.yaml"

    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),

        # 1. Map source
        Node(
            package="syncai_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "topic_name": "/map",
                "frame_id": "map",
            }],
        ),

        # 2. Fake localization: robot at the map origin
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link",
            output="screen",
            arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
        ),

        # 3. The costmap node (static layer only)
        Node(
            package="syncai_costmap_2d",
            executable="costmap_2d_node",
            name="costmap",
            output="screen",
            parameters=[params_file],
        ),
    ])
