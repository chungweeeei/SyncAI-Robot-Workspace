"""Standalone test for the keepout costmap filter.

Brings up:
  * map_server (syncai_map_server) publishing a latched OccupancyGrid on /map
  * a second map_server instance publishing the keepout mask on /keepout_filter_mask
  * costmap_filter_info_server publishing the CostmapFilterInfo on /costmap_filter_info
  * a static map -> base_link transform (robot parked at the map origin)
  * the costmap node with the static layer + keepout filter enabled

The default mask (map/keepout_mask_test.yaml) contains a ~1x1 m keepout
rectangle at world x:[0, 1], y:[0, 1]; /costmap/costmap should show occupancy
100 there even though the map itself is free.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    costmap_share = get_package_share_directory("syncai_costmap_2d")
    default_params = os.path.join(costmap_share, "params", "costmap_keepout_test.yaml")

    # Absolute paths inside the robot container; override with map_yaml:= /
    # mask_yaml:= when running elsewhere.
    default_map = "map/testmap.yaml"
    default_mask = "map/keepout_mask_test.yaml"

    map_yaml = LaunchConfiguration("map_yaml")
    mask_yaml = LaunchConfiguration("mask_yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument("mask_yaml", default_value=default_mask),
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

        # 2. Keepout mask source (second map_server instance)
        Node(
            package="syncai_map_server",
            executable="map_server",
            name="filter_mask_server",
            output="screen",
            parameters=[{
                "yaml_filename": mask_yaml,
                "topic_name": "/keepout_filter_mask",
                "frame_id": "map",
            }],
        ),

        # 3. Filter info source
        Node(
            package="syncai_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            parameters=[{
                "filter_info_topic": "/costmap_filter_info",
                "type": 0,
                "mask_topic": "/keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            }],
        ),

        # 4. Fake localization: robot at the map origin
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link",
            output="screen",
            arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
        ),

        # 5. The costmap node (static layer + keepout filter)
        Node(
            package="syncai_costmap_2d",
            executable="costmap_2d_node",
            name="costmap",
            output="screen",
            parameters=[params_file],
        ),
    ])
