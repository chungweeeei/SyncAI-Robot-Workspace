"""Standalone test for the full costmap layer stack (static + obstacle + inflation).

Brings up:
  * map_server  (syncai_map_server) publishing a latched OccupancyGrid on /map
  * map -> base_link        static transform (robot parked at the map origin)
  * base_link -> base_scan  static transform (the laser sensor frame)
  * the costmap node with static_layer + obstacle_layer + inflation_layer
  * (optional) a fake LaserScan publisher on /scan, enabled with use_fake_scan:=true

On a real robot just feed your own /scan (set the topic/frame in
params/costmap_three_layer.yaml) and launch with use_fake_scan:=false (default).

Inspect /costmap (transient_local QoS) in rviz or with `ros2 topic echo`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    costmap_share = get_package_share_directory("syncai_costmap_2d")
    default_params = os.path.join(costmap_share, "params", "costmap_three_layer.yaml")
    default_map = "/home/syncrobotic/Documents/SyncAI-Robot-Workspace/map/testmap.yaml"

    rviz_config = os.path.join(costmap_share, "rviz", "costmap.rviz")

    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    use_fake_scan = LaunchConfiguration("use_fake_scan")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "use_fake_scan", default_value="false",
            description="Publish a synthetic /scan so the obstacle layer has data"),
        DeclareLaunchArgument(
            "rviz", default_value="false", description="Open rviz2 with the costmap config"),

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

        # 3. Laser mount transform (the obstacle layer's sensor_frame)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_base_scan",
            output="screen",
            arguments=[
                "--x", "0.2", "--z", "0.15",
                "--frame-id", "base_link", "--child-frame-id", "base_scan"],
        ),

        # 4. The costmap node (3 layers)
        Node(
            package="syncai_costmap_2d",
            executable="costmap_2d_node",
            name="costmap",
            output="screen",
            parameters=[params_file],
        ),

        # 5. Optional synthetic scan for end-to-end testing without a robot.
        # Uses a small node (not `ros2 topic pub`) so the scan carries a current
        # timestamp; otherwise tf2_ros::MessageFilter drops it as too old.
        Node(
            condition=IfCondition(use_fake_scan),
            package="syncai_costmap_2d",
            executable="fake_scan.py",
            name="fake_scan",
            output="screen",
        ),

        # 6. Optional rviz2 visualization
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
