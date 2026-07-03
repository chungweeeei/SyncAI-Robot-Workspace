import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Top-level launch for bringing up the SyncAI navigation stack."""

    laser_scan_merger_dir = get_package_share_directory("ros2_laser_scan_merger")

    namespace = LaunchConfiguration("namespace")
    scan_height = LaunchConfiguration("scan_height")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot01",
        description="Namespace for the merger nodes / tf prefix for the frames",
    )
    declare_scan_height = DeclareLaunchArgument(
        "scan_height",
        default_value="0.1225",
        description="Lidar mount height (m) for the <ns>/base_link -> <ns>/scan TF "
        "(0.175 * 0.7 = 0.1225)",
    )

    # scan_front + scan_rear -> merged /<ns>/scan (LaserScan)
    merge_laser_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_scan_merger_dir, "launch", "merge_2_scan.launch.py")
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    # Static TF: <ns>/base_link -> <ns>/scan at the lidar mount height.
    # No namespace on the node so it publishes to the GLOBAL /tf shared with
    # the sim's odom->base_link tree.
    base_link_to_scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_scan_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            scan_height,
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            [namespace, "/base_link"],
            "--child-frame-id",
            [namespace, "/scan"],
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_scan_height,
            merge_laser_scan,
            base_link_to_scan_tf,
        ]
    )
