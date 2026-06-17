#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#
#   Adapted for SyncAI: the two merger nodes run under the `namespace` arg
#   (default robot01) so the params file's RELATIVE topics resolve to
#   /<ns>/scan_front, /<ns>/scan_rear, /<ns>/cloud_in and the merged /<ns>/scan.
#   A static_transform_publisher places the merged-scan frame (<ns>/scan) over
#   the robot base (<ns>/base_link) at the lidar mount height. It is left on the
#   GLOBAL /tf (no namespace) so it joins the same TF tree the sim publishes.
#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("ros2_laser_scan_merger"), "params", "params.yaml"
    )

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot01",
        description="Namespace for the merger nodes / tf prefix for the frames",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Full path to the laser scan merger parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_params_file,
            # scan_front + scan_rear -> merged PointCloud2 on cloud_in
            launch_ros.actions.Node(
                package="ros2_laser_scan_merger",
                executable="ros2_laser_scan_merger",
                namespace=namespace,
                parameters=[params_file],
                output="screen",
                respawn=True,
                respawn_delay=2,
            ),
            # cloud_in (PointCloud2) -> merged scan (LaserScan)
            launch_ros.actions.Node(
                name="pointcloud_to_laserscan",
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                namespace=namespace,
                parameters=[params_file],
            ),
            # Static TF: <ns>/base_link -> <ns>/scan at the lidar mount height
            # (0.175 * 0.7 = 0.1225 m). No namespace on the node so it publishes
            # to the GLOBAL /tf shared with the sim's odom->base_link tree.
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_scan_tf",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.1225",
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
            ),
        ]
    )
