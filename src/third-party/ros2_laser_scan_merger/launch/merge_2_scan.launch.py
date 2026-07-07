#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#
#   Adapted for SyncAI: the two merger nodes run under the `namespace` arg
#   (supplied by syncai_bringup from the config/system.ini robot_id) so the
#   params file's RELATIVE topics resolve to /<ns>/scan_front, /<ns>/scan_rear,
#   /<ns>/cloud_in and the merged /<ns>/scan. TF frame names are NOT namespaced
#   by ROS, so pointCloudFrameId / target_frame are overridden below with the
#   <ns>/ prefix. The <ns>/base_link -> <ns>/scan static TF is published by the
#   top-level bringup launch (syncai_bringup), not here.
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
            # scan_front + scan_rear -> merged PointCloud2 on cloud_in.
            # The frame override (substitutions concatenate to "<ns>/scan")
            # is placed AFTER the params file so it wins over the YAML value.
            launch_ros.actions.Node(
                package="ros2_laser_scan_merger",
                executable="ros2_laser_scan_merger",
                namespace=namespace,
                parameters=[
                    params_file,
                    {"pointCloudFrameId": [namespace, "/scan"]},
                ],
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
                parameters=[
                    params_file,
                    {"target_frame": [namespace, "/scan"]},
                ],
            ),
        ]
    )
