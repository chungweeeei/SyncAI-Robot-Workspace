#ifndef SYNCAI_POINTLIO__POINTLIO_NODE_HPP_
#define SYNCAI_POINTLIO__POINTLIO_NODE_HPP_

// The ROS shell around Point-LIO. Everything numeric lives in map_builder/
// (point_ekf, imu_initializer, lidar_processor, ikd_Tree); this class owns the
// subscriptions, the sync of a lidar frame with the IMU samples covering it,
// the publishers / TF broadcaster, and the in-place `reset` service.
//
// Ported into the workspace from SyncAI-Fast-LIO2's `pointlio` package in
// 2026-09 with the ROS surface unchanged: node name `pointlio_node`, namespace
// /<robot_id>/pointlio (set by the launch), topics lio_odom / body_cloud /
// world_cloud / lio_path, service `reset`, TF world_frame -> body_frame. The
// only rename is the service *type*, which moved from that repo's `interface`
// package to syncai_common when server and client stopped sharing a repo.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "syncai_common/srv/reset_lio.hpp"
#include "syncai_pointlio/map_builder/commons.h"
#include "syncai_pointlio/map_builder/map_builder.h"
#include "syncai_pointlio/utils.h"

namespace syncai_pointlio
{

// Input topics are NOT parameters: the node subscribes to the relative names
// "lidar" and "imu" and the launch file remaps them onto the driver's
// /<robot_id>/livox/{lidar,imu} (a relative name alone cannot reach them —
// the node runs in the /<robot_id>/pointlio namespace, so "lidar" would resolve
// to /<robot_id>/pointlio/lidar). Remapping is the ROS-native knob for this and
// keeps `ros2 node info` honest about where the data comes from.
struct NodeConfig
{
  std::string body_frame = "body";
  std::string world_frame = "lidar";
  bool print_time_cost = false;
  int lidar_type = 0;
  // 0: livox_ros_driver2 CustomMsg, 1: sensor_msgs/PointCloud2 (e.g. Isaac Sim)
  // Scale applied to linear_acceleration. Livox IMU reports g-units -> ~10.0 to get m/s^2.
  // Sensors already in m/s^2 (e.g. Isaac Sim, z~9.81 at rest) must use 1.0, since the
  // filter's gravity magnitude is fixed at 9.81 (point_ekf.cpp). satu_acc is compared
  // against the scaled value.
  double imu_acc_scale = 10.0;
};

struct StateData
{
  bool lidar_pushed = false;

  std::mutex imu_mutex;
  std::mutex lidar_mutex;

  double last_lidar_time = -1.0;
  double last_imu_time = -1.0;

  std::deque<IMUData> imu_buffer;
  std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;

  nav_msgs::msg::Path path;
};

class PointLIONode : public rclcpp::Node
{
public:
  PointLIONode();

  // Declares every parameter with the struct default as its default. See the
  // definition for why these are ROS parameters and not a hand-parsed YAML.
  void loadParameters();

  // Subscription callbacks: buffer only, no processing.
  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void pc2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Assembles m_package from one lidar frame plus the IMU samples that cover
  // it; false while the IMU has not caught up to the end of the scan.
  bool syncPackage();

  void publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
    std::string frame_id, const double & time);
  void publishOdometry(
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
    std::string child_frame, const double & time);
  void publishPath(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id,
    const double & time);
  void broadCastTF(
    std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
    std::string child_frame, const double & time);

  // 20 ms wall timer: sync, process, publish.
  void timerCB();

  // The `reset` service: throw the whole solution away and start over, in
  // place. Relies on the single-threaded executor (see main.cpp and the
  // definition's comment) -- it takes no lock against timerCB.
  void resetCB(
    const std::shared_ptr<syncai_common::srv::ResetLIO::Request> request,
    std::shared_ptr<syncai_common::srv::ResetLIO::Response> response);

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Service<syncai_common::srv::ResetLIO>::SharedPtr m_reset_srv;
  // Stamp of the last odometry sample this node published, in the lidar's clock
  // (the same value that goes into lio_odom's and body_cloud's headers). Kept
  // only so resetCB can hand it to pgo as the boundary between the two runs;
  // 0.0 until MAPPING is first reached, and again after every reset.
  double m_last_odom_time = 0.0;
  StateData m_state_data;
  SyncPackage m_package;
  NodeConfig m_node_config;
  Config m_builder_config;
  std::shared_ptr<PointEKF> m_kf;
  std::shared_ptr<MapBuilder> m_builder;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

}  // namespace syncai_pointlio

#endif  // SYNCAI_POINTLIO__POINTLIO_NODE_HPP_
