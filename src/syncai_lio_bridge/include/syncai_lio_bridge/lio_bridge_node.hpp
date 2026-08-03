#ifndef SYNCAI_LIO_BRIDGE__LIO_BRIDGE_NODE_HPP_
#define SYNCAI_LIO_BRIDGE__LIO_BRIDGE_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_lio_bridge
{

// Planar (x, y, yaw) projection of a rigid transform. Matches the Python
// project_2d(): yaw = atan2(R[1][0], R[0][0]), translation x/y taken directly.
//
// Lives in the header rather than the .cpp because it appears in the private
// make_tf() signature; the transform <-> Pose2D conversions stay private to the
// .cpp since nothing outside the node needs them.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// LIO odometry provider — the robot's only odometry source (C++ port).
//
// Wheel odometry is gone (the Isaac Sim OmniGraph odom publishers are
// disabled): the robot relies solely on Point-LIO lidar-inertial odometry.
// This node turns the LIO chain (map -> lio_odom -> lio_body) into the planar
// chain the nav stack consumes:
//
//     odom -> base_link   from /<robot_id>/pointlio/lio_odom, projected to 2D
//                         (lio_body is physically <robot_id>/lidar_top, so the
//                         static base_link->lidar_top extrinsic maps it to
//                         base)
//     /<robot_id>/odom    nav_msgs/Odometry republished for twist consumers
//                         (controller_server, task_runner, robot_state); linear
//                         velocity comes from LIO (body frame), angular.z from
//                         the lidar IMU gyro since LIO leaves twist.angular
//                         empty
//     map -> odom         AMCL-style correction using the localizer's
//                         map -> lio_odom TF:
//                         map->odom = P2D(map->base) * inv(P2D(odom->base))
//
// Everything is projected to 2D (x, y, yaw; z/roll/pitch zeroed) before
// broadcasting so the planar nav stack never sees a tilted frame.
//
// This is a straight port of the original Python node; the numpy 4x4
// homogeneous-matrix math is replaced with tf2::Transform to avoid the Python
// per-tick CPU cost of numpy allocations and the tf2 Python bindings.
class LioBridgeNode : public rclcpp::Node
{
public:
  LioBridgeNode();

private:
  void lio_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Cached static base_link -> lidar_top mount extrinsic.
  bool base_lidar(tf2::Transform & out);

  geometry_msgs::msg::TransformStamped make_tf(
    const rclcpp::Time & stamp, const std::string & frame_id, const std::string & child_frame_id,
    const Pose2D & p) const;

  void timer_cb();

  // Parameters.
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
  double transform_tolerance_{0.1};

  // TF.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ROS I/O.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest LIO state (set in callbacks, consumed by the timer).
  std::optional<tf2::Transform> lio_pose_;    // lio_odom -> lio_body
  std::string lio_odom_frame_;                // from the msg header
  double lio_linear_x_{0.0};                  // body-frame linear velocity x
  double lio_linear_y_{0.0};                  // body-frame linear velocity y
  double yaw_rate_{0.0};                      // from the lidar IMU gyro
  std::optional<tf2::Transform> base_lidar_;  // base_link -> lidar_top (cached)

  bool localized_{false};
};

}  // namespace syncai_lio_bridge

#endif  // SYNCAI_LIO_BRIDGE__LIO_BRIDGE_NODE_HPP_
