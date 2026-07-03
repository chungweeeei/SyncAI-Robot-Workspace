#ifndef SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_
#define SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "syncai_common/msg/robot_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_robot_state
{
class RobotStateNode : public rclcpp::Node
{
public:
  RobotStateNode();
  ~RobotStateNode();

private:
  void initParameters();

  // Periodic (1 Hz) build-and-publish of the robot state.
  void onTimer();
  // Caches the latest forward velocity from the odom topic.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Caches the latest battery state from the battery_state topic.
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<rclcpp::TimerBase> timer_;
  // Dedicated group so the timer's build/TF/publish work runs independently of
  // the (lightweight) sensor subscription callbacks under a multi-threaded executor.
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  std::shared_ptr<rclcpp::Publisher<syncai_common::msg::RobotState>> robot_state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // Latest odom / battery samples, guarded because they are written from their
  // subscription callbacks and read from the timer callback.
  std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;

  // Parameters.
  std::string robot_id_;
  std::string map_name_;
  std::string global_frame_, base_frame_;
  std::string odom_topic_;
  double transform_tolerance_;
};
}  // namespace syncai_robot_state

#endif  // SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_
