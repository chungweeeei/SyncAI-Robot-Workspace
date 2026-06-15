#ifndef SYNCAI_UTIL__ODOMETRY_UTILS_HPP_
#define SYNCAI_UTIL__ODOMETRY_UTILS_HPP_

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_util/node_utils.hpp"

namespace syncai_util
{
class OdomSmoother
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    const rclcpp::Node::SharedPtr & node, double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  /**
   * @brief Get twist msg from smoother
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getTwist() { return vel_smooth_.twist; }

  /**
   * @brief Get twist stamped msg from smoother
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getTwistStamped() { return vel_smooth_; }

protected:
  /**
   * @brief Callback of odometry subscriber to process
   * @param msg Odometry msg to smooth
   */
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update internal state of the smoother after getting new data
   */
  void updateState();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_cumulate_;
  geometry_msgs::msg::TwistStamped vel_smooth_;
  std::mutex odom_mutex_;

  rclcpp::Duration odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};
}  // namespace syncai_util

#endif