#include "syncai_util/odometry_utils.hpp"

#include <string>

using namespace std::chrono;           // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace syncai_util
{

OdomSmoother::OdomSmoother(
  const rclcpp::Node::SharedPtr & node, double filter_duration, const std::string & odom_topic)
: odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
{
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

void OdomSmoother::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  if (!odom_history_.empty()) {
    // To store current time (received msg timestamp)
    auto current_time = rclcpp::Time(msg->header.stamp);

    // To store time of the first odom in history
    auto front_time = rclcpp::Time(odom_history_.front().header.stamp);

    // update cumulated odom when duration has exceeded and pop earliest msg
    while (current_time - front_time > odom_history_duration_) {
      const auto & odom = odom_history_.front();
      // 舊的odometry msg離開window時 -> 把它「減」掉
      odom_cumulate_.twist.twist.linear.x -= odom.twist.twist.linear.x;
      odom_cumulate_.twist.twist.linear.y -= odom.twist.twist.linear.y;
      odom_cumulate_.twist.twist.linear.z -= odom.twist.twist.linear.z;
      odom_cumulate_.twist.twist.angular.x -= odom.twist.twist.angular.x;
      odom_cumulate_.twist.twist.angular.y -= odom.twist.twist.angular.y;
      odom_cumulate_.twist.twist.angular.z -= odom.twist.twist.angular.z;
      odom_history_.pop_front();

      if (odom_history_.empty()) {
        break;
      }

      // update with the timestamp of earliest odom message in history
      front_time = rclcpp::Time(odom_history_.front().header.stamp);
    }
  }

  odom_history_.push_back(*msg);
  updateState();
}

void OdomSmoother::updateState()
{
  const auto & odom = odom_history_.back();

  // odom_cumulate_ 是「目前窗口內所有odom速度的總和」。
  // 新的odometry msg進入window時 -> 把它「加」進總和
  odom_cumulate_.twist.twist.linear.x += odom.twist.twist.linear.x;
  odom_cumulate_.twist.twist.linear.y += odom.twist.twist.linear.y;
  odom_cumulate_.twist.twist.linear.z += odom.twist.twist.linear.z;
  odom_cumulate_.twist.twist.angular.x += odom.twist.twist.angular.x;
  odom_cumulate_.twist.twist.angular.y += odom.twist.twist.angular.y;
  odom_cumulate_.twist.twist.angular.z += odom.twist.twist.angular.z;

  vel_smooth_.header = odom.header;
  vel_smooth_.twist.linear.x = odom_cumulate_.twist.twist.linear.x / odom_history_.size();
  vel_smooth_.twist.linear.y = odom_cumulate_.twist.twist.linear.y / odom_history_.size();
  vel_smooth_.twist.linear.z = odom_cumulate_.twist.twist.linear.z / odom_history_.size();
  vel_smooth_.twist.angular.x = odom_cumulate_.twist.twist.angular.x / odom_history_.size();
  vel_smooth_.twist.angular.y = odom_cumulate_.twist.twist.angular.y / odom_history_.size();
  vel_smooth_.twist.angular.z = odom_cumulate_.twist.twist.angular.z / odom_history_.size();
}

}  // namespace syncai_util