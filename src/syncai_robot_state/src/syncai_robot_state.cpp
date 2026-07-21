#include "syncai_robot_state/syncai_robot_state.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <functional>

#include "syncai_common/msg/robot_mode.hpp"
#include "syncai_common/msg/robot_status.hpp"
#include "syncai_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace syncai_robot_state
{
using std::placeholders::_1;

RobotStateNode::RobotStateNode() : Node("syncai_robot_state")
{
  initParameters();

  // register tf buffer and listener
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  robot_state_pub_ = this->create_publisher<syncai_common::msg::RobotState>(
    "robot_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&RobotStateNode::odomCallback, this, _1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS(),
    std::bind(&RobotStateNode::batteryCallback, this, _1));

  wifi_sub_ = this->create_subscription<syncai_common::msg::WifiStatus>(
    "wifi_status", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    std::bind(&RobotStateNode::wifiStatusCallback, this, _1));

  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RobotStateNode::onTimer, this), timer_cb_group_);
}

RobotStateNode::~RobotStateNode()
{
}

void RobotStateNode::initParameters()
{
  this->declare_parameter("robot_id", std::string(""));
  this->get_parameter("robot_id", robot_id_);
  RCLCPP_INFO(this->get_logger(), "[RobotStateNode][%s] robot_id: %s", __func__, robot_id_.c_str());

  this->declare_parameter("map", std::string(""));
  this->get_parameter("map", map_name_);
  RCLCPP_INFO(this->get_logger(), "[RobotStateNode][%s] map: %s", __func__, map_name_.c_str());

  this->declare_parameter("global_frame", std::string("map"));
  this->get_parameter("global_frame", global_frame_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] global_frame: %s", __func__, global_frame_.c_str());

  this->declare_parameter("base_frame", std::string("base_link"));
  this->get_parameter("base_frame", base_frame_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] base_frame: %s", __func__, base_frame_.c_str());

  this->declare_parameter("transform_tolerance", 0.1);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] transform_tolerance: %f", __func__,
    transform_tolerance_);
}

void RobotStateNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_ = msg;
}

void RobotStateNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_battery_ = msg;
}

void RobotStateNode::wifiStatusCallback(const syncai_common::msg::WifiStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_wifi_status_ = msg;
}

void RobotStateNode::onTimer()
{
  syncai_common::msg::RobotState msg;
  msg.timestamp = static_cast<uint64_t>(this->now().seconds());
  msg.robot_id = robot_id_;
  msg.map = map_name_;
  // {TODO} mode is currently hardcoded to AUTO
  msg.mode = syncai_common::msg::RobotMode::AUTO;  // default for now
  // {TODO} state derivation not implemented yet; hardcoded to IDLE
  msg.state = syncai_common::msg::RobotStatus::IDLE;

  // position from TF (global_frame -> base_frame)
  geometry_msgs::msg::PoseStamped pose;
  if (!syncai_util::getCurrentPose(pose, *tf_, global_frame_, base_frame_, transform_tolerance_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF %s->%s unavailable; skipping this robot_state tick", global_frame_.c_str(),
      base_frame_.c_str());
    return;
  }
  msg.localization_status.position.x = pose.pose.position.x;
  msg.localization_status.position.y = pose.pose.position.y;
  msg.localization_status.position.z = pose.pose.position.z;
  msg.localization_status.position.yaw = tf2::getYaw(pose.pose.orientation);

  // velocity from odom (forward linear speed) and battery level from battery_state
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.localization_status.velocity = latest_odom_ ? latest_odom_->twist.twist.linear.x : 0.0;
    msg.battery_status.battery_percentage = latest_battery_ ? latest_battery_->percentage : 0.0;

    // Flatten the latest WifiStatus into a JSON string; "N/A" until the
    // first wifi_status message arrives.
    nlohmann::json wifi_json;
    if (latest_wifi_status_) {
      wifi_json = {
        {"ssid", latest_wifi_status_->ssid},
        {"bssid", latest_wifi_status_->bssid},
        {"rssi", latest_wifi_status_->rssi},
        {"ip_address", latest_wifi_status_->ip_address},
        {"mac_address", latest_wifi_status_->mac_address},
      };
    }
    msg.network_status.wifi_info = wifi_json.dump();
  }

  robot_state_pub_->publish(msg);
};
}  // namespace syncai_robot_state
