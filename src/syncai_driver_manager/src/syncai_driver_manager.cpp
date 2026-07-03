#include "syncai_driver_manager/syncai_driver_manager.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>

namespace syncai_driver_manager
{
// Battery state publish rate in Hz. Also drives the discharge/charge timestep.
constexpr double kBatteryUpdateRate = 10.0;
// Period (seconds) over which battery_discharge_rate_ percent is applied.
constexpr double kBatteryDischargePeriod = 30.0;

DriverManagerNode::DriverManagerNode() : Node("driver_manager")
{
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initParameters();
  initPubSub();

  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] Driver Manager initialized successfully",
    __func__);
}

DriverManagerNode::~DriverManagerNode()
{
}

void DriverManagerNode::initParameters()
{
  this->declare_parameter("battery_initial_level", 100.0);
  battery_level_ = this->get_parameter("battery_initial_level").as_double();
  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] battery_initial_level: %f", __func__,
    battery_level_);

  this->declare_parameter("battery_discharge_rate", 0.1);
  battery_discharge_rate_ = this->get_parameter("battery_discharge_rate").as_double();
  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] battery_discharge_rate: %f", __func__,
    battery_discharge_rate_);

  is_charging_ = false;

  this->declare_parameter("base_frame", "base_link");
  base_frame_ = this->get_parameter("base_frame").as_string();
  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] base_frame: %s", __func__, base_frame_.c_str());
}

void DriverManagerNode::initPubSub()
{
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration<double>(1.0 / kBatteryUpdateRate);
  battery_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&DriverManagerNode::onTimer, this), timer_cb_group_);
}

void DriverManagerNode::onTimer()
{
  // Simulate charge/discharge over one timestep (battery_discharge_rate_ is % per 30 s).
  const double dt = 1.0 / kBatteryUpdateRate;
  const double delta = battery_discharge_rate_ * (dt / kBatteryDischargePeriod);
  battery_level_ += is_charging_ ? delta : -delta;
  battery_level_ = std::clamp(battery_level_, 0.0, 100.0);

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = base_frame_;

  // percentage is reported in percent (0-100).
  msg.percentage = static_cast<float>(battery_level_);
  msg.present = true;
  msg.power_supply_status = is_charging_
                              ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                              : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  // Fields we do not model are reported as unknown (NaN) per the message spec.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  msg.voltage = nan;
  msg.temperature = nan;
  msg.current = nan;
  msg.charge = nan;
  msg.capacity = nan;
  msg.design_capacity = nan;

  battery_pub_->publish(msg);
}
}  // namespace syncai_driver_manager
