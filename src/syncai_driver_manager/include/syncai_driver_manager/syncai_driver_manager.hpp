#ifndef SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_
#define SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace syncai_driver_manager
{
class DriverManagerNode : public rclcpp::Node
{
public:
  DriverManagerNode();
  ~DriverManagerNode();

private:
  void initParameters();
  void initPubSub();

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr battery_timer_;
  void onTimer();

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  // Battery parameters
  double battery_level_;
  double battery_discharge_rate_;  // % per second
  bool is_charging_;

  std::string base_frame_;
};
}  // namespace syncai_driver_manager

#endif  // SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_
