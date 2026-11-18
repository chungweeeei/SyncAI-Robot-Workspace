#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_CHARGING_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_CHARGING_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "syncai_behavior_tree/charging_check.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief Return RUNNING until the battery reports charging for `hold_time`
 * ms, then SUCCESS; FAILURE after `timeout` ms. charge.xml's last step
 * (2026-11-18): lying on the charger is not the same as charging, and only
 * the battery can tell the two apart.
 *
 * Reads syncai_driver_manager's `battery_state` (sensor_msgs/BatteryState);
 * the rules are ChargingCheck's (charging_check.hpp): samples received after
 * onStart() only, CHARGING -- or not DISCHARGING at or above
 * `full_percentage` -- held for `hold_time` between samples.
 *
 * Unlike WaitForMotionState, whose latest sample counts whenever it
 * arrived, a sample from before onStart() is never used: it is from the walk
 * onto the charger. The subscription lives as long as the tree, so the
 * samples that arrive while another node runs are drained and dropped at
 * onStart().
 *
 * Own callback group and executor, spun without blocking on every tick, as
 * WaitForMotionState. On timeout it writes failed_node / failure_msg (the
 * report_failure() contract) with the last sample's status, current, voltage
 * and charge, which the navigator maps to DOCK_FAILED.
 */
class WaitForCharging : public BT::StatefulActionNode
{
public:
  WaitForCharging(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>(
        "hold_time", 2000, "Milliseconds the battery must read charging, between samples"),
      BT::InputPort<int>("timeout", 10000, "Milliseconds before giving up with FAILURE"),
      BT::InputPort<double>(
        "full_percentage", 0.95,
        "Charge (0-1) at or above which not discharging is enough: a full pack's current "
        "tapers into the deadband"),
      BT::InputPort<std::string>(
        "topic", "battery_state", "sensor_msgs/BatteryState topic, relative"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void onBattery(sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  // The last sample in words, for the failure message.
  std::string lastSample() const;
  void fail(const std::string & msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;

  // Fed only between onStart() and the end of the run; the callback runs on
  // the tick's thread (spin_some), so no lock.
  bool running_{false};
  ChargingCheck check_{2.0, 0.95};
  bool charging_{false};
  int samples_{0};
  sensor_msgs::msg::BatteryState::ConstSharedPtr last_;
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_CHARGING_HPP_
