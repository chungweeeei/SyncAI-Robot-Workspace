#include "syncai_behavior_tree/plugins/action/wait_for_charging.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

namespace
{

const char * statusName(uint8_t status)
{
  switch (status) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      return "CHARGING";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      return "DISCHARGING";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return "NOT_CHARGING";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      return "FULL";
    default:
      return "UNKNOWN";
  }
}

double secondsSince(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

static_assert(
  ChargingCheck::kStatusCharging == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING,
  "ChargingCheck's copy of POWER_SUPPLY_STATUS_CHARGING");
static_assert(
  ChargingCheck::kStatusDischarging ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
  "ChargingCheck's copy of POWER_SUPPLY_STATUS_DISCHARGING");

WaitForCharging::WaitForCharging(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic = "battery_state";
  getInput("topic", topic);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // Depth 10, reliable: what driver_manager publishes battery_state with.
  sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10),
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) { onBattery(msg); }, options);
}

BT::NodeStatus WaitForCharging::onStart()
{
  int hold_ms = 2000;
  int timeout_ms = 10000;
  double full_percentage = 0.95;
  getInput("hold_time", hold_ms);
  getInput("timeout", timeout_ms);
  getInput("full_percentage", full_percentage);
  if (hold_ms < 0 || timeout_ms <= 0 || !(full_percentage > 0.0)) {
    fail("WaitForCharging: hold_time must be >= 0, timeout > 0 and full_percentage > 0");
    return BT::NodeStatus::FAILURE;
  }

  // Drop what arrived while the node was idle (the walk onto the charger)
  // before the run starts: those samples say nothing about the contacts.
  running_ = false;
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));

  check_ = ChargingCheck(hold_ms / 1000.0, full_percentage);
  charging_ = false;
  samples_ = 0;
  last_.reset();
  running_ = true;
  start_ = std::chrono::steady_clock::now();
  deadline_ = start_ + std::chrono::milliseconds(timeout_ms);
  return onRunning();
}

BT::NodeStatus WaitForCharging::onRunning()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  if (charging_) {
    running_ = false;
    // The number the guessed timeout lacks: how long after the lie-down the
    // battery read charging.
    RCLCPP_INFO(
      node_->get_logger(), "[WaitForCharging] charging after %.1f s: %s", secondsSince(start_),
      lastSample().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    running_ = false;
    fail(
      "the battery did not report charging within " +
      std::to_string(static_cast<int>(std::lround(secondsSince(start_)))) + " s (" +
      std::to_string(samples_) + " samples on " + sub_->get_topic_name() +
      "; last seen: " + lastSample() + ")");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForCharging::onHalted()
{
  // The subscription stays; samples are ignored until the next onStart().
  running_ = false;
}

void WaitForCharging::onBattery(sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  if (!running_ || charging_) {
    return;
  }
  last_ = msg;
  ++samples_;
  // Arrival time, not the header stamp: driver_manager stamps on receipt
  // anyway, and the run is judged against the steady clock it started on.
  charging_ = check_.add(secondsSince(start_), msg->power_supply_status, msg->percentage);
}

std::string WaitForCharging::lastSample() const
{
  if (!last_) {
    return "nothing received";
  }
  char buf[128];
  std::snprintf(
    buf, sizeof(buf), "%s, %+.2f A, %.1f V, %.0f %%", statusName(last_->power_supply_status),
    static_cast<double>(last_->current), static_cast<double>(last_->voltage),
    100.0 * static_cast<double>(last_->percentage));
  return buf;
}

void WaitForCharging::fail(const std::string & msg)
{
  RCLCPP_ERROR(node_->get_logger(), "[WaitForCharging] %s", msg.c_str());
  // report_failure()'s contract, as WaitForMotionState::fail().
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kFailedNode, registrationName());
  config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, msg);  // NOLINT
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::WaitForCharging>("WaitForCharging");
}
