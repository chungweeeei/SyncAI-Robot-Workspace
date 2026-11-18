#include "syncai_behavior_tree/plugins/action/wait_for_motion_state.hpp"

#include <chrono>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

WaitForMotionState::WaitForMotionState(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic = "mode";
  getInput("topic", topic);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // Depth 10, reliable: what driver_manager publishes `mode` with. Only the
  // newest sample matters, so a backlog is drained and overwritten.
  sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    topic, rclcpp::QoS(10),
    [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) { latest_ = msg->data; }, options);
}

BT::NodeStatus WaitForMotionState::onStart()
{
  int timeout_ms = 5000;
  if (!getInput("state", target_)) {
    fail("WaitForMotionState: the state port must be set");
    return BT::NodeStatus::FAILURE;
  }
  getInput("index", index_);
  getInput("timeout", timeout_ms);
  if (index_ < 0 || timeout_ms <= 0) {
    fail("WaitForMotionState: index must be >= 0 and timeout > 0");
    return BT::NodeStatus::FAILURE;
  }
  start_ = std::chrono::steady_clock::now();
  deadline_ = start_ + std::chrono::milliseconds(timeout_ms);
  return onRunning();
}

BT::NodeStatus WaitForMotionState::onRunning()
{
  if (reached()) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_);
    // The number the fixed Waits never had: how long the gait took to report
    // the state after the tree reached this node.
    RCLCPP_INFO(
      node_->get_logger(), "[WaitForMotionState] mode[%d] == %d after %ld ms", index_, target_,
      static_cast<long>(elapsed.count()));  // NOLINT(runtime/int)
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    std::string last;
    if (static_cast<size_t>(index_) < latest_.size()) {
      last = std::to_string(latest_[static_cast<size_t>(index_)]);
    } else if (latest_.empty()) {
      last = std::string("nothing received on ") + sub_->get_topic_name();
    } else {
      last = "a sample too short for the index";
    }
    fail(
      "the gait controller did not report motion state " + std::to_string(target_) + " on mode[" +
      std::to_string(index_) + "] in time (last seen: " + last + ")");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForMotionState::onHalted()
{
  // Nothing to undo: the subscription stays, the next onStart() takes a
  // fresh deadline.
}

bool WaitForMotionState::reached()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  return static_cast<size_t>(index_) < latest_.size() &&
         latest_[static_cast<size_t>(index_)] == target_;
}

void WaitForMotionState::fail(const std::string & msg)
{
  RCLCPP_ERROR(node_->get_logger(), "[WaitForMotionState] %s", msg.c_str());
  // report_failure()'s contract (BtActionNode has it, a StatefulActionNode
  // does not): registration name + message, last writer wins.
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kFailedNode, registrationName());
  config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, msg);  // NOLINT
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::WaitForMotionState>("WaitForMotionState");
}
