#include "syncai_behavior_tree/plugins/action/wait_action.hpp"

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

WaitAction::WaitAction(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
}

BT::NodeStatus WaitAction::onStart()
{
  int msec = 0;
  if (!getInput("msec", msec) || msec < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Wait"), "[Wait] the msec port must be set and >= 0");
    config().blackboard->set<std::string>(blackboard_keys::kFailedNode, registrationName());
    config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, "Wait: msec unset or < 0");
    return BT::NodeStatus::FAILURE;
  }
  if (msec == 0) {
    return BT::NodeStatus::SUCCESS;
  }
  deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(msec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAction::onRunning()
{
  return std::chrono::steady_clock::now() >= deadline_ ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::RUNNING;
}

void WaitAction::onHalted()
{
  // Nothing to undo: the next onStart() takes a fresh deadline.
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::WaitAction>("Wait");
}
