#include "syncai_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

#include <memory>
#include <string>
#include <vector>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::on_tick()
{
  getInput("goals", goal_.goals);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  // The navigator's number_of_replans and status sentence, as in
  // ComputePathToPoseAction::on_success().
  int plans = 0;
  config().blackboard->get<int>(blackboard_keys::kNumberPlans, plans);      // NOLINT
  config().blackboard->set<int>(blackboard_keys::kNumberPlans, plans + 1);  // NOLINT
  config().blackboard->set<std::string>(                                    // NOLINT
    blackboard_keys::kNavStatusMsg, blackboard_keys::kStatusFollowing);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  // No reason in the result (Humble); the planner log names the leg and the
  // waypoint that failed. The navigator maps this node to PLAN_FAILED.
  report_failure(
    "planner " + action_name_ + " aborted the route through " + std::to_string(goal_.goals.size()) +
    " waypoints");
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    "Planner could not find a route through the waypoints; clearing obstacle memory and retrying");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputePathThroughPosesAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::ComputePathThroughPosesAction>(
      name, "compute_path_through_poses", config);
  };

  factory.registerBuilder<syncai_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
