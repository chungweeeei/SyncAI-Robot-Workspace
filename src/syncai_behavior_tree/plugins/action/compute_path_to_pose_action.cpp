#include "syncai_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

#include <memory>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  setOutput("path", result_.result->path);

  // Count plans for the navigator's number_of_replans feedback. Same channel
  // as increment_recovery_count(): a blackboard int, reset per goal by the
  // navigator, so the first plan of a goal reads as 0 replans there.
  int plans = 0;
  config().blackboard->get<int>(blackboard_keys::kNumberPlans, plans);      // NOLINT
  config().blackboard->set<int>(blackboard_keys::kNumberPlans, plans + 1);  // NOLINT
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  // Humble's ComputePathToPose result carries no reason (nav2 Iron's
  // error_code is not in this nav2_msgs), so the detail is the stage, not
  // the cause; the planner log has "failed to generate a valid path" against
  // the TF warning. The navigator turns this into PLAN_FAILED if the tree
  // ends on it.
  report_failure("planner " + action_name_ + " aborted the goal");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseAction::halt()
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
    return std::make_unique<syncai_behavior_tree::ComputePathToPoseAction>(
      name, "compute_path_to_pose", config);
  };

  factory.registerBuilder<syncai_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
