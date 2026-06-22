#include "syncai_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

#include <memory>
#include <string>
#include <vector>

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
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
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