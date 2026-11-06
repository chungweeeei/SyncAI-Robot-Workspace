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
: BtActionNode<syncai_common::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::on_tick()
{
  getInput("goals", goal_.goals);
  getInput("planner_id", goal_.planner_id);
}

void ComputePathThroughPosesAction::on_wait_for_result(
  std::shared_ptr<const syncai_common::action::ComputePathThroughPoses::Feedback> /*feedback*/)
{
  // ComputePathToPoseAction::on_wait_for_result() has why. {goals} here is
  // the navigator's remaining_goals, which also changes when the robot
  // passes a waypoint mid-plan: that re-sends too, and the route is planned
  // from the next waypoint, as the next replan would have.
  if (takeChangedGoals()) {
    goal_updated_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[ComputePathThroughPosesAction][%s] Waypoints changed while planning; re-sending them to "
      "the planner",
      __func__);
  }
}

bool ComputePathThroughPosesAction::takeChangedGoals()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  getInput("goals", goals);
  if (goals == goal_.goals) {
    return false;
  }
  goal_.goals = goals;
  return true;
}

bool ComputePathThroughPosesAction::resendIfGoalsChanged()
{
  // ComputePathToPoseAction::resendIfGoalChanged() has why.
  if (!goal_updated_ && !takeChangedGoals()) {
    return false;
  }
  goal_updated_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "[ComputePathThroughPosesAction][%s] The planner answered superseded waypoints; planning the "
    "current ones",
    __func__);
  send_new_goal();
  return true;
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  if (resendIfGoalsChanged()) {
    return BT::NodeStatus::RUNNING;
  }
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
  // The superseded waypoints' failure is not the current route's.
  if (resendIfGoalsChanged()) {
    return BT::NodeStatus::RUNNING;
  }
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  // The planner's reason, as in ComputePathToPoseAction::on_aborted(); its
  // sentence names the leg and the waypoint that failed.
  const auto & res = result_.result;
  report_failure(
    "planner " + action_name_ + " aborted the route through " + std::to_string(goal_.goals.size()) +
      " waypoints" + (res && !res->error_msg.empty() ? ": " + res->error_msg : std::string()),
    true, res ? res->error_code : 0);
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
