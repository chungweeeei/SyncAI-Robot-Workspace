#include "syncai_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

#include <memory>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<syncai_common::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
}

void ComputePathToPoseAction::on_wait_for_result(
  std::shared_ptr<const syncai_common::action::ComputePathToPose::Feedback> /*feedback*/)
{
  // Why (2026-11): a preempt only swaps {goal} on the blackboard; the tree
  // keeps running, and the non-reactive Fallback resumes this node without
  // asking GlobalUpdatedGoal again. Without a re-send the planner finished
  // the superseded goal's plan, on_success() wrote that path as the new
  // goal's first, FollowPath turned onto it, and the 1 Hz check replanned
  // for the new goal only a second or more later -- a detour started and
  // abandoned, one replan counted too many, and (since PR #88 tags a landed
  // path with the current goal generation) the new goal reported reached if
  // the robot came to the old goal's end meanwhile. Re-sent, the planner
  // server takes the new goal as a preempt once the plan in hand is done
  // (it cannot stop Octo3D mid-search; a cancel would not either, and
  // halt() would block this tick on the answer), and the old goal's result
  // is dropped by its goal id (BtActionNode::send_new_goal()).
  if (takeChangedGoal()) {
    goal_updated_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[ComputePathToPoseAction][%s] Goal changed while planning; re-sending it to the planner",
      __func__);
  }
}

bool ComputePathToPoseAction::takeChangedGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  if (goal == goal_.goal) {
    return false;
  }
  goal_.goal = goal;
  return true;
}

bool ComputePathToPoseAction::resendIfGoalChanged()
{
  // The planner can answer before on_wait_for_result()'s re-send goes out:
  // BtActionNode re-sends only while the goal is still ACCEPTED / EXECUTING,
  // so a goal_updated_ left set means the result in hand is the old goal's.
  // Or {goal} changed in this very tick, after on_wait_for_result() ran.
  if (!goal_updated_ && !takeChangedGoal()) {
    return false;
  }
  goal_updated_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "[ComputePathToPoseAction][%s] The planner answered a superseded goal; planning the current "
    "one",
    __func__);
  send_new_goal();
  return true;
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  // Not the current goal's path: BtActionNode::tick() comes back here on the
  // next tick with the new goal's request out (FollowPathAction::on_success()
  // waits the same way).
  if (resendIfGoalChanged()) {
    return BT::NodeStatus::RUNNING;
  }
  setOutput("path", result_.result->path);

  // Count plans for the navigator's number_of_replans feedback. Same channel
  // as increment_recovery_count(): a blackboard int, reset per goal by the
  // navigator, so the first plan of a goal reads as 0 replans there.
  int plans = 0;
  config().blackboard->get<int>(blackboard_keys::kNumberPlans, plans);      // NOLINT
  config().blackboard->set<int>(blackboard_keys::kNumberPlans, plans + 1);  // NOLINT
  // A fresh path is what FollowPath is now driving, whatever started the plan
  // (the first one, a blocked path, a changed goal, the planner's retry).
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg, blackboard_keys::kStatusFollowing);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  // The superseded goal's failure is not the current goal's.
  if (resendIfGoalChanged()) {
    return BT::NodeStatus::RUNNING;
  }
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  // The planner's reason (2026-11, syncai_common's ComputePathToPose): its
  // error_code goes to the navigator, which reports it in place of
  // PLAN_FAILED if the tree ends on this, and its sentence into the detail.
  // A result without one (a goal aborted before it was planned) reads as the
  // plain 200. Until then Humble's nav2 result carried no reason and the
  // detail named the stage only.
  const auto & res = result_.result;
  report_failure(
    "planner " + action_name_ + " aborted the goal" +
      (res && !res->error_msg.empty() ? ": " + res->error_msg : std::string()),
    true, res ? res->error_code : 0);
  // Said as a retry because move.xml's RecoveryNode clears the global costmap
  // and plans once more; if that fails too the goal ends and the result's
  // error_msg takes over, so the sentence is never left claiming a retry.
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    "Planner could not find a path; clearing the global costmap and retrying");
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
