#include "syncai_behavior_tree/plugins/action/dock_to_pose_action.hpp"

#include <cstdio>
#include <memory>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
// tf2::getYaw() on a geometry_msgs quaternion needs the fromMsg() overload
// this header provides in the same translation unit: without it a Release
// build links and then dies with a symbol lookup error when the plugin loads
// (syncai_controller's simple_progress_checker.cpp records the incident).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_behavior_tree
{

namespace
{

using DockGoal = syncai_common::action::DockToPose::Goal;

// The trees carry poses as PoseStamped ({goal}, {staging_pose}); the action
// takes a planar frame_id / x / y / yaw since 2026-10-14 (a quaternion is
// what a hand-written CLI goal gets wrong, and the controller uses nothing
// else). The conversion lives here, the one client, so the port stays a
// PoseStamped and neither tree changes. The quaternion was validated by
// ComputeStagingPose earlier in both trees. Returns whether anything changed.
bool fillPlanarPose(const geometry_msgs::msg::PoseStamped & pose, DockGoal & goal)
{
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const bool changed = goal.frame_id != pose.header.frame_id ||
                       goal.x != pose.pose.position.x || goal.y != pose.pose.position.y ||
                       goal.yaw != yaw;
  goal.frame_id = pose.header.frame_id;
  goal.x = pose.pose.position.x;
  goal.y = pose.pose.position.y;
  goal.yaw = yaw;
  return changed;
}

}  // namespace

DockToPoseAction::DockToPoseAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<syncai_common::action::DockToPose>(xml_tag_name, action_name, conf)
{
}

void DockToPoseAction::on_tick()
{
  geometry_msgs::msg::PoseStamped dock_pose;
  getInput("dock_pose", dock_pose);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  double timeout = 0.0;
  getInput("timeout", timeout);
  goal_.timeout = static_cast<float>(timeout);

  // The server would read an empty frame_id as "map" and dock somewhere
  // plausible-looking; failing here names the real cause (a pose built
  // without a header, usually) before anything is sent. tick() returns
  // FAILURE when should_send_goal_ is false.
  if (dock_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[DockToPoseAction][%s] dock_pose has no frame_id; not sending the goal",
      __func__);
    report_failure("dock_pose has no frame_id");
    should_send_goal_ = false;
    return;
  }
  fillPlanarPose(dock_pose, goal_);

  // The one hint this node has about direction. It only picks sentences;
  // the goal goes out as the ports say.
  is_undock_ = goal_.controller_id.find("Undock") != std::string::npos;
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    is_undock_ ? "Backing out of the dock" : "Approaching the dock");
}

void DockToPoseAction::on_wait_for_result(
  std::shared_ptr<const syncai_common::action::DockToPose::Feedback> feedback)
{
  // One short string set per tick at most: a mutex and a copy, cheaper than
  // the feedback publish the navigator does in the same tick. Two decimals,
  // so the text changes a few times a second and the console does not
  // flicker. feedback is null on a tick that brought none.
  if (feedback) {
    char buf[64];
    if (is_undock_) {
      std::snprintf(
        buf, sizeof(buf), "Undocking: %.2f m to the staging pose",
        static_cast<double>(feedback->distance_to_dock));
    } else {
      std::snprintf(
        buf, sizeof(buf), "Docking: %.2f m from the dock",
        static_cast<double>(feedback->distance_to_dock));
    }
    config().blackboard->set<std::string>(blackboard_keys::kNavStatusMsg, buf);  // NOLINT
  }

  // Re-send when a port changed, the FollowPath pattern. This is what makes a
  // same-tree preempt that moves the dock pose re-target the controller
  // mid-approach: the navigator swaps {goal}, this node sees the new pose and
  // sets goal_updated_, BtActionNode sends it, and the server's dock loop
  // accepts the pending goal in place (controller_server polls
  // is_preempt_requested() every cycle, as it does for FollowPath).
  geometry_msgs::msg::PoseStamped new_pose;
  getInput("dock_pose", new_pose);
  if (!new_pose.header.frame_id.empty() && fillPlanarPose(new_pose, goal_)) {
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);
  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);
  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

BT::NodeStatus DockToPoseAction::on_success()
{
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg, is_undock_ ? "Undocked" : "Docked");
  RCLCPP_INFO(
    node_->get_logger(), "[DockToPoseAction][%s] %s: final error %.3f m / %.3f rad", __func__,
    is_undock_ ? "undocked" : "docked", static_cast<double>(result_.result->final_distance_error),
    static_cast<double>(result_.result->final_yaw_error));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockToPoseAction::on_aborted()
{
  // Unlike FollowPath's nav2 result, DockToPose carries a reason. ABORTED +
  // NONE is the server's "superseded or not started" (another goal preempted
  // this one, or follow_path held the controller) and says nothing about the
  // approach itself; everything else is the controller's own words.
  std::string detail;
  if (result_.result->error_code == syncai_common::action::DockToPose::Result::NONE) {
    detail = "controller " + action_name_ +
             " aborted the goal before it started (no error code: the controller was busy or "
             "the goal was superseded)";
  } else {
    detail = "controller " + action_name_ + " aborted the goal: " + result_.result->error_msg +
             " (error_code " + std::to_string(result_.result->error_code) + ")";
  }
  // Logged as well as written: the tree's restore branch (SetMotionKey key=1)
  // may overwrite the blackboard keys afterwards if the driver refuses it,
  // and the log must keep why the approach failed.
  RCLCPP_ERROR(node_->get_logger(), "[DockToPoseAction][%s] %s", __func__, detail.c_str());
  report_failure(detail);
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg, is_undock_ ? "Undock back-out failed" : "Dock approach failed");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockToPoseAction::on_cancelled()
{
  // No report_failure(): on a halt the keys are irrelevant (the goal is not
  // FAILED), and attributing an external cancel to docking would be a guess.
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::DockToPoseAction>(name, "dock_to_pose", config);
  };

  factory.registerBuilder<syncai_behavior_tree::DockToPoseAction>("DockToPose", builder);
}
