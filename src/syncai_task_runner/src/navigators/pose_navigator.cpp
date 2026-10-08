#include "syncai_task_runner/navigators/pose_navigator.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "syncai_behavior_tree/blackboard_keys.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/robot_utils.hpp"

namespace syncai_task_runner
{

namespace
{

namespace keys = syncai_behavior_tree::blackboard_keys;
using GoalResult = syncai_common::action::NavigateToGoal::Result;

struct FailureReason
{
  uint16_t code;
  std::string msg;
};

// failed_node -> (error_code, operator sentence). The number lives here and
// not in the BT package because it belongs to the action this navigator
// serves (blackboard_keys.hpp has the argument); the BT nodes only say which
// of them failed. A node absent from this table maps to UNKNOWN with its own
// message, which is safe but uninformative -- add a row when a new node
// starts calling report_failure(). The sentences are what the console shows
// the operator; the node's detail follows in parentheses.
const std::map<std::string, std::pair<uint16_t, const char *>> kStageByFailedNode = {
  {"ComputePathToPose", {GoalResult::PLAN_FAILED, "Planning failed: no path to the goal"}},
  {"FollowPath",
   {GoalResult::FOLLOW_PATH_FAILED,
    "Path following failed: the robot could not reach the goal along its path"}},
};

FailureReason readFailureReason(const BT::Blackboard::Ptr & blackboard)
{
  std::string failed_node;
  std::string failure_msg;
  blackboard->get<std::string>(keys::kFailedNode, failed_node);
  blackboard->get<std::string>(keys::kFailureMsg, failure_msg);

  const auto stage = kStageByFailedNode.find(failed_node);
  if (stage != kStageByFailedNode.end()) {
    return {stage->second.first, std::string(stage->second.second) + " (" + failure_msg + ")"};
  }
  if (failed_node.empty() && !failure_msg.empty()) {
    // Unattributed by the node itself (a server never acknowledged the goal,
    // or rejected it): the message is the whole story, and blaming planning
    // or driving would be a guess.
    return {GoalResult::UNKNOWN, failure_msg};
  }
  if (failed_node.empty()) {
    // Nothing wrote: an exception inside the tree, which BehaviorTreeEngine
    // logs and swallows, or a node that fails without reporting.
    return {
      GoalResult::UNKNOWN,
      "Navigation failed without a node reporting a reason (see the task_runner log)"};
  }
  return {GoalResult::UNKNOWN, failed_node + ": " + failure_msg};
}

}  // namespace

template <class ActionT>
bool PoseNavigator<ActionT>::configure(
  rclcpp::Node::WeakPtr parent_node, std::shared_ptr<syncai_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  // Node-level parameters, shared by both instantiations: the second one to
  // configure finds them declared and reads the same values, which is the
  // point -- same tree, same blackboard keys.
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  odom_smoother_ = odom_smoother;

  if constexpr (Traits::kDrivesRvizGoalPose) {
    // register action client
    self_client_ = rclcpp_action::create_client<ActionT>(node, this->getName());

    // register subscription  to receiving goal pose from rviz
    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  }

  return true;
}

template <class ActionT>
std::string PoseNavigator<ActionT>::getDefaultBTFilepath(rclcpp::Node::WeakPtr parent_node)
{
  auto node = parent_node.lock();

  std::string default_bt_xml_filename;
  if (!node->has_parameter("default_bt_xml")) {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("syncai_task_runner");
    node->declare_parameter<std::string>(
      "default_bt_xml", pkg_share_dir + "/behavior_trees/move.xml");
  }

  node->get_parameter("default_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

template <class ActionT>
bool PoseNavigator<ActionT>::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

template <class ActionT>
bool PoseNavigator<ActionT>::goalReceived(typename ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!this->bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      this->logger_, "[PoseNavigator][%s] %s: BT file not found: %s. Navigation canceled.",
      __func__, this->getName().c_str(), bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

template <class ActionT>
void PoseNavigator<ActionT>::goalCompleted(
  typename ActionT::Result::SharedPtr result, const syncai_behavior_tree::BtStatus final_bt_status)
{
  // SUCCEEDED and CANCELED carry NONE: a default-constructed result already
  // says so. A superseded goal never reaches here at all (the action server
  // aborts it with an empty result from accept_pending_goal()), nor does one
  // rejected in goalReceived(); both read as ABORTED + NONE on the client,
  // which the .action file documents as "superseded or not started".
  if (final_bt_status != syncai_behavior_tree::BtStatus::FAILED) {
    return;
  }

  const FailureReason reason = readFailureReason(this->bt_action_server_->getBlackboard());

  // Logged for both actions: navigate_to_pose's clients get the reason in the
  // task_runner log before they migrate, and the log is the only place the
  // nav2 result can carry it.
  RCLCPP_ERROR(
    this->logger_, "[PoseNavigator][%s] %s: navigation failed (error_code %u): %s", __func__,
    this->getName().c_str(), static_cast<unsigned>(reason.code), reason.msg.c_str());

  if constexpr (Traits::kReportsFailure) {
    result->error_code = reason.code;
    result->error_msg = reason.msg;
  } else {
    (void)result;
  }
}

template <class ActionT>
void PoseNavigator<ActionT>::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<typename ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  syncai_util::getCurrentPose(
    current_pose, *this->feedback_utils_.tf, this->feedback_utils_.global_frame,
    this->feedback_utils_.robot_frame, this->feedback_utils_.transform_tolerance);

  // Named type, not auto: initialised from a dependent expression the variable
  // would be dependent too, and every get<T>() below would need `template`.
  BT::Blackboard::Ptr blackboard = this->bt_action_server_->getBlackboard();
  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx = [&current_pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = syncai_util::geometry_utils::euclidean_distance(
          current_pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

    double distance_remaining =
      syncai_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = this->clock_->now() - start_time_;

  if constexpr (Traits::kReportsFailure) {
    // Plans beyond the first = detours, since move.xml plans only when the
    // path is blocked or the goal changed.
    int plans = 0;
    blackboard->get<int>(keys::kNumberPlans, plans);
    feedback_msg->number_of_replans = static_cast<uint16_t>(std::max(plans - 1, 0));
  }

  this->bt_action_server_->publishFeedback(feedback_msg);
}

template <class ActionT>
void PoseNavigator<ActionT>::onPreempt(typename ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    this->logger_, "[PoseNavigator][%s] %s: received goal preemption request", __func__,
    this->getName().c_str());

  if (
    goal->behavior_tree == this->bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() && this->bt_action_server_->getCurrentBTFilename() ==
                                      this->bt_action_server_->getDefaultBTFilename())) {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(this->bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      this->logger_,
      "[PoseNavigator][%s] Preemption request was rejected since the requested BT XML "
      "file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.",
      __func__);
    this->bt_action_server_->terminatePendingGoal();
  }
}

template <class ActionT>
void PoseNavigator<ActionT>::initializeGoalPose(typename ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  syncai_util::getCurrentPose(
    current_pose, *this->feedback_utils_.tf, this->feedback_utils_.global_frame,
    this->feedback_utils_.robot_frame, this->feedback_utils_.transform_tolerance);

  RCLCPP_INFO(
    this->logger_, "Begin navigating from current location (%.2f, %.2f) to (%.2f, %.2f)",
    current_pose.pose.position.x, current_pose.pose.position.y, goal->pose.pose.position.x,
    goal->pose.pose.position.y);

  // Reset state for new action feedback. A preempt comes through here too, so
  // the superseding goal starts with a clean attribution and its first plan
  // is not counted as a replan.
  start_time_ = this->clock_->now();
  BT::Blackboard::Ptr blackboard = this->bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT
  blackboard->set<std::string>(keys::kFailedNode, "");
  blackboard->set<std::string>(keys::kFailureMsg, "");
  blackboard->set<int>(keys::kNumberPlans, 0);

  // Update the goal pose on the blackboard
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);

  // Drop the previous goal's path. move.xml keeps {path} for as long as
  // IsPathValid accepts it, and nothing else clears it between goals
  // (haltAllActions() only halts nodes still RUNNING, so ComputePathToPose's
  // halt() rarely runs). A stale path that is still free would otherwise be
  // followed from wherever it starts when the same goal is sent again. On a
  // preempt this is harmless: FollowPath ignores an empty path and keeps the
  // one it has until the replan lands.
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
}

template <class ActionT>
void PoseNavigator<ActionT>::onGoalPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // Instantiated for both actions, subscribed for one (Traits::kDrivesRvizGoalPose).
  if (!self_client_) {
    return;
  }

  typename ActionT::Goal goal;
  goal.pose = *pose;

  // subscribe the goal pose topic and send the goal to the action server
  self_client_->async_send_goal(goal);
}

template class PoseNavigator<nav2_msgs::action::NavigateToPose>;
template class PoseNavigator<syncai_common::action::NavigateToGoal>;

}  // namespace syncai_task_runner
