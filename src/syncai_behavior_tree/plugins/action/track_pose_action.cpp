#include "syncai_behavior_tree/plugins/action/track_pose_action.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/blackboard_keys.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
// The fromMsg() overload tf2::getYaw() needs on a geometry_msgs quaternion,
// in this translation unit (compute_staging_pose.cpp has why).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_behavior_tree
{

namespace
{

using Track = syncai_common::action::TrackPose;

builtin_interfaces::msg::Duration toDuration(double seconds)
{
  return rclcpp::Duration::from_seconds(std::max(seconds, 0.0));
}

}  // namespace

TrackPoseAction::TrackPoseAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<syncai_common::action::TrackPose>(xml_tag_name, action_name, conf)
{
}

void TrackPoseAction::on_tick()
{
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);

  double x = 0.0;
  double y = 0.0;
  double yaw = M_PI;
  geometry_msgs::msg::PoseStamped target;
  fixed_target_ = getInput("target_pose", target) && !target.header.frame_id.empty();
  if (fixed_target_) {
    // Its frame followed, the pose itself the offset (header). A zero
    // quaternion reads as yaw 0 here; ComputeStagingPose, which writes the
    // pose charge.xml passes, refuses one before it gets this far.
    goal_.pose_topic.clear();
    goal_.tracked_frame = target.header.frame_id;
    x = target.pose.position.x;
    y = target.pose.position.y;
    yaw = tf2::getYaw(target.pose.orientation);
  } else {
    getInput("pose_topic", goal_.pose_topic);
    getInput("tracked_frame", goal_.tracked_frame);
    getInput("offset_x", x);
    getInput("offset_y", y);
    getInput("offset_yaw", yaw);
  }
  goal_.target_offset.position.x = x;
  goal_.target_offset.position.y = y;
  goal_.target_offset.position.z = 0.0;
  goal_.target_offset.orientation.x = 0.0;
  goal_.target_offset.orientation.y = 0.0;
  goal_.target_offset.orientation.z = std::sin(0.5 * yaw);
  goal_.target_offset.orientation.w = std::cos(0.5 * yaw);

  double filter_window = 1.0;
  double pose_timeout = 0.0;
  double lost_timeout = 0.0;
  double max_duration = 0.0;
  getInput("filter_window", filter_window);
  getInput("pose_timeout", pose_timeout);
  getInput("lost_timeout", lost_timeout);
  getInput("max_duration", max_duration);
  goal_.target_filter_window = static_cast<float>(std::max(filter_window, 0.0));
  goal_.pose_timeout = toDuration(pose_timeout);
  goal_.lost_timeout = toDuration(lost_timeout);
  goal_.max_duration = toDuration(max_duration);

  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    fixed_target_ ? "Backing off to the staging pose" : "Approaching the charger");
}

void TrackPoseAction::on_wait_for_result(std::shared_ptr<const Track::Feedback> feedback)
{
  // Two decimals, so the text changes a few times a second (DockToPose has
  // the reasoning). feedback is null on a tick that brought none.
  if (!feedback) {
    return;
  }
  char buf[96];
  if (fixed_target_) {
    // Nothing to wait for but the odometry and TF: no marker in these.
    std::snprintf(
      buf, sizeof(buf), "Backing off: %.2f m from the staging pose",
      static_cast<double>(feedback->distance_to_target));
    config().blackboard->set<std::string>(blackboard_keys::kNavStatusMsg, buf);  // NOLINT
    return;
  }
  switch (feedback->state) {
    case Track::Feedback::TRACKING:
      std::snprintf(
        buf, sizeof(buf), "Charging approach: %.2f m from the charger",
        static_cast<double>(feedback->distance_to_target));
      break;
    case Track::Feedback::HOLDING:
      std::snprintf(
        buf, sizeof(buf), "Charging approach: stopped, waiting for the charger's marker");
      break;
    default:
      std::snprintf(buf, sizeof(buf), "Charging approach: waiting for the charger's marker");
      break;
  }
  config().blackboard->set<std::string>(blackboard_keys::kNavStatusMsg, buf);  // NOLINT
}

BT::NodeStatus TrackPoseAction::on_success()
{
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg, fixed_target_ ? "At the staging pose" : "On the charger");
  RCLCPP_INFO(
    node_->get_logger(),
    "[TrackPoseAction][%s] on the target: final error %.3f m (lateral %+.3f m) / %+.3f rad",
    __func__, static_cast<double>(result_.result->final_distance_error),
    static_cast<double>(result_.result->final_lateral_error),
    static_cast<double>(result_.result->final_yaw_error));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackPoseAction::on_aborted()
{
  // ABORTED + NONE is the server's "superseded or not started", as for
  // DockToPose; everything else is the server's own words.
  std::string detail;
  if (result_.result->error_code == Track::Result::NONE) {
    detail = "controller " + action_name_ +
             " aborted the goal before it started (no error code: the goal was superseded)";
  } else {
    char residuals[128] = "";
    if (result_.result->final_distance_error >= 0.0f) {
      std::snprintf(
        residuals, sizeof(residuals),
        "; last seen %.3f m from the target, lateral %+.3f m, %+.3f rad",
        static_cast<double>(result_.result->final_distance_error),
        static_cast<double>(result_.result->final_lateral_error),
        static_cast<double>(result_.result->final_yaw_error));
    }
    detail = "controller " + action_name_ + " aborted the goal: " + result_.result->error_msg +
             " (error_code " + std::to_string(result_.result->error_code) + ")" + residuals;
  }
  // Logged as well as written: the tree's restore branch may overwrite the
  // blackboard keys afterwards (DockToPose::on_aborted has the case).
  RCLCPP_ERROR(node_->get_logger(), "[TrackPoseAction][%s] %s", __func__, detail.c_str());
  report_failure(detail, true, result_.result->error_code);
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    fixed_target_ ? "Backing off to the staging pose failed" : "Charging approach failed");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrackPoseAction::on_cancelled()
{
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::TrackPoseAction>(name, "track_pose", config);
  };

  factory.registerBuilder<syncai_behavior_tree::TrackPoseAction>("TrackPose", builder);
}
