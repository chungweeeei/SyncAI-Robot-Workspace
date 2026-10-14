#include "syncai_behavior_tree/plugins/action/compute_staging_pose.hpp"

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
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

// A SyncActionNode has no report_failure(); the three-line equivalent, with
// the same semantics as BtActionNode's (registration name, last writer wins).
void reportFailure(BT::TreeNode & node, const std::string & msg)
{
  node.config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kFailedNode, node.registrationName());
  node.config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, msg);  // NOLINT
}

}  // namespace

ComputeStagingPose::ComputeStagingPose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ComputeStagingPose::tick()
{
  auto logger = rclcpp::get_logger("ComputeStagingPose");

  geometry_msgs::msg::PoseStamped dock;
  if (!getInput("dock_pose", dock)) {
    RCLCPP_ERROR(logger, "[ComputeStagingPose] no dock pose on the dock_pose port");
    reportFailure(*this, "no dock pose on the dock_pose port");
    return BT::NodeStatus::FAILURE;
  }

  double distance = 1.0;
  getInput("distance", distance);
  if (!(distance > 0.0)) {
    RCLCPP_ERROR(logger, "[ComputeStagingPose] distance must be > 0 (got %.3f)", distance);
    reportFailure(*this, "distance must be > 0 (got " + std::to_string(distance) + ")");
    return BT::NodeStatus::FAILURE;
  }

  const auto & q = dock.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 0.9 || norm > 1.1) {
    RCLCPP_ERROR(
      logger,
      "[ComputeStagingPose] dock pose has an invalid orientation (|q| = %.3f); a zero quaternion "
      "is the usual cause",
      norm);
    reportFailure(
      *this, "dock pose has an invalid orientation; a zero quaternion is the usual cause");
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = tf2::getYaw(q);
  geometry_msgs::msg::PoseStamped staging = dock;
  staging.pose.position.x -= distance * std::cos(yaw);
  staging.pose.position.y -= distance * std::sin(yaw);
  setOutput("staging_pose", staging);

  RCLCPP_INFO(
    logger,
    "[ComputeStagingPose] staging pose (%.2f, %.2f, yaw %.2f) from dock (%.2f, %.2f), %.2f m",
    staging.pose.position.x, staging.pose.position.y, yaw, dock.pose.position.x,
    dock.pose.position.y, distance);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::ComputeStagingPose>("ComputeStagingPose");
}
