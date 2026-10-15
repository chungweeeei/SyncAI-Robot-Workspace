#include "syncai_behavior_tree/plugins/action/compute_back_out_pose.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

namespace
{

// A SyncActionNode has no report_failure(); the same three lines as
// ComputeStagingPose's (registration name, last writer wins).
void reportFailure(BT::TreeNode & node, const std::string & msg)
{
  node.config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kFailedNode, node.registrationName());
  node.config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, msg);  // NOLINT
}

}  // namespace

ComputeBackOutPose::ComputeBackOutPose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ComputeBackOutPose::tick()
{
  auto logger = rclcpp::get_logger("ComputeBackOutPose");

  double distance = 1.0;
  getInput("distance", distance);
  if (!(distance > 0.0)) {
    RCLCPP_ERROR(logger, "[ComputeBackOutPose] distance must be > 0 (got %.3f)", distance);
    reportFailure(*this, "distance must be > 0 (got " + std::to_string(distance) + ")");
    return BT::NodeStatus::FAILURE;
  }

  // Read per tick rather than cached in the constructor: the tree is built
  // per goal anyway, and this keeps the node free of state.
  std::string base_frame;
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("base_frame", base_frame);
  if (base_frame.empty()) {
    RCLCPP_ERROR(logger, "[ComputeBackOutPose] the BT client node has no base_frame");
    reportFailure(*this, "the robot's base frame is unknown (empty base_frame parameter)");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame;
  // Stamp left at 0: DockToPose sends frame_id / x / y / yaw only, and the
  // server looks the transform up at Time(0), the latest.
  pose.pose.position.x = -distance;
  pose.pose.orientation.w = 1.0;
  setOutput("back_out_pose", pose);

  RCLCPP_INFO(
    logger, "[ComputeBackOutPose] back-out pose %.2f m behind the robot, in %s", distance,
    base_frame.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::ComputeBackOutPose>("ComputeBackOutPose");
}
