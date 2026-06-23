#include "syncai_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

namespace syncai_behavior_tree
{

BT::NodeStatus initialPoseReceived(BT::TreeNode & tree_node)
{
  auto initPoseReceived = tree_node.config().blackboard->get<bool>("initial_pose_received");
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerSimpleCondition(
    "InitialPoseReceived",
    std::bind(&syncai_behavior_tree::initialPoseReceived, std::placeholders::_1));
}
