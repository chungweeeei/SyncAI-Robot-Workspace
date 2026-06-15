#include <memory>
#include <string>

#include "syncai_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace syncai_behavior_tree
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseAction: goal not provided");
    return;
  }
  getInput("behavior_tree", goal_.behavior_tree);
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::NavigateToPoseAction>(
      name, "navigate_to_pose", config);
  };

  factory.registerBuilder<syncai_behavior_tree::NavigateToPoseAction>("NavigateToPose", builder);
}
