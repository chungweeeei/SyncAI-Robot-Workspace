#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_

#include <string>
#include <vector>

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "syncai_behavior_tree/bt_action_node.hpp"

namespace syncai_behavior_tree
{
class ComputePathThroughPosesAction
: public BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
public:
  ComputePathThroughPosesAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathThroughPoses node"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Destinations to plan through"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "start", "Start pose of the path if overriding current robot pose"),
      BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
    });
  }
};
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
