#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "syncai_behavior_tree/bt_action_node.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps nav2_msgs::action::NavigateToPose
 */
class NavigateToPoseAction : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::NavigateToPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  NavigateToPoseAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),
      });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
