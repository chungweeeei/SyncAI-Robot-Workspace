#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.h"
#include "syncai_behavior_tree/bt_action_node.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::ComputePathToPose
 */
class ComputePathToPoseAction : public BtActionNode<nav2_msgs::action::ComputePathToPose>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::ComputePathToPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputePathToPoseAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>(
          "planner_id", "", "Mapped name to the planner plugin type to use"),
      });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
