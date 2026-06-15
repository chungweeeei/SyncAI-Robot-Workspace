#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "syncai_behavior_tree/bt_action_node.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps nav2_msgs::action::FollowPath
 */
class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::FollowPathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowPathAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   * @param feedback shared_ptr to latest feedback message
   */
  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
      });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
