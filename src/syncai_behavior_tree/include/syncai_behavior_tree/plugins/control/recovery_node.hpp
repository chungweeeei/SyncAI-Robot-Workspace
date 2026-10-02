#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace syncai_behavior_tree
{
class RecoveryNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::RecoveryNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for syncai_behavior_tree::RecoveryNode
   */
  ~RecoveryNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("number_of_retries", 1, "Number of retries")};
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_