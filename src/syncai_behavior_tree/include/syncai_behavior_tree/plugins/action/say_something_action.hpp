#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SAY_SOMETHING_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SAY_SOMETHING_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace syncai_behavior_tree
{

/**
 * @class syncai_behavior_tree::SaySomething
 * @brief A trivial synchronous BT action that prints a message and returns
 * SUCCESS in a single tick. Serves as the minimal example of a custom BT node
 * registered as a plugin and loaded by the task runner.
 */
class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  /**
   * @brief Declare the ports this node exposes to the XML.
   * @return BT::PortsList One input port "message" (literal text or a {blackboard_key}).
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message", "text to say")};
  }

  /**
   * @brief Read the "message" port and print it. SyncActionNode must return
   * SUCCESS/FAILURE in this single tick (never RUNNING).
   */
  BT::NodeStatus tick() override;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SAY_SOMETHING_ACTION_HPP_
