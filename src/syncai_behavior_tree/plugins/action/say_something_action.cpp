#include "syncai_behavior_tree/plugins/action/say_something_action.hpp"

#include <iostream>
#include <string>

namespace syncai_behavior_tree
{

BT::NodeStatus SaySomething::tick()
{
  auto msg = getInput<std::string>("message");
  if (!msg) {
    // Port not provided / wrong type (e.g. XML missing message="..."): fail this node.
    std::cout << "[SaySomething] missing port: " << msg.error() << "\n";
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "[SaySomething] " << msg.value() << "\n";
  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

// Plugin entry point: maps the XML tag "SaySomething" to the C++ class.
// BehaviorTreeEngine loads this .so and calls this macro via registerFromPlugin().
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::SaySomething>("SaySomething");
}
