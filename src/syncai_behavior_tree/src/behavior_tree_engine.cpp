#include "syncai_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

namespace syncai_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus BehaviorTreeEngine::run(
  BT::Tree * tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }

      // tick signal from rootNode down to the entire tree
      // With a <Sequence> as rootNode this triggers SequenceNode::tick().
      // SequenceNode::tick() ticks each child in order => it only advances to the next child
      // once the current one returns NodeStatus::SUCCESS.
      // After a NodeStatus::SUCCESS the next child is ticked immediately, without waiting for
      // the next loopTimeout.
      // The three return values map to three paths:
      // 1. NodeStatus::SUCCESS => tick the next child right away
      // 2. NodeStatus::RUNNING => the next engine tick ticks the same child again
      // 3. NodeStatus::FAILURE => halt all children.
      result = tree->tickRoot();

      onLoop();

      if (!loopRate.sleep()) {
        RCLCPP_WARN(
          rclcpp::get_logger("BehaviorTreeEngine"), "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path, BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  // this halt signal should propagate through the entire tree.
  root_node->halt();

  // but, just in case...
  auto visitor = [](BT::TreeNode * node) {
    if (node->status() == BT::NodeStatus::RUNNING) {
      node->halt();
    }
  };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace syncai_behavior_tree
