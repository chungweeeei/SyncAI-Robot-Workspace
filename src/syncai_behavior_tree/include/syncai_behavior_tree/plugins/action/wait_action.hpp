#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace syncai_behavior_tree
{

/**
 * @brief Return RUNNING for `msec` milliseconds, then SUCCESS. No ROS, no
 * thread: a BT::StatefulActionNode that checks a steady_clock deadline on
 * each tick and hands control straight back to the engine's loop.
 *
 * Why not BT.CPP's built-in Delay: that one is a decorator that postpones
 * its child's first tick, so every wait in a tree has to wrap the step that
 * follows it, which reads badly once a tree has two. And the docking trees'
 * waits are placeholders: they stand in for a check that the gait controller
 * has reached the mode it was just sent (its reported code for MPC is not
 * known yet, see dock.xml's header), and when that check exists it replaces
 * this node in the same position -- a decorator around the next step would
 * not.
 */
class WaitAction : public BT::StatefulActionNode
{
public:
  WaitAction(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("msec", "How long to wait, in milliseconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
