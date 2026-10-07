#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief SUCCESS on the first tick after the blackboard's "goal" changed, FAILURE
 * otherwise. Port of nav2's GloballyUpdatedGoalCondition (XML tag GlobalUpdatedGoal),
 * reduced to the single "goal" key: there is no NavigateThroughPoses navigator here,
 * so upstream's "goals" vector has no writer.
 *
 * move.xml needs it because IsPathValid alone cannot see a preempt: the navigator
 * swaps "goal" under a running tree, and the path to the old goal is still free.
 */
class GloballyUpdatedGoalCondition : public BT::ConditionNode
{
public:
  GloballyUpdatedGoalCondition(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  GloballyUpdatedGoalCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  bool first_time_{true};
  geometry_msgs::msg::PoseStamped goal_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
