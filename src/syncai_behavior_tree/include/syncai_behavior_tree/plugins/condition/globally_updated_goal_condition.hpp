#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief SUCCESS on the first tick after the blackboard's "goal" or "goals"
 * changed, FAILURE otherwise. Port of nav2's GloballyUpdatedGoalCondition (XML
 * tag GlobalUpdatedGoal). "goals" came back on 2026-11 with the
 * navigate_through_poses navigator (patrol.xml), which writes the client's
 * waypoint list there; a tree reads whichever of the two its navigator
 * writes, the other being absent and equal to itself every tick. patrol.xml's
 * RemovePassedGoals works on a copy ("remaining_goals"), so passing a
 * waypoint is not read as a new goal and does not force a replan.
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
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
