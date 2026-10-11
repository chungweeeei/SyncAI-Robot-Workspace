#include "syncai_behavior_tree/plugins/condition/globally_updated_goal_condition.hpp"

#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

GloballyUpdatedGoalCondition::GloballyUpdatedGoalCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus GloballyUpdatedGoalCondition::tick()
{
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // The first tick only records the baseline. The instance outlives a goal
  // (the tree is reused unless always_reload_bt_xml), so on every later goal
  // the stored pose is the previous goal's and the first tick reports the
  // change, which is harmless: the navigator also clears {path} per goal.
  if (first_time_) {
    first_time_ = false;
    goal_ = current_goal;
    return BT::NodeStatus::FAILURE;
  }

  if (goal_ != current_goal) {
    goal_ = current_goal;
    // Said only once this goal has had a path: the first tick of every new
    // goal lands here too (see above), and there the navigator's "Planning"
    // sentence is the true one -- nothing is being re-planned yet.
    int plans = 0;
    config().blackboard->get<int>(blackboard_keys::kNumberPlans, plans);  // NOLINT
    if (plans > 0) {
      config().blackboard->set<std::string>(  // NOLINT
        blackboard_keys::kNavStatusMsg, "Goal changed; replanning");
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::GloballyUpdatedGoalCondition>("GlobalUpdatedGoal");
}
