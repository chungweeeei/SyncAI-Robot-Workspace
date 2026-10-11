#include "syncai_behavior_tree/plugins/action/follow_path_action.hpp"

#include <memory>
#include <string>

namespace syncai_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);

  // PipelineSequence reaches this node only after the planner branch returned
  // SUCCESS, which with a real plan is a non-empty path (planner_server
  // rejects an empty plan as ABORTED). The one SUCCESS that carries no path is
  // ComputePathToPose::on_cancelled -- its goal cancelled under it, which only
  // an external cancel on compute_path_to_pose does. Sent on, the controller
  // throws "Invalid path, Path is empty." at goal start, which reads as a
  // controller fault, and the RecoveryNode's retry is spent on a path that
  // cannot change until the planner branch runs again. Failing here (tick()
  // returns FAILURE when should_send_goal_ is false) names the real cause and
  // costs nothing: the retry would meet the same empty path. Waiting as
  // RUNNING until a path exists is not something BtActionNode offers -- its
  // tick() has no state between "goal sent" and "failed".
  if (goal_.path.poses.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[FollowPathAction][%s] No path to follow (empty path); not sending the goal", __func__);
    should_send_goal_ = false;
  }
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> /*feedback*/)
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Re-send when the path changed, unless there is nothing to follow. Upstream
  // screens only the all-default message (`new_path != nav_msgs::msg::Path()`),
  // which is what ComputePathToPose writes on abort / cancel / halt and what
  // the navigator writes at every new goal, so the two tests coincide today
  // -- but the property that matters is "no poses", and a stamped-but-empty
  // path would have been sent on and thrown in the controller mid-drive,
  // taking the retry with it.
  if (goal_.path != new_path && !new_path.poses.empty()) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::FollowPathAction>(name, "follow_path", config);
  };

  factory.registerBuilder<syncai_behavior_tree::FollowPathAction>("FollowPath", builder);
}
