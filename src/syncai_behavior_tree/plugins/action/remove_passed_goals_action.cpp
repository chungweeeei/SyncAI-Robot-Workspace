#include "syncai_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "syncai_util/geometry_utils.hpp"

namespace syncai_behavior_tree
{
RemovePassedGoalsAction::RemovePassedGoalsAction(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), viapoint_achieved_radius_(0.5)
{
  getInput<double>("radius", viapoint_achieved_radius_);

  getInput<std::string>("global_frame", global_frame_);
  getInput<std::string>("robot_base_frame", robot_frame_);

  auto node = config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoalsAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  // Fetch the TF buffer lazily: the navigator sets "tf_buffer" on the blackboard
  // only after the BT (and this node) has been constructed, so reading it in the
  // constructor would fail with "Missing key [tf_buffer]". By tick() time the
  // goal has arrived and the key is guaranteed to be present.
  if (!tf_) {
    tf_ = config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace syncai_util::geometry_utils;
  geometry_msgs::msg::PoseStamped current_pose;

  if (!syncai_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_frame_, transform_tolerance_)) {
    return BT::NodeStatus::FAILURE;
  }

  // calculate distance
  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    goal_poses.erase(goal_poses.begin());
  }

  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::RemovePassedGoalsAction>("RemovePassedGoals");
}
