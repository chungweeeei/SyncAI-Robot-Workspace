#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "syncai_behavior_tree/bt_action_node.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::ComputePathThroughPoses: one path from the robot through
 * every waypoint of `goals`, in order (patrol.xml, 2026-11). The twin of
 * ComputePathToPoseAction, down to the failure attribution, the plan count
 * and the status sentences, so the navigator reads a patrol's planning the
 * way it reads a move's.
 */
class ComputePathThroughPosesAction
: public BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
public:
  ComputePathThroughPosesAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path through every waypoint, in order"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Waypoints to plan through, the last one the destination"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "start", "Start pose of the path if overriding current robot pose"),
      BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
    });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
