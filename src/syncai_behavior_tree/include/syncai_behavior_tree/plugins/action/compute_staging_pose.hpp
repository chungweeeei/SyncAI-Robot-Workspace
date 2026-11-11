#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_STAGING_POSE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_STAGING_POSE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief Derive the docking staging pose from the dock pose: the dock pose
 * translated straight back along its own yaw by `distance`, same heading,
 * same frame and stamp. Pure math, no ROS, so it is a BT::SyncActionNode
 * rather than one of the two ROS bases.
 *
 * Where the robot goes before the blind last metre (dock.xml's staging leg,
 * with the normal costmap-aware navigation), and where it backs out to
 * (undock.xml). Both trees derive it here from the one pose the backend
 * sends -- the DOCK pose -- so the backend never has to know the staging
 * distance.
 *
 * With facing_landmark (charge.xml, 2026-11) the pose is not where the robot
 * ends but a LANDMARK it must look at -- a charger's tag, stored with its yaw
 * pointing out of its face -- and the staging pose is `distance` out along
 * that yaw, turned round to face it: where the camera sees the tag square on.
 *
 * Fails, with the reason on the blackboard, on a missing pose, a
 * non-positive distance, or a quaternion that is not a rotation (a goal sent
 * with `orientation: {}` would otherwise yield yaw 0 silently and the robot
 * would dock facing the wrong way).
 */
class ComputeStagingPose : public BT::SyncActionNode
{
public:
  ComputeStagingPose(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "dock_pose", "Dock pose; the staging pose is computed in its frame"),
      BT::InputPort<double>(
        "distance", 1.0, "Metres straight back from the dock along the dock's yaw"),
      BT::InputPort<bool>(
        "facing_landmark", false,
        "dock_pose is a landmark whose yaw points out of its face (a charger's tag): stage "
        "`distance` out along that yaw, turned to face it"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "staging_pose", "dock_pose translated back by distance, same yaw, frame and stamp"),
    };
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_STAGING_POSE_HPP_
