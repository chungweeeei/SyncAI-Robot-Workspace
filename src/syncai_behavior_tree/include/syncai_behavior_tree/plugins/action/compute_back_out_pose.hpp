#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_BACK_OUT_POSE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_BACK_OUT_POSE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief The pose `distance` metres straight behind the robot, same heading,
 * expressed in the robot's own base frame: (-distance, 0), identity
 * orientation, stamp 0. undock.xml's back-out target (2026-10-15).
 *
 * No TF lookup here, on purpose. DockToPose's server transforms its goal
 * into odom once, at acceptance, with the latest transform -- so a pose in
 * base_link becomes "1 m behind where the robot stands when the back-out
 * starts", which is exactly the target. Computing it here in map instead
 * would bake in whatever the localizer believed a tick earlier and add a
 * failure mode (no map -> base_link) the server already reports.
 *
 * Why this replaced ComputeStagingPose in undock.xml: the back-out used to
 * be derived from the dock pose the goal carried, so an undock needed the
 * backend to know which dock the robot was lying on. The robot already is
 * on the dock -- its own pose is the dock pose to within the dock goal
 * checker's 5 cm / 0.05 rad -- so the goal's pose is now ignored and an
 * undock needs none.
 *
 * The frame is the `base_frame` parameter of the BT client node (copied from
 * task_runner, whose launch sets it to <robot_id>/base_link), the same place
 * nav2's BT nodes read it from. Fails, with the reason on the blackboard, on
 * a non-positive distance or an empty frame.
 */
class ComputeBackOutPose : public BT::SyncActionNode
{
public:
  ComputeBackOutPose(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Metres straight back from the robot's current pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "back_out_pose", "(-distance, 0) in the robot's base frame, same heading"),
    };
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_BACK_OUT_POSE_HPP_
