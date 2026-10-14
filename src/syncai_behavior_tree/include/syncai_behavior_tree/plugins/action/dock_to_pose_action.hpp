#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_POSE_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "syncai_behavior_tree/bt_action_node.hpp"
#include "syncai_common/action/dock_to_pose.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps
 * syncai_common::action::DockToPose -- controller_server's pose-to-pose
 * docking approach (dock_to_pose). The last metre onto the dock in dock.xml
 * (controller_id "Docking"), the back-out to the staging pose in undock.xml
 * ("Undocking"): direction is the controller the goal names, not a goal
 * field, see DockToPose.action.
 *
 * The port is a PoseStamped (the trees' {goal} / {staging_pose}); the node
 * converts it to the action's planar frame_id / x / y / yaw.
 *
 * Writes the navigator's status sentence from the controller's feedback
 * ("Docking: 0.42 m from the dock") and report_failure()s on abort with the
 * controller's own reason, which the navigator maps to DOCK_FAILED.
 */
class DockToPoseAction : public BtActionNode<syncai_common::action::DockToPose>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::DockToPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  DockToPoseAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Fill the goal from the ports; refuse a pose without a frame_id
   */
  void on_tick() override;

  /**
   * @brief Status sentence from the feedback; re-send on a port change
   * @param feedback shared_ptr to latest feedback message
   */
  void on_wait_for_result(
    std::shared_ptr<const syncai_common::action::DockToPose::Feedback> feedback) override;

  /**
   * @brief "Docked" / "Undocked", with the residuals in the log
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Record the controller's reason on the blackboard for the
   * navigator's result (see report_failure()); the tree outcome is FAILURE
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief FAILURE, not the base class's SUCCESS: a cancelled approach is not
   * a docked robot. halt() ignores the value, so this only matters when
   * something other than this tree cancels the dock goal.
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "dock_pose",
        "Pose to end on, any TF frame; sent as frame_id / x / y / yaw, and the server "
        "transforms it to odom once"),
      BT::InputPort<std::string>(
        "controller_id", "Docking",
        "controller_server's controller_plugins id: Docking (forward) or Undocking (reverse)"),
      BT::InputPort<std::string>(
        "goal_checker_id", "dock_goal_checker", "controller_server's goal_checker_plugins id"),
      BT::InputPort<double>(
        "timeout", 0.0, "Seconds before the server aborts with TIMEOUT; <= 0 = server default"),
    });
  }

private:
  // Which sentences to write: decided from controller_id on each send, the
  // only thing this node knows about direction.
  bool is_undock_{false};
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_TO_POSE_ACTION_HPP_
