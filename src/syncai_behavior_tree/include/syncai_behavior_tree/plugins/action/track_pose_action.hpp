#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_POSE_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "syncai_behavior_tree/bt_action_node.hpp"
#include "syncai_common/action/track_pose.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode class that wraps
 * syncai_common::action::TrackPose -- controller_server's approach to a pose
 * that keeps being corrected (track_pose). The last metre of charge.xml: the
 * poses are the charger's tag as PublishLandmark publishes it, the offset is
 * where base_link stands on the charger, seen from the tag.
 *
 * The offset is planar on the ports (offset_x / offset_y / offset_yaw, in the
 * tracked pose's own frame), for the reason DockToPose's goal is: a
 * hand-written quaternion is what goes wrong. The charger's geometry lives
 * in the tree that names these ports, and nowhere else.
 *
 * Writes the navigator's status sentence from the server's feedback and
 * report_failure()s on abort with the server's own reason and code, which
 * the navigator maps to DOCK_FAILED.
 */
class TrackPoseAction : public BtActionNode<syncai_common::action::TrackPose>
{
public:
  TrackPoseAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /** @brief Fill the goal from the ports */
  void on_tick() override;

  /** @brief Status sentence from the feedback */
  void on_wait_for_result(
    std::shared_ptr<const syncai_common::action::TrackPose::Feedback> feedback) override;

  /** @brief "On the charger", with the residuals in the log */
  BT::NodeStatus on_success() override;

  /** @brief The server's reason on the blackboard; the tree outcome is FAILURE */
  BT::NodeStatus on_aborted() override;

  /** @brief FAILURE: a cancelled approach is not a robot on its target */
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>(
        "pose_topic", "tracked_landmark",
        "geometry_msgs/PoseStamped topic to follow (PublishLandmark's); empty with "
        "tracked_frame set"),
      BT::InputPort<std::string>("tracked_frame", "", "TF frame to follow instead of a topic"),
      BT::InputPort<double>(
        "offset_x", 0.0,
        "Where base_link ends, in the tracked pose's frame [m]: for a tag (+x out of its "
        "face), the distance in front of it"),
      BT::InputPort<double>("offset_y", 0.0, "... across it [m]"),
      BT::InputPort<double>(
        "offset_yaw", 3.141592653589793, "... and turned by [rad]: pi faces the tag"),
      BT::InputPort<std::string>(
        "controller_id", "Charging", "controller_server's controller_plugins id"),
      BT::InputPort<std::string>(
        "goal_checker_id", "charge_goal_checker", "controller_server's goal_checker_plugins id"),
      BT::InputPort<double>(
        "filter_window", 1.0,
        "Seconds of targets the server takes the median of (the tracked thing stands "
        "still); 0 = no filter"),
      BT::InputPort<double>("pose_timeout", 0.0, "Seconds; <= 0 = the server's default"),
      BT::InputPort<double>("lost_timeout", 0.0, "Seconds; <= 0 = the server's default"),
      BT::InputPort<double>("max_duration", 0.0, "Seconds; <= 0 = the server's default"),
    });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_POSE_ACTION_HPP_
