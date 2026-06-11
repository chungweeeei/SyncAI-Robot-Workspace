#ifndef SYNCAI_NAVIGATOR__NAVIGATOR_HPP_
#define SYNCAI_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace syncai_navigator
{

/**
 * @class syncai_navigator::Navigator
 * @brief Minimal replacement for nav2's bt_navigator: chains the planner and
 * controller action servers behind a single goal-pose topic.
 *
 * Subscribes to goal_pose (what RViz's "2D Goal Pose" tool publishes), asks
 * planner_server for a path via the compute_path_to_pose action, and on
 * success forwards the path to controller_server via the follow_path action.
 * Sending a new goal while one is in progress simply issues new action goals;
 * the SimpleActionServer on the receiving side preempts the old ones.
 */
class Navigator : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit Navigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief Callback for a new goal pose: requests a path from planner_server
   * @param goal The goal pose (in the planner's global frame, usually map)
   */
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal);

  /**
   * @brief Result callback of compute_path_to_pose: forwards the path to
   * controller_server on success
   */
  void onComputePathResult(const ComputePathGoalHandle::WrappedResult & result);

  /**
   * @brief Result callback of follow_path: logs the navigation outcome
   */
  void onFollowPathResult(const FollowPathGoalHandle::WrappedResult & result);

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace syncai_navigator

#endif  // SYNCAI_NAVIGATOR__NAVIGATOR_HPP_
