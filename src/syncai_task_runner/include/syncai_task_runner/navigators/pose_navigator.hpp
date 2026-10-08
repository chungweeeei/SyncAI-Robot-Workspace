#ifndef SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_
#define SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_common/action/navigate_to_goal.hpp"
#include "syncai_task_runner/navigator.hpp"
#include "syncai_util/odometry_utils.hpp"

namespace syncai_task_runner
{

/**
 * @brief What differs between the two actions PoseNavigator serves. One
 * specialisation per action; everything else (the tree, the blackboard keys,
 * feedback, preemption) is shared and compiles against the field names the two
 * actions have in common -- NavigateToGoal mirrors nav2's on purpose.
 */
template <class ActionT>
struct PoseNavigatorTraits;

template <>
struct PoseNavigatorTraits<nav2_msgs::action::NavigateToPose>
{
  static constexpr const char * kActionName = "navigate_to_pose";
  // Humble's result is std_msgs/Empty: nothing to fill on failure.
  static constexpr bool kReportsFailure = false;
  // rviz's "2D Goal Pose" (the goal_pose topic) drives this one. Moving the
  // subscriber to the other navigator would turn an rviz click during a
  // running navigate_to_pose goal from an in-place preempt into a mutex
  // rejection; it moves when the nav2 action is retired, by flipping these
  // two flags.
  static constexpr bool kDrivesRvizGoalPose = true;
};

template <>
struct PoseNavigatorTraits<syncai_common::action::NavigateToGoal>
{
  static constexpr const char * kActionName = "navigate_to_goal";
  static constexpr bool kReportsFailure = true;
  static constexpr bool kDrivesRvizGoalPose = false;
};

/**
 * @class PoseNavigator
 * @brief Navigate to a single pose by ticking move.xml. One template serving
 * both nav2_msgs/NavigateToPose (unchanged; what the backend sends today) and
 * syncai_common/NavigateToGoal (2026-10: the same navigation with error_code /
 * error_msg in the result and number_of_replans in the feedback, so the
 * operator can be told whether the robot failed to plan or failed to drive).
 * The three places the two differ sit behind PoseNavigatorTraits and
 * `if constexpr`; the explicit instantiations are at the bottom of the .cpp.
 *
 * Both instantiations share the host node's parameters (default_bt_xml,
 * goal_blackboard_id, path_blackboard_id, the BtActionServer timings) and
 * the one NavigatorMutex: same tree, same keys, one navigation at a time.
 */
template <class ActionT>
class PoseNavigator : public Navigator<ActionT>
{
public:
  using Traits = PoseNavigatorTraits<ActionT>;

  PoseNavigator() : Navigator<ActionT>() {}

  bool configure(
    rclcpp::Node::WeakPtr parent_node,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother) override;

  bool cleanup() override;

  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  std::string getName() override { return std::string(Traits::kActionName); }

  std::string getDefaultBTFilepath(rclcpp::Node::WeakPtr node) override;

protected:
  bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Fill the result for a tree that ran and FAILED: the failed_node /
   * failure_msg blackboard keys the BT action nodes wrote, mapped to the
   * action's error_code (NavigateToGoal only; the nav2 result has no fields).
   * SUCCEEDED and CANCELED leave the default (NONE). A goal superseded by a
   * preempt or rejected before the tree ran never gets here -- the action
   * server aborts it with an empty result, which is why ABORTED + NONE means
   * "not a navigation failure" to the client.
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPose(typename ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  // Only the rviz-driving instantiation creates these (Traits::kDrivesRvizGoalPose)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  typename rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;
};

using NavigateToPoseNavigator = PoseNavigator<nav2_msgs::action::NavigateToPose>;
using NavigateToGoalNavigator = PoseNavigator<syncai_common::action::NavigateToGoal>;

// Defined once, in pose_navigator.cpp, so the node -> error_code table stays a
// .cpp-private detail and the library serves both actions.
extern template class PoseNavigator<nav2_msgs::action::NavigateToPose>;
extern template class PoseNavigator<syncai_common::action::NavigateToGoal>;

}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_
