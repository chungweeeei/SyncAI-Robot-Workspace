#ifndef SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_
#define SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "syncai_common/action/navigate_through_goals.hpp"
#include "syncai_common/action/navigate_to_goal.hpp"
#include "syncai_task_runner/navigator.hpp"
#include "syncai_util/odometry_utils.hpp"
#include "syncai_util/path_progress.hpp"

namespace syncai_task_runner
{

/**
 * @brief What differs between the actions PoseNavigator serves. One
 * specialisation per action; everything else (the tree, the blackboard keys,
 * feedback, preemption) is shared and compiles against the field names the
 * actions have in common -- NavigateToGoal mirrors nav2's on purpose.
 * kThroughPoses marks the one whose goal is a list (`poses`, written to the
 * blackboard's `goals`) rather than one `pose` (`goal`), with its own default
 * tree.
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
  static constexpr bool kThroughPoses = false;
  static constexpr const char * kDefaultBtParam = "default_bt_xml";
  static constexpr const char * kDefaultBtFile = "move.xml";
};

template <>
struct PoseNavigatorTraits<syncai_common::action::NavigateToGoal>
{
  static constexpr const char * kActionName = "navigate_to_goal";
  static constexpr bool kReportsFailure = true;
  static constexpr bool kDrivesRvizGoalPose = false;
  static constexpr bool kThroughPoses = false;
  static constexpr const char * kDefaultBtParam = "default_bt_xml";
  static constexpr const char * kDefaultBtFile = "move.xml";
};

// The patrol route (2026-11): drive through every pose in order without
// stopping, the last one reached like a MOVE's goal (xy and heading). nav2's
// own action, so its result is Empty and its feedback has no status_msg;
// syncai_common's NavigateThroughGoals below is the twin that has them, and
// this one stays until the backend's PATROL step has moved to it. Its own
// default tree parameter, because the default trees differ while
// navigate_to_pose and navigate_to_goal share move.xml.
template <>
struct PoseNavigatorTraits<nav2_msgs::action::NavigateThroughPoses>
{
  static constexpr const char * kActionName = "navigate_through_poses";
  static constexpr bool kReportsFailure = false;
  static constexpr bool kDrivesRvizGoalPose = false;
  static constexpr bool kThroughPoses = true;
  static constexpr const char * kDefaultBtParam = "default_nav_through_poses_bt_xml";
  static constexpr const char * kDefaultBtFile = "patrol.xml";
};

// NavigateThroughGoals (2026-11): navigate_through_poses with
// NavigateToGoal's error_code / error_msg in the result and number_of_replans
// / status_msg in the feedback, so an aborted PATROL step says why -- a
// waypoint in a keepout zone is permanent, a person in the way is not, and
// the backend retried both. Same default tree parameter as the nav2 one:
// both run patrol.xml.
template <>
struct PoseNavigatorTraits<syncai_common::action::NavigateThroughGoals>
{
  static constexpr const char * kActionName = "navigate_through_goals";
  static constexpr bool kReportsFailure = true;
  static constexpr bool kDrivesRvizGoalPose = false;
  static constexpr bool kThroughPoses = true;
  static constexpr const char * kDefaultBtParam = "default_nav_through_poses_bt_xml";
  static constexpr const char * kDefaultBtFile = "patrol.xml";
};

/**
 * @class PoseNavigator
 * @brief Navigate to a single pose by ticking move.xml, or through a list of
 * them by ticking patrol.xml. One template serving nav2_msgs/NavigateToPose
 * (unchanged; what the backend sends today), syncai_common/NavigateToGoal
 * (2026-10: the same navigation with error_code / error_msg in the result and
 * number_of_replans in the feedback, so the operator can be told whether the
 * robot failed to plan or failed to drive), nav2_msgs/NavigateThroughPoses
 * (2026-11, the patrol route) and syncai_common/NavigateThroughGoals (2026-11:
 * the patrol route with NavigateToGoal's result and feedback). The places
 * they differ sit behind PoseNavigatorTraits and `if constexpr`; the explicit
 * instantiations are at the bottom of the .cpp.
 *
 * All instantiations share the host node's parameters (goal_blackboard_id,
 * path_blackboard_id, the BtActionServer timings) and the one NavigatorMutex:
 * one navigation at a time across the four actions. The single-pose two
 * share default_bt_xml, the two routes default_nav_through_poses_bt_xml.
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
   * SUCCEEDED and CANCELED leave the default (NONE); CANCELED also asks the
   * planner to clear the route it keeps re-sending on `plan`. A goal superseded by a
   * preempt or rejected before the tree ran never gets here -- the action
   * server aborts it with an empty result, which is why ABORTED + NONE means
   * "not a navigation failure" to the client.
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPose(typename ActionT::Goal::ConstSharedPtr goal);

  /**
   * @brief The tree file a goal runs: its behavior_tree resolved (a bare name
   * against behavior_trees/, empty = default_bt_xml). Used for a new goal
   * and for a preempt's same-tree comparison alike. From 2026-10-19 to
   * 2026-10-28 a 3D map swapped in the `<stem>_3d.xml` sibling; the 3D trees
   * are the only ones since, under the plain names.
   */
  std::string selectBtFilename(const std::string & requested) const;

  /**
   * @brief Ask the planner (clear_plan) to stop re-sending the last route on
   * `plan` and send an empty one. Fire and forget.
   */
  void clearPlannerPlan();

  rclcpp::Time start_time_;

  // Bumped at every goal and accepted preempt, written to the blackboard as
  // blackboard_keys::kGoalGeneration (FollowPath's stale-success test).
  int goal_generation_{0};

  // Only the rviz-driving instantiation creates these (Traits::kDrivesRvizGoalPose)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  typename rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  // The planner's clear_plan: called on a success or a cancel, so the console
  // stops showing the route of a navigation that no longer exists
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_plan_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;

  // The robot's place on the path, for the feedback's distance_remaining.
  // Forward only (path_progress.hpp): a patrol route passes back near itself,
  // and a whole-path nearest pose there reported the distance from a later
  // leg (2026-11). A replan (another stamp) starts it over.
  syncai_util::PathProgress feedback_progress_;

  // navigate_through_poses only: a waypoint within this of the robot is
  // passed and dropped from remaining_goals, so a replan starts from the
  // next one (the last one is never dropped).
  double waypoint_passed_radius_{1.0};
  // And within this of the robot's ground height (2026-11): the radius is
  // planar, so on a two-storey route the robot on 1F passed the 2F waypoint
  // right above it and dropped it. The ground is body_frame's z in the map
  // frame minus lidar_height (base_link is planar), as the controllers and
  // Octo3D read it.
  double waypoint_passed_z_tolerance_{1.0};
  std::string body_frame_{"pointlio_body"};
  double lidar_height_{0.481};
};

using NavigateToPoseNavigator = PoseNavigator<nav2_msgs::action::NavigateToPose>;
using NavigateToGoalNavigator = PoseNavigator<syncai_common::action::NavigateToGoal>;
using NavigateThroughPosesNavigator = PoseNavigator<nav2_msgs::action::NavigateThroughPoses>;
using NavigateThroughGoalsNavigator = PoseNavigator<syncai_common::action::NavigateThroughGoals>;

// Defined once, in pose_navigator.cpp, so the node -> error_code table stays a
// .cpp-private detail and the library serves both actions.
extern template class PoseNavigator<nav2_msgs::action::NavigateToPose>;
extern template class PoseNavigator<syncai_common::action::NavigateToGoal>;
extern template class PoseNavigator<nav2_msgs::action::NavigateThroughPoses>;

}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__NAVIGATORS__POSE_NAVIGATOR_HPP_
