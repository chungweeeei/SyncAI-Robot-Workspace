#ifndef SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
#define SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_TO_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_task_runner/navigator.hpp"
#include "syncai_util/odometry_utils.hpp"

namespace syncai_task_runner
{
class NavigateToPoseNavigator
: public syncai_task_runner::Navigator<nav2_msgs::action::NavigateToPose>
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;

  NavigateToPoseNavigator() : Navigator() {}

  bool configure(
    rclcpp::Node::WeakPtr parent_node,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother) override;

  bool cleanup() override;

  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  std::string getName() override { return std::string("navigate_to_pose"); }

  std::string getDefaultBTFilepath(rclcpp::Node::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;
};
}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
