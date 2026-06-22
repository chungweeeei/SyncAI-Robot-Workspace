#ifndef SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_
#define SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_task_runner/navigator.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/odometry_utils.hpp"

namespace syncai_task_runner
{
// Register navigate through poses action server
class NavigateThroughPosesNavigator
: public syncai_task_runner::Navigator<nav2_msgs::action::NavigateThroughPoses>
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  NavigateThroughPosesNavigator() : Navigator() {}

  bool configure(
    rclcpp::Node::WeakPtr parent_node,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother) override;

  std::string getName() override { return std::string("navigate_through_poses"); }

  std::string getDefaultBTFilepath(rclcpp::Node::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;
};
}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_