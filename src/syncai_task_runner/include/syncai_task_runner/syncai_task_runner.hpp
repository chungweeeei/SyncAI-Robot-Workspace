#ifndef SYNCAI_TASK_RUNNER__TASK_RUNNER_HPP_
#define SYNCAI_TASK_RUNNER__TASK_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_task_runner/navigator.hpp"
#include "syncai_task_runner/navigators/navigate_to_pose.hpp"
#include "syncai_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_task_runner
{

/**
 * @class TaskRunner
 * @brief A plain (non-lifecycle) node that hosts BT-based navigators. Owns the
 * shared TF / odometry / mutex and registers each navigator. initialize() must
 * run after the node is held by a shared_ptr (it uses shared_from_this()).
 */
class TaskRunner : public rclcpp::Node
{
public:
  explicit TaskRunner(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~TaskRunner();

  bool initialize();

  bool cleanup();

protected:
  // The navigators hosted on this node and the mutex that keeps only one active.
  std::unique_ptr<syncai_task_runner::Navigator<nav2_msgs::action::NavigateToPose>> pose_navigator_;
  syncai_task_runner::NavigatorMutex plugin_mutex_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;

  // Metrics for feedback
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__TASK_RUNNER_HPP_
