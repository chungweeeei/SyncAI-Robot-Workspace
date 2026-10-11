// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <algorithm>
#include <chrono>
#include "syncai_mppi_controller/controller.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_util/geometry_utils.hpp"

// #define BENCHMARK_TESTING

namespace syncai_mppi_controller
{

void MPPIController::initialize(
  const rclcpp::Node::SharedPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent_);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  // Not upstream; see clampAcceleration(). <= 0 disables either clamp.
  getParam(max_linear_accel_, "max_linear_accel", 1.0);
  getParam(max_angular_accel_, "max_angular_accel", 3.2);
  // The server's own parameter, read the way the Optimizer reads it.
  auto getParentParam = parameters_handler_->getParamGetter("");
  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 20.0, ParameterType::Static);
  control_duration_ = 1.0 / controller_frequency;

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  // Not upstream. The optimal trajectory is the "local path" the rest of
  // this stack has asked for since RPP, which has none to show; upstream only
  // draws it as markers inside `trajectories`, and only with visualize: true,
  // which also publishes every candidate (1000 x time_steps markers a cycle)
  // and costs real CPU. One Path of time_steps poses is cheap enough to send
  // every cycle. Relative name: it lands in the robot_id namespace.
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  // upstream activate(): no lifecycle here, so the dynamic parameter
  // callback is registered as soon as everything it touches exists.
  parameters_handler_->start();

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
  last_cmd_vel_ = geometry_msgs::msg::Twist();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  syncai_nav_core::GoalChecker * goal_checker)
{
  try {
    geometry_msgs::msg::TwistStamped cmd =
      computeVelocityCommandsImpl(robot_pose, robot_speed, goal_checker);
    clampAcceleration(cmd.twist);
    last_cmd_vel_ = cmd.twist;
    return cmd;
  } catch (...) {
    // Same rule as RPP's catch-all: every throw out of a cycle means the
    // server commands zero for it (failure_tolerance) or ends the goal, so
    // the clamp baseline must follow that stop. Left at the abandoned speed,
    // the first good cycle after "Optimizer fail to compute path" would be
    // allowed back up to it in one step.
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    throw;
  }
}

void MPPIController::clampAcceleration(geometry_msgs::msg::Twist & cmd) const
{
  // Humble's MPPI bounds velocity only (vx_max / wz_max); the acceleration
  // limits (ax_max, ...) arrived upstream after Humble. Its warm start and
  // vx_std keep consecutive outputs close in steady driving, but nothing
  // stops a step from rest, or out of a reset, to anything inside the
  // bounds, and this stack has no velocity smoother between the controller
  // and the gait controller. RPP clamps the same way for the same reason
  // (max_linear_accel 1.0 there too), around its own last command rather
  // than measured odometry, so a slow gait response cannot hold the window
  // shut. The optimizer is not told: its next cycle starts from measured
  // speed, and the warm-started sequence absorbs the difference.
  if (max_linear_accel_ > 0.0) {
    const double dv = max_linear_accel_ * control_duration_;
    cmd.linear.x =
      std::clamp(cmd.linear.x, last_cmd_vel_.linear.x - dv, last_cmd_vel_.linear.x + dv);
    cmd.linear.y =
      std::clamp(cmd.linear.y, last_cmd_vel_.linear.y - dv, last_cmd_vel_.linear.y + dv);
  }
  if (max_angular_accel_ > 0.0) {
    const double dw = max_angular_accel_ * control_duration_;
    cmd.angular.z =
      std::clamp(cmd.angular.z, last_cmd_vel_.angular.z - dw, last_cmd_vel_.angular.z + dw);
  }
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommandsImpl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  syncai_nav_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  syncai_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<syncai_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  publishLocalPlan(clock_->now());

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::publishLocalPlan(const rclcpp::Time & stamp)
{
  if (local_plan_pub_->get_subscription_count() == 0) {
    return;
  }

  // Columns are x, y, yaw in the costmap's global frame (<robot_id>/odom),
  // integrated from the control sequence just optimised, so this is the
  // path the command about to go out is the first step of.
  const auto trajectory = optimizer_.getOptimizedTrajectory();
  auto path = std::make_unique<nav_msgs::msg::Path>();
  path->header.frame_id = costmap_ros_->getGlobalFrameID();
  path->header.stamp = stamp;
  path->poses.reserve(trajectory.shape()[0]);
  for (size_t i = 0; i < trajectory.shape()[0]; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path->header;
    pose.pose.position.x = trajectory(i, 0);
    pose.pose.position.y = trajectory(i, 1);
    pose.pose.orientation = syncai_util::geometry_utils::orientationAroundZAxis(trajectory(i, 2));
    path->poses.push_back(std::move(pose));
  }
  local_plan_pub_->publish(std::move(path));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace syncai_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_mppi_controller::MPPIController, syncai_nav_core::Controller)
