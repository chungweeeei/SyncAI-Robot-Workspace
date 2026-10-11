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

#ifndef SYNCAI_MPPI_CONTROLLER__CONTROLLER_HPP_
#define SYNCAI_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

#include "syncai_mppi_controller/tools/path_handler.hpp"
#include "syncai_mppi_controller/optimizer.hpp"
#include "syncai_mppi_controller/tools/trajectory_visualizer.hpp"
#include "syncai_mppi_controller/models/constraints.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"

#include "syncai_nav_core/controller.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIController : public syncai_nav_core::Controller
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;

  /**
    * @brief Configure and start the controller (nav2's configure + activate)
    * @param parent Node hosting the plugin (the controller server)
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    *
    * syncai_nav_core has no lifecycle, so upstream's configure() and
    * activate() run back to back here: publishers are live and the dynamic
    * parameter callback is registered before this returns. There is no
    * cleanup() / deactivate(); the Optimizer's destructor still shuts the
    * noise generator's thread down.
    */
  void initialize(
    const rclcpp::Node::SharedPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
    * @brief Reset the optimizer's warm start
    *
    * The controller server calls this once per goal and after a cycle in
    * which it had no robot pose (see syncai_nav_core::Controller::reset()).
    * Both are moments where the previous control sequence no longer
    * describes what the robot is doing, which is what MPPI's own
    * reset_period soft reset also covers.
    */
  void reset() override;

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    syncai_nav_core::GoalChecker * goal_checker) override;

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);

  /**
    * @brief Upstream's computeVelocityCommands() body
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommandsImpl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    syncai_nav_core::GoalChecker * goal_checker);

  /**
    * @brief Limit the change from the last command to max_*_accel per cycle
    * @param cmd Command to clamp in place
    */
  void clampAcceleration(geometry_msgs::msg::Twist & cmd) const;

  /**
    * @brief Publish the optimal trajectory on local_plan as a nav_msgs/Path
    * @param stamp Stamp for the path header
    */
  void publishLocalPlan(const rclcpp::Time & stamp);

  std::string name_;
  rclcpp::Node::WeakPtr parent_;
  // The optimal trajectory as a path, every cycle (see publishLocalPlan()).
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;

  double reset_period_;
  double max_linear_accel_{1.0};
  double max_angular_accel_{3.2};
  double control_duration_{0.05};
  // The last command returned, the clamp's reference; zeroed by reset() and
  // by any throw out of computeVelocityCommands().
  geometry_msgs::msg::Twist last_cmd_vel_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;
};

}  // namespace syncai_mppi_controller

#endif  // SYNCAI_MPPI_CONTROLLER__CONTROLLER_HPP_
