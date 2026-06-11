// Copyright (c) 2025 Prabhav Saxena
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

#ifndef SYNCAI_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"

namespace syncai_controller
{

/**
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
 */
class PositionGoalChecker : public syncai_nav_core::GoalChecker
{
public:
  PositionGoalChecker();
  ~PositionGoalChecker() override = default;

  void initialize(
    const rclcpp::Node::SharedPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

  /**
   * @brief Set the XY goal tolerance
   * @param tolerance New tolerance value
   */
  void setXYGoalTolerance(double tolerance);

protected:
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;
  bool stateful_;
  bool position_reached_;
  std::string plugin_name_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
  * @brief Callback executed when a parameter change is detected
  * @param parameters list of changed parameters
  */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
