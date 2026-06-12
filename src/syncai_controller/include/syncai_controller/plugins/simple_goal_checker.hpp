#ifndef SYNCAI_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_nav_core/goal_checker.hpp"

namespace syncai_controller
{

/**
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
 */
class SimpleGoalChecker : public syncai_nav_core::GoalChecker
{
public:
  SimpleGoalChecker();
  // Standard GoalChecker Interface
  void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;

  // Returns true if the goal is reached, false otherwise.
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  bool symmetric_yaw_tolerance_;
  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
