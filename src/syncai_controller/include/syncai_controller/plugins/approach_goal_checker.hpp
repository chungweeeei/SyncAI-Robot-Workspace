#ifndef SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECKER_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_controller/plugins/approach_goal_check.hpp"
#include "syncai_nav_core/goal_checker.hpp"

namespace syncai_controller
{

/**
 * @class syncai_controller::ApproachGoalChecker
 * @brief ApproachGoalCheck as a syncai_nav_core::GoalChecker: the goal
 * checker of a charge (track_pose with the Charging controller). Per axis in
 * the goal's frame, settled, never latched -- approach_goal_check.hpp has
 * what that is and why dock_goal_checker is not it.
 *
 * getTolerances() reports along as position.x and lateral as position.y:
 * track_pose reads them to know when the robot has entered the tolerances
 * (it then freezes the target for the settling).
 */
class ApproachGoalChecker : public syncai_nav_core::GoalChecker
{
public:
  ApproachGoalChecker() = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros) override;
  void reset() override;

  /** @param velocity UNUSED: settling is dwell time (approach_goal_check.hpp) */
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ApproachGoalChecker")};
  rclcpp::Clock::SharedPtr clock_;
  ApproachGoalCheck check_;
  // isGoalReached() runs on an action loop's thread, the parameter callback
  // on the node's executor.
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECKER_HPP_
