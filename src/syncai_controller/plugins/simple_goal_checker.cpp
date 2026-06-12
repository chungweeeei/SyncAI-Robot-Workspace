#include "syncai_controller/plugins/simple_goal_checker.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace syncai_controller
{

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  symmetric_yaw_tolerance_(false),
  xy_goal_tolerance_sq_(0.0625)
{
}

void SimpleGoalChecker::initialize(
  const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
  const std::shared_ptr<syncai_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;

  syncai_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_INFO(
    node->get_logger(), "[SimpleGoalChecker][%s] xy_goal_tolerance: %f", __func__,
    xy_goal_tolerance_);

  syncai_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  RCLCPP_INFO(
    node->get_logger(), "[SimpleGoalChecker][%s] yaw_goal_tolerance: %f", __func__,
    yaw_goal_tolerance_);

  syncai_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));
  node->get_parameter(plugin_name + ".stateful", stateful_);
  RCLCPP_INFO(
    node->get_logger(), "[SimpleGoalChecker][%s] stateful: %s", __func__,
    stateful_ ? "true" : "false");

  syncai_util::declare_parameter_if_not_declared(
    node, plugin_name + ".symmetric_yaw_tolerance", rclcpp::ParameterValue(false));
  node->get_parameter(plugin_name + ".symmetric_yaw_tolerance", symmetric_yaw_tolerance_);
  RCLCPP_INFO(
    node->get_logger(), "[SimpleGoalChecker][%s] symmetric_yaw_tolerance: %s", __func__,
    symmetric_yaw_tolerance_ ? "true" : "false");

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleGoalChecker::dynamicParametersCallback, this, _1));
}

void SimpleGoalChecker::reset()
{
  // reset the state of the goal checker
  check_xy_ = true;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
           dy = query_pose.position.y - goal_pose.position.y;

    // calculate the squared distance and compare to the squared tolerance to avoid a sqrt
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window, If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }

  // start checking robot heading and goal orientation
  double query_yaw = tf2::getYaw(query_pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.orientation);

  if (symmetric_yaw_tolerance_) {
    // For symmetric robots: accept either goal orientation or goal + 180°
    double dyaw_forward = angles::shortest_angular_distance(query_yaw, goal_yaw);
    double dyaw_backward =
      angles::shortest_angular_distance(query_yaw, angles::normalize_angle(goal_yaw + M_PI));

    bool forward_match = fabs(dyaw_forward) <= yaw_goal_tolerance_;
    bool backward_match = fabs(dyaw_backward) <= yaw_goal_tolerance_;

    return forward_match || backward_match;
  } else {
    // For non symmetric robots: only accept goal orientation
    double dyaw = angles::shortest_angular_distance(query_yaw, goal_yaw);
    return fabs(dyaw) <= yaw_goal_tolerance_;
  }
}

bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    syncai_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

rcl_interfaces::msg::SetParametersResult SimpleGoalChecker::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".symmetric_yaw_tolerance") {
        symmetric_yaw_tolerance_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace syncai_controller

PLUGINLIB_EXPORT_CLASS(syncai_controller::SimpleGoalChecker, syncai_nav_core::GoalChecker)
