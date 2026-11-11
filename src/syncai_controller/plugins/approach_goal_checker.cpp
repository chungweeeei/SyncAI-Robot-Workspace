#include "syncai_controller/plugins/approach_goal_checker.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
// tf2::getYaw() on a geometry_msgs quaternion needs this in the same TU
// (Release-build symbol lookup error otherwise, see simple_progress_checker.cpp).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using rcl_interfaces::msg::ParameterType;

namespace syncai_controller
{

namespace
{

// Declared and set by one table, as the ApproachController's law parameters.
struct CheckDouble
{
  const char * name;
  double ApproachGoalCheckParams::*member;
};

const CheckDouble kCheckDoubles[] = {
  {"along_tolerance", &ApproachGoalCheckParams::along_tolerance},
  {"lateral_tolerance", &ApproachGoalCheckParams::lateral_tolerance},
  {"yaw_tolerance", &ApproachGoalCheckParams::yaw_tolerance},
  {"settle_time", &ApproachGoalCheckParams::settle_time},
};

Pose2 toPose2(const geometry_msgs::msg::Pose & pose)
{
  return Pose2{pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}

}  // namespace

void ApproachGoalChecker::initialize(
  const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
  const std::shared_ptr<syncai_costmap::CostmapROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  ApproachGoalCheckParams params;
  for (const auto & d : kCheckDoubles) {
    const std::string key = plugin_name_ + "." + d.name;
    syncai_util::declare_parameter_if_not_declared(
      node, key, rclcpp::ParameterValue(params.*d.member));
    node->get_parameter(key, params.*d.member);
  }
  // Refused at startup, as the ApproachController refuses its own: a goal
  // checker that can never accept ends every charge in its timeout.
  const std::string why = params.validate();
  if (!why.empty()) {
    RCLCPP_FATAL(
      logger_, "[ApproachGoalChecker][%s] invalid parameters: %s", plugin_name_.c_str(),
      why.c_str());
    throw std::runtime_error("ApproachGoalChecker " + plugin_name_ + ": " + why);
  }
  check_.setParams(params);

  RCLCPP_INFO(
    logger_,
    "[ApproachGoalChecker][%s] along %.3f m, lateral %.3f m, yaw %.3f rad in the goal's frame, "
    "held for %.2f s. Each must be wider than where the controller lets that axis rest (the "
    "ApproachLaw's tolerance + hysteresis), or the goal never ends.",
    plugin_name_.c_str(), params.along_tolerance, params.lateral_tolerance, params.yaw_tolerance,
    params.settle_time);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ApproachGoalChecker::dynamicParametersCallback, this, std::placeholders::_1));
}

void ApproachGoalChecker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  check_.reset();
}

bool ApproachGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return check_.update(
    targetErrors(toPose2(query_pose), toPose2(goal_pose)), clock_->now().seconds());
}

bool ApproachGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double invalid_field = std::numeric_limits<double>::lowest();

  // In the GOAL's frame: x along its heading, y across it.
  pose_tolerance.position.x = check_.params().along_tolerance;
  pose_tolerance.position.y = check_.params().lateral_tolerance;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    syncai_util::geometry_utils::orientationAroundZAxis(check_.params().yaw_tolerance);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;
  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;
  return true;
}

rcl_interfaces::msg::SetParametersResult ApproachGoalChecker::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(mutex_);

  // The batch on a copy, validated whole before anything is taken.
  ApproachGoalCheckParams next = check_.params();
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name.rfind(plugin_name_ + ".", 0) != 0) {
      continue;
    }
    const std::string key = name.substr(plugin_name_.size() + 1);
    if (parameter.get_type() == ParameterType::PARAMETER_DOUBLE) {
      for (const auto & d : kCheckDoubles) {
        if (key == d.name) {
          next.*d.member = parameter.as_double();
        }
      }
    }
  }

  const std::string why = next.validate();
  if (!why.empty()) {
    result.successful = false;
    result.reason = plugin_name_ + ": " + why;
    RCLCPP_WARN(
      logger_, "[ApproachGoalChecker][%s] rejecting parameter update: %s", plugin_name_.c_str(),
      why.c_str());
    return result;
  }
  check_.setParams(next);
  result.successful = true;
  return result;
}

}  // namespace syncai_controller

PLUGINLIB_EXPORT_CLASS(syncai_controller::ApproachGoalChecker, syncai_nav_core::GoalChecker)
