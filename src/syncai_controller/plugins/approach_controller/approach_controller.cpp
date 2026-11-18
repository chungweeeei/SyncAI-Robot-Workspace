// ApproachController: ApproachLaw as a syncai_nav_core::Controller. See the
// class comment in the header for what it is for and, as with
// DockingController, for what it deliberately is not: it never reads the
// costmap.

#include "syncai_controller/plugins/approach_controller/approach_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/robot_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using syncai_util::declare_parameter_if_not_declared;

namespace syncai_controller
{

namespace
{

// The law's scalar parameters, declared and set by one table so a parameter
// cannot be read at startup and forgotten by the dynamic callback.
struct LawDouble
{
  const char * name;
  double ApproachLawParams::*member;
};

const LawDouble kLawDoubles[] = {
  {"view_distance", &ApproachLawParams::view_distance},
  {"final_heading_distance", &ApproachLawParams::final_heading_distance},
  {"final_heading_hysteresis", &ApproachLawParams::final_heading_hysteresis},
  {"rotate_in_place_angle", &ApproachLawParams::rotate_in_place_angle},
  {"k_x", &ApproachLawParams::k_x},
  {"v_linear_min", &ApproachLawParams::v_linear_min},
  {"v_linear_max", &ApproachLawParams::v_linear_max},
  {"v_reverse_max", &ApproachLawParams::v_reverse_max},
  {"k_y", &ApproachLawParams::k_y},
  {"v_lateral_min", &ApproachLawParams::v_lateral_min},
  {"v_lateral_max", &ApproachLawParams::v_lateral_max},
  {"k_yaw", &ApproachLawParams::k_yaw},
  {"v_angular_min", &ApproachLawParams::v_angular_min},
  {"v_angular_max", &ApproachLawParams::v_angular_max},
  {"x_tolerance", &ApproachLawParams::x_tolerance},
  {"x_hysteresis", &ApproachLawParams::x_hysteresis},
  {"lateral_tolerance", &ApproachLawParams::lateral_tolerance},
  {"lateral_hysteresis", &ApproachLawParams::lateral_hysteresis},
  {"heading_tolerance", &ApproachLawParams::heading_tolerance},
  {"heading_hysteresis", &ApproachLawParams::heading_hysteresis},
  {"gate_lateral_min", &ApproachLawParams::gate_lateral_min},
  {"gate_lateral_slope", &ApproachLawParams::gate_lateral_slope},
  {"gate_heading", &ApproachLawParams::gate_heading},
  {"gate_reopen_ratio", &ApproachLawParams::gate_reopen_ratio},
};

// The speed table is two parallel arrays: ROS parameters have no array of
// pairs.
constexpr const char * kTableDistance = "speed_table_distance";
constexpr const char * kTableSpeed = "speed_table_speed";

// Empty when the two arrays pair up, else why not.
std::string pairTable(
  const std::vector<double> & distance, const std::vector<double> & speed,
  std::vector<std::pair<double, double>> & table)
{
  if (distance.size() != speed.size()) {
    return std::string(kTableDistance) + " has " + std::to_string(distance.size()) +
           " entries and " + kTableSpeed + " " + std::to_string(speed.size());
  }
  table.clear();
  for (std::size_t i = 0; i < distance.size(); ++i) {
    table.emplace_back(distance[i], speed[i]);
  }
  return "";
}

}  // namespace

void ApproachController::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros)
{
  // Frame names only. No getCostmap(), no collision checker: see the header.
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  ApproachLawParams params;
  for (const auto & d : kLawDoubles) {
    const std::string key = plugin_name_ + "." + d.name;
    declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(params.*d.member));
    node->get_parameter(key, params.*d.member);
  }
  std::vector<double> distance;
  std::vector<double> speed;
  for (const auto & [d, v] : params.speed_table) {
    distance.push_back(d);
    speed.push_back(v);
  }
  declare_parameter_if_not_declared(
    node, plugin_name_ + "." + kTableDistance, rclcpp::ParameterValue(distance));
  declare_parameter_if_not_declared(
    node, plugin_name_ + "." + kTableSpeed, rclcpp::ParameterValue(speed));
  node->get_parameter(plugin_name_ + "." + kTableDistance, distance);
  node->get_parameter(plugin_name_ + "." + kTableSpeed, speed);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(max_linear_accel_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(max_angular_accel_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  double control_frequency = 20.0;
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  // A configuration the law cannot run is refused at startup, not patched up
  // the way DockingController raises a cap: these limits interlock (a gate
  // narrower than a band never reopens), and a guessed fix would hide which
  // one the params file got wrong. The server does not catch this, so the
  // node goes down with the reason as its last line -- as MPPI's model_dt.
  std::string why = pairTable(distance, speed, params.speed_table);
  if (why.empty()) {
    why = params.validate();
  }
  if (why.empty() && (max_linear_accel_ <= 0.0 || max_angular_accel_ <= 0.0)) {
    why = "max_linear_accel and max_angular_accel must be > 0";
  }
  if (!why.empty()) {
    RCLCPP_FATAL(
      logger_, "[ApproachController][%s] invalid parameters: %s", plugin_name_.c_str(),
      why.c_str());
    throw std::runtime_error("ApproachController " + plugin_name_ + ": " + why);
  }
  law_ = std::make_unique<ApproachLaw>(params);

  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("approach_target", 1);

  RCLCPP_INFO(
    logger_,
    "[ApproachController][%s] v_linear [%.2f, %.2f] (reverse %.2f), v_lateral [%.2f, %.2f] m/s, "
    "v_angular [%.2f, %.2f] rad/s (pre-scale; each min is a gait floor); bands x %.3f m, "
    "lateral %.3f m, heading %.3f rad; view point %.2f m beyond the target. Obstacle avoidance "
    "is OFF: this controller does not read the local costmap.",
    plugin_name_.c_str(), params.v_linear_min, params.v_linear_max, params.v_reverse_max,
    params.v_lateral_min, params.v_lateral_max, params.v_angular_min, params.v_angular_max,
    params.x_tolerance, params.lateral_tolerance, params.heading_tolerance, params.view_distance);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ApproachController::dynamicParametersCallback, this, std::placeholders::_1));
}

void ApproachController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw syncai_nav_core::PlannerException("ApproachController: received an empty path");
  }

  geometry_msgs::msg::PoseStamped target = path.poses.back();
  if (!path.header.frame_id.empty()) {
    target.header.frame_id = path.header.frame_id;
  }

  // Track in odom and nothing else. track_pose hands odom already (each pose
  // transformed at its own stamp, which only the server knows how to do);
  // this branch is for a caller that does not, and takes the latest
  // transform, as DockingController's does.
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  if (target.header.frame_id != global_frame) {
    target.header.stamp = rclcpp::Time(0);
    geometry_msgs::msg::PoseStamped transformed;
    if (!syncai_util::transformPoseInTargetFrame(
          target, transformed, *tf_, global_frame, transform_tolerance_)) {
      throw syncai_nav_core::PlannerException(
        "ApproachController: cannot transform the target from '" + target.header.frame_id +
        "' into '" + global_frame + "'");
    }
    target = transformed;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  target_ = target;
  have_target_ = true;
  // Nothing else: a new target is the same goal seen again. The law's band
  // and gate state and the acceleration baseline carry on; resetting them on
  // every sighting (several a second) would restart a strafe that is halfway
  // across its band and re-ramp the command, the sawtooth RPP measured.
}

void ApproachController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_cmd_vel_ = geometry_msgs::msg::Twist();
  if (law_) {
    law_->reset();
  }
  have_last_mode_ = false;
}

geometry_msgs::msg::TwistStamped ApproachController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*velocity*/,
  syncai_nav_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    return computeVelocityCommandsImpl(pose);
  } catch (...) {
    // A throw is a stop (the server publishes zero), so the clamp baseline
    // follows it; DockingController's wrapper has the full reason.
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    throw;
  }
}

geometry_msgs::msg::TwistStamped ApproachController::computeVelocityCommandsImpl(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (!have_target_) {
    throw syncai_nav_core::PlannerException("ApproachController: no target set");
  }
  if (pose.header.frame_id != target_.header.frame_id) {
    throw syncai_nav_core::PlannerException(
      "ApproachController: robot pose is in '" + pose.header.frame_id + "' but the target in '" +
      target_.header.frame_id + "'");
  }

  const Pose2 robot{pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)};
  const Pose2 target{
    target_.pose.position.x, target_.pose.position.y, tf2::getYaw(target_.pose.orientation)};
  ApproachLaw::Mode mode;
  ApproachLaw::Errors e;
  const Velocity2 v = law_->compute(robot, target, &mode, &e);

  if (!have_last_mode_ || mode != last_mode_) {
    // The numbers that decided it: which band each error is against, and the
    // gate. A robot hunting across a band logs this pair every second.
    RCLCPP_INFO(
      logger_,
      "[ApproachController][%s] %s: along %+.3f m (band %.3f), lateral %+.3f m (band %.3f), "
      "heading %+.3f rad off the %s (band %.3f), yaw to the target %+.3f rad, gate %s",
      plugin_name_.c_str(), toString(mode), e.along, law_->params().x_tolerance, e.lateral,
      law_->params().lateral_tolerance, e.heading, e.on_view_point ? "view point" : "target's yaw",
      law_->params().heading_tolerance, e.yaw, e.gate_open ? "open" : "closed");
    last_mode_ = mode;
    have_last_mode_ = true;
  }

  if (target_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseStamped msg = target_;
    msg.header.stamp = clock_->now();
    target_pub_->publish(msg);
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = v.vx;
  cmd_vel.twist.linear.y = v.vy;
  cmd_vel.twist.angular.z = v.wz;
  clampAcceleration(cmd_vel.twist);
  last_cmd_vel_ = cmd_vel.twist;
  return cmd_vel;
}

void ApproachController::clampAcceleration(geometry_msgs::msg::Twist & cmd) const
{
  // Baseline: our previous command (gait sway in the measured twist). No
  // velocity smoother in the stack, so the controller owns the ramp up; the
  // ramp down it does not, see the header. At these floors a ramp up from
  // zero is one or two cycles anyway (0.05 m/s at 1.0 m/s^2 x 50 ms).
  const double dv = max_linear_accel_ * control_duration_;
  const double dw = max_angular_accel_ * control_duration_;
  const auto ramp = [](double & value, double last, double step) {
    if (value != 0.0) {
      value = std::clamp(value, last - step, last + step);
    }
  };
  ramp(cmd.linear.x, last_cmd_vel_.linear.x, dv);
  ramp(cmd.linear.y, last_cmd_vel_.linear.y, dv);
  ramp(cmd.angular.z, last_cmd_vel_.angular.z, dw);
}

rcl_interfaces::msg::SetParametersResult ApproachController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(mutex_);

  // The whole batch is applied to a copy and validated as a whole before
  // anything is taken (DockingController's rule: a rejection that still
  // reports success leaves the console showing a value that is not in force),
  // which also lets one set move a floor and its cap together.
  ApproachLawParams next = law_->params();
  std::vector<double> distance;
  std::vector<double> speed;
  for (const auto & [d, v] : next.speed_table) {
    distance.push_back(d);
    speed.push_back(v);
  }
  double next_linear_accel = max_linear_accel_;
  double next_angular_accel = max_angular_accel_;
  double next_transform_tolerance = transform_tolerance_;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name.rfind(plugin_name_ + ".", 0) != 0) {
      continue;
    }
    const std::string key = name.substr(plugin_name_.size() + 1);
    if (parameter.get_type() == ParameterType::PARAMETER_DOUBLE) {
      for (const auto & d : kLawDoubles) {
        if (key == d.name) {
          next.*d.member = parameter.as_double();
        }
      }
      if (key == "max_linear_accel") {
        next_linear_accel = parameter.as_double();
      } else if (key == "max_angular_accel") {
        next_angular_accel = parameter.as_double();
      } else if (key == "transform_tolerance") {
        next_transform_tolerance = parameter.as_double();
      }
    } else if (parameter.get_type() == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (key == kTableDistance) {
        distance = parameter.as_double_array();
      } else if (key == kTableSpeed) {
        speed = parameter.as_double_array();
      }
    }
  }

  std::string why = pairTable(distance, speed, next.speed_table);
  if (why.empty()) {
    why = next.validate();
  }
  if (why.empty() && (next_linear_accel <= 0.0 || next_angular_accel <= 0.0)) {
    why = "max_linear_accel and max_angular_accel must be > 0";
  }
  if (!why.empty()) {
    result.successful = false;
    result.reason = plugin_name_ + ": " + why;
    RCLCPP_WARN(
      logger_, "[ApproachController][%s] rejecting parameter update: %s", plugin_name_.c_str(),
      why.c_str());
    return result;
  }

  law_->setParams(next);
  max_linear_accel_ = next_linear_accel;
  max_angular_accel_ = next_angular_accel;
  transform_tolerance_ = next_transform_tolerance;
  result.successful = true;
  return result;
}

}  // namespace syncai_controller

// Register this controller as a syncai_nav_core plugin
PLUGINLIB_EXPORT_CLASS(syncai_controller::ApproachController, syncai_nav_core::Controller)
