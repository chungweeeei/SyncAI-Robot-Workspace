// DockingController: Park & Kuipers' smooth control law as a
// syncai_nav_core::Controller, ported from Humble's nav2_graceful_controller
// (Copyright (c) 2023 Alberto J. Tudela Roldán, Apache-2.0) with the path
// handling and the collision checking removed. See the class comment in the
// header for what this plugin is and, above all, for what it deliberately is
// not: it never reads the costmap.

#include "syncai_controller/plugins/docking_controller/docking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/robot_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using rcl_interfaces::msg::ParameterType;
using syncai_util::declare_parameter_if_not_declared;
using syncai_util::geometry_utils::euclidean_distance;
using syncai_util::geometry_utils::orientationAroundZAxis;

namespace syncai_controller
{

namespace
{
// nav2_msgs/SpeedLimit's "no limit" (upstream's nav2_costmap_2d::NO_SPEED_LIMIT).
constexpr double kNoSpeedLimit = 0.0;

// Horizon and step of the rviz preview (publishPreview()). Not parameters on
// purpose: the preview is a picture, the control law never reads it and
// nothing is checked against it, so there is nothing to tune.
constexpr double kPreviewSeconds = 3.0;
constexpr double kPreviewStep = 0.1;

// The operator's docking plan (publishDockingPlan()) runs the same
// integration to the dock instead of for 3 s: the console draws where the
// approach ends, and at the 0.05 m/s floor 3 s would stop 15 cm out. 60 s is
// the server's dock_approach_timeout, past which no approach survives anyway.
constexpr double kDockingPlanSeconds = 60.0;
// 2 Hz: the plan is a picture for a person, the backend thins it to 512
// points and the console rebuilds a mesh per message.
constexpr double kDockingPlanPeriod = 0.5;
}  // namespace

void DockingController::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros)
{
  // Frame names only. No getCostmap(), no collision checker: see the header.
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double control_frequency = 20.0;

  declare_parameter_if_not_declared(node, plugin_name_ + ".k_phi", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_delta", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".beta", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lambda", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(0.40));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(0.65));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".slowdown_radius", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_rotation_min_angle", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".final_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_scaling_factor", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_backward", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_docking_plan", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".docking_plan_frame", rclcpp::ParameterValue(std::string("map")));

  node->get_parameter(plugin_name_ + ".k_phi", k_phi_);
  node->get_parameter(plugin_name_ + ".k_delta", k_delta_);
  node->get_parameter(plugin_name_ + ".beta", beta_);
  node->get_parameter(plugin_name_ + ".lambda", lambda_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".slowdown_radius", slowdown_radius_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".initial_rotation", initial_rotation_);
  node->get_parameter(plugin_name_ + ".initial_rotation_min_angle", initial_rotation_min_angle_);
  node->get_parameter(plugin_name_ + ".final_rotation", final_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_scaling_factor", rotation_scaling_factor_);
  node->get_parameter(plugin_name_ + ".allow_backward", allow_backward_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".publish_docking_plan", publish_docking_plan_);
  node->get_parameter(plugin_name_ + ".docking_plan_frame", docking_plan_frame_);
  node->get_parameter("controller_frequency", control_frequency);

  control_duration_ = 1.0 / control_frequency;
  base_v_linear_max_ = v_linear_max_;

  // Upstream's rule: an initial rotation turns the nose toward the target,
  // which is the opposite of what a reversing approach wants.
  if (initial_rotation_ && allow_backward_) {
    RCLCPP_WARN(
      logger_,
      "[DockingController][%s] initial_rotation and allow_backward cannot both be true; "
      "disabling allow_backward",
      plugin_name_.c_str());
    allow_backward_ = false;
  }
  // The floor is physical (the gait), the cap is a preference: the cap moves.
  if (v_linear_min_ > v_linear_max_) {
    RCLCPP_WARN(
      logger_, "[DockingController][%s] v_linear_min %.2f > v_linear_max %.2f; raising the max",
      plugin_name_.c_str(), v_linear_min_, v_linear_max_);
    v_linear_max_ = v_linear_min_;
    base_v_linear_max_ = v_linear_max_;
  }
  if (v_angular_min_ > v_angular_max_) {
    RCLCPP_WARN(
      logger_, "[DockingController][%s] v_angular_min %.2f > v_angular_max %.2f; lowering the min",
      plugin_name_.c_str(), v_angular_min_, v_angular_max_);
    v_angular_min_ = v_angular_max_;
  }
  if (beta_ <= 0.0 || lambda_ < 1.0) {
    RCLCPP_WARN(
      logger_,
      "[DockingController][%s] beta must be > 0 and lambda >= 1 (got %.2f / %.2f); the law's "
      "speed-vs-curvature term is undefined outside that",
      plugin_name_.c_str(), beta_, lambda_);
  }

  control_law_ = std::make_unique<SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, v_linear_min_, v_linear_max_,
    v_angular_max_);

  preview_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("docking_target", 1);
  if (publish_docking_plan_) {
    // Same topic and QoS as controller_server's clear (kDockingPlanTopic
    // there): the server owns the "approach over" message, this plugin the
    // plan itself.
    docking_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("docking_plan", 1);
  }

  RCLCPP_INFO(
    logger_,
    "[DockingController][%s] v_linear [%.2f, %.2f] m/s (pre-scale; the min is the gait floor), "
    "v_angular [%.2f, %.2f] rad/s, slowdown_radius %.2f m, initial_rotation %s (> %.2f rad), "
    "final_rotation %s, allow_backward %s. Obstacle avoidance is OFF: this controller does not "
    "read the local costmap.",
    plugin_name_.c_str(), v_linear_min_, v_linear_max_, v_angular_min_, v_angular_max_,
    slowdown_radius_, initial_rotation_ ? "on" : "off", initial_rotation_min_angle_,
    final_rotation_ ? "on" : "off", allow_backward_ ? "on" : "off");

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingController::dynamicParametersCallback, this, std::placeholders::_1));
}

void DockingController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw syncai_nav_core::PlannerException("DockingController: received an empty path");
  }

  // The last pose is the target; everything before it is the server telling
  // rviz where the approach started. Do not assume there is more than one.
  geometry_msgs::msg::PoseStamped dock = path.poses.back();
  if (!path.header.frame_id.empty()) {
    dock.header.frame_id = path.header.frame_id;
  }

  // Track in the costmap's global frame (odom) and nothing else. The server
  // already transforms the goal there once at acceptance, so for dock_to_pose
  // this branch is a no-op; it exists so the plugin is also correct when
  // driven through follow_path with a map-frame path (controller_id:
  // Docking), and so the "transform once, then odom only" rule lives in the
  // plugin rather than in the trust it puts in its caller.
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  if (dock.header.frame_id != global_frame) {
    dock.header.stamp = rclcpp::Time(0);
    geometry_msgs::msg::PoseStamped transformed;
    if (!syncai_util::transformPoseInTargetFrame(
          dock, transformed, *tf_, global_frame, transform_tolerance_)) {
      throw syncai_nav_core::PlannerException(
        "DockingController: cannot transform the dock pose from '" + dock.header.frame_id +
        "' into '" + global_frame + "'");
    }
    dock = transformed;
  }

  dock_pose_ = dock;
  have_dock_pose_ = true;
  // New target, new decision: the latch belongs to the pose it was taken on.
  final_rotation_latched_ = false;
  // And a new picture at once rather than up to half a period late.
  have_last_docking_plan_ = false;
  last_branch_ = Branch::None;
  // last_cmd_vel_ deliberately survives, RPP's rule: setPlan() is also the
  // preempt path (a dock goal re-sent with a corrected pose arrives here
  // mid-approach), and zeroing the acceleration-clamp baseline on a preempt
  // collapses the command to one accel step and re-ramps it -- the cmd_vel
  // sawtooth RPP measured. Per-goal zeroing lives in reset().
}

void DockingController::reset()
{
  // New goal (or a cycle without a robot pose, on which the server commands
  // zero): the next command ramps up from a standstill. The final-rotation
  // latch is NOT touched here -- see the header.
  last_cmd_vel_ = geometry_msgs::msg::Twist();
}

geometry_msgs::msg::TwistStamped DockingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*velocity*/,
  syncai_nav_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    return computeVelocityCommandsImpl(pose, goal_checker);
  } catch (...) {
    // Every throw means this cycle commands a stop (the server publishes zero
    // under failure_tolerance and on the way out of the goal), so the
    // acceleration-clamp baseline follows the stop rather than the command
    // we abandoned halfway through; otherwise the first cycle after the
    // all-clear would step from 0 to the stale speed in 50 ms. Same design
    // and the same numbers as RPP's wrapper.
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    throw;
  }
}

geometry_msgs::msg::TwistStamped DockingController::computeVelocityCommandsImpl(
  const geometry_msgs::msg::PoseStamped & pose, syncai_nav_core::GoalChecker * goal_checker)
{
  if (!have_dock_pose_) {
    throw syncai_nav_core::PlannerException("DockingController: no dock pose set");
  }
  // Both poses must be in the same (odom) frame for the 2D math below to mean
  // anything; the server guarantees it, and this is the one place a frame
  // mix-up could otherwise hide.
  if (pose.header.frame_id != dock_pose_.header.frame_id) {
    throw syncai_nav_core::PlannerException(
      "DockingController: robot pose is in '" + pose.header.frame_id + "' but the dock pose in '" +
      dock_pose_.header.frame_id + "'");
  }

  // The goal checker's xy tolerance is where the final rotation starts, so
  // the two latches (ours and the stateful checker's) trip on the same number.
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Target in the robot's base frame by plain 2D math -- no TF per cycle.
  // Both poses are odom by construction, and odom is continuous, so a
  // localizer correction (map -> odom jumping under a relocalize) never moves
  // this target: that is the whole reason the server transforms once.
  const double yaw_r = tf2::getYaw(pose.pose.orientation);
  const double cos_r = std::cos(yaw_r);
  const double sin_r = std::sin(yaw_r);
  const double dx = dock_pose_.pose.position.x - pose.pose.position.x;
  const double dy = dock_pose_.pose.position.y - pose.pose.position.y;
  geometry_msgs::msg::Pose target;
  target.position.x = cos_r * dx + sin_r * dy;
  target.position.y = -sin_r * dx + cos_r * dy;
  const double yaw_to_dock =
    angles::normalize_angle(tf2::getYaw(dock_pose_.pose.orientation) - yaw_r);
  const double r = std::hypot(dx, dy);
  const double angle_to_target = std::atan2(target.position.y, target.position.x);

  // Reversing when allowed and the target is behind: the law then steers the
  // tail at the target. The target's orientation goes in UNflipped, on
  // purpose: EgocentricPolarCoordinates flips the line of sight by pi when
  // backward, and that one flip already gives the tail's view of both phi
  // and delta. Until 2026-10-15 the yaw was flipped here as well (graceful's
  // computeVelocityCommands does), which put phi pi off: with the target
  // dead astern at the robot's own yaw the law read phi = pi and commanded
  // w ~ 0.2 rad/s instead of 0, and the heading error then grew under it.
  // On robot01 both undocks veered ~0.5 rad in 5 s, slid the target to the
  // robot's side (x >= 0, so the law went forward), turned ~180 deg round it
  // and arrived 2.3 rad off the dock yaw. A closed-loop simulation of this
  // code reproduces that from a perfect start; unflipped, the same
  // simulation backs straight out 1 m in 5.3 s. So the Undocking instance's
  // 1 m back-out target gives r = 1, phi = 0, delta = 0 -> a straight
  // reverse. Overshoot puts the target ahead and the law drives forward to
  // fix it; allow_backward permits reversing, it does not demand it.
  const bool backward = allow_backward_ && target.position.x < 0.0;
  target.orientation = orientationAroundZAxis(yaw_to_dock);

  if (target_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.frame_id = costmap_ros_->getBaseFrameID();
    target_msg.header.stamp = clock_->now();
    target_msg.pose = target;
    target_pub_->publish(target_msg);
  }

  // Three branches, upstream's order:
  // 1. inside the xy window (or once inside it): rotate in place to the dock
  //    yaw, so the robot ends facing the way the pose says whichever way it
  //    arrived;
  // 2. the target is far off the nose: rotate in place toward it first,
  //    because the law converges from any pose but spends the wide-bearing
  //    part of that convergence in the arc the gait floor executes badly;
  // 3. otherwise the smooth control law.
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  bool instant_linear_stop = false;
  Branch branch;
  if (final_rotation_ && (r < goal_dist_tol_ || final_rotation_latched_)) {
    final_rotation_latched_ = true;
    cmd_vel.twist = rotateToTarget(yaw_to_dock);
    instant_linear_stop = true;
    branch = Branch::FinalRotation;
  } else if (initial_rotation_ && std::fabs(angle_to_target) > initial_rotation_min_angle_) {
    cmd_vel.twist = rotateToTarget(angle_to_target);
    branch = Branch::InitialRotation;
  } else {
    cmd_vel.twist = control_law_->calculateRegularVelocity(target, backward);
    branch = Branch::ControlLaw;
  }

  if (branch != last_branch_) {
    switch (branch) {
      case Branch::FinalRotation:
        RCLCPP_INFO(
          logger_,
          "[DockingController][%s] %.3f m from the dock (xy tolerance %.3f): rotating in place "
          "to its yaw, %+.3f rad to go",
          plugin_name_.c_str(), r, goal_dist_tol_, yaw_to_dock);
        break;
      case Branch::InitialRotation:
        RCLCPP_INFO(
          logger_,
          "[DockingController][%s] %.3f m from the dock, %+.3f rad off the nose (> %.2f): "
          "rotating in place toward it",
          plugin_name_.c_str(), r, angle_to_target, initial_rotation_min_angle_);
        break;
      case Branch::ControlLaw:
        RCLCPP_INFO(
          logger_,
          "[DockingController][%s] %.3f m from the dock, %+.3f rad off the nose: following the "
          "control law%s",
          plugin_name_.c_str(), r, angle_to_target, backward ? " backwards" : "");
        break;
      case Branch::None:
        break;
    }
    last_branch_ = branch;
  }

  clampAcceleration(cmd_vel.twist, instant_linear_stop);

  publishPreview(pose, target, backward);
  publishDockingPlan(pose, target, backward);

  // Remember what we actually commanded: next cycle's clamp baseline.
  last_cmd_vel_ = cmd_vel.twist;
  return cmd_vel;
}

geometry_msgs::msg::Twist DockingController::rotateToTarget(double angle_to_target) const
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  // Upstream: proportional, w = factor * angle * w_max, which at the 0.05 rad
  // yaw error of the final alignment is 0.016 rad/s -- nothing a gait turns
  // at. The floor is the local addition: the robot turns at least
  // v_angular_min until the goal checker says the yaw is in. 0.15 rad/s
  // since 2026-10-15; it was 0.03, from a 2026-10-14 MPC measurement that a
  // sustained in-place turn did not bear out -- both robot01 undocks stalled
  // at 0.09 rad/s with the yaw frozen until the progress checker aborted.
  // 0.15 is 7.5 mrad per 50 ms cycle, which is why dock_goal_checker's yaw
  // window went 0.05 -> 0.1 with it. No sqrt(2 * accel * angle) deceleration cap
  // (RPP's rotateToHeading has one): a cap that tends to zero would fight
  // the floor exactly where the floor is the point.
  const double w = rotation_scaling_factor_ * angle_to_target * v_angular_max_;
  const double magnitude = std::clamp(std::fabs(w), v_angular_min_, v_angular_max_);
  vel.angular.z = std::copysign(magnitude, angle_to_target);
  return vel;
}

void DockingController::clampAcceleration(
  geometry_msgs::msg::Twist & cmd, bool allow_instant_linear_stop) const
{
  // RPP's design, same reasons: the baseline is our previous COMMAND, not the
  // measured twist (gait sway), and there is no velocity smoother in the
  // stack, so the controller owns kinematic feasibility. The first command
  // after reset() ramps 0 -> v_linear_min over v_linear_min / (accel * dt)
  // cycles (one at the MPC floor of 0.05 m/s; six, ~0.3 s of standing, at
  // RL's 0.30).
  const double dv = max_linear_accel_ * control_duration_;
  const double dw = max_angular_accel_ * control_duration_;

  // The one exception, local: the stop into the final rotation is NOT
  // ramped. At the MPC floor (0.05 m/s) a ramp-down is one cycle anyway; the
  // rule is for a floor raised again -- at RL's 0.30 m/s a 1.0 m/s^2
  // ramp-down takes six cycles, ~4.5 cm of travel after the xy latch, the
  // entire 5 cm window the latch was taken on. A one-step zero is what publishZeroVelocity()
  // already sends at every goal end and on cancel, so the gait controller
  // sees nothing it has not seen before. The angular clamp stays on in
  // every branch (the hand-over out of a 0.65 rad/s rotation is RPP's
  // "cliff").
  if (!(allow_instant_linear_stop && cmd.linear.x == 0.0)) {
    cmd.linear.x =
      std::clamp(cmd.linear.x, last_cmd_vel_.linear.x - dv, last_cmd_vel_.linear.x + dv);
  }
  cmd.angular.z =
    std::clamp(cmd.angular.z, last_cmd_vel_.angular.z - dw, last_cmd_vel_.angular.z + dw);
}

void DockingController::publishPreview(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Pose & target_in_base, bool backward)
{
  if (preview_pub_->get_subscription_count() == 0) {
    return;
  }

  // The picture to tune k_phi / k_delta by; the law never reads it and
  // nothing is checked against it.
  nav_msgs::msg::Path path;
  path.header.frame_id = robot_pose.header.frame_id;
  path.header.stamp = clock_->now();
  integrateApproach(robot_pose, target_in_base, backward, kPreviewSeconds, path);
  preview_pub_->publish(path);
}

void DockingController::integrateApproach(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Pose & target_in_base, bool backward, double horizon,
  nav_msgs::msg::Path & path) const
{
  // Integrate the law from the robot (the origin of the base frame) and map
  // each pose back into the robot pose's frame by the inverse of the rotation
  // used in computeVelocityCommandsImpl().
  const double yaw_r = tf2::getYaw(robot_pose.pose.orientation);
  const double cos_r = std::cos(yaw_r);
  const double sin_r = std::sin(yaw_r);
  auto to_frame = [&](const geometry_msgs::msg::Pose & in_base) {
    geometry_msgs::msg::PoseStamped out;
    out.header = path.header;
    out.pose.position.x =
      robot_pose.pose.position.x + cos_r * in_base.position.x - sin_r * in_base.position.y;
    out.pose.position.y =
      robot_pose.pose.position.y + sin_r * in_base.position.x + cos_r * in_base.position.y;
    out.pose.orientation = orientationAroundZAxis(yaw_r + tf2::getYaw(in_base.orientation));
    return out;
  };

  geometry_msgs::msg::Pose next;
  next.orientation.w = 1.0;
  path.poses.push_back(to_frame(next));
  const int steps = static_cast<int>(horizon / kPreviewStep);
  for (int i = 0; i < steps; ++i) {
    next = control_law_->calculateNextPose(kPreviewStep, target_in_base, next, backward);
    path.poses.push_back(to_frame(next));
    if (euclidean_distance(target_in_base, next) < 0.01) {
      break;
    }
  }
}

void DockingController::publishDockingPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Pose & target_in_base, bool backward)
{
  if (!docking_plan_pub_) {
    return;
  }
  const rclcpp::Time now = clock_->now();
  if (have_last_docking_plan_ && (now - last_docking_plan_time_).seconds() < kDockingPlanPeriod) {
    return;
  }

  // Not gated on subscribers, unlike the rviz preview: the backend subscribes
  // for the whole life of the robot, so the gate would never close, and a
  // console that connects mid-approach should not wait for it to open.
  //
  // The map frame, not odom: the backend forwards map-frame paths only and
  // the console has no other frame. The plan is integrated in odom (where the
  // dock is tracked) and moved with ONE map <- odom lookup per message, so a
  // relocalize shifts the picture, as it shifts the robot on screen, and
  // never the target the law steers to.
  geometry_msgs::msg::TransformStamped odom_to_plan;
  try {
    odom_to_plan = tf_->lookupTransform(
      docking_plan_frame_, robot_pose.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[DockingController][%s] docking plan not published: %s",
      plugin_name_.c_str(), e.what());
    return;
  }

  nav_msgs::msg::Path in_odom;
  in_odom.header = robot_pose.header;
  integrateApproach(robot_pose, target_in_base, backward, kDockingPlanSeconds, in_odom);

  nav_msgs::msg::Path path;
  path.header.frame_id = docking_plan_frame_;
  path.header.stamp = now;
  path.poses.reserve(in_odom.poses.size());
  for (const auto & pose : in_odom.poses) {
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(pose, out, odom_to_plan);
    out.header = path.header;
    path.poses.push_back(out);
  }
  docking_plan_pub_->publish(path);
  last_docking_plan_time_ = now;
  have_last_docking_plan_ = true;
}

void DockingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (speed_limit == kNoSpeedLimit) {
    v_linear_max_ = base_v_linear_max_;
  } else if (percentage) {
    v_linear_max_ = base_v_linear_max_ * speed_limit / 100.0;
  } else {
    v_linear_max_ = speed_limit;
  }

  // A limit below the gait floor cannot be honoured: a command below the
  // floor is a stall, not a slow approach, and a silent stall is worse than
  // an ignored limit.
  if (v_linear_max_ < v_linear_min_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[DockingController][%s] speed limit %.2f m/s is below the gait floor v_linear_min %.2f; "
      "driving at the floor",
      plugin_name_.c_str(), v_linear_max_, v_linear_min_);
    v_linear_max_ = v_linear_min_;
  }
  control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
}

rcl_interfaces::msg::SetParametersResult DockingController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".k_phi") {
        k_phi_ = parameter.as_double();
      } else if (name == plugin_name_ + ".k_delta") {
        k_delta_ = parameter.as_double();
      } else if (name == plugin_name_ + ".beta") {
        beta_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lambda") {
        lambda_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        if (parameter.as_double() > v_linear_max_) {
          RCLCPP_WARN(
            logger_, "[DockingController][%s] v_linear_min above v_linear_max; rejecting update",
            plugin_name_.c_str());
          continue;
        }
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        if (parameter.as_double() < v_linear_min_) {
          RCLCPP_WARN(
            logger_, "[DockingController][%s] v_linear_max below v_linear_min; rejecting update",
            plugin_name_.c_str());
          continue;
        }
        v_linear_max_ = parameter.as_double();
        base_v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".slowdown_radius") {
        slowdown_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_linear_accel") {
        max_linear_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".initial_rotation_min_angle") {
        initial_rotation_min_angle_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_scaling_factor") {
        rotation_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".initial_rotation") {
        if (parameter.as_bool() && allow_backward_) {
          RCLCPP_WARN(
            logger_,
            "[DockingController][%s] initial_rotation and allow_backward cannot both be true; "
            "rejecting update",
            plugin_name_.c_str());
          continue;
        }
        initial_rotation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".final_rotation") {
        final_rotation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".allow_backward") {
        if (parameter.as_bool() && initial_rotation_) {
          RCLCPP_WARN(
            logger_,
            "[DockingController][%s] initial_rotation and allow_backward cannot both be true; "
            "rejecting update",
            plugin_name_.c_str());
          continue;
        }
        allow_backward_ = parameter.as_bool();
      }
    }
  }

  control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
  control_law_->setSlowdownRadius(slowdown_radius_);
  control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);

  result.successful = true;
  return result;
}

}  // namespace syncai_controller

// Register this controller as a syncai_nav_core plugin
PLUGINLIB_EXPORT_CLASS(syncai_controller::DockingController, syncai_nav_core::Controller)
