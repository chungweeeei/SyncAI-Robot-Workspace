// Port of nav2_graceful_controller/smooth_control_law.cpp (Humble, 1.1.x),
// Copyright (c) 2023 Alberto J. Tudela Roldán, Apache-2.0.

#include "syncai_controller/plugins/docking_controller/smooth_control_law.hpp"

#include <algorithm>
#include <cmath>

#include "syncai_controller/plugins/docking_controller/ego_polar_coords.hpp"
#include "syncai_util/geometry_utils.hpp"

namespace syncai_controller
{

SmoothControlLaw::SmoothControlLaw(
  double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
  double v_linear_min, double v_linear_max, double v_angular_max)
: k_phi_(k_phi),
  k_delta_(k_delta),
  beta_(beta),
  lambda_(lambda),
  slowdown_radius_(slowdown_radius),
  v_linear_min_(v_linear_min),
  v_linear_max_(v_linear_max),
  v_angular_max_(v_angular_max)
{
}

void SmoothControlLaw::setCurvatureConstants(
  double k_phi, double k_delta, double beta, double lambda)
{
  k_phi_ = k_phi;
  k_delta_ = k_delta;
  beta_ = beta;
  lambda_ = lambda;
}

void SmoothControlLaw::setSlowdownRadius(double slowdown_radius)
{
  slowdown_radius_ = slowdown_radius;
}

void SmoothControlLaw::setSpeedLimit(
  const double v_linear_min, const double v_linear_max, const double v_angular_max)
{
  v_linear_min_ = v_linear_min;
  v_linear_max_ = v_linear_max;
  v_angular_max_ = v_angular_max;
}

geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & current,
  const bool & backward)
{
  // Convert the target to polar coordinates
  auto ego_coords = EgocentricPolarCoordinates(target, current, backward);
  // Calculate the curvature
  double curvature = calculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);
  // Invert the curvature if the robot is moving backwards
  curvature = backward ? -curvature : curvature;

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  double v = v_linear_max_ / (1.0 + beta_ * std::pow(std::fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  v = std::min(v_linear_max_ * (ego_coords.r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  v = std::clamp(v, v_linear_min_, v_linear_max_);

  // Set the velocity to negative if the robot is moving backwards
  v = backward ? -v : v;

  // Compute the angular velocity
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  // LOCAL (not upstream): the linear floor is re-applied AFTER the angular
  // bound. Upstream's line above shrinks v to w_bound / curvature so the
  // executed arc keeps exactly the law's curvature, and for a wheeled base a
  // slow tight arc is fine. On the quadruped v_linear_min is the gait
  // controller's walking floor: a forward command below it does not make the
  // robot walk slowly, it makes it stand (0.05 m/s under MPC, measured
  // 2026-10-14; ~0.3 m/s under RL -- see controller_server_params.yaml).
  // Under RL, at the curvature the law asks for when the hand-over from the
  // initial rotation happens 1 m out (|kappa| ~ 4), w_bound / kappa is
  // 0.65 / 4 ~ 0.16 m/s -- below the floor -- and the robot would stand
  // there with a non-zero command until the server's progress checker ended
  // the goal 30 s later. Under MPC that hand-over is above the floor and the
  // rule only bites on the tightest arcs. Holding v at the floor
  // instead executes a WIDER arc than the law wants (curvature w_bound / v
  // rather than kappa) and lets the feedback law correct it next cycle:
  // slightly less graceful, but moving. This is also why docking must start
  // from a staging pose roughly facing the target (dock.xml's job): the
  // wider the initial bearing error, the longer the law spends in this
  // regime. The sign is kept, so a reversing command stays negative.
  v = std::copysign(std::max(std::fabs(v), v_linear_min_), v);

  // Return the velocity command
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w_bound;
  return cmd_vel;
}

geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const bool & backward)
{
  return calculateRegularVelocity(target, geometry_msgs::msg::Pose(), backward);
}

geometry_msgs::msg::Pose SmoothControlLaw::calculateNextPose(
  const double dt, const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Pose & current, const bool & backward)
{
  geometry_msgs::msg::Twist vel = calculateRegularVelocity(target, current, backward);
  geometry_msgs::msg::Pose next;
  double yaw = tf2::getYaw(current.orientation);
  next.position.x = current.position.x + vel.linear.x * dt * std::cos(yaw);
  next.position.y = current.position.y + vel.linear.x * dt * std::sin(yaw);
  yaw += vel.angular.z * dt;
  next.orientation = syncai_util::geometry_utils::orientationAroundZAxis(yaw);
  return next;
}

double SmoothControlLaw::calculateCurvature(double r, double phi, double delta)
{
  // Calculate the proportional term of the control law as the product of the gain and the error:
  // difference between the actual steering angle and the virtual control for the slow subsystem
  double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  double feedback_term = (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * phi, 2)))) * std::sin(delta);
  // Calculate the path curvature
  return -1.0 / r * (prop_term + feedback_term);
}

}  // namespace syncai_controller
