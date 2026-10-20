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

#include "syncai_mppi_controller/controller.hpp"

#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>

#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_nav_core/exceptions.hpp"
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
  // Not upstream; see applyWalkFloor(). walk_vx_min <= 0 disables it.
  getParam(walk_vx_min_, "walk_vx_min", 0.0);
  // Not upstream; see applyTurnFloor(). in_place_wz_min <= 0 disables it.
  getParam(in_place_wz_min_, "in_place_wz_min", 0.0);
  getParam(in_place_wz_deadband_, "in_place_wz_deadband", 0.05);
  getParam(in_place_vx_max_, "in_place_vx_max", 0.1);
  // Not upstream; see rotateToGoalHeading(). final_rotation_wz <= 0 disables it.
  getParam(final_rotation_wz_, "final_rotation_wz", 0.0);
  // Not upstream; see approachGoal(). approach_dist <= 0 disables it, and so
  // does walk_vx_min or final_rotation_wz <= 0: it walks at the one and hands
  // over to the other.
  getParam(approach_dist_, "approach_dist", 0.0);
  getParam(approach_rotate_min_angle_, "approach_rotate_min_angle", 0.25);
  // Not upstream; see climbStairs(). The detection and switch parameters
  // (use_stair_mode, stair_min_rise, ...) are StairMode's, declared under this
  // plugin's name; these two are the climb's.
  stair_mode_.initialize(node, name_);
  getParam(stair_linear_vel_, "stair_linear_vel", 0.5);
  getParam(stair_lookahead_dist_, "stair_lookahead_dist", 0.4);
  // The server's own parameter, read the way the Optimizer reads it.
  auto getParentParam = parameters_handler_->getParamGetter("");
  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 20.0, ParameterType::Static);
  control_duration_ = 1.0 / controller_frequency;

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  collision_checker_.setCostmap(costmap_ros_->getCostmap());
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
  rotating_to_goal_ = false;
  approaching_goal_ = false;
  // Stair mode's on_stairs survives (it is the policy the gait controller is
  // on); a half-done switch is settled or dropped (StairMode::reset()).
  stair_mode_.reset();
  climbing_stairs_ = false;
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  syncai_nav_core::GoalChecker * goal_checker)
{
  try {
    geometry_msgs::msg::TwistStamped cmd;
    // Stairs first, before the final rotation: a goal on the landing at the
    // top of a flight has the robot stop and switch its policy back before it
    // turns onto the goal heading, as RPP 3D does. A plan without heights
    // never enters this branch.
    if (climbStairs(robot_pose, cmd)) {
      last_cmd_vel_ = cmd.twist;
      return cmd;
    }
    // Before the optimizer, and instead of it: this branch does its own
    // clamping and flooring, which the three steps below would undo.
    if (rotateToGoalHeading(robot_pose, goal_checker, cmd)) {
      last_cmd_vel_ = cmd.twist;
      return cmd;
    }
    // So does a direct approach already under way, for the same reason. One
    // whose way has become blocked drops its latch and falls through to the
    // optimizer.
    if (approaching_goal_ && approachGoal(robot_pose, goal_checker, cmd)) {
      last_cmd_vel_ = cmd.twist;
      return cmd;
    }
    cmd = computeVelocityCommandsImpl(robot_pose, robot_speed, goal_checker);
    // The optimizer asking to stand (or to turn in place) is what opens the
    // approach, read off its raw output -- the same |vx| < in_place_vx_max
    // that the walking floor leaves alone, which is exactly the command that
    // stalled. The cycle's optimizer output is then discarded.
    if (
      std::fabs(cmd.twist.linear.x) < in_place_vx_max_ &&
      approachGoal(robot_pose, goal_checker, cmd)) {
      last_cmd_vel_ = cmd.twist;
      return cmd;
    }
    applyWalkFloor(cmd.twist);
    clampAcceleration(cmd.twist);
    applyTurnFloor(cmd.twist);
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

void MPPIController::applyWalkFloor(geometry_msgs::msg::Twist & cmd) const
{
  // The gait controller under RL does not walk below a floor either: on
  // robot01 (2026-10-15) a 0.13 m/s command left the robot standing. MPPI
  // slows down near the goal and around blockers, so it lands in that band
  // routinely -- the goal_checker stalls 0.23-0.29 m short with cmd_vel
  // non-zero and odom zero were this. Same shape as applyTurnFloor(): |vx|
  // under in_place_vx_max is MPPI asking to stand (or to turn in place) and
  // is left alone; between it and walk_vx_min it is raised to the floor,
  // sign kept.
  //
  // Applied to the optimizer's output BEFORE the acceleration clamp, unlike
  // the turn floor, because the decision has to be made on what MPPI wants,
  // not on the ramp: after the clamp, a stop from the floor (0.30 -> 0.25
  // in one cycle) would land in the band and be raised straight back to
  // 0.30, so the robot could never stop. Before it, a stop is an output under
  // in_place_vx_max and ramps down untouched. The cost is the ramp out of
  // rest: the clamp still walks the command up from zero at
  // max_linear_accel x dt, so the first ~0.3 s of every start are under the
  // floor -- RPP's ramp has the same dead time, and RPP is what has driven
  // the robot.
  if (walk_vx_min_ <= 0.0) {
    return;
  }
  const double vx = std::fabs(cmd.linear.x);
  if (vx >= in_place_vx_max_ && vx < walk_vx_min_) {
    cmd.linear.x = std::copysign(walk_vx_min_, cmd.linear.x);
  }
}

void MPPIController::applyTurnFloor(geometry_msgs::msg::Twist & cmd) const
{
  // The gait controller under RL does not turn in place below a floor: on
  // robot01 MPPI held -0.115 rad/s at the goal, odom read zero, and the robot
  // stood there until the progress checker aborted 30 s later (see
  // GoalAngleCritic in controller_server_params.yaml). MPPI tapers its turn
  // as the heading error closes, so near the goal yaw it lands under the
  // floor every time. A VelocityDeadbandCritic would score below-floor
  // samples down, but it penalises zero too, so it would fight stopping.
  //
  // Only in place (|vx| < in_place_vx_max): while walking, a small wz is a
  // gentle arc, and raising it to the floor would weave a straight line.
  // Below the deadband the command stays as it is -- that is MPPI's noise
  // around "do not turn", not a turn it wants. Applied after the
  // acceleration clamp on purpose: the clamp ramps out of rest at
  // max_angular_accel x dt (0.16 rad/s a cycle), under the floor, and the
  // floor has to win or the first cycle of every turn is a no-op. That one
  // step is the only place it exceeds max_angular_accel. The optimizer is
  // not told, as with the clamp: its next cycle starts from measured speed.
  if (in_place_wz_min_ <= 0.0 || std::fabs(cmd.linear.x) >= in_place_vx_max_) {
    return;
  }
  const double wz = std::fabs(cmd.angular.z);
  if (wz >= in_place_wz_deadband_ && wz < in_place_wz_min_) {
    cmd.angular.z = std::copysign(in_place_wz_min_, cmd.angular.z);
  }
}

bool MPPIController::rotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & robot_pose, syncai_nav_core::GoalChecker * goal_checker,
  geometry_msgs::msg::TwistStamped & cmd)
{
  // Not upstream (2026-10-15): the final turn to the goal heading is not left
  // to the optimizer. Inside the xy tolerance the stateful goal checker has
  // already accepted the position, but MPPI does not know that: GoalCritic
  // keeps pulling the robot onto the exact goal point while GoalAngleCritic
  // turns it to the goal yaw, and with no reversing (vx_min 0) the two pull
  // against each other. The output mixed a small forward vx with a small wz
  // -- 0.1 rad/s on robot01 -- and applyTurnFloor() only acts with |vx| under
  // in_place_vx_max, so the turn went out under the gait's floor, came in
  // jerks, and the progress checker (0.1 m in 30 s; a turn in place is no
  // progress) aborted the goal with the robot standing on it. This is RPP's
  // rotate-to-goal-heading instead, with its numbers: vx ramps down to 0 at
  // max_linear_accel, wz heads for final_rotation_wz at max_angular_accel,
  // capped at sqrt(2 * alpha * theta) to stop on the heading, and lifted to
  // in_place_wz_min last so neither the ramp nor the cap goes under the floor.
  //
  // Latched like the stateful goal checker: a quadruped turning in place
  // drifts a few centimetres, and handing a robot that drifted out of the
  // tolerance back to the optimizer would restart the tug of war. The goal is
  // the end of the path MPPI is tracking (the pruned path's last pose, as for
  // its own goal critics), which is also the goal checker's end pose.
  if (final_rotation_wz_ <= 0.0 || goal_checker == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  const nav_msgs::msg::Path plan = path_handler_.transformPath(robot_pose);
  const geometry_msgs::msg::Pose & goal = plan.poses.back().pose;

  if (!rotating_to_goal_) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      return false;
    }
    const double dist = std::hypot(
      goal.position.x - robot_pose.pose.position.x, goal.position.y - robot_pose.pose.position.y);
    if (dist >= pose_tolerance.position.x) {
      return false;
    }
    rotating_to_goal_ = true;
    // The direct approach, if it brought the robot here, is done: its walk
    // ends at this tolerance by design (see approachGoal()).
    approaching_goal_ = false;
    // The warm-started sequence is about driving to the goal point; should a
    // new path hand the robot back, the optimizer starts from scratch.
    optimizer_.reset();
    RCLCPP_INFO(
      logger_, "Within %.2f m of the goal (xy tolerance %.2f m): rotating in place to its heading",
      dist, pose_tolerance.position.x);
  }
  // The optimizer is not running, so keep computeVelocityCommandsImpl()'s
  // reset_period from firing on the hand-back: its reset() would zero
  // last_cmd_vel_, the clamp's baseline, under a robot still turning.
  last_time_called_ = clock_->now();

  const double angle = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(goal.orientation));
  double vx = 0.0;
  double wz = 0.0;
  rotateInPlace(angle, vx, wz);

  cmd = utils::toTwistStamped(vx, wz, robot_pose.header.stamp, costmap_ros_->getBaseFrameID());
  return true;
}

void MPPIController::rotateInPlace(double angle, double & vx, double & wz) const
{
  // RPP's rotateToHeading() with RPP's numbers; see rotateToGoalHeading() for
  // why each step is where it is. The floor goes last so that neither the
  // ramp nor the stopping cap can leave the turn under it.
  const double sign = angle >= 0.0 ? 1.0 : -1.0;
  wz = sign * final_rotation_wz_;
  vx = 0.0;
  if (max_angular_accel_ > 0.0) {
    const double dw = max_angular_accel_ * control_duration_;
    wz = std::clamp(wz, last_cmd_vel_.angular.z - dw, last_cmd_vel_.angular.z + dw);
    const double wz_to_stop = std::sqrt(2.0 * max_angular_accel_ * std::fabs(angle));
    if (std::fabs(wz) > wz_to_stop) {
      wz = sign * wz_to_stop;
    }
  }
  if (std::fabs(wz) < in_place_wz_min_) {
    wz = sign * in_place_wz_min_;
  }
  if (max_linear_accel_ > 0.0) {
    const double dv = max_linear_accel_ * control_duration_;
    vx = std::clamp(0.0, last_cmd_vel_.linear.x - dv, last_cmd_vel_.linear.x + dv);
  }
}

bool MPPIController::approachGoal(
  const geometry_msgs::msg::PoseStamped & robot_pose, syncai_nav_core::GoalChecker * goal_checker,
  geometry_msgs::msg::TwistStamped & cmd)
{
  // Not upstream (2026-10-16): the last few decimetres onto the goal point
  // are not left to the optimizer once it asks to stand short of them. On
  // robot01 the robot stopped 0.19 m from the goal -- outside
  // xy_goal_tolerance, so the final rotation never opened -- with the goal
  // 90 deg to its left and its heading 2.5 rad off. MPPI held vx 0.02-0.10
  // with wz either raised to the turn floor or inside its deadband, odom
  // read zero, and the progress checker aborted the goal twice before it was
  // cancelled; the goal before it had stalled the same way through three
  // aborts until the robot happened to drift inside the tolerance. The
  // walking floor does not touch that: under in_place_vx_max it is MPPI
  // asking to stand, which it was.
  //
  // It was asking to stand because nothing pays it to move. Inside 0.5 m
  // only GoalCritic, CostCritic and ConstraintCritic still score, and
  // GoalCritic is the mean distance to the goal over the 2 s horizon: for a
  // point d straight ahead the best constant speed is d * sqrt(2) / 2 s,
  // under in_place_vx_max inside ~0.14 m, and a point off to the side of a
  // DiffDrive robot that cannot reverse scores lower still. RPP never had
  // this, because past its rotate-to-heading it drives a fixed speed with
  // min_approach_linear_velocity as the floor -- which is what this does:
  // turn in place onto the goal point while its bearing is over
  // approach_rotate_min_angle (RPP's rotate-to-path), else walk at
  // walk_vx_min along the pure-pursuit arc through it, until the final
  // rotation takes over at xy_goal_tolerance.
  //
  // Opened only when the optimizer's own output would have left the robot
  // standing (the caller tests that), not on the distance alone, so every
  // approach MPPI manages by itself -- and its obstacle avoidance with it --
  // is unchanged. That avoidance is CostCritic's, which is given up here, so
  // the turn and the walk are first checked against LETHAL as RPP checks
  // its arc, every cycle; blocked, the latch drops and the optimizer has
  // the robot again, which is the behaviour before this existed (stand,
  // progress checker, the BT's retry). Rejected: widening
  // xy_goal_tolerance (0.3 for a day on 2026-10-14 ended an ordinary MOVE
  // too far from its goal), and lowering in_place_vx_max so the walking
  // floor raises the stall to 0.30 (it raises a turn in place too, and
  // walking straight on moves the robot away from a goal beside it).
  //
  // Latched like the final rotation, for the same reason: the turn drifts
  // a few centimetres and must not hand the robot back to the optimizer
  // across approach_dist, where it would stall again.
  if (
    approach_dist_ <= 0.0 || walk_vx_min_ <= 0.0 || final_rotation_wz_ <= 0.0 ||
    goal_checker == nullptr) {
    return false;
  }
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    approaching_goal_ = false;
    return false;
  }

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  // Same goal as the final rotation's: the end of the path MPPI tracks.
  const nav_msgs::msg::Path plan = path_handler_.transformPath(robot_pose);
  const geometry_msgs::msg::Pose & goal = plan.poses.back().pose;
  const double x = robot_pose.pose.position.x;
  const double y = robot_pose.pose.position.y;
  const double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double dx = goal.position.x - x;
  const double dy = goal.position.y - y;
  // At least xy_goal_tolerance: rotateToGoalHeading() ran first and would
  // have latched inside it.
  const double dist = std::hypot(dx, dy);
  if (!approaching_goal_ && dist >= approach_dist_) {
    return false;
  }
  const double bearing = angles::shortest_angular_distance(yaw, std::atan2(dy, dx));

  // The walk ends where the stateful goal checker accepts the position and
  // the final rotation starts. Checking on to the goal point itself would
  // test the footprint there at the approach heading, which can cross a wall
  // that the goal heading runs along.
  const double walk = std::max(0.0, dist - pose_tolerance.position.x);
  if (isApproachBlocked(x, y, yaw, walk, bearing)) {
    if (approaching_goal_) {
      approaching_goal_ = false;
      RCLCPP_WARN(
        logger_, "Direct approach to the goal blocked %.2f m out: handing back to the optimizer",
        dist);
    } else {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 5000,
        "Optimizer asks to stand %.2f m from the goal, but the way onto it is blocked", dist);
    }
    return false;
  }

  if (!approaching_goal_) {
    approaching_goal_ = true;
    // As for the final rotation: should the robot be handed back, the
    // optimizer starts from scratch rather than from the sequence that stalled.
    optimizer_.reset();
    RCLCPP_INFO(
      logger_,
      "Optimizer asks to stand %.2f m from the goal (xy tolerance %.2f m): approaching it "
      "directly, %+.2f rad off its bearing",
      dist, pose_tolerance.position.x, bearing);
  }
  // As in rotateToGoalHeading(): the optimizer is not running, so keep its
  // reset_period from zeroing the clamp baseline on a hand-back.
  last_time_called_ = clock_->now();

  double vx = 0.0;
  double wz = 0.0;
  if (std::fabs(bearing) > approach_rotate_min_angle_) {
    rotateInPlace(bearing, vx, wz);
  } else {
    trackPoint(bearing, dist, walk_vx_min_, vx, wz);
  }

  cmd = utils::toTwistStamped(vx, wz, robot_pose.header.stamp, costmap_ros_->getBaseFrameID());
  return true;
}

void MPPIController::trackPoint(
  double bearing, double dist, double speed, double & vx, double & wz) const
{
  // Out of a turn this ramps up from rest at max_linear_accel, under the
  // floor for its first ~0.3 s, exactly as RPP's ramp does. The arc is
  // RPP's curvature to its carrot with the point as the carrot:
  // k = 2 * lateral / dist^2, lateral = dist * sin(bearing). Aimed at the
  // point itself, its bearing shrinks as the robot closes in, so the walk
  // does not hunt back into the turn. wz from the clamped vx, as RPP does,
  // so the arc keeps its curvature through the ramp; capped at the turn
  // rate (= wz_max), then clamped like every other command.
  vx = speed;
  if (max_linear_accel_ > 0.0) {
    const double dv = max_linear_accel_ * control_duration_;
    vx = std::clamp(vx, last_cmd_vel_.linear.x - dv, last_cmd_vel_.linear.x + dv);
  }
  wz = std::clamp(vx * 2.0 * std::sin(bearing) / dist, -final_rotation_wz_, final_rotation_wz_);
  if (max_angular_accel_ > 0.0) {
    const double dw = max_angular_accel_ * control_duration_;
    wz = std::clamp(wz, last_cmd_vel_.angular.z - dw, last_cmd_vel_.angular.z + dw);
  }
}

void MPPIController::brake(double & vx, double & wz) const
{
  // RPP 3D's stair hold: to a stop at the clamps' own rates, so the policy
  // switch is made standing still and the stop is as smooth as any other.
  vx = 0.0;
  wz = 0.0;
  if (max_linear_accel_ > 0.0) {
    const double dv = max_linear_accel_ * control_duration_;
    vx = std::clamp(0.0, last_cmd_vel_.linear.x - dv, last_cmd_vel_.linear.x + dv);
  }
  if (max_angular_accel_ > 0.0) {
    const double dw = max_angular_accel_ * control_duration_;
    wz = std::clamp(0.0, last_cmd_vel_.angular.z - dw, last_cmd_vel_.angular.z + dw);
  }
}

bool MPPIController::climbStairs(
  const geometry_msgs::msg::PoseStamped & robot_pose, geometry_msgs::msg::TwistStamped & cmd)
{
  // Not upstream (2026-10-20): RPP 3D's stair mode, so that one controller
  // serves a 2D map and a 3D one. A stretch of a path with heights
  // (OctomapPlanner3D's stair links) that rises stair_min_rise within reach
  // is climbed standing still for the policy switch and then on a short
  // pure-pursuit carrot at stair_linear_vel -- not by the optimizer, and
  // without reading the costmap: no obstacle avoidance on a flight, by
  // decision (2026-10-20). The local costmap's terrain layer judges a stair
  // lethal (a riser over max_step_height, a flight over max_slope_deg) and
  // opens nothing along the path, so neither the optimizer nor a footprint
  // check could take a step onto it; what keeps the robot off a stair it
  // was not routed over is exactly that lethal. A stair is climbed
  // straight, facing the risers, which is what the global planner's
  // straight stair links already give the path. (A draft of the same day
  // opened a corridor one footprint wide along a climbing path and checked
  // the carrot's walk against it; it went with that decision.)
  //
  // The decision (enter, hold for the switch, climb, leave) is StairMode's,
  // the same object RPP 3D runs; what this method adds is the command: the
  // brake while holding, and the carrot once climbing -- the first pose of
  // the pruned path stair_lookahead_dist away, tracked like the direct
  // approach's goal point (turn in place onto it past
  // approach_rotate_min_angle, else trackPoint()). Blind from the moment
  // StairMode holds (stair_enter_dist before the rise) until it leaves
  // (stair_exit_behind_dist past the top), which also keeps the robot clear
  // of the flight's lethal cells when the optimizer takes over again.
  //
  // A plan without heights never asks for stairs (StairMode::setPlan() reads
  // pathHasHeight()), and is not even transformed here, so a 2D map's cycle
  // is what it was before this existed; the one exception is a robot left in
  // stair mode by a cancelled goal, which the next level plan takes back out.
  if (!stair_mode_.enabled()) {
    return false;
  }
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  // StairMode reads the robot's place in the plan off what the path handler
  // has pruned; prune first so the index is this cycle's, as RPP 3D's
  // transformGlobalPlan() runs before its stair block.
  nav_msgs::msg::Path plan;
  size_t remaining = path_handler_.getPath().poses.size();
  if (path_handler_.planHasHeight()) {
    plan = path_handler_.transformPath(robot_pose);
    remaining = path_handler_.remainingPoses();
  }
  const auto stair = stair_mode_.update(clock_->now(), remaining);
  if (!stair.refuse.empty()) {
    throw syncai_nav_core::PlannerException(stair.refuse);
  }
  if (!stair.hold && !stair.on_stairs) {
    if (climbing_stairs_) {
      climbing_stairs_ = false;
      // The warm-started sequence predates the stairs; start from scratch.
      optimizer_.reset();
      RCLCPP_INFO(logger_, "Off the stairs: handing back to the optimizer");
    }
    return false;
  }
  if (!climbing_stairs_) {
    climbing_stairs_ = true;
    approaching_goal_ = false;
    optimizer_.reset();
    RCLCPP_INFO(logger_, "Stairs: the optimizer is skipped until the path is level again");
  }
  // As in rotateToGoalHeading(): the optimizer is not running, so keep its
  // reset_period from zeroing the clamp baseline on the hand-back.
  last_time_called_ = clock_->now();

  double vx = 0.0;
  double wz = 0.0;
  if (stair.hold) {
    brake(vx, wz);
    if (vx == 0.0 && wz == 0.0) {
      // Standing: the policy goes out, or the settle time runs down.
      stair_mode_.standing(clock_->now());
    }
  } else {
    if (plan.poses.empty()) {
      plan = path_handler_.transformPath(robot_pose);
    }
    const double x = robot_pose.pose.position.x;
    const double y = robot_pose.pose.position.y;
    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    // The carrot: the first pose stair_lookahead_dist out, else the path's
    // end (the goal, inside that distance).
    auto carrot = std::find_if(
      plan.poses.begin(), plan.poses.end(), [&](const geometry_msgs::msg::PoseStamped & ps) {
        return std::hypot(ps.pose.position.x - x, ps.pose.position.y - y) >= stair_lookahead_dist_;
      });
    if (carrot == plan.poses.end()) {
      carrot = std::prev(plan.poses.end());
    }
    const double dx = carrot->pose.position.x - x;
    const double dy = carrot->pose.position.y - y;
    const double dist = std::hypot(dx, dy);
    const double bearing = angles::shortest_angular_distance(yaw, std::atan2(dy, dx));
    // No isApproachBlocked() here: the flight is lethal in the costmap.
    if (std::fabs(bearing) > approach_rotate_min_angle_) {
      rotateInPlace(bearing, vx, wz);
    } else if (dist > 1e-3) {
      trackPoint(bearing, dist, stair_linear_vel_, vx, wz);
    }
  }

  cmd = utils::toTwistStamped(vx, wz, robot_pose.header.stamp, costmap_ros_->getBaseFrameID());
  return true;
}

bool MPPIController::isApproachBlocked(double x, double y, double yaw, double walk, double bearing)
{
  // RPP's inCollision(): the padded footprint's perimeter on LETHAL is a
  // collision, unknown space is not while the costmap tracks it. The turn is
  // sampled so that the footprint's farthest corner moves about one cell a
  // step (RPP's projection step for a rotation), the walk at the costmap's
  // resolution along a straight line at the end heading -- under
  // approach_rotate_min_angle the arc stays within a cell or two of that
  // line over the few decimetres it runs.
  syncai_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<syncai_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
  const auto & footprint = costmap_ros_->getRobotFootprint();
  const bool tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  const auto in_collision = [&](double px, double py, double ptheta) {
    const double cost = collision_checker_.footprintCostAtPose(px, py, ptheta, footprint);
    if (cost == static_cast<double>(syncai_costmap_2d::NO_INFORMATION) && tracking_unknown) {
      return false;
    }
    return cost >= static_cast<double>(syncai_costmap_2d::LETHAL_OBSTACLE);
  };

  const double resolution = costmap->getResolution();
  const double radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  const double turn_step = radius > resolution ? resolution / radius : 0.1;
  const int turns = static_cast<int>(std::ceil(std::fabs(bearing) / turn_step));
  for (int i = 0; i <= turns; ++i) {
    if (in_collision(x, y, yaw + (turns > 0 ? bearing * i / turns : 0.0))) {
      return true;
    }
  }

  const double heading = yaw + bearing;
  const int steps = static_cast<int>(std::ceil(walk / resolution));
  for (int i = 1; i <= steps; ++i) {
    const double s = std::min(walk, i * resolution);
    if (in_collision(x + s * std::cos(heading), y + s * std::sin(heading), heading)) {
      return true;
    }
  }
  return false;
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
  stair_mode_.setPlan(path);
  // controller_server resets the goal checker with every path; the final
  // rotation's latch follows it (see rotateToGoalHeading()), and so does the
  // direct approach's, whose goal may have moved with the path.
  rotating_to_goal_ = false;
  approaching_goal_ = false;
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace syncai_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_mppi_controller::MPPIController, syncai_nav_core::Controller)
