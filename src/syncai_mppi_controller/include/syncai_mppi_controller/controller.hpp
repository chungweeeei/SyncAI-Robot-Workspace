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

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap/footprint_collision_checker.hpp"
#include "syncai_mppi_controller/models/constraints.hpp"
#include "syncai_mppi_controller/optimizer.hpp"
#include "syncai_mppi_controller/tools/path_handler.hpp"
#include "syncai_mppi_controller/tools/trajectory_visualizer.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_nav_core/controller.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "syncai_util/stair_mode.hpp"

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
    const rclcpp::Node::SharedPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap::Costmap2DROS> costmap_ros) override;

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
    * @brief Raise a slow walk under the gait's walking floor to the floor
    * @param cmd Optimizer output to adjust in place, before the clamp
    */
  void applyWalkFloor(geometry_msgs::msg::Twist & cmd) const;

  /**
    * @brief Raise an in-place turn under the gait's turn floor to the floor
    * @param cmd Command to adjust in place
    */
  void applyTurnFloor(geometry_msgs::msg::Twist & cmd) const;

  /**
    * @brief Inside the goal checker's xy tolerance, rotate in place to the goal
    * heading instead of running the optimizer
    * @param robot_pose Robot pose in the costmap's global frame
    * @param goal_checker Goal checker whose xy tolerance opens the phase
    * @param cmd Filled with the rotation command when this returns true
    * @return Whether this cycle is a final rotation (the optimizer is skipped)
    */
  bool rotateToGoalHeading(
    const geometry_msgs::msg::PoseStamped & robot_pose, syncai_nav_core::GoalChecker * goal_checker,
    geometry_msgs::msg::TwistStamped & cmd);

  /**
    * @brief Inside approach_dist of the goal, once the optimizer asks to stand,
    * drive onto the goal point directly instead of running the optimizer
    * @param robot_pose Robot pose in the costmap's global frame
    * @param goal_checker Goal checker whose xy tolerance ends the phase
    * @param cmd Filled with the approach command when this returns true, else untouched
    * @return Whether this cycle is a direct approach (the optimizer's output is discarded)
    */
  bool approachGoal(
    const geometry_msgs::msg::PoseStamped & robot_pose, syncai_nav_core::GoalChecker * goal_checker,
    geometry_msgs::msg::TwistStamped & cmd);

  /**
    * @brief From a stand at a new path's start, turn in place onto the path
    * before the optimizer runs (RPP's rotate-to-heading)
    * @param robot_pose Robot pose in the costmap's global frame
    * @param cmd Filled with the turn when this returns true, else untouched
    * @return Whether this cycle is the turn (the optimizer is skipped)
    */
  bool rotateToPath(
    const geometry_msgs::msg::PoseStamped & robot_pose, geometry_msgs::msg::TwistStamped & cmd);

  /**
    * @brief Start a back-out when every candidate of the optimizer collided:
    * from a stand, once per path, as far as back_out_dist is free of LETHAL
    * @param robot_pose Robot pose in the costmap's global frame
    * @return Whether a back-out was started (backOut() then drives it)
    */
  bool startBackOut(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
    * @brief Drive a back-out under way: straight back at walk_vx_min, then a
    * stop, then the turn onto the path is checked again
    * @param robot_pose Robot pose in the costmap's global frame
    * @param cmd Filled with the reverse command when this returns true, else untouched
    * @return Whether this cycle is the back-out's (the optimizer is skipped)
    */
  bool backOut(
    const geometry_msgs::msg::PoseStamped & robot_pose, geometry_msgs::msg::TwistStamped & cmd);

  /**
    * @brief On a path with heights, brake for the policy switch and climb a
    * stair link with a short pure-pursuit carrot instead of running the optimizer
    * @param robot_pose Robot pose in the costmap's global frame
    * @param cmd Filled with the stair command when this returns true, else untouched
    * @return Whether this cycle is stair mode's (the optimizer is skipped)
    */
  bool climbStairs(
    const geometry_msgs::msg::PoseStamped & robot_pose, geometry_msgs::msg::TwistStamped & cmd);

  /**
    * @brief The straight walk onto a point shared by the direct approach and
    * the stair carrot: vx towards `speed`, wz on the pure-pursuit arc through
    * the point, both acceleration-clamped
    * @param bearing Bearing of the point from the robot's heading, signed
    * @param dist Distance to the point
    * @param speed The linear speed to walk at
    * @param vx Set to the linear command
    * @param wz Set to the angular command
    */
  void trackPoint(double bearing, double dist, double speed, double & vx, double & wz) const;

  /**
    * @brief The linear speed stair mode commands this cycle: an S-curve from
    * the speed the robot had when `target` was first asked for to `target`,
    * whose peak acceleration (or deceleration) is max_linear_accel
    * @param target stair_linear_vel to walk, 0 to stop (a turn, a brake)
    * @param now The cycle's time
    */
  double stairSpeed(double target, const rclcpp::Time & now);

  /**
    * @brief Ramp both commands to zero at the clamps' own rates
    */
  void brake(double & vx, double & wz) const;

  /**
    * @brief Whether the direct approach would put the footprint on a LETHAL cell
    * @param x Robot x in the costmap's global frame
    * @param y Robot y
    * @param yaw Robot yaw
    * @param walk Distance to walk along the bearing after turning onto it;
    * negative walks it backwards (the back-out)
    * @param bearing Heading change of the turn onto the goal point
    * @return Whether any footprint along the turn or the walk is in collision
    */
  bool isApproachBlocked(double x, double y, double yaw, double walk, double bearing);

  /**
    * @brief The in-place turn shared by the final rotation and the direct
    * approach: wz towards final_rotation_wz, stopping on the angle, floored
    * @param angle Remaining heading change, signed
    * @param vx Set to the linear command (ramping down to 0)
    * @param wz Set to the angular command
    */
  void rotateInPlace(double angle, double & vx, double & wz) const;

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
  std::shared_ptr<syncai_costmap::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;

  double reset_period_;
  double max_linear_accel_{1.0};
  double max_angular_accel_{3.2};
  double walk_vx_min_{0.0};
  double in_place_wz_min_{0.0};
  double in_place_wz_deadband_{0.05};
  double in_place_vx_max_{0.1};
  double final_rotation_wz_{0.0};
  // Set when the robot first comes within the xy tolerance of the path's end,
  // cleared by setPlan() and reset() -- the same events that clear the
  // stateful goal checker's latch (see rotateToGoalHeading()).
  bool rotating_to_goal_{false};
  double approach_dist_{0.0};
  double approach_rotate_min_angle_{0.25};
  // Set when the optimizer first asks to stand inside approach_dist, cleared
  // when the final rotation takes over, when the way onto the goal is blocked,
  // and by setPlan() / reset() (see approachGoal()).
  bool approaching_goal_{false};
  double rotate_to_path_min_angle_{0.0};
  double rotate_to_path_sampling_dist_{0.5};
  // Set by setPlan(): the new path's start is still to be checked. Cleared
  // once it has been -- aligned, moving, blocked, or turned onto -- with
  // rotating_to_path_ latching the turn in between (see rotateToPath()).
  bool rotate_to_path_pending_{false};
  bool rotating_to_path_{false};
  // The back-out (backOut()): how far, whether this path has had its one,
  // and the one under way -- from where, how far it may go, and whether it
  // is past that and stopping.
  double back_out_dist_{0.0};
  bool back_out_used_{false};
  bool backing_out_{false};
  double back_out_x_{0.0};
  double back_out_y_{0.0};
  double back_out_target_{0.0};
  // Stairs (2026-10-20): the detection, policy switch and latched report are
  // syncai_util::StairMode's, shared with RPP 3D; the climb is climbStairs()'s.
  syncai_util::StairMode stair_mode_;
  double stair_linear_vel_{0.5};
  double stair_lookahead_dist_{0.4};
  // The landing corners (climbStairs()): a heading change past
  // stair_turn_angle is one; the robot stops stair_turn_dist before it and
  // turns in place until the carrot's bearing is under stair_turn_done_angle.
  double stair_turn_angle_{M_PI / 4};
  double stair_turn_dist_{0.15};
  double stair_turn_done_angle_{0.087};
  // The in-place turn on the stairs is latched: in past
  // approach_rotate_min_angle, out under stair_turn_done_angle.
  bool stair_turning_{false};
  // The turn round at the bottom of a descent (climbStairs()) keeps the way
  // round it started on (+1 / -1): near 180 deg the shortest way flips with
  // a degree of drift. Blocked both ways, or on the way, it is given up for
  // this descent (stair_turn_blocked_) and the optimizer gets the robot.
  double stair_turn_sign_{1.0};
  bool stair_turn_blocked_{false};
  // The landing corner the robot has reached (climbStairs()), in the plan's
  // frame: latched until the plan's first corner is another one (more than
  // syncai_util::StairMode::kSameCorner away), there is none, or the stairs
  // end. Without it a stop short of stair_turn_dist (inside the brake zone)
  // let the carrot flip between the corner and past it, the turn reversing
  // every ~0.4 s (see climbStairs()).
  bool stair_corner_latched_{false};
  double stair_corner_x_{0.0};
  double stair_corner_y_{0.0};
  // Descend backwards (climbStairs()): on a path going down the robot turns
  // round and walks in reverse, facing the risers, landings included, and
  // turns back to the path at the bottom before the policy goes back.
  bool stair_descend_backwards_{true};
  bool stair_reverse_{false};
  // Stair mode's speed S-curve (stairSpeed()): its target, where it started
  // and when. A new curve starts whenever the target changes; leaving the
  // stairs and reset() drop it.
  bool stair_ramp_valid_{false};
  double stair_ramp_to_{0.0};
  double stair_ramp_from_{0.0};
  rclcpp::Time stair_ramp_start_;
  // Set while climbStairs() owns the robot (hold or climb), so the optimizer
  // is reset once on the way in and once on the way out.
  bool climbing_stairs_{false};
  // The direct approach's and the stair carrot's only obstacle test (see
  // isApproachBlocked()).
  syncai_costmap::FootprintCollisionChecker<syncai_costmap::Costmap2D *> collision_checker_{
    nullptr};
  double control_duration_{0.05};
  // The last command returned, the clamp's reference; zeroed by reset() and
  // by any throw out of computeVelocityCommands().
  geometry_msgs::msg::Twist last_cmd_vel_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;
};

}  // namespace syncai_mppi_controller

#endif  // SYNCAI_MPPI_CONTROLLER__CONTROLLER_HPP_
