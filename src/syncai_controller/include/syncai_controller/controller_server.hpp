#ifndef SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
#define SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_common/action/dock_to_pose.hpp"
#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_nav_core/controller.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "syncai_nav_core/progress_checker.hpp"
#include "syncai_util/node_thread.hpp"
#include "syncai_util/odom_subscriber.hpp"
#include "syncai_util/simple_action_server.hpp"
#include "tf2_ros/transform_listener.h"

namespace syncai_controller
{

/**
 * @class syncai_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server -- and,
 * since 2026-10, from a second action server, DockToPose, which drives the
 * same plugins and the same cmd_vel at ONE pose instead of along a path.
 *
 * Unlike nav2_controller, this node derives from rclcpp::Node. Initialization
 * is split into a two-phase pattern: the constructor only declares parameters
 * and creates the (not yet initialized) local costmap node, and configure() —
 * which must be called after the node is owned by a shared_ptr — performs the
 * rest of the setup (nav2's on_configure + on_activate merged).
 *
 * The two action servers are mutually exclusive on dynamic_params_lock_: each
 * execute callback runs on its own worker thread (SimpleActionServer with
 * spin_thread=true), takes the lock for the whole goal, and a goal that finds
 * it held by the other server is refused at once (FollowPath: terminated with
 * a log line; DockToPose: CONTROLLER_BUSY). The members both loops write --
 * current_controller_, current_goal_checker_, end_pose_, current_path_,
 * last_valid_cmd_time_ -- are safe only because of that; do not touch them
 * from anywhere that does not hold the lock.
 */
class ControllerServer : public rclcpp::Node
{
public:
  using ControllerMap = std::unordered_map<std::string, syncai_nav_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, syncai_nav_core::GoalChecker::Ptr>;

  /**
   * @brief Constructor for syncai_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for syncai_controller::ControllerServer
   */
  ~ControllerServer();

  /**
   * @brief Second-phase init: initializes the costmap, loads progress/goal
   * checker and controller plugins and creates the action server. Must be
   * called after the node is owned by a shared_ptr (uses shared_from_this()).
   */
  void configure();

protected:
  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = syncai_util::SimpleActionServer<Action>;
  using DockAction = syncai_common::action::DockToPose;
  using DockActionServer = syncai_util::SimpleActionServer<DockAction>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;
  // ... and this one the DockToPose action (dock_to_pose), same plugins, same
  // cmd_vel, one goal at a time across the two (see the class comment).
  std::unique_ptr<DockActionServer> dock_action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw syncai_nav_core::PlannerException
   */
  void computeControl();

  /**
   * @brief DockToPose action server callback: drive to ONE pose with the
   * docking controller and stop there. The goal's planar pose (frame_id, x,
   * y, yaw) is transformed into the local costmap's global frame (odom)
   * once, here, and tracked in that frame for the rest of the goal; the
   * plugin gets it as the last pose of a 2-pose path [robot start, dock pose]
   * through setPlannerPath(), so everything after that is the FollowPath
   * machinery. Blocks until the goal ends, like computeControl().
   */
  void dockControl();

  /**
   * @brief Resolve a dock goal's plugin ids (empty -> the server defaults),
   * transform its planar pose into the costmap's global frame, and hand the
   * resulting 2-pose path to the controller. Called once per goal and again
   * on every preempt.
   * @param goal The accepted goal
   * @param is_preempt true when called for a pending goal accepted mid-approach:
   *   the controller's per-goal reset() is then skipped (it zeroes the
   *   acceleration-clamp baseline, and doing that on a preempt is the cmd_vel
   *   sawtooth RPP's setPlan() comment records), unless the preempt switches
   *   to a different controller
   * @param error_code DockToPose result code on failure
   * @param error_msg Human-readable reason on failure
   * @return false when the goal cannot be started; nothing has been commanded
   */
  bool acceptDockGoal(
    const DockAction::Goal & goal, bool is_preempt, uint16_t & error_code, std::string & error_msg);

  /**
   * @brief Goal check for the dock loop: the selected goal checker against
   * end_pose_, both already in the costmap's global frame, reusing the
   * cycle's robot pose. No goal_reached_max_remaining_path_ gate -- a 2-pose
   * path cannot loop back onto its own start.
   */
  bool isDockReached(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief One cycle's DockToPose feedback: distance / yaw to the dock pose,
   * commanded speed, time since acceptance. Returned as well, so the dock
   * loop's once-a-second log line reports the same numbers.
   */
  std::shared_ptr<DockAction::Feedback> publishDockFeedback(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::TwistStamped & cmd_vel);

  /**
   * @brief Build a DockToPose result with the final residuals measured on the
   * last cycle that had a robot pose (-1.0 distance when none did).
   */
  std::shared_ptr<DockAction::Result> makeDockResult(
    uint16_t error_code, const std::string & error_msg);

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic, with the
   * FollowPath feedback. computeVelocityCycle() plus the FollowPath-specific
   * tail; the dock loop shares the cycle and publishes its own feedback.
   */
  void computeAndPublishVelocity();

  /**
   * @brief The per-cycle body both action loops share: robot pose lookup
   * (with the failure_tolerance handling), progress checker, thresholded odom
   * twist, the controller call and its failure_tolerance handling.
   */
  struct VelocityCycle
  {
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TwistStamped cmd_vel;
  };

  /**
   * @brief Run one control cycle without publishing anything but the zero
   * velocity of a tolerated pose outage.
   * @param cycle Filled with the robot pose and the command on success
   * @return false when the cycle produced nothing to publish (robot pose
   *   unavailable within failure_tolerance -- zero velocity has already been
   *   published and the controller reset); true with `cycle` filled and the
   *   command NOT yet published, so the caller can add its feedback first
   * @throw syncai_nav_core::PlannerException as computeAndPublishVelocity did
   */
  bool computeVelocityCycle(VelocityCycle & cycle);

  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();

  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);

  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();

  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();

  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  geometry_msgs::msg::Twist getThresholdedTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::Twist twist_thresh;
    twist_thresh.linear.x = getThresholdedVelocity(twist.linear.x, min_x_velocity_threshold_);
    twist_thresh.linear.y = getThresholdedVelocity(twist.linear.y, min_y_velocity_threshold_);
    twist_thresh.angular.z = getThresholdedVelocity(twist.angular.z, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // Held for a whole goal by whichever action loop is running, and try_lock'ed
  // by dynamicParametersCallback() (so parameter changes are refused mid-goal).
  // A timed_mutex since 2026-10, because the two loops acquire it with a
  // 100 ms bound rather than a plain try_lock: the parameter callback holds
  // it for microseconds, a goal for seconds, and 100 ms tells the two apart
  // without ever refusing a goal that merely landed inside a parameter update.
  std::timed_mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros_;
  std::unique_ptr<syncai_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<syncai_util::OdomSubscriber> odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  // The empty Path that ends the docking plan the DockingController draws for
  // the console. The plugin publishes the plan but is never told the goal is
  // over, so every way out of dockControl() clears it here instead.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr docking_plan_clear_pub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<syncai_nav_core::ProgressChecker> progress_checker_loader_;
  syncai_nav_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<syncai_nav_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<syncai_nav_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;
  bool publish_zero_velocity_;

  // Goal checking is gated on path progress: proximity to the end pose alone
  // cannot decide completion of a path that loops back onto its own start
  // (patrol), so the goal checker is only consulted once the remaining path
  // length drops below this threshold (meters). <= 0 disables the gate.
  double goal_reached_max_remaining_path_;

  // Paths with heights (OctomapPlanner3D, path_height.hpp): the robot's ground
  // height is body_frame_'s z minus body_height_, the remaining-path gate above
  // measures from the pose nearest in 3D, and the goal is only reached within
  // goal_z_tolerance_ of the end pose's height -- the goal checkers are x / y
  // only, and a 2F goal can lie right above a 1F stretch of its own route.
  std::string body_frame_;
  double body_height_;
  double goal_z_tolerance_;

  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

  // DockToPose: the plugin ids an empty controller_id / goal_checker_id
  // resolve to (findControllerId("") only falls back when exactly one plugin
  // is loaded, and three are), and the approach timeout a goal with
  // timeout <= 0 gets.
  std::string default_dock_controller_id_;
  std::string default_dock_goal_checker_id_;
  double dock_approach_timeout_;
  // Per dock goal: the resolved timeout, when the clock started, and the
  // robot pose of the last cycle that had one (the result's final residuals).
  double dock_timeout_;
  rclcpp::Time dock_start_time_;
  geometry_msgs::msg::PoseStamped last_dock_robot_pose_;
  bool have_last_dock_robot_pose_{false};
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
