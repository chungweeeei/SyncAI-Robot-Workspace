#ifndef SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
#define SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_common/action/track_pose.hpp"
#include "syncai_controller/track_target_filter.hpp"
#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_nav_core/controller.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "syncai_nav_core/progress_checker.hpp"
#include "syncai_util/node_thread.hpp"
#include "syncai_util/odom_subscriber.hpp"
#include "syncai_util/path_cusps.hpp"
#include "syncai_util/path_progress.hpp"
#include "syncai_util/simple_action_server.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_controller
{

/**
 * @class syncai_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 *
 * Unlike nav2_controller, this node derives from rclcpp::Node. Initialization
 * is split into a two-phase pattern: the constructor only declares parameters
 * and creates the (not yet initialized) local costmap node, and configure() —
 * which must be called after the node is owned by a shared_ptr — performs the
 * rest of the setup (nav2's on_configure + on_activate merged).
 *
 * The action servers -- two since 2026-11, with TrackPose (track_pose; DockToPose, the third, went on
 * 2026-11-12) --
 * are mutually exclusive on dynamic_params_lock_: each
 * execute callback runs on its own worker thread (SimpleActionServer with
 * spin_thread=true), takes the lock for the whole goal, and a goal that finds
 * it held by another server is refused at once (FollowPath: terminated with
 * a log line; TrackPose: CONTROLLER_BUSY). The members the loops write --
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

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

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
   * tail.
   */
  void computeAndPublishVelocity();

  /**
   * @brief The per-cycle body: robot pose lookup (with the
   * failure_tolerance handling), progress checker, thresholded odom
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
   * @brief The first half of computeVelocityCycle(): the robot pose, with the
   * failure_tolerance handling of an outage. Split out (2026-11) for
   * track_pose, which needs the pose on cycles it commands nothing (holding
   * for a pose) and runs no progress checker.
   * @return false when the pose is unavailable within failure_tolerance: zero
   *   velocity has been published and the controller reset
   * @throw syncai_nav_core::PlannerException past failure_tolerance
   */
  bool cycleRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief The second half: the controller's command for `pose`, with the
   * failure_tolerance handling of a controller that throws (a zero command is
   * returned inside the window).
   * @throw syncai_nav_core::PlannerException past failure_tolerance
   */
  geometry_msgs::msg::TwistStamped controllerCommand(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   * @return false when a preempting goal named an unknown controller or goal
   * checker and was terminated; the caller stops the robot and leaves the loop
   */
  bool updateGlobalPath();

  /**
   * @brief One INFO line describing a path just handed to the controller (a
   * new goal or a replan): plugins, length, frame, heights, end pose.
   * @param what Prefix naming the occasion
   * @param path The path, non-empty (setPlannerPath() has checked it)
   */
  void logPath(const char * what, const nav_msgs::msg::Path & path);

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
   * @brief Hold the robot at zero until the local costmap is current again
   *        (computeControl()).
   * @return false when the goal was cancelled meanwhile (terminated here)
   * @throws syncai_nav_core::PlannerException once failure_tolerance (at
   *         least 1 s) has passed without a valid command
   */
  bool waitForCurrentCostmap();

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
  // Held for a whole goal by computeControl(), and try_lock'ed by
  // dynamicParametersCallback() (so parameter changes are refused mid-goal).
  // A timed_mutex since 2026-10, acquired with a 100 ms bound rather than a
  // plain try_lock: the parameter callback holds it for microseconds, a goal
  // for its whole run, and 100 ms tells the two apart without ever refusing a
  // goal that merely landed inside a parameter update.
  std::timed_mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros_;
  std::unique_ptr<syncai_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<syncai_util::OdomSubscriber> odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

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
  // The robot's place on current_path_, forward only (path_progress.hpp):
  // updated by isGoalReached() every cycle, read by its remaining-length gate
  // and by the feedback's distance_to_goal. Reset with the path.
  syncai_util::PathProgress path_progress_;

  // current_path_ cut where it turns back on itself (path_cusps.hpp,
  // 2026-11): the controller plugin is handed legs_[leg_] only, end_pose_ is
  // that leg's end, and isGoalReached() hands over the next leg once the goal
  // checker accepts it. leg_ends_[k] is legs_[k]'s last pose as an index into
  // current_path_. One leg -- the path itself -- for a path with no cusp,
  // which is every MOVE.
  syncai_util::CuspParams cusp_params_;
  std::vector<nav_msgs::msg::Path> legs_;
  std::vector<size_t> leg_ends_;
  size_t leg_{0};

  /**
   * @brief Hands legs_[k] to the controller: its plan, end_pose_, and a reset
   * goal checker (a stateful one has latched the previous leg's end)
   */
  void startLeg(size_t k);

  // How far from the robot a track goal's first target may be, and how far
  // off the target's own axis the robot may stand (trackControl() has why).
  // Named for the dock_to_pose start gate they were written for (gone
  // 2026-11-12).
  double dock_max_start_distance_{1.5};
  double dock_max_start_lateral_{0.3};

  // ---- TrackPose (track_pose, 2026-11) ----------------------------------
  // The second action on the same lock: drive to a pose that keeps being
  // corrected (a charger's tag, sighting by sighting), held in odom. See
  // trackControl().

  /**
   * @brief TrackPose action server callback. Each tracked pose is taken into
   * the costmap's global frame (odom) at its own stamp, the goal's offset
   * applied, projected onto the floor and put through a TrackTargetFilter;
   * the controller plugin is handed the filter's target through setPlan().
   * No path, no costmap, no progress checker. Blocks until the goal ends.
   *
   * The odometry is trusted from one pose to the next and no further, and
   * what it does when it fails is guarded, because the approach is blind:
   * a pose it has not reached yet is waited for (track_transform_wait) and
   * then dropped; targets that scatter in odom stop the robot and, past
   * lost_timeout, end the goal TARGET_UNSTABLE; a robot pose moving faster
   * than track_max_odom_speed (nothing here commands a quarter of it) stops
   * the robot and, past failure_tolerance, ends the goal.
   */
  void trackControl();

  /** @brief One track goal's resolved settings and running state. */
  struct TrackRun
  {
    std::string pose_topic;
    std::string tracked_frame;
    tf2::Transform offset;
    // False until startTrackGoal() has accepted the goal (a goal refused for
    // the lock has no clock to report an elapsed time from).
    bool started{false};
    double pose_timeout{0.0};
    double lost_timeout{0.0};
    double max_duration{0.0};
    rclcpp::Time start;
    TrackTargetFilter filter;
    // Counters for the feedback and for telling LOST from TRANSFORM_FAILED.
    uint32_t received{0};
    uint32_t rejected{0};
    uint32_t accepted{0};
    // tracked_frame mode: the stamp of the last transform taken as a pose.
    rclcpp::Time last_frame_stamp{0, 0, RCL_ROS_TIME};
    // The last cycle that had a target to drive to (or the start), and the
    // counters then: what lost_timeout is measured from and sorted by.
    rclcpp::Time last_usable;
    uint32_t received_at_usable{0};
    uint32_t accepted_at_usable{0};
    bool checked_start{false};
    uint8_t state{0};
    // The last cycle that had both a robot pose and a target (the result).
    bool have_errors{false};
    TargetErrors errors;
    // Recent robot poses (stamp [s], x, y), for the odometry speed guard.
    std::deque<std::array<double, 3>> pose_history;
    rclcpp::Time odom_fast_since{0, 0, RCL_ROS_TIME};
    bool odom_fast{false};
  };

  /**
   * @brief Resolve a track goal (plugins, offset, timeouts), subscribe to its
   * topic and reset the per-goal state. Nothing is commanded.
   * @return false with the result code and reason when the goal is refused
   */
  bool startTrackGoal(
    const syncai_common::action::TrackPose::Goal & goal, TrackRun & run, uint16_t & error_code,
    std::string & error_msg);

  /**
   * @brief Move what has arrived on the tracked topic (or the tracked frame's
   * newest transform) into run.filter as targets in odom. A pose that cannot
   * be transformed at its stamp yet stays queued for track_transform_wait.
   */
  void intakeTrackedPoses(TrackRun & run);

  std::shared_ptr<syncai_common::action::TrackPose::Result> makeTrackResult(
    const TrackRun & run, uint16_t error_code, const std::string & error_msg);

  using TrackAction = syncai_common::action::TrackPose;
  using TrackActionServer = syncai_util::SimpleActionServer<TrackAction>;
  std::unique_ptr<TrackActionServer> track_action_server_;

  // What an empty controller_id / goal_checker_id and a zero Duration
  // resolve to.
  std::string default_track_controller_id_;
  std::string default_track_goal_checker_id_;
  double track_pose_timeout_{0.5};
  double track_lost_timeout_{5.0};
  double track_max_duration_{90.0};
  // TrackTargetFilter's limits and minimum, for a goal with a filter window.
  double track_target_max_spread_xy_{0.08};
  double track_target_max_spread_yaw_{0.2};
  int track_min_poses_{3};
  // How long a pose waits for the odometry to reach its stamp.
  double track_transform_wait_{0.3};
  // Net robot speed in odom, over kTrackSpeedBaseline, above which the
  // odometry is not believed.
  double track_max_odom_speed_{1.0};

  // The tracked topic's subscription (alive for one goal) and what it has
  // received, handed from the node's executor thread to the track loop.
  struct TrackedPose
  {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time arrival;
    // Already in poses_received (a pose waiting for its transform is seen by
    // more than one cycle).
    bool counted{false};
  };
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr track_pose_sub_;
  std::mutex track_inbox_mutex_;
  std::deque<TrackedPose> track_inbox_;

  // Which action holds dynamic_params_lock_, for the line a refused goal
  // logs: a string literal, written by the holder once it owns the lock.
  std::atomic<const char *> driving_{"another action"};
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
