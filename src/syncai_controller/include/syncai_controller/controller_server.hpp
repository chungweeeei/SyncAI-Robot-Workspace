#ifndef SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
#define SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
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
 * complete control tasks from the exposed FollowPath action server.
 *
 * Unlike nav2_controller, this node derives from rclcpp::Node. Initialization
 * is split into a two-phase pattern: the constructor only declares parameters
 * and creates the (not yet initialized) local costmap node, and configure() —
 * which must be called after the node is owned by a shared_ptr — performs the
 * rest of the setup (nav2's on_configure + on_activate merged).
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
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();

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
    twist_thresh.angular.z =
      getThresholdedVelocity(twist.angular.z, min_theta_velocity_threshold_);
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
  std::mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<syncai_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<syncai_util::OdomSubscriber> odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

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

  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__CONTROLLER_SERVER_HPP_
