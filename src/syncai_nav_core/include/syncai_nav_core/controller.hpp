#ifndef SYNCAI_NAV_CORE__CONTROLLER_HPP_
#define SYNCAI_NAV_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_nav_core
{

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins.
 * Unlike nav2_core, plugins receive a plain rclcpp::Node and there is no
 * activate/deactivate/cleanup: configure() does the full setup and teardown
 * happens in the plugin destructor.
 */
class Controller
{
public:
  using Ptr = std::shared_ptr<syncai_nav_core::Controller>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Controller() {}

  /**
   * @param node The node that hosts the controller server
   * @param name The name of this controller instance (parameter namespace)
   * @param tf A pointer to a TF buffer
   * @param costmap_ros A pointer to the costmap
   */
  virtual void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   *
   * Called both when a goal starts and on every mid-navigation replan (the BT
   * re-ticks FollowPath with a fresh path at ~0.333 Hz, which reaches the
   * controller as an action preempt). A plugin must therefore treat this as
   * "the path changed", never as "a new goal started" — per-goal state belongs
   * in reset(). Not distinguishing the two is what produced the cmd_vel
   * sawtooth described there.
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Clear per-goal controller state. Called once when a goal is
   * accepted, before the first setPlan(), and never again for that goal.
   *
   * Default is a no-op: only plugins that carry state across control cycles
   * (RPP's acceleration-clamp baseline) need to override it. Not pure virtual
   * so existing plugins keep compiling unchanged.
   */
  virtual void reset() {}

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    syncai_nav_core::GoalChecker * goal_checker) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__CONTROLLER_HPP_
