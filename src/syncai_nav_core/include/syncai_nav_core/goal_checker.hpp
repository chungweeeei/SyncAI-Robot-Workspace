#ifndef SYNCAI_NAV_CORE__GOAL_CHECKER_HPP_
#define SYNCAI_NAV_CORE__GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"

namespace syncai_nav_core
{

/**
 * @class GoalChecker
 * @brief Function-object for checking whether a goal has been reached
 *
 * This class defines the plugin interface for determining whether you have reached
 * the goal state. This primarily consists of checking the relative positions of two poses
 * (which are presumed to be in the same frame). It can also check the velocity, as some
 * applications require that robot be stopped to be considered as having reached the goal.
 */
class GoalChecker
{
public:
  typedef std::shared_ptr<syncai_nav_core::GoalChecker> Ptr;

  virtual ~GoalChecker() {}

  /**
   * @brief Initialize any parameters from the node
   * @param node Node pointer for grabbing parameters
   * @param plugin_name The name of this checker instance (parameter namespace)
   * @param costmap_ros A pointer to the costmap
   */
  virtual void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void reset() = 0;

  /**
   * @brief Check whether the goal should be considered reached
   * @param query_pose The pose to check
   * @param goal_pose The pose to check against
   * @param velocity The robot's current velocity
   * @return True if goal is reached
   */
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  /**
   * @brief Get the maximum possible tolerances used for goal checking in the major types.
   * Any field without a valid entry is replaced with std::numeric_limits<double>::lowest()
   * to indicate that it is not measured. For tolerance across multiple entries
   * (e.x. XY tolerances), both fields will contain this value since it is the maximum tolerance
   * that each independent field could be assuming the other has no error (e.x. X and Y).
   * @param pose_tolerance The tolerance used for checking in Pose fields
   * @param vel_tolerance The tolerance used for checking velocity fields
   * @return True if the tolerances are valid to use
   */
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) = 0;
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__GOAL_CHECKER_HPP_
