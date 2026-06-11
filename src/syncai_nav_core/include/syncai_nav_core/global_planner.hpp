#ifndef SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_
#define SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_nav_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib.
 */
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  virtual ~GlobalPlanner() {}

  /**
   * @param node The node that hosts the planner server
   * @param name The name of this planner instance (parameter namespace)
   * @param tf A pointer to a TF buffer
   * @param costmap_ros A pointer to the costmap
   */
  virtual void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief Method to create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal The goal pose of the robot
   * @return The sequence of poses to get from start to goal, if any
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_
