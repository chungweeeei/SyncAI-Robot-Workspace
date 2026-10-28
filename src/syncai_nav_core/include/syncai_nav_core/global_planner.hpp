#ifndef SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_
#define SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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

  /**
   * @brief Is the part of `path` still ahead of `robot_pose` free, in this
   * planner's own representation of the world?
   *
   * The planner server's is_path_valid service asks the planner that made
   * the last plan, and the planner answers from whatever it keeps about
   * dynamic obstacles -- for OctomapPlanner3D, a map-frame overlay of lidar
   * returns above its surface map (2026-10-21). The default body says
   * "valid" for one that keeps nothing, so the BT keeps its path. Until
   * 2026-10-28 a planner that read the costmap (usesCostmap(), removed with
   * GridBased) was never asked: the server judged its path on the 2D global
   * costmap, which has no layers since.
   *
   * Appends the index of every blocked pose to invalid_pose_indices and
   * returns false if there is one. Must answer fast (the BT calls it at 1 Hz
   * with a 100 ms timeout) and must not wait behind a createPlan() in
   * progress: that call can hold the planner's lock for its whole search.
   */
  virtual bool isPathValid(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot_pose,
    std::vector<int32_t> & invalid_pose_indices)
  {
    (void)path;
    (void)robot_pose;
    (void)invalid_pose_indices;
    return true;
  }
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_
