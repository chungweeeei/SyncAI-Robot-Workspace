#ifndef SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_
#define SYNCAI_NAV_CORE__GLOBAL_PLANNER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap/costmap_ros.hpp"
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
    std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros) = 0;

  /**
   * @brief Method to create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal The goal pose of the robot
   * @return The sequence of poses to get from start to goal
   * @throw One of exceptions.hpp's typed PlannerExceptions when there is
   * none (2026-11): the planner server answers the goal with the error_code
   * it maps to and what() as error_msg. An empty path is still accepted as
   * a failure, with no finer reason than PLAN_FAILED.
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;

  /**
   * @brief One path from `start` through every pose of `goals`, in order
   * (compute_path_through_poses, 2026-11: navigate_through_poses' patrol
   * route).
   *
   * The default plans each leg with createPlan() from where the previous one
   * ended and joins them, dropping each later leg's first pose when it
   * repeats the joint; every pose but the last then faces the next one, so
   * a waypoint's own heading is ignored and only the last goal's is kept.
   * A planner whose createPlan() takes its start from the robot rather than
   * the request (OctomapPlanner3D reads the start's height from TF) must
   * override it: later legs start at a waypoint, not at the robot. A leg's
   * failure ends the route: createPlan()'s exception passes through, and an
   * empty leg gives an empty path.
   */
  virtual nav_msgs::msg::Path createPlanThroughPoses(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & goals)
  {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped leg_start = start;
    for (const auto & goal : goals) {
      nav_msgs::msg::Path leg = createPlan(leg_start, goal);
      if (leg.poses.empty()) {
        return nav_msgs::msg::Path();
      }
      path.header = leg.header;
      auto first = leg.poses.begin();
      if (!path.poses.empty()) {
        const auto & a = path.poses.back().pose.position;
        const auto & b = first->pose.position;
        if (a.x == b.x && a.y == b.y && a.z == b.z) {
          ++first;
        }
      }
      path.poses.insert(path.poses.end(), first, leg.poses.end());
      leg_start = leg.poses.back();
    }
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      path.poses[i].pose.orientation.x = 0.0;
      path.poses[i].pose.orientation.y = 0.0;
      path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }
    return path;
  }

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
