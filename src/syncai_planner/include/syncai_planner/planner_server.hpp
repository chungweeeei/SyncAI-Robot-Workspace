#ifndef SYNCAI_PLANNER__PLANNER_SERVER_HPP_
#define SYNCAI_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
// #include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_costmap_2d/footprint_collision_checker.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "syncai_util/robot_utils.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace syncai_nav_planner
{
/**
 * @class syncai_nav_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */

class PlannerServer : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for syncai_nav_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
}
}  // namespace syncai_nav_planner

#endif  // SYNCAI_PLANNER__PLANNER_SERVER_HPP_
