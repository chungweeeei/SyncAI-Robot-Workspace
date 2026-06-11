#include "syncai_planner/plugins/straight_line_planner/straight_line_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "syncai_util/node_utils.hpp"

using syncai_util::declare_parameter_if_not_declared;

namespace syncai_planner
{

void StraightLinePlanner::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;

  // Get the costmap from the costmap_ros object and store the global frame
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Initialize the clock and logger
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "[StraightLinePlanner][%s] Configuring plugin %s of type StraightLinePlanner",
    __func__, name_.c_str());

  // Initialize parameters, declare this plugin's parameters.
  declare_parameter_if_not_declared(
    node, name + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);
  RCLCPP_INFO(
    logger_, "[StraightLinePlanner][%s] interpolation_resolution set to %.2f", __func__,
    interpolation_resolution_);
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;

  // Corner case of the start(x,y) = goal(x,y): the interpolation loop has
  // nothing to do, so return a single-pose path holding the goal directly.
  if (
    start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y) {
    geometry_msgs::msg::PoseStamped pose = goal;
    pose.header.stamp = global_path.header.stamp;
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
    return global_path;
  }

  // calculating the number of loops for current value of interpolation_resolution.
  // Clamp to at least 1 so start/goal closer than one resolution step still
  // produce a valid two-pose path instead of dividing by zero below.
  int total_number_of_loop = std::max(
    1,
    static_cast<int>(
      std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y) /
      interpolation_resolution_));

  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = clock_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = clock_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace syncai_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_planner::StraightLinePlanner, syncai_nav_core::GlobalPlanner)
