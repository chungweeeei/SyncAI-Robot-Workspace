// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SYNCAI_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
#define SYNCAI_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_mppi_controller/tools/parameters_handler.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "tf2_ros/buffer.h"

namespace mppi
{

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 */

class PathHandler
{
public:
  /**
    * @brief Constructor for mppi::PathHandler
    */
  PathHandler() = default;

  /**
    * @brief Destructor for mppi::PathHandler
    */
  ~PathHandler() = default;

  /**
    * @brief Initialize path handler on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros CostmapROS object of environment
    * @param tf TF buffer for transformations
    * @param dynamic_parameter_handler Parameter handler object
    */
  void initialize(
    rclcpp::Node::WeakPtr parent, const std::string & name,
    std::shared_ptr<syncai_costmap::CostmapROS>, std::shared_ptr<tf2_ros::Buffer>,
    ParametersHandler *);

  /**
    * @brief Set new reference path
    * @param Plan Path to use
    */
  void setPath(const nav_msgs::msg::Path & plan);

  /**
    * @brief Get reference path
    * @return Path
    */
  nav_msgs::msg::Path & getPath();

  /**
    * @brief Whether the reference path carries ground heights (its poses do
    * not all share one z: OctomapPlanner3D). Set by setPath().
    */
  bool planHasHeight() const { return plan_has_height_; }

  /**
    * @brief How many poses of the path are still ahead of the robot, i.e. not
    * yet pruned by transformPath(). The robot's index in the path as setPath()
    * received it is getPath().poses.size() - remainingPoses(), which is what
    * the stair mode reads; it holds while enforce_path_inversion is off (the
    * shipped configuration), because an inversion cuts the tracked copy short
    * of the full path.
    */
  size_t remainingPoses() const { return global_plan_up_to_inversion_.poses.size(); }

  /**
    * @brief The goal -- the last pose of the path being tracked (the
    * inversion point while enforce_path_inversion cuts it there) -- in the
    * costmap frame, as of the last transformPath(), or nullopt when the path
    * that call returned stops short of it: pruned at prune_distance, or cut at
    * the local costmap's edge. Not the pruned path's last pose, which Humble
    * took for the goal (see CriticData::goal); and not the goal at any
    * distance either, as later nav2 passes it, because the critics' and the
    * controller's tests are planar and straight-line: the goal on the floor
    * above, or behind a wall, can be 1 m away in x / y and 30 m along the
    * route, while one in view is at most prune_distance along it.
    */
  const std::optional<geometry_msgs::msg::Pose> & transformedGoal() const
  {
    return transformed_goal_;
  }

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @return global plan in local frame
   */
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  /**
    * @brief Transform a pose to another frame
    * @param frame Frame to transform to
    * @param in_pose Input pose
    * @param out_pose Output pose
    * @return Bool if successful
    */
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
    * @brief Get largest dimension of costmap (radially)
    * @return Max distance from center of costmap to edge
    */
  double getMaxCostmapDist();

  /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  geometry_msgs::msg::PoseStamped
  transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Get global plan within window of the local costmap size
    * @param global_pose Robot pose
    * @return plan transformed in the costmap frame and iterator to the first pose of the global
    * plan (for pruning)
    */
  std::pair<nav_msgs::msg::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::msg::PoseStamped & global_pose);

  /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  std::string name_;
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  ParametersHandler * parameters_handler_;

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  std::optional<geometry_msgs::msg::Pose> transformed_goal_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};

  // Height-aware matching (syncai_util/path_height.hpp, 2026-10-20): a path
  // with heights is matched to the robot in 3D, with the robot's ground height
  // from body_frame_ (the controller_server's `body_frame` / `lidar_height`,
  // shared with its goal check and RPP 3D). A path without heights (every 2D
  // planner) takes the planar code path unchanged.
  bool plan_has_height_{false};
  std::string body_frame_;
  double body_height_{0.0};

  double max_robot_pose_search_dist_{0};
  double prune_distance_{0};
  double transform_tolerance_{0};
  float inversion_xy_tolerance_{0.2};
  float inversion_yaw_tolerance{0.4};
  bool enforce_path_inversion_{false};
  unsigned int inversion_locale_{0u};
};
}  // namespace mppi

#endif  // SYNCAI_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
