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

#include "syncai_mppi_controller/tools/path_handler.hpp"

#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/path_height.hpp"

namespace mppi
{

void PathHandler::initialize(
  rclcpp::Node::WeakPtr parent, const std::string & name,
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer, ParametersHandler * param_handler)
{
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  auto node = parent.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(prune_distance_, "prune_distance", 1.5);
  getParam(transform_tolerance_, "transform_tolerance", 0.1);
  getParam(enforce_path_inversion_, "enforce_path_inversion", false);
  if (enforce_path_inversion_) {
    getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);
    getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);
    inversion_locale_ = 0u;
  }
  // Server-level, not per plugin: the controller_server's goal check and RPP
  // 3D read the same two, and two copies of a frame name drift. Static: a
  // frame does not change at runtime. The server declares them; the empty
  // prefix reads them the way the Optimizer reads controller_frequency.
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParentParam(body_frame_, "body_frame", std::string("pointlio_body"), ParameterType::Static);
  getParentParam(body_height_, "lidar_height", 0.481, ParameterType::Static);
}

std::pair<nav_msgs::msg::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using syncai_util::geometry_utils::euclidean_distance;

  auto begin = global_plan_up_to_inversion_.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    syncai_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
    max_robot_pose_search_dist_);

  // Find closest point to the robot. In 3D when the plan has heights: on a
  // spiral stair the turn 2.5 m above the robot is as close on x / y as the
  // one it stands on, and the 10 m search window holds both (path_height.hpp).
  const bool is_3d = plan_has_height_;
  auto closest_point = syncai_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose, is_3d](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps, is_3d);
    });

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  auto pruned_plan_end =
    syncai_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  unsigned int mx, my;
  // Find the furthest relevent pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    transformPose(costmap_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);

    // Check if pose is inside the costmap
    if (!costmap_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      return {transformed_plan, closest_point};
    }

    // Filling the transformed plan to return with the transformed pose. The
    // optimizer and the critics read x / y / yaw only; a height on the local
    // path would only reach local_plan's and the visualizer's consumers, which
    // draw it on the planar costmap.
    if (is_3d) {
      costmap_plan_pose.pose.position.z = 0.0;
    }
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  return {transformed_plan, closest_point};
}

geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw std::runtime_error("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error(
            "Unable to transform robot pose into global plan's frame");
  }

  // A plan with heights (OctomapPlanner3D) is matched in 3D: the robot's pose
  // comes from the planar base_link (z = 0), so its z is replaced by the
  // ground height under the robot in the plan's frame, exactly as RPP 3D
  // does. No height, no matching -- on a stair that would mean steering by
  // whichever turn is nearest in x / y -- so this cycle fails, with the
  // exception the server tolerates for failure_tolerance (the file's other
  // throws are std::runtime_error, which ends the goal at once; a TF that is
  // a scan late is not that kind of failure).
  if (plan_has_height_) {
    std::string error;
    double ground_z = 0.0;
    if (!syncai_util::lookupGroundZ(
          *tf_buffer_, global_plan_up_to_inversion_.header.frame_id, body_frame_, body_height_,
          ground_z, &error)) {
      throw syncai_nav_core::PlannerException(
        "Plan has heights but the robot's (" + body_frame_ + " in " +
        global_plan_up_to_inversion_.header.frame_id + ") is unknown: " + error);
    }
    robot_pose.pose.position.z = ground_z;
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevent bounds of path to use
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  // The goal is in view when the transformed plan holds every pose from the
  // robot's to the last one, i.e. neither prune_distance nor the costmap's
  // edge cut it short (transformedGoal()). Read before prunePlan(), which
  // invalidates lower_bound.
  const auto poses_to_goal =
    static_cast<size_t>(std::distance(lower_bound, global_plan_up_to_inversion_.poses.end()));
  transformed_goal_.reset();
  if (!transformed_plan.poses.empty() && transformed_plan.poses.size() == poses_to_goal) {
    transformed_goal_ = transformed_plan.poses.back().pose;
  }

  prunePlan(global_plan_up_to_inversion_, lower_bound);

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  transformed_goal_.reset();
  plan_has_height_ = syncai_util::pathHasHeight(plan);
  if (enforce_path_inversion_) {
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi
