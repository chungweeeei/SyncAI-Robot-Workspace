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

#include <cstdio>

#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/path_height.hpp"
#include "syncai_util/path_progress.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi
{

void PathHandler::initialize(
  rclcpp::Node::WeakPtr parent, const std::string & name,
  std::shared_ptr<syncai_costmap::CostmapROS> costmap, std::shared_ptr<tf2_ros::Buffer> buffer,
  ParametersHandler * param_handler)
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
  // The FIRST local minimum from the pruned front, not the window's minimum
  // (2026-11, path_progress.hpp): a patrol route (navigate_through_poses)
  // walks a corridor out and back, the return leg on top of the outgoing one
  // and inside the window for the last 5 m before the turn, and upstream's
  // min_by -- '<=', the later pose on a tie -- would prune onto the return leg
  // and turn the robot back short of its waypoint. Pruning is permanent, so
  // that could never be undone. The pruned front is this search's memory.
  const bool is_3d = plan_has_height_;
  auto closest_point = syncai_util::firstLocalMinimum(
    begin, closest_pose_upper_bound,
    [&global_pose, is_3d](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps, is_3d);
    },
    syncai_util::PathProgress::kDefaultHysteresis);

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  auto pruned_plan_end =
    syncai_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  // One plan frame -> costmap frame lookup, at the robot's stamp, for every
  // pose of the window (2026-11). Upstream transformed pose by pose, each a
  // buffer lookup with its own transform_tolerance wait: ~20 poses per call
  // and up to four calls a cycle on a 3D path (climbStairs,
  // rotateToGoalHeading, rotateToPath, the optimizer), so a stale map -> odom
  // stalled one cycle for N x the tolerance, and a failed lookup left a zero
  // pose (the robot's origin) in the local path, the return value unchecked.
  // Now a failure is one PlannerException, inside failure_tolerance -- though
  // transformToGlobalPlanFrame() has just looked the same chain up the other
  // way at the same stamp, so it does not fail in practice.
  const std::string & costmap_frame = costmap_->getGlobalFrameID();
  const std::string & plan_frame = global_plan_.header.frame_id;
  geometry_msgs::msg::TransformStamped plan_to_costmap;
  if (plan_frame == costmap_frame) {
    plan_to_costmap.transform.rotation.w = 1.0;
  } else {
    try {
      plan_to_costmap = tf_buffer_->lookupTransform(
        costmap_frame, plan_frame, tf2_ros::fromMsg(global_pose.header.stamp),
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      throw syncai_nav_core::PlannerException(
        "Unable to transform the plan from " + plan_frame + " into the costmap's " + costmap_frame +
        ": " + ex.what());
    }
  }

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
    global_plan_pose->header.frame_id = plan_frame;
    tf2::doTransform(*global_plan_pose, costmap_plan_pose, plan_to_costmap);
    costmap_plan_pose.header.frame_id = costmap_frame;  // also when the frames are one
    costmap_plan_pose.header.stamp = global_pose.header.stamp;

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
  // Every failure in this file is a PlannerException (2026-11), as RPP 3D's
  // are at the same three spots: the server tolerates it for
  // failure_tolerance, commanding zero, and ends the goal only after that.
  // Until then these were std::runtime_error, which the server's catch-all
  // ends the goal on at once -- so one map -> odom a scan late past
  // transform_tolerance aborted FollowPath and burned a RecoveryNode retry,
  // while the same miss in RPP 3D, or in the ground-z lookup below, waited.
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw syncai_nav_core::PlannerException("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    char buf[256];
    std::snprintf(
      buf, sizeof buf,
      "Unable to transform the robot pose (%s) into the plan's frame (%s) within "
      "transform_tolerance %.2f s",
      pose.header.frame_id.c_str(), global_plan_up_to_inversion_.header.frame_id.c_str(),
      transform_tolerance_);
    throw syncai_nav_core::PlannerException(buf);
  }

  // A plan with heights (OctomapPlanner3D) is matched in 3D: the robot's pose
  // comes from the planar base_link (z = 0), so its z is replaced by the
  // ground height under the robot in the plan's frame, exactly as RPP 3D
  // does. No height, no matching -- on a stair that would mean steering by
  // whichever turn is nearest in x / y -- so this cycle fails, with the
  // exception the server tolerates for failure_tolerance (a TF that is a
  // scan late is not a reason to end the goal).
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
    // No pose of the plan within prune_distance of the robot falls inside the
    // local costmap: the robot is far off its path (a localizer jump). A
    // replan fixes it, which the BT only starts after this goal ends, so
    // this still ends it, after failure_tolerance rather than at once.
    throw syncai_nav_core::PlannerException(
      "No pose of the plan near the robot lies inside the local costmap (robot far off the path)");
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
