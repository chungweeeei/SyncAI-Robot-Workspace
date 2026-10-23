// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef SYNCAI_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define SYNCAI_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "syncai_nav_core/goal_checker.hpp"
#include "syncai_mppi_controller/models/state.hpp"
#include "syncai_mppi_controller/models/trajectories.hpp"
#include "syncai_mppi_controller/models/path.hpp"
#include "syncai_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  syncai_nav_core::GoalChecker * goal_checker;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;

  // Not upstream (2026-10-20): the navigation goal in the costmap frame, for
  // every critic's "near the goal" test (utils::withinPositionGoalTolerance)
  // and GoalCritic / GoalAngleCritic's target. Humble took the last pose of
  // `path` -- the path pruned to prune_distance -- for the goal, so at a
  // corner, where that pose comes back within 1.4 m of the robot, GoalCritic
  // pulled every trajectory at a point mid-path and PathFollowCritic
  // switched off: MPPI cut the corner toward it. nullopt while `path` stops
  // short of the goal (PathHandler::transformedGoal()), which no tolerance
  // contains -- rather than later nav2's goal at any distance, because a
  // goal on the floor above, or behind a wall, can be 1 m away in x / y with
  // the route to it 30 m long.
  std::optional<geometry_msgs::msg::Pose> goal;
};

}  // namespace mppi

#endif  // SYNCAI_MPPI_CONTROLLER__CRITIC_DATA_HPP_
