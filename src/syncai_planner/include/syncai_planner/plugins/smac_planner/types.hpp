// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__TYPES_HPP_
#define SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_util/node_utils.hpp"

namespace syncai_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;

/**
 * @struct syncai_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  // Upstream carries a dozen fields here: turning radius, the direction-change
  // and reverse penalties, the analytic-expansion tuning, the lattice file
  // path. Every one of them was read only by NodeHybrid or NodeLattice, and
  // neither is built in this port, so Node2D's single weight is all that is
  // left. It is spelled cost_penalty in the struct but exposed as the
  // cost_travel_multiplier parameter, matching upstream's naming on both ends.
  float cost_penalty;
};

/**
 * @struct syncai_planner::SmootherParams
 * @brief Parameters for the smoother
 */
struct SmootherParams
{
  /**
   * @brief A constructor for syncai_planner::SmootherParams
   */
  SmootherParams()
  : holonomic_(false)
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(const rclcpp::Node::SharedPtr & node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.");

    // Smoother params
    syncai_util::declare_parameter_if_not_declared(
      node, local_name + "tolerance", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "tolerance", tolerance_);
    syncai_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(local_name + "max_iterations", max_its_);
    syncai_util::declare_parameter_if_not_declared(
      node, local_name + "w_data", rclcpp::ParameterValue(0.2));
    node->get_parameter(local_name + "w_data", w_data_);
    syncai_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(0.3));
    node->get_parameter(local_name + "w_smooth", w_smooth_);
    syncai_util::declare_parameter_if_not_declared(
      node, local_name + "do_refinement", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "do_refinement", do_refinement_);
  }

  double tolerance_;
  int max_its_;
  double w_data_;
  double w_smooth_;
  bool holonomic_;
  bool do_refinement_;
};

}  // namespace syncai_planner

#endif  // SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__TYPES_HPP_
