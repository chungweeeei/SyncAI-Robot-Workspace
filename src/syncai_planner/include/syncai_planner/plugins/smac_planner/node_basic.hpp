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

#ifndef SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__NODE_BASIC_HPP_
#define SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__NODE_BASIC_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "syncai_planner/plugins/smac_planner/constants.hpp"
#include "syncai_planner/plugins/smac_planner/node_2d.hpp"
#include "syncai_planner/plugins/smac_planner/types.hpp"
#include "syncai_planner/plugins/smac_planner/collision_checker.hpp"

namespace syncai_planner
{

/**
 * @class syncai_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for syncai_planner::NodeBasic
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const unsigned int index)
  : index(index),
    graph_node_ptr(nullptr)
  {
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeBasic
   */
  void populateSearchNode(NodeT * & node);

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeTs.
   * @param node Search node (basic) object to initialize internal node
   * with state
   */
  void processSearchNode();

  // Upstream also caches a pose, a motion-primitive pointer and a reverse
  // flag here. Those were read only by the NodeHybrid and NodeLattice
  // specializations of populateSearchNode()/processSearchNode(), neither of
  // which this port builds, so Node2D is left with the two members the queue
  // actually needs.
  NodeT * graph_node_ptr;
  unsigned int index;
};

}  // namespace syncai_planner

#endif  // SYNCAI_PLANNER__PLUGINS__SMAC_PLANNER__NODE_BASIC_HPP_
