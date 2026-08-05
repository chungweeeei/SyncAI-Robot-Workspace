// Copyright (c) 2020, Carlos Luis
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

#include "syncai_planner/plugins/smac_planner/costmap_downsampler.hpp"

#include <string>
#include <memory>
#include <algorithm>

namespace syncai_planner
{

CostmapDownsampler::CostmapDownsampler()
: _costmap(nullptr),
  _downsampled_costmap(nullptr),
  _downsampled_costmap_pub(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::on_configure(
  const rclcpp::Node::WeakPtr & node,
  const std::string & global_frame,
  const std::string & topic_name,
  syncai_costmap_2d::Costmap2D * const costmap,
  const unsigned int & downsampling_factor,
  const bool & use_min_cost_neighbor)
{
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  _use_min_cost_neighbor = use_min_cost_neighbor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<syncai_costmap_2d::Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

  // The empty-node case is deliberate, not defensive: NodeHybrid's obstacle
  // heuristic constructs a downsampler with a default (empty) WeakPtr exactly
  // so that its internal downsampled costmap is never published.
  if (auto locked_node = node.lock()) {
    _downsampled_costmap_pub = std::make_unique<syncai_costmap_2d::Costmap2DPublisher>(
      locked_node, _downsampled_costmap.get(), global_frame, topic_name, false);
  }
}

void CostmapDownsampler::on_activate()
{
  // No-op: syncai_costmap_2d's publisher is a plain rclcpp publisher, live
  // from construction. Kept so upstream call sites (NodeHybrid's obstacle
  // heuristic and the plugin wrappers) port without edits.
}

void CostmapDownsampler::on_deactivate()
{
  // No-op: see on_activate().
}

void CostmapDownsampler::on_cleanup()
{
  _costmap = nullptr;
  _downsampled_costmap.reset();
  _downsampled_costmap_pub.reset();
}

syncai_costmap_2d::Costmap2D * CostmapDownsampler::downsample(
  const unsigned int & downsampling_factor)
{
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  // Adjust costmap size if needed
  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
    _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
    _downsampled_costmap->getResolution() != _downsampled_resolution)
  {
    resizeCostmap();
  }

  // Assign costs
  for (unsigned int i = 0; i < _downsampled_size_x; ++i) {
    for (unsigned int j = 0; j < _downsampled_size_y; ++j) {
      setCostOfCell(i, j);
    }
  }

  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->publishCostmap();
  }
  return _downsampled_costmap.get();
}

void CostmapDownsampler::updateCostmapSize()
{
  _size_x = _costmap->getSizeInCellsX();
  _size_y = _costmap->getSizeInCellsY();
  _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
  _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
  _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
}

void CostmapDownsampler::resizeCostmap()
{
  _downsampled_costmap->resizeMap(
    _downsampled_size_x,
    _downsampled_size_y,
    _downsampled_resolution,
    _costmap->getOriginX(),
    _costmap->getOriginY());
}

void CostmapDownsampler::setCostOfCell(
  const unsigned int & new_mx,
  const unsigned int & new_my)
{
  unsigned int mx, my;
  unsigned char cost = _use_min_cost_neighbor ? 255 : 0;
  unsigned int x_offset = new_mx * _downsampling_factor;
  unsigned int y_offset = new_my * _downsampling_factor;

  for (unsigned int i = 0; i < _downsampling_factor; ++i) {
    mx = x_offset + i;
    if (mx >= _size_x) {
      continue;
    }
    for (unsigned int j = 0; j < _downsampling_factor; ++j) {
      my = y_offset + j;
      if (my >= _size_y) {
        continue;
      }
      cost = _use_min_cost_neighbor ?
        std::min(cost, _costmap->getCost(mx, my)) :
        std::max(cost, _costmap->getCost(mx, my));
    }
  }

  _downsampled_costmap->setCost(new_mx, new_my, cost);
}

}  // namespace syncai_planner
