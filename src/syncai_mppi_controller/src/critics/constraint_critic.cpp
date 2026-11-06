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

#include "syncai_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  // The Optimizer registered these three first, and a getParam() binds a
  // name's dynamic callback only once, so read them as startup values and add
  // this critic's own callbacks (2026-11): the console sets FollowPath.vx_max
  // live, and until then the Optimizer's constraints followed it while this
  // critic kept penalising against the startup limit.
  getParentParam(vx_max_, "vx_max", 0.5, ParameterType::Static);
  getParentParam(vy_max_, "vy_max", 0.0, ParameterType::Static);
  getParentParam(vx_min_, "vx_min", -0.35, ParameterType::Static);
  updateLimits();
  const auto follow = [this](float & limit) {
    return [this, &limit](const rclcpp::Parameter & p) {
      limit = static_cast<float>(p.as_double());
      updateLimits();
    };
  };
  parameters_handler_->addDynamicParamCallback(parent_name_ + ".vx_max", follow(vx_max_));
  parameters_handler_->addDynamicParamCallback(parent_name_ + ".vy_max", follow(vy_max_));
  parameters_handler_->addDynamicParamCallback(parent_name_ + ".vx_min", follow(vx_min_));
}

void ConstraintCritic::updateLimits()
{
  const float min_sgn = vx_min_ > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max_ * vx_max_ + vy_max_ * vy_max_);
  min_vel_ = min_sgn * sqrtf(vx_min_ * vx_min_ + vy_max_ * vy_max_);
}

void ConstraintCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
