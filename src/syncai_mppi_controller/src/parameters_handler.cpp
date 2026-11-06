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

#include "syncai_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(
  const rclcpp::Node::WeakPtr & parent)
{
  node_ = parent;
  auto node = node_.lock();
  node_name_ = node->get_name();
  logger_ = node->get_logger();
}

void ParametersHandler::start()
{
  auto node = node_.lock();
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::dynamicParamsCallback, this,
      std::placeholders::_1));

  auto get_param = getParamGetter(node_name_);
  get_param(verbose_, "verbose", false);
}

rcl_interfaces::msg::SetParametersResult
ParametersHandler::dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);

  // Not upstream (2026-11): the batch is vetted before anything is applied,
  // and the pre / post callbacks run only when it holds one of this
  // handler's own dynamic parameters. Upstream ran them for every set on the
  // node -- its callback sees the server's parameters and every other
  // plugin's too -- and the post callback is the Optimizer's: a console
  // change of goal_checker.xy_goal_tolerance or FollowPathRPP3D.* mid-goal
  // dropped MPPI's control sequence and warm start, and the robot braked.
  // It also warned "Parameter ... not found" for each of those.
  bool any_own = false;
  for (const auto & param : parameters) {
    const std::string & param_name = param.get_name();
    if (auto ro = read_only_.find(param_name); ro != read_only_.end()) {
      result.successful = false;
      result.reason = param_name + " cannot be changed at runtime: " + ro->second;
      RCLCPP_WARN(logger_, "Set refused: %s", result.reason.c_str());
      return result;
    }
    if (get_param_callbacks_.count(param_name) != 0) {
      any_own = true;
      continue;
    }
    // One of this plugin's parameters read at startup only (Static): the
    // set succeeds -- rclcpp stores it -- but nothing reads it until the
    // node restarts, which is worth a line. Another namespace's is silent.
    const auto dot = param_name.find('.');
    if (dot != std::string::npos && prefixes_.count(param_name.substr(0, dot)) != 0) {
      RCLCPP_WARN(
        logger_, "Parameter %s is read at startup only; the new value is used after a restart",
        param_name.c_str());
    }
  }
  if (!any_own) {
    return result;
  }

  for (auto & pre_cb : pre_callbacks_) {
    pre_cb();
  }

  for (auto & param : parameters) {
    if (auto callbacks = get_param_callbacks_.find(param.get_name());
        callbacks != get_param_callbacks_.end()) {
      for (auto & callback : callbacks->second) {
        callback(param);
      }
    }
  }

  for (auto & post_cb : post_callbacks_) {
    post_cb();
  }

  return result;
}

}  // namespace mppi
