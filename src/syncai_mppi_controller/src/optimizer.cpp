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

#include "syncai_mppi_controller/optimizer.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xrandom.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "syncai_nav_core/exceptions.hpp"

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
  rclcpp::Node::WeakPtr parent, const std::string & name,
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  // Static and refused at runtime (2026-11): setOffset() below pairs it with
  // controller_frequency once, at startup, and the server refuses a runtime
  // controller_frequency for the same reason.
  getParam(s.model_dt, "model_dt", 0.05f, ParameterType::Static);
  parameters_handler_->addReadOnly(
    name_ + ".model_dt",
    "it must equal 1 / controller_frequency, and both are read at startup; change the two "
    "together and restart controller_server");
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.9);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  // Static and refused at runtime (2026-11): upstream registered it dynamic,
  // bound to motion_model_name, a local of this function -- a runtime set
  // wrote through a reference to a dead stack variable, and setMotionModel()
  // never ran again anyway.
  getParam(motion_model_name, "motion_model", std::string("DiffDrive"), ParameterType::Static);
  parameters_handler_->addReadOnly(
    name_ + ".motion_model", "the motion model is built at startup; restart controller_server");

  // Not upstream: TC-MPPI (controller_server_params.yaml has the notes).
  getParam(noise_model_, "noise_model", std::string("iid"));
  getParam(tc_depth_, "tc_depth", 2);
  getParam(tc_vx_weights_, "tc_vx_weights", std::vector<double>{1.1, 0.0, 0.0011});
  getParam(tc_wz_weights_, "tc_wz_weights", std::vector<double>{0.63, 0.0, 0.00063});

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  rebuildPriors(true);
  parameters_handler_->addPostCallback([this]() { onParametersChanged(); });

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};

  settings_.constraints = settings_.base_constraints;

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic(), tc_priors_);
  tc_head_stamp_.reset();
  RCLCPP_INFO(logger_, "Optimizer reset");
}

void Optimizer::onParametersChanged()
{
  // Not upstream (2026-11), which called reset() here: the console sets
  // MPPI's parameters while the robot drives (vx_max, accelerations, critic
  // weights), and each set zeroed the control sequence and the warm start,
  // so the robot braked for a few cycles and built its speed up again. Only
  // a change of the tensors' shapes needs that now; anything else keeps the
  // sequence and takes effect on the next cycle. The constraints are what
  // reset() used to copy from the parameters, and the noise is regenerated
  // so a new vx_std / wz_std is sampled at once.
  // TC-MPPI's priors first: they are built for one time_steps / model_dt as
  // well as for the tc_* values, and both branches below hand them on.
  rebuildPriors(false);
  if (
    costs_.shape(0) != settings_.batch_size ||
    control_sequence_.vx.shape(0) != settings_.time_steps) {
    RCLCPP_INFO(
      logger_, "batch_size / time_steps changed (%u x %u): resetting the optimizer",
      settings_.batch_size, settings_.time_steps);
    reset();
    return;
  }
  settings_.constraints = settings_.base_constraints;
  noise_generator_.reset(settings_, isHolonomic(), tc_priors_);
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, const std::optional<geometry_msgs::msg::Pose> & goal,
  syncai_nav_core::GoalChecker * goal_checker, const geometry_msgs::msg::Twist & last_cmd)
{
  prepare(robot_pose, robot_speed, plan, goal, goal_checker);
  if (tc_priors_) {
    updateTcHead(rclcpp::Time(robot_pose.header.stamp).seconds(), last_cmd);
  }

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  // TC-MPPI's smoothness is in its samples; filtering their weighted mean as
  // well would smooth twice and pull the first command off the sent ones the
  // samples were joined to.
  if (!tc_priors_) {
    utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  }
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset();
  // Not upstream (2026-11; Humble and nav2 main alike): the flag was only
  // ever cleared in prepare(), at the start of a cycle, so a retry ran with
  // it still set, CriticManager::evalTrajectoriesScores() broke out before
  // the first critic, and the retry failed without scoring a trajectory --
  // retry_attempt_limit was dead. Cleared here, a retry is a real one. The
  // params file keeps it at 0 all the same (its comment has why).
  critics_data_.fail_flag = false;

  if (++counter > settings_.retry_attempt_limit) {
    counter = 0;
    // Not upstream (2026-10-21): PlannerException instead of Humble's
    // std::runtime_error. controller_server applies failure_tolerance to
    // PlannerException only and ends the goal on anything else, so the first
    // cycle in which every candidate collided aborted FollowPath outright --
    // on robot01 three aborts in 4.4 s beside one blocker, each spending one
    // of move.xml's FollowPath retries and clearing the local costmap --
    // while the params file, both READMEs and move.xml already counted on the
    // 3 s window. Now each such cycle is a zero cmd_vel and a fresh optimizer
    // (reset() above), and the goal ends only if no cycle succeeds for the
    // whole window.
    throw syncai_nav_core::PlannerException("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, const std::optional<geometry_msgs::msg::Pose> & goal,
  syncai_nav_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.goal = goal;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);


  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  if (tc_priors_) {
    // TC-MPPI: the samples are centred on the warm start low-passed and
    // joined to the sent commands (U_hat, eq. 19), not on the warm start.
    const auto & p = *tc_priors_;
    const auto u_hat_vx = tc::samplingMean(p.vx, control_sequence_.vx.data(), tc_head_vx_.data());
    const auto u_hat_wz = tc::samplingMean(p.wz, control_sequence_.wz.data(), tc_head_wz_.data());
    tc_ubar_vx_ = tc::priorMean(p.vx, tc_head_vx_.data());
    tc_ubar_wz_ = tc::priorMean(p.wz, tc_head_wz_.data());
    tc_mean_ = control_sequence_;
    std::copy(u_hat_vx.begin(), u_hat_vx.end(), tc_mean_.vx.begin());
    std::copy(u_hat_wz.begin(), u_hat_wz.end(), tc_mean_.wz.begin());
    noise_generator_.setNoisedControls(state_, tc_mean_);
  } else {
    noise_generator_.setNoisedControls(state_, control_sequence_);
  }
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) = cumsum_1d(wz * settings_.model_dt) + initial_yaw;

  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + cumsum_1d(dx * settings_.model_dt);
  xt::noalias(traj_y) = state_.pose.pose.position.y + cumsum_1d(dy * settings_.model_dt);
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  xt::noalias(trajectories.yaws) =
    cumsum_2d(state.wz * settings_.model_dt, 1) + initial_yaw;

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  auto && dx = xt::eval(state.vx * yaw_cos);
  auto && dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    cumsum_2d(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    cumsum_2d(dy * settings_.model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  auto && sequence =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  auto & s = settings_;
  if (tc_priors_) {
    // TC-MPPI's importance weight (eq. 23) in place of the gamma term.
    const auto & p = *tc_priors_;
    tc::addWeightTerm(
      p.vx, tc_mean_.vx.data(), tc_ubar_vx_.data(), state_.cvx.data(), s.temperature,
      costs_.data(), s.batch_size);
    tc::addWeightTerm(
      p.wz, tc_mean_.wz.data(), tc_ubar_wz_.data(), state_.cwz.data(), s.temperature,
      costs_.data(), s.batch_size);
  } else {
    addGammaControlCost();
  }

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  applyControlSequenceConstraints();
}

void Optimizer::addGammaControlCost()
{
  // Upstream's control cost, moved out unchanged: the iid sampler's
  // importance term.
  auto & s = settings_;
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }
}

void Optimizer::rebuildPriors(bool throw_on_error)
{
  // Every runtime set runs this (onParametersChanged()), a critic weight or
  // vx_max too; only a change of what the priors are built from rebuilds
  // them, or each console set would restart the head and log the std again.
  const TcInputs inputs{
    noise_model_, tc_depth_, settings_.time_steps, settings_.model_dt, tc_vx_weights_,
    tc_wz_weights_};
  if (tc_built_ && *tc_built_ == inputs) {
    return;
  }
  std::string error;
  std::shared_ptr<const tc::Priors> priors;
  if (noise_model_ == "time_correlated") {
    if (tc_depth_ < 0) {
      error = "tc_depth must be >= 0";
    } else {
      priors = tc::buildPriors(
        settings_.time_steps, static_cast<size_t>(tc_depth_), settings_.model_dt,
        tc_vx_weights_, tc_wz_weights_, error);
    }
  } else if (noise_model_ != "iid") {
    error = "noise_model '" + noise_model_ + "' is neither 'iid' nor 'time_correlated'";
  }
  if (!error.empty()) {
    if (throw_on_error) {
      throw std::runtime_error(error);
    }
    // Kept only while they still fit: priors for another time_steps would
    // meet a batch of the new shape in the noise generator.
    if (tc_priors_ && tc_priors_->vx.T != settings_.time_steps) {
      RCLCPP_ERROR(
        logger_, "%s; the TC-MPPI priors were built for %zu steps, not %u: sampling iid",
        error.c_str(), tc_priors_->vx.T, settings_.time_steps);
      tc_priors_.reset();
      return;
    }
    RCLCPP_ERROR(logger_, "%s; keeping the sampler as it was", error.c_str());
    return;
  }
  tc_built_ = inputs;
  tc_priors_ = priors;
  if (!tc_priors_) {
    return;
  }
  const size_t d = tc_priors_->vx.d;
  tc_head_vx_.assign(d, 0.0f);
  tc_head_wz_.assign(d, 0.0f);
  tc_head_stamp_.reset();
  // The spread is the weights' now: say what it came out at, to compare
  // with the iid vx_std / wz_std it replaces.
  const auto vx_sd = tc::marginalStd(tc_priors_->vx);
  const auto wz_sd = tc::marginalStd(tc_priors_->wz);
  const size_t m = vx_sd.size() / 2;
  RCLCPP_INFO(
    logger_,
    "TC-MPPI sampling, depth %zu: marginal std vx %.3f / %.3f / %.3f, wz %.3f / %.3f / %.3f "
    "(first / middle / last step); vx_std, wz_std, gamma and the Savitzky-Golay filter are "
    "unused",
    d, vx_sd.front(), vx_sd[m], vx_sd.back(), wz_sd.front(), wz_sd[m], wz_sd.back());
}

void Optimizer::updateTcHead(double stamp, const geometry_msgs::msg::Twist & last_cmd)
{
  // The head is the last d commands sent. A cycle that was not this
  // optimizer's -- a reset, or a gap longer than one period while the
  // controller drove by itself (stairs, the final rotation, the approach) --
  // leaves no history to trust, so the head starts over as d copies of the
  // last command: continuous with it, at rest in every derivative.
  const auto vx = static_cast<float>(last_cmd.linear.x);
  const auto wz = static_cast<float>(last_cmd.angular.z);
  const double dt = tc_head_stamp_ ? stamp - *tc_head_stamp_ : -1.0;
  if (dt <= 0.0 || dt > 1.5 * settings_.model_dt) {
    std::fill(tc_head_vx_.begin(), tc_head_vx_.end(), vx);
    std::fill(tc_head_wz_.begin(), tc_head_wz_.end(), wz);
  } else if (!tc_head_vx_.empty()) {
    std::rotate(tc_head_vx_.begin(), tc_head_vx_.begin() + 1, tc_head_vx_.end());
    std::rotate(tc_head_wz_.begin(), tc_head_wz_.begin() + 1, tc_head_wz_.end());
    tc_head_vx_.back() = vx;
    tc_head_wz_.back() = wz;
  }
  tc_head_stamp_ = stamp;
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
  // DiffDrive only (2026-10-28; Omni and Ackermann were never configured,
  // see motion_models.hpp).
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else {
    throw std::runtime_error(
      std::string("Model " + model + " is not valid! The only option is DiffDrive"));
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
