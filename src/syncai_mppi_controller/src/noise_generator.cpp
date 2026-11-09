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

#include "syncai_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  std::shared_ptr<const tc::Priors> priors)
{
  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    // Not upstream: the settings (upstream wrote them before taking the
    // lock, racing the noise thread) and TC-MPPI's priors change hands
    // together under the one lock, so the thread never pairs a basis built
    // for one time_steps with a batch shaped for another.
    settings_ = settings;
    is_holonomic_ = is_holonomic;
    priors_ = std::move(priors);
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  // Not upstream: TC-MPPI. Standard normals through each axis' basis B,
  // so every sample row is a smooth sequence with covariance Htt^-1 (the
  // spread is the weights', not vx_std / wz_std). The optimizer adds the
  // sampling mean, not the warm start, in setNoisedControls()'s caller.
  if (priors_) {
    const auto correlated = [&](const tc::AxisPrior & axis, xt::xtensor<float, 2> & out) {
      const auto E = xt::eval(xt::random::randn<float>({s.batch_size, s.time_steps}));
      out = xt::xtensor<float, 2>::from_shape({s.batch_size, s.time_steps});
      tc::correlate(axis, E.data(), out.data(), s.batch_size);
    };
    correlated(priors_->vx, noises_vx_);
    correlated(priors_->wz, noises_wz_);
    return;
  }

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
