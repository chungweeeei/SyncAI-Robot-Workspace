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

#ifndef SYNCAI_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define SYNCAI_MPPI_CONTROLLER__OPTIMIZER_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_mppi_controller/critic_manager.hpp"
#include "syncai_mppi_controller/models/optimizer_settings.hpp"
#include "syncai_mppi_controller/models/path.hpp"
#include "syncai_mppi_controller/models/state.hpp"
#include "syncai_mppi_controller/models/trajectories.hpp"
#include "syncai_mppi_controller/motion_models.hpp"
#include "syncai_mppi_controller/tools/noise_generator.hpp"
#include "syncai_mppi_controller/tools/parameters_handler.hpp"
#include "syncai_mppi_controller/tools/utils.hpp"
#include "syncai_nav_core/goal_checker.hpp"

#ifdef __APPLE__
  #include "syncai_mppi_controller/tools/apple_utils.hpp"
#endif

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */
class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  ~Optimizer() {shutdown();}

  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros CostmapROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
  void initialize(
    rclcpp::Node::WeakPtr parent, const std::string & name,
    std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler);

  /**
   * @brief Shutdown for optimizer at process end
   */
  void shutdown();

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal The goal in the plan's frame, nullopt while the plan stops short of it
   * (CriticData::goal)
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
  // last_cmd: the command the controller sent last cycle (after its own
  // floors and clamps). Not upstream; TC-MPPI conditions its samples on it,
  // the iid sampler ignores it.
  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    const std::optional<geometry_msgs::msg::Pose> & goal,
    syncai_nav_core::GoalChecker * goal_checker, const geometry_msgs::msg::Twist & last_cmd);

  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
  models::Trajectories & getGeneratedTrajectories();

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
  xt::xtensor<float, 2> getOptimizedTrajectory();

  /**
   * @brief Reset the optimization problem to initial conditions
   */
  void reset();

  /**
   * @brief Cap the forward speed the optimized sequence may ask for, as
   * upstream's setSpeedLimit() does through the same constraints. Not upstream
   * here (the port had none; see MPPIController::narrowSpeedLimit()). reset()
   * restores vx_max, so a caller sets it every cycle it wants it.
   * @param vx_max The cap (m/s); <= 0 restores the vx_max parameter
   */
  void setSpeedLimit(double vx_max);

protected:
  /**
   * @brief Main function to generate, score, and return trajectories
   */
  void optimize();

  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal The goal in the plan's frame, or nullopt (CriticData::goal)
   * @param goal_checker Object to check if goal is completed
   */
  void prepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    const std::optional<geometry_msgs::msg::Pose> & goal,
    syncai_nav_core::GoalChecker * goal_checker);

  /**
   * @brief Obtain the main controller's parameters
   */
  void getParams();

  /**
   * @brief Apply a runtime parameter change: a full reset() only when the
   * batch or horizon size changed, else the constraints and the noise alone
   */
  void onParametersChanged();

  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
  void setMotionModel(const std::string & model);

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
  void shiftControlSequence();

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
  void generateNoisedTrajectories();

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
  void applyControlSequenceConstraints();

  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
  void updateStateVelocities(models::State & state) const;

  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
  void updateInitialStateVelocities(models::State & state) const;

  /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    xt::xtensor<float, 2> & trajectories,
    const xt::xtensor<float, 2> & state) const;

  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
  void updateControlSequence();

  // Upstream's gamma control cost, the iid sampler's importance term.
  void addGammaControlCost();

  /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
  bool isHolonomic() const;

  /**
   * @brief Using control frequence and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
  void setOffset(double controller_frequency);

  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
  bool fallback(bool fail);

  // Not upstream: TC-MPPI (tools/time_correlated.hpp). The one parse of
  // noise_model / tc_*: on initialisation a bad value throws, as an unknown
  // motion_model does; on a runtime change it logs and keeps the priors
  // (dropped to iid if they no longer fit time_steps). A no-op unless one of
  // its inputs changed since the last successful build.
  void rebuildPriors(bool throw_on_error);

  // Pushes last_cmd onto the head the samples are conditioned on, or refills
  // the head with it when the last cycle was not this optimizer's.
  void updateTcHead(double stamp, const geometry_msgs::msg::Twist & last_cmd);

protected:
  rclcpp::Node::WeakPtr parent_;
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros_;
  syncai_costmap::Costmap * costmap_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ = {
    state_,
    generated_trajectories_,
    path_,
    costs_,
    settings_.model_dt,
    false,
    nullptr,
    nullptr,
    std::nullopt,
    std::nullopt,
    std::nullopt};  /// Caution, keep references

  // TC-MPPI. tc_priors_ == nullptr is the iid sampler (upstream's), and
  // everything below it is unused then.
  std::string noise_model_;
  int tc_depth_{2};
  std::vector<double> tc_vx_weights_, tc_wz_weights_;
  std::shared_ptr<const tc::Priors> tc_priors_;
  // What tc_priors_ was last built from: noise_model, tc_depth, time_steps,
  // model_dt and the two weight lists. nullopt until the first build.
  using TcInputs =
    std::tuple<std::string, int, unsigned int, float, std::vector<double>, std::vector<double>>;
  std::optional<TcInputs> tc_built_;
  std::vector<float> tc_head_vx_, tc_head_wz_;  // the last d sent, oldest first
  std::optional<double> tc_head_stamp_;         // nullopt: refill the head
  std::vector<float> tc_ubar_vx_, tc_ubar_wz_;  // prior means (eq. 18)
  models::ControlSequence tc_mean_;             // sampling means (eq. 19)

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

template<typename E>
inline auto cumsum_1d(const E & expression)
{
  #ifdef __APPLE__
  return utils::manual_cumsum_1d(expression);
  #else
  return xt::cumsum(expression, 0);
  #endif
}

template<typename E>
inline auto cumsum_2d(const E & expression, int axis)
{
 #ifdef __APPLE__
  return utils::manual_cumsum_2d(expression, axis);
 #else
  return xt::cumsum(expression, axis);
 #endif
}

}  // namespace mppi

#endif  // SYNCAI_MPPI_CONTROLLER__OPTIMIZER_HPP_
