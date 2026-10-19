#ifndef SYNCAI_UTIL__STAIR_MODE_HPP_
#define SYNCAI_UTIL__STAIR_MODE_HPP_

// The stair state machine shared by the controllers (2026-10).
//
// A stretch of a plan with heights (OctomapPlanner3D's stair links) that rises
// `min_rise` within reach is climbed in "stair mode": the gait controller's RL
// policy is switched to `stair_policy` over `set_policy_mode`, standing still,
// the robot stands `settle_time`, and the controller then follows the path
// with its own stair carrot and speed; past the top (or bottom) it stops again
// and the policy goes back. This class owns the detection, the policy switch
// and the latched `stair_mode` report. It does NOT own the braking or the
// climbing command: each controller brakes with its own acceleration clamp and
// tracks with its own pursuit maths (RPP 3D caps its lookahead and speed, MPPI
// skips its optimizer for a pure-pursuit carrot), and tells this class when the
// robot has been commanded to a stop (standing()).
//
// Written first inside RegulatedPurePursuitController3D and lifted out when
// MPPI needed the same behaviour: a controller that climbs a stair link on a
// different policy switch, or that settles for a different time, would be a
// second calibration of the same gait. It lives in syncai_util rather than
// syncai_controller because syncai_mppi_controller cannot depend on the
// package that loads it (syncai_controller exec_depends on MPPI), and
// syncai_nav_core is the plugin interface and carries no code.
//
// Two ways in: initialize() for a ROS node (declares the parameters under the
// plugin's name, creates the service client, the `mode` subscription and the
// latched `stair_mode` publisher), configure() for a test, which injects the
// three side effects as callbacks. Everything else is plain C++ on a Path.

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "syncai_common/srv/set_policy_mode.hpp"

namespace syncai_util
{

class StairMode
{
public:
  // The detection / switch parameters. Defaults are the values the params file
  // ships for every controller (controller_server_params.yaml, "Stair mode").
  struct Params
  {
    // use_stair_mode. Off: update() never asks for stairs and never holds.
    bool enabled{true};
    // stair_min_rise: height span of the path, within the windows below, that
    // counts as a stair. dp1f_1012's level paths rise 0.10-0.20 m within 1 m,
    // its stair link 0.68 m and its 2F ramp 0.47 m, so 0.3 takes the stairs
    // and the ramp and nothing else, once each.
    double min_rise{0.3};
    // stair_enter_dist: how far ahead the rise is looked for off the stairs.
    double enter_dist{1.0};
    // stair_exit_behind_dist / stair_exit_ahead_dist: the window that must
    // have gone level before leaving. 2.5 m ahead keeps a landing (rarely
    // longer than ~2 m on a spiral) in stair mode instead of switching the
    // policy twice per turn; 0.8 m behind has the hind legs off the last step.
    double exit_behind_dist{0.8};
    double exit_ahead_dist{2.5};
    // stair_policy_settle_time: how long the robot stands after each switch.
    double settle_time{3.0};
    // stair_policy_mode: the RL policy index for stairs (syncai_common
    // RobotLowLevelMode: 0 PPO, 1 HIMLOCO, 2 CHAMP, 3 ISSAC). -1 = never
    // switch: the controller changes speed and carrot only, with no stops.
    int stair_policy{1};
    // flat_policy_mode: the index to go back to; -1 = whatever the gait
    // controller reported on `mode` before the switch (with no report heard,
    // the robot stays on the stair policy rather than guess an index).
    int flat_policy{-1};
  };

  // What this cycle is, decided by update().
  struct Decision
  {
    // Brake to a stop and hold it: a policy switch is pending. The controller
    // calls standing() once its command is zero.
    bool hold{false};
    // The robot is set up for stairs: climb with the stair carrot and speed.
    bool on_stairs{false};
    // Non-empty: this cycle must fail (the controller throws a
    // PlannerException with it, and the server holds the robot). Only raised
    // on the way in, when set_policy_mode is not there to switch the policy.
    std::string refuse;
  };

  using SendPolicyFn = std::function<void(int)>;
  using ServiceReadyFn = std::function<bool()>;
  using PublishFn = std::function<void(bool)>;

  StairMode() = default;

  /**
   * @brief Set up on a ROS node, as the controller plugins do
   * @param node The controller server
   * @param plugin_name The plugin's parameter prefix (`<plugin>.use_stair_mode` ...)
   *
   * Relative names: /<robot_id>/set_policy_mode and /<robot_id>/mode are
   * syncai_driver_manager's; `stair_mode` (std_msgs/Bool, latched) is this
   * class's report, published once here and at every change.
   */
  void initialize(const rclcpp::Node::SharedPtr & node, const std::string & plugin_name);

  /**
   * @brief Set up without ROS (tests): the side effects are callbacks
   * @param params Detection / switch parameters
   * @param send_policy Called with the policy index to send
   * @param service_ready Whether set_policy_mode is there; empty = always
   * @param publish Called with the new on_stairs value; empty = none
   */
  void configure(
    const Params & params, SendPolicyFn send_policy, ServiceReadyFn service_ready = {},
    PublishFn publish = {});

  /// The plan as the controller's setPlan() received it (before any pruning).
  void setPlan(const nav_msgs::msg::Path & full_plan);

  /**
   * @brief Run the state machine for one control cycle
   * @param now The controller's clock
   * @param remaining_poses How many poses of the plan the controller still
   * tracks: it prunes from the front, so the robot's index in the full plan is
   * `full_plan.size() - remaining_poses`
   */
  Decision update(const rclcpp::Time & now, size_t remaining_poses);

  /**
   * @brief The controller has commanded a stop while hold is set: send the
   * policy, or finish the switch once settle_time has passed
   */
  void standing(const rclcpp::Time & now);

  /**
   * @brief New goal: a transition whose policy went out counts as done, one
   * that never sent as never started. on_stairs survives: it is the policy
   * the gait controller is on, which outlives the goal that set it.
   */
  void reset();

  /// The gait controller's live policy index (from `mode` data[0]).
  void setReportedPolicy(int policy) { reported_policy_.store(policy); }

  bool enabled() const { return params_.enabled; }
  bool onStairs() const { return on_stairs_; }
  // The live parameters: a dynamic-parameter callback edits them in place.
  Params & params() { return params_; }
  const Params & params() const { return params_; }

  /**
   * @brief Height span (max z - min z) of `path` from `behind` metres before
   * pose `idx` to `ahead` metres after it, along the path on x / y
   */
  static double stairRise(
    const nav_msgs::msg::Path & path, size_t idx, double behind, double ahead);

private:
  enum class Transition { None, Entering, Leaving };

  bool wantStairs(size_t remaining_poses) const;
  // Ask the gait controller for `policy` (fire and forget: the driver's UDP has
  // no acknowledgement to wait for).
  void sendPolicy(int policy);
  void publish();

  Params params_;
  rclcpp::Logger logger_{rclcpp::get_logger("StairMode")};

  SendPolicyFn send_policy_;
  ServiceReadyFn service_ready_;
  PublishFn publish_;
  rclcpp::Client<syncai_common::srv::SetPolicyMode>::SharedPtr policy_client_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stair_mode_pub_;

  nav_msgs::msg::Path full_plan_;
  bool plan_has_height_{false};
  // What the robot is set up for: true once the stair policy was commanded and
  // has settled.
  bool on_stairs_{false};
  Transition transition_{Transition::None};
  bool policy_sent_{false};
  rclcpp::Time settle_until_;
  // The policy to go back to after the stairs (-1: none known).
  int restore_policy_{-1};
  // Written from the executor (the `mode` subscription), read in the control loop.
  std::atomic<int> reported_policy_{-1};
};

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__STAIR_MODE_HPP_
