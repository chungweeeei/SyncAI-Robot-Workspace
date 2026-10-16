#ifndef SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_controller/plugins/docking_controller/smooth_control_law.hpp"
#include "syncai_nav_core/controller.hpp"

namespace syncai_controller
{

/**
 * @class syncai_controller::DockingController
 * @brief Pose-to-pose controller for the last metre onto (or off) a dock.
 *
 * A syncai_nav_core::Controller that tracks ONE pose -- the last pose of the
 * path it is given -- with Park & Kuipers' smooth control law (the law nav2's
 * graceful controller and opennav_docking use), ported from Humble's
 * nav2_graceful_controller with the path handling removed: there is no
 * lookahead, no motion target along a path, the target IS the pose. Served
 * through controller_server's `dock_to_pose` action, which hands it a 2-pose
 * path [robot start, dock pose] in the local costmap's global frame (odom).
 *
 * Three things distinguish it from the upstream plugin, all forced by the
 * quadruped:
 *
 *  - v_linear_min is the gait controller's walking floor, not a comfort
 *    value, and wins over the law's arc fidelity (SmoothControlLaw).
 *  - Both in-place rotation branches have an angular floor, v_angular_min:
 *    upstream's proportional rotate-to-target tends to zero exactly where the
 *    final alignment happens, and a 0.016 rad/s command does not turn a gait.
 *  - The stop into the final rotation is one step, not a ramp (clampAcceleration):
 *    ramping down from the floor speed walks through the whole 5 cm window.
 *
 * And one thing the upstream plugin has that this one deliberately does NOT:
 * collision checking. THIS CONTROLLER IS BLIND. It never reads the local
 * costmap (costmap_ros is kept for its frame names and TF buffer only), never
 * simulates the approach against it, and has no footprint checker. The dock
 * is itself a lethal obstacle in the costmap (the lidar sees it above
 * min_obstacle_height), so any footprint test refuses the last half metre of
 * every successful dock -- opennav_docking carves out a dock_collision_threshold
 * for exactly that; here the decision (2026-10) was to switch obstacle
 * avoidance off for the whole approach instead. Avoidance happens in the
 * staging leg of dock.xml (MPPI, local costmap), which ends the robot ~1 m
 * in front of the dock in known-clear space; from there this plugin drives
 * the straight line it is told to. Something that steps in between is not
 * detected: the robot pushes against it until the server's progress checker
 * (no movement for movement_time_allowance) or failure_tolerance ends the
 * goal. The same holds for backing out (undock.xml), where the lidar would
 * not see behind the robot anyway.
 *
 * Two instances are loaded from one params file: `Docking` (forward, rotates
 * in place first when the dock is far off the nose) and `Undocking`
 * (allow_backward, no initial rotation). Direction is this configuration,
 * not a goal field: the law reverses by itself when the target is behind the
 * robot and allow_backward is set.
 */
class DockingController : public syncai_nav_core::Controller
{
public:
  DockingController() = default;
  ~DockingController() override = default;

  /**
   * @brief Read parameters, build the control law, create the debug publishers.
   * @param node The node that hosts the controller server
   * @param name Plugin id, also the parameter prefix (`<name>.k_phi`, ...)
   * @param tf TF buffer
   * @param costmap_ros The local costmap -- used for getGlobalFrameID() /
   *   getBaseFrameID() only, never for cost data
   */
  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief One control cycle: the (v, w) that drives the robot toward the dock
   * pose from `pose`.
   * @param pose Robot pose in the costmap's global frame (odom), from the server
   * @param velocity Measured odom twist. UNUSED, as upstream: the law is pose
   *   only. Do not "fix" this by feeding it in -- the raw Point-LIO twist
   *   carries the gait's body sway, which is why RPP's acceleration clamp
   *   baselines on its own last command too.
   * @param goal_checker Read for its xy tolerance, which is where the final
   *   rotation branch starts
   * @throw syncai_nav_core::PlannerException when no dock pose is set or the
   *   pose arrives in a different frame than the dock pose
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    syncai_nav_core::GoalChecker * goal_checker) override;

  /**
   * @brief Take the dock pose: the LAST pose of `path`, in the path header's
   * frame, transformed once into the costmap's global frame if it is not
   * already there. The rest of the path is ignored.
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Per-goal reset: zero the acceleration-clamp baseline so the first
   * command ramps up from a standstill. Also called by the server on a cycle
   * whose robot-pose lookup failed, for the same reason.
   */
  void reset() override;

  /**
   * @brief Limit v_linear_max, absolute or in percent; NO_SPEED_LIMIT (0.0)
   * restores the configured value. A limit below v_linear_min is raised to
   * it with a warning, because a command below the gait floor is a stall.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief The body of computeVelocityCommands(), run with mutex_ held by the
   * wrapper, which zeroes the acceleration-clamp baseline on any throw.
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommandsImpl(
    const geometry_msgs::msg::PoseStamped & pose, syncai_nav_core::GoalChecker * goal_checker);

  /**
   * @brief In-place rotation command toward an angle: proportional in the
   * angle, floored at v_angular_min, capped at v_angular_max.
   */
  geometry_msgs::msg::Twist rotateToTarget(double angle_to_target) const;

  /**
   * @brief Clamp the command's linear / angular change against the previous
   * command (last_cmd_vel_). With `allow_instant_linear_stop` a zero linear
   * command is let through unramped -- the final-rotation branch.
   */
  void clampAcceleration(geometry_msgs::msg::Twist & cmd, bool allow_instant_linear_stop) const;

  /**
   * @brief rviz only: integrate the law forward from the robot and publish the
   * predicted approach on `local_plan`. Skipped when nobody subscribes. The
   * control law never reads this, and nothing checks it against anything.
   */
  void publishPreview(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Pose & target_in_base, bool backward);

  /**
   * @brief Integrate the law forward from `robot_pose` for at most `horizon`
   * seconds (or until it reaches the target) and append the poses to `path`,
   * in the robot pose's frame, stamped with `path.header`.
   */
  void integrateApproach(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Pose & target_in_base, bool backward, double horizon,
    nav_msgs::msg::Path & path) const;

  /**
   * @brief For the operator console: the predicted approach all the way to
   * the dock, in docking_plan_frame (map), on `docking_plan` at 2 Hz. Only
   * when publish_docking_plan is set (the Docking instance; backing out is
   * not drawn). controller_server publishes the empty Path that clears it
   * when the dock goal ends, since this plugin is never told that.
   */
  void publishDockingPlan(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Pose & target_in_base, bool backward);

  /**
   * @brief Callback executed when a parameter change is detected
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  // Frame names and the TF buffer only. Nothing in this plugin reads a cost.
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("DockingController")};
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<SmoothControlLaw> control_law_;

  // Parameters (see the yaml for the rationale behind each default)
  double k_phi_;
  double k_delta_;
  double beta_;
  double lambda_;
  double v_linear_min_;
  double v_linear_max_, base_v_linear_max_;
  double v_angular_max_;
  double v_angular_min_;
  double slowdown_radius_;
  double max_linear_accel_;
  double max_angular_accel_;
  bool initial_rotation_;
  double initial_rotation_min_angle_;
  bool final_rotation_;
  double rotation_scaling_factor_;
  bool allow_backward_;
  double transform_tolerance_;
  double control_duration_;
  // Read once in initialize(): the publisher exists or it does not.
  bool publish_docking_plan_{false};
  std::string docking_plan_frame_{"map"};

  // The one pose this controller tracks. ALWAYS in costmap_ros_->getGlobalFrameID()
  // (odom): setPlan() transforms it once if it arrives in another frame, and
  // computeVelocityCommands() refuses a robot pose in any other frame. odom
  // is continuous, map is not -- a localizer correction must never move the
  // target under a robot that is 20 cm from it.
  geometry_msgs::msg::PoseStamped dock_pose_;
  bool have_dock_pose_{false};

  // Once the robot has been inside the goal checker's xy tolerance it only
  // rotates, even if it drifts a centimetre back out while turning. Mirrors
  // the stateful SimpleGoalChecker's own xy latch on the server side; the two
  // must agree, or this plugin would resume driving against a checker that
  // has stopped checking xy. Cleared by setPlan() (new target), NOT by
  // reset(): reset() also runs on a robot-pose outage mid-approach, and the
  // checker's latch survives that.
  bool final_rotation_latched_{false};

  // From the goal checker each cycle (its xy tolerance); where the final
  // rotation branch starts.
  double goal_dist_tol_{0.05};

  // Which of computeVelocityCommandsImpl()'s three branches ran last, for the
  // INFO line logged when it changes (2026-10-15). The server's once-a-second
  // dock line has the pose and the command but cannot see the branch, and
  // the branch is what tells "never got inside the xy window" from "inside it
  // and not turning". A burst of these lines alternating between the initial
  // rotation and the control law is a finding, not noise: the robot hunting
  // at initial_rotation_min_angle. None after setPlan(), so every target
  // logs its first branch; reset() leaves it alone, like the latch.
  enum class Branch { None, InitialRotation, ControlLaw, FinalRotation };
  Branch last_branch_{Branch::None};

  // Acceleration-clamp baseline: our own previous command, not the measured
  // odom twist, for the reasons RPP records (gait sway in the raw Point-LIO
  // twist). Zeroed in reset() and in the catch-all of computeVelocityCommands(),
  // never in setPlan(), which is also the preempt path.
  geometry_msgs::msg::Twist last_cmd_vel_;

  // Debug publishers, subscriber-gated. local_plan is deliberately the same
  // topic MPPI publishes its optimal trajectory on: one controller runs at a
  // time, and one rviz display then shows whichever is active.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

  // Not a debug publisher: the console draws it (publishDockingPlan()). Null
  // unless publish_docking_plan.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr docking_plan_pub_;
  rclcpp::Time last_docking_plan_time_;
  bool have_last_docking_plan_{false};

  // Serialises computeVelocityCommands() / setSpeedLimit() against the
  // dynamic-parameter callback, which runs on the node's executor thread.
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_
