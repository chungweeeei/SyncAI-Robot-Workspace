#ifndef SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_CONTROLLER_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_controller/plugins/approach_controller/approach_law.hpp"
#include "syncai_nav_core/controller.hpp"

namespace syncai_controller
{

/**
 * @class syncai_controller::ApproachController
 * @brief Holonomic pose-to-pose controller for a target that keeps being
 * corrected: the last metre of a charge, where every new sighting of the
 * charger's tag moves the target a little.
 *
 * Built for controller_server's `track_pose` (syncai_common/TrackPose), which
 * turns each tracked pose into a target in odom and hands it over through
 * setPlan() as often as one arrives; usable through any action that hands a
 * path, of which it reads the LAST pose. Where DockingController (the graceful
 * law) only walks forward and turns, this one STRAFES -- MPC does -- because
 * the charge's tolerances are lateral: SyncAI-Robot-Charger had to put its
 * robot within ~3 cm of the tag's axis at 0.3 m, which a forward-and-turn law
 * can only do by backing out and coming in again, and an arc swings the
 * camera off the tag on the way. The law (ApproachLaw, ROS-free and simulated
 * in test/test_approach_law.cpp) turns the nose onto a view point -- the tag
 * -- strafes onto the target's axis, walks in only while lateral and heading
 * errors are inside a gate, and backs up when past the target.
 *
 * What it inherits from DockingController, for the same reasons: IT IS BLIND
 * (no costmap; the dock is lethal in it, and avoidance is the staging leg's);
 * the floors are gait floors; the acceleration clamp baselines on its own
 * previous command, never on the measured twist (gait sway); setPlan() is a
 * target change, never a new goal, so it touches no per-goal state.
 *
 * What it deliberately does not do: judge the target. A tracked pose that is
 * wrong (a mirrored PnP solution) is driven to; whoever publishes the poses
 * decides which ones are trusted (the charge tree, from DetectLandmark's
 * `ambiguous` / `consecutive`).
 */
class ApproachController : public syncai_nav_core::Controller
{
public:
  ApproachController() = default;
  ~ApproachController() override = default;

  /**
   * @param costmap_ros The local costmap -- getGlobalFrameID() /
   *   getBaseFrameID() only, never cost data
   */
  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros) override;

  /**
   * @brief Take the target: the LAST pose of `path`, transformed into the
   * costmap's global frame (odom) if it is not already there. Called on every
   * tracked-pose update; keeps the law's band state and the acceleration
   * baseline, which belong to the goal, not to the target.
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /** @brief Per goal, and on a cycle the server stopped the robot without us. */
  void reset() override;

  /**
   * @param pose Robot pose in the costmap's global frame (odom)
   * @param velocity UNUSED: the law is pose only (DockingController has why)
   * @param goal_checker UNUSED: the law has its own bands, which must sit
   *   inside the checker's tolerances (see the params file)
   * @throw syncai_nav_core::PlannerException with no target, or a robot pose
   *   in another frame than the target
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    syncai_nav_core::GoalChecker * goal_checker) override;

protected:
  geometry_msgs::msg::TwistStamped computeVelocityCommandsImpl(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Clamp each axis's change against the previous command. A zero on
   * an axis is let through unramped: every stop the law asks for is a band
   * edge it has to stop at, and the server's own stops are one-step zeros too.
   */
  void clampAcceleration(geometry_msgs::msg::Twist & cmd) const;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  // Frame names and the TF buffer only. Nothing in this plugin reads a cost.
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("ApproachController")};
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<ApproachLaw> law_;
  double max_linear_accel_{1.0};
  double max_angular_accel_{3.2};
  double transform_tolerance_{0.2};
  double control_duration_{0.05};

  // The target, ALWAYS in costmap_ros_->getGlobalFrameID() (odom).
  geometry_msgs::msg::PoseStamped target_;
  bool have_target_{false};

  // The mode of the last cycle, for the INFO line logged when it changes:
  // which axis the law is working on is what tells a robot hunting across a
  // band (align <-> hold every second) from one waiting on a gate.
  bool have_last_mode_{false};
  ApproachLaw::Mode last_mode_{ApproachLaw::Mode::Hold};

  // Acceleration-clamp baseline: our previous command. Zeroed in reset() and
  // on any throw, never in setPlan().
  geometry_msgs::msg::Twist last_cmd_vel_;

  // rviz only, subscriber-gated: the target in odom.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

  // Serialises computeVelocityCommands() / setPlan() against the
  // dynamic-parameter callback, which runs on the node's executor thread.
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_CONTROLLER_HPP_
