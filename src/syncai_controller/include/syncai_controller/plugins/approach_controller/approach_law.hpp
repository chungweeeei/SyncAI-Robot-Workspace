#ifndef SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_LAW_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_LAW_HPP_

#include <string>
#include <utility>
#include <vector>

#include "syncai_controller/planar_pose.hpp"

namespace syncai_controller
{

/** @brief A body-frame command: forward, left, counter-clockwise. */
struct Velocity2
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/**
 * @brief The ApproachLaw's parameters. Every speed is pre-scale (the cmd_vel
 * this node publishes, before driver_manager's scale_*), every floor a gait
 * floor: a command below it does not move a quadruped slowly, it does not
 * move it at all.
 */
struct ApproachLawParams
{
  // Where the nose points while walking in: at the point view_distance ahead
  // of the target along its heading -- for a charger, the tag itself, so the
  // camera keeps it in the middle of the image while the strafe takes out the
  // lateral error (the SyncAI-Robot-Charger aligner turned on the tag's
  // bearing for the same reason). Within final_heading_distance of the target
  // (along its axis) the nose points the target's own yaw instead. 0: always
  // the target's yaw. Once on the target's yaw it goes back to the view point
  // only beyond final_heading_distance + final_heading_hysteresis: the two
  // references differ by up to ~0.14 rad at the hand-over (the gate's lateral
  // limit seen from 0.4 m), a step a robot rocking across one threshold would
  // take on every pass, with gate_heading 0.15 just above it.
  double view_distance{0.30};
  double final_heading_distance{0.10};
  double final_heading_hysteresis{0.05};
  // Further off the reference heading than this: turn in place, nothing
  // else, until it is not (the aligner's first step, "face the marker").
  double rotate_in_place_angle{0.5};

  // Along the target's axis. Forward: k_x * error, at least v_linear_min, at
  // most the speed_table speed for the distance left (or v_linear_max beyond
  // the table). Backward (past the target): at most v_reverse_max.
  double k_x{0.5};
  double v_linear_min{0.05};
  double v_linear_max{0.25};
  double v_reverse_max{0.10};
  // (distance left [m], speed cap [m/s]), distances strictly decreasing: at or
  // under distance i the cap is speed i (the last entry that applies).
  std::vector<std::pair<double, double>> speed_table{{0.25, 0.20}, {0.15, 0.10}, {0.08, 0.05}};

  // Across the target's axis (the strafe) and the heading.
  double k_y{0.5};
  double v_lateral_min{0.10};
  double v_lateral_max{0.20};
  double k_yaw{1.0};
  double v_angular_min{0.15};
  double v_angular_max{0.30};

  // The floors are the GAIT's, so they are owed in the body frame: with the
  // nose on the view point the body is turned off the target's axis by some
  // delta, and a strafe of v_lateral_min along that axis is only
  // v_lateral_min * cos(delta) of body vy -- under the floor, a strafe that
  // never starts. compute() therefore raises both floors by 1 / cos(delta),
  // delta capped at kMaxFloorDelta so a robot far off the axis is not sent
  // twice the floor. The small cross component this leaves on the other body
  // axis is under its floor by construction; whether the gait drops it (and
  // the strafe then creeps along the axis) is to be measured.

  // Per axis: inside `tolerance` the axis stops; stopped, it starts again only
  // beyond tolerance + hysteresis. With every axis floored, a single band
  // would hunt across it: the floor carries the robot through the band in the
  // gait's latency, and the floor brings it back.
  double x_tolerance{0.01};
  double x_hysteresis{0.01};
  double lateral_tolerance{0.015};
  double lateral_hysteresis{0.015};
  double heading_tolerance{0.04};
  double heading_hysteresis{0.04};

  // Walking in is allowed only while |lateral| <= gate_lateral_min +
  // gate_lateral_slope * (distance left) and |heading error| <= gate_heading;
  // once closed (strafe and turn in place) it opens again only below
  // gate_reopen_ratio of both. The aligner's FAST / SLOW / STOP tiers, reduced
  // to open / closed.
  double gate_lateral_min{0.04};
  double gate_lateral_slope{0.15};
  double gate_heading{0.15};
  double gate_reopen_ratio{0.5};

  /** @brief Empty when the parameters are usable, else why not. */
  std::string validate() const;
};

/**
 * @class syncai_controller::ApproachLaw
 * @brief The control law of the ApproachController: drive a holonomic gait
 * (MPC strafes) to a planar target pose, ROS-free so its closed loop can be
 * simulated in a unit test.
 *
 * Errors are taken in the TARGET's frame -- along its axis, across it, and
 * the heading -- and each axis is controlled on its own, the command then
 * rotated into the body frame. What SyncAI-Robot-Charger's aligner did as
 * phases falls out of one gate:
 *  - far off the reference heading: turn in place (RotateToView);
 *  - lateral or heading outside the gate: strafe and turn, no forward (Align);
 *  - inside it: walk in at the speed table's pace, still strafing (Approach);
 *  - past the target: back up to it (Reverse), the gate not consulted;
 *  - every axis inside its band: zero (Hold).
 */
class ApproachLaw
{
public:
  enum class Mode { RotateToView, Align, Approach, Reverse, Hold };

  /** @brief The errors a command was computed from, for logs and feedback. */
  struct Errors
  {
    // Along the target's axis, + = the target is still ahead of the robot.
    double along{0.0};
    // Across it, + = the target is to the left of its axis through the robot.
    double lateral{0.0};
    // Reference heading - robot yaw, and target yaw - robot yaw (rad).
    double heading{0.0};
    double yaw{0.0};
    double heading_ref{0.0};
    // heading_ref is the view point's bearing (true) or the target's yaw.
    bool on_view_point{false};
    bool gate_open{false};
  };

  explicit ApproachLaw(const ApproachLawParams & params);

  /** @brief Replace the parameters (already validated); the band state is kept. */
  void setParams(const ApproachLawParams & params);
  const ApproachLawParams & params() const { return params_; }

  /** @brief Forget which axes are moving and close the gate: a new goal, or a stop. */
  void reset();

  /**
   * @brief One cycle: the body-frame command that moves `robot` toward
   * `target`, both in the same frame. Not acceleration-limited: the caller
   * owns the ramp.
   */
  Velocity2 compute(const Pose2 & robot, const Pose2 & target, Mode * mode, Errors * errors);

private:
  // A stop band with hysteresis: active outside tolerance + hysteresis,
  // inactive inside tolerance, unchanged between.
  static bool band(bool active, double error, double tolerance, double hysteresis);
  // sign(error) * clamp(gain * |error|, floor, cap).
  static double floored(double error, double gain, double floor, double cap);
  double forwardCap(double along) const;

  ApproachLawParams params_;
  bool x_active_{false};
  bool lateral_active_{false};
  bool heading_active_{false};
  bool gate_open_{false};
  // The heading reference has been handed over to the target's yaw (see
  // final_heading_hysteresis).
  bool on_target_yaw_{false};
};

// The most the floors are raised for: 1 / cos(1.0 rad) = 1.85.
constexpr double kMaxFloorDelta = 1.0;

const char * toString(ApproachLaw::Mode mode);

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__APPROACH_CONTROLLER__APPROACH_LAW_HPP_
