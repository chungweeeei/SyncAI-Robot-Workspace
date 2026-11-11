#ifndef SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECK_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECK_HPP_

#include <cmath>
#include <string>

#include "syncai_controller/planar_pose.hpp"

namespace syncai_controller
{

/**
 * @brief The ApproachGoalChecker's tolerances. Each must be WIDER than where
 * the ApproachLaw lets its axis rest (that axis's tolerance + hysteresis:
 * 0.02 m, 0.03 m and 0.08 rad at the law's defaults), or the law holds still
 * short of what this accepts and the goal runs into its max_duration. Nothing
 * checks that at run time -- the two are separate plugins -- so
 * test_approach_law reads both blocks of the params file and fails on it.
 */
struct ApproachGoalCheckParams
{
  double along_tolerance{0.025};
  double lateral_tolerance{0.035};
  double yaw_tolerance{0.09};
  // All three inside for this long, without a break, is "reached". Longer
  // than a pass straight through a tolerance at that axis's gait floor (the
  // strafe: 0.07 m at 0.10 m/s = 0.7 s), so a robot crossing the target is
  // never there; the gait's latency (~0.3 s) is well inside it.
  double settle_time{1.0};

  /** @brief Empty when usable, else why not. */
  std::string validate() const
  {
    if (along_tolerance <= 0.0 || lateral_tolerance <= 0.0 || yaw_tolerance <= 0.0) {
      return "along_tolerance, lateral_tolerance and yaw_tolerance must be > 0";
    }
    if (settle_time < 0.0) {
      return "settle_time must be >= 0";
    }
    return "";
  }
};

/**
 * @class syncai_controller::ApproachGoalCheck
 * @brief "Is the robot on the target, and has it stopped there": the test of
 * the ApproachGoalChecker, ROS-free.
 *
 * Why not SimpleGoalChecker (dock_goal_checker, 0.05 m / 0.1 rad, stateful):
 * a charge's tolerance is per axis and mostly lateral, a radius is not; and
 * the stateful latch accepts a robot the moment it enters the radius, still
 * walking, so the goal ended up to 5 cm out and the ApproachLaw's 1-1.5 cm
 * bands were never reached. Here the three axes are taken in the target's
 * frame, as the law takes them, nothing latches (in and out again is not
 * reached), and the robot has to STAY inside for settle_time. Dwell time and
 * not the odometry's twist, because the twist carries the gait's sway and one
 * sample of it cannot tell standing from walking.
 */
class ApproachGoalCheck
{
public:
  ApproachGoalCheck() = default;
  explicit ApproachGoalCheck(const ApproachGoalCheckParams & params) : params_(params) {}

  void setParams(const ApproachGoalCheckParams & params) { params_ = params; }
  const ApproachGoalCheckParams & params() const { return params_; }

  /** @brief Forget how long the robot has been inside: a new goal. */
  void reset() { inside_since_ = -1.0; }

  bool inside(const TargetErrors & e) const
  {
    return std::fabs(e.along) <= params_.along_tolerance &&
           std::fabs(e.lateral) <= params_.lateral_tolerance &&
           std::fabs(e.yaw) <= params_.yaw_tolerance;
  }

  /**
   * @brief One cycle.
   * @param now Seconds on any clock that does not jump
   * @return true once the errors have been inside for settle_time
   */
  bool update(const TargetErrors & e, double now)
  {
    if (!inside(e)) {
      inside_since_ = -1.0;
      return false;
    }
    if (inside_since_ < 0.0) {
      inside_since_ = now;
    }
    return now - inside_since_ >= params_.settle_time;
  }

private:
  ApproachGoalCheckParams params_;
  double inside_since_{-1.0};
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__APPROACH_GOAL_CHECK_HPP_
