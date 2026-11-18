#ifndef SYNCAI_CONTROLLER__PLANAR_POSE_HPP_
#define SYNCAI_CONTROLLER__PLANAR_POSE_HPP_

#include <cmath>

// ROS-free: shared by the approach law, the approach goal checker, the
// track_pose target filter and their tests.

namespace syncai_controller
{

/** @brief A planar pose: x / y in metres, yaw in radians, all in one frame (odom). */
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/**
 * @brief Where the robot is against a target, in the TARGET's frame -- the
 * axes a charge is judged on, and the ones TrackPose reports.
 */
struct TargetErrors
{
  // Along the target's heading, + = the target is still ahead of the robot.
  double along{0.0};
  // Across it, + = the robot is LEFT of the target's axis (TrackPose's sign;
  // ApproachLaw::Errors::lateral is the other way round, the target seen from
  // the robot).
  double lateral{0.0};
  // Target yaw - robot yaw, normalized.
  double yaw{0.0};
};

inline TargetErrors targetErrors(const Pose2 & robot, const Pose2 & target)
{
  const double c = std::cos(target.yaw);
  const double s = std::sin(target.yaw);
  const double dx = robot.x - target.x;
  const double dy = robot.y - target.y;
  TargetErrors e;
  e.along = -(c * dx + s * dy);
  e.lateral = -s * dx + c * dy;
  e.yaw = normalizeAngle(target.yaw - robot.yaw);
  return e;
}

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLANAR_POSE_HPP_
