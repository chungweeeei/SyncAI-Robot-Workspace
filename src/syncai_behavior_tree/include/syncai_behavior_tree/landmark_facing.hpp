#ifndef SYNCAI_BEHAVIOR_TREE__LANDMARK_FACING_HPP_
#define SYNCAI_BEHAVIOR_TREE__LANDMARK_FACING_HPP_

#include <optional>

#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace syncai_behavior_tree
{

/**
 * @brief A landmark's pose with its position kept and its face turned
 * square to the sensor: +x from the landmark to the sensor's origin, +z the
 * detected +z made perpendicular to that.
 *
 * For a sighting whose orientation cannot be trusted while its position can
 * (2026-11): a single ArUco marker further than ~0.6 m has two mirrored PnP
 * solutions, the same position with the normal reflected about the viewing
 * ray, and the detector alternated between them frame by frame (robot01,
 * 2026-11-18: a docking target built on the detected normal changed side 131
 * times in 85 s, and track_pose stopped 76 times). The viewing ray is what
 * both solutions share, so a target built on it does not jump, and a robot
 * that follows it walks straight at the landmark until the orientation can
 * be read.
 *
 * Frame-agnostic: the sensor is the origin of the frame `detected` is in,
 * and "up" is taken from the detected pose, not from an axis of that frame
 * (a camera's optical frame has none pointing up). nullopt when the landmark
 * is at the origin or its +z lies along the ray, where no such pose exists.
 */
inline std::optional<geometry_msgs::msg::Pose> faceLandmarkToSensor(
  const geometry_msgs::msg::Pose & detected)
{
  constexpr double kMinLength = 1e-6;

  // +x: from the landmark back to the sensor.
  tf2::Vector3 to_sensor(-detected.position.x, -detected.position.y, -detected.position.z);
  if (to_sensor.length() < kMinLength) {
    return std::nullopt;
  }
  to_sensor.normalize();

  // +z: the detected up, less its part along the ray.
  const tf2::Quaternion detected_q(
    detected.orientation.x, detected.orientation.y, detected.orientation.z,
    detected.orientation.w);
  if (detected_q.length2() < kMinLength) {
    return std::nullopt;
  }
  const tf2::Vector3 detected_up = tf2::Matrix3x3(detected_q.normalized()).getColumn(2);
  tf2::Vector3 up = detected_up - to_sensor * detected_up.dot(to_sensor);
  if (up.length() < kMinLength) {
    return std::nullopt;
  }
  up.normalize();

  // +y completes the right-handed frame.
  const tf2::Vector3 left = up.cross(to_sensor);
  tf2::Quaternion facing_q;
  tf2::Matrix3x3(
    to_sensor.x(), left.x(), up.x(),
    to_sensor.y(), left.y(), up.y(),
    to_sensor.z(), left.z(), up.z()).getRotation(facing_q);
  facing_q.normalize();

  geometry_msgs::msg::Pose facing;
  facing.position = detected.position;
  facing.orientation.x = facing_q.x();
  facing.orientation.y = facing_q.y();
  facing.orientation.z = facing_q.z();
  facing.orientation.w = facing_q.w();
  return facing;
}

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__LANDMARK_FACING_HPP_
