// Port of nav2_graceful_controller/ego_polar_coords.hpp (Humble, 1.1.x),
// Copyright (c) 2023 Alberto J. Tudela Roldán, Apache-2.0. Namespace and
// include paths renamed; the math is upstream's.

#ifndef SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__EGO_POLAR_COORDS_HPP_
#define SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__EGO_POLAR_COORDS_HPP_

#include <cmath>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
// tf2::getYaw() on a geometry_msgs quaternion needs the fromMsg() overload
// this header provides in the same translation unit; without it a Release
// build links but dies with a symbol lookup error at load (see
// simple_progress_checker.cpp).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_controller
{

/**
 * @brief Egocentric polar coordinates of a target pose as seen from the robot:
 * the three numbers Park & Kuipers' smooth control law is written in.
 *
 *   r      distance from the robot to the target
 *   phi    the target's heading relative to the line of sight robot -> target
 *   delta  the robot's heading relative to that same line of sight
 *
 * Both poses are given in one common frame (the docking controller passes the
 * target in the robot's base frame and leaves `current` at the origin), and
 * the coordinates come out the same whichever frame that is. With `backward`
 * the line of sight is flipped by pi, so the law steers the robot's tail at
 * the target instead of its nose. Pass the target's own orientation then:
 * this flip already covers phi as well as delta, and a target yaw flipped by
 * the caller too puts phi pi off and makes a straight reverse unstable (the
 * DockingController's undock bug, fixed 2026-10-15).
 */
struct EgocentricPolarCoordinates
{
  float r;      // Radial distance between the robot pose and the target pose.
  float phi;    // Orientation of target with respect to the line of sight
                // from the robot to the target.
  float delta;  // Steering angle of the robot with respect to the line of sight.

  EgocentricPolarCoordinates(
    const float & r_in = 0.0, const float & phi_in = 0.0, const float & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in)
  {
  }

  /**
   * @brief Construct the egocentric polar coordinates of `target` relative to
   * `current`, both referenced to the same frame.
   * @param target Target pose.
   * @param current Current pose. Defaults to the origin.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current = geometry_msgs::msg::Pose(), bool backward = false)
  {
    // Compute the difference between the target and the current pose
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    float line_of_sight = backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
    // Compute the ego polar coordinates
    r = std::sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(tf2::getYaw(current.orientation) + line_of_sight);
  }
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PLUGINS__DOCKING_CONTROLLER__EGO_POLAR_COORDS_HPP_
