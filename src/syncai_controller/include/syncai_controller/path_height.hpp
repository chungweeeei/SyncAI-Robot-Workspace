#ifndef SYNCAI_CONTROLLER__PATH_HEIGHT_HPP_
#define SYNCAI_CONTROLLER__PATH_HEIGHT_HPP_

// Height awareness for paths that leave the floor (2026-10).
//
// Every 2D planner here writes z = 0 on every pose, and the controllers were
// written for that: they match the robot to the path, prune it and decide
// "goal reached" on x / y alone. OctomapPlanner3D ("Octo3D") writes the
// ground height instead, and its stair links make that matter. The
// dp1f_1012 stair link is a spiral: going 1F -> 2F the path is back over the
// same x / y every 8.1 m, 2.5 m higher. Matched on x / y, the robot is as
// close to the turn above it as to the one it stands on, and RPP's 10 m
// search window holds both, so it could prune a whole turn and steer by the
// wrong one; a 2F goal above a 1F stretch of the route could likewise be
// "reached" from 1F. So when a path carries heights, the robot is matched
// to it in 3D.
//
// The robot's height cannot come from the controller's own pose: lio_bridge
// projects odom -> base_link onto the plane (z = 0), and map -> odom too.
// The one frame that still climbs is Point-LIO's body, the lidar, so the
// ground under the robot is that frame's z in the path's frame minus the
// lidar's height above the feet -- the same lookup OctomapPlanner3D uses for
// its start height (`body_frame` / `lidar_height` in planner_server_params).

#include <algorithm>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace syncai_controller
{

// A path "has height" when its poses do not all share one z. A flat path from
// Octo3D on a level floor (z spread of a cell or two) counts too, which is
// harmless: on one level a constant height offset changes no nearest-pose
// decision. A plan of all-equal z (every 2D planner) keeps the controllers'
// planar behaviour bit for bit, including on 2F, where a 3D distance to a
// z = 0 path would put every pose 5 m away.
inline bool pathHasHeight(const nav_msgs::msg::Path & path, double min_span = 1e-3)
{
  if (path.poses.empty()) {
    return false;
  }
  double lo = path.poses.front().pose.position.z;
  double hi = lo;
  for (const auto & ps : path.poses) {
    lo = std::min(lo, ps.pose.position.z);
    hi = std::max(hi, ps.pose.position.z);
  }
  return hi - lo > min_span;
}

// Ground height under the robot in `frame`: the origin of `body_frame` (the
// lidar) minus `body_height` (its height above the feet). Latest transform:
// the body frame is published at the LIO rate, and a height a scan old is
// centimetres off on a stair, far inside what it is used to tell apart.
inline bool lookupGroundZ(
  const tf2_ros::Buffer & tf, const std::string & frame, const std::string & body_frame,
  double body_height, double & ground_z, std::string * error = nullptr)
{
  try {
    const auto t = tf.lookupTransform(frame, body_frame, tf2::TimePointZero);
    ground_z = t.transform.translation.z - body_height;
    return true;
  } catch (const tf2::TransformException & ex) {
    if (error) {
      *error = ex.what();
    }
    return false;
  }
}

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__PATH_HEIGHT_HPP_
