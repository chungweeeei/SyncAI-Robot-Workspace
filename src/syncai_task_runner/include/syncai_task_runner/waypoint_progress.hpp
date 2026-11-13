#ifndef SYNCAI_TASK_RUNNER__WAYPOINT_PROGRESS_HPP_
#define SYNCAI_TASK_RUNNER__WAYPOINT_PROGRESS_HPP_

// Which waypoints of a patrol route the robot has passed (2026-11).
//
// The navigator drops a passed waypoint from the front of remaining_goals
// (blackboard_keys.hpp), so a replan mid-route starts from the next one. Until
// 2026-11 "passed" was one test: the robot within waypoint_passed_radius of
// the waypoint, on its floor. A waypoint is passed through, not stopped on,
// and the route only comes within the planner's snap of it (up to
// snap_xy_radius, 0.5 m, round an obstacle) with the controller's tracking
// error on top -- more on an MPPI detour round a person standing there. A
// robot that went by further off than the radius never passed that waypoint,
// and since the list is dropped from the front only, in order, no later one
// either: it stayed at the head of the list, every waypoint the robot then
// drove through stayed behind it, and the next replan (a blocked path) went
// back to the missed one and through all of them again, while
// number_of_poses_remaining stood still.
//
// So there is a second test, on the path being driven: a waypoint is also
// passed once the robot's place on the path (forward only,
// syncai_util/path_progress.hpp) is beyond the path's own pass of it -- the
// nearest the path comes to the waypoint the first time it enters the
// waypoint's disc. The path is the planner's route through these waypoints
// in order, so its pass of each lies after its pass of the one before:
// `path_from` carries that lower bound from one waypoint to the next (and
// from one call to the next; the caller resets it for a new path), which is
// what keeps A -> B -> A's second A from matching the path's start.
//
// It cannot drop a waypoint the radius test would not: the pass is looked for
// inside the same radius and z tolerance, of the path instead of the robot.
// A path that never enters a waypoint's disc (it is not the plan through
// these waypoints: none yet, or the previous goal's) passes nothing.
//
// ROS-free apart from the message types; test/test_waypoint_progress.cpp.

#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace syncai_task_runner
{

struct WaypointPassParams
{
  // Planar distance (m) within which the robot, or the path, is at a waypoint.
  double radius = 1.0;
  // ... and the height difference (m) within which it is on the waypoint's
  // floor: a waypoint's z names its floor.
  double z_tolerance = 1.0;
};

namespace waypoint_progress_detail
{

inline double planarDistance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace waypoint_progress_detail

// The index of `path`, at or after `from`, where it passes `waypoint`: the
// nearest pose of the first run of poses within p.radius of the waypoint and
// p.z_tolerance of its height. path.poses.size() when it never comes that
// near from there on.
inline size_t waypointPassIndex(
  const nav_msgs::msg::Path & path, size_t from, const geometry_msgs::msg::Pose & waypoint,
  const WaypointPassParams & p)
{
  const size_t n = path.poses.size();
  const auto distanceIfNear = [&](size_t i, double & d) {
    const auto & pos = path.poses[i].pose.position;
    d = waypoint_progress_detail::planarDistance(pos, waypoint.position);
    return d <= p.radius && std::fabs(pos.z - waypoint.position.z) <= p.z_tolerance;
  };
  for (size_t i = from; i < n; ++i) {
    double best_d = 0.0;
    if (!distanceIfNear(i, best_d)) {
      continue;
    }
    size_t best = i;
    for (size_t k = i + 1; k < n; ++k) {
      double d = 0.0;
      if (!distanceIfNear(k, d)) {
        break;  // the run ends: a later pass is another leg's
      }
      if (d < best_d) {
        best_d = d;
        best = k;
      }
    }
    return best;
  }
  return n;
}

// How many waypoints at the front of `remaining` the robot has passed, in
// order; never the last one, which is the goal. A waypoint is passed when the
// robot is at it (within p.radius of `robot` on x / y, and its z within
// p.z_tolerance of `ground_z`, the height of the ground under the robot), or
// when `place`, the robot's index on `path`, is beyond the path's pass of it
// (waypointPassIndex() from `path_from`). `path_from` is moved to each passed
// waypoint's place on the path; the caller keeps it between calls and sets it
// to 0 for a new path or a new route.
inline size_t countPassedWaypoints(
  const std::vector<geometry_msgs::msg::PoseStamped> & remaining,
  const geometry_msgs::msg::Pose & robot, double ground_z, const nav_msgs::msg::Path & path,
  size_t place, size_t & path_from, const WaypointPassParams & p)
{
  size_t passed = 0;
  while (remaining.size() - passed > 1) {
    const geometry_msgs::msg::Pose & waypoint = remaining[passed].pose;
    const size_t pass = waypointPassIndex(path, path_from, waypoint, p);
    const bool on_path = pass < path.poses.size();
    const bool robot_at_it =
      waypoint_progress_detail::planarDistance(waypoint.position, robot.position) <= p.radius &&
      std::fabs(waypoint.position.z - ground_z) <= p.z_tolerance;
    const bool gone_by = on_path && place > pass;
    if (!robot_at_it && !gone_by) {
      break;
    }
    if (on_path) {
      path_from = pass;
    }
    ++passed;
  }
  return passed;
}

}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__WAYPOINT_PROGRESS_HPP_
