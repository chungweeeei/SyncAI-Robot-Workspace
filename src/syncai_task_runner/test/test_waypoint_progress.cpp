// Which waypoints of a patrol route are passed (waypoint_progress.hpp), on
// synthetic routes: the robot at a waypoint, the robot gone by wide of one,
// an out-and-back route whose later waypoint lies on the path's start, and
// two floors. Header-only, no ROS node.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "syncai_task_runner/waypoint_progress.hpp"

namespace
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using syncai_task_runner::countPassedWaypoints;
using syncai_task_runner::waypointPassIndex;
using syncai_task_runner::WaypointPassParams;

constexpr double kStep = 0.1;

Pose pose(double x, double y, double z = 0.0)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  return p;
}

PoseStamped waypoint(double x, double y, double z = 0.0)
{
  PoseStamped p;
  p.pose = pose(x, y, z);
  return p;
}

// Poses every kStep along the polyline through `corners` (x, y), at height z.
Path polyline(const std::vector<std::pair<double, double>> & corners, double z = 0.0)
{
  Path path;
  for (size_t i = 0; i + 1 < corners.size(); ++i) {
    const double dx = corners[i + 1].first - corners[i].first;
    const double dy = corners[i + 1].second - corners[i].second;
    const int n = static_cast<int>(std::lround(std::hypot(dx, dy) / kStep));
    for (int k = (i == 0 ? 0 : 1); k <= n; ++k) {
      PoseStamped p;
      p.pose = pose(corners[i].first + dx * k / n, corners[i].second + dy * k / n, z);
      path.poses.push_back(p);
    }
  }
  return path;
}

// The path's index nearest (x, y), searched from `from`: where a robot
// standing there is on the path.
size_t placeAt(const Path & path, double x, double y, size_t from = 0)
{
  size_t best = from;
  double best_d = 1e9;
  for (size_t i = from; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - x, p.y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

const WaypointPassParams kParams{1.0, 1.0};

}  // namespace

// The path's pass of a waypoint is its nearest pose in the first run inside
// the waypoint's disc.
TEST(WaypointProgress, PassIndexIsTheNearestPoseOfTheFirstRun)
{
  const Path path = polyline({{0.0, 0.0}, {10.0, 0.0}});
  // A waypoint 0.4 m beside the line at x = 5: nearest pose is index 50.
  EXPECT_EQ(waypointPassIndex(path, 0, pose(5.0, 0.4), kParams), 50u);
  // Searched from beyond the run: not found.
  EXPECT_EQ(waypointPassIndex(path, 70, pose(5.0, 0.4), kParams), path.poses.size());
  // Further from the line than the radius: never passed by this path.
  EXPECT_EQ(waypointPassIndex(path, 0, pose(5.0, 1.5), kParams), path.poses.size());
  // On another floor: not this path's waypoint.
  EXPECT_EQ(waypointPassIndex(path, 0, pose(5.0, 0.0, 3.0), kParams), path.poses.size());
}

// The robot within the radius of the next waypoint, on its floor: passed, as
// before. The last waypoint never is.
TEST(WaypointProgress, RobotAtTheWaypoint)
{
  const Path path = polyline({{0.0, 0.0}, {10.0, 0.0}});
  const std::vector<PoseStamped> remaining = {waypoint(5.0, 0.0), waypoint(10.0, 0.0)};
  size_t from = 0;

  // 2 m short of it: nothing.
  EXPECT_EQ(
    countPassedWaypoints(
      remaining, pose(3.0, 0.0), 0.0, path, placeAt(path, 3.0, 0.0), from, kParams),
    0u);
  // 0.8 m short of it: passed.
  EXPECT_EQ(
    countPassedWaypoints(
      remaining, pose(4.2, 0.0), 0.0, path, placeAt(path, 4.2, 0.0), from, kParams),
    1u);
  // Standing on the goal: the last one stays.
  from = 0;
  const std::vector<PoseStamped> last = {waypoint(10.0, 0.0)};
  EXPECT_EQ(
    countPassedWaypoints(last, pose(10.0, 0.0), 0.0, path, placeAt(path, 10.0, 0.0), from, kParams),
    0u);
  // On the floor below the waypoint, x / y on it: not passed.
  from = 0;
  const std::vector<PoseStamped> upstairs = {waypoint(5.0, 0.0, 3.0), waypoint(10.0, 0.0, 3.0)};
  EXPECT_EQ(
    countPassedWaypoints(
      upstairs, pose(5.0, 0.0), 0.0, path, placeAt(path, 5.0, 0.0), from, kParams),
    0u);
}

// The case the path test exists for: the robot goes by a waypoint wide of the
// radius (a detour round someone standing on it) and carries on. By the
// radius alone the waypoint stayed at the head of the list and kept every
// later one behind it.
TEST(WaypointProgress, GoneByWideOfTheRadius)
{
  const Path path = polyline({{0.0, 0.0}, {12.0, 0.0}});
  const std::vector<PoseStamped> remaining = {
    waypoint(4.0, 0.0), waypoint(8.0, 0.0), waypoint(12.0, 0.0)};
  size_t from = 0;

  // Abeam of the first waypoint, 1.5 m off the path: not yet beyond its pass.
  EXPECT_EQ(
    countPassedWaypoints(
      remaining, pose(4.0, 1.5), 0.0, path, placeAt(path, 4.0, 0.0), from, kParams),
    0u);
  EXPECT_EQ(from, 0u);
  // Half a metre on, still 1.5 m off: beyond the pass, so passed -- and only
  // that one.
  EXPECT_EQ(
    countPassedWaypoints(
      remaining, pose(4.5, 1.5), 0.0, path, placeAt(path, 4.5, 0.0), from, kParams),
    1u);
  EXPECT_EQ(from, 40u);

  // Never noticed until past the second one too: both go, in order.
  from = 0;
  EXPECT_EQ(
    countPassedWaypoints(
      remaining, pose(9.5, 1.5), 0.0, path, placeAt(path, 9.5, 0.0), from, kParams),
    2u);
  EXPECT_EQ(from, 80u);
}

// A -> B -> A -> C: the route's second waypoint lies on the path's start.
// Its pass is looked for from the first waypoint's on, so a robot that has
// just turned round at B has not passed A.
TEST(WaypointProgress, OutAndBackDoesNotMatchThePathsStart)
{
  // Out along y = 0 to B (5, 0), back along y = 0.2 to A (0, 0.2), on to C.
  const Path path = polyline({{0.0, 0.0}, {5.0, 0.0}, {5.0, 0.2}, {0.0, 0.2}, {0.0, 4.0}});
  const std::vector<PoseStamped> remaining = {
    waypoint(5.0, 0.0), waypoint(0.0, 0.0), waypoint(0.0, 4.0)};
  size_t from = 0;

  // At B: B is passed. A is not, although the path's first pose is on it.
  const size_t at_b = placeAt(path, 5.0, 0.0);
  EXPECT_EQ(countPassedWaypoints(remaining, pose(5.0, 0.0), 0.0, path, at_b, from, kParams), 1u);
  EXPECT_EQ(from, at_b);

  // On the way back, 2 m from A: A still ahead.
  const std::vector<PoseStamped> after_b = {waypoint(0.0, 0.0), waypoint(0.0, 4.0)};
  const size_t returning = placeAt(path, 2.0, 0.2, at_b);
  EXPECT_EQ(countPassedWaypoints(after_b, pose(2.0, 0.2), 0.0, path, returning, from, kParams), 0u);

  // The same question without the bound: the path's start would answer for
  // A, and the robot would have "passed" it from 2 m away.
  size_t unbounded = 0;
  EXPECT_EQ(
    countPassedWaypoints(after_b, pose(2.0, 0.2), 0.0, path, returning, unbounded, kParams), 1u);
}

// A path that does not go through the waypoints (none yet: the blackboard's
// path is empty until the first plan lands) passes nothing; the robot's own
// distance still does.
TEST(WaypointProgress, NoPath)
{
  const Path empty;
  const std::vector<PoseStamped> remaining = {waypoint(5.0, 0.0), waypoint(10.0, 0.0)};
  size_t from = 0;
  EXPECT_EQ(countPassedWaypoints(remaining, pose(0.0, 0.0), 0.0, empty, 0, from, kParams), 0u);
  EXPECT_EQ(countPassedWaypoints(remaining, pose(4.5, 0.0), 0.0, empty, 0, from, kParams), 1u);
  EXPECT_EQ(from, 0u);
}
