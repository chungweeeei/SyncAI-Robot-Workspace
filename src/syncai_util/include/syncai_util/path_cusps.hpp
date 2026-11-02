#ifndef SYNCAI_UTIL__PATH_CUSPS_HPP_
#define SYNCAI_UTIL__PATH_CUSPS_HPP_

// Where a path turns back on itself, and the path cut into legs there
// (2026-11).
//
// A patrol route (navigate_through_poses) is drawn the way an operator walks
// it: P1 -> P2 -> P3 -> P2 -> P4 runs a corridor out to P3 and straight back.
// On robot01 (2026-11-02) the robot stopped 0.84 m short of P3 and stood there
// silently until it was cancelled; the same points sent as four MOVEs went
// through. Nothing in the stack knew that a path can reverse:
//
// - Every consumer places the robot on the path forward-only
//   (path_progress.hpp), which only passes a U-turn once the robot has been
//   within its 0.3 m hysteresis of the tip -- and nothing made a controller
//   go there.
// - MPPI's critics place each candidate trajectory on the pruned path by the
//   nearest pose over all of it. Within prune_distance of the tip the return
//   leg is in view, the slow candidates match it, and PathFollowCritic's
//   carrot lands at 2 D - prune_distance (D = the robot's distance to the
//   tip): under the robot at D = 1 m, where it stopped, and PathAngleCritic
//   then turned it towards a carrot behind it.
// - RPP 3D's lookahead is the first pose L away, which for D < L is on the
//   return leg behind the robot: it turns back short of the tip.
//
// A MOVE never meets any of that, because a MOVE's path ends where the robot
// has to turn round, and the final rotation does the turn. So the
// controller_server does what the MOVEs did: it cuts the path at every cusp
// and hands the controller one leg at a time, each ending on its cusp facing
// the next leg (splitAtCusps()), and every consumer above sees a path that
// never comes back on itself. The robot stops on a cusp and turns in place --
// a robot that cannot reverse has no other way round one.
//
// A cusp is a pose where the direction `window` of path before it and the
// direction `window` after it differ by more than `min_angle`. Upstream MPPI's
// own test (findFirstPathInversion(), a negative dot product of two
// consecutive steps) is not used: an A* return leg one cell beside the
// outgoing one turns through two 90 deg corners, each a zero dot product, and
// a single 0.1 m step of a smoothed path is noise. Not where the path climbs
// more than `max_z_span` within `z_check_dist` of the pose: a switchback
// stairwell's landing turns 180 deg between two flights, and stair mode
// climbs it blind (stair_mode.hpp); its legs are a storey apart, never on top
// of each other.
//
// ROS-free apart from the message types; test/test_path_cusps.cpp.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace syncai_util
{

struct CuspParams
{
  // Arc length read before and after a pose for its two directions. Longer
  // than a cell (0.1 m) so a one-cell side step at the tip reads as one turn,
  // shorter than any leg worth driving.
  double window{0.4};
  // Turn (rad) above which the pose is a cusp. 2.0 rad ~ 115 deg: a 90 deg
  // corner is driven through, as on any MOVE; a turn this sharp is one the
  // robot has to stop for anyway. <= 0 finds none.
  double min_angle{2.0};
  // Rise (m) within z_check_dist (m) of arc either side of a pose above which
  // its turn is a stair's, not a cusp: the stair mode's rise (stair_mode.hpp,
  // 0.3 m within 1 m). Longer than `window` so that a landing narrower than
  // the window, where the turn reads 180 deg, still sees its flights.
  double max_z_span{0.3};
  double z_check_dist{1.0};
};

namespace detail
{

// Cumulative planar arc length at each pose.
inline std::vector<double> arcLengths(const nav_msgs::msg::Path & path)
{
  std::vector<double> s(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    s[i] = s[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  return s;
}

}  // namespace detail

// Indices of the cusps of `path`, ascending, never its first or last pose.
inline std::vector<size_t> findCusps(const nav_msgs::msg::Path & path, const CuspParams & params)
{
  std::vector<size_t> cusps;
  const size_t n = path.poses.size();
  if (params.min_angle <= 0.0 || params.window <= 0.0 || n < 3) {
    return cusps;
  }
  const std::vector<double> s = detail::arcLengths(path);
  const auto pos = [&path](size_t i) -> const geometry_msgs::msg::Point & {
    return path.poses[i].pose.position;
  };

  // One run of consecutive candidates is one turn. Its cusp is the pose that
  // got furthest along the direction the run was entered with -- the tip --
  // not the sharpest angle: on a U-turn every pose within window / 2 of the
  // tip reads a full 180 deg.
  size_t run_first = 0;
  size_t run_best = 0;
  double run_dir_x = 0.0;
  double run_dir_y = 0.0;
  double run_best_proj = 0.0;
  double run_last_s = 0.0;
  bool in_run = false;
  const auto close_run = [&]() {
    if (in_run) {
      cusps.push_back(run_best);
      in_run = false;
    }
  };

  size_t a = 0;   // last pose at least `window` before i
  size_t b = 0;   // first pose at least `window` after i
  size_t za = 0;  // first pose within z_check_dist before i
  size_t zb = 0;  // last pose within z_check_dist after i
  for (size_t i = 1; i + 1 < n; ++i) {
    while (a + 1 < i && s[i] - s[a + 1] >= params.window) {
      ++a;
    }
    if (b < i) {
      b = i;
    }
    while (b < n && s[b] - s[i] < params.window) {
      ++b;
    }
    if (s[i] - s[a] < params.window || b >= n) {
      continue;  // too near an end to have both directions
    }
    const double in_x = pos(i).x - pos(a).x;
    const double in_y = pos(i).y - pos(a).y;
    const double out_x = pos(b).x - pos(i).x;
    const double out_y = pos(b).y - pos(i).y;
    // The chord, not the arc: on a U-turn the pose `window` ahead can be back
    // beside this one.
    if (std::hypot(in_x, in_y) < 1e-6 || std::hypot(out_x, out_y) < 1e-6) {
      continue;
    }
    const double turn =
      std::fabs(std::atan2(in_x * out_y - in_y * out_x, in_x * out_x + in_y * out_y));
    while (s[i] - s[za] > params.z_check_dist) {
      ++za;
    }
    if (zb < i) {
      zb = i;
    }
    while (zb + 1 < n && s[zb + 1] - s[i] <= params.z_check_dist) {
      ++zb;
    }
    double z_min = pos(za).z;
    double z_max = pos(za).z;
    for (size_t k = za; k <= zb; ++k) {
      z_min = std::min(z_min, pos(k).z);
      z_max = std::max(z_max, pos(k).z);
    }
    if (turn <= params.min_angle || z_max - z_min > params.max_z_span) {
      continue;
    }

    if (in_run && s[i] - run_last_s > params.window) {
      close_run();
    }
    if (!in_run) {
      in_run = true;
      run_first = i;
      const double norm = std::hypot(in_x, in_y);
      run_dir_x = in_x / norm;
      run_dir_y = in_y / norm;
      run_best = i;
      run_best_proj = 0.0;
    }
    const double proj =
      (pos(i).x - pos(run_first).x) * run_dir_x + (pos(i).y - pos(run_first).y) * run_dir_y;
    if (proj > run_best_proj + 1e-9) {
      run_best_proj = proj;
      run_best = i;
    }
    run_last_s = s[i];
  }
  close_run();
  return cusps;
}

// The direction (yaw) the path leaves `cusp` in: towards the pose `window`
// of arc after it, or the path's last pose when the rest is shorter -- not
// the next pose, which after a one-cell side step at the tip is sideways.
// False when nothing after the cusp is away from it.
inline bool directionAfterCusp(
  const nav_msgs::msg::Path & path, size_t cusp, double window, double & yaw)
{
  if (cusp + 1 >= path.poses.size()) {
    return false;
  }
  const auto & tip = path.poses[cusp].pose.position;
  size_t ahead = cusp + 1;
  double along = 0.0;
  while (ahead + 1 < path.poses.size()) {
    const auto & p = path.poses[ahead].pose.position;
    const auto & q = path.poses[ahead - 1].pose.position;
    along += std::hypot(p.x - q.x, p.y - q.y);
    if (along >= window && std::hypot(p.x - tip.x, p.y - tip.y) > 1e-6) {
      break;
    }
    ++ahead;
  }
  const auto & next = path.poses[ahead].pose.position;
  if (std::hypot(next.x - tip.x, next.y - tip.y) <= 1e-6) {
    return false;
  }
  yaw = std::atan2(next.y - tip.y, next.x - tip.x);
  return true;
}

// `path` cut at `cusps` (findCusps()'s, ascending): leg k runs from cusp k-1
// (the path's start for k = 0) to cusp k inclusive (the path's end for the
// last), so two legs share their cusp pose. Every leg but the last ends
// facing the next leg (directionAfterCusp()), which is where the
// controller's final rotation turns the robot before the next leg is handed
// over. The last leg keeps the path's end pose. Every leg carries the path's
// header. No cusps: the path itself.
inline std::vector<nav_msgs::msg::Path> splitAtCusps(
  const nav_msgs::msg::Path & path, const std::vector<size_t> & cusps, double window)
{
  std::vector<nav_msgs::msg::Path> legs;
  size_t first = 0;
  for (const size_t cusp : cusps) {
    if (cusp <= first || cusp + 1 >= path.poses.size()) {
      continue;
    }
    nav_msgs::msg::Path leg;
    leg.header = path.header;
    leg.poses.assign(
      path.poses.begin() + static_cast<std::ptrdiff_t>(first),
      path.poses.begin() + static_cast<std::ptrdiff_t>(cusp) + 1);

    double yaw = 0.0;
    if (directionAfterCusp(path, cusp, window, yaw)) {
      auto & q = leg.poses.back().pose.orientation;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(yaw / 2.0);
      q.w = std::cos(yaw / 2.0);
    }
    legs.push_back(std::move(leg));
    first = cusp;
  }
  nav_msgs::msg::Path last;
  last.header = path.header;
  last.poses.assign(path.poses.begin() + static_cast<std::ptrdiff_t>(first), path.poses.end());
  legs.push_back(std::move(last));
  return legs;
}

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__PATH_CUSPS_HPP_
