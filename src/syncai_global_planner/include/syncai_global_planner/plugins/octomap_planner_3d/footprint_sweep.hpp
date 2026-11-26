#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__FOOTPRINT_SWEEP_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__FOOTPRINT_SWEEP_HPP_

// The robot's own footprint swept over the motions a stair link lays
// (2026-11-26). ROS-free and header-only, like the rest of the core.
//
// Why: the surface map and the A* see the robot as a disc of robot_radius
// (the padded footprint's inscribed radius), which is right for a round
// robot and good enough on the floor for the G23 (0.70 x 0.44 m), but the
// Lynx M20 is 0.82 x 0.48 m -- 1.7 to 1 -- and a disc of its half-width
// leaves out the 0.41 m of body either end. On a stair link that mattered
// most: a U landing's corners were placed with a disc of
// stair_flights_turn_radius round each turn and robot_radius either side of
// the leg between them, and on robot01's first descent of dp_1125's
// switchback stairwell (2026-11-26) the M20's side came within 0.02-0.17 m
// of the landings' inner walls and newels, while on the flights, which the
// planner centres, it had 0.35-0.55 m either side. On a stair link the
// motion is fully known -- straight along a flight's axis, stop, turn in
// place, straight across the landing, turn, straight on -- so the real
// polygon can be swept over it exactly, which the floor search, with no
// heading in its state, cannot do.
//
// Distances are signed: positive is the gap between the polygon and the
// nearest obstacle column's near edge, negative how far that column reaches
// into the polygon. Obstacle columns are given as their centres; half a cell
// is taken off for the near edge, as landingClearance does.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace syncai_global_planner::octomap_planner_3d
{

using FootprintPoints = std::vector<std::pair<double, double>>;

// "[[x, y], [x, y], ...]" (m, base_link frame: x forward, y left), the
// format of the costmaps' `footprint` parameter, so the planner's value can
// be the local costmap's string verbatim. At least three points. False, with
// error set, on anything else; "" and "[]" parse to an empty footprint.
inline bool parseFootprint(const std::string & s, FootprintPoints & out, std::string & error)
{
  out.clear();
  std::vector<double> nums;
  int depth = 0, max_depth = 0;
  for (size_t i = 0; i < s.size();) {
    const char c = s[i];
    if (c == '[') {
      max_depth = std::max(max_depth, ++depth);
      ++i;
    } else if (c == ']') {
      if (--depth < 0) {
        error = "unbalanced ']'";
        return false;
      }
      ++i;
    } else if (c == ',' || std::isspace(static_cast<unsigned char>(c))) {
      ++i;
    } else {
      size_t used = 0;
      double v = 0.0;
      try {
        v = std::stod(s.substr(i), &used);
      } catch (...) {
        error = "not a number at '" + s.substr(i, 12) + "'";
        return false;
      }
      if (depth != 2) {
        error = "a number outside a [x, y] pair";
        return false;
      }
      nums.push_back(v);
      i += used;
    }
  }
  if (depth != 0) {
    error = "unbalanced '['";
    return false;
  }
  if (nums.empty()) {
    return true;
  }
  if (max_depth != 2 || nums.size() % 2 != 0 || nums.size() < 6) {
    error = "want at least three [x, y] points";
    return false;
  }
  for (size_t i = 0; i < nums.size(); i += 2) {
    out.push_back({nums[i], nums[i + 1]});
  }
  return true;
}

inline std::string footprintString(const FootprintPoints & fp)
{
  std::string s = "[";
  char buf[48];
  for (size_t i = 0; i < fp.size(); ++i) {
    std::snprintf(buf, sizeof buf, "%s[%.3g,%.3g]", i ? "," : "", fp[i].first, fp[i].second);
    s += buf;
  }
  return s + "]";
}

// Furthest a vertex lies from base_link: the disc the polygon turns in.
inline double footprintRadius(const FootprintPoints & fp)
{
  double r = 0.0;
  for (const auto & [x, y] : fp) {
    r = std::max(r, std::hypot(x, y));
  }
  return r;
}

// Furthest the polygon reaches along its own x (forward / back) and y (to
// the side): what a straight run along a flight leaves either side of it.
inline double footprintReachX(const FootprintPoints & fp)
{
  double r = 0.0;
  for (const auto & pt : fp) {
    r = std::max(r, std::fabs(pt.first));
  }
  return r;
}
inline double footprintReachY(const FootprintPoints & fp)
{
  double r = 0.0;
  for (const auto & pt : fp) {
    r = std::max(r, std::fabs(pt.second));
  }
  return r;
}

// Signed distance from the point (px, py), in the polygon's own frame, to
// the polygon: positive outside, negative inside.
inline double polygonSignedDistance(const FootprintPoints & fp, double px, double py)
{
  double best = std::numeric_limits<double>::infinity();
  bool inside = false;
  const size_t n = fp.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double ax = fp[j].first, ay = fp[j].second, bx = fp[i].first, by = fp[i].second;
    const double ex = bx - ax, ey = by - ay;
    const double len2 = ex * ex + ey * ey;
    double t = len2 > 0.0 ? ((px - ax) * ex + (py - ay) * ey) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    best = std::min(best, std::hypot(px - (ax + t * ex), py - (ay + t * ey)));
    if ((ay > py) != (by > py) && px < (bx - ax) * (py - ay) / (by - ay) + ax) {
      inside = !inside;
    }
  }
  return inside ? -best : best;
}

// A footprint over a fixed set of obstacle columns. Every query is taken at
// the given heading and at the heading + pi: a stair link is walked both
// ways, and the G23 goes down a flight backwards (HIMLOCO), so which end of
// the body leads is not known here.
class FootprintSweep
{
public:
  // obstacles: column centres (m); cell_half: half a cell, taken off for a
  // column's near edge; cap: a clearance larger than this reads as cap.
  FootprintSweep(
    FootprintPoints fp, std::vector<std::pair<double, double>> obstacles, double cell_half,
    double cap)
  : fp_(std::move(fp)),
    obstacles_(std::move(obstacles)),
    cell_half_(cell_half),
    cap_(cap),
    radius_(footprintRadius(fp_))
  {
  }

  double cap() const { return cap_; }

  // Clearance of the polygon at (x, y, yaw), and at yaw + pi.
  double pose(double x, double y, double yaw) const
  {
    const double reach = radius_ + cap_ + cell_half_;
    double best = cap_;
    for (const double a : {yaw, yaw + M_PI}) {
      const double c = std::cos(a), s = std::sin(a);
      for (const auto & [ox, oy] : obstacles_) {
        const double dx = ox - x, dy = oy - y;
        if (std::fabs(dx) > reach || std::fabs(dy) > reach) {
          continue;
        }
        // Into the polygon's frame.
        const double u = c * dx + s * dy, v = -s * dx + c * dy;
        best = std::min(best, polygonSignedDistance(fp_, u, v) - cell_half_);
      }
    }
    return best;
  }

  // A turn in place at (x, y) from yaw0 to yaw1 the short way round, each
  // end overshot by tol (the turn stops late on a robot whose pose lags),
  // sampled every step rad.
  double turn(double x, double y, double yaw0, double yaw1, double tol, double step = 0.087) const
  {
    double d = std::remainder(yaw1 - yaw0, 2.0 * M_PI);
    const double sgn = d >= 0.0 ? 1.0 : -1.0;
    const double from = yaw0 - sgn * tol;
    d += 2.0 * sgn * tol;
    const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(d) / step)));
    double best = cap_;
    for (int k = 0; k <= n; ++k) {
      best = std::min(best, pose(x, y, from + d * k / n));
    }
    return best;
  }

  // A straight run from (x0, y0) to (x1, y1) facing yaw, give or take tol,
  // sampled every step m.
  double segment(
    double x0, double y0, double x1, double y1, double yaw, double tol, double step) const
  {
    const double len = std::hypot(x1 - x0, y1 - y0);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    double best = cap_;
    for (int k = 0; k <= n; ++k) {
      const double a = static_cast<double>(k) / n;
      const double x = x0 + a * (x1 - x0), y = y0 + a * (y1 - y0);
      for (const double e : {-tol, 0.0, tol}) {
        best = std::min(best, pose(x, y, yaw + e));
      }
    }
    return best;
  }

private:
  FootprintPoints fp_;
  std::vector<std::pair<double, double>> obstacles_;
  double cell_half_, cap_, radius_;
};

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__FOOTPRINT_SWEEP_HPP_
