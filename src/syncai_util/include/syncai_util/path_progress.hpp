#ifndef SYNCAI_UTIL__PATH_PROGRESS_HPP_
#define SYNCAI_UTIL__PATH_PROGRESS_HPP_

// Where the robot is along a path that may pass back over itself (2026-11).
//
// Every consumer of a path used to place the robot on it by the nearest pose
// in a window: the whole path (controller_server's goal gate and feedback,
// planner_server's `plan` trim and is_path_valid) or 10 m of it (MPPI, RPP
// 3D). That holds while a path never comes back near itself. A patrol route
// (navigate_through_poses) does by design: A -> B -> A runs the same corridor
// twice, the two legs a cell or two apart, and A -> B -> C -> A ends where it
// starts. The nearest pose then flips between legs: within 5 m of B the
// return leg is inside a 10 m window and as close as the outgoing one, so MPPI
// could prune onto the return leg and turn back short of B (its min_by keeps
// the later pose on a tie); on the loop a whole-path search can put a robot at
// A on the first pose (remaining length the whole loop: the goal check never
// passes) or on the last (it passes at the start). Reasoned from the code
// before the first patrol ran, not observed.
//
// The fix is to read the path forward from where the robot was last placed
// and stop at the FIRST local minimum of the distance, not the global one: the
// distance from the robot falls towards its place on the leg it is walking,
// then rises, and only falls again on a later leg -- which is never reached
// once the rise exceeds `hysteresis`. The hysteresis absorbs the ripple of a
// path sampled every 0.1 m and of a robot off to one side of it (an MPPI
// detour): along a straight stretch the distance is convex, so any rise is
// real, and 0.3 m is well under the separation that makes two legs two legs.
// A robot within the hysteresis of a U-turn's tip may read as already on the
// way back, which is what it is about to be.
//
// It passes a U-turn only once the robot has been within that hysteresis of
// the tip, and nothing made a controller go there: on robot01 (2026-11-02)
// MPPI, handed an out-and-back whole, stopped 0.84 m short of the tip for good
// and every place on the path stayed on the outgoing leg. The controller
// server therefore cuts a path at its cusps and drives it leg by leg
// (path_cusps.hpp); each leg ends on its tip, so the robot gets there.
//
// Gets there, but not to within the hysteresis for sure: the goal checker
// latches the position at xy_goal_tolerance (0.10 m) and the turn in place
// that follows is not held on the spot -- in the sim it ended 0.29 m from
// the tip, 0.01 m inside. A place left on the outgoing leg never moves
// again (every pose after it is further from a robot walking back), so the
// controller server's goal gate stayed shut for the whole return leg and
// the goal was never reached; the planner's `plan` trim and is_path_valid
// and the task_runner's feedback, which hold the same whole path, stayed on
// the outgoing leg with it. So PathProgress passes a cusp of its own
// (2026-11): once the robot is within `cusp_pass_radius` of the next cusp
// and faces the way the path leaves it, its place is moved onto the cusp and
// searched forward from there. The heading tells a robot that has turned
// round from one still walking up to the tip, which faces the other way.
// passTo() is the same move for a caller that knows the robot has passed a
// pose (the controller server, on handing over the next leg).
//
// ROS-free apart from the message types; test/test_path_progress.cpp.

#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "syncai_util/path_cusps.hpp"

namespace syncai_util
{

// The pose in [begin, end) nearest the robot by the first-local-minimum rule
// above. `dist` maps a pose to its distance from the robot. Ties keep the
// earlier pose. Returns `end` only for an empty range.
template <typename Iter, typename DistFn>
inline Iter firstLocalMinimum(Iter begin, Iter end, DistFn dist, double hysteresis)
{
  if (begin == end) {
    return end;
  }
  Iter best = begin;
  double best_dist = dist(*begin);
  for (Iter it = std::next(begin); it != end; ++it) {
    const double d = dist(*it);
    if (d < best_dist) {
      best_dist = d;
      best = it;
    } else if (d > best_dist + hysteresis) {
      break;
    }
  }
  return best;
}

inline double poseDistance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b, bool is_3d)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = is_3d ? a.position.z - b.position.z : 0.0;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// The robot's place on one path, kept between calls so it only moves forward.
// For the consumers that hold the whole path and are asked repeatedly
// (controller_server, planner_server); MPPI and RPP 3D prune their copy
// instead, so the pruned front is their memory and they call
// firstLocalMinimum() from it.
class PathProgress
{
public:
  // The rise past the robot's place that ends the search; see the header.
  static constexpr double kDefaultHysteresis = 0.3;
  // How near the next cusp a robot facing the way the path leaves it must be
  // for its place to move onto the cusp; see the header. Twice the 0.29 m
  // the sim's turn in place drifted, and still well under any leg.
  static constexpr double kDefaultCuspPassRadius = 0.6;

  // `max_search_dist`: how much path length past the last place one update
  // reads, at most. It caps the work, and it is far beyond what a robot
  // covers between two updates (the slowest caller asks at 1 Hz).
  explicit PathProgress(
    double hysteresis = kDefaultHysteresis, double max_search_dist = 10.0,
    double cusp_pass_radius = kDefaultCuspPassRadius)
  : hysteresis_(hysteresis), max_search_dist_(max_search_dist), cusp_pass_radius_(cusp_pass_radius)
  {
  }

  void reset()
  {
    index_ = 0;
    has_path_ = false;
    cusps_.clear();
  }

  // The cusps' definition, the controller server's own (its cusp_min_angle /
  // cusp_window), so that it passes the cusps it cuts its legs at. Read at
  // the next new path. Every other caller keeps CuspParams' defaults, which
  // are the server's defaults.
  void setCuspParams(const CuspParams & params) { cusp_params_ = params; }

  // The robot is known to be at or past `index` of the current path: the
  // place moves there if it is behind. Never backwards.
  void passTo(size_t index)
  {
    if (has_path_ && index < size_ && index > index_) {
      index_ = index;
    }
  }

  // Index of the robot's pose on `path`. A path other than the last one (by
  // stamp, size and end pose: a replan always restamps, see planner_server's
  // publishPlan) starts again from its first pose. `robot` must be in the
  // path's frame, with the ground height under the robot as z when `is_3d`.
  size_t update(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot, bool is_3d)
  {
    if (path.poses.empty()) {
      reset();
      return 0;
    }
    if (!samePath(path)) {
      remember(path);
      index_ = 0;
    }
    if (index_ >= path.poses.size()) {
      index_ = path.poses.size() - 1;
    }

    index_ = search(path, robot, is_3d, index_);
    // Past the next cusp when the robot is at it, turned round (header). In
    // a loop for a route with cusps closer together than a robot's step.
    for (const Cusp & cusp : cusps_) {
      if (cusp.index <= index_) {
        continue;
      }
      if (!atAndTurned(path, cusp, robot, is_3d)) {
        break;
      }
      index_ = search(path, robot, is_3d, cusp.index);
    }
    return index_;
  }

  size_t index() const { return index_; }

private:
  struct Cusp
  {
    size_t index;
    double leave_x;  // unit vector of the direction the path leaves it in
    double leave_y;
  };

  // The first-local-minimum place from `from` on, within max_search_dist_.
  size_t search(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot, bool is_3d,
    size_t from) const
  {
    auto begin = path.poses.begin() + static_cast<std::ptrdiff_t>(from);
    auto end = begin;
    double length = 0.0;
    for (++end; end != path.poses.end(); ++end) {
      length += poseDistance(std::prev(end)->pose, end->pose, false);
      if (length > max_search_dist_) {
        break;
      }
    }
    auto it = firstLocalMinimum(
      begin, end,
      [&robot, is_3d](const geometry_msgs::msg::PoseStamped & ps) {
        return poseDistance(robot, ps.pose, is_3d);
      },
      hysteresis_);
    return static_cast<size_t>(std::distance(path.poses.begin(), it));
  }

  // Within cusp_pass_radius_ of the cusp, facing within 90 deg of the way the
  // path leaves it. Planar yaw: every caller's robot pose is a base_link one.
  bool atAndTurned(
    const nav_msgs::msg::Path & path, const Cusp & cusp, const geometry_msgs::msg::Pose & robot,
    bool is_3d) const
  {
    if (poseDistance(robot, path.poses[cusp.index].pose, is_3d) > cusp_pass_radius_) {
      return false;
    }
    const auto & q = robot.orientation;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return std::cos(yaw) * cusp.leave_x + std::sin(yaw) * cusp.leave_y > 0.0;
  }

  bool samePath(const nav_msgs::msg::Path & path) const
  {
    return has_path_ && path.header.stamp == stamp_ && path.poses.size() == size_ &&
           path.poses.back().pose.position.x == end_x_ &&
           path.poses.back().pose.position.y == end_y_;
  }

  void remember(const nav_msgs::msg::Path & path)
  {
    has_path_ = true;
    stamp_ = path.header.stamp;
    size_ = path.poses.size();
    end_x_ = path.poses.back().pose.position.x;
    end_y_ = path.poses.back().pose.position.y;
    cusps_.clear();
    if (cusp_pass_radius_ > 0.0) {
      for (const size_t c : findCusps(path, cusp_params_)) {
        double yaw = 0.0;
        if (directionAfterCusp(path, c, cusp_params_.window, yaw)) {
          cusps_.push_back({c, std::cos(yaw), std::sin(yaw)});
        }
      }
    }
  }

  double hysteresis_;
  double max_search_dist_;
  double cusp_pass_radius_;
  CuspParams cusp_params_;
  std::vector<Cusp> cusps_;
  size_t index_{0};
  bool has_path_{false};
  builtin_interfaces::msg::Time stamp_;
  size_t size_{0};
  double end_x_{0.0};
  double end_y_{0.0};
};

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__PATH_PROGRESS_HPP_
