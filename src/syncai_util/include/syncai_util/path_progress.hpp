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
// ROS-free apart from the message types; test/test_path_progress.cpp.

#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

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

  // `max_search_dist`: how much path length past the last place one update
  // reads, at most. It caps the work, and it is far beyond what a robot
  // covers between two updates (the slowest caller asks at 1 Hz).
  explicit PathProgress(double hysteresis = kDefaultHysteresis, double max_search_dist = 10.0)
  : hysteresis_(hysteresis), max_search_dist_(max_search_dist)
  {
  }

  void reset()
  {
    index_ = 0;
    has_path_ = false;
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

    auto begin = path.poses.begin() + static_cast<std::ptrdiff_t>(index_);
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
    index_ = static_cast<size_t>(std::distance(path.poses.begin(), it));
    return index_;
  }

  size_t index() const { return index_; }

private:
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
  }

  double hysteresis_;
  double max_search_dist_;
  size_t index_{0};
  bool has_path_{false};
  builtin_interfaces::msg::Time stamp_;
  size_t size_{0};
  double end_x_{0.0};
  double end_y_{0.0};
};

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__PATH_PROGRESS_HPP_
