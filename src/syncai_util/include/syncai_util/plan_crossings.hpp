#ifndef SYNCAI_UTIL__PLAN_CROSSINGS_HPP_
#define SYNCAI_UTIL__PLAN_CROSSINGS_HPP_

// What the planner knows about the terrain a plan crosses, for the nodes
// that follow it (2026-11-02): the `plan_crossings` sidecar.
//
// Why: three places used to decide on their own what a stretch of path is --
// the planner (its geometry rules and links), the controllers' stair mode
// (a rise of stair_min_rise within stair_enter_dist of the path's heights)
// and the local costmap's terrain layer (its own step and slope rules on the
// lidar's returns) -- with three copies of the same thresholds. A short
// slope the planner laid as a flat-gait crossing still stopped the robot to
// switch to the stair policy, and still read lethal to the costmap. The
// planner is the one that knows, so it says so: every crossing the plan
// takes (a stair, a kerb, a walked gap), the gait it is taken in, and where
// it is on the plan, and the followers read it instead of guessing.
//
// One JSON object on a std_msgs/String (`plan_crossings`, reliable,
// transient_local, depth 1, published by planner_server beside `plan`):
//
//   {"version": 1, "known": true,
//    "plan_stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map", "poses": 532,
//    "crossings": [{"kind": "stair", "gait": "stair", "evidence": "track",
//                   "rise": 5.4, "first": 120, "last": 341, "turns": [200, 260],
//                   "pts": [[x, y, z], ...]}]}
//
// - plan_stamp / poses identify the plan (nav_msgs/Path) it was made for:
//   the path the planner returned, as ComputePathToPose hands it to the BT
//   and FollowPath to the controller. A follower applies the indices only
//   to that plan; `plan` itself is trimmed and restamped on every republish,
//   so the terrain layer reads pts instead.
// - first / last are inclusive pose indices into that plan; turns are pose
//   indices too (a switchback's landing corners); pts are those poses'
//   positions, first to last, in frame_id.
// - known: false says the planner that made the plan reports nothing (a
//   planner without crossings): a follower falls back to its own judgement.
//   true with no crossings says the plan crosses nothing -- every rise on it
//   is floor the flat gait walks.
//
// Not a syncai_common message on purpose: the backend does not read it, and a
// typed message is a change in another repository (CLAUDE.md, "Out of tree");
// the format is this header's, read and written only here.
//
// ROS-free (nlohmann-json behind the .cpp), so the planner's core, its CLI
// and the tests use it as they are.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace syncai_util
{

struct PlanCrossing
{
  struct Point
  {
    double x, y, z;
  };
  std::string kind;           // "stair", "kerb", "ramp"
  bool stair_gait{false};     // "gait": "stair" (true) or "flat"; the route's, so a kerb
                              // onto a stair's top step is "stair" too
  std::string evidence;       // "track", "geometry", "declared"
  double rise{0.0};           // height between its two ends (m)
  size_t first{0}, last{0};   // inclusive pose indices into the plan
  std::vector<size_t> turns;  // pose indices of the turns in place (a switchback's landings)
  std::vector<Point> pts;     // the plan's poses first .. last
};

struct PlanCrossings
{
  bool known{false};
  int32_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  std::string frame_id;
  size_t poses{0};
  std::vector<PlanCrossing> crossings;

  // Whether the indices are for a plan with this stamp and pose count.
  bool isFor(int32_t sec, uint32_t nanosec, size_t n_poses) const
  {
    return known && stamp_sec == sec && stamp_nanosec == nanosec && poses == n_poses;
  }
};

// The topic, relative: /<robot_id>/plan_crossings.
constexpr const char * kPlanCrossingsTopic = "plan_crossings";

std::string toJson(const PlanCrossings & c);
// False with `error` set on anything it cannot read; `out` is then untouched.
bool fromJson(const std::string & json, PlanCrossings & out, std::string & error);

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__PLAN_CROSSINGS_HPP_
