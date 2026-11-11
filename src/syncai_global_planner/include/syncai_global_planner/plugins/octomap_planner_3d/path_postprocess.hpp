#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__PATH_POSTPROCESS_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__PATH_POSTPROCESS_HPP_

// Turns the A* cell chain into a path: a line-of-sight shortcut that stays on
// the surface, then a resample at a fixed spacing with z read back from the
// surface map.
//
// The shortcut is validated with a supercover traversal (every cell the
// segment touches, corner cells included -- a Bresenham line skips the cell a
// segment clips at a corner, which is exactly where a wall foot sits), and
// each column along it is resolved to the layer within one step of the
// previous one, so a shortcut can no more cross a kerb or jump floors than
// the search can. It is also not allowed to come closer to the traversable
// boundary than the sub-path it replaces, so smoothing never pulls the path
// into the clearance band (the same rule as Smac's "never above cost 252") --
// but for a cell's worth of tolerance since 2026-11-10 (ShortcutSlack), which
// the exact rule turned out to need.
//
// A route that takes a stair link (stair_links.hpp) is assembled by
// buildPath. Over a walked link the shortcut and the floor-height lookup
// apply to the floor stretches only, and the flight follows the keyframes
// the robot walked. Over a straight link (2026-10-19) the shortcut may also
// span the stair: the last vertex before the link and the first after it
// are joined by one segment when that segment runs within
// kStairCrossingToleranceDeg of the flight's uphill direction, over
// lower-floor cells, then surfaces within one step of each other (the
// flight), then upper-floor cells, keeping the clearance the vertices it
// replaces had. Without this the link's two ends sit on cell centres, so a
// 0.5 m link can only point in ~11 deg steps and the path bent ~15 deg at
// the stair head on dp; with it the route is one line through the stair,
// facing the risers to within the tolerance.
//
// Every function takes an optional BlockedView (dynamic_overlay.hpp,
// 2026-10-21): with one, a cell the dynamic overlay blocks -- or, since
// 2026-10-23, a cell in a keepout zone's core or ring (keepout_zones.hpp) --
// is not in sight, and the clearance the shortcut keeps counts the marks and
// the zones as boundary, so a straightened segment can no more cut through a
// person or a zone than the search could. The flight phase of a stair
// crossing is not checked against it (stair cells are not surface cells, and
// the climb is blind by decision); a link the zones cut never reaches here,
// since the search does not take it.
//
// With the links (2026-10-29) the shortcut also refuses to span a kerb a
// kerb link crosses (StairLinks::kerbCrossing, the search's own rule), so a
// straightened floor stretch cannot put the diagonal back over the kerb that
// the search took square; the kerb link's own points, a's centre, the kerb,
// b's centre, are spliced in like a walked flight's (stairCrossing does not
// apply: a kerb's two sides are one component).
//
// ROS-free (see surface_map.hpp).

#include <algorithm>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

// How far a segment straightened across a straight stair link may point
// away from the flight's uphill direction. A quadruped should meet the
// risers square; 15 deg is what a hand-driven robot does anyway, and below
// RPP's rotate-to-heading threshold (0.25 rad), so it never pivots on the
// stairs for it.
constexpr double kStairCrossingToleranceDeg = 15.0;

// True if the straight segment between the centres of a and b runs over
// traversable cells only (step-continuous from a) with clearance_d >=
// min_clearance everywhere, and, with links, over no kerb a kerb link
// crosses. With out set, the cells visited, a first.
bool lineOfSight(
  const SurfaceMap & m, CellId a, CellId b, float min_clearance,
  std::vector<CellId> * out = nullptr, const BlockedView * blocked = nullptr,
  const StairLinks * links = nullptr);

// How much closer to the boundary than the cells it replaces a shortcut
// segment may come (2026-11-10). Held exactly, "never closer" left the route
// a staircase: down a corridor the search runs on the ridge of the clearance
// field, which on a grid not square to the corridor steps sideways a cell at
// a time, and a straight line between two ridge cells crosses cells 5 cm less
// clear -- refused, so the shortcut advanced cell by cell and the plan came
// out in 45 deg doglegs 0.1-0.6 m long (dp, seven routes, 570 m: 258
// vertices, 101 turning 20 deg or more; with 0.1 m of tolerance 90 and 48,
// the total turning 4469 -> 2357 deg, the length 2 m shorter).
//
// A segment may come `tolerance` closer than the least clear cell of the
// sub-path it replaces, but not under `floor` on the strength of it: where
// the sub-path itself is under the floor (a doorway, 0.10 m), the rule is
// exact again, as it was. Not cumulative -- every segment is held to the
// search's own cells, never to another segment. Both 0 = the exact rule.
struct ShortcutSlack
{
  double tolerance = 0.0;  // m
  double floor = 0.0;      // m
  // The clearance a segment has to keep, replacing cells no less clear than
  // `replaced`.
  float limit(float replaced) const
  {
    const float eased =
      std::max(replaced - static_cast<float>(tolerance), static_cast<float>(floor));
    return std::min(replaced, eased);
  }
};

// Greedy forward shortcut: from an anchor, extend the segment while the next
// cell is still in sight, then anchor there. The result is a sub-sequence of
// the input (first and last cells kept).
std::vector<CellId> shortcutPath(
  const SurfaceMap & m, const std::vector<CellId> & cells, const BlockedView * blocked = nullptr,
  const StairLinks * links = nullptr, const ShortcutSlack & slack = {});

// A corner of a shortcut chain, rounded (2026-11-10; buildPath lays it with
// corner_radius > 0, the CLI's --corners reports it per vertex).
// The shortcut leaves a polyline, every vertex a sharp turn; this is the arc
// of `radius` tangent to both segments at vertex v (a and b its neighbours in
// the chain), walked over the floor as lineOfSight walks a segment --
// traversable, step-continuous, off the marks and the zones, over no kerb a
// link crosses -- and landing on the layer the segment itself reaches.
//
// What it cannot decide is the clearance rule, which is why both numbers come
// back. An arc lies inside its corner, and a shortcut's vertices are where
// the line of sight broke, usually round the very obstacle the corner turns
// about: held to the shortcut's own rule (no closer to the boundary than the
// part it replaces, `replaced`) such a corner cannot be rounded at all, and
// only a floor on `clearance` in metres lets it. --corners counts the corners
// either rule rounds on a real map before one is chosen.
//
// The arc is sampled every quarter cell, not supercovered: a cell it clips by
// less than that (2.5 cm) is not seen. With pts set, those samples, each at
// its cell's floor height.
struct CornerArc
{
  double turn = 0.0;  // how far the route turns at v (rad, 0 = straight on)
  double max_radius =
    0.0;                   // the radius whose tangent points sit half-way along the shorter segment
  bool ok = false;         // the arc of `radius` fits (<= max_radius) and stays on the floor
  float clearance = 0.0f;  // the arc's smallest clearance (ok only)
  float replaced = 0.0f;   // the smallest clearance of the two segment ends it replaces (ok only)
  double deviation = 0.0;  // how far inside v the arc passes (m)
};
CornerArc cornerArc(
  const SurfaceMap & m, CellId a, CellId v, CellId b, double radius,
  const BlockedView * blocked = nullptr, const StairLinks * links = nullptr,
  std::vector<Waypoint> * pts = nullptr);

// Points every `spacing` metres (xy) along the polyline through the cell
// centres, first and last cell included. z is the top face of the surface
// under each point when one exists within three cells of the interpolated
// height (a ramp is a straight line, a shortcut over a quantised floor is
// not), else the interpolation.
std::vector<Waypoint> resamplePath(
  const SurfaceMap & m, const std::vector<CellId> & cells, double spacing);

// Where one link use of a plan landed in buildPath's output (2026-11-02): the
// waypoints first .. last (inclusive) that are the crossing -- the link's own
// points, or the stairCrossing segment that replaced them -- and, for a
// flights link, the waypoint nearest each of its landing corners. What the
// planner's plan_crossings sidecar tells the followers.
struct PathSpan
{
  size_t use = 0;  // index into the uses buildPath was given
  size_t first = 0, last = 0;
  std::vector<size_t> turns;
};

// The whole route of a plan: the cell chain split at its stair links, each
// floor stretch shortcut (when asked) and resampled on its own -- a shortcut
// never spans a link -- and each link's recorded ground points, oriented the
// way it is taken and resampled at the same spacing, spliced in between.
// With vertices set, the number of vertices before resampling (shortcut
// vertices plus link points); with spans set, one PathSpan per use, in order.
// With corner_radius > 0 (2026-11-10) each corner of a floor stretch takes
// the largest arc up to that radius that fits half-way along its shorter
// segment and keeps the shortcut's own clearance rule, slack included; a
// corner with no such arc, a cusp (a turn past 2.0 rad, where the controller
// server cuts the route and the robot turns standing) and the vertex next to
// a crossing stay sharp.
std::vector<Waypoint> buildPath(
  const SurfaceMap & m, const StairLinks * links, const std::vector<CellId> & cells,
  const std::vector<LinkUse> & uses, bool shortcut, double spacing, size_t * vertices = nullptr,
  const BlockedView * blocked = nullptr, std::vector<PathSpan> * spans = nullptr,
  const ShortcutSlack & slack = {}, double corner_radius = 0.0);

// Narrow passages (2026-11-02): the stretches of a route where the floor
// between the walls either side is narrower than the robot can turn in. dp's
// 2F lab door, 0.75 m between its jambs: twice the robot reached it turned
// 20-70 deg off its axis, and with ~6 cm a side no MPPI candidate cleared the
// jambs (stuck going in, and again coming out). A follower that knows where
// such a stretch is can square up before it and drive it straight; the
// local costmap cannot tell, since the floor in a doorway is in the lidar's
// blind range and its jambs read wider than they are. This map saw both
// jambs from the mapping run.
//
// At each waypoint on a traversable cell, the floor is walked square to the
// route both ways, step-continuously, to the first column that is not
// traversable; the passage's width is the two distances plus 2 x
// robot_radius, because traversable already stops robot_radius short of a
// wall (to a cell, so +-0.1 m at 0.1 m). A width under max_width is narrow --
// the footprint's diagonal (0.70 x 0.44 m + padding: 0.85) is the width it
// can turn round in. Measuring both sides is what tells a door from walking
// beside one wall; clearance_d is the nearer side only. A waypoint off the
// traversable cells (a stair or kerb link's own points) is never narrow:
// the links carry their own crossings. Runs of narrow waypoints become
// spans, each grown by `lead` metres of route either side (where to square
// up, a body length) and merged with the next one if they then touch.
struct NarrowSpan
{
  size_t first = 0, last = 0;  // inclusive waypoint indices, lead included
  double width = 0.0;          // the narrowest width inside (m)
  double heading = 0.0;        // the passage's axis as the route takes it (rad, map frame)
};
std::vector<NarrowSpan> findNarrowPassages(
  const SurfaceMap & m, const std::vector<Waypoint> & path, double max_width, double lead);

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__PATH_POSTPROCESS_HPP_
