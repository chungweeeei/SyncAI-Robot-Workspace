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
// into the clearance band (the same rule as Smac's "never above cost 252").
//
// ROS-free (see surface_map.hpp).

#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

struct Waypoint
{
  double x, y, z;
};

// True if the straight segment between the centres of a and b runs over
// traversable cells only (step-continuous from a) with clearance_d >=
// min_clearance everywhere. With out set, the cells visited, a first.
bool lineOfSight(
  const SurfaceMap & m, CellId a, CellId b, float min_clearance,
  std::vector<CellId> * out = nullptr);

// Greedy forward shortcut: from an anchor, extend the segment while the next
// cell is still in sight, then anchor there. The result is a sub-sequence of
// the input (first and last cells kept).
std::vector<CellId> shortcutPath(const SurfaceMap & m, const std::vector<CellId> & cells);

// Points every `spacing` metres (xy) along the polyline through the cell
// centres, first and last cell included. z is the top face of the surface
// under each point when one exists within three cells of the interpolated
// height (a ramp is a straight line, a shortcut over a quantised floor is
// not), else the interpolation.
std::vector<Waypoint> resamplePath(
  const SurfaceMap & m, const std::vector<CellId> & cells, double spacing);

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__PATH_POSTPROCESS_HPP_
