#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__DYNAMIC_OVERLAY_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__DYNAMIC_OVERLAY_HPP_

// The dynamic-obstacle overlay of OctomapPlanner3D (2026-10-21): what the 2D
// global costmap's obstacle layer is to GridBased, in the surface map's own
// terms, so that a 3D map gets the 2D stack's "path blocked for 10 s ->
// replan round it" and not only the controller's local detour.
//
// Why it exists. The surface map is static: it is the octree the map was
// saved with, reduced once at load. A person standing in a corridor is not
// in it, so a replan returns the same route and is_path_valid, judged on the
// 2D costmap, is wrong the moment the path leaves the floor that costmap was
// made from (on a 3D-only map there is no gridmap at all, and the costmap is
// a 5 x 5 m default grid at the origin). The local costmap's terrain layer
// sees the person, but it is 4 x 4 m in the odom frame and its verdict has
// no reason code (an island or an unknown cell would close a ramp whose
// foot was never seen), so it cannot be the planner's memory either. This
// overlay is: the same lidar cloud, in the map frame, judged against the
// surface map's own ground.
//
// What it does. Every return within obstacle_max_range of the sensor is
// dropped onto the surface it lies above -- the highest layer of its column
// whose top face is at most one cell above the return, so two floors never
// see each other's people -- and if it lies between min_obstacle_height and
// the body clearance above that surface, and the surface is a traversable
// cell, the cell is MARKED with the scan's stamp. Before that, every return
// within raytrace_max_range CLEARS the marks its ray passes through: at each
// column the ray crosses, the height the ray has there is checked against
// the band of the surface under it, and a mark whose band the ray crosses
// is seen empty. A mark the robot cannot see (behind it, out of range,
// occluded) is forgotten after obstacle_memory. snapshot() turns the live
// marks into a mask the search reads: per cell, the xy distance to the
// nearest mark on the same floor, so a mark blocks everything within
// block_radius of it (the robot's half-width plus a margin, as the 2D
// footprint test sweeps) and costs everything within clearance_cost_radius
// beyond that with the same curve the static walls do.
//
// Three decisions worth knowing:
//   - Only traversable cells are marked. A wall's returns land in the
//     columns against the wall, which hold footprint-blocked surfaces (the
//     0.25 m ring the footprint rule cut); marking those and inflating by
//     block_radius would eat 0.1-0.2 m of every corridor on each side and
//     close every door narrower than ~0.9 m while a wall is within range.
//     The price: a person flattened against a wall (inside the ring) is not
//     marked, and is the local costmap's to see.
//   - Returns within self_radius of the sensor are neither marks nor rays.
//     The robot's own legs are 0.2-0.3 m above the floor, ~0.5 m from the
//     lidar in the plane -- inside the band -- and they would mark the
//     cells under the robot on every scan, so that the path's first poses
//     read blocked for good and the BT replanned every 10 s.
//   - Marks decay (the 2D obstacle layer's do not, only rays clear them):
//     the overlay has no rays into what the robot has walked away from, and
//     a mark it cannot refresh or clear must not shape the next goal's
//     route for ever. obstacle_memory outlasts the BT's blocked_time plus a
//     plan, which is all a mark has to do.
//
// The keepout zones (keepout_zones.hpp, 2026-10-23) ride on the same
// machinery: a KeepoutMask is a DynamicMask's shape -- per cell, the xy
// distance to the nearest zone core on the same floor -- built once from the
// map's keepout.json instead of from scans, and BlockedView reads both, so
// every consumer of the view (snap, search, cost, shortcut, validity) keeps
// off a zone exactly as it keeps off a person. KeepoutMask lives in this
// header because BlockedView::blocked() reads it inline; what makes one is
// in keepout_zones.hpp.
//
// ROS-free (see surface_map.hpp): the plugin feeds it transformed clouds and
// stamps in seconds; the CLI feeds it synthetic returns.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

struct DynamicOverlayParams
{
  // Planar distance from the sensor within which a return marks. The 2D
  // global obstacle layer's obstacle_max_range, so a 3D map replans round
  // the same blocker at the same distance.
  double obstacle_max_range = 2.5;
  // Planar distance within which a return's ray clears; a return further out
  // clears along its first raytrace_max_range and marks nothing. 0 = no
  // clearing (marks only age out). The obstacle layer's value.
  double raytrace_max_range = 3.0;
  // Returns closer than this to the sensor (in the plane) are ignored: the
  // robot's own body and legs.
  double self_radius = 0.6;
  // Band above a surface's top face in which a return is an obstacle:
  // (min_obstacle_height, SurfaceMapParams::body_clearance). The terrain
  // layer's band, and the body clearance the surface map was judged with:
  // one calibration. A top face is quantised to the voxel, so 0.1 keeps the
  // floor's own returns out.
  double min_obstacle_height = 0.1;
  // Seconds a mark lives without being refreshed or cleared.
  double obstacle_memory = 30.0;
  // Cells within this distance of a mark are not traversable for the search
  // and read blocked to the validity check. Between the inscribed radius
  // (0.25, the surface map's own wall inflation) and the circumscribed one
  // (0.455); the 2D footprint-perimeter test at the path heading +-0.35 rad
  // reaches ~0.36 m sideways, so a 3D map refuses a passage beside a person
  // at the same width a 2D one does.
  double block_radius = 0.35;
};

// What snapshot() hands the search: per cell, the xy distance to the nearest
// live mark on the same floor, +inf beyond block_radius +
// clearance_cost_radius. Immutable once built, so a plan keeps reading the
// snapshot it took while the next scan builds another.
struct DynamicMask
{
  std::vector<float> d;       // per CellId
  double block_radius = 0.0;  // d <= block_radius is blocked
  size_t marked = 0;          // live marks the mask was built from
};
using DynamicMaskPtr = std::shared_ptr<const DynamicMask>;

// The keepout zones of a map, rasterised onto one SurfaceMap
// (keepout_zones.hpp builds it): per cell, the xy distance to the nearest
// cell inside a zone's core on the same floor, +inf beyond block_radius +
// clearance_cost_radius -- a DynamicMask's shape, so the search reads both
// the same way. block_radius is the robot's radius (the costmap's inscribed
// radius, what the 2D KeepoutFilter inflates its mask with), so a zone
// closes a gap narrower than the robot exactly as a wall does and the drawn
// polygon is the forbidden area only. A stair link any of whose points lies
// in a core is cut: not expanded by the search, so a zone over a stair foot
// forbids that stair. Immutable once built; indexes the CellIds of the map
// it was built on, so a reload builds a new one.
struct KeepoutMask
{
  std::vector<float> d;           // per CellId
  double block_radius = 0.25;     // d <= block_radius is blocked
  size_t zones = 0;               // zones rasterised
  size_t core = 0;                // cells with d == 0 (inside a polygon, in its z band)
  size_t blocked = 0;             // traversable cells with d <= block_radius
  std::vector<uint8_t> link_cut;  // per StairLinks::links index: a point of the link is in a core
  size_t links_cut = 0;

  bool core_(CellId c) const { return d[c] <= 0.0f; }
  bool linkCut(uint32_t li) const { return li < link_cut.size() && link_cut[li] != 0; }
};
using KeepoutMaskPtr = std::shared_ptr<const KeepoutMask>;

// One plan's (or one validity check's) view of the masks: which cells are
// blocked, and how clear the others are. The disc of exempt_radius around
// `exempt` -- the start cell, i.e. the robot's own surroundings -- is not
// blocked by a mark's or a zone's dilation, so a person right beside the
// robot, or a zone edge it stands next to, does not make every first step
// illegal (the 2D stack clears the robot's footprint from its costmap for
// the same reason); a marked cell or a zone's core cell itself (d == 0) is
// blocked even there, since the robot is not standing on the person, and a
// robot standing inside a zone is refused a plan (start_in_keepout) rather
// than let out through the core. Both masks may be null; the view then
// blocks nothing, and consumers test empty() for the fast path.
struct BlockedView
{
  const DynamicMask * mask = nullptr;
  const KeepoutMask * keepout = nullptr;
  CellId exempt = kNoCell;
  // One disc for both rings: the larger of the two block radii, set by the
  // caller (a 0.35 m disc round the robot is exempt from a 0.25 m zone ring
  // too, which only matters within 0.1 m of a zone edge).
  double exempt_radius = 0.0;

  bool empty() const { return mask == nullptr && keepout == nullptr; }

  bool blocked(const SurfaceMap & m, CellId c) const
  {
    return blockedByMark(m, c) || blockedByZone(m, c);
  }
  bool blockedByMark(const SurfaceMap & m, CellId c) const
  {
    return mask && ring(m, c, mask->d[c], mask->block_radius);
  }
  bool blockedByZone(const SurfaceMap & m, CellId c) const
  {
    return keepout && ring(m, c, keepout->d[c], keepout->block_radius);
  }

  // The cell's clearance with the marks and the zones counted as boundary:
  // the smallest of its static clearance_d and its distance beyond the
  // blocked ring of the nearest mark and of the nearest zone. What the cost
  // factor and the shortcut's clearance rule read, so a person or a zone is
  // paid for and kept away from exactly as a wall is.
  float clearance(const SurfaceMap & m, CellId c) const
  {
    float cl = m.cells[c].clearance_d;
    if (mask) {
      const float e = mask->d[c] - static_cast<float>(mask->block_radius);
      cl = std::min(cl, std::max(0.0f, e));
    }
    if (keepout) {
      const float e = keepout->d[c] - static_cast<float>(keepout->block_radius);
      cl = std::min(cl, std::max(0.0f, e));
    }
    return cl;
  }

private:
  // d <= r blocks, except inside the exempt disc, which never exempts a
  // core (d == 0).
  bool ring(const SurfaceMap & m, CellId c, float d, double r) const
  {
    if (d > r) {
      return false;
    }
    if (d <= 0.0f || exempt == kNoCell) {
      return true;
    }
    double x, y, ex, ey;
    m.cellXY(c, x, y);
    m.cellXY(exempt, ex, ey);
    const bool same_floor = std::fabs(m.zTop(c) - m.zTop(exempt)) < m.params.body_clearance;
    return !(same_floor && std::hypot(x - ex, y - ey) <= exempt_radius + 1e-9);
  }
};

// The one definition of "the cells within reach of a seed, on its floor":
// every seed gets d = 0, and every column of `disc` round it is resolved to
// the layer within the slope cone of the seed's (step_cells plus the rise of
// max_slope over the distance, the footprint rule's own tolerance,
// surface_map.cpp), so a ramp's uphill cells are reached and the floor a
// body height below is not; each cell keeps the distance of the nearest
// seed. Cells of any kind are stamped; only traversable ones are ever read.
// `d` must be sized to m.cells and hold +inf where nothing has reached yet.
// Shared by the dynamic overlay's snapshot and the keepout rasteriser, so a
// zone's ring follows a ramp and stops at a floor edge exactly as a mark's.
void dilateOnSurface(
  const SurfaceMap & m, const std::vector<CellId> & seeds, const std::vector<Offset> & disc,
  std::vector<float> & d);

struct ScanStats
{
  size_t points = 0;    // returns given
  size_t in_range = 0;  // within obstacle_max_range and outside self_radius
  size_t marked = 0;    // returns that marked a cell (refreshes included)
  size_t cleared = 0;   // marks a ray cleared
};

class DynamicOverlay
{
public:
  explicit DynamicOverlay(const DynamicOverlayParams & p = DynamicOverlayParams{});

  // Bind to a map (every mark is dropped, the arrays resized, the keepout
  // reset -- it indexed the old map's cells) or to none.
  void setMap(std::shared_ptr<const SurfaceMap> map);
  std::shared_ptr<const SurfaceMap> map() const;

  // The map's keepout mask, carried here beside the map pointer because this
  // is the one object the plugin's isPathValid() reads without the plugin's
  // own lock (a plan holds that for its whole search). Swapped by the plugin
  // after a build or a load_keepout; nullptr = no zones. clear() leaves it
  // alone: clear_overlay drops marks, never zones.
  void setKeepout(KeepoutMaskPtr keepout);
  // Both under one lock, so a reader never pairs a mask with another map.
  std::pair<std::shared_ptr<const SurfaceMap>, KeepoutMaskPtr> mapAndKeepout() const;

  void setParams(const DynamicOverlayParams & p);
  DynamicOverlayParams params() const;

  // One scan: returns and the sensor origin in the map frame, stamped t
  // seconds (any clock, as long as snapshot() gets the same one). Clears
  // first, then marks, so a return's own cell survives its neighbours' rays.
  ScanStats addScan(const std::vector<Waypoint> & points, const Waypoint & origin, double t);

  // The mask of every mark younger than obstacle_memory at `now`, or nullptr
  // when there is none (the fast path: no mask, nothing to read). Rebuilt
  // only after a scan changed something or a mark aged out, so the 1 Hz
  // validity check and a plan share one.
  DynamicMaskPtr snapshot(double now);

  // Drop every mark. Returns how many there were.
  size_t clear();

  // Live marks (not aged out as of the last snapshot).
  size_t hotCount() const;

  // The surface of column c a point at height z lies on: the highest layer
  // whose top face is at most one cell above z, or kNoCell. Not layerNear(),
  // which picks the layer nearest in height and may pick one above the
  // point.
  static CellId layerUnder(const SurfaceMap & m, uint32_t c, double z);

private:
  void rebuildDisc();  // under mutex_

  static constexpr float kNever = -std::numeric_limits<float>::infinity();

  mutable std::mutex mutex_;
  std::shared_ptr<const SurfaceMap> map_;
  KeepoutMaskPtr keepout_;
  DynamicOverlayParams p_;
  // Per CellId, the stamp of the last return in its band, seconds since
  // epoch_ (the first stamp seen, so a float keeps ms precision for days);
  // kNever when there is none.
  std::vector<float> last_hit_;
  std::vector<uint8_t> in_hot_;  // cell is in hot_
  std::vector<CellId> hot_;      // cells with a mark, compacted at snapshot()
  std::vector<Offset> disc_;     // block_radius + clearance_cost_radius, per map
  double epoch_ = 0.0;
  bool have_epoch_ = false;
  DynamicMaskPtr mask_;
  bool dirty_ = true;
};

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__DYNAMIC_OVERLAY_HPP_
