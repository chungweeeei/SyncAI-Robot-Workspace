#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_ASTAR_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_ASTAR_HPP_

// A* over the traversable cells of a SurfaceMap, plus the start / goal snap.
//
// The graph is the map's move rule (SurfaceMap::forEachMove): 8 neighbours in
// xy, each resolved to the layer within one step of the current cell. The
// edge cost is the 3D length of the move in metres times a clearance factor
// (1 inside open space, up to 1 + clearance_cost_weight against the
// traversable boundary), times a small multiplier on inferred cells. It is a
// multiplier and not a per-node penalty on purpose: a per-node penalty costs
// a 0.141 m diagonal the same as a 0.1 m straight move, so inside a cost
// band A* prefers diagonals and the path takes a 45 degree bias; the 2D Smac
// planner this sits beside pays per metre for the same reason. Every edge is
// at least its 3D length, so the 3D Euclidean distance to the goal is an
// admissible heuristic (heuristic_weight 1 = optimal paths).
//
// With StairLinks (stair_links.hpp) the graph also has the links between the
// ends of each stair the robot walked while mapping: a link is one move whose
// cost is its 3D length times stair_cost_multiplier (>= 1, so the heuristic
// stays admissible), and two cells are reachable when their components are in
// the same linked group rather than the same component.
//
// With a BlockedView (dynamic_overlay.hpp, 2026-10-21) the search also keeps
// off the cells the dynamic overlay has marked: a blocked cell is never
// expanded (except in the disc around the start, the robot's own
// surroundings) and the clearance the cost factor reads counts the marks as
// boundary, so a replan round a person detours exactly as it would round a
// wall that had been there at build time. The map's components are baked, so
// a corridor a mark closes ends the search with no_path, not disconnected.
//
// ROS-free (see surface_map.hpp).

#include <memory>
#include <string>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

// Search-time parameters. None of these touch the map, so the ROS plugin
// accepts them as dynamic parameters.
struct SearchParams
{
  // Factor on the move length against the traversable boundary:
  // 1 + weight * exp(-scaling * clearance_d) while clearance_d is inside
  // clearance_cost_radius, 1 beyond. Mirrors the 2D costmap's inflation
  // (cost_scaling_factor 1.5) and Smac's cost_travel_multiplier (1.0): in a
  // passage narrower than twice the band the cheapest line is the centre, in
  // open space the path rides the band edge.
  double clearance_cost_weight = 1.0;
  double clearance_cost_scaling = 1.5;
  // Prefer observed floor over hole-filled floor, mildly.
  double inferred_cost_multiplier = 1.2;
  // Factor 1 + weight * (gaps / footprint columns) on a move into a cell
  // whose footprint disc has columns nothing was seen in: the planner
  // prefers ground it has seen all of, and takes sparse ground when that is
  // what there is.
  double gap_cost_weight = 1.0;
  // > 1 is weighted A*: faster, paths up to this factor longer than optimal.
  double heuristic_weight = 1.0;
  // Guards. Iterations are heap pops; the time is checked every 1024 of them.
  size_t max_iterations = 2000000;
  double max_planning_time = 2.0;  // seconds, 0 = unlimited
  // Snap: the nearest traversable cell within snap_xy_radius whose top face
  // is within snap_z_window of the height hint. 0.5 m is Smac's tolerance;
  // 1.0 m covers the lidar-to-floor offset of a goal given at the robot's
  // height (0.48 m) and is far below the spacing of two floors.
  double snap_xy_radius = 0.5;
  double snap_z_window = 1.0;
  // Factor on a stair link's length: a flight is slower and riskier than
  // the same length of floor, so a level detour up to this many times longer
  // wins. Clamped to >= 1.
  double stair_cost_multiplier = 2.0;
};

enum class PlanFailure {
  None = 0,
  MapNotLoaded,
  MapLoading,
  StartOutsideMap,
  GoalOutsideMap,
  StartUnavailable,
  GoalUnavailable,
  Disconnected,
  SearchLimit,
  Timeout,
  NoPath,
};
const char * planFailureName(PlanFailure f);

struct SnapResult
{
  CellId cell = kNoCell;
  bool outside = false;  // (x, y) is not even inside the grid
  double xy_dist = 0.0;  // from the request to the cell centre
  double dz = 0.0;       // cell top face minus the height hint
  std::string detail;    // why nothing qualified, when cell == kNoCell
};

struct PlanResult
{
  PlanFailure failure = PlanFailure::None;
  std::string detail;
  size_t iterations = 0;
  double search_s = 0.0;
  double cost = 0.0;           // g of the goal
  std::vector<CellId> cells;   // start .. goal inclusive, adjacent under the move rule or a link
  std::vector<LinkUse> links;  // the moves of cells that are stair links, in order
};

class SurfaceAStar
{
public:
  explicit SurfaceAStar(
    std::shared_ptr<const SurfaceMap> map, std::shared_ptr<const StairLinks> links = nullptr);

  const SurfaceMap & map() const { return *map_; }
  const StairLinks * links() const { return links_.get(); }

  // With prefer_component set, only cells reachable from that component
  // qualify -- the same component, or with stair links the same linked group
  // (a goal is snapped to the floor the robot can get to, not to the floor
  // below a balcony edge); the detail then names the nearest cell that was
  // refused for being elsewhere, so the message says "wrong floor" and not
  // "nothing". With blocked set, cells the dynamic overlay blocks do not
  // qualify either (a goal under a person fails, as a goal in a lethal cell
  // does in 2D), and the detail counts them; the start is snapped without
  // it, since the robot stands where it stands.
  SnapResult snap(
    double x, double y, double z_hint, const SearchParams & p,
    uint32_t prefer_component = kNoComponent, const BlockedView * blocked = nullptr) const;

  // blocked: the dynamic overlay's mask for this plan, with its exempt disc
  // around `start` (see BlockedView); the default blocks nothing.
  PlanResult plan(
    CellId start, CellId goal, const SearchParams & p, const BlockedView & blocked = BlockedView{});

  // The multiplier on the length of a move INTO cell c; with blocked, the
  // clearance it reads counts the overlay's marks as boundary.
  double cellCostFactor(
    CellId c, const SearchParams & p, const BlockedView * blocked = nullptr) const;

private:
  // The linked group of a cell's component (the component itself without links).
  uint32_t reach(CellId c) const;

  std::shared_ptr<const SurfaceMap> map_;
  std::shared_ptr<const StairLinks> links_;
  // Scratch, sized to the map once; a generation stamp instead of a memset
  // per plan.
  std::vector<float> g_;
  std::vector<CellId> parent_;
  std::vector<int32_t> parent_link_;  // the link used to reach a cell, -1 for a move
  std::vector<uint32_t> open_stamp_, closed_stamp_;
  uint32_t gen_ = 0;
};

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_ASTAR_HPP_
