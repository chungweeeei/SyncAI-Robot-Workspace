#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_MAP_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_MAP_HPP_

// The multi-layer 2.5D surface map OctomapPlanner3D plans on, built once from
// an octomap::OcTree (map/<name>/octomap.bt, written by syncai_mapping's
// clean_map).
//
// Why a surface map and not the octree itself: the robot is a quadruped, not
// a drone. Its path has to lie on ground it can stand on, so the search space
// is "floor cells", not "free voxels". A free-space 3D search with a
// "something occupied one cell below" patch (the approach this replaces) lets
// the path hug a wall face at any height, climb it where the point cloud is
// sparse, and never asks whether the body fits or the step is one the legs
// can take. Here every xy column of the tree is reduced, once at load, to the
// list of surfaces it holds -- an occupied voxel with nothing occupied above
// it and a body-height band of non-occupied voxels on top -- and each surface
// cell is then judged with the robot's geometry: the footprint disc has to
// find a surface at every column inside it, those surfaces have to lie on a
// plane the legs can stand on, and every neighbour has to be within one step.
// The A* in surface_astar.hpp only ever moves between cells that passed, so a
// path can neither leave the ground nor cross a stair riser, a kerb, a
// drop-off or a wall foot; it can take a ramp, because a ramp is a run of
// one-cell steps on a plane under the slope limit.
//
// A column can hold several surfaces (two floors, a table top above a floor),
// hence "multi-layer": the map is a dense column grid with a CSR list of
// surface cells per column, sorted by height. The step rule is what keeps
// layers apart in the search: a move to a neighbouring column picks the layer
// within max_step_height of the current one, and two layers of one column are
// never that close (the body-clearance band of the lower one forbids it), so
// the pick is unique and a floor never "connects" to the floor above it.
//
// ROS-free on purpose: this header, surface_astar.hpp and path_postprocess.hpp
// depend on octomap and the STL only, so the same code is run offline by
// octomap_planner_3d_cli (on a workstation with homebrew/apt octomap, no ROS)
// against the .bt a map was saved with, and the ROS plugin is a thin wrapper.

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace octomap
{
class OcTree;
}

namespace syncai_global_planner::octomap_planner_3d
{

using CellId = uint32_t;
constexpr CellId kNoCell = ~CellId(0);
constexpr uint32_t kNoComponent = ~uint32_t(0);

// Geometry the map is judged with. All of these bake into the map at build
// time (a change needs a rebuild -- "requires restart" in the ROS plugin).
struct SurfaceMapParams
{
  // Hard clearance: every column within this radius of a cell must hold a
  // usable surface, otherwise the cell is footprint_blocked. This is the
  // implicit inflation of walls, drop-offs and unfillable holes. 0.25 m is the
  // 2D global costmap's inscribed radius (padded half-width of the G23
  // footprint), so Octo3D and GridBased refuse the same gaps.
  double robot_radius = 0.25;
  // Height of non-occupied voxels needed above a surface for the body to fit.
  // The lidar top is ~0.55 m above the ground; 0.6 = 6 cells at 0.1 m.
  double body_clearance = 0.6;
  // Largest height change between two neighbouring cells the legs can take
  // in one move. Quantised to whole cells: at res 0.1 every value in
  // [0.1, 0.2) means "one cell", and 0.2 would let a 0.17 m stair riser
  // through, since a riser lands on either one or two cells.
  double max_step_height = 0.10;
  // Largest tilt of the plane fitted through the surfaces under the
  // footprint disc. Stairs (riser 0.17 / tread 0.29 ~ 30 deg) are already
  // rejected by the step rule; this is the second guard, and the only one
  // against a shallow stair with 0.1 m risers, which is a 19 deg ramp at
  // this resolution. The plane fit averages the quantisation noise out --
  // a flat floor read cell by cell can look 11 deg tilted across a 0.5 m
  // disc -- which is why this is a fit and not a max-minus-min test.
  double max_slope_deg = 20.0;
  // How many voxels of the clearance band may be unknown (never observed).
  // -1 = all of them. The lidar sits 0.48 m up and looks mostly outward, so
  // the 0.1-0.6 m band above a floor is observed only where the robot
  // actually passed; a strict setting punches holes in exactly the corridors
  // the path needs. A surface with an *occupied* voxel in the band is always
  // rejected (clearance_occupied).
  int max_unknown_clearance_cells = -1;
  // The lowest this many voxels of the band must have been *observed* free.
  // A ceiling, a shelf underside or the bottom of a slab seen from below
  // is an occupied voxel with unknown above it, exactly like a floor whose
  // air was never looked at -- except that the lidar standing on a floor
  // always sweeps the voxel right above it. 1 keeps the ceilings of a
  // multi-storey map from becoming traversable islands; 0 disables the rule.
  int min_known_free_above = 1;
  // Floor voxels exist only where some scan hit them (clean_map restores the
  // floor band, but grazing rays miss ~4 % of the columns under the robot's
  // own track). A column with no surface, whose neighbours within this
  // radius agree on a floor height from at least three of the four
  // quadrants around it and with at least min_support of them, and which
  // has no occupied voxel anywhere near that height, gets an *inferred*
  // surface there. The quadrant rule is what stops the fill from extending a
  // balcony or a floor edge into the void: an edge has support on one side
  // only. Passes run the fill again on the result, so a hole up to ~2x the
  // radius closes from the rim inward.
  double hole_fill_radius = 0.3;
  double hole_fill_min_support = 0.5;
  int hole_fill_passes = 2;
  // How far from the nearest non-traversable cell the soft clearance cost
  // reaches (clearance_d is clamped to this). Measured from the traversable
  // boundary, which already sits robot_radius from the wall, so 0.55 puts
  // the band edge 0.8 m from the wall -- the 2D costmap's inflation_radius.
  double clearance_cost_radius = 0.55;
};

// Why a surface cell is not traversable. Kept per cell so the CLI dump and a
// failed snap can say *what* is at a column, not just that nothing is.
enum class Reject : uint8_t {
  None = 0,
  ClearanceOccupied,  // an occupied voxel inside the body-clearance band
  ClearanceUnknown,   // too many unknown voxels in the band
  NoFreeAbove,        // the voxel(s) right above were never observed: a ceiling seen from below
  Step,               // an 8-neighbour column has no usable surface within max_step_height
  FootprintBlocked,   // a column inside the footprint disc has no usable surface on the slope cone
  Slope,              // the fitted plane under the disc is steeper than max_slope_deg
  Residual,           // the surfaces under the disc do not lie on a plane (rough / broken ground)
  Count_
};
const char * rejectName(Reject r);

enum CellFlag : uint8_t {
  kTraversable = 1u << 0,
  kInferred = 1u << 1,  // hole-fill put it here; no voxel was ever observed
};

struct SurfaceCell
{
  int16_t kz;           // grid z index of the floor voxel; its top face is at z0 + (kz + 1) * res
  uint8_t flags;        // CellFlag bits
  uint8_t reject;       // Reject
  uint8_t unknown_cnt;  // unknown voxels in the clearance band (information only)
  float
    clearance_d;  // xy distance (m) to the nearest non-traversable cell, clamped to clearance_cost_radius
  uint32_t component;  // 8-connected component id among traversable cells, kNoComponent otherwise
};

struct BuildStats
{
  size_t leaves = 0, occupied_leaves = 0, free_leaves = 0;
  size_t columns = 0, candidates = 0, inferred = 0, traversable = 0, components = 0;
  size_t rejected[static_cast<size_t>(Reject::Count_)] = {};
  std::vector<size_t> component_sizes;  // descending
  double t_columns = 0, t_candidates = 0, t_hole_fill = 0, t_step = 0, t_footprint = 0,
         t_components = 0, t_clearance = 0, t_total = 0;
};

class SurfaceMap
{
public:
  // Grid geometry. Cell (ix, iy, kz) spans [x0 + ix*res, x0 + (ix+1)*res) etc.
  double res = 0.1, x0 = 0, y0 = 0, z0 = 0;
  int nx = 0, ny = 0, nz = 0;
  int step_cells = 1;  // max_step_height in cells
  SurfaceMapParams params;

  // CSR: the cells of column c are cells[col_begin[c] .. col_begin[c+1]),
  // sorted by kz. cell_col[i] is the column of cells[i].
  std::vector<uint32_t> col_begin;
  std::vector<SurfaceCell> cells;
  std::vector<uint32_t> cell_col;

  size_t columns() const { return static_cast<size_t>(nx) * ny; }
  bool inGrid(int ix, int iy) const { return ix >= 0 && iy >= 0 && ix < nx && iy < ny; }
  uint32_t col(int ix, int iy) const { return static_cast<uint32_t>(iy) * nx + ix; }
  int colIx(uint32_t c) const { return static_cast<int>(c % nx); }
  int colIy(uint32_t c) const { return static_cast<int>(c / nx); }
  int ixOf(double x) const { return static_cast<int>(std::floor((x - x0) / res)); }
  int iyOf(double y) const { return static_cast<int>(std::floor((y - y0) / res)); }
  double cx(int ix) const { return x0 + (ix + 0.5) * res; }
  double cy(int iy) const { return y0 + (iy + 0.5) * res; }
  double zTop(CellId c) const { return z0 + (cells[c].kz + 1) * res; }
  bool traversable(CellId c) const { return (cells[c].flags & kTraversable) != 0; }
  bool inferred(CellId c) const { return (cells[c].flags & kInferred) != 0; }
  void cellXY(CellId c, double & x, double & y) const
  {
    x = cx(colIx(cell_col[c]));
    y = cy(colIy(cell_col[c]));
  }

  // The layer of column c within +-max_dkz cells of kz, or kNoCell. Unique
  // whenever max_dkz is below the clearance band (see the header comment).
  CellId layerNear(uint32_t c, int kz, int max_dkz) const;
  // layerNear restricted to traversable cells and the step rule: the cell
  // the search / connectivity / clearance passes may move to from (c, kz).
  CellId traversableNeighbour(int ix, int iy, int kz) const;

  // The move rule shared by the search, the component labelling, the
  // clearance field and the line-of-sight check: 8 neighbours, each resolved
  // to its traversable layer within one step, and a diagonal only when both
  // orthogonal neighbours are traversable too (no squeezing corner to corner
  // between two blocked cells). fn(neighbour, k) with k the kDx8/kDy8 index;
  // the diagonals are the odd k.
  static constexpr int kDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static constexpr int kDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  template <typename Fn>
  void forEachMove(CellId c, Fn && fn) const
  {
    const int ix = colIx(cell_col[c]), iy = colIy(cell_col[c]);
    const int kz = cells[c].kz;
    CellId nb[8];
    for (int k = 0; k < 8; ++k) {
      nb[k] = traversableNeighbour(ix + kDx8[k], iy + kDy8[k], kz);
    }
    for (int k = 0; k < 8; ++k) {
      if (nb[k] == kNoCell) {
        continue;
      }
      if ((k & 1) && (nb[(k + 7) & 7] == kNoCell || nb[(k + 1) & 7] == kNoCell)) {
        continue;
      }
      fn(nb[k], k);
    }
  }

  // "z=[-0.40 (traversable, component 3), 4.80 (untraversable: footprint_blocked)]"
  // or "no surface" -- for failure messages and the CLI.
  std::string describeColumn(uint32_t c) const;
};

// Reads a .bt into tree (the tree's resolution comes from the file). False
// with error set on a missing / unreadable / empty file; never throws.
bool loadOctomapBt(const std::string & path, octomap::OcTree & tree, std::string & error);

// The one-time build. The tree is read only; the caller may free it right
// after, which the plugin does because a 7 M node tree is ~200-300 MB and the
// map is ~30 MB.
std::shared_ptr<const SurfaceMap> buildSurfaceMap(
  const octomap::OcTree & tree, const SurfaceMapParams & params, BuildStats * stats = nullptr);

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_MAP_HPP_
