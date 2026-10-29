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
// find a surface at nearly every column inside it (a column nothing was ever
// seen in is a gap, tolerated up to a share and paid for in cost; a column
// with a wall, a drop or an obstacle in it is not), those surfaces have to
// lie on a plane the legs can stand on, and every neighbour has to be within
// one step. The A* in surface_astar.hpp only ever moves between cells that
// passed, so a path can neither leave the ground nor cross a stair flight, a
// drop-off or a wall foot; it can take a kerb up to max_step_height and a
// ramp, which is a run of steps on a plane under the slope limit.
//
// A column can hold several surfaces (two floors, a table top above a floor),
// hence "multi-layer": the map is a dense column grid with a CSR list of
// surface cells per column, sorted by height. The step rule is what keeps
// layers apart in the search: a move to a neighbouring column picks the layer
// within max_step_height of the current one, and two layers of one column are
// never that close (the body-clearance band of the lower one forbids it), so
// the pick is unique and a floor never "connects" to the floor above it.
//
// A step the rule allows is still a step (2026-10-29). The kerb between a
// road and its pavement is within max_step_height, so the two are one piece
// of ground and a route used to run along the kerb's edge with one pair of
// feet on each side -- on outdoor_1027 the G23 fell twice at the same kerb,
// where the shortest line to the goal cut the pavement's corner and followed
// the edge at ~15 deg (sim, 2026-10-28). So every traversable cell records
// the largest height change to a neighbour it can move to (step_rise) and
// its distance to the nearest cell with one (step_d, a second distance field
// like the clearance one, out to step_edge_cost_radius), and the search pays
// for moving inside that band (SearchParams::step_edge_cost_weight), so
// running along an edge costs more than keeping off it. At 0.1 m voxels a
// one-cell step is as often a slope's contour line as a kerb
// (kSureStepCells), so the cells on the low side of a step are also strung
// into **lines** -- neighbours whose step faces the same way, so a contour
// line that runs into a kerb stays its own line -- and a line at least
// kerb_min_length long that holds kerb_min_sure_cells cells of two cells or
// more is a **kerb** (kKerb, SurfaceMap::kerb_lines): a real step of 0.1 m
// or more somewhere along it, which no slope under max_slope_deg makes, and
// the one-cell stretches are the same kerb where the road has not fallen
// quite a voxel's worth (outdoor_1027's 12 cm kerb: one cell along most of
// its length, two where the road is lowest). A kerb's cells count as sure
// steps in the band, and stair_links.hpp crosses a kerb on kerb links only,
// square to it. Nothing else in the map changes: the components, the
// clearance and what is traversable are as before.
//
// The voxels cannot tell a 12 cm kerb from a contour line, but the map's
// point cloud can (2026-10-29): outdoor_1027's kerb, where the G23 fell a
// third time in the sim with the kerb links in, was one cell along its whole
// length and no kerb to the map, while map.pcd puts it at 0.12 m. So when
// the build is given the map.pcd beside the tree, every traversable cell
// gets a **measured floor** (SurfaceCell::floor_z): the median height of the
// cloud's points inside its own top voxel, smoothed edge-preserving over its
// neighbours on the same surface (a kerb's other side never votes). Only
// points that fall in a traversable cell's voxel are kept while the file is
// streamed -- walls, trees and ceilings are dropped as they are read -- so
// the cost is one pass over the file. Between two cells that both have one
// the step edges and the kerbs read metric heights (SurfaceMap::stepCells):
// under step_edge_min_rise is level, so a slope's contour lines vanish, and
// above it is a sure step, so a kerb is one along its whole length. Where
// either cell has none (no pcd, an inferred cell, a voxel with too few
// returns) the voxel tops are read as before. The traversability rules
// never read the measured floor.
//
// ROS-free on purpose: this header, surface_astar.hpp and path_postprocess.hpp
// depend on octomap and the STL only, so the same code is run offline by
// octomap_planner_3d_cli (on a workstation with homebrew/apt octomap, no ROS)
// against the .bt a map was saved with, and the ROS plugin is a thin wrapper.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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

// A height change of this many cells between two neighbouring columns is a
// step the map is sure of: a slope within max_slope_deg (20 deg) rises
// 0.036 m over one 0.1 m column, so it changes the voxel top by one cell at
// most, and a flat floor read cell by cell flickers by one cell too; two
// cells (0.1-0.3 m of true height) never come from either. A one-cell change
// may be a kerb or a slope's contour line, and the two cannot be told apart
// at this resolution (outdoor_1027's 12 cm kerb shows as one cell along most
// of its length, two where the road has fallen a little further).
constexpr int kSureStepCells = 2;

// A point of a path or of a stair link: x, y and the ground's height.
struct Waypoint
{
  double x, y, z;
};

// A column of a disc around a centre column: the offset, the distance of its
// centre from the disc's centre, and the quadrant it lies in. discOffsets()
// is the one definition of "the columns within r of a column", shared by the
// footprint rule, the hole fill and the dynamic overlay's dilation
// (dynamic_overlay.hpp), so a mark is inflated over exactly the disc the
// footprint rule judges.
struct Offset
{
  int dx, dy;
  float d_m;     // distance of the column centre from the disc centre (m)
  int quadrant;  // 0..3, the four quadrants around the centre with the axes split evenly
};

// Every column whose centre is within radius_m of the centre column's centre,
// nearest first. (0,0) is included only if include_centre.
std::vector<Offset> discOffsets(double radius_m, double res, bool include_centre);

// Supercover line (E. Dedu): every grid cell the segment from (x0, y0) to
// (x1, y1) passes through, in order, both cells at an exact corner crossing
// -- a Bresenham line skips the cell a segment clips at a corner, which is
// exactly where a wall foot sits. visit(ix, iy) returns false to stop the
// walk, and the result is false then. Shared by the line-of-sight shortcut
// and the stair crossing (path_postprocess.cpp) and by the dynamic overlay's
// ray clearing (dynamic_overlay.cpp), which must agree on which columns a
// line touches.
template <typename Fn>
bool supercover(int x0, int y0, int x1, int y1, Fn && visit)
{
  int x = x0, y = y0;
  int dx = x1 - x0, dy = y1 - y0;
  const int xstep = dx < 0 ? -1 : 1, ystep = dy < 0 ? -1 : 1;
  dx = std::abs(dx);
  dy = std::abs(dy);
  const int ddx = 2 * dx, ddy = 2 * dy;
  if (!visit(x, y)) {
    return false;
  }
  if (ddx >= ddy) {
    int error = dx, errorprev = dx;
    for (int i = 0; i < dx; ++i) {
      x += xstep;
      error += ddy;
      if (error > ddx) {
        y += ystep;
        error -= ddx;
        if (error + errorprev < ddx) {
          if (!visit(x, y - ystep)) {
            return false;
          }
        } else if (error + errorprev > ddx) {
          if (!visit(x - xstep, y)) {
            return false;
          }
        } else {
          if (!visit(x, y - ystep) || !visit(x - xstep, y)) {
            return false;
          }
        }
      }
      if (!visit(x, y)) {
        return false;
      }
      errorprev = error;
    }
  } else {
    int error = dy, errorprev = dy;
    for (int i = 0; i < dy; ++i) {
      y += ystep;
      error += ddx;
      if (error > ddy) {
        x += xstep;
        error -= ddy;
        if (error + errorprev < ddy) {
          if (!visit(x - xstep, y)) {
            return false;
          }
        } else if (error + errorprev > ddy) {
          if (!visit(x, y - ystep)) {
            return false;
          }
        } else {
          if (!visit(x - xstep, y) || !visit(x, y - ystep)) {
            return false;
          }
        }
      }
      if (!visit(x, y)) {
        return false;
      }
      errorprev = error;
    }
  }
  return true;
}

// Geometry the map is judged with. All of these bake into the map at build
// time (a change needs a rebuild -- "requires restart" in the ROS plugin).
struct SurfaceMapParams
{
  // Hard clearance: every column within this radius of a cell must hold a
  // usable surface, otherwise the cell is footprint_blocked. This is the
  // implicit inflation of walls, drop-offs and unfillable holes. 0.25 m is the
  // padded half-width of the G23 footprint (the 2D global costmap's inscribed
  // radius until 2026-10-28, so Octo3D and GridBased refused the same gaps).
  double robot_radius = 0.25;
  // Height of non-occupied voxels needed above a surface for the body to fit.
  // The lidar top is ~0.55 m above the ground; 0.6 = 6 cells at 0.1 m.
  double body_clearance = 0.6;
  // Largest height change between two neighbouring cells the legs can take
  // in one move: the quadruped's step capability (0.17 m since 2026-10-14,
  // the riser of a standard stair, which the G23 was driven up while mapping
  // dp1f_1012; 0.10 before, chosen to refuse stairs, which the stair links
  // of stair_links.hpp now handle). Quantised to the nearest whole cell: at
  // res 0.1, 0.17 is two cells, so two surfaces whose top faces are up to
  // 0.2 m apart are one step (true heights up to ~0.3 m can land that way).
  // A flight of such steps is still refused, by the slope rule below.
  double max_step_height = 0.17;
  // Largest tilt of the plane fitted through the surfaces under the
  // footprint disc. This is what refuses a stair flight (riser 0.17 / tread
  // 0.29 ~ 30 deg) now that single risers pass the step rule -- a disc whose
  // surfaces span no more than one step is level ground with a kerb and is
  // not fitted at all, so the cells at a flight's risers pass as islands
  // while its treads, which see two risers, do not -- and the only guard
  // against a shallow stair with 0.1 m risers, which is a 19 deg ramp at
  // this resolution. The plane fit averages the quantisation noise out --
  // a flat floor read cell by cell can look 11 deg tilted across a 0.5 m
  // disc -- which is why this is a fit and not a max-minus-min test.
  double max_slope_deg = 20.0;
  // Share of the footprint disc's columns that must hold a usable surface.
  // The rest may be *gaps* -- columns nothing was ever seen in -- never a
  // column with a surface out of reach (a drop, a ledge) or an unusable one
  // (a wall foot, an obstacle top), which block the cell outright. The lidar
  // sees the floor at grazing angles only, so the observed floor is sparse
  // away from the track and under it where the hole fill gives up; with
  // every column required (the rule until 2026-10-14) 13 % of dp1f_1012's
  // own keyframes stood on "untraversable" ground. The missing share is
  // also paid for in cost (SearchParams::gap_cost_weight). 1.0 = the old
  // rule.
  double footprint_min_support = 0.8;
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
  // How far from a step edge (a traversable cell with a traversable
  // neighbour at another height, see the header) the step-edge cost reaches:
  // robot_radius plus a foot's reach beyond the body (the feet land ~0.15 m
  // outside the padded footprint in a turn), so a route the band has pushed
  // out keeps every foot on one side of the kerb.
  double step_edge_cost_radius = 0.4;
  // A line of step cells (see the header) is a kerb when it is at least this
  // long (m; shorter is a pothole's rim, a drain cover or noise, left to the
  // band cost) and holds at least this many cells whose step is
  // kSureStepCells or more (one such cell is a stone; three in a line, 0.3 m
  // of a step the map is sure of, is a kerb).
  double kerb_min_length = 1.0;
  int kerb_min_sure_cells = 3;
  // The measured floor (see the header). Between two cells that both have
  // one, a height change under step_edge_min_rise is level and one at or
  // above it is a sure step: a 20 deg slope rises 0.036 m per column, the
  // floor's returns scatter ~1-2 cm after the smoothing, and the kerbs worth
  // keeping a route off are 0.1 m and up. floor_same_surface is how far a
  // neighbour's raw median may be from a cell's own to vote in its smoothing
  // (half a kerb, so a kerb's other side never does).
  double step_edge_min_rise = 0.06;
  double floor_same_surface = 0.05;
};

// Why a surface cell is not traversable. Kept per cell so the CLI dump and a
// failed snap can say *what* is at a column, not just that nothing is.
enum class Reject : uint8_t {
  None = 0,
  ClearanceOccupied,  // an occupied voxel inside the body-clearance band
  ClearanceUnknown,   // too many unknown voxels in the band
  NoFreeAbove,        // the voxel(s) right above were never observed: a ceiling seen from below
  Step,               // an 8-neighbour column has no usable surface within max_step_height
  FootprintBlocked,   // a column inside the footprint disc holds a surface out of reach or unusable
  FootprintGap,       // more than 1 - footprint_min_support of the disc's columns hold nothing
  Slope,              // the fitted plane under the disc is steeper than max_slope_deg
  Residual,           // the surfaces under the disc do not lie on a plane (rough / broken ground)
  Count_
};
const char * rejectName(Reject r);

enum CellFlag : uint8_t {
  kTraversable = 1u << 0,
  kInferred = 1u << 1,  // hole-fill put it here; no voxel was ever observed
  kKerb = 1u << 2,      // on the low side of a kerb line (see the header); SurfaceMap::kerb_lines
};

struct SurfaceCell
{
  int16_t kz;           // grid z index of the floor voxel; its top face is at z0 + (kz + 1) * res
  uint8_t flags;        // CellFlag bits
  uint8_t reject;       // Reject
  uint8_t unknown_cnt;  // unknown voxels in the clearance band (information only)
  uint8_t gap_cnt;  // footprint-disc columns with no surface at all (of SurfaceMap::footprint_n)
  float
    clearance_d;  // xy distance (m) to the nearest non-traversable cell, clamped to clearance_cost_radius
  uint32_t component;  // 8-connected component id among traversable cells, kNoComponent otherwise
  // The step edges (see the header): the largest height change, in cells, to
  // a traversable neighbour the move rule allows (0 on level ground); the xy
  // distance (m) to the nearest cell with one, clamped to
  // step_edge_cost_radius; and that cell's rise (0 beyond the band), a kerb
  // line's cells counting as kSureStepCells whatever their own step.
  uint8_t step_rise;
  uint8_t edge_rise;
  float step_d;
  // The measured floor height (m) from map.pcd (see the header); NaN where
  // there is none -- no pcd, an untraversable or inferred cell, too few returns.
  float floor_z;
};

struct BuildStats
{
  size_t leaves = 0, occupied_leaves = 0, free_leaves = 0;
  size_t columns = 0, candidates = 0, inferred = 0, traversable = 0, components = 0;
  size_t rejected[static_cast<size_t>(Reject::Count_)] = {};
  std::vector<size_t> component_sizes;         // descending
  size_t step_edges = 0, sure_step_edges = 0;  // cells with step_rise >= 1, >= kSureStepCells
  size_t kerb_lines = 0, kerb_cells = 0;       // kerbs (see the header) and their low-side cells
  // The measured floor: the pcd's points, those in a traversable cell's top
  // voxel, and the traversable cells that got a height. floor_error says why
  // a pcd that was given was not read (the map is then built on the voxels).
  size_t floor_points = 0, floor_points_kept = 0, floor_cells = 0;
  std::string floor_error;
  double t_columns = 0, t_candidates = 0, t_hole_fill = 0, t_step = 0, t_footprint = 0,
         t_components = 0, t_clearance = 0, t_floor = 0, t_step_edges = 0, t_total = 0;
};

class SurfaceMap
{
public:
  // Grid geometry. Cell (ix, iy, kz) spans [x0 + ix*res, x0 + (ix+1)*res) etc.
  double res = 0.1, x0 = 0, y0 = 0, z0 = 0;
  int nx = 0, ny = 0, nz = 0;
  int step_cells = 1;   // max_step_height in cells
  int footprint_n = 1;  // columns in the footprint disc (gap_cnt's denominator)
  SurfaceMapParams params;

  // CSR: the cells of column c are cells[col_begin[c] .. col_begin[c+1]),
  // sorted by kz. cell_col[i] is the column of cells[i].
  std::vector<uint32_t> col_begin;
  std::vector<SurfaceCell> cells;
  std::vector<uint32_t> cell_col;
  // The kerbs (see the header): each the low-side cells of one line, the
  // cells flagged kKerb. What stair_links.hpp lays its kerb links along.
  std::vector<std::vector<CellId>> kerb_lines;

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
  // The kz of a surface whose top face is at height z: the inverse of zTop.
  int kzOfTop(double z) const { return static_cast<int>(std::floor((z - z0) / res)) - 1; }
  bool traversable(CellId c) const { return (cells[c].flags & kTraversable) != 0; }
  // The traversable surface under (x, y) nearest in height to z, within
  // max_dkz cells of it, or kNoCell: what a path pose with a height stands
  // on. Three cells covers the quantisation of a resampled path's z and a
  // TF height a few cm off the voxel face, and is far below the spacing of
  // two floors; a pose on a stair finds nothing, since a flight's cells are
  // all rejected.
  CellId traversableAt(double x, double y, double z, int max_dkz = 3) const
  {
    const int ix = ixOf(x), iy = iyOf(y);
    if (!inGrid(ix, iy)) {
      return kNoCell;
    }
    const CellId c = layerNear(col(ix, iy), kzOfTop(z), max_dkz);
    return (c != kNoCell && traversable(c)) ? c : kNoCell;
  }
  bool inferred(CellId c) const { return (cells[c].flags & kInferred) != 0; }
  // The step from a to b, in cells, as the step edges and the kerbs read it
  // (see the header): with a measured floor at both, 0 under
  // step_edge_min_rise and at least kSureStepCells (signed) from there; else
  // kz_b - kz_a, the voxel heights the caller reads (raw or filtered).
  int stepCells(CellId a, CellId b, int kz_a, int kz_b) const
  {
    const float za = cells[a].floor_z, zb = cells[b].floor_z;
    if (std::isnan(za) || std::isnan(zb)) {
      return kz_b - kz_a;
    }
    const double dz = static_cast<double>(zb) - za;
    if (std::fabs(dz) < params.step_edge_min_rise) {
      return 0;
    }
    const int n = std::max(kSureStepCells, static_cast<int>(std::lround(std::fabs(dz) / res)));
    return dz > 0 ? n : -n;
  }
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
  // The surface of column c whose top face is nearest z, if within tol of
  // it, traversable or not -- how the stair links read a flight, whose cells
  // the rules all rejected. kNoCell for an empty column or nothing near.
  CellId surfaceNear(uint32_t c, double z, double tol) const
  {
    CellId best = kNoCell;
    double best_dz = tol;
    for (uint32_t i = col_begin[c], e = col_begin[c + 1]; i < e; ++i) {
      const double dz = std::fabs(zTop(i) - z);
      if (dz <= best_dz) {
        best = i;
        best_dz = dz;
      }
    }
    return best;
  }

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
// map is ~30 MB. floor_pcd, when not empty, is the map's point cloud (an
// x y z float PCD, binary or ascii) the measured floor is read from (see the
// header); one that cannot be read leaves every floor_z NaN and says why in
// BuildStats::floor_error, and the map is built on the voxels alone.
std::shared_ptr<const SurfaceMap> buildSurfaceMap(
  const octomap::OcTree & tree, const SurfaceMapParams & params, BuildStats * stats = nullptr,
  const std::string & floor_pcd = std::string());

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SURFACE_MAP_HPP_
