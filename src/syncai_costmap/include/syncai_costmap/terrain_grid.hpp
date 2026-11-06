#ifndef SYNCAI_COSTMAP__TERRAIN_GRID_HPP_
#define SYNCAI_COSTMAP__TERRAIN_GRID_HPP_

// The local terrain map behind TerrainLayer (2026-10-20): a robot-centric
// 2.5D grid of recent lidar returns, judged cell by cell with the geometry
// rules OctomapPlanner3D judges its static map with (surface_map.hpp's
// SurfaceMapParams -- same names, same values), so that the local costmap
// sees what the global planner sees: a step higher than the legs take, a
// drop-off whose lower floor it has seen (rarely: the -7 deg lower FOV keeps
// ~8x the drop's depth past its edge in shadow, README "A drop-off is mostly
// unseen"), a slope too steep, a thing lower than the old obstacle layer's
// 0.58 m floor -- and nothing on a ramp or a kerb.
//
// Why this and not the obstacle layer's height band: every point above a
// fixed height over the robot's own ground was an obstacle there, so a ramp
// read as a wall after a metre, a kerb or a box under 0.58 m was invisible,
// and a stair's treads were a wall from the second step. Every quadruped
// stack that runs on this kind of hardware (CMU's terrain_analysis on the
// Go2, SCAN-Planner, RAEM) keeps a rolling 2.5D grid with a ground height
// per cell and judges points against *that* ground; this is that grid, with
// Octo3D's rules on top. ROS-free so it can be unit-tested with synthetic
// scenes (test/test_terrain_grid.cpp); TerrainLayer feeds it the cloud and
// reads the costs out.
//
// What a cell holds: a ring of the newest samples (z and time) that fell in
// it, in the costmap's global frame. Two memories are read off the same
// ring: ground is the lower quantile of the 3x3 neighbourhood's samples
// younger than ground_memory (long, because the MID360's -7 deg lower FOV
// never sees the floor within ~3.9 m of the robot: the floor under and just
// ahead of it was seen seconds ago, from further away, or not at all), an
// obstacle is a sample of the cell itself younger than obstacle_memory
// (short, so a person who walks on is gone in two seconds without
// raytracing). A cell with too few samples for a ground estimate borrows
// the robot's own ground height, which is right on a level floor and is
// what keeps a low obstacle on never-seen floor an obstacle: its own
// points, 0.2-0.5 m up, are then in the band.
//
// Sparse cells on a slope (2026-10-24): that borrowed height is wrong where
// the floor ahead is not the robot's height. Coming to dp's arched ramp
// (0.25 m crown), the cells the lidar has only just begun to see -- the
// crown first shows ~1.9 m out -- hold a return or two, borrow the robot's
// ground, and their returns sit 0.20-0.25 m over it: a lethal band 1-2 m
// ahead across the way. Replayed from robot01's dp bag through this grid,
// 554 lethal cells over 32 frames of the approach, every one of them on
// borrowed ground. So a cell with returns but no ground of its own judges
// them against the nearest cell within sparse_ground_radius that has a ground
// of its own and that the robot can walk onto from where it stands (a flood
// from its own cell, over max_step_height steps -- not from the path, whose
// seeds reach a stair's treads): on the ramp that is the ramp beside it, and
// the band is gone (1 cell in 1 frame). Only the obstacle test reads that reference;
// the ground estimate, the reach flood, the step and the slope rules are
// unchanged. Rejected: estimating the ground over a wider neighbourhood (or
// from the path's heights). It took a sparse box top or the next tread of a
// flight for floor, and the box on never-seen floor fell from 78 % lethal to
// 28 %, the flight seen from its foot from 70 % to 33 %; a box or a tread is
// an island, never walked onto, so here neither is ever a reference, and both
// scenes (and the dp stair's approach, replayed) keep every lethal cell.
// What this does not open: a slope whose foot was never seen at all, so that
// nothing between the robot and its upper part is known (the sim's
// voxel-stepped ramp, a 0.1 m tread below the lidar's view).
//
// Overhead (2026-10-22): a return more than body_clearance + max_ground_lift
// above the lowest one its cell holds is clearance the robot walks under --
// a stair's underside, a table top, a beam. Whatever ground the estimate
// returns (at most the lowest + max_ground_lift) it lies above the obstacle
// band, so it is neither an obstacle nor the ground, and addSample() does
// not store it. On robot01 the underside of dp's lobby stair, 1.0-1.4 m over
// the floor and so inside TerrainLayer's max_rel_z, filled the rings with
// returns from right overhead, pushed out the floor seen from further off,
// and became the ground of the cells under it: islands and steps, lethal
// round the robot, MPPI with every candidate colliding in half its cycles
// for 18 s, the robot standing and twitching under a stair the global
// planner had rightly routed it under. The ground estimate drops the same
// returns against the 3x3 neighbourhood's lowest, for cells whose own ring
// took them before any floor. What this cannot fix: a neighbourhood with no
// floor return younger than ground_memory -- never seen, or the robot stood
// under the overhang longer than that -- still takes the underside for its
// ground, and the robot is boxed in by islands.
//
// Clearing (2026-10-23): obstacle_memory alone is how long a person who has
// walked on stays an obstacle -- 2 s, plus a costmap cycle, plus MPPI's
// start from a stop, was the 2-3 s robot01 stood after a blocker left. And
// a person who stood for longer than the floor memory round them lasted (the
// floor within 3.9 m is never seen while the robot stands) left a ghost for
// ground_memory: their legs' returns, the lowest ~0.3 m up (the lower FOV
// edge at 1.5 m), were the only ground evidence in the cell, an island 0.45
// m up that no flood reached. So a return's ray is traced through the cells
// between the lidar and it (clearRay(); the obstacle layer's and Octo3D's
// dynamic overlay's raytracing, in 2.5D), and a sample within
// ray_clear_tolerance of the ray's height in its cell, older than the ray, is
// removed from the ring: the ray proves there was nothing at that height
// when it passed. Removed, not only ignored as an obstacle, so the ghost's
// ground evidence goes with it. Two things a ray does not clear. A sample
// at the cell's ground -- within min_obstacle_height over the last
// classify()'s estimate -- is cleared only by a ray passing *under* it: a
// floor is only ever passed over (by a ray to the same floor further on, to
// a lower floor across a drop-off, along a ramp, just over a box top), and
// a ray over a surface proves nothing against it, while a leg's lowest
// returns, ~0.3 m up with the ground taken from them once the floor memory
// is gone, are passed under at the lower FOV edge. And nothing within
// ray_clear_end_margin of the ray's end: the ray grazes the surface it ends
// on for its last stretch (a floor return at the -7 deg edge is within
// 0.05 m of the floor for its last 0.41 m, tolerance / tan 7 deg), and the
// ground rule alone needs a ground estimate the cell may not have yet. A
// box top is re-seen by the returns that end on it in the same scan either
// way; the cell stays an obstacle as long as one return a scan lands in its
// band. What no ray crosses (the cells behind an occluder, a person standing
// closer than the margin to a wall) still leaves after obstacle_memory, the
// fallback.
//
// Reach: a surface is floor only if the robot can get onto it over steps
// the legs take (classify() has the rule). The lidar's lower FOV makes a
// box, a table or a platform look like a patch of floor 0.3 m up with
// nothing seen under it; flooded from the robot's own cell, such a patch
// is an island and lethal. Octo3D's connected components, locally. The
// path's heights seed the flood too, so a ramp whose foot the lidar never
// looked at is still the floor the planner says it is.
//
// Slope beside an obstacle (2026-11-02): a cell the obstacle rule judges
// lethal -- a door jamb, a wall foot, a box -- has a "ground" that is the
// lower quantile of the thing's own returns, 0.15-0.3 m over the floor when
// the foot is hidden, and the slope rule fitted it with the floor beside it:
// every disc that reached a jamb spanned more than a step and tilted past
// max_slope_deg, so a band of slope cells a disc's radius wide stood in front
// of each jamb. dp's 2F lab door, 0.75 m between its jambs in map.pcd and
// lethal on robot01, replayed from the sim's scans on the approach: 0.5 m of
// free lane for a 0.46 m footprint, MPPI's every candidate in collision. The
// fit now leaves the obstacle cells out (they are lethal anyway): 0.6 m.
// Rejected: leaving out their 8-neighbours as well, whose 3x3 ground took the
// jamb's returns too -- the door opened to 0.7 m, but that ring is the wall
// foot's lethal band and a stair's edge cells, and those tests failed; and
// estimating the ground a second time without the obstacles' returns, which
// broke the wall foot, the box on unseen floor and the crossing's person.
//
// A ray through a cell bounds its ground (2026-11-03). The jamb's returns
// also stand in the 3x3 ground of the doorway cells beside it, and a
// wall's in the cells before it wherever the lower FOV hides its foot: a
// borrowed ground 0.15-0.5 m up that the reach flood cannot step onto, an
// island band along the wall and into the lane. elevation_mapping's
// visibility cleanup (ETH, 2022) deletes a cell whose height a ray passes
// under; here each ray records the lowest height it crossed every cell at
// (Cell::ray_lo, in clearRay()), and a cell's estimate takes no gathered
// sample above that height + ray_clear_tolerance: the ray proves the
// column empty there, so a neighbour's return up there is not this cell's
// ground. Not in the cell the ray ends in (the surface it hits), so a wall
// keeps its own cell; the rays that end on it do cap the cells before it,
// which is where the band was. A floor, a box top, a tread, the lower floor
// across a drop-off: every surface lies under the rays that end on or
// beyond it, so their estimates stay (a box under the rays to a wall behind
// it is in the test). dp's 2F lab, the robot stuck in the pocket between the
// door's east jamb and the east wall, replayed from the sim's scans: 73 ->
// 96 of the pocket's 132 cells not lethal, the lane out of the door 0.5 ->
// 0.6 m.
//
// And the slope fit takes only grounds the cell's own returns stand on
// (groundSupported(), 2026-11-03). A jamb's inner cells hold too few
// returns for the obstacle rule and the jamb's next door for a ground;
// fitted, they tilted every disc that reached them: the lab door on the way
// in, 0.6 m of lane between the jambs with the obstacle cells out of the fit
// (above), 0.7 with these out too, 0.75 in map.pcd. It also ends the
// 0.15 m slope band in front of a seen wall foot, whose cells' grounds came
// from the wall's returns the same way: lethal starts at the wall's own
// cell, as the obstacle layer had it, and the tests say so.
//
// Stairs are not this grid's business: the step and slope rules refuse a
// flight, as Octo3D's do, and nothing here opens one. A stair is crossed
// only over the global planner's stair link, by the controllers' stair mode,
// which climbs blind -- no obstacle avoidance on a flight, by decision
// (2026-10-20). An earlier draft of this layer opened a corridor along a
// climbing path (the path's height as the ground, the band starting higher);
// it was taken out with that decision, and git history has it.
//
// The plan's corridor (2026-11-07). Along the plan -- every pose of it the
// planner lays for the flat gait, which is every pose outside a stair-gait
// span of plan_crossings -- this grid judges no terrain at all: within
// plan_radius of such a pose, on ground within plan_z_band of its height,
// the step, slope and reach rules are not run and the obstacle rule's ground
// is the plan's own height there (the planner's measured floor, interpolated
// along the plan), so a return more than min_obstacle_height above the floor
// the map says is there is an obstacle and the floor itself, a sill's riser
// or a slope's rise included, is not. Why: these rules are the planner's,
// re-run here on worse data (the -7 deg blind cone, grazing-angle returns,
// 0.05 m cells), and each walked sill, short slope and kerb the planner laid
// for the flat gait was walled off again until a matching waiver was added
// -- one per kind of crossing from 2026-11-02 (crossing_radius,
// crossing_z_band, a lift of the obstacle band by the crossing's rise), and
// the next kind would have needed the next. Off the plan the rules run as
// before, so a wall, a drop-off or a box beside the route is what it was.
// What the corridor trusts is the map: a floor the map has 0.3 m too high
// (dp's drift layer) hides a 0.25 m box on the real floor there, as the
// crossing waiver's band did. What it keeps off: a stair-gait span, where
// the flight stays lethal to the flat gait as above (the draft's climbing
// corridor is still not this), and a plan on another floor in the same
// columns (plan_z_band). Not a plan the planner says nothing about
// (plan_crossings `known: false`, or none yet): then there is no word on
// where the stairs are, and no corridor.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace syncai_costmap
{

struct TerrainParams
{
  // ---- samples ----------------------------------------------------------
  // Newest samples kept per cell; older ones are overwritten.
  int max_samples_per_cell = 24;
  // How long a sample serves the ground estimate. The floor near the robot
  // is only ever seen from >= 3.9 m away (MID360 lower FOV), i.e. 5-13 s
  // before the robot arrives at walking speed, so this is long.
  double ground_memory = 10.0;
  // How long a sample counts as an obstacle when no ray clears it (CMU
  // terrain_analysis's decayTime). With clearRay() it is the fallback for
  // cells no ray crosses; until 2026-10-23 it was the only way out.
  double obstacle_memory = 2.0;
  // Ray clearing (the header has the design). A sample this close in height
  // to a later ray through its cell is removed; 0 disables clearing. The
  // lidar's range noise (~0.02) plus the ray's drop across a cell: tight,
  // so a ray just over a box top or a floor's noise clears little.
  double ray_clear_tolerance = 0.05;
  // Nothing is cleared within this of the ray's end: the ray is grazing the
  // surface it ends on there. At the MID360's -7 deg lower edge a floor
  // return's ray is within 0.05 m of the floor for the last 0.41 m
  // (0.05 / tan 7 deg); 0.6 leaves room for noise. A person standing closer
  // than this to the wall behind them is left to obstacle_memory.
  double ray_clear_end_margin = 0.6;
  // Fewest samples (3x3 neighbourhood, within ground_memory) for a cell to
  // have a ground estimate of its own; under it the robot's ground is used.
  int min_ground_samples = 6;
  // Ground = the sample at this quantile of the sorted neighbourhood
  // (CMU's quantileZ): robust to a few stray returns under the floor, and
  // to an obstacle's points above it while the floor still dominates ...
  double ground_quantile = 0.25;
  // ... and capped at the lowest sample plus this (CMU's maxGroundLift), so
  // a wall's or a box's returns cannot lift the ground past the floor they
  // stand on when the floor has been seen at all.
  double max_ground_lift = 0.15;
  // A cell with returns but too few for a ground of its own judges them
  // against the nearest walkable known ground this close, if any, rather than
  // the robot's (the header has why). On robot01's dp ramp replay the
  // approach's 554 lethal cells over 32 frames fell to 132 over 22 at 0.15
  // m, 50 / 16 at 0.25, 15 / 7 at 0.35 and 1 / 1 at 0.4 (0 from 0.45); the
  // dp stair's approach kept every island, step and slope cell at each. 0.4,
  // not more: the further the reference, the likelier it is a floor at
  // another height than the cell's. 0 = off.
  double sparse_ground_radius = 0.4;

  // ---- the geometry rules: SurfaceMapParams' values, one calibration ----
  // Disc the slope is judged under; the costmap's inscribed radius.
  double robot_radius = 0.25;
  // A sample this far above the ground and below body_clearance is an
  // obstacle (0.1 lets a kerb's riser, a doormat and the quantile's own
  // noise through; the step rule judges kerbs).
  double min_obstacle_height = 0.1;
  double body_clearance = 0.6;
  // Largest ground-height change between 8-neighbours the legs take in one
  // move (a stair riser). More is a wall foot, a ledge, a drop-off.
  double max_step_height = 0.17;
  // Largest tilt of the plane fitted under the disc, judged only when the
  // disc's ground spans more than one step (a level floor with a kerb on it
  // is not fitted at all).
  double max_slope_deg = 20.0;
  // Share of the disc's cells that must have a ground estimate for the
  // slope to be judged at all.
  double slope_min_known = 0.5;

  // ---- the plan's corridor (2026-11-07, setPath(); the header) ----
  // A cell this close (m, x / y) to a flat-gait pose of the plan is judged
  // by the obstacle rule alone, against the plan's height there as its
  // ground. robot_radius would just cover the body's half-width centred on
  // the line; 0.4 also keeps the inflation of a step left beside the plan
  // off its centre line. Wider would open a real drop-off beside an
  // unrailed ramp. (crossing_radius from 2026-11-02 to 2026-11-07, over the
  // flat-gait crossings only, with the step and slope rules waived and the
  // obstacle band lifted by each crossing's whole rise -- on a sill's upper
  // landing too, where a 0.35 m box went unseen.)
  double plan_radius = 0.4;
  // ... and whose ground estimate is within this (m) of the plan's height
  // there: a floor above or below the plan in the same column is not it.
  double plan_z_band = 0.5;
};

// A pose of the global path in the grid's frame: its place seeds the reach
// flood, and its height is the corridor's ground (the header) -- unless it
// lies on a stretch the planner lays in the stair gait, which seeds the
// flood and opens no corridor (the flight stays lethal).
struct TerrainPathPose
{
  double x, y, z;
  bool stair = false;
};

struct TerrainStats
{
  size_t known = 0, lethal_obstacle = 0, lethal_step = 0, lethal_slope = 0, lethal_island = 0;
  // Cells on the plan's corridor: judged by the obstacle rule alone, against
  // the plan's height.
  size_t on_plan = 0;
};

class TerrainGrid
{
public:
  struct Sample
  {
    float z;
    float t;  // seconds since the grid's epoch
  };

  TerrainGrid() = default;

  // Geometry: nx x ny cells of `res` metres, the lower-left corner at
  // (ox, oy). Drops every sample.
  void resize(int nx, int ny, double res, double ox, double oy);
  // Move the window to a new lower-left corner, keeping the samples of the
  // cells that stay inside it (Costmap::updateOrigin's shift).
  void updateOrigin(double ox, double oy);
  void clear();

  // A return at (x, y, z) in the grid's frame, seen at time t (seconds;
  // any epoch, the same for every call). Outside the window, or overhead
  // (more than body_clearance + max_ground_lift above the lowest return the
  // cell holds from the last ground_memory): ignored.
  void addSample(double x, double y, double z, double t);

  // The ray of a return at (x, y, z), seen at time t from the lidar at
  // (ox, oy, oz), both in the grid's frame: every cell it crosses up to
  // ray_clear_end_margin before its end loses the samples older than t that
  // lie within ray_clear_tolerance of the ray's height there (the header has
  // why). The return itself is not added -- call addSample() for that; a
  // return outside the window or the layer's height band still clears along
  // its ray, which is what a wall at 6 m is good for. Every cell but the
  // last also keeps the lowest height a ray crossed it at, a bound on its
  // ground (the header).
  void clearRay(double ox, double oy, double oz, double x, double y, double z, double t);
  // Samples clearRay() has removed since construction, for the debug log.
  uint64_t clearedSamples() const { return cleared_; }

  // The current path in the grid's frame -- only when it has heights
  // (OctomapPlanner3D's; a 2D planner's z = 0 says nothing about where the
  // floor goes) -- or an empty list. Replaces the previous call's. Every
  // pose seeds the reach rule (see classify()); every pose not marked
  // `stair` opens the corridor along it (the header, TerrainParams::
  // plan_radius). With `corridor` false the poses seed only: the planner
  // has not said where the stairs on this plan are, so none may open one.
  void setPath(const std::vector<TerrainPathPose> & poses, bool corridor = true);

  // Judge every cell at time `now`, with the robot at (robot_x, robot_y) on
  // ground `robot_ground_z`, which is also the estimate for cells that have
  // none. Writes one costmap value per cell (cost_values.hpp:
  // LETHAL_OBSTACLE, FREE_SPACE or NO_INFORMATION) to `cost`, sized nx * ny,
  // row-major as Costmap is.
  //
  // The reach rule: a surface is floor only if the robot can get onto it
  // over steps the legs take, so the cells are flooded from the robot's own
  // (and from the path's, whose heights say where the floor goes where the
  // lidar has not looked) over 8-neighbours within max_step_height, unknown
  // cells passing at the robot's ground. A known cell the flood does not
  // reach is an island -- a box top, a table, a platform -- seen without
  // the floor it stands on, which the lidar's -7 deg lower FOV makes the
  // normal view of anything low and near: lethal. Octo3D's connected
  // components, locally.
  void classify(
    double now, double robot_x, double robot_y, double robot_ground_z, unsigned char * cost,
    TerrainStats * stats = nullptr) const;

  // The ground estimate the last classify() made per cell (NaN = none), for
  // tests and debugging.
  const std::vector<float> & lastGround() const { return ground_; }

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  double resolution() const { return res_; }
  double originX() const { return ox_; }
  double originY() const { return oy_; }
  bool worldToCell(double x, double y, int & ix, int & iy) const;
  size_t index(int ix, int iy) const { return static_cast<size_t>(iy) * nx_ + ix; }
  int sampleCount(int ix, int iy) const { return cells_[index(ix, iy)].count; }

  TerrainParams params;

private:
  struct Cell
  {
    std::vector<Sample> ring;
    int count = 0;  // samples held (<= ring.size())
    int head = 0;   // next slot to overwrite
    // The lowest height a ray crossed this cell at, and when (grid time):
    // the ground is under it (the header). Replaced by a lower ray, or by
    // any ray once it is older than ground_memory.
    float ray_lo = std::numeric_limits<float>::infinity();
    float ray_lo_t = -std::numeric_limits<float>::infinity();
  };

  // Ground estimate per cell from the samples, the robot's ground where
  // there are too few.
  void estimateGround(double now, double robot_ground_z) const;
  // The slope rule's plane under the disc at (ix, iy), through the known
  // grounds of the cells that are not obstacles (obstacle_) and that their
  // own returns stand on (groundSupported()).
  bool fitSlope(int ix, int iy, double & tilt_deg) const;
  // Whether a return of cell i's own, within the last estimate's memory,
  // lies within max_ground_lift of its ground: a surface in this column,
  // not a neighbour's borrowed through the 3x3 gather.
  bool groundSupported(size_t i) const;
  bool cellHasObstacle(size_t i, double now, double ground) const;
  // The reach flood (classify()) into `reached`: from the robot's cell, and
  // from the path's poses when `with_path`.
  void floodReach(
    double robot_x, double robot_y, bool with_path, std::vector<uint8_t> & reached) const;
  // The height each cell's returns are judged against for the obstacle rule:
  // its ground, or for a sparse cell the nearest walkable known ground within
  // sparse_ground_radius (the header has why).
  void obstacleReference() const;
  float rel(double t) const;
  // How far above the lowest return a return is overhead (see the header).
  float overheadMargin() const;
  // The cell's lowest z among samples no older than `oldest` (grid time);
  // +infinity when there is none.
  static float lowestSince(const Cell & c, float oldest);
  // clearRay()'s work on cell `i`: drop the samples older than `ts` within
  // `tol` of height `h` -- except those at the cell's ground that the ray
  // passes over (the header has why) -- keeping the rest in their order.
  void clearCell(size_t i, float h, float ts, float tol);

  int nx_ = 0, ny_ = 0;
  double res_ = 0.05, ox_ = 0.0, oy_ = 0.0;
  mutable double epoch_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t cleared_ = 0;
  std::vector<Sample> ring_scratch_;
  std::vector<Cell> cells_;
  // updateOrigin()'s other buffer, the same size with its rings allocated,
  // swapped with cells_ on each shift.
  std::vector<Cell> shifted_;
  std::vector<TerrainPathPose> path_;
  bool corridor_ = true;
  // Scratch of classify(): the cells on the plan's corridor, the plan's
  // height at the nearest flat-gait sample over each (the obstacle rule's
  // ground there) and that sample's distance, for the nearest to win.
  mutable std::vector<uint8_t> on_plan_;
  mutable std::vector<float> plan_z_;
  mutable std::vector<float> plan_dist_;
  void markPlan() const;
  // Scratch of classify(): the ground per cell (NaN = none), whether it came
  // from samples, the reach flood, and the neighbourhood's samples.
  mutable std::vector<float> ground_;
  mutable std::vector<uint8_t> ground_known_;
  mutable std::vector<uint8_t> reached_;
  // The flood from the robot's own cell only, and the obstacle rule's
  // reference height per cell.
  mutable std::vector<uint8_t> walkable_;
  mutable std::vector<float> obstacle_ref_;
  // The obstacle rule's verdict per cell, before the other rules: the slope
  // fit leaves these out (the header has why).
  mutable std::vector<uint8_t> obstacle_;
  mutable std::vector<float> scratch_;
  // The last estimate's memory window (grid time), for groundSupported().
  mutable float ground_oldest_ = 0.f;
  mutable std::vector<int> queue_;
};

}  // namespace syncai_costmap

#endif  // SYNCAI_COSTMAP__TERRAIN_GRID_HPP_
