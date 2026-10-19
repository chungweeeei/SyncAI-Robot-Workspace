#ifndef SYNCAI_COSTMAP_2D__TERRAIN_GRID_HPP_
#define SYNCAI_COSTMAP_2D__TERRAIN_GRID_HPP_

// The local terrain map behind TerrainLayer (2026-10-20): a robot-centric
// 2.5D grid of recent lidar returns, judged cell by cell with the geometry
// rules OctomapPlanner3D judges its static map with (surface_map.hpp's
// SurfaceMapParams -- same names, same values), so that the local costmap
// sees what the global planner sees: a step higher than the legs take, a
// drop-off, a slope too steep, a thing lower than the old obstacle layer's
// 0.58 m floor -- and nothing on a ramp or a stair the path climbs.
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
// Reach: a surface is floor only if the robot can get onto it over steps
// the legs take (classify() has the rule). The lidar's lower FOV makes a
// box, a table or a platform look like a patch of floor 0.3 m up with
// nothing seen under it; flooded from the robot's own cell, such a patch
// is an island and lethal. Octo3D's connected components, locally. The
// path's heights seed the flood too, so a ramp whose foot the lidar never
// looked at is still the floor the planner says it is.
//
// Stairs: the step and slope rules refuse a flight, as Octo3D's do, and the
// global planner climbs one only over a stair link in its path. So the path
// is this grid's second input: along a stretch of it that rises
// stair_min_rise within stair_window (the controllers' stair-mode test),
// cells within stair_corridor_half_width take the path's own height as
// their ground, skip the step and slope rules, and start their obstacle
// band at stair_obstacle_height -- high enough for the tread above, low
// enough for a box on the stairs. A level path (every 2D planner) relaxes
// nothing, so a 2D map keeps every stair lethal.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace syncai_costmap_2d
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
  // How long a sample counts as an obstacle. CMU terrain_analysis's
  // decayTime; the old obstacle layer cleared by raytracing within a cycle.
  double obstacle_memory = 2.0;
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

  // ---- stairs -----------------------------------------------------------
  // A stretch of the path whose height spans stair_min_rise within
  // stair_window (half before, half after a pose) is a climb: the
  // controllers' stair-mode test (stair_min_rise 0.3 within 1 m ahead).
  double stair_min_rise = 0.3;
  double stair_window = 1.0;
  // Cells within this of a climbing pose are the stair corridor: 0.6 m is a
  // typical stair's half-width and leaves the stairwell's walls outside it.
  double stair_corridor_half_width = 0.6;
  // The obstacle band starts here above the path's height on the corridor:
  // above one riser (0.17) and the quantisation under it, under a box.
  double stair_obstacle_height = 0.3;
};

// A pose of the global path in the grid's frame, with whether the path is
// climbing there (the stretch's height spans stair_min_rise within
// stair_window).
struct TerrainPathPose
{
  double x, y, z;
  bool climbing;
};

struct TerrainStats
{
  size_t known = 0, lethal_obstacle = 0, lethal_step = 0, lethal_slope = 0, lethal_island = 0,
         corridor = 0;
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
  // cells that stay inside it (Costmap2D::updateOrigin's shift).
  void updateOrigin(double ox, double oy);
  void clear();

  // A return at (x, y, z) in the grid's frame, seen at time t (seconds;
  // any epoch, the same for every call). Outside the window: ignored.
  void addSample(double x, double y, double z, double t);

  // The current path in the grid's frame, with its heights -- only when it
  // has them (OctomapPlanner3D's; a 2D planner's z = 0 is not a height) --
  // or an empty list. Replaces the previous call's. Its climbing poses open
  // the stair corridor; every pose seeds the reach rule (see classify()).
  void setPath(const std::vector<TerrainPathPose> & poses);

  // Judge every cell at time `now`, with the robot at (robot_x, robot_y) on
  // ground `robot_ground_z`, which is also the estimate for cells that have
  // none. Writes one costmap value per cell (cost_values.hpp:
  // LETHAL_OBSTACLE, FREE_SPACE or NO_INFORMATION) to `cost`, sized nx * ny,
  // row-major as Costmap2D is.
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
  };

  // Ground estimate per cell from the samples alone (NaN where there are too
  // few), then the corridor's and the robot's fallbacks.
  void estimateGround(double now, double robot_ground_z) const;
  bool fitSlope(int ix, int iy, double & tilt_deg) const;
  bool cellHasObstacle(size_t i, double now, double ground, double band_from) const;
  void floodReach(double robot_x, double robot_y) const;
  float rel(double t) const;

  int nx_ = 0, ny_ = 0;
  double res_ = 0.05, ox_ = 0.0, oy_ = 0.0;
  mutable double epoch_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Cell> cells_;
  std::vector<TerrainPathPose> path_;
  // Scratch of classify(): the ground per cell (NaN = none), whether it came
  // from samples, the corridor's height per cell (NaN off the corridor), and
  // the disc's offsets.
  mutable std::vector<float> ground_;
  mutable std::vector<uint8_t> ground_known_;
  mutable std::vector<float> corridor_z_;
  mutable std::vector<uint8_t> reached_;
  mutable std::vector<float> scratch_;
  mutable std::vector<int> queue_;
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__TERRAIN_GRID_HPP_
