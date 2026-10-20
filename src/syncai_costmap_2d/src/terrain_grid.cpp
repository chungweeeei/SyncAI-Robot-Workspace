#include "syncai_costmap_2d/terrain_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "syncai_costmap_2d/cost_values.hpp"

namespace syncai_costmap_2d
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr int kDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int kDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
}  // namespace

void TerrainGrid::resize(int nx, int ny, double res, double ox, double oy)
{
  nx_ = std::max(0, nx);
  ny_ = std::max(0, ny);
  res_ = res;
  ox_ = ox;
  oy_ = oy;
  cells_.assign(static_cast<size_t>(nx_) * ny_, Cell());
  for (auto & c : cells_) {
    c.ring.resize(std::max(1, params.max_samples_per_cell));
  }
  ground_.assign(cells_.size(), kNaN);
  ground_known_.assign(cells_.size(), 0);
}

void TerrainGrid::clear()
{
  for (auto & c : cells_) {
    c.count = 0;
    c.head = 0;
  }
}

void TerrainGrid::updateOrigin(double ox, double oy)
{
  // Whole cells only, as Costmap2D::updateOrigin: the grid's cells stay
  // aligned with the costmap's, which lands each sample in the same cell
  // after the shift as before it.
  const int dx = static_cast<int>(std::floor((ox - ox_) / res_ + 1e-9));
  const int dy = static_cast<int>(std::floor((oy - oy_) / res_ + 1e-9));
  if (dx == 0 && dy == 0) {
    return;
  }
  std::vector<Cell> moved(cells_.size());
  for (auto & c : moved) {
    c.ring.resize(std::max(1, params.max_samples_per_cell));
  }
  for (int iy = 0; iy < ny_; ++iy) {
    const int sy = iy + dy;
    if (sy < 0 || sy >= ny_) {
      continue;
    }
    for (int ix = 0; ix < nx_; ++ix) {
      const int sx = ix + dx;
      if (sx < 0 || sx >= nx_) {
        continue;
      }
      moved[index(ix, iy)] = std::move(cells_[index(sx, sy)]);
    }
  }
  cells_ = std::move(moved);
  ox_ += dx * res_;
  oy_ += dy * res_;
}

bool TerrainGrid::worldToCell(double x, double y, int & ix, int & iy) const
{
  ix = static_cast<int>(std::floor((x - ox_) / res_));
  iy = static_cast<int>(std::floor((y - oy_) / res_));
  return ix >= 0 && iy >= 0 && ix < nx_ && iy < ny_;
}

float TerrainGrid::rel(double t) const
{
  // Sample times are floats relative to the first time seen: a float of
  // absolute ROS seconds (1.7e9) resolves 128 s, a relative one milliseconds
  // for hours.
  if (std::isnan(epoch_)) {
    epoch_ = t;
  }
  return static_cast<float>(t - epoch_);
}

void TerrainGrid::addSample(double x, double y, double z, double t)
{
  int ix, iy;
  if (!worldToCell(x, y, ix, iy)) {
    return;
  }
  Cell & c = cells_[index(ix, iy)];
  c.ring[c.head] = Sample{static_cast<float>(z), rel(t)};
  c.head = (c.head + 1) % static_cast<int>(c.ring.size());
  c.count = std::min(c.count + 1, static_cast<int>(c.ring.size()));
}

void TerrainGrid::setPath(const std::vector<TerrainPathPose> & poses)
{
  path_ = poses;
}

void TerrainGrid::estimateGround(double now, double robot_ground_z) const
{
  ground_.assign(cells_.size(), kNaN);
  ground_known_.assign(cells_.size(), 0);

  const float oldest = rel(now) - static_cast<float>(params.ground_memory);
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      const size_t i = index(ix, iy);
      // The 3x3 neighbourhood's samples within ground_memory (CMU splats
      // each point into its neighbours; gathering is the same thing).
      scratch_.clear();
      for (int ddy = -1; ddy <= 1; ++ddy) {
        for (int ddx = -1; ddx <= 1; ++ddx) {
          const int nx = ix + ddx, ny = iy + ddy;
          if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
            continue;
          }
          const Cell & c = cells_[index(nx, ny)];
          for (int k = 0; k < c.count; ++k) {
            const Sample & s = c.ring[k];
            if (s.t >= oldest) {
              scratch_.push_back(s.z);
            }
          }
        }
      }
      if (static_cast<int>(scratch_.size()) < params.min_ground_samples) {
        // Too few: the robot's own ground. Right on a level floor; and the
        // best there is on the floor the lidar has never seen.
        ground_[i] = static_cast<float>(robot_ground_z);
        ground_known_[i] = 0;
        continue;
      }
      const size_t q =
        static_cast<size_t>(std::clamp(params.ground_quantile, 0.0, 1.0) * (scratch_.size() - 1));
      std::nth_element(scratch_.begin(), scratch_.begin() + q, scratch_.end());
      const float zq = scratch_[q];
      const float zmin = *std::min_element(scratch_.begin(), scratch_.begin() + q + 1);
      ground_[i] = std::min(zq, zmin + static_cast<float>(params.max_ground_lift));
      ground_known_[i] = 1;
    }
  }
}

bool TerrainGrid::cellHasObstacle(size_t i, double now, double ground) const
{
  const Cell & c = cells_[i];
  const float oldest = rel(now) - static_cast<float>(params.obstacle_memory);
  const float lo = static_cast<float>(ground + params.min_obstacle_height);
  const float hi = static_cast<float>(ground + params.body_clearance);
  for (int k = 0; k < c.count; ++k) {
    const Sample & s = c.ring[k];
    if (s.t >= oldest && s.z > lo && s.z < hi) {
      return true;
    }
  }
  return false;
}

bool TerrainGrid::fitSlope(int ix, int iy, double & tilt_deg) const
{
  // Least-squares plane z = a x + b y + c through the known grounds under
  // the disc; tilt = atan(hypot(a, b)). surface_map.cpp's stage 5, without
  // its residual test: a quantile ground is smoother than a voxel top face.
  const int r = static_cast<int>(std::ceil(params.robot_radius / res_));
  const double r2 = params.robot_radius * params.robot_radius;
  double sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
  int n = 0, total = 0;
  float zlo = std::numeric_limits<float>::infinity(), zhi = -zlo;
  for (int ddy = -r; ddy <= r; ++ddy) {
    for (int ddx = -r; ddx <= r; ++ddx) {
      const double x = ddx * res_, y = ddy * res_;
      if (x * x + y * y > r2) {
        continue;
      }
      ++total;
      const int nx = ix + ddx, ny = iy + ddy;
      if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
        continue;
      }
      const size_t j = index(nx, ny);
      if (!ground_known_[j]) {
        continue;
      }
      const double z = ground_[j];
      zlo = std::min(zlo, ground_[j]);
      zhi = std::max(zhi, ground_[j]);
      sx += x;
      sy += y;
      sz += z;
      sxx += x * x;
      sxy += x * y;
      syy += y * y;
      sxz += x * z;
      syz += y * z;
      ++n;
    }
  }
  if (n < 3 || n < params.slope_min_known * total) {
    return false;
  }
  // One step or less across the disc: level ground with a kerb, not fitted.
  if (zhi - zlo <= params.max_step_height + 1e-6) {
    tilt_deg = 0.0;
    return true;
  }
  // Centre the data and solve the 2x2 normal equations.
  const double mx = sx / n, my = sy / n, mz = sz / n;
  const double cxx = sxx - n * mx * mx;
  const double cxy = sxy - n * mx * my;
  const double cyy = syy - n * my * my;
  const double cxz = sxz - n * mx * mz;
  const double cyz = syz - n * my * mz;
  const double det = cxx * cyy - cxy * cxy;
  if (std::fabs(det) < 1e-12) {
    return false;
  }
  const double a = (cxz * cyy - cyz * cxy) / det;
  const double b = (cyz * cxx - cxz * cxy) / det;
  tilt_deg = std::atan(std::hypot(a, b)) * 180.0 / M_PI;
  return true;
}

void TerrainGrid::floodReach(double robot_x, double robot_y) const
{
  reached_.assign(cells_.size(), 0);
  queue_.clear();
  auto seed = [&](double x, double y) {
    int ix, iy;
    if (worldToCell(x, y, ix, iy)) {
      const size_t i = index(ix, iy);
      if (!reached_[i]) {
        reached_[i] = 1;
        queue_.push_back(static_cast<int>(i));
      }
    }
  };
  seed(robot_x, robot_y);
  for (const auto & p : path_) {
    seed(p.x, p.y);
  }
  for (size_t head = 0; head < queue_.size(); ++head) {
    const int i = queue_[head];
    const int ix = i % nx_, iy = i / nx_;
    const float g = ground_[i];
    for (int k = 0; k < 8; ++k) {
      const int nx = ix + kDx8[k], ny = iy + kDy8[k];
      if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
        continue;
      }
      const size_t j = index(nx, ny);
      if (reached_[j] || std::fabs(ground_[j] - g) > params.max_step_height) {
        continue;
      }
      reached_[j] = 1;
      queue_.push_back(static_cast<int>(j));
    }
  }
}

void TerrainGrid::classify(
  double now, double robot_x, double robot_y, double robot_ground_z, unsigned char * cost,
  TerrainStats * stats) const
{
  estimateGround(now, robot_ground_z);
  floodReach(robot_x, robot_y);
  TerrainStats st;
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      const size_t i = index(ix, iy);
      const bool known = ground_known_[i] != 0;
      if (known) {
        ++st.known;
      }
      const double ground = ground_[i];

      // 1. A return in the band above the ground: a wall foot, a box, a
      // person, a tread two risers up.
      if (cellHasObstacle(i, now, ground)) {
        cost[i] = LETHAL_OBSTACLE;
        ++st.lethal_obstacle;
        continue;
      }
      // A cell with neither a ground of its own nor an obstacle is unknown:
      // the floor under the robot, never seen; the far side of a drop.
      if (!known) {
        cost[i] = NO_INFORMATION;
        continue;
      }
      // 2. The reach rule (see the header): a surface the robot cannot get
      // onto over steps it takes is not floor.
      if (!reached_[i]) {
        cost[i] = LETHAL_OBSTACLE;
        ++st.lethal_island;
        continue;
      }
      // 3. The step rule: a neighbour's ground out of the legs' reach -- up
      // (a wall foot, a ledge, a stair's riser above a step) or down (a
      // drop-off, the negative obstacle the obstacle layer could never see).
      // Only known neighbours: the unknown floor under the robot is not a
      // cliff.
      bool step = false;
      for (int k = 0; k < 8 && !step; ++k) {
        const int nx = ix + kDx8[k], ny = iy + kDy8[k];
        if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
          continue;
        }
        const size_t j = index(nx, ny);
        if (!ground_known_[j]) {
          continue;
        }
        if (std::fabs(ground_[j] - ground) > params.max_step_height) {
          step = true;
        }
      }
      if (step) {
        cost[i] = LETHAL_OBSTACLE;
        ++st.lethal_step;
        continue;
      }
      // 4. The slope rule under the footprint disc: a ramp passes, a flight
      // does not.
      double tilt = 0.0;
      if (fitSlope(ix, iy, tilt) && tilt > params.max_slope_deg) {
        cost[i] = LETHAL_OBSTACLE;
        ++st.lethal_slope;
        continue;
      }
      cost[i] = FREE_SPACE;
    }
  }
  if (stats) {
    *stats = st;
  }
}

}  // namespace syncai_costmap_2d
