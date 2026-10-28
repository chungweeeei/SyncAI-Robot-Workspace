#include "syncai_costmap/terrain_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "syncai_costmap/cost_values.hpp"

namespace syncai_costmap
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
  // The ground estimates with them: clearCell() reads them, and a stale one
  // would protect the new samples of a cell that has moved.
  ground_.assign(cells_.size(), kNaN);
  ground_known_.assign(cells_.size(), 0);
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
  // The ground estimates are per old cell; until the next classify() (the
  // caller's next step) clearCell() treats every sample as ground.
  ground_.assign(cells_.size(), kNaN);
  ground_known_.assign(cells_.size(), 0);
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

float TerrainGrid::overheadMargin() const
{
  // The ground estimate is at most the lowest return + max_ground_lift, and
  // the obstacle band ends body_clearance above the ground: a return further
  // than this over the lowest is above the band whatever the ground comes
  // out as. The cost of the lift in the sum: an underside between
  // body_clearance and this above the floor is kept, can lift the ground,
  // and reads as an obstacle -- conservative, for a clearance of a few
  // centimetres over the body's.
  return static_cast<float>(params.body_clearance + params.max_ground_lift);
}

float TerrainGrid::lowestSince(const Cell & c, float oldest)
{
  float low = std::numeric_limits<float>::infinity();
  for (int k = 0; k < c.count; ++k) {
    if (c.ring[k].t >= oldest) {
      low = std::min(low, c.ring[k].z);
    }
  }
  return low;
}

void TerrainGrid::addSample(double x, double y, double z, double t)
{
  int ix, iy;
  if (!worldToCell(x, y, ix, iy)) {
    return;
  }
  Cell & c = cells_[index(ix, iy)];
  const float ts = rel(t);
  // Overhead returns are not stored (2026-10-22; the header has the case).
  // Rejected here, not only left out of the estimate, because the ring keeps
  // the newest max_samples_per_cell: under a stair's underside the returns
  // from right overhead arrive many times faster than the floor's, seen
  // seconds earlier from further off, and stored they push it out of the
  // ring. Against the cell's own lowest return (an empty cell takes
  // anything): the 3x3 lowest would also drop the upper floor's returns next
  // to a drop-off, and estimateGround() repeats the test over the 3x3 for
  // the cells whose own ring got overhead returns before any floor.
  if (
    static_cast<float>(z) >
    lowestSince(c, ts - static_cast<float>(params.ground_memory)) + overheadMargin()) {
    return;
  }
  c.ring[c.head] = Sample{static_cast<float>(z), ts};
  c.head = (c.head + 1) % static_cast<int>(c.ring.size());
  c.count = std::min(c.count + 1, static_cast<int>(c.ring.size()));
}

void TerrainGrid::clearCell(size_t i, float h, float ts, float tol)
{
  Cell & c = cells_[i];
  if (c.count == 0) {
    return;
  }
  // The ground rule (the header): a sample within min_obstacle_height over
  // the cell's last ground estimate is cleared only by a ray under it. A cell
  // never classified yet (NaN) has every sample at its ground.
  const float g = ground_[i];
  const float ground_top = std::isnan(g) ? std::numeric_limits<float>::infinity()
                                         : g + static_cast<float>(params.min_obstacle_height);
  auto disproved = [&](const Sample & s) {
    return s.t < ts && std::fabs(s.z - h) <= tol && !(s.z <= ground_top && h >= s.z);
  };
  // Samples occupy ring[(head - count + k) % size], k = 0 .. count - 1,
  // oldest first. A sample of the same scan (t == ts) is kept: this scan's
  // returns on a box prove the box, whatever its other rays pass over.
  const int size = static_cast<int>(c.ring.size());
  const int start = ((c.head - c.count) % size + size) % size;
  int drop = 0;
  for (int k = 0; k < c.count; ++k) {
    if (disproved(c.ring[(start + k) % size])) {
      ++drop;
    }
  }
  if (drop == 0) {
    return;
  }
  // Compact the survivors to the front, in order; the ring goes on from
  // there, so the next overwrite is still the oldest.
  ring_scratch_.clear();
  for (int k = 0; k < c.count; ++k) {
    const Sample & s = c.ring[(start + k) % size];
    if (!disproved(s)) {
      ring_scratch_.push_back(s);
    }
  }
  std::copy(ring_scratch_.begin(), ring_scratch_.end(), c.ring.begin());
  c.count = static_cast<int>(ring_scratch_.size());
  c.head = c.count % size;
  cleared_ += static_cast<uint64_t>(drop);
}

void TerrainGrid::clearRay(double ox, double oy, double oz, double x, double y, double z, double t)
{
  const float tol = static_cast<float>(params.ray_clear_tolerance);
  if (tol <= 0.0f || cells_.empty()) {
    return;
  }
  const double dx = x - ox, dy = y - oy;
  const double len = std::hypot(dx, dy);
  // The part of the ray that clears: up to the end margin before the return.
  // A return closer than the margin (the robot's own legs, the floor right
  // under a lidar held low) clears nothing; a near-vertical ray crosses no
  // cell but the lidar's own.
  const double reach = len - params.ray_clear_end_margin;
  if (reach <= 0.0 || len < 1e-6) {
    return;
  }
  int ix, iy;
  if (!worldToCell(ox, oy, ix, iy)) {
    // The lidar outside the window: the rolling window is centred on the
    // robot, so only before the first updateOrigin(). Nothing to do.
    return;
  }
  const float ts = rel(t);
  const double dz_ds = (z - oz) / len;

  // Amanatides-Woo through the cells, parameterised by the distance s along
  // the ray's projection: s_x / s_y are where it next crosses a vertical /
  // horizontal grid line, step_* the cell step then, d_* the s per cell.
  const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
  const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
  const double inf = std::numeric_limits<double>::infinity();
  const double d_x = step_x != 0 ? res_ * len / std::fabs(dx) : inf;
  const double d_y = step_y != 0 ? res_ * len / std::fabs(dy) : inf;
  // Clamped to [0, d]: the lidar on a cell boundary within rounding, with a
  // ray almost along it, otherwise gives a crossing metres *behind* the
  // lidar (-4e-16 / 2e-16), and the first cell's height is read there.
  double s_x = inf, s_y = inf;
  if (step_x > 0) {
    s_x = std::clamp((ox_ + (ix + 1) * res_ - ox) * len / dx, 0.0, d_x);
  } else if (step_x < 0) {
    s_x = std::clamp((ox - (ox_ + ix * res_)) * len / -dx, 0.0, d_x);
  }
  if (step_y > 0) {
    s_y = std::clamp((oy_ + (iy + 1) * res_ - oy) * len / dy, 0.0, d_y);
  } else if (step_y < 0) {
    s_y = std::clamp((oy - (oy_ + iy * res_)) * len / -dy, 0.0, d_y);
  }

  double s_in = 0.0;
  bool first = true;
  while (s_in < reach) {
    const double s_out = std::min(std::min(s_x, s_y), reach);
    // The lidar's own cell holds the robot's legs; the footprint is cleared
    // anyway. Every other cell: the ray's height at the middle of its
    // crossing (it drops at most a centimetre across a cell at the angles
    // the lidar sees the floor at, inside the tolerance).
    if (!first) {
      const float h = static_cast<float>(oz + dz_ds * 0.5 * (s_in + s_out));
      clearCell(index(ix, iy), h, ts, tol);
    }
    first = false;
    if (s_x < s_y) {
      ix += step_x;
      s_in = s_x;
      s_x += d_x;
    } else {
      iy += step_y;
      s_in = s_y;
      s_y += d_y;
    }
    if (ix < 0 || iy < 0 || ix >= nx_ || iy >= ny_) {
      break;
    }
  }
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
      // Overhead returns out, against the neighbourhood's lowest (2026-10-22;
      // addSample() keeps them out of a ring that already holds a floor, this
      // catches those a ring took before it had one). They are not ground
      // evidence either, so min_ground_samples counts what is left: a few
      // floor returns under a lot of underside borrow the robot's ground.
      float zmin = std::numeric_limits<float>::infinity();
      for (const float z : scratch_) {
        zmin = std::min(zmin, z);
      }
      const float ceiling = zmin + overheadMargin();
      scratch_.erase(
        std::remove_if(
          scratch_.begin(), scratch_.end(), [ceiling](float z) { return z > ceiling; }),
        scratch_.end());
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

void TerrainGrid::floodReach(
  double robot_x, double robot_y, bool with_path, std::vector<uint8_t> & reached) const
{
  reached.assign(cells_.size(), 0);
  queue_.clear();
  auto seed = [&](double x, double y) {
    int ix, iy;
    if (worldToCell(x, y, ix, iy)) {
      const size_t i = index(ix, iy);
      if (!reached[i]) {
        reached[i] = 1;
        queue_.push_back(static_cast<int>(i));
      }
    }
  };
  seed(robot_x, robot_y);
  if (with_path) {
    for (const auto & p : path_) {
      seed(p.x, p.y);
    }
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
      if (reached[j] || std::fabs(ground_[j] - g) > params.max_step_height) {
        continue;
      }
      reached[j] = 1;
      queue_.push_back(static_cast<int>(j));
    }
  }
}

void TerrainGrid::obstacleReference() const
{
  obstacle_ref_.assign(ground_.begin(), ground_.end());
  const int r = static_cast<int>(std::floor(params.sparse_ground_radius / res_ + 1e-9));
  if (r <= 0) {
    return;
  }
  const int r2 = r * r;
  for (int iy = 0; iy < ny_; ++iy) {
    for (int ix = 0; ix < nx_; ++ix) {
      const size_t i = index(ix, iy);
      // Only a cell with returns and no ground of its own; one with no
      // returns has nothing to judge.
      if (ground_known_[i] || cells_[i].count == 0) {
        continue;
      }
      int best = r2 + 1;
      for (int ddy = -r; ddy <= r; ++ddy) {
        for (int ddx = -r; ddx <= r; ++ddx) {
          const int d2 = ddx * ddx + ddy * ddy;
          const int nx = ix + ddx, ny = iy + ddy;
          if (d2 >= best || nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
            continue;
          }
          const size_t j = index(nx, ny);
          if (ground_known_[j] && walkable_[j]) {
            best = d2;
            obstacle_ref_[i] = ground_[j];
          }
        }
      }
    }
  }
}

void TerrainGrid::classify(
  double now, double robot_x, double robot_y, double robot_ground_z, unsigned char * cost,
  TerrainStats * stats) const
{
  estimateGround(now, robot_ground_z);
  floodReach(robot_x, robot_y, true, reached_);
  // The floor the robot can walk onto from where it stands, without the
  // path's seeds (they reach a stair's treads): what a sparse cell may be
  // judged against.
  if (params.sparse_ground_radius > 0.0) {
    floodReach(robot_x, robot_y, false, walkable_);
  }
  obstacleReference();
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
      // person, a tread two risers up. A sparse cell's band starts from
      // obstacleReference()'s height, not the borrowed one.
      if (cellHasObstacle(i, now, obstacle_ref_[i])) {
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

}  // namespace syncai_costmap
