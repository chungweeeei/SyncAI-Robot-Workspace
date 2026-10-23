#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace syncai_global_planner::octomap_planner_3d
{

void dilateOnSurface(
  const SurfaceMap & m, const std::vector<CellId> & seeds, const std::vector<Offset> & disc,
  std::vector<float> & d)
{
  const double tan_slope = std::tan(m.params.max_slope_deg * M_PI / 180.0);
  for (const CellId h : seeds) {
    d[h] = 0.0f;
    const int ix = m.colIx(m.cell_col[h]), iy = m.colIy(m.cell_col[h]);
    const int kz = m.cells[h].kz;
    for (const Offset & o : disc) {
      const int nx_ = ix + o.dx, ny_ = iy + o.dy;
      if (!m.inGrid(nx_, ny_)) {
        continue;
      }
      const int tol = m.step_cells + static_cast<int>(std::ceil(tan_slope * o.d_m / m.res - 1e-9));
      const CellId n = m.layerNear(m.col(nx_, ny_), kz, tol);
      if (n != kNoCell && o.d_m < d[n]) {
        d[n] = o.d_m;
      }
    }
  }
}

DynamicOverlay::DynamicOverlay(const DynamicOverlayParams & p) : p_(p)
{
}

void DynamicOverlay::setMap(std::shared_ptr<const SurfaceMap> map)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = std::move(map);
  keepout_.reset();
  const size_t n = map_ ? map_->cells.size() : 0;
  last_hit_.assign(n, kNever);
  in_hot_.assign(n, 0);
  hot_.clear();
  mask_.reset();
  dirty_ = true;
  have_epoch_ = false;
  rebuildDisc();
}

std::shared_ptr<const SurfaceMap> DynamicOverlay::map() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return map_;
}

void DynamicOverlay::setKeepout(KeepoutMaskPtr keepout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // A mask is per map; without one there is nothing it could index.
  keepout_ = map_ ? std::move(keepout) : nullptr;
}

std::pair<std::shared_ptr<const SurfaceMap>, KeepoutMaskPtr> DynamicOverlay::mapAndKeepout() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {map_, keepout_};
}

void DynamicOverlay::setParams(const DynamicOverlayParams & p)
{
  std::lock_guard<std::mutex> lock(mutex_);
  p_ = p;
  rebuildDisc();
  mask_.reset();
  dirty_ = true;
}

DynamicOverlayParams DynamicOverlay::params() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return p_;
}

void DynamicOverlay::rebuildDisc()
{
  disc_.clear();
  if (map_) {
    disc_ = discOffsets(p_.block_radius + map_->params.clearance_cost_radius, map_->res, false);
  }
}

CellId DynamicOverlay::layerUnder(const SurfaceMap & m, uint32_t c, double z)
{
  // Sorted by kz: the last layer whose top face is at most one cell above z.
  CellId best = kNoCell;
  const double lim = z + m.res;
  for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
    if (m.zTop(i) > lim) {
      break;
    }
    best = i;
  }
  return best;
}

ScanStats DynamicOverlay::addScan(
  const std::vector<Waypoint> & points, const Waypoint & origin, double t)
{
  ScanStats st;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_) {
    return st;
  }
  const SurfaceMap & m = *map_;
  if (!have_epoch_) {
    epoch_ = t;
    have_epoch_ = true;
  }
  const float stamp = static_cast<float>(t - epoch_);
  const double self2 = p_.self_radius * p_.self_radius;
  const double mark2 = p_.obstacle_max_range * p_.obstacle_max_range;
  const double band_lo = p_.min_obstacle_height, band_hi = m.params.body_clearance;
  const int oix = m.ixOf(origin.x), oiy = m.iyOf(origin.y);
  st.points = points.size();

  // Pass 1: clearing. Each ray is walked column by column from the sensor
  // to its return (or to raytrace_max_range, whichever is nearer), with the
  // height the ray has over each column interpolated from its ends; the
  // surface under that height is the one whose band the ray crosses there,
  // and a mark in it is seen empty. The return's own column is left to pass
  // 2: the point is evidence for that column, not against it. A ray at 1.2 m
  // (a wall point) clears the band near the sensor, where it is still low,
  // and nothing further out; a ray that ends below a band (the floor of a
  // ramp seen from above) clears nothing there. An origin off the map (the
  // robot outside the octree's bounds) traces nothing.
  if (p_.raytrace_max_range > 0.0 && m.inGrid(oix, oiy)) {
    for (const Waypoint & p : points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      double rx = p.x - origin.x, ry = p.y - origin.y, rz = p.z - origin.z;
      const double d2 = rx * rx + ry * ry;
      if (d2 < self2 || d2 <= 1e-12) {
        continue;
      }
      const double d = std::sqrt(d2);
      if (d > p_.raytrace_max_range) {
        const double s = p_.raytrace_max_range / d;
        rx *= s;
        ry *= s;
        rz *= s;
      }
      const double r2 = rx * rx + ry * ry;
      const int tix = m.ixOf(origin.x + rx), tiy = m.iyOf(origin.y + ry);
      supercover(oix, oiy, tix, tiy, [&](int ix, int iy) {
        if (!m.inGrid(ix, iy) || (ix == tix && iy == tiy)) {
          return false;
        }
        const double s =
          std::clamp(((m.cx(ix) - origin.x) * rx + (m.cy(iy) - origin.y) * ry) / r2, 0.0, 1.0);
        const double z_ray = origin.z + s * rz;
        const CellId cell = layerUnder(m, m.col(ix, iy), z_ray);
        if (cell != kNoCell && last_hit_[cell] != kNever) {
          const double rel = z_ray - m.zTop(cell);
          if (rel > band_lo && rel < band_hi) {
            last_hit_[cell] = kNever;
            ++st.cleared;
            dirty_ = true;
          }
        }
        return true;
      });
    }
  }

  // Pass 2: marking.
  for (const Waypoint & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double dx = p.x - origin.x, dy = p.y - origin.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < self2 || d2 > mark2) {
      continue;
    }
    ++st.in_range;
    const int ix = m.ixOf(p.x), iy = m.iyOf(p.y);
    if (!m.inGrid(ix, iy)) {
      continue;
    }
    const CellId cell = layerUnder(m, m.col(ix, iy), p.z);
    if (cell == kNoCell || !m.traversable(cell)) {
      continue;
    }
    const double rel = p.z - m.zTop(cell);
    if (rel <= band_lo || rel >= band_hi) {
      continue;
    }
    if (!in_hot_[cell]) {
      in_hot_[cell] = 1;
      hot_.push_back(cell);
    }
    last_hit_[cell] = stamp;
    ++st.marked;
    dirty_ = true;
  }
  return st;
}

DynamicMaskPtr DynamicOverlay::snapshot(double now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_ || hot_.empty()) {
    mask_.reset();
    dirty_ = false;
    return nullptr;
  }
  // Age out, and drop the cells a ray cleared since the last snapshot.
  const float oldest = static_cast<float>(now - epoch_ - p_.obstacle_memory);
  size_t w = 0;
  for (const CellId c : hot_) {
    if (last_hit_[c] == kNever || last_hit_[c] < oldest) {
      last_hit_[c] = kNever;
      in_hot_[c] = 0;
      dirty_ = true;
      continue;
    }
    hot_[w++] = c;
  }
  hot_.resize(w);
  if (hot_.empty()) {
    mask_.reset();
    dirty_ = false;
    return nullptr;
  }
  if (!dirty_ && mask_) {
    return mask_;
  }

  // Dilate: every column within block_radius + clearance_cost_radius of a
  // mark, on the mark's floor (dilateOnSurface, shared with the keepout
  // rasteriser).
  const SurfaceMap & m = *map_;
  auto mask = std::make_shared<DynamicMask>();
  mask->block_radius = p_.block_radius;
  mask->marked = hot_.size();
  mask->d.assign(m.cells.size(), std::numeric_limits<float>::infinity());
  dilateOnSurface(m, hot_, disc_, mask->d);
  mask_ = mask;
  dirty_ = false;
  return mask_;
}

size_t DynamicOverlay::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t n = 0;
  for (const CellId c : hot_) {
    n += last_hit_[c] != kNever;
    last_hit_[c] = kNever;
    in_hot_[c] = 0;
  }
  hot_.clear();
  mask_.reset();
  dirty_ = false;
  return n;
}

size_t DynamicOverlay::hotCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t n = 0;
  for (const CellId c : hot_) {
    n += last_hit_[c] != kNever;
  }
  return n;
}

}  // namespace syncai_global_planner::octomap_planner_3d
