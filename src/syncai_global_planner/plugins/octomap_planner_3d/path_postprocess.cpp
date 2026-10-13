#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace syncai_global_planner::octomap_planner_3d
{

namespace
{

// Supercover line (E. Dedu): every grid cell the segment from (x0, y0) to
// (x1, y1) passes through, in order, both cells at an exact corner crossing.
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

}  // namespace

bool lineOfSight(
  const SurfaceMap & m, CellId a, CellId b, float min_clearance, std::vector<CellId> * out)
{
  const int ax = m.colIx(m.cell_col[a]), ay = m.colIy(m.cell_col[a]);
  const int bx = m.colIx(m.cell_col[b]), by = m.colIy(m.cell_col[b]);
  int kz = m.cells[a].kz;
  CellId last = kNoCell;
  if (out) {
    out->clear();
  }
  const bool ok = supercover(ax, ay, bx, by, [&](int ix, int iy) {
    if (!m.inGrid(ix, iy)) {
      return false;
    }
    const CellId c = m.layerNear(m.col(ix, iy), kz, m.step_cells);
    if (c == kNoCell || !m.traversable(c) || m.cells[c].clearance_d < min_clearance) {
      return false;
    }
    kz = m.cells[c].kz;
    if (c != last && out) {
      out->push_back(c);
    }
    last = c;
    return true;
  });
  return ok && last == b;
}

std::vector<CellId> shortcutPath(const SurfaceMap & m, const std::vector<CellId> & cells)
{
  std::vector<CellId> out;
  if (cells.size() < 3) {
    return cells;
  }
  out.push_back(cells[0]);
  size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    size_t j = anchor + 1;
    // The clearance the replaced sub-path kept; the shortcut may not do worse.
    float min_clear = std::min(m.cells[cells[anchor]].clearance_d, m.cells[cells[j]].clearance_d);
    while (j + 1 < cells.size()) {
      const float next_clear = std::min(min_clear, m.cells[cells[j + 1]].clearance_d);
      if (!lineOfSight(m, cells[anchor], cells[j + 1], next_clear - 1e-4f)) {
        break;
      }
      min_clear = next_clear;
      ++j;
    }
    out.push_back(cells[j]);
    anchor = j;
  }
  return out;
}

std::vector<Waypoint> resamplePath(
  const SurfaceMap & m, const std::vector<CellId> & cells, double spacing)
{
  std::vector<Waypoint> out;
  if (cells.empty()) {
    return out;
  }
  spacing = std::max(spacing, m.res * 0.25);
  auto at = [&](CellId c) {
    Waypoint w;
    m.cellXY(c, w.x, w.y);
    w.z = m.zTop(c);
    return w;
  };
  auto surfaceZ = [&](double x, double y, double z_guess) {
    const int ix = m.ixOf(x), iy = m.iyOf(y);
    if (!m.inGrid(ix, iy)) {
      return z_guess;
    }
    const int kz = static_cast<int>(std::floor((z_guess - m.z0) / m.res)) - 1;
    const CellId c = m.layerNear(m.col(ix, iy), kz, 3);
    return c == kNoCell ? z_guess : m.zTop(c);
  };
  Waypoint prev = at(cells[0]);
  out.push_back(prev);
  double carry = 0.0;  // distance already covered toward the next sample
  for (size_t i = 1; i < cells.size(); ++i) {
    const Waypoint next = at(cells[i]);
    const double seg = std::hypot(next.x - prev.x, next.y - prev.y);
    double s = spacing - carry;
    while (s < seg - 1e-9) {
      const double t = s / seg;
      Waypoint w{
        prev.x + t * (next.x - prev.x), prev.y + t * (next.y - prev.y),
        prev.z + t * (next.z - prev.z)};
      w.z = surfaceZ(w.x, w.y, w.z);
      out.push_back(w);
      s += spacing;
    }
    carry = seg - (s - spacing);
    prev = next;
  }
  const Waypoint & last = out.back();
  if (std::hypot(last.x - prev.x, last.y - prev.y) > 1e-9) {
    out.push_back(prev);
  }
  return out;
}

}  // namespace syncai_global_planner::octomap_planner_3d
