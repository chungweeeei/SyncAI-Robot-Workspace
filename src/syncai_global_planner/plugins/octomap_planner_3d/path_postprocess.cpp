#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace syncai_global_planner::octomap_planner_3d
{

bool lineOfSight(
  const SurfaceMap & m, CellId a, CellId b, float min_clearance, std::vector<CellId> * out,
  const BlockedView * blocked)
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
    if (c == kNoCell || !m.traversable(c) || (blocked && blocked->blocked(m, c))) {
      return false;
    }
    const float clearance = blocked ? blocked->clearance(m, c) : m.cells[c].clearance_d;
    if (clearance < min_clearance) {
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

std::vector<CellId> shortcutPath(
  const SurfaceMap & m, const std::vector<CellId> & cells, const BlockedView * blocked)
{
  std::vector<CellId> out;
  if (cells.size() < 3) {
    return cells;
  }
  const auto clearOf = [&](CellId c) {
    return blocked ? blocked->clearance(m, c) : m.cells[c].clearance_d;
  };
  out.push_back(cells[0]);
  size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    size_t j = anchor + 1;
    // The clearance the replaced sub-path kept; the shortcut may not do worse.
    float min_clear = std::min(clearOf(cells[anchor]), clearOf(cells[j]));
    while (j + 1 < cells.size()) {
      const float next_clear = std::min(min_clear, clearOf(cells[j + 1]));
      if (!lineOfSight(m, cells[anchor], cells[j + 1], next_clear - 1e-4f, nullptr, blocked)) {
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

namespace
{

// Points every `spacing` metres (xy) along a polyline, both ends included,
// z interpolated: the recorded ground of a flight is already the height the
// path should have, with nothing to snap it to.
void resamplePolyline(
  const std::vector<Waypoint> & pts, double spacing, std::vector<Waypoint> & out)
{
  if (pts.empty()) {
    return;
  }
  out.push_back(pts.front());
  double carry = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const Waypoint & p = pts[i - 1];
    const Waypoint & q = pts[i];
    const double seg = std::hypot(q.x - p.x, q.y - p.y);
    double s = spacing - carry;
    while (s < seg - 1e-9) {
      const double t = s / seg;
      out.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), p.z + t * (q.z - p.z)});
      s += spacing;
    }
    carry = seg - (s - spacing);
  }
  const Waypoint & last = out.back();
  const Waypoint & end = pts.back();
  if (std::hypot(last.x - end.x, last.y - end.y) > 1e-9 || std::fabs(last.z - end.z) > 1e-9) {
    out.push_back(end);
  }
}

// Appends b to a, dropping b's first point when it repeats a's last.
void append(std::vector<Waypoint> & a, const std::vector<Waypoint> & b)
{
  size_t from = 0;
  if (!a.empty() && !b.empty()) {
    const Waypoint & p = a.back();
    const Waypoint & q = b.front();
    if (std::hypot(p.x - q.x, p.y - q.y) < 1e-9 && std::fabs(p.z - q.z) < 1e-9) {
      from = 1;
    }
  }
  a.insert(a.end(), b.begin() + static_cast<std::ptrdiff_t>(from), b.end());
}

// The segment from cell p (one floor) to cell n (the other), straight across
// the straight stair link `link`: valid when it points within
// kStairCrossingToleranceDeg of the flight's uphill direction (either way)
// and its columns are, in order, traversable cells of p's component with at
// least clear_p clearance, then the flight -- surfaces within one step of
// each other, traversable or not, never p's floor again, no longer than the
// link plus a metre -- then traversable cells of n's component with at
// least clear_n clearance, ending on n. With out set, the points every
// `spacing` along it: z from the surface on the floors, one ramp over the
// flight.
bool stairCrossing(
  const SurfaceMap & m, CellId p, CellId n, const StairLink & link, float clear_p, float clear_n,
  double spacing, std::vector<Waypoint> * out, const BlockedView * blocked)
{
  double px, py, nx, ny;
  m.cellXY(p, px, py);
  m.cellXY(n, nx, ny);
  const double dx = nx - px, dy = ny - py, len = std::hypot(dx, dy);
  if (len < m.res) {
    return false;
  }
  const double along_up = (dx * link.up_x + dy * link.up_y) / len;
  if (std::fabs(along_up) < std::cos(kStairCrossingToleranceDeg * M_PI / 180.0)) {
    return false;
  }
  const uint32_t comp_p = m.cells[p].component, comp_n = m.cells[n].component;
  const double step_tol = (m.step_cells + 1) * m.res + 1e-6;
  const double flight_max = link.length + 1.0;
  // Phase 0: p's floor; 1: the flight; 2: n's floor. The floors' edges are
  // ragged where the footprint rule cut them, so a line may step floor ->
  // flight -> floor before it has climbed anything, and flight -> floor ->
  // flight at the top: the phase goes back as long as the height says the
  // line is still on that floor. Clearance is checked on the floors, but a
  // cell at a floor's edge is only as clear as its distance to the flight
  // itself, which the line has to cross; a shortfall within edge_band of
  // where it leaves one floor or lands on the other is therefore allowed,
  // and one anywhere else (a wall) is not.
  int phase = 0;
  int kz = m.cells[p].kz;
  double z = m.zTop(p), flight_len = 0.0;
  double z_lo = z, s_lo = 0.0, z_hi = 0.0, s_hi = 0.0;  // the ramp's two ends, by distance
  const double edge_band = 3 * m.res;
  const auto s_of = [&](int ix, int iy) {
    return ((m.cx(ix) - px) * dx + (m.cy(iy) - py) * dy) / len;
  };
  std::vector<std::pair<uint32_t, int>> phases;  // column -> phase, for the samples
  std::vector<double> short_at;                  // s of the clearance shortfalls
  CellId last = kNoCell;
  const auto onFloor = [&](int ix, int iy, uint32_t comp, float clear, int ph) {
    const uint32_t c = m.col(ix, iy);
    const CellId f = m.layerNear(c, kz, m.step_cells);
    if (
      f == kNoCell || !m.traversable(f) || m.cells[f].component != comp ||
      (blocked && blocked->blocked(m, f))) {
      return false;
    }
    if ((blocked ? blocked->clearance(m, f) : m.cells[f].clearance_d) < clear) {
      short_at.push_back(s_of(ix, iy));
    }
    kz = m.cells[f].kz;
    z = m.zTop(f);
    last = f;
    phase = ph;
    phases.emplace_back(c, ph);
    return true;
  };
  const bool ok = supercover(m.ixOf(px), m.iyOf(py), m.ixOf(nx), m.iyOf(ny), [&](int ix, int iy) {
    if (!m.inGrid(ix, iy)) {
      return false;
    }
    const uint32_t c = m.col(ix, iy);
    if (phase == 0 && onFloor(ix, iy, comp_p, clear_p, 0)) {
      z_lo = z;
      s_lo = s_of(ix, iy);
      return true;
    }
    if (phase == 2 && onFloor(ix, iy, comp_n, clear_n, 2)) {
      return true;
    }
    // On the flight, or at a ragged edge of either floor.
    const CellId s = m.surfaceNear(c, z, step_tol);
    if (s == kNoCell) {
      return false;
    }
    const double zs = m.zTop(s);
    if (m.traversable(s) && m.cells[s].component == comp_p) {
      if (std::fabs(zs - z_lo) > step_tol || phase == 2) {
        return false;  // back on the lower floor after climbing: not a crossing
      }
      kz = m.cells[s].kz;
      return onFloor(ix, iy, comp_p, clear_p, 0) && ((z_lo = z), (s_lo = s_of(ix, iy)), true);
    }
    if (m.traversable(s) && m.cells[s].component == comp_n) {
      kz = m.cells[s].kz;
      if (!onFloor(ix, iy, comp_n, clear_n, 2)) {
        return false;
      }
      if (phase == 2 && s_hi == 0.0) {
        z_hi = z;
        s_hi = s_of(ix, iy);
      }
      return true;
    }
    phase = 1;
    flight_len += m.res;
    if (flight_len > flight_max) {
      return false;
    }
    z = zs;
    phases.emplace_back(c, 1);
    return true;
  });
  if (!ok || phase != 2 || last != n || s_hi <= s_lo) {
    return false;
  }
  for (const double s : short_at) {
    if (std::fabs(s - s_lo) > edge_band && std::fabs(s - s_hi) > edge_band) {
      return false;
    }
  }
  if (!out) {
    return true;
  }
  out->clear();
  const auto phaseOf = [&](uint32_t c) {
    for (const auto & pc : phases) {
      if (pc.first == c) {
        return pc.second;
      }
    }
    return 1;
  };
  const auto sample = [&](double s) {
    Waypoint w{px + dx * s / len, py + dy * s / len, 0.0};
    const int ix = m.ixOf(w.x), iy = m.iyOf(w.y);
    const int ph = m.inGrid(ix, iy) ? phaseOf(m.col(ix, iy)) : 1;
    if (ph == 1) {
      const double t = s_hi > s_lo ? std::clamp((s - s_lo) / (s_hi - s_lo), 0.0, 1.0) : 1.0;
      w.z = z_lo + t * (z_hi - z_lo);
    } else {
      const double guess = ph == 0 ? z_lo : z_hi;
      const int gk = static_cast<int>(std::floor((guess - m.z0) / m.res)) - 1;
      const CellId c = m.layerNear(m.col(ix, iy), gk, 3);
      w.z = c == kNoCell ? guess : m.zTop(c);
    }
    return w;
  };
  for (double s = 0.0; s < len - 1e-9; s += spacing) {
    out->push_back(sample(s));
  }
  out->push_back({nx, ny, m.zTop(n)});
  out->front().z = m.zTop(p);
  return true;
}

}  // namespace

std::vector<Waypoint> buildPath(
  const SurfaceMap & m, const StairLinks * links, const std::vector<CellId> & cells,
  const std::vector<LinkUse> & uses, bool shortcut, double spacing, size_t * vertices,
  const BlockedView * blocked)
{
  std::vector<Waypoint> out;
  if (cells.empty()) {
    if (vertices) {
      *vertices = 0;
    }
    return out;
  }
  spacing = std::max(spacing, m.res * 0.25);
  const auto clearOf = [&](CellId c) {
    return blocked ? blocked->clearance(m, c) : m.cells[c].clearance_d;
  };
  // The floor stretches between the links, each its own (shortcut) chain.
  std::vector<std::vector<CellId>> chains;
  size_t from = 0;
  const auto stretch = [&](size_t to) {
    const std::vector<CellId> seg(
      cells.begin() + static_cast<std::ptrdiff_t>(from),
      cells.begin() + static_cast<std::ptrdiff_t>(to) + 1);
    chains.push_back(shortcut ? shortcutPath(m, seg, blocked) : seg);
    from = to + 1;
  };
  for (const LinkUse & u : uses) {
    stretch(u.index);
  }
  stretch(cells.size() - 1);

  // Over a straight link, try to join the vertex before it to the vertex
  // after it with one segment through the stair (stairCrossing): the
  // longest span first, then the two that keep one of the link's ends.
  std::vector<std::vector<Waypoint>> crossings(uses.size());
  if (shortcut) {
    for (size_t i = 0; i < uses.size(); ++i) {
      const StairLink & link = links->links[uses[i].link];
      if (!link.straight) {
        continue;
      }
      std::vector<CellId> & A = chains[i];
      std::vector<CellId> & B = chains[i + 1];
      const size_t na = A.size(), nb = B.size();
      // (how many vertices to drop at A's end, at B's start)
      const std::pair<size_t, size_t> tries[3] = {{1, 1}, {1, 0}, {0, 1}};
      for (const auto & t : tries) {
        if (t.first >= na || t.second >= nb || (t.first == 0 && t.second == 0)) {
          continue;
        }
        const CellId p = A[na - 1 - t.first], n = B[t.second];
        const float clear_p = std::min(clearOf(p), clearOf(A[na - 1])) - 1e-4f;
        const float clear_n = std::min(clearOf(n), clearOf(B[0])) - 1e-4f;
        std::vector<Waypoint> pts;
        if (stairCrossing(m, p, n, link, clear_p, clear_n, spacing, &pts, blocked)) {
          crossings[i] = std::move(pts);
          A.resize(na - t.first);
          B.erase(B.begin(), B.begin() + static_cast<std::ptrdiff_t>(t.second));
          break;
        }
      }
    }
  }

  size_t nvert = 0;
  for (size_t i = 0; i < chains.size(); ++i) {
    nvert += chains[i].size();
    append(out, resamplePath(m, chains[i], spacing));
    if (i >= uses.size()) {
      break;
    }
    if (!crossings[i].empty()) {
      nvert += 2;
      append(out, crossings[i]);
      continue;
    }
    std::vector<Waypoint> pts = links->links[uses[i].link].pts;
    if (uses[i].reversed) {
      std::reverse(pts.begin(), pts.end());
    }
    nvert += pts.size();
    std::vector<Waypoint> flight;
    resamplePolyline(pts, spacing, flight);
    append(out, flight);
  }
  if (vertices) {
    *vertices = nvert;
  }
  return out;
}

}  // namespace syncai_global_planner::octomap_planner_3d
