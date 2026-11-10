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
  const BlockedView * blocked, const StairLinks * links)
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
    if (links && last != kNoCell && links->kerbCrossing(m, last, c)) {
      return false;  // a kerb is crossed on its link, not by a shortcut
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
  const SurfaceMap & m, const std::vector<CellId> & cells, const BlockedView * blocked,
  const StairLinks * links)
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
      if (!lineOfSight(
            m, cells[anchor], cells[j + 1], next_clear - 1e-4f, nullptr, blocked, links)) {
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

namespace
{

// A walk over the floor by points rather than cells (an arc's samples are not
// cell centres): each point's column is resolved to the layer within one step
// of the last one, as lineOfSight() resolves a segment's, and the smallest
// clearance met is kept.
struct FloorWalk
{
  const SurfaceMap & m;
  const BlockedView * blocked;
  const StairLinks * links;
  int kz;
  CellId last = kNoCell;
  float clearance = std::numeric_limits<float>::infinity();

  bool step(double x, double y)
  {
    const int ix = m.ixOf(x), iy = m.iyOf(y);
    if (!m.inGrid(ix, iy)) {
      return false;
    }
    const CellId c = m.layerNear(m.col(ix, iy), kz, m.step_cells);
    if (c == kNoCell || !m.traversable(c) || (blocked && blocked->blocked(m, c))) {
      return false;
    }
    if (c == last) {
      return true;
    }
    if (links && last != kNoCell && links->kerbCrossing(m, last, c)) {
      return false;
    }
    clearance = std::min(clearance, blocked ? blocked->clearance(m, c) : m.cells[c].clearance_d);
    kz = m.cells[c].kz;
    last = c;
    return true;
  }
  // The straight line from (x0, y0), not sampled itself, to (x1, y1).
  bool line(double x0, double y0, double x1, double y1)
  {
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) / ds())));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      if (!step(x0 + t * (x1 - x0), y0 + t * (y1 - y0))) {
        return false;
      }
    }
    return true;
  }
  double ds() const { return m.res / 4; }
};

}  // namespace

CornerArc cornerArc(
  const SurfaceMap & m, CellId a, CellId v, CellId b, double radius, const BlockedView * blocked,
  const StairLinks * links)
{
  CornerArc out;
  double ax, ay, vx, vy, bx, by;
  m.cellXY(a, ax, ay);
  m.cellXY(v, vx, vy);
  m.cellXY(b, bx, by);
  double u1x = ax - vx, u1y = ay - vy, u2x = bx - vx, u2y = by - vy;
  const double l1 = std::hypot(u1x, u1y), l2 = std::hypot(u2x, u2y);
  if (l1 < 1e-9 || l2 < 1e-9) {
    return out;
  }
  u1x /= l1, u1y /= l1, u2x /= l2, u2y /= l2;
  // The angle between the two segments at v: pi on a straight line, 0 where
  // the route turns back on itself.
  const double phi = std::acos(std::clamp(u1x * u2x + u1y * u2y, -1.0, 1.0));
  out.turn = M_PI - phi;
  if (phi < 1e-3 || out.turn < 1e-3) {
    return out;  // a reversal has no arc, a straight line needs none
  }
  const double half = phi / 2;
  // Tangent points no further than half-way along either segment, so the
  // arcs of two neighbouring corners never overlap.
  out.max_radius = std::tan(half) * std::min(l1, l2) / 2;
  if (radius <= 0.0 || radius > out.max_radius + 1e-9) {
    return out;
  }
  const double t = radius / std::tan(half);
  const double t1x = vx + u1x * t, t1y = vy + u1y * t, t2x = vx + u2x * t, t2y = vy + u2y * t;
  const double bl = std::hypot(u1x + u2x, u1y + u2y);
  const double to_centre = radius / std::sin(half);
  const double cx = vx + (u1x + u2x) / bl * to_centre, cy = vy + (u1y + u2y) / bl * to_centre;
  out.deviation = to_centre - radius;

  // The two segment ends the arc replaces, each walked out from v: their
  // clearance, and the layer the arc has to start on and land on.
  FloorWalk in{m, blocked, links, m.cells[v].kz};
  FloorWalk outw{m, blocked, links, m.cells[v].kz};
  if (
    !in.step(vx, vy) || !in.line(vx, vy, t1x, t1y) || !outw.step(vx, vy) ||
    !outw.line(vx, vy, t2x, t2y)) {
    return out;
  }
  FloorWalk arc{m, blocked, links, in.kz};
  if (!arc.step(t1x, t1y)) {
    return out;
  }
  const double a0 = std::atan2(t1y - cy, t1x - cx);
  const double side = (t1x - cx) * (t2y - cy) - (t1y - cy) * (t2x - cx) > 0 ? 1.0 : -1.0;
  const int n = std::max(1, static_cast<int>(std::ceil(radius * out.turn / arc.ds())));
  for (int k = 1; k <= n; ++k) {
    const double ang = a0 + side * out.turn * k / n;
    if (!arc.step(cx + radius * std::cos(ang), cy + radius * std::sin(ang))) {
      return out;
    }
  }
  if (arc.last != outw.last) {
    return out;  // came down on another layer than the segment does
  }
  out.ok = true;
  out.clearance = arc.clearance;
  out.replaced = std::min(in.clearance, outw.clearance);
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
  // A pose's height is the measured floor where there is one (2026-11-07,
  // SurfaceMap::zFloor), the voxel's top face otherwise: the height every
  // judge of the route then reads, from the gap links' kinds to the terrain
  // layer's ground along the plan. The voxel tops stayed on the stair
  // readers' own points (stairCrossing, the walked links), where a flight's
  // 0.3 m and more swallows the difference.
  auto at = [&](CellId c) {
    Waypoint w;
    m.cellXY(c, w.x, w.y);
    w.z = m.zFloor(c);
    return w;
  };
  auto surfaceZ = [&](double x, double y, double z_guess) {
    const int ix = m.ixOf(x), iy = m.iyOf(y);
    if (!m.inGrid(ix, iy)) {
      return z_guess;
    }
    const int kz = static_cast<int>(std::floor((z_guess - m.z0) / m.res)) - 1;
    const CellId c = m.layerNear(m.col(ix, iy), kz, 3);
    return c == kNoCell ? z_guess : m.zFloor(c);
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

// Appends b to a, dropping b's first point when it repeats a's last (the
// same place, within z_tol in height: a route's poses stand at the measured
// floor and a stair link's at the voxel tops, up to a voxel apart over the
// same cell -- 2026-11-07). Returns how many points it dropped (0 or 1).
size_t append(std::vector<Waypoint> & a, const std::vector<Waypoint> & b, double z_tol = 1e-9)
{
  size_t from = 0;
  if (!a.empty() && !b.empty()) {
    const Waypoint & p = a.back();
    const Waypoint & q = b.front();
    if (std::hypot(p.x - q.x, p.y - q.y) < 1e-9 && std::fabs(p.z - q.z) <= z_tol) {
      from = 1;
    }
  }
  a.insert(a.end(), b.begin() + static_cast<std::ptrdiff_t>(from), b.end());
  return from;
}

// The segment from cell p (one floor) to cell n (the other), straight across
// the straight stair link `link`: valid when it points within
// kStairCrossingToleranceDeg of the flight's uphill direction (either way)
// and its columns are, in order, traversable cells of p's component with at
// least clear_p clearance, then the flight -- surfaces within one step of
// each other, traversable or not, never p's floor again, no longer than the
// link plus a metre -- then traversable cells of n's component with at
// least clear_n clearance, ending on n. With out set, the points every
// `spacing` along it: z on the floors from the cell this walk took in that
// column, one ramp over the flight.
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
  // Column -> phase and, on a floor, the height of the cell taken there, for
  // the samples.
  struct ColumnPhase
  {
    uint32_t col;
    int phase;
    double z;
  };
  std::vector<ColumnPhase> phases;
  std::vector<double> short_at;  // s of the clearance shortfalls
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
    phases.push_back({c, ph, z});
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
    phases.push_back({c, 1, zs});
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
  const auto phaseOf = [&](uint32_t c) -> const ColumnPhase * {
    for (const auto & pc : phases) {
      if (pc.col == c) {
        return &pc;
      }
    }
    return nullptr;
  };
  // A floor sample takes the cell the walk above took in its column
  // (2026-10-22). It used to take the surface nearest the height at which
  // the line left or reached the flight, within 0.3 m. Past dp's 2F step the
  // floor rises 0.3 m in 4 m, and that found a rejected layer under the
  // floor in one column -- a pose 0.6 m low, which MPPI read as a stair going
  // down and walked the rest of the way backwards -- and, where the floor
  // had risen further, nothing, which left the flight's top height (0.3 m
  // low). The walk tracked the floor column by column, as lineOfSight()
  // does, so its cell is the floor's own. A column it did not record (not
  // expected: the samples lie on its line) ramps on the flight and keeps the
  // previous sample's height off it, so it cannot dip either.
  double last_z = z_lo;
  const auto sample = [&](double s) {
    Waypoint w{px + dx * s / len, py + dy * s / len, 0.0};
    const int ix = m.ixOf(w.x), iy = m.iyOf(w.y);
    const ColumnPhase * pc = m.inGrid(ix, iy) ? phaseOf(m.col(ix, iy)) : nullptr;
    if (pc != nullptr && pc->phase != 1) {
      w.z = pc->z;
    } else if (pc != nullptr || (s > s_lo && s < s_hi)) {
      const double t = s_hi > s_lo ? std::clamp((s - s_lo) / (s_hi - s_lo), 0.0, 1.0) : 1.0;
      w.z = z_lo + t * (z_hi - z_lo);
    } else {
      w.z = last_z;
    }
    last_z = w.z;
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
  const BlockedView * blocked, std::vector<PathSpan> * spans)
{
  std::vector<Waypoint> out;
  if (spans) {
    spans->clear();
  }
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
    chains.push_back(shortcut ? shortcutPath(m, seg, blocked, links) : seg);
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
      if (!link.straight || link.leads) {
        continue;  // a kerb's or a gap's own points, leads and all, are the crossing
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

  // A joint is the same cell at the floor and at its voxel top (append()).
  const double z_tol = m.res + 1e-9;
  // The crossing's block, appended: its first waypoint is the last one already
  // out when append() dropped it as the joint -- append() says whether it
  // did, so the span and the joint test cannot drift apart.
  const auto appendSpan = [&](size_t use, const std::vector<Waypoint> & block) -> PathSpan * {
    const size_t before = out.size();
    const size_t dropped = append(out, block, z_tol);
    if (!spans || out.empty()) {
      return nullptr;
    }
    const size_t first = dropped > 0 && before > 0 ? before - 1 : before;
    spans->push_back({use, std::min(first, out.size() - 1), out.size() - 1, {}});
    return &spans->back();
  };

  size_t nvert = 0;
  for (size_t i = 0; i < chains.size(); ++i) {
    nvert += chains[i].size();
    append(out, resamplePath(m, chains[i], spacing), z_tol);
    if (i >= uses.size()) {
      break;
    }
    if (!crossings[i].empty()) {
      nvert += 2;
      appendSpan(i, crossings[i]);
      continue;
    }
    const StairLink & link = links->links[uses[i].link];
    std::vector<Waypoint> pts = link.pts;
    if (uses[i].reversed) {
      std::reverse(pts.begin(), pts.end());
    }
    nvert += pts.size();
    std::vector<Waypoint> flight;
    resamplePolyline(pts, spacing, flight);
    PathSpan * span = appendSpan(i, flight);
    // A landing corner is a point of the link; the resample keeps the
    // polyline's vertices on it or within half a spacing, so the nearest
    // waypoint of the span is the corner.
    if (span) {
      for (const size_t t : link.turns) {
        if (t >= link.pts.size()) {
          continue;
        }
        const Waypoint & c = link.pts[t];
        size_t best = span->first;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t k = span->first; k <= span->last; ++k) {
          const double d = std::hypot(out[k].x - c.x, out[k].y - c.y) + std::fabs(out[k].z - c.z);
          if (d < best_d) {
            best_d = d;
            best = k;
          }
        }
        span->turns.push_back(best);
      }
      std::sort(span->turns.begin(), span->turns.end());
    }
  }
  if (vertices) {
    *vertices = nvert;
  }
  return out;
}

namespace
{

// From cell c, how far (m, xy) the floor stays traversable along (nx, ny):
// the distance to the first sample whose column has no traversable cell
// within a step of the last one, or `limit` if there is none that close.
double floorTo(
  const SurfaceMap & m, CellId c, double x, double y, double nx, double ny, double limit)
{
  const double ds = m.res / 2;
  int kz = m.cells[c].kz;
  int ix = m.ixOf(x), iy = m.iyOf(y);
  for (double s = ds; s < limit; s += ds) {
    const int jx = m.ixOf(x + nx * s), jy = m.iyOf(y + ny * s);
    if (jx == ix && jy == iy) {
      continue;
    }
    const CellId n = m.traversableNeighbour(jx, jy, kz);
    if (n == kNoCell) {
      return s;
    }
    kz = m.cells[n].kz;
    ix = jx;
    iy = jy;
  }
  return limit;
}

}  // namespace

std::vector<NarrowSpan> findNarrowPassages(
  const SurfaceMap & m, const std::vector<Waypoint> & path, double max_width, double lead)
{
  std::vector<NarrowSpan> out;
  if (max_width <= 0.0 || path.size() < 2) {
    return out;
  }
  const double rr = m.params.robot_radius;
  const double reach = std::max(0.0, max_width - 2 * rr);  // one side alone can say "wide"
  std::vector<double> width(path.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < path.size(); ++i) {
    const CellId c = m.traversableAt(path[i].x, path[i].y, path[i].z);
    if (c == kNoCell) {
      continue;
    }
    const Waypoint & a = path[i == 0 ? 0 : i - 1];
    const Waypoint & b = path[std::min(i + 1, path.size() - 1)];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 1e-9) {
      continue;
    }
    const double nx = -(b.y - a.y) / len, ny = (b.x - a.x) / len;
    const double left = floorTo(m, c, path[i].x, path[i].y, nx, ny, reach);
    const double right = floorTo(m, c, path[i].x, path[i].y, -nx, -ny, reach);
    width[i] = left + right + 2 * rr;
  }
  // Route distance at each waypoint, for the lead.
  std::vector<double> along(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    along[i] = along[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  for (size_t i = 0; i < path.size();) {
    if (!(width[i] < max_width)) {
      ++i;
      continue;
    }
    size_t j = i;
    double narrowest = width[i];
    while (j + 1 < path.size() && width[j + 1] < max_width) {
      ++j;
      narrowest = std::min(narrowest, width[j]);
    }
    NarrowSpan sp;
    sp.width = narrowest;
    // The axis: the route through the run, or the local direction for a
    // run of one waypoint.
    const Waypoint & a = path[i == j && i > 0 ? i - 1 : i];
    const Waypoint & b = path[i == j ? std::min(j + 1, path.size() - 1) : j];
    sp.heading = std::atan2(b.y - a.y, b.x - a.x);
    sp.first = i;
    while (sp.first > 0 && along[i] - along[sp.first - 1] <= lead + 1e-9) {
      --sp.first;
    }
    sp.last = j;
    while (sp.last + 1 < path.size() && along[sp.last + 1] - along[j] <= lead + 1e-9) {
      ++sp.last;
    }
    if (!out.empty() && sp.first <= out.back().last + 1) {
      NarrowSpan & prev = out.back();
      prev.last = std::max(prev.last, sp.last);
      if (sp.width < prev.width) {
        prev.width = sp.width;
        prev.heading = sp.heading;
      }
    } else {
      out.push_back(sp);
    }
    i = j + 1;
  }
  return out;
}

}  // namespace syncai_global_planner::octomap_planner_3d
