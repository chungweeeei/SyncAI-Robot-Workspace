#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <sstream>

namespace syncai_global_planner::octomap_planner_3d
{

bool readTrack(
  const std::string & poses_path, double lidar_height, std::vector<Waypoint> & out,
  std::string & error)
{
  std::ifstream in(poses_path);
  if (!in) {
    error = "cannot open '" + poses_path + "'";
    return false;
  }
  out.clear();
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string name;
    double x, y, z;
    if (ss >> name >> x >> y >> z) {
      out.push_back({x, y, z - lidar_height});
    }
  }
  return true;
}

namespace
{

// The traversable cell a ground point stands on: within one cell in xy (the
// footprint rule trims the floor right next to a wall foot or a stair
// nosing, where the robot still stood) and anchor_z_window in z, nearest in
// xy first, then in z.
CellId anchorCell(const SurfaceMap & m, const Waypoint & p, double z_window)
{
  const int cx = m.ixOf(p.x), cy = m.iyOf(p.y);
  CellId best = kNoCell;
  double best_xy = 0, best_dz = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int ix = cx + dx, iy = cy + dy;
      if (!m.inGrid(ix, iy)) {
        continue;
      }
      const double xy = std::hypot(m.cx(ix) - p.x, m.cy(iy) - p.y);
      const uint32_t c = m.col(ix, iy);
      for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
        const double dz = std::fabs(m.zTop(i) - p.z);
        if (!m.traversable(i) || dz > z_window) {
          continue;
        }
        if (
          best == kNoCell || xy < best_xy - 1e-9 ||
          (std::fabs(xy - best_xy) <= 1e-9 && dz < best_dz)) {
          best = i;
          best_xy = xy;
          best_dz = dz;
        }
      }
    }
  }
  return best;
}

// The polyline with every loop cut out: from each kept point, jump to the
// last later point that comes back near it.
std::vector<Waypoint> cutLoops(const std::vector<Waypoint> & p, double radius, double dz)
{
  std::vector<Waypoint> out;
  if (p.empty()) {
    return out;
  }
  const auto near = [&](const Waypoint & a, const Waypoint & b) {
    return std::hypot(a.x - b.x, a.y - b.y) <= radius && std::fabs(a.z - b.z) <= dz;
  };
  const size_t n = p.size();
  out.push_back(p[0]);
  size_t i = 0;
  while (i + 1 < n) {
    size_t jump = i;
    for (size_t j = n - 1; j > i + 1; --j) {
      if (near(p[i], p[j])) {
        jump = j;
        break;
      }
    }
    if (jump > i) {
      i = jump;
      if (i == n - 1 && out.size() > 1) {
        out.back() = p[n - 1];  // end exactly on the far anchor
      }
      continue;
    }
    ++i;
    out.push_back(p[i]);
  }
  if (out.size() == 1 && n > 1) {
    out.push_back(p[n - 1]);
  }
  return out;
}

// smooth_passes of a three-point moving average, the first and last point
// fixed (they are the anchors' cell centres).
void smoothPolyline(std::vector<Waypoint> & p, int passes)
{
  if (p.size() < 3) {
    return;
  }
  std::vector<Waypoint> q(p.size());
  for (int pass = 0; pass < passes; ++pass) {
    q.front() = p.front();
    q.back() = p.back();
    for (size_t i = 1; i + 1 < p.size(); ++i) {
      q[i] = {
        (p[i - 1].x + p[i].x + p[i + 1].x) / 3.0, (p[i - 1].y + p[i].y + p[i + 1].y) / 3.0,
        (p[i - 1].z + p[i].z + p[i + 1].z) / 3.0};
    }
    p.swap(q);
  }
}

uint32_t findRoot(std::vector<uint32_t> & parent, uint32_t i)
{
  while (parent[i] != i) {
    parent[i] = parent[parent[i]];
    i = parent[i];
  }
  return i;
}

double length3d(const std::vector<Waypoint> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const Waypoint & p = pts[i - 1];
    const Waypoint & q = pts[i];
    len +=
      std::sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
  }
  return len;
}

// The stair's uphill direction: the gradient of a plane fitted through the
// surfaces within radius of the walked stretch track[ka..kb] that lie
// strictly between the two floors' heights -- the risers and treads, which
// the geometry rules rejected but the column pass still recorded. The walk
// itself is not used for the direction on purpose: it is what this replaces.
// False, with why, when the fit is too thin, too flat to have a direction,
// or points more than 60 deg away from the way the robot climbed.
bool fitUphill(
  const SurfaceMap & m, const std::vector<Waypoint> & track, size_t ka, size_t kb, double z_lo,
  double z_hi, double radius, double walk_x, double walk_y, double & ux, double & uy,
  std::string & why)
{
  double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
  for (size_t k = ka; k <= kb; ++k) {
    xmin = std::min(xmin, track[k].x - radius);
    xmax = std::max(xmax, track[k].x + radius);
    ymin = std::min(ymin, track[k].y - radius);
    ymax = std::max(ymax, track[k].y + radius);
  }
  const double band_lo = z_lo + m.res / 2, band_hi = z_hi - m.res / 2;
  std::vector<double> xs, ys, zs;
  for (int iy = std::max(0, m.iyOf(ymin)); iy <= std::min(m.ny - 1, m.iyOf(ymax)); ++iy) {
    for (int ix = std::max(0, m.ixOf(xmin)); ix <= std::min(m.nx - 1, m.ixOf(xmax)); ++ix) {
      const double x = m.cx(ix), y = m.cy(iy);
      bool near = false;
      for (size_t k = ka; k <= kb && !near; ++k) {
        near = std::hypot(track[k].x - x, track[k].y - y) <= radius;
      }
      if (!near) {
        continue;
      }
      const uint32_t c = m.col(ix, iy);
      for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
        const double z = m.zTop(i);
        if (z > band_lo && z < band_hi) {
          xs.push_back(x);
          ys.push_back(y);
          zs.push_back(z);
        }
      }
    }
  }
  const size_t n = xs.size();
  if (n < 12) {
    why = "only " + std::to_string(n) + " surface cells between the floors to fit a plane to";
    return false;
  }
  double mx = 0, my = 0, mz = 0;
  for (size_t i = 0; i < n; ++i) {
    mx += xs[i];
    my += ys[i];
    mz += zs[i];
  }
  mx /= n;
  my /= n;
  mz /= n;
  double sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
  for (size_t i = 0; i < n; ++i) {
    const double dx = xs[i] - mx, dy = ys[i] - my, dz = zs[i] - mz;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
    sxz += dx * dz;
    syz += dy * dz;
  }
  const double det = sxx * syy - sxy * sxy;
  if (det < 1e-9) {
    why = "the surfaces between the floors lie on a line, no plane fits";
    return false;
  }
  const double gx = (sxz * syy - syz * sxy) / det, gy = (syz * sxx - sxz * sxy) / det;
  const double g = std::hypot(gx, gy);
  if (g < std::tan(5.0 * M_PI / 180.0)) {
    why = "the fitted plane is flatter than 5 deg, no uphill direction";
    return false;
  }
  ux = gx / g;
  uy = gy / g;
  const double w = std::hypot(walk_x, walk_y);
  if (w > 1e-9 && (ux * walk_x + uy * walk_y) / w < std::cos(60.0 * M_PI / 180.0)) {
    char buf[120];
    std::snprintf(
      buf, sizeof buf, "the fitted uphill direction (%.0f deg) is more than 60 deg off the walk",
      std::atan2(uy, ux) * 180.0 / M_PI);
    why = buf;
    return false;
  }
  return true;
}

// From the lower-floor cell `entry`, march uphill along (ux, uy) one cell at
// a time over surfaces within one step of each other until a traversable
// cell of component `hi` is stepped on. The result's points are the column
// centres passed, z interpolated from the entry's height to the exit's.
// Empty when the line runs off the flight (no surface within a step), drops
// back (more than a cell downhill), wanders back onto the lower floor (the
// entry was not on the stair's edge, or the direction is skew), or runs
// level for longer than exit_reach before the next rise -- a landing, or no
// upper floor within reach of the top step.
std::vector<Waypoint> marchStraight(
  const SurfaceMap & m, CellId entry, uint32_t lo, uint32_t hi, double ux, double uy,
  double max_len, double exit_reach, CellId & exit)
{
  std::vector<Waypoint> pts;
  double ex, ey;
  m.cellXY(entry, ex, ey);
  const double z_entry = m.zTop(entry);
  const double step_tol = (m.step_cells + 1) * m.res + 1e-6;
  pts.push_back({ex, ey, z_entry});
  uint32_t last_col = m.cell_col[entry];
  double z = z_entry, level = 0.0;
  exit = kNoCell;
  for (int i = 1; i * m.res <= max_len + 2.0; ++i) {
    const double x = ex + i * m.res * ux, y = ey + i * m.res * uy;
    const int ix = m.ixOf(x), iy = m.iyOf(y);
    if (!m.inGrid(ix, iy)) {
      return {};
    }
    const uint32_t c = m.col(ix, iy);
    if (c == last_col) {
      continue;
    }
    last_col = c;
    const CellId s = m.surfaceNear(c, z, step_tol);
    if (s == kNoCell) {
      return {};
    }
    const double zs = m.zTop(s);
    if (zs < z - m.res - 1e-6) {
      return {};
    }
    level = zs > z + m.res / 2 ? 0.0 : level + m.res;
    z = zs;
    pts.push_back({m.cx(ix), m.cy(iy), z});
    if (m.traversable(s)) {
      if (m.cells[s].component == hi) {
        exit = s;
        break;
      }
      if (m.cells[s].component == lo) {
        return {};
      }
    }
    if (level > exit_reach + 1e-6) {
      return {};
    }
  }
  if (exit == kNoCell) {
    return {};
  }
  // The link is the segment between the two cell centres, not the column
  // centres the march passed: those zigzag by a cell where the line crosses
  // a row, and the path would turn 45 deg twice over 0.14 m there. The
  // segment runs within half a cell of the columns the march checked, and
  // buildPath resamples it at the path's spacing with z as one ramp.
  return {pts.front(), pts.back()};
}

// Registers a finished link: its length, the cells it touches and the two
// components it joins.
void addLink(const SurfaceMap & m, StairLink link, StairLinks & out)
{
  link.length = length3d(link.pts);
  const uint32_t id = static_cast<uint32_t>(out.links.size());
  out.at[link.a].push_back(id);
  out.at[link.b].push_back(id);
  const uint32_t ra = findRoot(out.group, m.cells[link.a].component);
  const uint32_t rb = findRoot(out.group, m.cells[link.b].component);
  if (ra != rb) {
    out.group[std::max(ra, rb)] = std::min(ra, rb);
  }
  out.links.push_back(std::move(link));
}

// The row of straight links across the flight the robot walked between
// keyframes ka and kb, from cell a (walked first) to cell b. Returns how many
// it laid; 0, with `why`, when the stretch is not one straight flight -- the
// caller then keeps the walked link.
size_t straightLinks(
  const SurfaceMap & m, const std::vector<Waypoint> & track, size_t ka, size_t kb, CellId a,
  CellId b, const StairLinkParams & p, StairLinks & out, std::string & why)
{
  double walked = 0.0;
  for (size_t k = ka + 1; k <= kb; ++k) {
    walked += std::hypot(track[k].x - track[k - 1].x, track[k].y - track[k - 1].y);
  }
  if (walked > p.straight_max_flight) {
    char buf[120];
    std::snprintf(
      buf, sizeof buf, "%.1f m of track is more than straight_max_flight %.1f m", walked,
      p.straight_max_flight);
    why = buf;
    return 0;
  }
  const bool a_is_lower = m.zTop(a) < m.zTop(b);
  const CellId lo_cell = a_is_lower ? a : b, hi_cell = a_is_lower ? b : a;
  const uint32_t lo = m.cells[lo_cell].component, hi = m.cells[hi_cell].component;
  const double z_lo = m.zTop(lo_cell), z_hi = m.zTop(hi_cell);
  double lx, ly, hx, hy;
  m.cellXY(lo_cell, lx, ly);
  m.cellXY(hi_cell, hx, hy);
  double ux, uy;
  if (!fitUphill(
        m, track, ka, kb, z_lo, z_hi, p.straight_fit_radius, hx - lx, hy - ly, ux, uy, why)) {
    return 0;
  }
  // Every lower-floor cell near the walked foot is tried as an entry; the
  // march itself sorts the stair's edge from the floor behind it (a line
  // from further back runs onto lower-floor cells and is dropped).
  const double reach = p.straight_half_width + 1.0;
  const uint32_t first = static_cast<uint32_t>(out.links.size());
  double perp_min = 1e300, perp_max = -1e300;
  for (int iy = std::max(0, m.iyOf(ly - reach)); iy <= std::min(m.ny - 1, m.iyOf(ly + reach));
       ++iy) {
    for (int ix = std::max(0, m.ixOf(lx - reach)); ix <= std::min(m.nx - 1, m.ixOf(lx + reach));
         ++ix) {
      const double dx = m.cx(ix) - lx, dy = m.cy(iy) - ly;
      const double along = dx * ux + dy * uy, perp = dy * ux - dx * uy;
      if (std::fabs(perp) > p.straight_half_width || std::fabs(along) > 1.0) {
        continue;
      }
      const uint32_t col = m.col(ix, iy);
      for (uint32_t e = m.col_begin[col], end = m.col_begin[col + 1]; e < end; ++e) {
        if (!m.traversable(e) || m.cells[e].component != lo) {
          continue;
        }
        CellId exit = kNoCell;
        std::vector<Waypoint> pts =
          marchStraight(m, e, lo, hi, ux, uy, p.straight_max_flight, p.straight_exit_reach, exit);
        if (pts.empty()) {
          continue;
        }
        StairLink link;
        link.kf_a = ka;
        link.kf_b = kb;
        link.straight = true;
        link.up_x = ux;
        link.up_y = uy;
        if (a_is_lower) {
          link.a = e;
          link.b = exit;
        } else {
          link.a = exit;
          link.b = e;
          std::reverse(pts.begin(), pts.end());
        }
        link.pts = std::move(pts);
        addLink(m, std::move(link), out);
        perp_min = std::min(perp_min, perp);
        perp_max = std::max(perp_max, perp);
      }
    }
  }
  const size_t laid = out.links.size() - first;
  if (laid == 0) {
    why = "no straight line from the lower floor reached the upper one";
    return 0;
  }
  StairFlight f;
  f.kf_a = ka;
  f.kf_b = kb;
  f.first = first;
  f.count = static_cast<uint32_t>(laid);
  f.heading_deg = std::atan2(uy, ux) * 180.0 / M_PI;
  f.width = perp_max - perp_min + m.res;
  f.z_lo = z_lo;
  f.z_hi = z_hi;
  out.flights.push_back(f);
  return laid;
}

}  // namespace

std::shared_ptr<const StairLinks> buildStairLinks(
  const SurfaceMap & m, const std::vector<Waypoint> & track, const StairLinkParams & params)
{
  auto out = std::make_shared<StairLinks>();
  uint32_t ncomp = 0;
  for (const SurfaceCell & c : m.cells) {
    if (c.component != kNoComponent) {
      ncomp = std::max(ncomp, c.component + 1);
    }
  }
  out->group.resize(ncomp);
  std::iota(out->group.begin(), out->group.end(), 0u);
  std::vector<size_t> comp_cells(ncomp, 0);
  for (const SurfaceCell & c : m.cells) {
    if (c.component != kNoComponent) {
      ++comp_cells[c.component];
    }
  }
  const double min_cells = params.anchor_min_area / (m.res * m.res);

  // The anchor the next link would start from, and the track length walked
  // since it (xy, the same measure as max_length).
  size_t anchor_kf = 0;
  CellId anchor = kNoCell;
  double walked = 0.0;
  for (size_t k = 0; k < track.size(); ++k) {
    if (k > 0) {
      walked += std::hypot(track[k].x - track[k - 1].x, track[k].y - track[k - 1].y);
    }
    const CellId c = anchorCell(m, track[k], params.anchor_z_window);
    if (c == kNoCell || static_cast<double>(comp_cells[m.cells[c].component]) < min_cells) {
      continue;
    }
    if (anchor == kNoCell || m.cells[c].component == m.cells[anchor].component) {
      anchor = c;
      anchor_kf = k;
      walked = 0.0;
      continue;
    }
    if (walked > params.max_length) {
      // Lost the ground for too long to trust what happened in between.
      if (std::fabs(m.zTop(c) - m.zTop(anchor)) >= params.min_rise) {
        char buf[200];
        std::snprintf(
          buf, sizeof buf,
          "keyframes %zu..%zu climb %.2f m (z %.2f -> %.2f) over %.0f m of track, more than "
          "stair_max_length %.0f m: no link",
          anchor_kf, k, m.zTop(c) - m.zTop(anchor), m.zTop(anchor), m.zTop(c), walked,
          params.max_length);
        out->notes.push_back(buf);
      }
      anchor = c;
      anchor_kf = k;
      walked = 0.0;
      continue;
    }
    if (std::fabs(m.zTop(c) - m.zTop(anchor)) < params.min_rise) {
      // Not a stair (see the header): walked on from here.
      anchor = c;
      anchor_kf = k;
      walked = 0.0;
      continue;
    }
    std::string why;
    size_t laid = 0;
    if (params.straight) {
      laid = straightLinks(m, track, anchor_kf, k, anchor, c, params, *out, why);
      if (laid == 0) {
        char buf[240];
        std::snprintf(
          buf, sizeof buf,
          "keyframes %zu..%zu (z %.2f -> %.2f): %s; the link follows the walked track", anchor_kf,
          k, m.zTop(anchor), m.zTop(c), why.c_str());
        out->notes.push_back(buf);
      }
    }
    if (laid == 0) {
      // The walk itself.
      StairLink link;
      link.a = anchor;
      link.b = c;
      link.kf_a = anchor_kf;
      link.kf_b = k;
      double ax, ay, bx, by;
      m.cellXY(anchor, ax, ay);
      m.cellXY(c, bx, by);
      link.pts.push_back({ax, ay, m.zTop(anchor)});
      for (size_t i = anchor_kf + 1; i < k; ++i) {
        link.pts.push_back(track[i]);
      }
      link.pts.push_back({bx, by, m.zTop(c)});
      link.pts = cutLoops(link.pts, params.loop_radius, params.loop_dz);
      smoothPolyline(link.pts, params.smooth_passes);
      addLink(m, std::move(link), *out);
    }
    anchor = c;
    anchor_kf = k;
    walked = 0.0;
  }
  for (uint32_t i = 0; i < ncomp; ++i) {
    out->group[i] = findRoot(out->group, i);
  }
  // Large components the links leave apart although they lie at different
  // heights: the floors of this map the planner cannot get between.
  std::vector<double> zsum(ncomp, 0.0);
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.cells[c].component != kNoComponent) {
      zsum[m.cells[c].component] += m.zTop(c);
    }
  }
  std::vector<uint32_t> large;
  for (uint32_t i = 0; i < ncomp; ++i) {
    if (static_cast<double>(comp_cells[i]) >= min_cells) {
      large.push_back(i);
    }
  }
  for (size_t i = 0; i < large.size(); ++i) {
    for (size_t j = i + 1; j < large.size(); ++j) {
      const uint32_t a = large[i], b = large[j];
      const double za = zsum[a] / comp_cells[a], zb = zsum[b] / comp_cells[b];
      if (std::fabs(za - zb) >= params.min_rise && out->group[a] != out->group[b]) {
        char buf[200];
        std::snprintf(
          buf, sizeof buf,
          "component %u (%.0f m2, z ~%.2f) and component %u (%.0f m2, z ~%.2f) are not joined by "
          "any stair link",
          a, comp_cells[a] * m.res * m.res, za, b, comp_cells[b] * m.res * m.res, zb);
        out->notes.push_back(buf);
      }
    }
  }
  return out;
}

}  // namespace syncai_global_planner::octomap_planner_3d
