#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>

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

// How far the stairwell's cross-section reaches from (x, y) along (nx, ny),
// at the tread height z (see the header): the distance to the last column
// that still holds a usable surface within dz of z. Columns with no surface
// at all are gaps -- treads the lidar never saw, common on a flight -- and
// up to two in a row are crossed when a usable column follows. Returns
// max_reach (or more) when the side never ends.
double sideReach(
  const SurfaceMap & m, double x, double y, double z, double nx, double ny, double dz,
  double max_reach, bool * wall = nullptr)
{
  constexpr int kMaxGap = 2;
  double reach = 0.0;
  int gap = 0;
  if (wall) {
    *wall = false;
  }
  for (double s = m.res; s <= max_reach + 1e-9; s += m.res) {
    const int ix = m.ixOf(x + s * nx), iy = m.iyOf(y + s * ny);
    if (!m.inGrid(ix, iy)) {
      break;
    }
    const uint32_t c = m.col(ix, iy);
    if (m.col_begin[c] == m.col_begin[c + 1]) {
      if (++gap > kMaxGap) {
        break;
      }
      continue;
    }
    if (m.surfaceNear(c, z, dz) == kNoCell) {
      if (wall) {
        *wall = true;  // something observed here, and no floor at this height
      }
      break;  // a wall, a rail or the drop to the next flight
    }
    gap = 0;
    reach = s;
  }
  return gap > kMaxGap || reach + m.res > max_reach ? std::max(reach, max_reach) : reach;
}

// Whether column (ix, iy) blocks the body on a landing whose floor is at z:
// a wall, a newel or something standing on the landing -- an occupied voxel
// in the body's band, above a step and below body_clearance. Read off the
// tree, because the surface map cannot tell a wall from a landing nobody
// saw the floor of: it holds top faces only, a wall that runs into the slab
// above has none near the landing, and a landing column whose floor was
// never seen (dp's landings are seen edge-on from the flights) holds only
// what is overhead or a stray face under the floor -- dp's first landing
// has surfaces 0.3 m below and 0.1 m above its floor side by side. A
// handrail (~0.9 m) is above the band; the body passes under it. The drop
// to a flight is not tested: the corners stay half a body out from the
// later stair edge, and the leg between them runs square to the flights.
// Without a tree (the gtest's synthetic maps) a column that holds no
// surface near z but one within a flight below or the band above stands in.
bool landingObstacle(
  const SurfaceMap & m, const octomap::OcTree * tree, int ix, int iy, double z, double dz)
{
  const uint32_t c = m.col(ix, iy);
  const double band_lo = z + m.params.max_step_height + 0.05;
  const double band_hi = z + m.params.body_clearance;
  if (tree) {
    for (double h = band_lo; h <= band_hi + 1e-9; h += m.res) {
      const octomap::OcTreeNode * n = tree->search(m.cx(ix), m.cy(iy), h);
      if (n && tree->isNodeOccupied(n)) {
        return true;
      }
    }
    return false;
  }
  bool near = false, other = false;
  for (uint32_t q = m.col_begin[c]; q < m.col_begin[c + 1]; ++q) {
    const double t = m.zTop(q);
    near = near || std::fabs(t - z) <= dz;
    other = other || (t > z - 0.8 && t <= band_hi);
  }
  return !near && other;
}

// xy distance from (x, y) to the nearest blocked column (obstacle(ix, iy))
// within cap, measured to the column's near edge, half a cell inside its
// centre. cap when nothing is that close.
template <typename Obstacle>
double landingClearance(const SurfaceMap & m, Obstacle && obstacle, double x, double y, double cap)
{
  const int r = static_cast<int>(std::ceil(cap / m.res)) + 1;
  const int ix0 = m.ixOf(x), iy0 = m.iyOf(y);
  double best = cap;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int ix = ix0 + dx, iy = iy0 + dy;
      if (!m.inGrid(ix, iy)) {
        continue;
      }
      const double d = std::hypot(m.cx(ix) - x, m.cy(iy) - y) - m.res / 2.0;
      if (d < best && obstacle(ix, iy)) {
        best = std::max(d, 0.0);
      }
    }
  }
  return best;
}

// Moves the interior points of a walked link to the middle of the
// stairwell's cross-section (see the header) and records what it did.
void centreWalk(const SurfaceMap & m, const StairLinkParams & p, StairLink & link)
{
  // A cross-section narrower than the robot is a misread (a side ending on
  // the next tread at once), not a stair the robot walked up.
  constexpr double kMinWidth = 0.5;
  constexpr size_t kMedianHalf = 3;
  std::vector<Waypoint> & pts = link.pts;
  const size_t n = pts.size();
  if (n < 3) {
    return;
  }
  // Signed shift along the left normal per point, 0 where not confined.
  std::vector<double> shift(n, 0.0), nxs(n, 0.0), nys(n, 0.0);
  std::vector<bool> confined(n, false);
  double min_width = 0.0;
  for (size_t i = 1; i + 1 < n; ++i) {
    double tx = pts[i + 1].x - pts[i - 1].x, ty = pts[i + 1].y - pts[i - 1].y;
    const double t = std::hypot(tx, ty);
    if (t < 1e-6) {
      continue;
    }
    tx /= t;
    ty /= t;
    nxs[i] = -ty;
    nys[i] = tx;
    // The tread under the point: the walk's ground estimate (pose z -
    // lidar_height) pitches with the body on a flight, the surface does not.
    double z = pts[i].z;
    const int ix = m.ixOf(pts[i].x), iy = m.iyOf(pts[i].y);
    if (m.inGrid(ix, iy)) {
      const CellId c = m.surfaceNear(m.col(ix, iy), z, p.anchor_z_window);
      if (c != kNoCell) {
        z = m.zTop(c);
      }
    }
    const double left =
      sideReach(m, pts[i].x, pts[i].y, z, nxs[i], nys[i], p.centre_dz, p.centre_max_half_width);
    const double right =
      sideReach(m, pts[i].x, pts[i].y, z, -nxs[i], -nys[i], p.centre_dz, p.centre_max_half_width);
    const double width = left + right + m.res;
    if (left >= p.centre_max_half_width || right >= p.centre_max_half_width || width < kMinWidth) {
      continue;
    }
    confined[i] = true;
    shift[i] = (left - right) / 2.0;
    if (min_width == 0.0 || width < min_width) {
      min_width = width;
    }
  }
  // One cross-section is noisy -- the walk crosses the risers at an angle,
  // so a side may end on the next tread up, and the ground estimate the
  // tread is looked up from pitches with the body -- so each confined
  // point takes the median of the confined points within kMedianHalf of it,
  // then the shifts are smoothed along the link (an unconfined point counts
  // as 0, so a shift fades out towards the floors instead of stopping dead).
  {
    std::vector<double> med(n, 0.0), win;
    for (size_t i = 1; i + 1 < n; ++i) {
      if (!confined[i]) {
        continue;
      }
      win.clear();
      const size_t lo = i > kMedianHalf ? i - kMedianHalf : 0;
      for (size_t j = lo; j <= std::min(n - 1, i + kMedianHalf); ++j) {
        if (confined[j]) {
          win.push_back(shift[j]);
        }
      }
      std::nth_element(win.begin(), win.begin() + win.size() / 2, win.end());
      med[i] = std::clamp(win[win.size() / 2], -p.centre_max_shift, p.centre_max_shift);
    }
    shift.swap(med);
  }
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<double> q(shift);
    for (size_t i = 1; i + 1 < n; ++i) {
      q[i] = (shift[i - 1] + shift[i] + shift[i + 1]) / 3.0;
    }
    shift.swap(q);
  }
  double sum = 0.0, max_abs = 0.0;
  size_t moved = 0;
  for (size_t i = 1; i + 1 < n; ++i) {
    if (!confined[i] && std::fabs(shift[i]) < 1e-3) {
      continue;
    }
    pts[i].x += shift[i] * nxs[i];
    pts[i].y += shift[i] * nys[i];
    ++moved;
    sum += std::fabs(shift[i]);
    max_abs = std::max(max_abs, std::fabs(shift[i]));
  }
  link.centred = moved;
  link.centre_mean_shift = moved ? sum / moved : 0.0;
  link.centre_max_shift = max_abs;
  link.centre_min_width = min_width;
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

// ---- Flights and landing corners (2026-10-21; see the header) ----

// The traversable cell of component comp nearest (x, y) within radius, at
// about z. kNoCell when there is none.
CellId nearestTraversable(
  const SurfaceMap & m, double x, double y, double z, uint32_t comp, double radius)
{
  CellId best = kNoCell;
  double bd = 1e300;
  for (int iy = std::max(0, m.iyOf(y - radius)); iy <= std::min(m.ny - 1, m.iyOf(y + radius));
       ++iy) {
    for (int ix = std::max(0, m.ixOf(x - radius)); ix <= std::min(m.nx - 1, m.ixOf(x + radius));
         ++ix) {
      const double d = std::hypot(m.cx(ix) - x, m.cy(iy) - y);
      if (d > radius || d >= bd) {
        continue;
      }
      const uint32_t c = m.col(ix, iy);
      for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
        if (m.traversable(i) && m.cells[i].component == comp && std::fabs(m.zTop(i) - z) <= 0.5) {
          best = i;
          bd = d;
        }
      }
    }
  }
  return best;
}

// Follows the surface from (x, y, z) along (dx, dy), column by column within
// one step of the last height (up to two unseen columns in a row crossed),
// until it has run level for kLevelRun at about z_target: the landing at a
// flight's foot or head. `reached` is where that level run began -- the
// landing's edge. A tread is 0.25-0.3 m deep, so a 0.45 m run is a landing;
// the height guard keeps a deep tread mid-flight from passing. False when
// the surface is lost or max_len runs out first.
bool marchToHeight(
  const SurfaceMap & m, double x, double y, double z, double dx, double dy, double z_target,
  double max_len, Waypoint & reached)
{
  constexpr double kLevelRun = 0.45;
  const double step_tol = (m.step_cells + 1) * m.res + 1e-6;
  uint32_t last_col = ~uint32_t(0);
  int gap = 0;
  double level = 0.0;
  Waypoint level_start = {x, y, z};
  for (double s = 0.0; s <= max_len + 1e-9; s += m.res) {
    const double px = x + s * dx, py = y + s * dy;
    const int ix = m.ixOf(px), iy = m.iyOf(py);
    if (!m.inGrid(ix, iy)) {
      return false;
    }
    const uint32_t c = m.col(ix, iy);
    if (c == last_col) {
      continue;
    }
    last_col = c;
    if (m.col_begin[c] == m.col_begin[c + 1]) {
      if (++gap > 2) {
        return false;
      }
      continue;
    }
    const CellId i = m.surfaceNear(c, z, step_tol);
    if (i == kNoCell) {
      return false;
    }
    gap = 0;
    const double zs = m.zTop(i);
    if (std::fabs(zs - z) > m.res / 2) {
      level = 0.0;
      level_start = {px, py, zs};
    } else {
      level += m.res;
    }
    z = zs;
    if (level >= kLevelRun - 1e-9 && std::fabs(z - z_target) <= 0.25) {
      reached = level_start;
      return true;
    }
  }
  return false;
}

double median(std::vector<double> v)
{
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

struct Landing
{
  size_t first, last;  // walk points
  double z, cx, cy;    // median height, centroid
};

struct Flight
{
  size_t first, last;   // walk points (the landings' edge points included)
  double ux, uy;        // uphill (fitted)
  double dx, dy;        // travel direction along the walk (+-u)
  Waypoint foot, head;  // in travel order: on the landing left, on the landing reached
  double width = 0.0;   // median cross-section
};

// The walk's level runs and rises, as landings: the floors first and last.
std::vector<Landing> landings(
  const std::vector<Waypoint> & w, double min_len, double slope_deg, double min_rise)
{
  const size_t n = w.size();
  const double tan_lvl = std::tan(slope_deg * M_PI / 180.0);
  // level[i]: segment i -> i + 1 is level
  std::vector<bool> level(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    const double ds = std::hypot(w[i + 1].x - w[i].x, w[i + 1].y - w[i].y);
    level[i] = std::fabs(w[i + 1].z - w[i].z) <= tan_lvl * std::max(ds, 0.05);
  }
  std::vector<Landing> out;
  size_t i = 0;
  while (i < n) {
    size_t j = i;
    double len = 0.0;
    while (j + 1 < n && level[j]) {
      len += std::hypot(w[j + 1].x - w[j].x, w[j + 1].y - w[j].y);
      ++j;
    }
    const bool floor = i == 0 || j == n - 1;
    if (floor || len >= min_len) {
      std::vector<double> zs;
      double cx = 0, cy = 0;
      for (size_t k = i; k <= j; ++k) {
        zs.push_back(w[k].z);
        cx += w[k].x;
        cy += w[k].y;
      }
      const double cnt = static_cast<double>(j - i + 1);
      out.push_back({i, j, median(zs), cx / cnt, cy / cnt});
    }
    i = j + 1;
  }
  // Two landings less than a flight apart are one landing (a wide tread, a
  // pause on the stair).
  for (size_t k = 1; k < out.size();) {
    if (std::fabs(out[k].z - out[k - 1].z) < min_rise) {
      Landing & a = out[k - 1];
      const Landing & b = out[k];
      std::vector<double> zs;
      double cx = 0, cy = 0;
      for (size_t q = a.first; q <= b.last; ++q) {
        zs.push_back(w[q].z);
        cx += w[q].x;
        cy += w[q].y;
      }
      const double cnt = static_cast<double>(b.last - a.first + 1);
      a = {a.first, b.last, median(zs), cx / cnt, cy / cnt};
      out.erase(out.begin() + static_cast<std::ptrdiff_t>(k));
    } else {
      ++k;
    }
  }
  return out;
}

// Lays the walked stretch (walk: a's centre, the keyframes, b's centre,
// loops cut) as flights and landing corners. Returns false, with why, when
// the geometry cannot be read; the caller keeps the walked link then.
bool flightLinks(
  const SurfaceMap & m, const octomap::OcTree * tree, const std::vector<Waypoint> & walk, CellId a,
  CellId b, size_t ka, size_t kb, const StairLinkParams & p, double min_width, StairLinks & out,
  std::string & why)
{
  if (walk.size() < 4) {
    why = "too few points to read a profile from";
    return false;
  }
  const std::vector<Landing> L =
    landings(walk, p.flights_landing_min_len, p.flights_landing_slope_deg, p.min_rise);
  if (L.size() < 2) {
    why = "no rise between two level runs of the walk";
    return false;
  }
  const double half_w = p.centre_max_half_width;
  // The surface under a walk point, else its own ground estimate.
  const auto surface_z = [&](const Waypoint & w) {
    const int ix = m.ixOf(w.x), iy = m.iyOf(w.y);
    if (m.inGrid(ix, iy)) {
      const CellId c = m.surfaceNear(m.col(ix, iy), w.z, p.anchor_z_window);
      if (c != kNoCell) {
        return m.zTop(c);
      }
    }
    return w.z;
  };
  // The line through a set of points (total least squares), robust to a few
  // strays: fitted, the points more than 0.15 m off dropped, fitted again.
  // Returns the RMS residual, or a negative number with fewer than three
  // points left. (ux, uy) is oriented along the direction given.
  const auto fitLine =
    [](std::vector<Waypoint> pts, double & ux, double & uy, double & cx, double & cy) {
      double rms = -1.0;
      for (int pass = 0; pass < 2; ++pass) {
        if (pts.size() < 3) {
          return -1.0;
        }
        double mx = 0, my = 0;
        for (const Waypoint & q : pts) {
          mx += q.x;
          my += q.y;
        }
        mx /= pts.size();
        my /= pts.size();
        double sxx = 0, syy = 0, sxy = 0;
        for (const Waypoint & q : pts) {
          sxx += (q.x - mx) * (q.x - mx);
          syy += (q.y - my) * (q.y - my);
          sxy += (q.x - mx) * (q.y - my);
        }
        const double theta = 0.5 * std::atan2(2 * sxy, sxx - syy);
        double vx = std::cos(theta), vy = std::sin(theta);
        if (vx * ux + vy * uy < 0) {
          vx = -vx;
          vy = -vy;
        }
        std::vector<Waypoint> kept;
        double sq = 0;
        for (const Waypoint & q : pts) {
          const double off = (q.x - mx) * -vy + (q.y - my) * vx;
          sq += off * off;
          if (std::fabs(off) <= 0.15) {
            kept.push_back(q);
          }
        }
        rms = std::sqrt(sq / pts.size());
        ux = vx;
        uy = vy;
        cx = mx;
        cy = my;
        if (kept.size() == pts.size()) {
          break;
        }
        pts.swap(kept);
      }
      return rms;
    };

  // Pass 1: each flight's own direction. The walk's over the flight (held
  // straight by the treads, unlike a walk across a landing), refined to the
  // line through the points where the cross-sections square to it end on
  // the straighter side: one side of a flight is the well's wall and its
  // hits line up to a few centimetres, the other is the open well, the
  // rail or the parallel flight at a near height, and its hits scatter
  // (dp, flight 2: the wall side within 0.1 m of a line over 2 m, the well
  // side spread 0.9 m). The plane fit a straight flight uses was tried
  // first: on that flight it came out 17 deg off the walk and the wall,
  // pulled by the parallel flight's surfaces inside its radius.
  std::vector<Flight> F;
  std::vector<double> dir_weight;
  for (size_t j = 0; j + 1 < L.size(); ++j) {
    Flight f;
    f.first = L[j].last;
    f.last = L[j + 1].first;
    if (f.last <= f.first) {
      why = "landings " + std::to_string(j) + " and " + std::to_string(j + 1) + " touch";
      return false;
    }
    double dx = walk[f.last].x - walk[f.first].x, dy = walk[f.last].y - walk[f.first].y;
    const double dlen = std::hypot(dx, dy);
    if (dlen < 0.5) {
      why = "flight " + std::to_string(j + 1) + ": shorter than 0.5 m of walk";
      return false;
    }
    dx /= dlen;
    dy /= dlen;
    std::vector<Waypoint> left_hits, right_hits;
    {
      const double nx = -dy, ny = dx;
      for (size_t i = f.first; i <= f.last; ++i) {
        const double z = surface_z(walk[i]);
        const double l = sideReach(m, walk[i].x, walk[i].y, z, nx, ny, p.centre_dz, half_w);
        const double r = sideReach(m, walk[i].x, walk[i].y, z, -nx, -ny, p.centre_dz, half_w);
        if (l < half_w) {
          left_hits.push_back({walk[i].x + l * nx, walk[i].y + l * ny, z});
        }
        if (r < half_w) {
          right_hits.push_back({walk[i].x - r * nx, walk[i].y - r * ny, z});
        }
      }
    }
    double lux = dx, luy = dy, lcx = 0, lcy = 0, rux = dx, ruy = dy, rcx = 0, rcy = 0;
    const double lrms = fitLine(left_hits, lux, luy, lcx, lcy);
    const double rrms = fitLine(right_hits, rux, ruy, rcx, rcy);
    const double cos25 = std::cos(25.0 * M_PI / 180.0);
    double weight = 1.0;  // the walk alone
    if (lrms >= 0 && (rrms < 0 || lrms <= rrms) && lrms < 0.12 && lux * dx + luy * dy >= cos25) {
      dx = lux;
      dy = luy;
      weight = 0.02 / std::max(lrms, 0.02) * left_hits.size();
    } else if (rrms >= 0 && rrms < 0.12 && rux * dx + ruy * dy >= cos25) {
      dx = rux;
      dy = ruy;
      weight = 0.02 / std::max(rrms, 0.02) * right_hits.size();
    }
    f.dx = dx;
    f.dy = dy;
    F.push_back(f);
    dir_weight.push_back(weight);
  }
  // The flights of one stairwell are parallel or square to each other, so
  // their directions are one orientation: the weighted circular mean of
  // the estimates folded to a quarter turn. Each flight then takes the
  // quarter-turn multiple nearest its own estimate -- on dp the four
  // estimates ran 5-13 deg off the map's y axis and settle on one 9 deg.
  {
    double sx = 0, sy = 0;
    for (size_t j = 0; j < F.size(); ++j) {
      const double a4 = 4.0 * std::atan2(F[j].dy, F[j].dx);
      sx += dir_weight[j] * std::cos(a4);
      sy += dir_weight[j] * std::sin(a4);
    }
    const double grid = std::atan2(sy, sx) / 4.0;
    for (Flight & f : F) {
      const double own = std::atan2(f.dy, f.dx);
      const double a = grid + std::round((own - grid) / (M_PI / 2)) * (M_PI / 2);
      f.dx = std::cos(a);
      f.dy = std::sin(a);
    }
  }

  // Pass 2: each flight's centre axis, width, foot and head.
  for (size_t j = 0; j < F.size(); ++j) {
    Flight & f = F[j];
    const double dx = f.dx, dy = f.dy;
    const bool rising = L[j + 1].z > L[j].z;
    f.ux = rising ? dx : -dx;
    f.uy = rising ? dy : -dy;
    // The axis: the walk's first point moved square to the direction by the
    // median of the cross-sections' centring shifts, measured square to it.
    const double nx = -dy, ny = dx;
    const Waypoint & o = walk[f.first];
    std::vector<double> perps, widths;
    for (size_t i = f.first; i <= f.last; ++i) {
      const double z = surface_z(walk[i]);
      const double l = sideReach(m, walk[i].x, walk[i].y, z, nx, ny, p.centre_dz, half_w);
      const double r = sideReach(m, walk[i].x, walk[i].y, z, -nx, -ny, p.centre_dz, half_w);
      if (l >= half_w || r >= half_w || l + r + m.res < 0.5) {
        continue;
      }
      perps.push_back((walk[i].x - o.x) * nx + (walk[i].y - o.y) * ny + (l - r) / 2.0);
      widths.push_back(l + r + m.res);
    }
    if (perps.size() < 2) {
      why = "flight " + std::to_string(j + 1) + ": fewer than two cross-sections of the well";
      return false;
    }
    const double cx = o.x + median(perps) * nx, cy = o.y + median(perps) * ny;
    f.width = median(widths);
    if (f.width < min_width) {
      char wbuf[120];
      std::snprintf(
        wbuf, sizeof wbuf, "flight %zu: %.2f m wide, narrower than %.2f m", j + 1, f.width,
        min_width);
      why = wbuf;
      return false;
    }
    // The flight's extent along the axis, from its walk points.
    double s_min = 1e300, s_max = -1e300;
    for (size_t i = f.first; i <= f.last; ++i) {
      const double along = (walk[i].x - cx) * dx + (walk[i].y - cy) * dy;
      s_min = std::min(s_min, along);
      s_max = std::max(s_max, along);
    }
    const double s_mid = (s_min + s_max) / 2.0;
    // The foot and head: where a march along the axis from the flight's
    // middle, down and up, finds the surface level for a landing's worth
    // at about the landings' heights -- an inner landing's median, a
    // floor's local surface under the walk's edge point (the lower floor's
    // median is taken over a corridor's length).
    const double z_foot = j == 0 ? surface_z(walk[f.first]) : L[j].z;
    const double z_head = j + 2 == L.size() ? surface_z(walk[f.last]) : L[j + 1].z;
    const double mx = cx + s_mid * dx, my = cy + s_mid * dy;
    double z_mid = (z_foot + z_head) / 2.0;
    {
      const int ix = m.ixOf(mx), iy = m.iyOf(my);
      if (m.inGrid(ix, iy)) {
        const CellId c = m.surfaceNear(m.col(ix, iy), z_mid, std::fabs(z_head - z_foot) / 2.0);
        if (c != kNoCell) {
          z_mid = m.zTop(c);
        }
      }
    }
    const double reach = (s_max - s_min) / 2.0 + 2.0;
    if (!marchToHeight(m, mx, my, z_mid, -dx, -dy, z_foot, reach, f.foot)) {
      f.foot = {cx + s_min * dx, cy + s_min * dy, z_foot};
      out.notes.push_back(
        "flight " + std::to_string(j + 1) +
        ": its foot was not found on the map, the walk's first point on the axis stands in");
    }
    if (!marchToHeight(m, mx, my, z_mid, dx, dy, z_head, reach, f.head)) {
      f.head = {cx + s_max * dx, cy + s_max * dy, z_head};
      out.notes.push_back(
        "flight " + std::to_string(j + 1) +
        ": its head was not found on the map, the walk's last point on the axis stands in");
    }
  }

  // The polyline: lead-in, flights, corners on the landings, lead-out.
  StairLink link;
  link.kf_a = ka;
  link.kf_b = kb;
  link.flights = true;
  link.n_flights = F.size();
  std::vector<Waypoint> & pts = link.pts;
  char buf[400];
  std::string shapes;
  const Waypoint lead_in = {
    F.front().foot.x - p.flights_lead * F.front().dx,
    F.front().foot.y - p.flights_lead * F.front().dy, L.front().z};
  const Waypoint lead_out = {
    F.back().head.x + p.flights_lead * F.back().dx, F.back().head.y + p.flights_lead * F.back().dy,
    L.back().z};
  // The link's ends are re-anchored to the floor cells nearest the lead
  // points, so the A* arrives in line with the first flight; failing that,
  // to the first floor cell straight on along the flight's axis. The floor
  // just past a stair head is often not in the map -- dp's 2F, for 1.3 m:
  // the lidar is below it on the flight and cannot see it from on it, so
  // the voxel above it was never observed free -- and with no cell within a
  // metre the link used to follow the walk to the operator's own anchor,
  // which on dp swung 35 deg off the axis and 56 deg back, just where the
  // robot stops to switch the policy back (2026-10-24).
  const uint32_t comp_a = m.cells[a].component, comp_b = m.cells[b].component;
  const auto straightOn = [&](const Waypoint & from, double dx, double dy, uint32_t comp) {
    constexpr double kReach = 2.0;
    for (double s = 0.0; s <= kReach + 1e-9; s += m.res) {
      const CellId c =
        nearestTraversable(m, from.x + s * dx, from.y + s * dy, from.z, comp, m.res * 1.5);
      if (c != kNoCell) {
        return c;
      }
    }
    return kNoCell;
  };
  CellId a2 = nearestTraversable(m, lead_in.x, lead_in.y, lead_in.z, comp_a, 1.0);
  if (a2 == kNoCell) {
    a2 = straightOn(lead_in, -F.front().dx, -F.front().dy, comp_a);
  }
  CellId b2 = nearestTraversable(m, lead_out.x, lead_out.y, lead_out.z, comp_b, 1.0);
  if (b2 == kNoCell) {
    b2 = straightOn(lead_out, F.back().dx, F.back().dy, comp_b);
  }
  link.a = a2 != kNoCell ? a2 : a;
  link.b = b2 != kNoCell ? b2 : b;
  double ex, ey;
  m.cellXY(link.a, ex, ey);
  pts.push_back({ex, ey, m.zTop(link.a)});
  if (a2 == kNoCell) {
    // No floor cell near the lead-in (the approach to dp's stair is a
    // corridor the rules reject): keep the walk's floor points up to it,
    // smoothed as a walked link is -- curves are fine on the floor.
    std::vector<Waypoint> prefix;
    for (size_t q = 0; q <= L.front().last; ++q) {
      const double along = (walk[q].x - F.front().foot.x) * F.front().dx +
                           (walk[q].y - F.front().foot.y) * F.front().dy;
      if (along < -p.flights_lead - 0.2) {
        prefix.push_back(walk[q]);
      }
    }
    prefix.push_back(lead_in);
    smoothPolyline(prefix, p.smooth_passes);
    pts.insert(pts.end(), prefix.begin() + 1, prefix.end());
  } else if (std::hypot(ex - lead_in.x, ey - lead_in.y) > 0.05) {
    pts.push_back(lead_in);
  }
  for (size_t j = 0; j < F.size(); ++j) {
    const Flight & f = F[j];
    pts.push_back(f.foot);
    pts.push_back(f.head);
    if (j + 1 == F.size()) {
      break;
    }
    const Flight & g = F[j + 1];
    const double cosang = f.dx * g.dx + f.dy * g.dy;
    const double z = L[j + 1].z;
    if (cosang > std::cos(30.0 * M_PI / 180.0)) {
      shapes += shapes.empty() ? "straight on" : ", straight on";
      continue;  // head -> next foot, no turn
    }
    if (std::fabs(cosang) < std::cos(60.0 * M_PI / 180.0)) {
      // An L: the corner is where the two axes meet.
      const double den = f.dx * (-g.dy) - f.dy * (-g.dx);
      const double rx = g.foot.x - f.head.x, ry = g.foot.y - f.head.y;
      const double t = (rx * (-g.dy) - ry * (-g.dx)) / den;
      const double u = (f.dx * ry - f.dy * rx) / den;
      if (t < 0.0 || u > 0.0) {
        why = "landing " + std::to_string(j + 1) +
              ": the flights' axes meet behind the head or "
              "past the next foot";
        return false;
      }
      const Waypoint c = {f.head.x + t * f.dx, f.head.y + t * f.dy, z};
      link.turns.push_back(pts.size());
      pts.push_back(c);
      std::snprintf(
        buf, sizeof buf, "%sL (%.0f deg)", shapes.empty() ? "" : ", ",
        std::acos(std::clamp(cosang, -1.0, 1.0)) * 180.0 / M_PI);
      shapes += buf;
      continue;
    }
    if (cosang < -std::cos(30.0 * M_PI / 180.0)) {
      // A U: out onto the landing, across, and in. The robot turns in place
      // at both corners, sweeping a disc of flights_turn_radius, and walks
      // the leg between them sideways to the flights, half a body wide
      // (robot_radius). Both corners lie the same distance out, so the leg
      // is square to the flights: each flight's own head / foot put them
      // out of line on dp's second landing, whose flights start 0.36 m
      // apart, and the slanted leg passed 0.3 m from the newel. How far out
      // is read off the map: the distance that leaves the most room round
      // the two turns and along the leg (landingClearance), the nearest
      // such when there is room to spare. Until 2026-10-24 it came from
      // the walk (where the operator turned, 0.5-1 m) capped at half the
      // landing's depth measured along one ray from the head -- which on
      // that landing went through a gap in the scanned far wall and read
      // 2.3 m for a 1.05 m landing, and on all three left the turn closer
      // to a wall than the body's reach.
      const double R = p.flights_turn_radius, hw = m.params.robot_radius;
      // The landing's floor as the map holds it, not the walk's ground (pose
      // z - lidar_height): on dp's first landing that is 0.2 m under the
      // floor, the body still pitched from the flight, and the map holds a
      // surface 0.2 m below it too, so neither the walk's z nor the surface
      // nearest it under the walk reads the floor; with either, most of the
      // landing came out a wall. The height most columns of the landing's
      // near metre hold a surface at, within a riser and a half of the walk.
      double zl = z;
      {
        std::unordered_map<int, int> votes;
        const double along_g0 = (g.foot.x - f.head.x) * f.dx + (g.foot.y - f.head.y) * f.dy;
        const double sa = std::max(0.0, along_g0);
        const double ex = g.foot.x - f.head.x - along_g0 * f.dx;  // f's axis to g's, across
        const double ey = g.foot.y - f.head.y - along_g0 * f.dy;
        for (double s = sa + 0.2; s <= sa + 1.0 + 1e-9; s += m.res) {
          for (double a = 0.0; a <= 1.0 + 1e-9; a += 0.05) {
            const int ix = m.ixOf(f.head.x + s * f.dx + a * ex);
            const int iy = m.iyOf(f.head.y + s * f.dy + a * ey);
            if (!m.inGrid(ix, iy)) {
              continue;
            }
            const uint32_t c = m.col(ix, iy);
            for (uint32_t q = m.col_begin[c]; q < m.col_begin[c + 1]; ++q) {
              if (std::fabs(m.zTop(q) - z) <= 0.3) {
                ++votes[m.cells[q].kz];
              }
            }
          }
        }
        int best_n = 0;
        for (const auto & [kz, n] : votes) {
          if (n > best_n) {
            best_n = n;
            zl = m.z0 + (kz + 1) * m.res;
          }
        }
      }
      const double cap = R + 0.3;
      std::unordered_map<uint32_t, bool> blocked;
      const auto obstacle = [&](int ix, int iy) {
        const auto [it, fresh] = blocked.try_emplace(m.col(ix, iy), false);
        if (fresh) {
          it->second = landingObstacle(m, tree, ix, iy, zl, p.centre_dz);
        }
        return it->second;
      };
      const double ux = f.dx, uy = f.dy;  // out, onto the landing
      const double along_g = (g.foot.x - f.head.x) * ux + (g.foot.y - f.head.y) * uy;
      const double s0 = std::max(0.0, along_g);  // the later of the two stair edges
      const auto corners = [&](double t, Waypoint & c1, Waypoint & c2) {
        const double s = s0 + t;
        c1 = {f.head.x + s * ux, f.head.y + s * uy, z};
        c2 = {g.foot.x + (s - along_g) * ux, g.foot.y + (s - along_g) * uy, z};
      };
      // Room left at t: the two turns' discs and the leg's half-width,
      // whichever is tightest. Negative = the body reaches something.
      const auto room = [&](double t) {
        Waypoint c1, c2;
        corners(t, c1, c2);
        double r = std::min(
          landingClearance(m, obstacle, c1.x, c1.y, cap) - R,
          landingClearance(m, obstacle, c2.x, c2.y, cap) - R);
        const double len = std::hypot(c2.x - c1.x, c2.y - c1.y);
        const int n = std::max(1, static_cast<int>(std::ceil(len / (m.res / 2.0))));
        for (int k = 1; k < n; ++k) {
          const double a = static_cast<double>(k) / n;
          const double x = c1.x + a * (c2.x - c1.x), y = c1.y + a * (c2.y - c1.y);
          r = std::min(r, landingClearance(m, obstacle, x, y, cap) - hw);
        }
        return r;
      };
      // At least half a body out from the later edge (the hind legs are
      // 0.3 m behind the centre), at most 1.5 m.
      std::vector<std::pair<double, double>> tried;  // t, room
      double best = -std::numeric_limits<double>::infinity();
      for (double t = 0.5; t <= 1.5 + 1e-9; t += m.res / 2.0) {
        tried.push_back({t, room(t)});
        best = std::max(best, tried.back().second);
      }
      double t_pick = tried.front().first, room_pick = tried.front().second;
      for (const auto & [t, r] : tried) {
        if (r >= best - 0.01) {
          t_pick = t;
          room_pick = r;
          break;
        }
      }
      Waypoint c1, c2;
      corners(t_pick, c1, c2);
      link.turns.push_back(pts.size());
      pts.push_back(c1);
      link.turns.push_back(pts.size());
      pts.push_back(c2);
      std::snprintf(
        buf, sizeof buf, "%sU (turns %.2f m out, %.2f m %s)", shapes.empty() ? "" : ", ",
        s0 + t_pick, std::fabs(room_pick), room_pick >= 0.0 ? "to spare" : "short");
      shapes += buf;
      if (room_pick < 0.0) {
        std::snprintf(
          buf, sizeof buf,
          "landing %zu: no room to turn in place -- the body reaches %.2f m into a wall, a "
          "newel or something on the landing wherever the corners go",
          j + 1, -room_pick);
        out.notes.push_back(buf);
      }
      continue;
    }
    std::snprintf(
      buf, sizeof buf, "landing %zu: the flights are %.0f deg apart, neither an L nor a U", j + 1,
      std::acos(std::clamp(cosang, -1.0, 1.0)) * 180.0 / M_PI);
    why = buf;
    return false;
  }
  m.cellXY(link.b, ex, ey);
  if (b2 == kNoCell) {
    std::vector<Waypoint> suffix = {lead_out};
    for (size_t q = L.back().first; q < walk.size() - 1; ++q) {
      const double along =
        (walk[q].x - F.back().head.x) * F.back().dx + (walk[q].y - F.back().head.y) * F.back().dy;
      if (along > p.flights_lead + 0.2) {
        suffix.push_back(walk[q]);
      }
    }
    suffix.push_back({ex, ey, m.zTop(link.b)});
    smoothPolyline(suffix, p.smooth_passes);
    pts.insert(pts.end(), suffix.begin(), suffix.end());
  } else {
    if (std::hypot(ex - lead_out.x, ey - lead_out.y) > 0.05) {
      pts.push_back(lead_out);
    }
    pts.push_back({ex, ey, m.zTop(link.b)});
  }
  std::string widths;
  for (const Flight & f : F) {
    std::snprintf(buf, sizeof buf, "%s%.1f", widths.empty() ? "" : "/", f.width);
    widths += buf;
  }
  std::snprintf(
    buf, sizeof buf, "%zu flights %s m wide, landings: %s, %zu turns", F.size(), widths.c_str(),
    shapes.c_str(), link.turns.size());
  link.geometry = buf;
  addLink(m, std::move(link), out);
  return true;
}

// The row of straight links across the flight the robot walked between
// keyframes ka and kb, from cell a (walked first) to cell b. Returns how many
// it laid; 0, with `why`, when the stretch is not one straight flight or the
// row is narrower than min_width -- the caller then keeps the walked link.
size_t straightLinks(
  const SurfaceMap & m, const std::vector<Waypoint> & track, size_t ka, size_t kb, CellId a,
  CellId b, const StairLinkParams & p, double min_width, StairLinks & out, std::string & why)
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
  double ux = 0.0, uy = 0.0;
  if (!fitUphill(
        m, track, ka, kb, z_lo, z_hi, p.straight_fit_radius, hx - lx, hy - ly, ux, uy, why)) {
    return 0;
  }
  // Every lower-floor cell near the walked foot is tried as an entry; the
  // march itself sorts the stair's edge from the floor behind it (a line
  // from further back runs onto lower-floor cells and is dropped).
  const double reach = p.straight_half_width + 1.0;
  const uint32_t first = static_cast<uint32_t>(out.links.size());
  std::vector<StairLink> row;  // added once the row's width is known
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
        row.push_back(std::move(link));
        perp_min = std::min(perp_min, perp);
        perp_max = std::max(perp_max, perp);
      }
    }
  }
  const size_t laid = row.size();
  if (laid == 0) {
    why = "no straight line from the lower floor reached the upper one";
    return 0;
  }
  const double width = perp_max - perp_min + m.res;
  if (width < min_width) {
    char buf[120];
    std::snprintf(
      buf, sizeof buf, "the straight links span %.2f m, narrower than %.2f m", width, min_width);
    why = buf;
    return 0;
  }
  for (StairLink & link : row) {
    addLink(m, std::move(link), out);
  }
  StairFlight f;
  f.kf_a = ka;
  f.kf_b = kb;
  f.first = first;
  f.count = static_cast<uint32_t>(laid);
  f.heading_deg = std::atan2(uy, ux) * 180.0 / M_PI;
  f.width = width;
  f.z_lo = z_lo;
  f.z_hi = z_hi;
  out.flights.push_back(f);
  return laid;
}

// ---- Stairs the track never took (2026-10-25; see the header) ----

// fn(neighbour) for every surface of the 8 neighbouring columns within one
// step of c, in any state: the step rule without the traversability rules.
template <typename Fn>
void forEachStepNeighbour(const SurfaceMap & m, CellId c, Fn && fn)
{
  const int ix = m.colIx(m.cell_col[c]), iy = m.colIy(m.cell_col[c]);
  const int kz = m.cells[c].kz;
  for (int k = 0; k < 8; ++k) {
    const int jx = ix + SurfaceMap::kDx8[k], jy = iy + SurfaceMap::kDy8[k];
    if (!m.inGrid(jx, jy)) {
      continue;
    }
    const uint32_t col = m.col(jx, jy);
    for (uint32_t i = m.col_begin[col], e = m.col_begin[col + 1]; i < e; ++i) {
      if (std::abs(m.cells[i].kz - kz) <= m.step_cells) {
        fn(static_cast<CellId>(i));
      }
    }
  }
}

// Lays the stairs of the blobs that join two floors (see the header).
// Runs after the track's links, whose stairs it skips.
void detectStairs(
  const SurfaceMap & m, const octomap::OcTree * tree, const StairLinkParams & p,
  const std::vector<size_t> & comp_cells, double min_cells, StairLinks & out)
{
  const size_t n = m.cells.size();
  const auto large = [&](CellId c) {
    return m.traversable(c) && static_cast<double>(comp_cells[m.cells[c].component]) >= min_cells;
  };
  // A rail or the flight overhead in the body band rejects a tread too, but
  // a blob that crosses such cells is a squeeze under something, not a
  // stair: on dp's two stairs the treads are step / footprint / slope and a
  // landing's no_free_above.
  const auto usable = [&](CellId c) {
    return !large(c) && m.cells[c].reject != static_cast<uint8_t>(Reject::ClearanceOccupied);
  };

  // The floors and their edges: owner = the large component, href = the
  // height of the floor cell the edge was reached from (the floor's own
  // height there; growing from each edge cell's own height would creep up a
  // flight a voxel at a time).
  std::vector<uint32_t> owner(n, kNoComponent);
  std::vector<float> href(n, 0.0f);
  std::vector<CellId> queue;
  for (CellId c = 0; c < n; ++c) {
    if (large(c)) {
      owner[c] = m.cells[c].component;
      href[c] = static_cast<float>(m.zTop(c));
      queue.push_back(c);
    }
  }
  for (size_t q = 0; q < queue.size(); ++q) {
    const CellId c = queue[q];
    forEachStepNeighbour(m, c, [&](CellId nb) {
      if (
        owner[nb] != kNoComponent || !usable(nb) ||
        std::fabs(m.zTop(nb) - href[c]) > p.detect_floor_band) {
        return;
      }
      owner[nb] = owner[c];
      href[nb] = href[c];
      queue.push_back(nb);
    });
  }

  // The cells the track's links already cross: their blobs are those stairs.
  std::vector<bool> covered(n, false);
  for (const StairLink & l : out.links) {
    for (size_t i = 1; i < l.pts.size(); ++i) {
      const Waypoint & a = l.pts[i - 1];
      const Waypoint & b = l.pts[i];
      const int steps =
        std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / (m.res / 2))));
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / steps;
        const int ix = m.ixOf(a.x + t * (b.x - a.x)), iy = m.iyOf(a.y + t * (b.y - a.y));
        if (!m.inGrid(ix, iy)) {
          continue;
        }
        const double z = a.z + t * (b.z - a.z);
        const uint32_t col = m.col(ix, iy);
        for (uint32_t c = m.col_begin[col], e = m.col_begin[col + 1]; c < e; ++c) {
          // A straight link is one ramp over the risers: within a riser and
          // a half of every tread it crosses.
          if (std::fabs(m.zTop(c) - z) <= 0.4) {
            covered[c] = true;
          }
        }
      }
    }
  }

  struct Contact
  {
    CellId cell;  // the blob's cell
    uint32_t floor;
    float z;  // the floor's height there
  };
  std::vector<uint32_t> blob(n, kNoComponent);
  std::vector<CellId> members;
  std::vector<Contact> contacts;
  uint32_t nblobs = 0;
  const size_t min_members = static_cast<size_t>(std::ceil(p.detect_min_area / (m.res * m.res)));
  for (CellId s = 0; s < n; ++s) {
    if (blob[s] != kNoComponent || owner[s] != kNoComponent || !usable(s)) {
      continue;
    }
    const uint32_t id = nblobs++;
    members.assign(1, s);
    contacts.clear();
    blob[s] = id;
    for (size_t q = 0; q < members.size(); ++q) {
      const CellId c = members[q];
      forEachStepNeighbour(m, c, [&](CellId nb) {
        if (owner[nb] != kNoComponent) {
          contacts.push_back({c, owner[nb], href[nb]});
          return;
        }
        if (blob[nb] != kNoComponent || !usable(nb)) {
          return;
        }
        blob[nb] = id;
        members.push_back(nb);
      });
    }
    if (members.size() < min_members || contacts.empty()) {
      continue;
    }
    size_t n_covered = 0;
    for (const CellId c : members) {
      n_covered += covered[c] ? 1 : 0;
    }
    if (n_covered >= 3) {
      continue;
    }
    // The floors it touches, each at the median height of its contacts.
    std::unordered_map<uint32_t, std::vector<double>> floor_z;
    for (const Contact & k : contacts) {
      floor_z[k.floor].push_back(k.z);
    }
    uint32_t lo = kNoComponent, hi = kNoComponent;
    double z_lo = 0.0, z_hi = 0.0;
    for (auto & [f, zs] : floor_z) {
      const double z = median(zs);
      if (lo == kNoComponent || z < z_lo) {
        lo = f;
        z_lo = z;
      }
      if (hi == kNoComponent || z > z_hi) {
        hi = f;
        z_hi = z;
      }
    }
    if (z_hi - z_lo < p.min_rise) {
      continue;
    }

    // The stand-in walk: the shortest way through the blob from the lower
    // floor's edge to the upper one's (Dijkstra, 3D cell-centre distances),
    // kept off the blob's edges. The plain shortest way cuts a U landing's
    // inside corner, round the newel, in under 0.5 m of level walk, and
    // flightLinks then reads no landing at all (dp's middle stair, read as
    // one 5.5 m flight). Each move costs more the nearer it ends to an edge
    // than kKeepOff, up to (1 + kEdgeWeight) times its length at the edge, so
    // the walk takes a landing across its middle as an operator would.
    constexpr double kKeepOff = 0.5, kEdgeWeight = 20.0;
    std::unordered_map<CellId, uint32_t> local;
    local.reserve(members.size() * 2);
    for (uint32_t i = 0; i < members.size(); ++i) {
      local.emplace(members[i], i);
    }
    // Distance to the edge, in moves: a cell is on the edge when one of its
    // eight neighbouring columns holds surfaces but none within kEdgeDz of
    // it -- a wall, the drop to the floor or the next flight. Not the step
    // rule: dp's middle landing holds two levels 0.3 m apart side by side
    // (two mapping passes), and read by steps its middle was all edges and
    // the walk went round its rim. A column with nothing in it is no edge
    // (an unseen tread); then a breadth-first pass inward.
    constexpr double kEdgeDz = 0.35;
    std::vector<int> edge(members.size(), -1);
    {
      std::vector<uint32_t> bfs;
      for (uint32_t i = 0; i < members.size(); ++i) {
        const CellId c = members[i];
        const int ix = m.colIx(m.cell_col[c]), iy = m.colIy(m.cell_col[c]);
        bool on_edge = false;
        for (int k = 0; k < 8 && !on_edge; ++k) {
          const int jx = ix + SurfaceMap::kDx8[k], jy = iy + SurfaceMap::kDy8[k];
          if (!m.inGrid(jx, jy)) {
            on_edge = true;
            continue;
          }
          const uint32_t col = m.col(jx, jy);
          on_edge = m.col_begin[col] != m.col_begin[col + 1] &&
                    m.surfaceNear(col, m.zTop(c), kEdgeDz) == kNoCell;
        }
        if (on_edge) {
          edge[i] = 0;
          bfs.push_back(i);
        }
      }
      for (size_t q = 0; q < bfs.size(); ++q) {
        const uint32_t i = bfs[q];
        forEachStepNeighbour(m, members[i], [&](CellId nb) {
          const auto it = local.find(nb);
          if (it != local.end() && edge[it->second] < 0) {
            edge[it->second] = edge[i] + 1;
            bfs.push_back(it->second);
          }
        });
      }
    }
    const auto edge_cost = [&](uint32_t i) {
      const double d = std::max(0, edge[i]) * m.res;
      return 1.0 + kEdgeWeight * std::max(0.0, kKeepOff - d) / kKeepOff;
    };
    const auto pos = [&](CellId c) {
      double x, y;
      m.cellXY(c, x, y);
      return Waypoint{x, y, m.zTop(c)};
    };
    std::vector<double> dist(members.size(), std::numeric_limits<double>::infinity());
    std::vector<uint32_t> prev(members.size(), ~uint32_t(0));
    std::vector<bool> goal(members.size(), false);
    using Item = std::pair<double, uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    for (const Contact & k : contacts) {
      const uint32_t i = local.at(k.cell);
      if (k.floor == lo && std::fabs(k.z - z_lo) < p.min_rise / 2 && dist[i] > 0.0) {
        dist[i] = 0.0;
        open.push({0.0, i});
      }
      if (k.floor == hi && std::fabs(k.z - z_hi) < p.min_rise / 2) {
        goal[i] = true;
      }
    }
    uint32_t reached = ~uint32_t(0);
    while (!open.empty()) {
      const auto [d, i] = open.top();
      open.pop();
      if (d > dist[i]) {
        continue;
      }
      if (goal[i]) {
        reached = i;
        break;
      }
      const Waypoint a = pos(members[i]);
      forEachStepNeighbour(m, members[i], [&](CellId nb) {
        const auto it = local.find(nb);
        if (it == local.end()) {
          return;
        }
        const Waypoint b = pos(nb);
        const double nd =
          d + edge_cost(it->second) * std::sqrt(
                                        (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) +
                                        (b.z - a.z) * (b.z - a.z));
        if (nd < dist[it->second]) {
          dist[it->second] = nd;
          prev[it->second] = i;
          open.push({nd, it->second});
        }
      });
    }
    double bx = 0, by = 0;
    m.cellXY(s, bx, by);
    char where[160];
    std::snprintf(
      where, sizeof where, "a stair-like stretch at (%.1f, %.1f), z %.2f -> %.2f, %.1f m2", bx, by,
      z_lo, z_hi, members.size() * m.res * m.res);
    if (reached == ~uint32_t(0)) {
      out.notes.push_back(std::string(where) + ": no way through it from floor to floor");
      continue;
    }
    std::vector<Waypoint> path;
    for (uint32_t i = reached; i != ~uint32_t(0); i = prev[i]) {
      path.push_back(pos(members[i]));
    }
    std::reverse(path.begin(), path.end());
    const CellId a = nearestTraversable(m, path.front().x, path.front().y, z_lo, lo, 2.0);
    const CellId b = nearestTraversable(m, path.back().x, path.back().y, z_hi, hi, 2.0);
    if (a == kNoCell || b == kNoCell) {
      out.notes.push_back(std::string(where) + ": no floor cell within 2 m of an end");
      continue;
    }
    std::vector<Waypoint> walk;
    walk.push_back(pos(a));
    walk.insert(walk.end(), path.begin(), path.end());
    walk.push_back(pos(b));

    const size_t first = out.links.size(), first_flight = out.flights.size();
    std::string why, fwhy;
    size_t laid = 0;
    if (p.straight) {
      laid = straightLinks(m, walk, 0, walk.size() - 1, a, b, p, p.detect_min_width, out, why);
    }
    if (laid == 0 && p.flights) {
      if (flightLinks(m, tree, walk, a, b, 0, walk.size() - 1, p, p.detect_min_width, out, fwhy)) {
        laid = out.links.size() - first;
      }
    }
    if (laid == 0) {
      out.notes.push_back(
        std::string(where) + ": not laid -- " + (p.straight ? why : "not laid straight") +
        (p.flights ? "; " + fwhy : ""));
      continue;
    }
    for (size_t i = first; i < out.links.size(); ++i) {
      out.links[i].detected = true;
      out.links[i].kf_a = out.links[i].kf_b = 0;
    }
    for (size_t i = first_flight; i < out.flights.size(); ++i) {
      out.flights[i].detected = true;
      out.flights[i].kf_a = out.flights[i].kf_b = 0;
    }
  }
}

// ---- Kerbs (2026-10-29; see the header) ----

// The direction of the line through points (xs, ys) by its principal axis:
// (tx, ty) unit, and the spread of the points along it (m). False with fewer
// than three points, or when they are a blob rather than a line (the minor
// axis holds more than half the major one's spread).
bool fitLine(
  const std::vector<double> & xs, const std::vector<double> & ys, double & tx, double & ty,
  double & extent)
{
  const size_t n = xs.size();
  if (n < 3) {
    return false;
  }
  double mx = 0, my = 0;
  for (size_t i = 0; i < n; ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;
  double sxx = 0, syy = 0, sxy = 0;
  for (size_t i = 0; i < n; ++i) {
    const double dx = xs[i] - mx, dy = ys[i] - my;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  const double theta = 0.5 * std::atan2(2 * sxy, sxx - syy);
  tx = std::cos(theta);
  ty = std::sin(theta);
  double lo = 1e300, hi = -1e300, lo2 = 1e300, hi2 = -1e300;
  for (size_t i = 0; i < n; ++i) {
    const double s = (xs[i] - mx) * tx + (ys[i] - my) * ty;
    const double q = -(xs[i] - mx) * ty + (ys[i] - my) * tx;
    lo = std::min(lo, s);
    hi = std::max(hi, s);
    lo2 = std::min(lo2, q);
    hi2 = std::max(hi2, q);
  }
  extent = hi - lo;
  return extent > 0 && (hi2 - lo2) <= 0.5 * extent + 1e-9;
}

// Lays the kerb links (see the header): at each low-side cell of every kerb
// the surface map found, a straight crossing square to the kerb, as far as
// the lead either side runs over level traversable ground. Runs last, after
// the stairs: a stair's first riser is never a kerb (the flight's cells are
// all rejected, so no traversable cell steps up onto it).
void kerbLinks(const SurfaceMap & m, const StairLinkParams & p, StairLinks & out)
{
  out.kerb_edge.assign(m.cells.size(), 0);
  if (!p.kerb_links) {
    return;
  }
  const double lead = std::max(p.kerb_lead, m.res);
  const double half_step = m.res / 2;
  std::unordered_map<uint64_t, uint8_t> laid;  // (a, b) pairs already linked
  std::vector<double> xs, ys;
  for (const std::vector<CellId> & cells : m.kerb_lines) {
    int rise = 0;
    size_t links_here = 0;
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (const CellId c : cells) {
      const int kz = m.cells[c].kz;
      double cx, cy;
      m.cellXY(c, cx, cy);
      xmin = std::min(xmin, cx);
      xmax = std::max(xmax, cx);
      ymin = std::min(ymin, cy);
      ymax = std::max(ymax, cy);
      // The way up: the mean offset to the higher orthogonal neighbours,
      // which also orients the normal below.
      double hx = 0, hy = 0;
      m.forEachMove(c, [&](CellId nb, int k) {
        if ((k & 1) == 0 && m.cells[nb].kz > kz) {
          hx += SurfaceMap::kDx8[k];
          hy += SurfaceMap::kDy8[k];
          rise = std::max(rise, m.cells[nb].kz - kz);
        }
      });
      if (std::hypot(hx, hy) < 1e-9) {
        continue;  // two high neighbours opposite each other: a one-cell gutter, no way across
      }
      // The line's direction here: a fit over its cells within the radius;
      // a stretch too short to fit (a corner's few cells) takes the step's
      // own direction.
      xs.clear();
      ys.clear();
      for (const CellId o : cells) {
        double ox, oy;
        m.cellXY(o, ox, oy);
        if (std::hypot(ox - cx, oy - cy) <= p.kerb_fit_radius) {
          xs.push_back(ox);
          ys.push_back(oy);
        }
      }
      double lx, ly, local_extent;
      double ux, uy;
      if (fitLine(xs, ys, lx, ly, local_extent) && local_extent >= 2 * m.res) {
        ux = -ly;
        uy = lx;
        if (ux * hx + uy * hy < 0) {
          ux = -ux;
          uy = -uy;
        }
      } else {
        const double h = std::hypot(hx, hy);
        ux = hx / h;
        uy = hy / h;
      }
      // March the crossing: from lead before the edge (half a cell past
      // this cell's centre, the way up) to lead past it, half a cell at a
      // time, column by column, each resolved to its layer within one step
      // of the last. Up to and including this cell's column the ground must
      // stay within a cell of this cell's height, the first column past it
      // must be higher, and from there on the ground must stay within a
      // cell of that: a second step, a drop or a wall within the lead means
      // no link here. The samples sit a quarter cell off the half-cell
      // grid, so none lands on a column boundary, where floor() and the
      // rounding of -lead + k * half_step would decide the column between
      // them.
      const double ex = cx + half_step * ux, ey = cy + half_step * uy;
      const uint32_t col_c = m.cell_col[c];
      CellId a = kNoCell, b = kNoCell, before = kNoCell, after = kNoCell;
      uint32_t last_col = ~uint32_t(0);
      int kz_prev = kz, kz_hi = kz;
      bool ok = true, passed = false;
      for (double s = -lead + half_step / 2; s <= lead; s += half_step) {
        const double x = ex + s * ux, y = ey + s * uy;
        const int ix = m.ixOf(x), iy = m.iyOf(y);
        if (!m.inGrid(ix, iy)) {
          ok = false;
          break;
        }
        const uint32_t col = m.col(ix, iy);
        if (col == last_col) {
          continue;
        }
        last_col = col;
        const CellId cell = m.layerNear(col, kz_prev, m.step_cells);
        if (cell == kNoCell || !m.traversable(cell)) {
          ok = false;
          break;
        }
        const int kzc = m.cells[cell].kz;
        if (col == col_c) {
          passed = true;
          before = cell;
        } else if (!passed) {
          if (std::abs(kzc - kz) > 1) {
            ok = false;
            break;
          }
          before = cell;
        } else if (after == kNoCell) {
          if (kzc <= kz) {
            ok = false;  // the first column past the edge is not higher: no kerb here
            break;
          }
          after = cell;
          kz_hi = kzc;
        } else if (std::abs(kzc - kz_hi) > 1) {
          ok = false;
          break;
        }
        kz_prev = kzc;
        if (a == kNoCell) {
          a = cell;
        }
        b = cell;
      }
      if (!ok || !passed || before == kNoCell || after == kNoCell || a == b) {
        continue;
      }
      const uint64_t key = (static_cast<uint64_t>(a) << 32) | b;
      if (laid.count(key)) {
        continue;
      }
      laid[key] = 1;
      StairLink link;
      link.a = a;
      link.b = b;
      link.kerb = true;
      link.straight = true;
      link.detected = true;
      link.up_x = ux;
      link.up_y = uy;
      double ax, ay, bx, by;
      m.cellXY(a, ax, ay);
      m.cellXY(b, bx, by);
      link.pts.push_back({ax, ay, m.zTop(a)});
      link.pts.push_back({cx, cy, m.zTop(before)});
      link.pts.push_back({cx + m.res * ux, cy + m.res * uy, m.zTop(after)});
      link.pts.push_back({bx, by, m.zTop(b)});
      addLink(m, std::move(link), out);
      ++links_here;
      // The crossing is a link's from here on: this cell, the high
      // neighbours it steps up to, and the two cells the march stepped
      // between.
      out.kerb_edge[c] = 1;
      out.kerb_edge[before] = 1;
      out.kerb_edge[after] = 1;
      m.forEachMove(c, [&](CellId nb, int k) {
        if ((k & 1) == 0 && m.cells[nb].kz > kz) {
          out.kerb_edge[nb] = 1;
        }
      });
    }
    if (links_here > 0) {
      ++out.kerbs;
    } else {
      double x, y;
      m.cellXY(cells.front(), x, y);
      char buf[200];
      std::snprintf(
        buf, sizeof buf,
        "kerb at (%.1f, %.1f) (%.1f m long, %d cells high): no straight crossing of %.1f m either "
        "side could be laid, so it is crossed as before",
        x, y, std::max(xmax - xmin, ymax - ymin) + m.res, rise, lead);
      out.notes.push_back(buf);
    }
  }
}

}  // namespace

std::shared_ptr<const StairLinks> buildStairLinks(
  const SurfaceMap & m, const std::vector<Waypoint> & track, const StairLinkParams & params,
  const octomap::OcTree * tree)
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
      laid = straightLinks(m, track, anchor_kf, k, anchor, c, params, 0.0, *out, why);
    }
    if (laid == 0) {
      // The walk itself, loops cut.
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
      std::string fwhy;
      if (params.flights) {
        // Flights and landing corners read off the map (see the header).
        if (flightLinks(m, tree, link.pts, anchor, c, anchor_kf, k, params, 0.0, *out, fwhy)) {
          anchor = c;
          anchor_kf = k;
          walked = 0.0;
          continue;
        }
      }
      char buf[320];
      std::snprintf(
        buf, sizeof buf,
        "keyframes %zu..%zu (z %.2f -> %.2f): %s%s%s; the link follows the walked "
        "track",
        anchor_kf, k, m.zTop(anchor), m.zTop(c),
        params.straight ? why.c_str() : "not laid straight", params.flights ? "; " : "",
        params.flights ? fwhy.c_str() : "");
      out->notes.push_back(buf);
      smoothPolyline(link.pts, params.smooth_passes);
      if (params.centre) {
        // Measured square to the smoothed walk, then smoothed again: the
        // shifts are per point, and the walk's weave is gone by then.
        centreWalk(m, params, link);
        smoothPolyline(link.pts, params.smooth_passes);
      }
      addLink(m, std::move(link), *out);
    }
    anchor = c;
    anchor_kf = k;
    walked = 0.0;
  }
  if (params.detect) {
    detectStairs(m, tree, params, comp_cells, min_cells, *out);
  }
  kerbLinks(m, params, *out);
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
