#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"

#include <algorithm>
#include <cmath>
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

uint32_t findRoot(std::vector<uint32_t> & parent, uint32_t i)
{
  while (parent[i] != i) {
    parent[i] = parent[parent[i]];
    i = parent[i];
  }
  return i;
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
    for (size_t i = 1; i < link.pts.size(); ++i) {
      const Waypoint & p = link.pts[i - 1];
      const Waypoint & q = link.pts[i];
      link.length += std::sqrt(
        (q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
    }
    const uint32_t id = static_cast<uint32_t>(out->links.size());
    out->at[link.a].push_back(id);
    out->at[link.b].push_back(id);
    const uint32_t ra = findRoot(out->group, m.cells[link.a].component);
    const uint32_t rb = findRoot(out->group, m.cells[link.b].component);
    if (ra != rb) {
      out->group[std::max(ra, rb)] = std::min(ra, rb);
    }
    out->links.push_back(std::move(link));
    anchor = c;
    anchor_kf = k;
    walked = 0.0;
  }
  for (uint32_t i = 0; i < ncomp; ++i) {
    out->group[i] = findRoot(out->group, i);
  }
  return out;
}

}  // namespace syncai_global_planner::octomap_planner_3d
