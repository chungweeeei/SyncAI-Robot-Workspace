#include "syncai_global_planner/plugins/octomap_planner_3d/keepout_zones.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace syncai_global_planner::octomap_planner_3d
{

namespace
{

// The number of a JSON value, or false when it is not one or not finite.
bool finiteNumber(const nlohmann::json & j, double & out)
{
  if (!j.is_number()) {
    return false;
  }
  out = j.get<double>();
  return std::isfinite(out);
}

}  // namespace

bool parseKeepoutJson(
  const std::string & text, std::vector<KeepoutZone> & zones, std::string & error)
{
  // No exceptions out of here (the plugin runs this on a service thread and
  // a truncated file is an answer, not a crash): the parser is told to
  // return a discarded value, and every field is type-checked before it is
  // read, as syncai_localizer's last_pose_store does.
  const nlohmann::json j = nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) {
    error = "not valid JSON";
    return false;
  }
  if (!j.is_object()) {
    error = "the top level is not a JSON object";
    return false;
  }
  if (j.contains("version")) {
    if (!j["version"].is_number_integer()) {
      error = "'version' is not an integer";
      return false;
    }
    const auto v = j["version"].get<int64_t>();
    if (v > 1) {
      error = "version " + std::to_string(v) + " is newer than this planner understands (1)";
      return false;
    }
  }
  if (!j.contains("zones") || !j["zones"].is_array()) {
    error = "'zones' is missing or not an array";
    return false;
  }
  std::vector<KeepoutZone> out;
  std::set<std::string> ids;
  size_t index = 0;
  for (const nlohmann::json & jz : j["zones"]) {
    const std::string where = "zone " + std::to_string(index++);
    if (!jz.is_object()) {
      error = where + " is not an object";
      return false;
    }
    KeepoutZone z;
    if (!jz.contains("id") || !jz["id"].is_string() || jz["id"].get<std::string>().empty()) {
      error = where + ": 'id' is missing or not a non-empty string";
      return false;
    }
    z.id = jz["id"].get<std::string>();
    if (!ids.insert(z.id).second) {
      error = where + ": duplicate id '" + z.id + "'";
      return false;
    }
    if (jz.contains("name")) {
      if (!jz["name"].is_string()) {
        error = where + " ('" + z.id + "'): 'name' is not a string";
        return false;
      }
      z.name = jz["name"].get<std::string>();
    }
    if (!jz.contains("polygon") || !jz["polygon"].is_array()) {
      error = where + " ('" + z.id + "'): 'polygon' is missing or not an array";
      return false;
    }
    for (const nlohmann::json & v : jz["polygon"]) {
      double x, y;
      if (!v.is_array() || v.size() != 2 || !finiteNumber(v[0], x) || !finiteNumber(v[1], y)) {
        error = where + " ('" + z.id + "'): a polygon vertex is not a finite [x, y] pair";
        return false;
      }
      z.polygon.emplace_back(x, y);
    }
    // An editor that closes its rings explicitly repeats the first vertex;
    // the point-in-polygon test closes the ring itself.
    if (z.polygon.size() >= 2 && z.polygon.front() == z.polygon.back()) {
      z.polygon.pop_back();
    }
    if (z.polygon.size() < 3) {
      error = where + " ('" + z.id + "'): 'polygon' needs at least three distinct vertices";
      return false;
    }
    for (const char * key : {"z_min", "z_max"}) {
      if (!jz.contains(key)) {
        continue;
      }
      double v;
      if (!finiteNumber(jz[key], v)) {
        error = where + " ('" + z.id + "'): '" + key + "' is not a finite number";
        return false;
      }
      (std::string(key) == "z_min" ? z.z_min : z.z_max) = v;
    }
    if (z.z_min > z.z_max) {
      error = where + " ('" + z.id + "'): z_min is above z_max";
      return false;
    }
    out.push_back(std::move(z));
  }
  zones = std::move(out);
  return true;
}

bool readKeepoutFile(
  const std::string & path, std::vector<KeepoutZone> & zones, std::string & error)
{
  std::ifstream in(path);
  if (!in) {
    error = "cannot open " + path;
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  if (!parseKeepoutJson(ss.str(), zones, error)) {
    error = path + ": " + error;
    return false;
  }
  return true;
}

bool zoneContains(const KeepoutZone & z, double x, double y, double zz)
{
  if (zz < z.z_min || zz > z.z_max) {
    return false;
  }
  // Even-odd ray casting along +x: inside when the ray from (x, y) crosses
  // an odd number of edges. A vertex exactly on the ray is counted once
  // (the half-open test on y), the usual rule.
  bool inside = false;
  const size_t n = z.polygon.size();
  for (size_t i = 0, k = n - 1; i < n; k = i++) {
    const auto & [xi, yi] = z.polygon[i];
    const auto & [xk, yk] = z.polygon[k];
    if ((yi > y) != (yk > y)) {
      const double xc = xk + (y - yk) * (xi - xk) / (yi - yk);
      if (x < xc) {
        inside = !inside;
      }
    }
  }
  return inside;
}

KeepoutMaskPtr buildKeepoutMask(
  const SurfaceMap & m, const StairLinks * links, const std::vector<KeepoutZone> & zones,
  double block_radius)
{
  auto mask = std::make_shared<KeepoutMask>();
  mask->block_radius = block_radius;
  mask->zones = zones.size();
  mask->d.assign(m.cells.size(), std::numeric_limits<float>::infinity());
  std::vector<CellId> seeds;
  for (const KeepoutZone & z : zones) {
    // The polygon's bounding box in columns, clipped to the grid; every cell
    // of every column in it whose centre the polygon holds and whose top
    // face the band holds is a seed -- traversable or not, as the 2D mask
    // marks whatever it is drawn over (a zone drawn flush against a wall
    // covers the wall's ring, and its dilation still reaches 0.25 m into the
    // corridor, which is the point).
    double xmin = z.polygon[0].first, xmax = xmin, ymin = z.polygon[0].second, ymax = ymin;
    for (const auto & [x, y] : z.polygon) {
      xmin = std::min(xmin, x);
      xmax = std::max(xmax, x);
      ymin = std::min(ymin, y);
      ymax = std::max(ymax, y);
    }
    const int ix0 = std::max(0, m.ixOf(xmin)), ix1 = std::min(m.nx - 1, m.ixOf(xmax));
    const int iy0 = std::max(0, m.iyOf(ymin)), iy1 = std::min(m.ny - 1, m.iyOf(ymax));
    for (int iy = iy0; iy <= iy1; ++iy) {
      for (int ix = ix0; ix <= ix1; ++ix) {
        const double cx = m.cx(ix), cy = m.cy(iy);
        const uint32_t c = m.col(ix, iy);
        for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
          if (mask->d[i] <= 0.0f) {
            continue;  // already a seed of an earlier zone
          }
          if (zoneContains(z, cx, cy, m.zTop(i))) {
            mask->d[i] = 0.0f;
            seeds.push_back(i);
          }
        }
      }
    }
  }
  if (links) {
    mask->link_cut.assign(links->links.size(), 0);
    for (size_t li = 0; li < links->links.size(); ++li) {
      for (const Waypoint & w : links->links[li].pts) {
        bool in = false;
        for (const KeepoutZone & z : zones) {
          if (zoneContains(z, w.x, w.y, w.z)) {
            in = true;
            break;
          }
        }
        if (in) {
          mask->link_cut[li] = 1;
          ++mask->links_cut;
          break;
        }
      }
    }
  }
  mask->core = seeds.size();
  if (seeds.empty() && mask->links_cut == 0) {
    return nullptr;
  }
  dilateOnSurface(
    m, seeds, discOffsets(block_radius + m.params.clearance_cost_radius, m.res, false), mask->d);
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.traversable(c) && mask->d[c] <= block_radius) {
      ++mask->blocked;
    }
  }
  return mask;
}

const KeepoutZone * zoneAt(const std::vector<KeepoutZone> & zones, const SurfaceMap & m, CellId c)
{
  double x, y;
  m.cellXY(c, x, y);
  const double z = m.zTop(c);
  for (const KeepoutZone & zone : zones) {
    if (zoneContains(zone, x, y, z)) {
      return &zone;
    }
  }
  return nullptr;
}

std::string zoneLabel(const KeepoutZone & z)
{
  return "'" + z.id + "'" + (z.name.empty() ? std::string() : " (" + z.name + ")");
}

}  // namespace syncai_global_planner::octomap_planner_3d
