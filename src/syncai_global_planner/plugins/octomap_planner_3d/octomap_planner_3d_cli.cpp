// Offline driver for OctomapPlanner3D's core: builds the surface map from a
// map's octomap.bt (or a synthetic scene), prints what it made of it, checks
// it against the robot's own trajectory, dumps it for a viewer, and plans.
// No ROS: the same binary builds in the workspace (ros2 run
// syncai_global_planner octomap_planner_3d_cli) and from
// plugins/octomap_planner_3d/standalone/CMakeLists.txt on a workstation.
//
// This is the tool the geometry parameters were tuned with, and the first
// thing to run on a new map: --stats says how much of the map is ground the
// robot may stand on and why the rest is not, --check-poses says whether the
// floor the robot actually walked on survived as traversable (the hole-fill's
// acceptance test), --dump-pcd shows both in any point-cloud viewer. With
// --poses, the track's flights also become stair links (stair_links.hpp), as
// in the plugin, so --plan / --plan-poses can go between floors.

#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/keepout_zones.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/synthetic_octree.hpp"

using namespace syncai_global_planner::octomap_planner_3d;  // NOLINT

namespace
{

void usage()
{
  std::fprintf(
    stderr,
    "usage: octomap_planner_3d_cli (<octomap.bt> | --synthetic <scene>) [options]\n"
    "  --param <name>=<value>   override a SurfaceMapParams / SearchParams field (repeatable)\n"
    "  --stats                  build statistics (the default action)\n"
    "  --self-test [N]          recompute N random columns with OcTree::search() and compare\n"
    "  --poses <poses.txt>      keyframe poses (name x y z qw qx qy qz), the map's own track\n"
    "  --check-poses            how many keyframe ground points lie on a traversable cell\n"
    "  --lidar-height <m>       height of the keyframe poses above the ground (0.481)\n"
    "  --dump-pcd <out.pcd>     every surface cell as an ASCII PCD (x y z label reject unknown "
    "clearance component)\n"
    "  --dump-image <out.bmp>   top-down raster of the layer nearest --image-z (default: the "
    "lowest floor)\n"
    "  --image-z <z>            reference height for --dump-image and --inspect\n"
    "  --inspect <x> <y>        ASCII map of the 15 x 15 columns around a point (layer nearest "
    "--image-z)\n"
    "  --inspect-misses [N]     the same around the first N keyframes that failed --check-poses\n"
    "  --inspect-kf <i>         the same around keyframe i of --poses (repeatable)\n"
    "  --image-region <xmin> <ymin> <xmax> <ymax>   crop --dump-image to a region (m)\n"
    "  --image-scale <k>        enlarge --dump-image k times\n"
    "  --image-components       colour --dump-image by component instead of by state\n"
    "  --debug-fill <x> <y>     replay the hole-fill decision for one column (uses --image-z)\n"
    "  --plan <sx> <sy> <sz> <gx> <gy> <gz>   plan between two points (z = height hints)\n"
    "  --dynamic <x> <y> <z>    a person standing at (x, y) on the floor at height z: eighteen\n"
    "                           returns 0.3 and 0.5 m up on a 0.3 m square, marked in the\n"
    "                           dynamic overlay before every --plan as the robot's scans would\n"
    "                           mark them (repeatable; the plan detours round them)\n"
    "  --keepout <keepout.json> the map's keepout zones (keepout_zones.hpp): rasterised on the\n"
    "                           map with robot_radius as the ring, read by every --plan as the\n"
    "                           plugin reads them, drawn by --dump-image (crimson core, dark red\n"
    "                           ring)\n"
    "  --plan-poses <i> <j>     plan between keyframes i and j of --poses (ground = z - lidar "
    "height)\n"
    "  --out <path.csv>         write the planned path (x,y,z per line)\n"
    "  --no-shortcut            keep the raw A* cell chain\n"
    "  --no-stair-links         no stair links at all: neither the --poses track's flights\n"
    "                           nor the stairs detected off the map (--param stair_detect=0\n"
    "                           keeps the track's only)\n"
    "  --dump-html <out.html>   self-contained three.js viewer of the surface map, the planned "
    "paths and (with --poses) the stair links\n"
    "  --write-bt <out.bt>      write the tree (a --synthetic scene, say) as an OctoMap .bt; "
    "alone, nothing else is done\n"
    "synthetic scenes: flat wall ramp stairs kerb drop hole two_floors\n");
}

struct Pose
{
  double x, y, z;
};

// A planned route, kept for --dump-image / --dump-html.
struct PlannedPath
{
  std::string label;
  Waypoint start, goal;
  std::vector<Waypoint> pts;
};

bool readPoses(const std::string & path, std::vector<Pose> & out)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string name;
    Pose p{};
    if (ss >> name >> p.x >> p.y >> p.z) {
      out.push_back(p);
    }
  }
  return true;
}

// "kf 2221 -> 2358" for a stair the track found, "detected at (5.9, -12.2)"
// (its first point) for one found off the map.
std::string linkName(const StairLink & l, const char * arrow = "->")
{
  char buf[80];
  if (l.detected) {
    std::snprintf(buf, sizeof buf, "detected at (%.1f, %.1f)", l.pts.front().x, l.pts.front().y);
  } else {
    std::snprintf(buf, sizeof buf, "kf %zu %s %zu", l.kf_a, arrow, l.kf_b);
  }
  return buf;
}

bool setParam(SurfaceMapParams & p, SearchParams & sp, StairLinkParams & st, const std::string & kv)
{
  const auto eq = kv.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  const std::string k = kv.substr(0, eq);
  const double v = std::atof(kv.substr(eq + 1).c_str());
  if (k == "clearance_cost_weight") {
    sp.clearance_cost_weight = v;
  } else if (k == "clearance_cost_scaling") {
    sp.clearance_cost_scaling = v;
  } else if (k == "inferred_cost_multiplier") {
    sp.inferred_cost_multiplier = v;
  } else if (k == "heuristic_weight") {
    sp.heuristic_weight = v;
  } else if (k == "max_iterations") {
    sp.max_iterations = static_cast<size_t>(v);
  } else if (k == "max_planning_time") {
    sp.max_planning_time = v;
  } else if (k == "snap_xy_radius") {
    sp.snap_xy_radius = v;
  } else if (k == "snap_z_window") {
    sp.snap_z_window = v;
  } else if (k == "snap_single_layer") {
    sp.snap_single_layer = v != 0.0;
  } else if (k == "stair_cost_multiplier") {
    sp.stair_cost_multiplier = v;
  } else if (k == "stair_min_rise") {
    st.min_rise = v;
  } else if (k == "stair_max_length") {
    st.max_length = v;
  } else if (k == "stair_anchor_z_window") {
    st.anchor_z_window = v;
  } else if (k == "stair_anchor_min_area") {
    st.anchor_min_area = v;
  } else if (k == "stair_loop_radius") {
    st.loop_radius = v;
  } else if (k == "stair_loop_dz") {
    st.loop_dz = v;
  } else if (k == "stair_smooth_passes") {
    st.smooth_passes = static_cast<int>(v);
  } else if (k == "stair_straight") {
    st.straight = v != 0.0;
  } else if (k == "stair_straight_max_flight") {
    st.straight_max_flight = v;
  } else if (k == "stair_straight_fit_radius") {
    st.straight_fit_radius = v;
  } else if (k == "stair_straight_half_width") {
    st.straight_half_width = v;
  } else if (k == "stair_flights") {
    st.flights = v != 0.0;
  } else if (k == "stair_flights_landing_min_len") {
    st.flights_landing_min_len = v;
  } else if (k == "stair_flights_landing_slope_deg") {
    st.flights_landing_slope_deg = v;
  } else if (k == "stair_flights_lead") {
    st.flights_lead = v;
  } else if (k == "stair_flights_turn_radius") {
    st.flights_turn_radius = v;
  } else if (k == "stair_centre") {
    st.centre = v != 0.0;
  } else if (k == "stair_detect") {
    st.detect = v != 0.0;
  } else if (k == "stair_detect_floor_band") {
    st.detect_floor_band = v;
  } else if (k == "stair_detect_min_width") {
    st.detect_min_width = v;
  } else if (k == "stair_detect_min_area") {
    st.detect_min_area = v;
  } else if (k == "stair_centre_dz") {
    st.centre_dz = v;
  } else if (k == "stair_centre_max_half_width") {
    st.centre_max_half_width = v;
  } else if (k == "stair_centre_max_shift") {
    st.centre_max_shift = v;
  } else if (k == "stair_straight_exit_reach") {
    st.straight_exit_reach = v;
  } else if (k == "robot_radius") {
    p.robot_radius = v;
  } else if (k == "body_clearance") {
    p.body_clearance = v;
  } else if (k == "max_step_height") {
    p.max_step_height = v;
  } else if (k == "max_slope_deg") {
    p.max_slope_deg = v;
  } else if (k == "max_unknown_clearance_cells") {
    p.max_unknown_clearance_cells = static_cast<int>(v);
  } else if (k == "min_known_free_above") {
    p.min_known_free_above = static_cast<int>(v);
  } else if (k == "hole_fill_radius") {
    p.hole_fill_radius = v;
  } else if (k == "hole_fill_min_support") {
    p.hole_fill_min_support = v;
  } else if (k == "hole_fill_passes") {
    p.hole_fill_passes = static_cast<int>(v);
  } else if (k == "clearance_cost_radius") {
    p.clearance_cost_radius = v;
  } else if (k == "footprint_min_support") {
    p.footprint_min_support = v;
  } else if (k == "gap_cost_weight") {
    sp.gap_cost_weight = v;
  } else {
    return false;
  }
  return true;
}

void printStats(const SurfaceMap & m, const BuildStats & s)
{
  std::printf(
    "grid: res %.3f  nx %d ny %d nz %d  (%zu columns)  origin (%.2f, %.2f, %.2f)\n", m.res, m.nx,
    m.ny, m.nz, s.columns, m.x0, m.y0, m.z0);
  std::printf("leaves: %zu (occupied %zu, free %zu)\n", s.leaves, s.occupied_leaves, s.free_leaves);
  std::printf(
    "surface cells: %zu candidates + %zu inferred = %zu;  traversable %zu (%.1f %%)\n",
    s.candidates, s.inferred, m.cells.size(), s.traversable,
    m.cells.empty() ? 0.0 : 100.0 * s.traversable / m.cells.size());
  for (size_t r = 1; r < static_cast<size_t>(Reject::Count_); ++r) {
    std::printf("  rejected %-20s %zu\n", rejectName(static_cast<Reject>(r)), s.rejected[r]);
  }
  std::printf("components: %zu", s.components);
  for (size_t i = 0; i < std::min<size_t>(8, s.component_sizes.size()); ++i) {
    std::printf("%s%zu", i == 0 ? "  sizes: " : ", ", s.component_sizes[i]);
  }
  std::printf("\n");
  // Height histogram of the traversable cells: on a multi-floor map this is
  // where the floors are (and what a goal's z has to be near).
  std::map<int, size_t> hist;
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.traversable(c)) {
      ++hist[static_cast<int>(std::floor(m.zTop(c) / 0.5))];
    }
  }
  std::printf("traversable cells by height (0.5 m bins):\n");
  for (const auto & [bin, n] : hist) {
    if (n * 1000 >= s.traversable) {
      std::printf("  z %6.1f .. %6.1f : %zu\n", bin * 0.5, bin * 0.5 + 0.5, n);
    }
  }
  std::printf(
    "build: columns %.2fs, candidates %.2fs, hole-fill %.2fs, step %.2fs, footprint %.2fs, "
    "components %.2fs, clearance %.2fs, total %.2fs\n",
    s.t_columns, s.t_candidates, s.t_hole_fill, s.t_step, s.t_footprint, s.t_components,
    s.t_clearance, s.t_total);
}

// Recomputes N random columns the slow way (OcTree::search per voxel) and
// compares the candidate surfaces and their unknown counts with the map's:
// a mismatch is a key-arithmetic or pruned-block bug in fillColumnMasks.
int selfTest(const octomap::OcTree & tree, const SurfaceMap & m, size_t n)
{
  std::mt19937 rng(12345);
  std::uniform_int_distribution<int> dx(0, m.nx - 1), dy(0, m.ny - 1);
  const int C = static_cast<int>(std::ceil(m.params.body_clearance / m.res - 1e-9));
  size_t mismatches = 0, cells_checked = 0;
  std::vector<uint8_t> occ(m.nz + 1), known(m.nz + 1);
  for (size_t t = 0; t < n; ++t) {
    const int ix = dx(rng), iy = dy(rng);
    const double x = m.cx(ix), y = m.cy(iy);
    for (int kz = 0; kz <= m.nz; ++kz) {
      const auto * node =
        kz < m.nz ? tree.search(octomap::point3d(x, y, m.z0 + (kz + 0.5) * m.res)) : nullptr;
      known[kz] = node != nullptr;
      occ[kz] = node != nullptr && tree.isNodeOccupied(node);
    }
    std::vector<std::pair<int, int>> expect;  // (kz, unknown in band)
    for (int kz = 0; kz < m.nz; ++kz) {
      if (occ[kz] && !occ[kz + 1]) {
        int kn = 0;
        for (int z = kz + 1; z < std::min(kz + 1 + C, m.nz); ++z) {
          kn += known[z];
        }
        expect.emplace_back(kz, C - kn);
      }
    }
    std::vector<std::pair<int, int>> got;
    const uint32_t c = m.col(ix, iy);
    for (uint32_t i = m.col_begin[c]; i < m.col_begin[c + 1]; ++i) {
      if (!m.inferred(i)) {
        got.emplace_back(m.cells[i].kz, m.cells[i].unknown_cnt);
      }
    }
    cells_checked += expect.size();
    if (expect != got) {
      if (++mismatches <= 5) {
        std::printf(
          "self-test mismatch at column (%d, %d): expected %zu surfaces, map has %zu\n", ix, iy,
          expect.size(), got.size());
      }
    }
  }
  std::printf(
    "self-test: %zu columns, %zu surfaces, %zu mismatching columns\n", n, cells_checked,
    mismatches);
  return mismatches == 0 ? 0 : 1;
}

std::vector<size_t> checkPoses(
  const SurfaceMap & m, const std::vector<Pose> & poses, double lidar_height)
{
  // The ground under a keyframe is lidar_height below it; the stance height
  // varies by a few cm and the surface is quantised to res, hence the window.
  const double z_tol = 0.35;
  size_t on_traversable = 0, near_traversable = 0, on_surface = 0, off_map = 0;
  std::vector<size_t> misses;
  std::map<uint32_t, size_t> by_component;  // of the covered keyframes
  std::map<uint32_t, size_t> first_in_component;
  for (size_t i = 0; i < poses.size(); ++i) {
    const double zg = poses[i].z - lidar_height;
    const int ix = m.ixOf(poses[i].x), iy = m.iyOf(poses[i].y);
    if (!m.inGrid(ix, iy)) {
      ++off_map;
      continue;
    }
    bool trav = false, surf = false, near = false;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!m.inGrid(ix + dx, iy + dy)) {
          continue;
        }
        const uint32_t c = m.col(ix + dx, iy + dy);
        for (uint32_t k = m.col_begin[c]; k < m.col_begin[c + 1]; ++k) {
          if (std::fabs(m.zTop(k) - zg) > z_tol) {
            continue;
          }
          if (dx == 0 && dy == 0) {
            surf = true;
            trav = trav || m.traversable(k);
          }
          if (m.traversable(k)) {
            if (!near) {
              ++by_component[m.cells[k].component];
              first_in_component.emplace(m.cells[k].component, i);
            }
            near = true;
          }
        }
      }
    }
    on_traversable += trav;
    near_traversable += near;
    on_surface += surf;
    if (!near) {
      misses.push_back(i);
    }
  }
  const double n = static_cast<double>(poses.size());
  std::printf(
    "check-poses: %zu keyframes (lidar height %.3f): on a traversable cell %zu (%.1f %%), "
    "within one cell of one %zu (%.1f %%), on any surface %zu (%.1f %%), outside the grid %zu\n",
    poses.size(), lidar_height, on_traversable, 100 * on_traversable / n, near_traversable,
    100 * near_traversable / n, on_surface, 100 * on_surface / n, off_map);
  std::printf("  covered keyframes by component:");
  for (const auto & [comp, n] : by_component) {
    std::printf(" %u:%zu(first kf %zu)", comp, n, first_in_component[comp]);
  }
  std::printf("\n");
  for (size_t k = 0; k < std::min<size_t>(10, misses.size()); ++k) {
    const size_t i = misses[k];
    const int ix = m.ixOf(poses[i].x), iy = m.iyOf(poses[i].y);
    std::printf(
      "  keyframe %zu at (%.2f, %.2f, ground %.2f): %s\n", i, poses[i].x, poses[i].y,
      poses[i].z - lidar_height, m.describeColumn(m.col(ix, iy)).c_str());
  }
  return misses;
}

// One letter per column: the state of the layer nearest to z (within 1 m).
char stateLetter(const SurfaceMap & m, uint32_t c, double z, int * dkz = nullptr)
{
  const int kz = static_cast<int>(std::floor((z - m.z0) / m.res)) - 1;
  const CellId l = m.layerNear(c, kz, 10);
  if (l == kNoCell) {
    return ' ';
  }
  if (dkz) {
    *dkz = m.cells[l].kz - kz;
  }
  if (m.traversable(l)) {
    return m.inferred(l) ? 'i' : '.';
  }
  switch (static_cast<Reject>(m.cells[l].reject)) {
    case Reject::ClearanceOccupied:
      return 'O';
    case Reject::ClearanceUnknown:
      return 'U';
    case Reject::NoFreeAbove:
      return 'N';
    case Reject::Step:
      return 'S';
    case Reject::FootprintBlocked:
      return 'F';
    case Reject::FootprintGap:
      return 'G';
    case Reject::Slope:
      return 'L';
    case Reject::Residual:
      return 'R';
    default:
      return '?';
  }
}

void inspect(const SurfaceMap & m, double x, double y, double z)
{
  const int cx = m.ixOf(x), cy = m.iyOf(y), r = 7;
  std::printf(
    "inspect (%.2f, %.2f) at z %.2f -- state | kz offset from z (. traversable, i inferred, "
    "S step, F footprint, G footprint_gap, O clearance_occupied, N no_free_above, U "
    "clearance_unknown, L slope, "
    "R residual, blank = no layer within 1 m); y grows upward, x to the right\n",
    x, y, z);
  for (int iy = cy + r; iy >= cy - r; --iy) {
    std::string a, b;
    for (int ix = cx - r; ix <= cx + r; ++ix) {
      if (!m.inGrid(ix, iy)) {
        a += "# ";
        b += "## ";
        continue;
      }
      int d = 0;
      const char s = stateLetter(m, m.col(ix, iy), z, &d);
      a += s;
      a += (ix == cx && iy == cy) ? '*' : ' ';
      if (s == ' ') {
        b += "   ";
      } else {
        char buf[8];
        std::snprintf(buf, sizeof buf, "%+2d ", d);
        b += buf;
      }
    }
    std::printf("  %s | %s\n", a.c_str(), b.c_str());
  }
}

// 24-bit BMP, one pixel per column, y up. Converted with e.g. sips -s format png.
struct ImageRegion
{
  int ix0 = 0, iy0 = 0, ix1 = 0, iy1 = 0;  // cell range, exclusive end
  int scale = 1;
  bool by_component = false;
};

// keepout: the map's keepout mask (--keepout) or null; its cores and rings
// on the layer nearest z are drawn over the state colours.
bool dumpImage(
  const SurfaceMap & m, double z, const std::vector<Pose> & poses,
  const std::vector<PlannedPath> & paths, const ImageRegion & reg, const KeepoutMask * keepout,
  const std::string & path)
{
  const int W = (reg.ix1 - reg.ix0) * reg.scale, H = (reg.iy1 - reg.iy0) * reg.scale;
  const int row = (W * 3 + 3) & ~3;
  if (W <= 0 || H <= 0) {
    return false;
  }
  std::vector<uint8_t> img(static_cast<size_t>(row) * H, 0);
  auto put = [&](int ix, int iy, uint8_t r, uint8_t g, uint8_t b) {
    if (ix < reg.ix0 || iy < reg.iy0 || ix >= reg.ix1 || iy >= reg.iy1) {
      return;
    }
    for (int sy = 0; sy < reg.scale; ++sy) {
      for (int sx = 0; sx < reg.scale; ++sx) {
        uint8_t * p = &img
                        [static_cast<size_t>((iy - reg.iy0) * reg.scale + sy) * row +
                         ((ix - reg.ix0) * reg.scale + sx) * 3];
        p[0] = b;
        p[1] = g;
        p[2] = r;
      }
    }
  };
  for (int iy = reg.iy0; iy < reg.iy1; ++iy) {
    for (int ix = reg.ix0; ix < reg.ix1; ++ix) {
      if (reg.by_component) {
        const int kz = static_cast<int>(std::floor((z - m.z0) / m.res)) - 1;
        const CellId l = m.layerNear(m.col(ix, iy), kz, 10);
        if (l != kNoCell && m.traversable(l)) {
          // A hash of the id into a bright colour; neighbouring ids differ.
          const uint32_t h = m.cells[l].component * 2654435761u;
          put(ix, iy, 80 + (h & 127), 80 + ((h >> 8) & 127), 80 + ((h >> 16) & 127));
        } else if (l != kNoCell) {
          put(ix, iy, 40, 40, 40);
        }
        continue;
      }
      switch (stateLetter(m, m.col(ix, iy), z)) {
        case '.':
          put(ix, iy, 40, 190, 40);
          break;
        case 'i':
          put(ix, iy, 40, 220, 220);
          break;
        case 'S':
          put(ix, iy, 230, 40, 40);
          break;
        case 'F':
          put(ix, iy, 255, 150, 0);
          break;
        case 'O':
          put(ix, iy, 110, 110, 110);
          break;
        case 'N':
          put(ix, iy, 70, 70, 140);
          break;
        case 'U':
          put(ix, iy, 40, 40, 200);
          break;
        case 'L':
          put(ix, iy, 220, 40, 220);
          break;
        case 'R':
          put(ix, iy, 150, 40, 150);
          break;
        default:
          break;
      }
    }
  }
  if (keepout) {
    const int kz = static_cast<int>(std::floor((z - m.z0) / m.res)) - 1;
    for (int iy = reg.iy0; iy < reg.iy1; ++iy) {
      for (int ix = reg.ix0; ix < reg.ix1; ++ix) {
        const CellId l = m.layerNear(m.col(ix, iy), kz, 10);
        if (l == kNoCell) {
          continue;
        }
        if (keepout->core_(l)) {
          put(ix, iy, 200, 30, 60);
        } else if (keepout->d[l] <= keepout->block_radius) {
          put(ix, iy, 120, 30, 40);
        }
      }
    }
  }
  for (const Pose & p : poses) {
    const int ix = m.ixOf(p.x), iy = m.iyOf(p.y);
    if (m.inGrid(ix, iy) && std::fabs(p.z - z) < 1.5) {
      put(ix, iy, 255, 255, 255);
    }
  }
  // Planned paths (3 px wide, blue), start (magenta) and goal (yellow) 5 px.
  auto blob = [&](double x, double y, int r, uint8_t cr, uint8_t cg, uint8_t cb) {
    const int ix = m.ixOf(x), iy = m.iyOf(y);
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (m.inGrid(ix + dx, iy + dy)) {
          put(ix + dx, iy + dy, cr, cg, cb);
        }
      }
    }
  };
  for (const PlannedPath & pp : paths) {
    for (const Waypoint & w : pp.pts) {
      if (std::fabs(w.z - z) < 1.5) {
        blob(w.x, w.y, 1, 60, 120, 255);
      }
    }
    if (std::fabs(pp.start.z - z) < 1.5) {
      blob(pp.start.x, pp.start.y, 2, 255, 0, 255);
    }
    if (std::fabs(pp.goal.z - z) < 1.5) {
      blob(pp.goal.x, pp.goal.y, 2, 255, 230, 0);
    }
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  const uint32_t size = 54 + static_cast<uint32_t>(img.size());
  uint8_t h[54] = {'B', 'M'};
  auto u32 = [&](int off, uint32_t v) {
    h[off] = v & 255;
    h[off + 1] = (v >> 8) & 255;
    h[off + 2] = (v >> 16) & 255;
    h[off + 3] = v >> 24;
  };
  u32(2, size);
  u32(10, 54);
  u32(14, 40);
  u32(18, W);
  u32(22, H);
  h[26] = 1;
  h[28] = 24;
  u32(34, static_cast<uint32_t>(img.size()));
  out.write(reinterpret_cast<const char *>(h), 54);
  out.write(reinterpret_cast<const char *>(img.data()), img.size());
  return static_cast<bool>(out);
}

// Replays the hole-fill decision for one column (the map's CSR for the
// neighbours, the tree for the column's own occupancy) and prints every
// condition, for when --inspect shows a hole that should have been filled.
void debugFill(const octomap::OcTree & tree, const SurfaceMap & m, double x, double y, double z)
{
  const int ix = m.ixOf(x), iy = m.iyOf(y);
  if (!m.inGrid(ix, iy)) {
    std::printf("debug-fill: outside the grid\n");
    return;
  }
  const uint32_t c = m.col(ix, iy);
  const int r = static_cast<int>(std::ceil(m.params.hole_fill_radius / m.res));
  std::vector<std::pair<int, int>> gathered;
  int disc_n = 0;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if ((dx == 0 && dy == 0) || std::hypot(dx, dy) * m.res > m.params.hole_fill_radius + 1e-9) {
        continue;
      }
      ++disc_n;
      if (!m.inGrid(ix + dx, iy + dy)) {
        continue;
      }
      const int q = (dx >= 0 && dy > 0) ? 0 : (dx < 0 && dy >= 0) ? 1 : (dx <= 0 && dy < 0) ? 2 : 3;
      const uint32_t nc = m.col(ix + dx, iy + dy);
      for (uint32_t i = m.col_begin[nc]; i < m.col_begin[nc + 1]; ++i) {
        const Reject rj = static_cast<Reject>(m.cells[i].reject);
        // At fill time only the clearance rejects exist; everything else was decided later.
        if (
          rj == Reject::None || rj == Reject::Step || rj == Reject::FootprintBlocked ||
          rj == Reject::FootprintGap || rj == Reject::Slope || rj == Reject::Residual) {
          gathered.emplace_back(m.cells[i].kz, q);
        }
      }
    }
  }
  std::sort(gathered.begin(), gathered.end());
  const int step = m.step_cells;
  std::printf(
    "debug-fill (%.2f, %.2f) column (%d, %d), reference z %.2f (kz %d): %s\n", x, y, ix, iy, z,
    static_cast<int>(std::floor((z - m.z0) / m.res)) - 1, m.describeColumn(c).c_str());
  std::printf("  gathered %zu layers from %d neighbours\n", gathered.size(), disc_n);
  // Every height group the neighbours know about, judged as holeFillPass
  // judges it (one group per floor; the largest alone is the wrong one to
  // look at on a multi-storey map -- under the robot's own track it is the
  // complete floor, and the hole is in the other).
  const int C = static_cast<int>(std::ceil(m.params.body_clearance / m.res - 1e-9));
  for (size_t lo = 0; lo < gathered.size();) {
    size_t hi = lo + 1;
    while (hi < gathered.size() && gathered[hi].first - gathered[hi - 1].first <= step) {
      ++hi;
    }
    bool quad[4] = {false, false, false, false};
    for (size_t i = lo; i < hi; ++i) {
      quad[gathered[i].second] = true;
    }
    const int v = gathered[(lo + hi) / 2].first;
    const double support = disc_n ? static_cast<double>(hi - lo) / disc_n : 0.0;
    const bool has_layer = m.layerNear(c, v, step + 1) != kNoCell;
    std::string occ;
    bool occupied_near = false;
    for (int kz = v - 3; kz < v + 1 + C + 2; ++kz) {
      const auto * n = tree.search(octomap::point3d(m.cx(ix), m.cy(iy), m.z0 + (kz + 0.5) * m.res));
      const bool o = n != nullptr && tree.isNodeOccupied(n);
      occ += n == nullptr ? '?' : (o ? '#' : '.');
      if (o && kz >= v - 1 && kz < v + 1 + C) {
        occupied_near = true;
      }
    }
    const char * verdict =
      support < m.params.hole_fill_min_support    ? "no: support too low"
      : quad[0] + quad[1] + quad[2] + quad[3] < 3 ? "no: one-sided (a floor edge)"
      : has_layer                                 ? "no: the column already has a layer there"
      : occupied_near ? "no: an occupied voxel near that height (something was seen here)"
                      : "FILL";
    std::printf(
      "  group at kz %d (z top %.2f): %zu layers, support %.2f (need %.2f), quadrants %d%d%d%d, "
      "existing layer within +-%d: %s -> %s\n",
      v, m.z0 + (v + 1) * m.res, hi - lo, support, m.params.hole_fill_min_support, quad[0], quad[1],
      quad[2], quad[3], step + 1, has_layer ? "yes" : "none", verdict);
    std::printf(
      "    column voxels kz %d..%d (? unknown, . free, # occupied): %s\n", v - 3, v + C + 2,
      occ.c_str());
    lo = hi;
  }
}

bool dumpPcd(const SurfaceMap & m, const std::string & path)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "# .PCD v0.7 - surface map of OctomapPlanner3D\n"
         "VERSION 0.7\n"
         "FIELDS x y z label reject unknown clearance component\n"
         "SIZE 4 4 4 4 4 4 4 4\n"
         "TYPE F F F I I I F I\n"
         "COUNT 1 1 1 1 1 1 1 1\n"
         "WIDTH "
      << m.cells.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << m.cells.size()
      << "\nDATA ascii\n";
  char line[160];
  for (CellId c = 0; c < m.cells.size(); ++c) {
    double x, y;
    m.cellXY(c, x, y);
    const int label = m.traversable(c) ? (m.inferred(c) ? 2 : 1) : 0;
    std::snprintf(
      line, sizeof line, "%.3f %.3f %.3f %d %d %d %.3f %d\n", x, y, m.zTop(c), label,
      m.cells[c].reject, m.cells[c].unknown_cnt, m.cells[c].clearance_d,
      m.cells[c].component == kNoComponent ? -1 : static_cast<int>(m.cells[c].component));
    out << line;
  }
  return static_cast<bool>(out);
}

// mask: the dynamic overlay's snapshot (--dynamic), or null; keepout: the
// map's keepout mask (--keepout), or null. Read exactly as the plugin reads
// them: the goal snap, the search and the shortcut keep off both, with the
// disc round the start exempt from the dilation, and a start inside a zone's
// core is refused.
int runPlan(
  SurfaceAStar & astar, const SearchParams & sp, double sx, double sy, double sz, double gx,
  double gy, double gz, bool shortcut, const std::string & out_path,
  std::vector<PlannedPath> * planned, const DynamicMask * mask, const KeepoutMask * keepout,
  const std::vector<KeepoutZone> & zones)
{
  const SurfaceMap & m = astar.map();
  const auto t0 = std::chrono::steady_clock::now();
  const SnapResult s = astar.snap(sx, sy, sz, sp);
  if (s.cell == kNoCell) {
    std::printf(
      "plan: %s: %s\n",
      planFailureName(s.outside ? PlanFailure::StartOutsideMap : PlanFailure::StartUnavailable),
      s.detail.c_str());
    return 1;
  }
  if (keepout && keepout->core_(s.cell)) {
    const KeepoutZone * z = zoneAt(zones, m, s.cell);
    std::printf(
      "plan: %s: the start stands inside keepout zone %s\n",
      planFailureName(PlanFailure::StartInKeepout), z ? zoneLabel(*z).c_str() : "?");
    return 1;
  }
  BlockedView blocked;
  blocked.mask = mask;
  blocked.keepout = keepout;
  blocked.exempt = s.cell;
  blocked.exempt_radius =
    std::max(mask ? mask->block_radius : 0.0, keepout ? keepout->block_radius : 0.0);
  const SnapResult g = astar.snap(gx, gy, gz, sp, m.cells[s.cell].component, &blocked);
  if (g.cell == kNoCell) {
    std::printf(
      "plan: %s: %s\n",
      planFailureName(g.outside ? PlanFailure::GoalOutsideMap : PlanFailure::GoalUnavailable),
      g.detail.c_str());
    return 1;
  }
  double sxc, syc, gxc, gyc;
  m.cellXY(s.cell, sxc, syc);
  m.cellXY(g.cell, gxc, gyc);
  std::printf(
    "plan: start snapped to (%.2f, %.2f, %.2f) [%.2f m xy, %+.2f m z], goal to "
    "(%.2f, %.2f, %.2f) [%.2f m xy, %+.2f m z%s], component %u\n",
    sxc, syc, m.zTop(s.cell), s.xy_dist, s.dz, gxc, gyc, m.zTop(g.cell), g.xy_dist, g.dz,
    g.z_fallback ? ", single layer" : "", m.cells[s.cell].component);
  PlanResult r = astar.plan(s.cell, g.cell, sp, blocked);
  if (r.failure != PlanFailure::None) {
    std::printf(
      "plan: %s: %s (%zu iterations, %.3f s)\n", planFailureName(r.failure), r.detail.c_str(),
      r.iterations, r.search_s);
    return 1;
  }
  const size_t raw_cells = r.cells.size();
  double raw_len = 0;
  for (size_t i = 1; i < r.cells.size(); ++i) {
    double ax, ay, bx, by;
    m.cellXY(r.cells[i - 1], ax, ay);
    m.cellXY(r.cells[i], bx, by);
    raw_len += std::hypot(bx - ax, by - ay);
  }
  size_t vertices = 0;
  const auto path =
    buildPath(m, astar.links(), r.cells, r.links, shortcut, 0.1, &vertices, &blocked);
  if (planned) {
    PlannedPath pp;
    pp.start = Waypoint{sxc, syc, m.zTop(s.cell)};
    pp.goal = Waypoint{gxc, gyc, m.zTop(g.cell)};
    pp.pts = path;
    char label[128];
    std::snprintf(
      label, sizeof label, "plan %zu: (%.1f, %.1f) -> (%.1f, %.1f)", planned->size() + 1, sxc, syc,
      gxc, gyc);
    pp.label = label;
    planned->push_back(std::move(pp));
  }
  double len = 0, min_clear = std::numeric_limits<double>::infinity(), zmin = path[0].z,
         zmax = path[0].z;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    zmin = std::min(zmin, path[i].z);
    zmax = std::max(zmax, path[i].z);
  }
  for (CellId c : r.cells) {
    min_clear = std::min<double>(min_clear, m.cells[c].clearance_d);
  }
  const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf(
    "plan: ok -- %zu cells, cost %.2f, %zu iterations, search %.3f s, total %.3f s; "
    "shortcut to %zu vertices; %zu poses, %.2f m (raw %.2f m), z %.2f..%.2f, min clearance %.2f "
    "m, %zu stair links, %zu dynamic marks, %zu keepout cells\n",
    raw_cells, r.cost, r.iterations, r.search_s, total, vertices, path.size(), len, raw_len, zmin,
    zmax, min_clear, r.links.size(), mask ? mask->marked : size_t{0},
    keepout ? keepout->blocked : size_t{0});
  for (const LinkUse & u : r.links) {
    const StairLink & l = astar.links()->links[u.link];
    std::printf(
      "plan:   stairs %s: z %.2f -> %.2f, %.1f m\n", linkName(l, u.reversed ? "<-" : "->").c_str(),
      m.zTop(u.reversed ? l.b : l.a), m.zTop(u.reversed ? l.a : l.b), l.length);
  }
  if (!out_path.empty()) {
    std::ofstream out(out_path);
    out << "x,y,z\n";
    for (const Waypoint & w : path) {
      out << w.x << ',' << w.y << ',' << w.z << '\n';
    }
    std::printf("wrote %zu poses to %s\n", path.size(), out_path.c_str());
  }
  return 0;
}

std::string base64(const uint8_t * data, size_t n)
{
  static const char * T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((n + 2) / 3 * 4);
  for (size_t i = 0; i < n; i += 3) {
    const uint32_t v = (static_cast<uint32_t>(data[i]) << 16) |
                       (i + 1 < n ? static_cast<uint32_t>(data[i + 1]) << 8 : 0u) |
                       (i + 2 < n ? static_cast<uint32_t>(data[i + 2]) : 0u);
    out += T[(v >> 18) & 63];
    out += T[(v >> 12) & 63];
    out += i + 1 < n ? T[(v >> 6) & 63] : '=';
    out += i + 2 < n ? T[v & 63] : '=';
  }
  return out;
}

void replaceAll(std::string & s, const std::string & from, const std::string & to)
{
  for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size()) {
    s.replace(pos, from.size(), to);
  }
}

// A self-contained three.js page: the traversable cells coloured by height
// (inferred ones tinted), a subsample of the rejected cells in grey for the
// walls, and every planned path as a tube with its start and goal. Points are
// sorted by z so the "hide above z" slider is a draw range, which is what
// makes a two-storey map readable floor by floor. ~5 MB for a site map.
// `links` may be null (no --poses, or --no-stair-links). They are drawn on
// their own because the stairwell itself is all rejected cells -- a flight
// fails the step and slope rules by design -- so without them the only trace
// of a stair in the viewer is a planned route that happens to take one.
bool dumpHtml(
  const SurfaceMap & m, const BuildStats & stats, const std::vector<PlannedPath> & paths,
  const StairLinks * links, const std::string & title, const std::string & path)
{
  constexpr int kRejectedStep = 3;
  std::vector<CellId> trav, rej;
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.traversable(c)) {
      trav.push_back(c);
    } else if (rej.size() * kRejectedStep <= c) {
      rej.push_back(c);
    }
  }
  const auto by_z = [&m](CellId a, CellId b) { return m.cells[a].kz < m.cells[b].kz; };
  std::sort(trav.begin(), trav.end(), by_z);
  std::sort(rej.begin(), rej.end(), by_z);
  double xmin = 1e30, xmax = -1e30, ymin = 1e30, ymax = -1e30, zmin = 1e30, zmax = -1e30;
  size_t inferred = 0;
  const auto pack = [&](const std::vector<CellId> & ids, std::vector<float> & pos) {
    pos.reserve(ids.size() * 3);
    for (CellId c : ids) {
      double x, y;
      m.cellXY(c, x, y);
      const double z = m.zTop(c);
      pos.push_back(static_cast<float>(x));
      pos.push_back(static_cast<float>(y));
      pos.push_back(static_cast<float>(z));
      xmin = std::min(xmin, x);
      xmax = std::max(xmax, x);
      ymin = std::min(ymin, y);
      ymax = std::max(ymax, y);
      zmin = std::min(zmin, z);
      zmax = std::max(zmax, z);
    }
  };
  std::vector<float> trav_pos, rej_pos;
  pack(trav, trav_pos);
  const double tz_min = zmin, tz_max = zmax;  // the ramp spans the traversable cells only
  pack(rej, rej_pos);
  std::vector<uint8_t> flags;
  flags.reserve(trav.size());
  for (CellId c : trav) {
    flags.push_back(m.inferred(c) ? 1 : 0);
    inferred += m.inferred(c);
  }
  if (trav.empty()) {
    xmin = ymin = zmin = 0;
    xmax = ymax = zmax = 1;
  }

  std::ostringstream data;
  char num[64];
  auto f = [&](double v) {
    std::snprintf(num, sizeof num, "%.3f", v);
    return std::string(num);
  };
  data << "{\"res\":" << f(m.res) << ",\"xmin\":" << f(xmin) << ",\"xmax\":" << f(xmax)
       << ",\"ymin\":" << f(ymin) << ",\"ymax\":" << f(ymax) << ",\"zmin\":" << f(tz_min)
       << ",\"zmax\":" << f(tz_max) << ",\"trav\":\""
       << base64(reinterpret_cast<const uint8_t *>(trav_pos.data()), trav_pos.size() * 4)
       << "\",\"flags\":\"" << base64(flags.data(), flags.size()) << "\",\"rej\":\""
       << base64(reinterpret_cast<const uint8_t *>(rej_pos.data()), rej_pos.size() * 4)
       << "\",\"paths\":[";
  for (size_t i = 0; i < paths.size(); ++i) {
    const PlannedPath & pp = paths[i];
    double pz = 1e30;
    for (const Waypoint & w : pp.pts) {
      pz = std::min(pz, w.z);
    }
    data << (i ? "," : "") << "{\"label\":\"" << pp.label << "\",\"zmin\":" << f(pz)
         << ",\"start\":[" << f(pp.start.x) << "," << f(pp.start.y) << "," << f(pp.start.z)
         << "],\"goal\":[" << f(pp.goal.x) << "," << f(pp.goal.y) << "," << f(pp.goal.z)
         << "],\"pts\":[";
    for (size_t k = 0; k < pp.pts.size(); ++k) {
      data << (k ? "," : "") << "[" << f(pp.pts[k].x) << "," << f(pp.pts[k].y) << ","
           << f(pp.pts[k].z) << "]";
    }
    data << "]}";
  }
  data << "],\"links\":[";
  if (links) {
    for (size_t i = 0; i < links->links.size(); ++i) {
      const StairLink & l = links->links[i];
      double lz = 1e30;
      for (const Waypoint & w : l.pts) {
        lz = std::min(lz, w.z);
      }
      char label[160];
      std::snprintf(
        label, sizeof label, "stairs %s: z %.2f - %.2f, %.1f m", linkName(l, "-").c_str(),
        m.zTop(l.a), m.zTop(l.b), l.length);
      data << (i ? "," : "") << "{\"label\":\"" << label << "\",\"zmin\":" << f(lz) << ",\"pts\":[";
      for (size_t k = 0; k < l.pts.size(); ++k) {
        data << (k ? "," : "") << "[" << f(l.pts[k].x) << "," << f(l.pts[k].y) << ","
             << f(l.pts[k].z) << "]";
      }
      data << "]}";
    }
  }
  data << "]}";

  std::string html = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{{TITLE}}</title>
<style>
  html, body { margin: 0; height: 100%; background: #16161a; color: #ececee;
    font: 13px/1.45 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif; overflow: hidden; }
  #view { position: fixed; inset: 0; }
  .panel { position: fixed; top: 16px; left: 16px; width: min(320px, calc(100vw - 32px)); box-sizing: border-box;
    background: rgba(28,28,32,.92); border: 1px solid #34343a; border-radius: 10px; padding: 12px 14px;
    backdrop-filter: blur(6px); max-height: calc(100vh - 32px); overflow: auto; }
  h1 { font-size: 14px; margin: 0 0 2px; }
  .muted { color: #9a9aa2; }
  .stats { display: grid; grid-template-columns: auto 1fr; gap: 1px 10px; margin: 8px 0 10px; font-variant-numeric: tabular-nums; }
  .stats dd { margin: 0; text-align: right; }
  label { display: block; margin-top: 8px; }
  .row { display: flex; justify-content: space-between; font-variant-numeric: tabular-nums; }
  input[type=range] { width: 100%; }
  .ramp { height: 8px; border-radius: 4px; margin-top: 10px; background: linear-gradient(90deg, #2c3e8f, #1f8a9e, #3fae5a, #e3b23c, #d1495b); }
  .ramp-labels { display: flex; justify-content: space-between; font-variant-numeric: tabular-nums; }
  .paths { margin-top: 10px; }
  .paths div { display: flex; align-items: center; gap: 6px; margin-top: 4px; }
  .sw { width: 10px; height: 10px; border-radius: 2px; display: inline-block; flex: none; }
  .legend { display: flex; gap: 10px; margin-top: 8px; }
  .legend span { display: flex; align-items: center; gap: 4px; }
</style>
<script type="importmap">
{ "imports": {
  "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
  "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/" } }
</script>
</head>
<body>
<div id="view"></div>
<div class="panel">
  <h1>{{TITLE}}</h1>
  <div class="muted">OctomapPlanner3D surface map &middot; drag to orbit, scroll to zoom, right-drag to pan</div>
  <dl class="stats">
    <dt class="muted">resolution</dt><dd>{{RES}} m</dd>
    <dt class="muted">traversable cells</dt><dd>{{N_TRAV}}</dd>
    <dt class="muted">of which inferred</dt><dd>{{N_INF}}</dd>
    <dt class="muted">rejected cells shown</dt><dd>{{N_REJ}} of {{N_REJ_ALL}}</dd>
    <dt class="muted">components</dt><dd>{{N_COMP}}</dd>
  </dl>
  <div class="ramp"></div>
  <div class="ramp-labels muted"><span>floor z {{ZMIN}} m</span><span>{{ZMAX}} m</span></div>
  <div class="legend muted"><span><span class="sw" style="background:#595959"></span>rejected</span><span><span class="sw" style="background:#3fae5a"></span>start</span><span><span class="sw" style="background:#d1495b"></span>goal</span><span><span class="sw" style="background:#ff2bd6"></span>stair link</span></div>
  <label><input type="checkbox" id="rej" checked> rejected cells</label>
  <label><input type="checkbox" id="lnk" checked> stair links ({{N_LINKS}})</label>
  <label><input type="checkbox" id="inf" checked> tint inferred (hole-filled) cells</label>
  <label><div class="row"><span>point size</span><span id="sizev">0.10</span></div><input type="range" id="size" min="0.05" max="0.40" step="0.01" value="0.10"></label>
  <label><div class="row"><span>hide ground above z</span><span id="zcutv"></span></div><input type="range" id="zcut" min="0" max="1000" value="1000"></label>
  <div class="paths" id="links"></div>
  <div class="paths" id="paths"></div>
</div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const D = {{DATA}};
function bytes(b64) { const bin = atob(b64); const u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return u; }
const trav = new Float32Array(bytes(D.trav).buffer), flags = bytes(D.flags), rej = new Float32Array(bytes(D.rej).buffer);
const stops = [[0x2c,0x3e,0x8f],[0x1f,0x8a,0x9e],[0x3f,0xae,0x5a],[0xe3,0xb2,0x3c],[0xd1,0x49,0x5b]];
function ramp(t) { t = Math.min(1, Math.max(0, t)) * (stops.length - 1); const i = Math.min(stops.length - 2, Math.floor(t)); const f = t - i; return [0, 1, 2].map(k => (stops[i][k] + (stops[i + 1][k] - stops[i][k]) * f) / 255); }
const zspan = Math.max(1e-6, D.zmax - D.zmin);
function colorsFor(pos, fl, tint) { const c = new Float32Array(pos.length); for (let i = 0; i < pos.length / 3; i++) { let [r, g, b] = ramp((pos[3 * i + 2] - D.zmin) / zspan); if (tint && fl && fl[i]) { r = r * 0.4 + 0.6; g = g * 0.4 + 0.6; b = b * 0.4 + 0.6; } c[3 * i] = r; c[3 * i + 1] = g; c[3 * i + 2] = b; } return c; }
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x16161a);
const renderer = new THREE.WebGLRenderer({ antialias: true }); renderer.setPixelRatio(Math.min(2, devicePixelRatio)); renderer.setSize(innerWidth, innerHeight); document.getElementById('view').appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.1, 10000); camera.up.set(0, 0, 1);
const cx = (D.xmin + D.xmax) / 2, cy = (D.ymin + D.ymax) / 2, span = Math.max(1, D.xmax - D.xmin, D.ymax - D.ymin);
camera.position.set(cx, cy - span * 0.55, D.zmax + span * 0.6);
const controls = new OrbitControls(camera, renderer.domElement); controls.target.set(cx, cy, (D.zmin + D.zmax) / 2); controls.update();
function points(pos, colors, size, opacity) { const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(colors, 3)); const mat = new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true, transparent: opacity < 1, opacity }); return new THREE.Points(g, mat); }
const travPts = points(trav, colorsFor(trav, flags, true), 0.1, 1); scene.add(travPts);
const rejPts = points(rej, new Float32Array(rej.length).fill(0.35), 0.07, 0.7); scene.add(rejPts);
const palette = [0x4ea3ff, 0xff7f50, 0xc77dff, 0xffd166, 0x06d6a0, 0xff5c8a];
const pathObjs = [], pathChecks = [];
D.paths.forEach((p, i) => {
  const col = palette[i % palette.length];
  const grp = new THREE.Group();
  const pts = p.pts.map(q => new THREE.Vector3(q[0], q[1], q[2] + 0.05));
  if (pts.length > 1) { const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); grp.add(new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(8, pts.length * 2), 0.06, 6, false), new THREE.MeshBasicMaterial({ color: col }))); }
  const sphere = (q, c) => { const mm = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 12), new THREE.MeshBasicMaterial({ color: c })); mm.position.set(q[0], q[1], q[2] + 0.1); return mm; };
  grp.add(sphere(p.start, 0x3fae5a)); grp.add(sphere(p.goal, 0xd1495b));
  scene.add(grp); pathObjs.push(grp);
  const el = document.createElement('div');
  const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = true; cb.onchange = applyZ;
  const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = '#' + col.toString(16).padStart(6, '0');
  const tx = document.createElement('span'); tx.textContent = p.label + ' (' + p.pts.length + ' poses)';
  el.append(cb, sw, tx); document.getElementById('paths').appendChild(el); pathChecks.push(cb);
});
// Stair links: a thick magenta tube with a cube on each anchor, drawn above
// the paths' tubes so a route that takes one does not hide it.
const linkObjs = [];
D.links.forEach(l => {
  const grp = new THREE.Group();
  const pts = l.pts.map(q => new THREE.Vector3(q[0], q[1], q[2] + 0.08));
  if (pts.length > 1) { const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); grp.add(new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(8, pts.length * 3), 0.12, 8, false), new THREE.MeshBasicMaterial({ color: 0xff2bd6, transparent: true, opacity: 0.85 }))); }
  for (const q of [l.pts[0], l.pts[l.pts.length - 1]]) { const mm = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.4, 0.4), new THREE.MeshBasicMaterial({ color: 0xff2bd6 })); mm.position.set(q[0], q[1], q[2] + 0.2); grp.add(mm); }
  scene.add(grp); linkObjs.push(grp);
  const el = document.createElement('div');
  const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = '#ff2bd6';
  const tx = document.createElement('span'); tx.textContent = l.label;
  el.append(sw, tx); document.getElementById('links').appendChild(el);
});
function upperBound(pos, z) { let lo = 0, hi = pos.length / 3; while (lo < hi) { const mid = (lo + hi) >> 1; if (pos[3 * mid + 2] <= z) lo = mid + 1; else hi = mid; } return lo; }
const lnk = document.getElementById('lnk'); lnk.onchange = applyZ;
const zcut = document.getElementById('zcut'), zcutv = document.getElementById('zcutv');
function applyZ() { const z = D.zmin + zspan * zcut.value / 1000 + 0.01; zcutv.textContent = z.toFixed(2) + ' m'; travPts.geometry.setDrawRange(0, upperBound(trav, z)); rejPts.geometry.setDrawRange(0, upperBound(rej, z)); pathObjs.forEach((g, i) => { g.visible = pathChecks[i].checked && D.paths[i].zmin <= z; }); linkObjs.forEach((g, i) => { g.visible = lnk.checked && D.links[i].zmin <= z; }); }
zcut.oninput = applyZ; applyZ();
document.getElementById('rej').onchange = e => { rejPts.visible = e.target.checked; };
document.getElementById('inf').onchange = e => { travPts.geometry.setAttribute('color', new THREE.BufferAttribute(colorsFor(trav, flags, e.target.checked), 3)); };
const size = document.getElementById('size'), sizev = document.getElementById('sizev');
size.oninput = () => { travPts.material.size = +size.value; rejPts.material.size = +size.value * 0.7; sizev.textContent = (+size.value).toFixed(2); };
addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script>
</body>
</html>
)HTML";
  size_t rejected_all = 0;
  for (size_t r = 1; r < static_cast<size_t>(Reject::Count_); ++r) {
    rejected_all += stats.rejected[r];
  }
  replaceAll(html, "{{TITLE}}", title);
  replaceAll(html, "{{RES}}", f(m.res));
  replaceAll(html, "{{N_TRAV}}", std::to_string(trav.size()));
  replaceAll(html, "{{N_INF}}", std::to_string(inferred));
  replaceAll(html, "{{N_REJ}}", std::to_string(rej.size()));
  replaceAll(html, "{{N_REJ_ALL}}", std::to_string(rejected_all));
  replaceAll(html, "{{N_LINKS}}", std::to_string(links ? links->links.size() : 0));
  replaceAll(html, "{{N_COMP}}", std::to_string(stats.components));
  replaceAll(html, "{{ZMIN}}", f(tz_min));
  replaceAll(html, "{{ZMAX}}", f(tz_max));
  replaceAll(html, "{{DATA}}", data.str());
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << html;
  return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string bt_path, scene, poses_path, pcd_path, image_path;
  bool stats = false, self_test = false, check_poses = false, have_image_z = false,
       do_inspect = false;
  size_t self_test_n = 5000, inspect_misses = 0;
  double image_z = 0.0, inspect_x = 0.0, inspect_y = 0.0;
  std::vector<std::pair<double, double>> debug_fill;
  std::vector<size_t> inspect_kf;
  double region[4] = {0, 0, 0, 0};
  bool have_region = false, image_components = false;
  int image_scale = 1;
  double lidar_height = 0.481;
  SurfaceMapParams params;
  SearchParams search;
  StairLinkParams stair_params;
  bool stair_links = true;
  struct PlanReq
  {
    bool by_pose;
    double v[6];
  };
  std::vector<PlanReq> plans;
  std::vector<PlannedPath> planned;
  std::vector<Waypoint> people;  // --dynamic
  std::string keepout_path;      // --keepout
  std::string html_path;
  std::string out_path;
  std::string write_bt_path;
  bool shortcut = true;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * what) -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs an argument\n", what);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--synthetic") {
      scene = next("--synthetic");
    } else if (a == "--param") {
      const std::string kv = next("--param");
      if (!setParam(params, search, stair_params, kv)) {
        std::fprintf(stderr, "unknown parameter '%s'\n", kv.c_str());
        return 2;
      }
    } else if (a == "--stats") {
      stats = true;
    } else if (a == "--self-test") {
      self_test = true;
      if (i + 1 < argc && std::isdigit(argv[i + 1][0])) {
        self_test_n = std::strtoul(argv[++i], nullptr, 10);
      }
    } else if (a == "--poses") {
      poses_path = next("--poses");
    } else if (a == "--check-poses") {
      check_poses = true;
    } else if (a == "--lidar-height") {
      lidar_height = std::atof(next("--lidar-height").c_str());
    } else if (a == "--dump-pcd") {
      pcd_path = next("--dump-pcd");
    } else if (a == "--dump-html") {
      html_path = next("--dump-html");
    } else if (a == "--dump-image") {
      image_path = next("--dump-image");
    } else if (a == "--image-z") {
      image_z = std::atof(next("--image-z").c_str());
      have_image_z = true;
    } else if (a == "--inspect") {
      inspect_x = std::atof(next("--inspect").c_str());
      inspect_y = std::atof(next("--inspect").c_str());
      do_inspect = true;
    } else if (a == "--plan") {
      PlanReq r{false, {}};
      for (double & v : r.v) {
        v = std::atof(next("--plan").c_str());
      }
      plans.push_back(r);
    } else if (a == "--plan-poses") {
      PlanReq r{true, {}};
      r.v[0] = std::atof(next("--plan-poses").c_str());
      r.v[1] = std::atof(next("--plan-poses").c_str());
      plans.push_back(r);
    } else if (a == "--dynamic") {
      Waypoint w;
      w.x = std::atof(next("--dynamic").c_str());
      w.y = std::atof(next("--dynamic").c_str());
      w.z = std::atof(next("--dynamic").c_str());
      people.push_back(w);
    } else if (a == "--keepout") {
      keepout_path = next("--keepout");
    } else if (a == "--out") {
      out_path = next("--out");
    } else if (a == "--write-bt") {
      write_bt_path = next("--write-bt");
    } else if (a == "--no-shortcut") {
      shortcut = false;
    } else if (a == "--no-stair-links") {
      stair_links = false;
    } else if (a == "--inspect-kf") {
      inspect_kf.push_back(std::strtoul(next("--inspect-kf").c_str(), nullptr, 10));
    } else if (a == "--image-region") {
      for (double & v : region) {
        v = std::atof(next("--image-region").c_str());
      }
      have_region = true;
    } else if (a == "--image-scale") {
      image_scale = std::atoi(next("--image-scale").c_str());
    } else if (a == "--image-components") {
      image_components = true;
    } else if (a == "--debug-fill") {
      const double fx = std::atof(next("--debug-fill").c_str());
      const double fy = std::atof(next("--debug-fill").c_str());
      debug_fill.emplace_back(fx, fy);
    } else if (a == "--inspect-misses") {
      inspect_misses = 3;
      if (i + 1 < argc && std::isdigit(argv[i + 1][0])) {
        inspect_misses = std::strtoul(argv[++i], nullptr, 10);
      }
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else if (a[0] == '-') {
      std::fprintf(stderr, "unknown option '%s'\n", a.c_str());
      usage();
      return 2;
    } else {
      bt_path = a;
    }
  }
  if (bt_path.empty() == scene.empty()) {
    usage();
    return 2;
  }
  const bool any_action = stats || self_test || check_poses || !pcd_path.empty() ||
                          !image_path.empty() || do_inspect || !debug_fill.empty() ||
                          !plans.empty() || !html_path.empty() || !inspect_kf.empty();
  if (!any_action && write_bt_path.empty()) {
    stats = true;
  }

  std::unique_ptr<octomap::OcTree> tree;
  const auto t0 = std::chrono::steady_clock::now();
  if (!scene.empty()) {
    tree = synthetic::makeScene(scene);
    if (!tree) {
      std::fprintf(stderr, "unknown synthetic scene '%s'\n", scene.c_str());
      return 2;
    }
  } else {
    tree = std::make_unique<octomap::OcTree>(0.1);
    std::string error;
    if (!loadOctomapBt(bt_path, *tree, error)) {
      std::fprintf(stderr, "%s\n", error.c_str());
      return 1;
    }
  }
  std::printf(
    "tree: %zu nodes, res %.3f, loaded in %.2fs\n", tree->size(), tree->getResolution(),
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

  // A synthetic scene as a real map file: the CI smoke test boots the nav
  // session on one (a 3D-only map: octomap.bt, no gridmap.yaml), and a scene
  // can be handed to anything else that reads a .bt. Written before the build
  // and before anything else touches the tree; writeBinary() converts it to
  // maximum likelihood, which keeps every voxel's free / occupied state.
  if (!write_bt_path.empty()) {
    if (!tree->writeBinary(write_bt_path)) {
      std::fprintf(stderr, "cannot write '%s'\n", write_bt_path.c_str());
      return 1;
    }
    std::printf("wrote %s\n", write_bt_path.c_str());
    if (!any_action) {
      return 0;
    }
  }

  BuildStats build_stats;
  const auto map = buildSurfaceMap(*tree, params, &build_stats);
  if (stats) {
    printStats(*map, build_stats);
  }
  int rc = 0;
  if (self_test) {
    rc |= selfTest(*tree, *map, self_test_n);
  }
  for (const auto & [fx, fy] : debug_fill) {
    debugFill(*tree, *map, fx, fy, have_image_z ? image_z : 0.0);
  }

  std::vector<Pose> poses;
  if (!poses_path.empty() && !readPoses(poses_path, poses)) {
    std::fprintf(stderr, "cannot read poses '%s'\n", poses_path.c_str());
    return 1;
  }
  if (check_poses || inspect_misses > 0) {
    if (poses.empty()) {
      std::fprintf(stderr, "--check-poses needs --poses <poses.txt>\n");
      return 2;
    }
    const auto misses = checkPoses(*map, poses, lidar_height);
    for (size_t k = 0; k < std::min(inspect_misses, misses.size()); ++k) {
      const Pose & p = poses[misses[k]];
      inspect(*map, p.x, p.y, p.z - lidar_height);
    }
  }
  if (!have_image_z) {
    // Default to the lowest floor: the most populated 0.5 m bin of traversable cells.
    std::map<int, size_t> hist;
    for (CellId c = 0; c < map->cells.size(); ++c) {
      if (map->traversable(c)) {
        ++hist[static_cast<int>(std::floor(map->zTop(c) / 0.5))];
      }
    }
    size_t best = 0;
    for (const auto & [bin, n] : hist) {
      if (n > best) {
        best = n;
        image_z = bin * 0.5 + 0.25;
      }
    }
  }
  if (do_inspect) {
    inspect(*map, inspect_x, inspect_y, image_z);
  }
  for (size_t i : inspect_kf) {
    if (i >= poses.size()) {
      std::fprintf(stderr, "--inspect-kf %zu: no such keyframe\n", i);
      return 2;
    }
    std::printf("keyframe %zu:\n", i);
    inspect(*map, poses[i].x, poses[i].y, poses[i].z - lidar_height);
  }
  std::shared_ptr<const StairLinks> links;
  if (stair_links && (!poses.empty() || stair_params.detect)) {
    std::vector<Waypoint> track;
    track.reserve(poses.size());
    for (const Pose & p : poses) {
      track.push_back({p.x, p.y, p.z - lidar_height});
    }
    links = buildStairLinks(*map, track, stair_params, tree.get());
    std::printf(
      "stair links: %zu (min_rise %.2f m, %zu straight flights)\n", links->links.size(),
      stair_params.min_rise, links->flights.size());
    for (const StairFlight & f : links->flights) {
      const StairLink & l = links->links[f.first];
      std::printf(
        "  %s: z %.2f -> %.2f (component %u -> %u), %u straight links, uphill %.0f "
        "deg, %.1f m wide, %.1f m long\n",
        linkName(l).c_str(), map->zTop(l.a), map->zTop(l.b), map->cells[l.a].component,
        map->cells[l.b].component, f.count, f.heading_deg, f.width, l.length);
    }
    for (const StairLink & l : links->links) {
      if (l.straight) {
        continue;
      }
      std::printf(
        "  %s: z %.2f -> %.2f (component %u -> %u), %.1f m, %zu points (%s)\n", linkName(l).c_str(),
        map->zTop(l.a), map->zTop(l.b), map->cells[l.a].component, map->cells[l.b].component,
        l.length, l.pts.size(), l.flights ? l.geometry.c_str() : "walked track");
      if (l.centred > 0) {
        std::printf(
          "    centred: %zu points moved, mean %.2f m, max %.2f m; narrowest cross-section %.2f "
          "m\n",
          l.centred, l.centre_mean_shift, l.centre_max_shift, l.centre_min_width);
      }
    }
    for (const std::string & note : links->notes) {
      std::printf("  note: %s\n", note.c_str());
    }
  }
  // The stair links read the tree (the landings' walls); nothing below does.
  tree.reset();
  // --keepout: the zones as the plugin loads them beside the tree, with
  // robot_radius as the ring (the plugin's keepout.block_radius, 0.25, is
  // that same number by design).
  std::vector<KeepoutZone> zones;
  KeepoutMaskPtr keepout;
  if (!keepout_path.empty()) {
    std::string error;
    if (!readKeepoutFile(keepout_path, zones, error)) {
      std::fprintf(stderr, "%s\n", error.c_str());
      return 1;
    }
    keepout = buildKeepoutMask(*map, links.get(), zones, params.robot_radius);
    std::printf(
      "keepout: %zu zones from %s, %zu cells blocked (%zu core), %zu stair links cut (ring %.2f "
      "m)\n",
      zones.size(), keepout_path.c_str(), keepout ? keepout->blocked : size_t{0},
      keepout ? keepout->core : size_t{0}, keepout ? keepout->links_cut : size_t{0},
      params.robot_radius);
    for (const KeepoutZone & z : zones) {
      std::printf(
        "  zone %s: %zu vertices, z %.2f..%.2f\n", zoneLabel(z).c_str(), z.polygon.size(), z.z_min,
        z.z_max);
    }
  }
  if (!plans.empty()) {
    SurfaceAStar astar(map, links);
    // --dynamic: the people are placed by hand, not seen, so the ranges the
    // robot's scans are judged with do not apply (a person 10 m from the
    // start marks all the same) and no ray clears anything; the band and
    // the self radius are the plugin's.
    DynamicOverlayParams dyn;
    dyn.obstacle_max_range = 1e9;
    dyn.raytrace_max_range = 0.0;
    DynamicOverlay overlay(dyn);
    if (!people.empty()) {
      overlay.setMap(map);
    }
    for (const PlanReq & r : plans) {
      double v[6];
      std::copy(r.v, r.v + 6, v);
      if (r.by_pose) {
        const size_t i = static_cast<size_t>(r.v[0]), j = static_cast<size_t>(r.v[1]);
        if (i >= poses.size() || j >= poses.size()) {
          std::fprintf(
            stderr, "--plan-poses: need --poses with more than %zu entries\n", std::max(i, j));
          return 2;
        }
        v[0] = poses[i].x;
        v[1] = poses[i].y;
        v[2] = poses[i].z - lidar_height;
        v[3] = poses[j].x;
        v[4] = poses[j].y;
        v[5] = poses[j].z - lidar_height;
      }
      DynamicMaskPtr mask;
      if (!people.empty()) {
        overlay.clear();
        std::vector<Waypoint> returns;
        for (const Waypoint & w : people) {
          for (const double dz : {0.3, 0.5}) {
            for (const double dx : {-0.15, 0.0, 0.15}) {
              for (const double dy : {-0.15, 0.0, 0.15}) {
                returns.push_back({w.x + dx, w.y + dy, w.z + dz});
              }
            }
          }
        }
        // Seen from the plan's start, the lidar at its height over the floor.
        const Waypoint origin{v[0], v[1], v[2] + lidar_height};
        const ScanStats st = overlay.addScan(returns, origin, 0.0);
        mask = overlay.snapshot(0.0);
        std::printf(
          "dynamic: %zu people, %zu returns, %zu on traversable cells in the band -> %zu cells "
          "marked\n",
          people.size(), returns.size(), st.marked, mask ? mask->marked : size_t{0});
      }
      rc |= runPlan(
        astar, search, v[0], v[1], v[2], v[3], v[4], v[5], shortcut, out_path, &planned, mask.get(),
        keepout.get(), zones);
    }
  }
  if (!image_path.empty()) {
    ImageRegion reg;
    reg.ix1 = map->nx;
    reg.iy1 = map->ny;
    reg.scale = std::max(1, image_scale);
    reg.by_component = image_components;
    if (have_region) {
      reg.ix0 = std::max(0, map->ixOf(region[0]));
      reg.iy0 = std::max(0, map->iyOf(region[1]));
      reg.ix1 = std::min(map->nx, map->ixOf(region[2]) + 1);
      reg.iy1 = std::min(map->ny, map->iyOf(region[3]) + 1);
    }
    if (!dumpImage(*map, image_z, poses, planned, reg, keepout.get(), image_path)) {
      std::fprintf(stderr, "cannot write '%s'\n", image_path.c_str());
      return 1;
    }
    std::printf(
      "wrote %d x %d image of the layer near z %.2f to %s\n", (reg.ix1 - reg.ix0) * reg.scale,
      (reg.iy1 - reg.iy0) * reg.scale, image_z, image_path.c_str());
  }
  if (!html_path.empty()) {
    const std::string title = scene.empty() ? bt_path.substr(bt_path.find_last_of('/') + 1) : scene;
    if (!dumpHtml(*map, build_stats, planned, links.get(), title + " surface map", html_path)) {
      std::fprintf(stderr, "cannot write '%s'\n", html_path.c_str());
      return 1;
    }
    std::printf(
      "wrote %zu traversable cells and %zu paths to %s\n", build_stats.traversable, planned.size(),
      html_path.c_str());
  }
  if (!pcd_path.empty()) {
    if (!dumpPcd(*map, pcd_path)) {
      std::fprintf(stderr, "cannot write '%s'\n", pcd_path.c_str());
      return 1;
    }
    std::printf("wrote %zu cells to %s\n", map->cells.size(), pcd_path.c_str());
  }
  return rc;
}
