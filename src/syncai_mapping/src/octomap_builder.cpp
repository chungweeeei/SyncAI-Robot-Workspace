#include "syncai_mapping/octomap_builder.hpp"

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/stat.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

namespace syncai_mapping::octomap_builder
{

namespace
{

namespace fs = std::filesystem;

struct PoseLine
{
  std::string name;
  double tx, ty, tz, qw, qx, qy, qz;
};

// A keyframe's xy and the floor height under it (z - lidar_height).
struct FloorRef
{
  double x, y, f;
};

// The two parses are the prototype's two, kept apart on purpose:
// pcd2octomap took a line only with all eight fields, export_ply took any line
// that starts with a name and three numbers. Folding them into one would move
// a voxel on a hand-edited poses.txt and break the byte-for-byte match.
void readPoses(
  const fs::path & poses_path, double lidar_height, std::vector<PoseLine> & poses,
  std::vector<FloorRef> & floors)
{
  std::ifstream in(poses_path);
  if (!in) throw InputError("cannot open poses.txt (saved without patches?)");
  std::string line;
  while (std::getline(in, line)) {
    {
      std::istringstream ss(line);
      PoseLine p;
      if (ss >> p.name >> p.tx >> p.ty >> p.tz >> p.qw >> p.qx >> p.qy >> p.qz) {
        poses.push_back(p);
      }
    }
    std::istringstream ss(line);
    std::string n;
    double x, y, z;
    if (ss >> n >> x >> y >> z) floors.push_back({x, y, z - lidar_height});
  }
  if (poses.empty()) throw InputError("poses.txt lists no keyframe");
}

// pcl's IOException and std::filesystem's errors both put the absolute path
// in what(), and this text lands in the sidecar. Re-say it with the bare name.
void savePcd(const fs::path & dir, const char * name, const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  const fs::path tmp = dir / (std::string(name) + ".tmp");
  try {
    if (pcl::io::savePCDFileBinary(tmp.string(), cloud) != 0) {
      throw std::runtime_error(std::string("cannot write ") + name);
    }
  } catch (const pcl::IOException &) {
    throw std::runtime_error(std::string("cannot write ") + name);
  }
}

void commit(const fs::path & dir, const char * name)
{
  std::error_code ec;
  fs::rename(dir / (std::string(name) + ".tmp"), dir / name, ec);
  if (ec) throw std::runtime_error(std::string("cannot rename ") + name + ": " + ec.message());
}

void removeTmps(const fs::path & dir)
{
  for (const char * name : {kOctomapFile, kOccupiedFile, kRoadFile, kMapFile}) {
    std::error_code ec;
    fs::remove(dir / (std::string(name) + ".tmp"), ec);
  }
}

// What identifies "the save this build started from". A second save_maps
// into the same directory rewrites poses.txt (new inode via the ofstream
// truncate is not guaranteed, so size and mtime are compared too).
struct Fingerprint
{
  dev_t dev = 0;
  ino_t ino = 0;
  off_t size = 0;
  timespec mtime = {};
  bool operator==(const Fingerprint & o) const
  {
    return dev == o.dev && ino == o.ino && size == o.size && mtime.tv_sec == o.mtime.tv_sec &&
           mtime.tv_nsec == o.mtime.tv_nsec;
  }
};

Fingerprint fingerprint(const fs::path & poses_path)
{
  struct stat st = {};
  if (stat(poses_path.c_str(), &st) != 0) return {};
  Fingerprint f;
  f.dev = st.st_dev;
  f.ino = st.st_ino;
  f.size = st.st_size;
  f.mtime = st.st_mtim;
  return f;
}

// One keyframe's patch: the raw body-frame cloud as pgo saved it, plus its
// pose. false when the patch cannot be read.
bool loadPatch(
  const fs::path & dir, const PoseLine & p, pcl::PointCloud<pcl::PointXYZI> & cloud,
  Eigen::Quaterniond & q, Eigen::Vector3d & t)
{
  if (pcl::io::loadPCDFile((dir / "patches" / p.name).string(), cloud) < 0) return false;
  q = Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz);
  q.normalize();
  t = Eigen::Vector3d(p.tx, p.ty, p.tz);
  return true;
}

// The ray-cast input of one patch: world points at or beyond min_range,
// split into the ones inserted as usual and the ones below the keyframe's
// floor (Params::clip_below_floor), whose ray stops at the clip height.
// Pass 1 and pass 2 must see exactly the same scan, or a voxel's hit and
// miss counts would come from different scans.
struct Scan
{
  octomap::Pointcloud endpoints;
  std::vector<octomap::point3d> clipped_ends;  // where a clipped ray stops
  std::vector<octomap::point3d> clipped_hits;  // the clipped returns themselves
};

Scan worldScan(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const Eigen::Quaterniond & q,
  const Eigen::Vector3d & t, const Params & params)
{
  Scan s;
  s.endpoints.reserve(cloud.size());
  // One voxel inside the band, not at its edge: the dense input's grazing
  // rays carve all the way down to wherever the clip is, and a column whose
  // lowest free voxel sits just under -floor_band is not a road column. At
  // the band edge 21 % of dp1f_1008_2's corridor was still lost that way.
  const double clip_z =
    t.z() - params.lidar_height - std::max(0.0, params.floor_band - params.resolution);
  for (const auto & pt : cloud) {
    const Eigen::Vector3d b(pt.x, pt.y, pt.z);
    const double r = b.norm();
    if (!std::isfinite(r) || r < params.min_range) continue;
    const Eigen::Vector3d w = q * b + t;
    if (params.clip_below_floor && w.z() < clip_z) {
      const double f = (t.z() - clip_z) / (t.z() - w.z());
      const Eigen::Vector3d c = t + f * (w - t);
      s.clipped_ends.emplace_back(c.x(), c.y(), c.z());
      s.clipped_hits.emplace_back(w.x(), w.y(), w.z());
      continue;
    }
    s.endpoints.push_back(w.x(), w.y(), w.z());
  }
  return s;
}

// What insertPointCloud(lazy false, discretize true) computes for one scan,
// plus the clipped rays: free up to their clip point, no endpoint. Occupied
// still wins over free within the scan, as in OctoMap's own update.
void castScan(
  octomap::OcTree & tree, const Scan & s, const octomap::point3d & origin, double max_range,
  octomap::KeySet & free_cells, octomap::KeySet & occupied_cells)
{
  tree.computeDiscreteUpdate(s.endpoints, origin, free_cells, occupied_cells, max_range);
  if (s.clipped_ends.empty()) return;
  octomap::KeyRay ray;
  for (octomap::point3d c : s.clipped_ends) {
    const octomap::point3d d = c - origin;
    if (max_range > 0.0 && d.norm() > max_range) c = origin + d.normalized() * max_range;
    if (tree.computeRayKeys(origin, c, ray)) free_cells.insert(ray.begin(), ray.end());
  }
  for (const auto & k : occupied_cells) free_cells.erase(k);
}

// Per hit voxel: how many scans ended in it, how many passed through it, and
// what the cleaning made of it. Keyed on HIT voxels only: keying every voxel a
// ray crosses would hold every free voxel of the site (tens of millions at
// 0.1 m, 1-2 GB of hash map); hit voxels are a few million.
struct Counts
{
  uint32_t hits = 0;     // scans whose (inserted) return ended here
  uint32_t clipped = 0;  // scans whose below-floor return ended here (not inserted)
  uint32_t misses = 0;
  bool removed = false;
};
using CountMap = std::unordered_map<octomap::OcTreeKey, Counts, octomap::OcTreeKey::KeyHash>;

bool buildImpl(
  const fs::path & dir, const Params & params, Measurements & out, const Progress & progress)
{
  const auto t0 = std::chrono::steady_clock::now();
  const fs::path poses_path = dir / "poses.txt";
  const Fingerprint started_from = fingerprint(poses_path);
  std::vector<PoseLine> poses;
  std::vector<FloorRef> floors;
  readPoses(poses_path, params.lidar_height, poses, floors);
  const bool cleaning = params.cleaning();

  // ---- Pass 1: ray-cast (the prototype's pcd2octomap) ----
  //
  // map.pcd would be the wrong input: it carries no sensor origins, so every
  // point could only be marked occupied and free space would stay unknown --
  // and free space at floor level is the whole point of the road layer. Each
  // patch is inserted from its own keyframe position instead.
  //
  // That position is the keyframe pose, i.e. the IMU origin; the lidar's is
  // t + R * t_il, 5.1 cm away on this robot (syncai_pointlio's extrinsic).
  // Both rays end at the same world point, so they never diverge by more than
  // those 5 cm and meet at the endpoint: under one voxel at 0.1 m, and inside
  // the first metres that min_range already writes off as the robot's body.
  // Not worth coupling a display job to pointlio's parameters.
  //
  // computeDiscreteUpdate + the two updateNode loops are exactly what
  // insertPointCloud(..., lazy_eval false, discretize true) does inside
  // (OccupancyOcTreeBase.hxx): one update per voxel per scan, free first,
  // occupied winning. Spelt out so the occupied keys can be counted; with the
  // cleaning off the .bt is byte-identical to the insertPointCloud build.
  octomap::OcTree tree(params.resolution);
  CountMap counts;
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (!loadPatch(dir, p, cloud, q, t)) {
      // Skipped, as the prototype did: one unreadable patch costs one scan
      // of free space, not the whole map. All of them is an InputError below.
      ++out.skipped_patches;
      continue;
    }
    const Scan scan = worldScan(cloud, q, t, params);
    octomap::KeySet free_cells, occupied_cells;
    castScan(
      tree, scan, octomap::point3d(p.tx, p.ty, p.tz), params.max_range, free_cells, occupied_cells);
    for (const auto & k : free_cells) tree.updateNode(k, false, false);
    for (const auto & k : occupied_cells) {
      tree.updateNode(k, true, false);
      if (cleaning) ++counts[k].hits;
    }
    if (cleaning && !scan.clipped_hits.empty()) {
      octomap::KeySet clipped_keys;  // once per scan, like the hits
      for (const auto & w : scan.clipped_hits) {
        octomap::OcTreeKey k;
        if (tree.coordToKeyChecked(w, k)) clipped_keys.insert(k);
      }
      for (const auto & k : clipped_keys) ++counts[k].clipped;
    }
    out.clipped_rays += scan.clipped_ends.size();
    out.points += scan.endpoints.size() + scan.clipped_ends.size();
    ++out.keyframes;
    if (
      progress &&
      !progress("insert", out.keyframes + out.skipped_patches, poses.size(), out.points)) {
      return false;
    }
  }
  if (out.keyframes == 0) throw InputError("no patch under patches/ could be read");

  const double r = tree.getResolution();
  const double band = params.floor_band, maxh = params.max_height, rad = params.floor_radius;
  // "Local floor" = a nearby keyframe's z minus lidar_height, looked up per
  // 1 m cell within floor_radius. One global floor level does not work:
  // dp1f_1006's floor changes height by about 1 m across the site, which puts
  // half of it outside any band. A 4 m radius left square holes in a hall
  // wider than 8 m; 10 m does not.
  //
  // Which keyframe depends on the voxel's z as well (2026-10-09). The nearest
  // one in xy alone was right on a single floor and wrong under a second one:
  // on dp1f_1008_2, 51 of the 54 1F path cells with no road lay under 2F,
  // whose keyframes were nearer in xy, so the 1F floor sat 5 m outside the
  // band. Now only keyframes whose floor is at or below the voxel (within
  // floor_band) are candidates, of those only the highest level (within
  // kLevelTol of the highest candidate floor), and of those the nearest in
  // xy. On a single floor that is the nearest keyframe as before, for every
  // voxel the layers keep.
  constexpr double kLevelTol = 1.0;  // m; well under any storey height
  struct CellFloor
  {
    double f, d2;
  };
  // 1 m cell -> keyframe floors within rad, ascending f
  std::unordered_map<long long, std::vector<CellFloor>> cell_floors;
  auto localFloor = [&](double x, double y, double z) {
    long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = cell_floors.find(k);
    if (it == cell_floors.end()) {
      const double cx = std::floor(x) + .5, cy = std::floor(y) + .5;
      std::vector<CellFloor> v;
      for (auto & p : floors) {
        const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < rad * rad) v.push_back({p.f, d2});
      }
      std::sort(
        v.begin(), v.end(), [](const CellFloor & a, const CellFloor & b) { return a.f < b.f; });
      it = cell_floors.emplace(k, std::move(v)).first;
    }
    const auto & v = it->second;
    // The candidates are v[0 .. hi): floors no higher than z + band.
    auto hi = std::upper_bound(
      v.begin(), v.end(), z + band, [](double val, const CellFloor & c) { return val < c.f; });
    if (hi == v.begin()) return (double)NAN;
    const double top = (hi - 1)->f;
    double best = std::numeric_limits<double>::max(), fz = NAN;
    for (auto c = hi; c != v.begin();) {
      --c;
      if (c->f < top - kLevelTol) break;
      if (c->d2 < best) {
        best = c->d2;
        fz = c->f;
      }
    }
    return fz;
  };

  if (cleaning) {
    // ---- Pass 2: count the misses of every hit voxel ----
    //
    // The same scans, ray-cast again with no tree update. Done after pass 1
    // rather than inside it, so a voxel first hit late still gets the misses
    // of the scans that crossed it earlier.
    size_t done = 0;
    for (const auto & p : poses) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      if (loadPatch(dir, p, cloud, q, t)) {
        const Scan scan = worldScan(cloud, q, t, params);
        octomap::KeySet free_cells, occupied_cells;
        castScan(
          tree, scan, octomap::point3d(p.tx, p.ty, p.tz), params.max_range, free_cells,
          occupied_cells);
        for (const auto & k : free_cells) {
          auto it = counts.find(k);
          if (it != counts.end()) ++it->second.misses;
        }
      }
      if (progress && !progress("count", ++done, poses.size(), out.points)) return false;
    }

    // ---- Classify, and free what goes ----
    //
    // Set to the clamping minimum rather than deleted: a deleted node is
    // UNKNOWN, and unknown-below is what makes a free voxel a road candidate
    // in the layers below, so deleting a person's feet would put a road voxel
    // a band higher. Done before updateInnerOccupancy / prune, so the .bt and
    // both layers see the cleaned tree.
    const float free_log = tree.getClampingThresMinLog();
    for (auto & [key, c] : counts) {
      const octomap::point3d v = tree.keyToCoord(key);
      const double fz = localFloor(v.x(), v.y(), v.z());
      bool dynamic = false;
      if (
        params.dynamic_min_miss > 0 && c.misses >= (uint32_t)params.dynamic_min_miss &&
        c.misses >= params.dynamic_miss_ratio * c.hits) {
        dynamic = !std::isnan(fz) && v.z() >= fz + band && v.z() <= fz + maxh;
      }
      // The floor band is exempt: the floor is a real surface the lidar
      // samples thinly (12 % of a keyframe's points on dp1f_1008_2), so a
      // single-hit floor voxel is the norm, not noise -- applied there, this
      // rule took 17 % of the corridor floor out of map.pcd.
      const bool on_floor = !std::isnan(fz) && std::fabs(v.z() - fz) <= band;
      // Clipped returns count as hits here (a real lower level seen from many
      // keyframes stays in map.pcd), but they are not in the tree: a voxel
      // only ever hit by them is dropped from map.pcd and never created --
      // creating it free would put free space under the floor again.
      const bool sparse = !dynamic && !on_floor && params.min_hits > 1 &&
                          c.hits + c.clipped < (uint32_t)params.min_hits;
      if (!dynamic && !sparse) continue;
      c.removed = true;
      (dynamic ? out.dynamic_voxels : out.sparse_voxels)++;
      if (c.hits > 0) tree.setNodeValue(key, free_log, false);
    }
  }

  tree.updateInnerOccupancy();
  tree.prune();
  // The non-const writeBinary converts the tree to max-likelihood and prunes
  // it again before writing, in place. The layers below are read from that
  // same in-memory tree, which is exactly what the prototype's export_ply
  // read back from the .bt -- so there is no second 1-4 GB read here.
  const fs::path bt_tmp = dir / (std::string(kOctomapFile) + ".tmp");
  if (!tree.writeBinary(bt_tmp.string())) {
    throw std::runtime_error(std::string("cannot write ") + kOctomapFile);
  }
  out.leaves = tree.getNumLeafNodes();
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    (tree.isNodeOccupied(*it) ? out.occupied_leaves : out.free_leaves)++;
  }

  // ---- The two display layers (the prototype's export_ply) ----
  //
  // The free voxels themselves are useless to look at: every one of them is
  // air a ray passed through, tens of millions on a site, and they bury the
  // floor. What the operator wants is the free layer AT the floor.
  //
  // Road = the LOWEST free voxel of a column inside +-floor_band, i.e. one
  // whose neighbour below is not free. The obvious "free voxel with an
  // occupied voxel under it" finds almost nothing (391 m2 against 4338 m2 on
  // dp1f_1006): a ray to a distant floor point crosses the near floor at a
  // grazing angle and integrates misses into it, so the floor itself ends up
  // free, and the voxel under the lowest free one is usually UNKNOWN (never
  // seen from below), not occupied. Expect free floor beyond the walls too --
  // rays through windows and doorways observe it, as radial fans from above.
  pcl::PointCloud<pcl::PointXYZ> road, occ;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    double s = it.getSize(), bx = it.getX() - s / 2 + r / 2, by = it.getY() - s / 2 + r / 2,
           bz = it.getZ() - s / 2 + r / 2;
    if (tree.isNodeOccupied(*it)) {
      for (double x = bx; x < it.getX() + s / 2; x += r) {
        for (double y = by; y < it.getY() + s / 2; y += r) {
          for (double z = bz; z < it.getZ() + s / 2; z += r) {
            double fz = localFloor(x, y, z);
            if (std::isnan(fz) || z < fz - band || z > fz + maxh) continue;
            occ.push_back(pcl::PointXYZ((float)x, (float)y, (float)z));
          }
        }
      }
    } else {  // only the bottom face of a (pruned, larger) free node can touch the floor
      for (double x = bx; x < it.getX() + s / 2; x += r) {
        for (double y = by; y < it.getY() + s / 2; y += r) {
          double fz = localFloor(x, y, bz);
          if (std::isnan(fz) || std::fabs(bz - fz) > band) continue;
          auto * b = tree.search(x, y, bz - r);
          if (b && !tree.isNodeOccupied(b)) continue;
          road.push_back(pcl::PointXYZ((float)x, (float)y, (float)bz));
        }
      }
    }
  }
  out.road_voxels = road.size();
  out.occupied_voxels = occ.size();
  out.road_area_m2 = road.size() * r * r;
  // pcl cannot write an empty binary PCD -- and an empty layer is a wrong
  // map, not an empty one: the floor band missed the floor (lidar_height for
  // another robot, or a save whose z is off by metres).
  if (road.empty()) {
    throw std::runtime_error(
      "road layer is empty: lidar_height / floor_band do not match this save");
  }
  if (occ.empty()) {
    throw std::runtime_error(
      "occupied layer is empty: lidar_height / max_height do not match this save");
  }
  road.is_dense = occ.is_dense = true;  // voxel centres: never NaN
  savePcd(dir, kOccupiedFile, occ);
  savePcd(dir, kRoadFile, road);

  if (cleaning) {
    // ---- Pass 3: map.pcd without the removed voxels ----
    //
    // The same concatenation pgo's saveMapsCB writes -- every patch, in
    // poses.txt order, PointXYZI, pcl::transformPointCloud with the saved
    // pose, no min_range cut and no voxel filter -- minus every point whose
    // voxel was freed above. A point no scan counted (beyond max_range or
    // inside min_range of its own keyframe) is kept unless another scan's hit
    // classified its voxel. The pose is poses.txt's, which pgo prints with 6
    // significant digits: sub-millimetre at site scale.
    pcl::PointCloud<pcl::PointXYZI> map;
    size_t done = 0;
    for (const auto & p : poses) {
      pcl::PointCloud<pcl::PointXYZI> cloud, world;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      if (loadPatch(dir, p, cloud, q, t)) {
        pcl::transformPointCloud(cloud, world, t, q);
        map.reserve(map.size() + world.size());
        for (size_t i = 0; i < cloud.size(); ++i) {
          // The key from the same double arithmetic pass 1 used, so a point
          // lands in the voxel it was counted in.
          const auto & b = cloud[i];
          const Eigen::Vector3d w = q * Eigen::Vector3d(b.x, b.y, b.z) + t;
          octomap::OcTreeKey key;
          if (tree.coordToKeyChecked(w.x(), w.y(), w.z(), key)) {
            auto it = counts.find(key);
            if (it != counts.end() && it->second.removed) {
              ++out.map_points_removed;
              continue;
            }
          }
          map.push_back(world[i]);
        }
      }
      if (progress && !progress("map", ++done, poses.size(), out.points)) return false;
    }
    out.map_points_kept = map.size();
    if (map.empty()) throw std::runtime_error("cleaned map.pcd would be empty");
    const fs::path tmp = dir / (std::string(kMapFile) + ".tmp");
    try {
      if (pcl::io::savePCDFileBinary(tmp.string(), map) != 0) {
        throw std::runtime_error(std::string("cannot write ") + kMapFile);
      }
    } catch (const pcl::IOException &) {
      throw std::runtime_error(std::string("cannot write ") + kMapFile);
    }
  }

  // A save into this directory while the build ran replaced poses.txt and
  // patches/; renaming now would put this build's outputs -- a map.pcd built
  // from the OLD save among them -- over the new one.
  if (!(fingerprint(poses_path) == started_from)) {
    throw std::runtime_error("superseded: poses.txt changed during the build (a newer save)");
  }
  // Renamed last and together, so a failure above leaves the previous
  // outputs (or none) rather than a .bt that disagrees with its layers.
  commit(dir, kOctomapFile);
  commit(dir, kOccupiedFile);
  commit(dir, kRoadFile);
  if (cleaning) {
    commit(dir, kMapFile);
    out.map_pcd_rewritten = true;
  }
  out.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return true;
}

}  // namespace

bool build(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress)
{
  out = Measurements{};
  try {
    const bool done = buildImpl(dir, params, out, progress);
    if (!done) removeTmps(dir);
    return done;
  } catch (...) {
    removeTmps(dir);
    throw;
  }
}

}  // namespace syncai_mapping::octomap_builder
