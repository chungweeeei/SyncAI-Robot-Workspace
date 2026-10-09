#include "syncai_mapping/map_cleaner.hpp"

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

namespace syncai_mapping::map_cleaner
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

void readPoses(
  const fs::path & poses_path, double lidar_height, std::vector<PoseLine> & poses,
  std::vector<FloorRef> & floors)
{
  std::ifstream in(poses_path);
  if (!in) throw InputError("cannot open poses.txt (saved without patches?)");
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    PoseLine p;
    if (ss >> p.name >> p.tx >> p.ty >> p.tz >> p.qw >> p.qx >> p.qy >> p.qz) {
      poses.push_back(p);
      floors.push_back({p.tx, p.ty, p.tz - lidar_height});
    }
  }
  if (poses.empty()) throw InputError("poses.txt lists no keyframe");
}

// What identifies "the save this clean started from". A second save_maps into
// the same directory rewrites poses.txt; size and mtime are compared too
// because a truncating rewrite may keep the inode.
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
  try {
    if (pcl::io::loadPCDFile((dir / "patches" / p.name).string(), cloud) < 0) return false;
  } catch (const pcl::PCLException &) {
    return false;
  }
  q = Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz);
  q.normalize();
  t = Eigen::Vector3d(p.tx, p.ty, p.tz);
  return true;
}

// One keyframe's returns, as the counting passes see them: world points at or
// beyond min_range, split into the ones judged as usual and the ones below the
// keyframe's floor (Params::clip_below_floor), whose ray stops at the clip
// height. Both passes must see exactly the same scan, or a voxel's hit and
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
  // One voxel inside the floor band, not at its edge: grazing rays carve all
  // the way down to wherever the clip is (measured on the OctoMap prototype).
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

// The voxels one scan passes through without ending in them: OctoMap's own
// discretized update (one ray per endpoint voxel, truncated at max_range),
// plus the clipped rays up to their clip point. A voxel the scan also ends in
// is not a miss, as in OctoMap's update, where occupied wins within a scan.
void scanMisses(
  octomap::OcTree & tree, const Scan & s, const octomap::point3d & origin, double max_range,
  octomap::KeySet & free_cells)
{
  octomap::KeySet occupied_cells;
  tree.computeDiscreteUpdate(s.endpoints, origin, free_cells, occupied_cells, max_range);
  if (!s.clipped_ends.empty()) {
    octomap::KeyRay ray;
    for (octomap::point3d c : s.clipped_ends) {
      const octomap::point3d d = c - origin;
      if (max_range > 0.0 && d.norm() > max_range) c = origin + d.normalized() * max_range;
      if (tree.computeRayKeys(origin, c, ray)) free_cells.insert(ray.begin(), ray.end());
    }
  }
  for (const auto & k : occupied_cells) free_cells.erase(k);
}

// Per hit voxel: how many scans ended in it, ended in it below their floor,
// and passed through it, and what the classification made of it. Keyed on HIT
// voxels only: keying every voxel a ray crosses would hold every free voxel
// of the site (tens of millions at 0.1 m, GBs); hit voxels are a few million.
struct Counts
{
  uint32_t hits = 0;
  uint32_t clipped = 0;
  uint32_t misses = 0;
  bool removed = false;
};
using CountMap = std::unordered_map<octomap::OcTreeKey, Counts, octomap::OcTreeKey::KeyHash>;

void savePcd(const fs::path & tmp, const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  // pcl's IOException puts the absolute path in what(), and this text lands
  // in the sidecar. Re-say it with the bare name.
  try {
    if (pcl::io::savePCDFileBinary(tmp.string(), cloud) != 0) {
      throw std::runtime_error(std::string("cannot write ") + kMapFile);
    }
  } catch (const pcl::IOException &) {
    throw std::runtime_error(std::string("cannot write ") + kMapFile);
  }
}

fs::path tmpPath(const fs::path & dir)
{
  return dir / (std::string(kMapFile) + ".tmp");
}

bool cleanImpl(
  const fs::path & dir, const Params & params, Measurements & out, const Progress & progress)
{
  const auto t0 = std::chrono::steady_clock::now();
  const fs::path poses_path = dir / "poses.txt";
  const Fingerprint started_from = fingerprint(poses_path);
  std::vector<PoseLine> poses;
  std::vector<FloorRef> floors;
  readPoses(poses_path, params.lidar_height, poses, floors);

  // For the key arithmetic and ray traversal only; never updated. Not const:
  // computeDiscreteUpdate reuses scratch members of the tree.
  octomap::OcTree tree(params.resolution);
  const double band = params.floor_band, maxh = params.max_height, rad = params.floor_radius;
  CountMap counts;

  // ---- Pass 1: hits ----
  //
  // The ray origin is the keyframe pose, i.e. the IMU origin; the lidar's is
  // t + R * t_il, 5.1 cm away on this robot. Both rays end at the same world
  // point, so they never diverge by more than those 5 cm: under one voxel,
  // and inside the metres min_range writes off as the robot's body.
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (!loadPatch(dir, p, cloud, q, t)) {
      ++out.skipped_patches;
      continue;
    }
    const Scan scan = worldScan(cloud, q, t, params);
    const octomap::point3d origin(p.tx, p.ty, p.tz);
    octomap::KeySet hit_keys, clipped_keys;  // once per scan
    for (const auto & w : scan.endpoints) {
      if (params.max_range > 0.0 && (w - origin).norm() > params.max_range) continue;
      octomap::OcTreeKey k;
      if (tree.coordToKeyChecked(w, k)) hit_keys.insert(k);
    }
    for (const auto & w : scan.clipped_hits) {
      octomap::OcTreeKey k;
      if (tree.coordToKeyChecked(w, k)) clipped_keys.insert(k);
    }
    for (const auto & k : hit_keys) ++counts[k].hits;
    for (const auto & k : clipped_keys) ++counts[k].clipped;
    out.clipped_rays += scan.clipped_ends.size();
    out.points += scan.endpoints.size() + scan.clipped_ends.size();
    ++out.keyframes;
    if (progress && !progress("hits", out.keyframes + out.skipped_patches, poses.size())) {
      return false;
    }
  }
  if (out.keyframes == 0) throw InputError("no patch under patches/ could be read");

  // ---- Pass 2: misses ----
  //
  // The same scans again, after pass 1 rather than inside it, so a voxel
  // first hit late still gets the misses of the scans that crossed it earlier.
  size_t done = 0;
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (loadPatch(dir, p, cloud, q, t)) {
      octomap::KeySet free_cells;
      scanMisses(
        tree, worldScan(cloud, q, t, params), octomap::point3d(p.tx, p.ty, p.tz), params.max_range,
        free_cells);
      for (const auto & k : free_cells) {
        auto it = counts.find(k);
        if (it != counts.end()) ++it->second.misses;
      }
    }
    if (progress && !progress("misses", ++done, poses.size())) return false;
  }

  // ---- The local floor ----
  //
  // A nearby keyframe's z minus lidar_height, looked up per 1 m cell within
  // floor_radius. One global floor level does not work: dp1f_1006's floor
  // changes height by about 1 m across the site. Which keyframe depends on
  // the voxel's height too: only keyframes whose floor is at or below the
  // voxel (within floor_band) qualify, of those the highest level (within
  // kLevelTol of the highest), and of that level the nearest in xy. On one
  // floor that is simply the nearest keyframe. Under a second floor the
  // xy-nearest keyframe is often upstairs: on the prototype, 51 of the 54 1F
  // cells it got wrong on dp1f_1008_2 lay under 2F and took 2F's floor, 5 m off.
  constexpr double kLevelTol = 1.0;  // m; well under any storey height
  struct CellFloor
  {
    double f, d2;
  };
  std::unordered_map<long long, std::vector<CellFloor>> cell_floors;  // ascending f
  auto localFloor = [&](double x, double y, double z) {
    const long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = cell_floors.find(k);
    if (it == cell_floors.end()) {
      const double cx = std::floor(x) + .5, cy = std::floor(y) + .5;
      std::vector<CellFloor> v;
      for (const auto & p : floors) {
        const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < rad * rad) v.push_back({p.f, d2});
      }
      std::sort(
        v.begin(), v.end(), [](const CellFloor & a, const CellFloor & b) { return a.f < b.f; });
      it = cell_floors.emplace(k, std::move(v)).first;
    }
    const auto & v = it->second;
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

  // ---- Classify ----
  for (auto & [key, c] : counts) {
    const octomap::point3d v = tree.keyToCoord(key);
    const double fz = localFloor(v.x(), v.y(), v.z());
    bool dynamic = false;
    if (
      params.dynamic_min_miss > 0 && c.hits > 0 && c.misses >= (uint32_t)params.dynamic_min_miss &&
      c.misses >= params.dynamic_miss_ratio * c.hits) {
      dynamic = !std::isnan(fz) && v.z() >= fz + band && v.z() <= fz + maxh;
    }
    const bool on_floor = !std::isnan(fz) && std::fabs(v.z() - fz) <= band;
    const bool sparse = !dynamic && !on_floor && params.min_hits > 1 &&
                        c.hits + c.clipped < (uint32_t)params.min_hits;
    if (!dynamic && !sparse) continue;
    c.removed = true;
    (dynamic ? out.dynamic_voxels : out.sparse_voxels)++;
  }

  // ---- Pass 3: map.pcd without the removed voxels ----
  //
  // The same concatenation pgo's saveMapsCB writes -- every patch, in
  // poses.txt order, PointXYZI, pcl::transformPointCloud with the saved pose,
  // no min_range cut and no voxel filter -- minus every point whose voxel was
  // classified away. A point no scan counted (beyond max_range, inside
  // min_range) is kept unless another scan's hit classified its voxel. The
  // pose is poses.txt's, which pgo prints with 6 significant digits:
  // sub-millimetre at site scale.
  pcl::PointCloud<pcl::PointXYZI> map;
  done = 0;
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
    if (progress && !progress("map", ++done, poses.size())) return false;
  }
  out.map_points_kept = map.size();
  if (map.empty()) throw std::runtime_error("cleaned map.pcd would be empty");
  savePcd(tmpPath(dir), map);

  // A save into this directory while the clean ran replaced poses.txt and
  // patches/; renaming now would put a map built from the OLD save over it.
  if (!(fingerprint(poses_path) == started_from)) {
    throw std::runtime_error("superseded: poses.txt changed during the clean (a newer save)");
  }
  std::error_code ec;
  fs::rename(tmpPath(dir), dir / kMapFile, ec);
  if (ec) throw std::runtime_error(std::string("cannot rename ") + kMapFile + ": " + ec.message());
  out.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return true;
}

}  // namespace

bool clean(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress)
{
  out = Measurements{};
  auto removeTmp = [&]() {
    std::error_code ec;
    std::filesystem::remove(tmpPath(dir), ec);
  };
  try {
    const bool done = cleanImpl(dir, params, out, progress);
    if (!done) removeTmp();
    return done;
  } catch (...) {
    removeTmp();
    throw;
  }
}

}  // namespace syncai_mapping::map_cleaner
