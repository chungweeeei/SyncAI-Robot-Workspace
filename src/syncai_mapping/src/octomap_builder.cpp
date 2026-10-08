#include "syncai_mapping/octomap_builder.hpp"

#include <octomap/octomap.h>
#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
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
  for (const char * name : {kOctomapFile, kOccupiedFile, kRoadFile}) {
    std::error_code ec;
    fs::remove(dir / (std::string(name) + ".tmp"), ec);
  }
}

bool buildImpl(
  const fs::path & dir, const Params & params, Measurements & out, const Progress & progress)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<PoseLine> poses;
  std::vector<FloorRef> floors;
  readPoses(dir / "poses.txt", params.lidar_height, poses, floors);

  // ---- Ray-cast (the prototype's pcd2octomap) ----
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
  octomap::OcTree tree(params.resolution);
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile((dir / "patches" / p.name).string(), cloud) < 0) {
      // Skipped, as the prototype did: one unreadable patch costs one scan
      // of free space, not the whole map. All of them is an InputError below.
      ++out.skipped_patches;
      continue;
    }
    Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
    q.normalize();
    const Eigen::Vector3d t(p.tx, p.ty, p.tz);
    octomap::Pointcloud oc;
    oc.reserve(cloud.size());
    for (const auto & pt : cloud) {
      const Eigen::Vector3d b(pt.x, pt.y, pt.z);
      const double r = b.norm();
      if (!std::isfinite(r) || r < params.min_range) continue;
      const Eigen::Vector3d w = q * b + t;
      oc.push_back(w.x(), w.y(), w.z());
    }
    // lazy_eval false, discretize true: a voxel hit by several rays of one
    // scan is updated once rather than once per ray.
    tree.insertPointCloud(oc, octomap::point3d(p.tx, p.ty, p.tz), params.max_range, false, true);
    out.points += oc.size();
    ++out.keyframes;
    if (progress && !progress(out.keyframes + out.skipped_patches, poses.size(), out.points)) {
      return false;
    }
  }
  if (out.keyframes == 0) throw InputError("no patch under patches/ could be read");

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
  // "Local floor" = the nearest keyframe's z minus lidar_height, looked up
  // per 1 m cell within floor_radius. One global floor level does not work:
  // dp1f_1006's floor changes height by about 1 m across the site, which puts
  // half of it outside any band. A 4 m radius left square holes in a hall
  // wider than 8 m; 10 m does not.
  //
  // Road = the LOWEST free voxel of a column inside +-floor_band, i.e. one
  // whose neighbour below is not free. The obvious "free voxel with an
  // occupied voxel under it" finds almost nothing (391 m2 against 4338 m2 on
  // dp1f_1006): a ray to a distant floor point crosses the near floor at a
  // grazing angle and integrates misses into it, so the floor itself ends up
  // free, and the voxel under the lowest free one is usually UNKNOWN (never
  // seen from below), not occupied. Expect free floor beyond the walls too --
  // rays through windows and doorways observe it, as radial fans from above.
  const double r = tree.getResolution();
  const double band = params.floor_band, maxh = params.max_height, rad = params.floor_radius;
  std::unordered_map<long long, double> cache;  // 1 m cell -> local floor (NaN: none within rad)
  auto localFloor = [&](double x, double y) {
    long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = cache.find(k);
    if (it != cache.end()) return it->second;
    double cx = std::floor(x) + .5, cy = std::floor(y) + .5, best = rad * rad, fz = NAN;
    for (auto & p : floors) {
      double d = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
      if (d < best) {
        best = d;
        fz = p.f;
      }
    }
    return cache[k] = fz;
  };
  pcl::PointCloud<pcl::PointXYZ> road, occ;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    double s = it.getSize(), bx = it.getX() - s / 2 + r / 2, by = it.getY() - s / 2 + r / 2,
           bz = it.getZ() - s / 2 + r / 2;
    if (tree.isNodeOccupied(*it)) {
      for (double x = bx; x < it.getX() + s / 2; x += r) {
        for (double y = by; y < it.getY() + s / 2; y += r) {
          for (double z = bz; z < it.getZ() + s / 2; z += r) {
            double fz = localFloor(x, y);
            if (std::isnan(fz) || z < fz - band || z > fz + maxh) continue;
            occ.push_back(pcl::PointXYZ((float)x, (float)y, (float)z));
          }
        }
      }
    } else {  // only the bottom face of a (pruned, larger) free node can touch the floor
      for (double x = bx; x < it.getX() + s / 2; x += r) {
        for (double y = by; y < it.getY() + s / 2; y += r) {
          double fz = localFloor(x, y);
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

  // Renamed last and together, so a failure above leaves the previous
  // outputs (or none) rather than a .bt that disagrees with its layers.
  commit(dir, kOctomapFile);
  commit(dir, kOccupiedFile);
  commit(dir, kRoadFile);
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
