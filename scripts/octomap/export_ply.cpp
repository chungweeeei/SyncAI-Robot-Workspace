// Export a ray-cast OctoMap (pcd2octomap's output) as two PLY clouds of voxel
// centres, for viewing the drivable floor without the 40 M+ free voxels of air
// that hide it in octovis:
//
//  road_free.ply : per column, the LOWEST free voxel within +-band of the LOCAL
//                  floor -- observed-free space at ground level.
//  occupied.ply  : OCCUPIED voxels from local floor - band up to + max_h, so the
//                  ceiling is cut and a top-down view stays readable.
//
// "Local floor" is the nearest keyframe's z minus lidar_h (the lidar's height
// above the floor, 0.481 m on robot01 as the backend's local-floor measurement
// reports it), searched within `radius` of a 1 m cell. The site this was written
// on (dp1f_1006) has ~1 m of floor height change, so one global floor level
// would put half of it outside any band. radius 4 m left square holes in a hall
// wider than 8 m; 10 m does not.
//
// Why not "free voxel above an occupied floor voxel": tried first, and it finds
// almost nothing (391 m2 of ~1800 m2 drivable on dp1f_1006). A ray to a distant
// floor point crosses the near floor voxels at a grazing angle and integrates
// misses into them, so the floor itself ends up free and the voxel under the
// lowest free one is usually UNKNOWN (never observed from below), not occupied.
// Hence the test is "not free below", inside the floor band.
//
// Expect free beyond the walls too: rays through windows and doorways observe
// free space outside, which shows up as radial fans in a top-down view.
#include <octomap/octomap.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>
struct KF { double x, y, f; };
static void writePly(const std::string& p, const std::vector<float>& v) {
  std::ofstream f(p, std::ios::binary);
  f << "ply\nformat binary_little_endian 1.0\nelement vertex " << v.size() / 3
    << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
  f.write((const char*)v.data(), v.size() * sizeof(float));
}
int main(int argc, char** argv) {
  if (argc < 4) { std::cerr << "export_ply <map.bt> <poses.txt> <out_dir> [lidar_h=0.481] [band=0.25] [max_h=2.0] [radius=10]\n"; return 1; }
  double lh = argc > 4 ? atof(argv[4]) : 0.481, band = argc > 5 ? atof(argv[5]) : 0.25;
  double maxh = argc > 6 ? atof(argv[6]) : 2.0, rad = argc > 7 ? atof(argv[7]) : 10.0;
  std::string od = argv[3];
  octomap::OcTree t(argv[1]); double r = t.getResolution();
  std::vector<KF> kf; { std::ifstream f(argv[2]); std::string l; while (std::getline(f, l)) {
    std::istringstream s(l); std::string n; double x, y, z; if (s >> n >> x >> y >> z) kf.push_back({x, y, z - lh}); } }
  std::unordered_map<long long, double> cache;  // 1 m cell -> local floor (NaN: no keyframe within rad)
  auto localFloor = [&](double x, double y) {
    long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = cache.find(k); if (it != cache.end()) return it->second;
    double cx = std::floor(x) + .5, cy = std::floor(y) + .5, best = rad * rad, fz = NAN;
    for (auto& p : kf) { double d = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy); if (d < best) { best = d; fz = p.f; } }
    return cache[k] = fz; };
  std::vector<float> road, occ;
  for (auto it = t.begin_leafs(); it != t.end_leafs(); ++it) {
    double s = it.getSize(), bx = it.getX() - s / 2 + r / 2, by = it.getY() - s / 2 + r / 2, bz = it.getZ() - s / 2 + r / 2;
    if (t.isNodeOccupied(*it)) {
      for (double x = bx; x < it.getX() + s / 2; x += r) for (double y = by; y < it.getY() + s / 2; y += r)
        for (double z = bz; z < it.getZ() + s / 2; z += r) {
          double fz = localFloor(x, y); if (std::isnan(fz) || z < fz - band || z > fz + maxh) continue;
          occ.insert(occ.end(), {(float)x, (float)y, (float)z}); }
    } else {  // only the bottom face of a free node can touch the ground
      for (double x = bx; x < it.getX() + s / 2; x += r) for (double y = by; y < it.getY() + s / 2; y += r) {
        // Lowest free voxel of the column inside the floor band (see header).
        double fz = localFloor(x, y); if (std::isnan(fz) || std::fabs(bz - fz) > band) continue;
        auto* b = t.search(x, y, bz - r); if (b && !t.isNodeOccupied(b) ) continue;
        road.insert(road.end(), {(float)x, (float)y, (float)bz}); }
    }
  }
  writePly(od + "/road_free.ply", road); writePly(od + "/occupied.ply", occ);
  std::cout << "road_free " << road.size() / 3 << " voxels (" << road.size() / 3 * r * r << " m2), occupied " << occ.size() / 3 << std::endl;
}
