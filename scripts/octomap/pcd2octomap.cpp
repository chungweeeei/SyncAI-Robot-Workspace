// Build an OctoMap from a pgo save (patches/<i>.pcd + poses.txt), ray-casting
// every keyframe's body cloud from its keyframe origin so free space is carved,
// not just occupied voxels marked.
#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

int main(int argc, char** argv) {
  if (argc < 3) { std::cerr << "usage: pcd2octomap <map_dir> <out.bt> [res=0.05] [max_range=20] [min_range=0.5]\n"; return 1; }
  std::string dir = argv[1], out = argv[2];
  double res = argc > 3 ? atof(argv[3]) : 0.05;
  double max_range = argc > 4 ? atof(argv[4]) : 20.0;
  double min_range = argc > 5 ? atof(argv[5]) : 0.5;
  octomap::OcTree tree(res);
  std::ifstream poses(dir + "/poses.txt");
  std::string line; size_t n = 0, pts = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (std::getline(poses, line)) {
    std::istringstream ss(line); std::string name; double tx, ty, tz, qw, qx, qy, qz;
    if (!(ss >> name >> tx >> ty >> tz >> qw >> qx >> qy >> qz)) continue;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile(dir + "/patches/" + name, cloud) < 0) { std::cerr << "skip " << name << "\n"; continue; }
    Eigen::Quaterniond q(qw, qx, qy, qz); q.normalize(); Eigen::Vector3d t(tx, ty, tz);
    octomap::Pointcloud oc; oc.reserve(cloud.size());
    for (auto& p : cloud) {
      Eigen::Vector3d b(p.x, p.y, p.z); double r = b.norm();
      if (!std::isfinite(r) || r < min_range) continue;
      Eigen::Vector3d w = q * b + t; oc.push_back(w.x(), w.y(), w.z());
    }
    tree.insertPointCloud(oc, octomap::point3d(tx, ty, tz), max_range, false, true);
    pts += oc.size();
    if (++n % 100 == 0) std::cout << n << " keyframes, " << pts << " points, "
      << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s" << std::endl;
  }
  tree.updateInnerOccupancy(); tree.prune();
  tree.writeBinary(out);
  double x, y, z, X, Y, Z; tree.getMetricMin(x, y, z); tree.getMetricMax(X, Y, Z);
  size_t occ = 0, fr = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) (tree.isNodeOccupied(*it) ? occ : fr)++;
  std::cout << "DONE keyframes=" << n << " points=" << pts << " res=" << res << " leafs=" << tree.getNumLeafNodes()
            << " occupied_leafs=" << occ << " free_leafs=" << fr
            << " bbox=[" << x << "," << y << "," << z << "]..[" << X << "," << Y << "," << Z << "]"
            << " mem=" << tree.memoryUsage() / 1e6 << "MB" << std::endl;
}
