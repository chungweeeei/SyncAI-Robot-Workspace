#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SYNTHETIC_OCTREE_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SYNTHETIC_OCTREE_HPP_

// Small synthetic scenes for the surface map's tests and for the CLI's
// --synthetic option: a floor with a ramp, a stair, a kerb, holes, a balcony
// edge, a wall with a door, a low table, two floors. They exist so the
// geometry rules can be checked for the right answer without a 300 MB site
// map, and so a regression shows up as "the stair became traversable" rather
// than as a changed cell count.
//
// Convention, matching what clean_map writes: the surface voxel of a column
// at height h is the voxel whose top face is at h (the one containing
// h - res/2), the free_cells voxels above it are observed free, and nothing
// else is touched, so everything above that band is unknown -- as it is in a
// real map, where the lidar sees the ceiling only occasionally.

#include <octomap/OcTree.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace syncai_global_planner::octomap_planner_3d::synthetic
{

class SceneBuilder
{
public:
  explicit SceneBuilder(double res = 0.1) : res_(res), tree_(std::make_unique<octomap::OcTree>(res))
  {
  }

  double res() const { return res_; }

  // Floor over [x_min, x_max) x [y_min, y_max) at height(x, y), with
  // free_cells observed-free voxels on top.
  void floor(
    double x_min, double x_max, double y_min, double y_max,
    const std::function<double(double, double)> & height, int free_cells = 12)
  {
    forEachColumn(x_min, x_max, y_min, y_max, [&](double x, double y) {
      const double h = height(x, y);
      const octomap::OcTreeKey k = tree_->coordToKey(octomap::point3d(x, y, h - res_ / 2));
      tree_->updateNode(k, true);
      for (int i = 1; i <= free_cells; ++i) {
        octomap::OcTreeKey ka = k;
        ka[2] = k[2] + i;
        const auto * n = tree_->search(ka);
        if (n == nullptr || !tree_->isNodeOccupied(n)) {
          tree_->updateNode(ka, false);
        }
      }
    });
  }

  void flatFloor(
    double x_min, double x_max, double y_min, double y_max, double h, int free_cells = 12)
  {
    floor(x_min, x_max, y_min, y_max, [h](double, double) { return h; }, free_cells);
  }

  // A solid block of occupied voxels: every voxel the box intersects, so a
  // 5 cm slab still occupies the voxel it lies in.
  void box(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  {
    forEachColumn(x_min, x_max, y_min, y_max, [&](double x, double y) {
      for (double z = lower(z_min); z < z_max - 1e-9; z += res_) {
        tree_->updateNode(octomap::point3d(x, y, z + res_ / 2), true);
      }
    });
  }

  // Removes every node of the columns in the rectangle between z_min and
  // z_max: a hole in the floor (or in a free band) that was never observed.
  void erase(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  {
    forEachColumn(x_min, x_max, y_min, y_max, [&](double x, double y) {
      for (double z = lower(z_min); z < z_max - 1e-9; z += res_) {
        tree_->deleteNode(octomap::point3d(x, y, z + res_ / 2));
      }
    });
  }

  // Same post-processing as clean_map's writeBinary: max-likelihood values
  // and pruning, so blocks of identical children collapse into one leaf and
  // the build's pruned-leaf expansion is exercised.
  std::unique_ptr<octomap::OcTree> finish()
  {
    tree_->updateInnerOccupancy();
    tree_->toMaxLikelihood();
    tree_->prune();
    return std::move(tree_);
  }

private:
  // Lower face of the voxel containing v.
  double lower(double v) const { return std::floor(v / res_ + 1e-9) * res_; }

  // Every column the rectangle intersects, at the column centres.
  template <typename Fn>
  void forEachColumn(double x_min, double x_max, double y_min, double y_max, Fn && fn)
  {
    for (double y = lower(y_min); y < y_max - 1e-9; y += res_) {
      for (double x = lower(x_min); x < x_max - 1e-9; x += res_) {
        fn(x + res_ / 2, y + res_ / 2);
      }
    }
  }

  double res_;
  std::unique_ptr<octomap::OcTree> tree_;
};

// The named scenes. All are 12 x 8 m with the ground floor at z = 0.
//   flat       a bare floor
//   wall       + a 2 m wall across x = 6 with a 0.8 m door at y 3.6..4.4,
//                and a 0.5 m high table (slab + four legs) at x 2..3.2, y 1..1.8
//   ramp       floor rising 15 deg between x = 4 and x = 8, flat beyond
//   stairs     six 0.17 m risers / 0.29 m treads from x = 4, flat beyond
//   kerb       a 0.15 m step up at x = 6
//   hole       single missing columns, a 1 x 6 crack, a 3 x 3 and a 7 x 7 hole
//              in the floor, and a balcony (second floor at 1.5 m over
//              x >= 9 with the ground floor continuing underneath)
//   two_floors a second floor 5 m above the first over the whole area
inline std::unique_ptr<octomap::OcTree> makeScene(const std::string & name, double res = 0.1)
{
  SceneBuilder b(res);
  const double X = 12.0, Y = 8.0;
  if (name == "flat") {
    b.flatFloor(0, X, 0, Y, 0.0);
  } else if (name == "wall") {
    b.flatFloor(0, X, 0, Y, 0.0);
    b.box(6.0, 6.1, 0.0, 3.6, 0.0, 2.0);
    b.box(6.0, 6.1, 4.4, Y, 0.0, 2.0);
    b.box(2.0, 3.2, 1.0, 1.8, 0.5, 0.55);
    for (double lx : {2.0, 3.1}) {
      for (double ly : {1.0, 1.7}) {
        b.box(lx, lx + 0.1, ly, ly + 0.1, 0.0, 0.5);
      }
    }
  } else if (name == "ramp") {
    const double t = std::tan(15.0 * M_PI / 180.0);
    b.floor(0, X, 0, Y, [t](double x, double) {
      return x < 4.0 ? 0.0 : (x < 8.0 ? (x - 4.0) * t : 4.0 * t);
    });
  } else if (name == "stairs") {
    b.floor(0, X, 0, Y, [](double x, double) {
      if (x < 4.0) {
        return 0.0;
      }
      const int step = std::min(6, static_cast<int>(std::floor((x - 4.0) / 0.29)) + 1);
      return 0.17 * step;
    });
  } else if (name == "kerb") {
    b.floor(0, X, 0, Y, [](double x, double) { return x < 6.0 ? 0.0 : 0.15; });
  } else if (name == "hole") {
    b.flatFloor(0, X, 0, Y, 0.0);
    b.flatFloor(9.0, X, 0, Y, 1.5);
    b.erase(2.0, 2.1, 2.0, 2.1, -0.2, 0.2);  // single columns
    b.erase(2.0, 2.1, 5.0, 5.1, -0.2, 0.2);
    b.erase(5.0, 5.1, 2.0, 2.6, -0.2, 0.2);  // 1 x 6 crack
    b.erase(7.0, 7.3, 3.0, 3.3, -0.2, 0.2);  // 3 x 3
    b.erase(3.0, 3.7, 5.0, 5.7, -0.2, 0.2);  // 7 x 7: stays a hole
  } else if (name == "two_floors") {
    b.flatFloor(0, X, 0, Y, 0.0);
    b.flatFloor(0, X, 0, Y, 5.0);
  } else {
    return nullptr;
  }
  return b.finish();
}

}  // namespace syncai_global_planner::octomap_planner_3d::synthetic

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__SYNTHETIC_OCTREE_HPP_
