// TerrainGrid on synthetic scenes. Each scene is a 4 x 4 m window at 0.05 m
// (the local costmap's) with the robot at its centre on a floor at z = 0,
// sampled the way the lidar samples it -- a few returns per cell, with a
// little noise -- and one feature; the cost expected at the feature and on
// the plain floor beside it is what each test checks. The ROS side
// (TerrainLayer) is the thin part: subscriptions, TF, the costmap write.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_costmap_2d/terrain_grid.hpp"

using syncai_costmap_2d::FREE_SPACE;
using syncai_costmap_2d::LETHAL_OBSTACLE;
using syncai_costmap_2d::NO_INFORMATION;
using syncai_costmap_2d::TerrainGrid;
using syncai_costmap_2d::TerrainPathPose;
using syncai_costmap_2d::TerrainStats;

namespace
{

constexpr double kRes = 0.05;
constexpr int kN = 80;  // 4 m
constexpr double kOrigin = -2.0;
constexpr double kNow = 100.0;

struct Scene
{
  TerrainGrid grid;
  std::mt19937 rng{42};
  std::normal_distribution<double> noise{0.0, 0.01};
  std::vector<unsigned char> cost;

  Scene()
  {
    grid.resize(kN, kN, kRes, kOrigin, kOrigin);
    cost.assign(kN * kN, 0);
  }

  // A surface z = f(x, y) over [x0, x1] x [y0, y1], `per_cell` returns per
  // cell, `age` seconds before kNow.
  template <typename F>
  void surface(double x0, double x1, double y0, double y1, F f, int per_cell = 4, double age = 1.0)
  {
    std::uniform_real_distribution<double> jitter(0.0, kRes);
    for (double y = y0; y < y1; y += kRes) {
      for (double x = x0; x < x1; x += kRes) {
        for (int k = 0; k < per_cell; ++k) {
          const double px = x + jitter(rng), py = y + jitter(rng);
          grid.addSample(px, py, f(px, py) + noise(rng), kNow - age);
        }
      }
    }
  }
  void floor(double x0 = kOrigin, double x1 = -kOrigin, double y0 = kOrigin, double y1 = -kOrigin)
  {
    surface(x0, x1, y0, y1, [](double, double) { return 0.0; });
  }
  // A vertical face at x = xf between y0 and y1, from z0 to z1.
  void wall(double xf, double y0, double y1, double z0, double z1, double age = 1.0)
  {
    for (double y = y0; y < y1; y += kRes / 2) {
      for (double z = z0; z < z1; z += 0.03) {
        grid.addSample(xf, y + kRes / 4, z + noise(rng), kNow - age);
      }
    }
  }
  TerrainStats classify(double robot_ground = 0.0, double robot_x = 0.0, double robot_y = 0.0)
  {
    TerrainStats st;
    grid.classify(kNow, robot_x, robot_y, robot_ground, cost.data(), &st);
    return st;
  }
  unsigned char at(double x, double y) const
  {
    int ix, iy;
    EXPECT_TRUE(grid.worldToCell(x, y, ix, iy));
    return cost[grid.index(ix, iy)];
  }
  // Share of cells in [x0, x1] x [y0, y1] with the given cost.
  double share(double x0, double x1, double y0, double y1, unsigned char c) const
  {
    int n = 0, hit = 0;
    for (double y = y0; y < y1; y += kRes) {
      for (double x = x0; x < x1; x += kRes) {
        ++n;
        if (at(x + kRes / 2, y + kRes / 2) == c) {
          ++hit;
        }
      }
    }
    return n ? static_cast<double>(hit) / n : 0.0;
  }
};

}  // namespace

TEST(TerrainGrid, PlainFloorIsFreeAndUnseenIsUnknown)
{
  Scene s;
  s.floor(kOrigin, 0.0, kOrigin, -kOrigin);  // left half only
  const auto st = s.classify();
  EXPECT_GT(s.share(-1.9, -0.1, -1.9, 1.9, FREE_SPACE), 0.98);
  EXPECT_GT(s.share(0.2, 1.9, -1.9, 1.9, NO_INFORMATION), 0.98);
  EXPECT_EQ(st.lethal_obstacle + st.lethal_step + st.lethal_slope, 0u);
}

TEST(TerrainGrid, KerbIsFreeAndStepIsLethal)
{
  {
    // A 0.10 m kerb across the window at x = 0: within max_step_height.
    Scene s;
    s.surface(
      kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x > 0.0 ? 0.10 : 0.0; });
    s.classify();
    EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  }
  {
    // A 0.30 m step: the edge is lethal, the floor the robot stands on is
    // free, and the other floor -- which it cannot get onto -- is an island
    // (the reach rule), whichever side it stands on.
    Scene s;
    s.surface(
      kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x > 0.0 ? 0.30 : 0.0; });
    auto st = s.classify(0.0, -1.0, 0.0);
    EXPECT_GT(st.lethal_island, 0u);
    EXPECT_GT(s.share(-0.1, 0.1, -1.9, 1.9, LETHAL_OBSTACLE), 0.9);
    EXPECT_GT(s.share(-1.9, -0.4, -1.9, 1.9, FREE_SPACE), 0.98);
    EXPECT_GT(s.share(0.4, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.98);
    st = s.classify(0.3, 1.0, 0.0);
    EXPECT_GT(s.share(-1.9, -0.4, -1.9, 1.9, LETHAL_OBSTACLE), 0.98);
    EXPECT_GT(s.share(0.4, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  }
}

TEST(TerrainGrid, DropOffEdgeIsLethal)
{
  // The floor ends at x = 0.5 and the next one is 1 m down: the upper edge
  // reads a neighbour out of reach (the negative obstacle the obstacle
  // layer's height band could never see), and the floor behind it is free.
  Scene s;
  s.surface(
    kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x > 0.5 ? -1.0 : 0.0; });
  const auto st = s.classify();
  EXPECT_GT(st.lethal_step, 0u);
  EXPECT_GT(s.share(0.4, 0.6, -1.9, 1.9, LETHAL_OBSTACLE), 0.9);
  EXPECT_GT(s.share(-1.9, 0.2, -1.9, 1.9, FREE_SPACE), 0.98);
  // The lower floor is an island from up here.
  EXPECT_GT(s.share(0.8, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.98);
}

TEST(TerrainGrid, RampIsFreeAndSteepSlopeIsLethal)
{
  {
    // 15 deg ramp rising with x: under max_slope_deg, and every step between
    // neighbours is 1.3 cm.
    Scene s;
    const double k = std::tan(15.0 * M_PI / 180.0);
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [k](double x, double) { return k * x; });
    const auto st = s.classify();
    EXPECT_EQ(st.lethal_slope, 0u);
    EXPECT_EQ(st.lethal_step, 0u);
    EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  }
  {
    // 30 deg: the disc under every cell spans more than a step and tilts
    // past the limit.
    Scene s;
    const double k = std::tan(30.0 * M_PI / 180.0);
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [k](double x, double) { return k * x; });
    const auto st = s.classify();
    EXPECT_GT(st.lethal_slope, 0u);
    EXPECT_GT(s.share(-1.5, 1.5, -1.5, 1.5, LETHAL_OBSTACLE), 0.95);
  }
}

TEST(TerrainGrid, LowBoxAndPersonAreLethalOnSeenAndUnseenFloor)
{
  {
    // A 0.3 m box, 0.4 x 0.4 m, at x = 1 on a seen floor: its top is in the
    // band, so are its faces; the floor round it stays free.
    Scene s;
    s.floor();
    s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; });
    s.wall(0.8, -0.2, 0.2, 0.0, 0.30);
    const auto st = s.classify();
    EXPECT_GT(st.lethal_obstacle, 0u);
    EXPECT_GT(s.share(0.8, 1.2, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);
    EXPECT_GT(s.share(-1.9, 0.5, -1.9, 1.9, FREE_SPACE), 0.98);
  }
  {
    // The same box on a floor the lidar never saw (within 3.9 m of a robot
    // that has just started), seen as the lidar sees it: its top only. Its
    // own returns make a ground estimate 0.3 m up, so nothing is in its
    // band; it is an island -- not reachable from the robot's floor over a
    // step -- and lethal for that (the reach rule).
    Scene s;
    s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; });
    const auto st = s.classify(0.0);
    EXPECT_GT(st.lethal_island, 0u);
    EXPECT_GT(s.share(0.8, 1.2, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);
    EXPECT_GT(s.share(-1.9, 0.5, -1.9, 1.9, NO_INFORMATION), 0.98);
  }
  {
    // A person's legs: two columns of returns 0.1-0.9 m up, no floor under
    // them.
    Scene s;
    s.floor();
    for (double z = 0.1; z < 0.9; z += 0.03) {
      s.grid.addSample(1.0, 0.1, z, kNow - 0.5);
      s.grid.addSample(1.0, -0.1, z, kNow - 0.5);
    }
    s.classify();
    EXPECT_EQ(s.at(1.0, 0.1), LETHAL_OBSTACLE);
    EXPECT_EQ(s.at(1.0, -0.1), LETHAL_OBSTACLE);
    EXPECT_EQ(s.at(1.0, 0.5), FREE_SPACE);
  }
}

TEST(TerrainGrid, ObstacleForgetsAfterObstacleMemoryAndGroundRemembersLonger)
{
  Scene s;
  s.floor(kOrigin, -kOrigin, kOrigin, -kOrigin);
  // The floor was seen 6 s ago (inside ground_memory 10 s); a person stood
  // at x = 1 until 3 s ago (outside obstacle_memory 2 s).
  Scene old;
  old.surface(
    kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
  for (double z = 0.1; z < 0.9; z += 0.03) {
    old.grid.addSample(1.0, 0.0, z, kNow - 3.0);
  }
  old.classify();
  EXPECT_EQ(old.at(1.0, 0.0), FREE_SPACE);
  EXPECT_GT(old.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  // 12 s ago: the floor is forgotten too.
  Scene gone;
  gone.surface(
    kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 12.0);
  gone.classify();
  EXPECT_GT(gone.share(-1.9, 1.9, -1.9, 1.9, NO_INFORMATION), 0.98);
}

TEST(TerrainGrid, WallFootIsLethalWhetherOrNotTheFloorUnderItWasSeen)
{
  Scene s;
  s.floor(kOrigin, 1.0, kOrigin, -kOrigin);
  s.wall(1.0, kOrigin, -kOrigin, 0.0, 1.4);
  s.classify();
  EXPECT_GT(s.share(0.95, 1.05, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
  EXPECT_GT(s.share(-1.9, 0.8, -1.9, 1.9, FREE_SPACE), 0.98);
  // Only the wall's upper part seen (the foot hidden): still an obstacle.
  Scene hidden;
  hidden.floor(kOrigin, 0.9, kOrigin, -kOrigin);
  hidden.wall(1.0, kOrigin, -kOrigin, 0.4, 1.4);
  hidden.classify();
  EXPECT_GT(hidden.share(0.95, 1.05, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
}

namespace
{
// A stair across the window: level floor to x = 0, then risers of 0.17 m
// every 0.28 m up to x = 1.4 (five steps), level after. Sampled as the lidar
// sees it: treads only (risers are near-vertical and sparse).
double stairZ(double x)
{
  if (x <= 0.0) {
    return 0.0;
  }
  const int step = std::min(5, static_cast<int>(std::floor(x / 0.28)) + 1);
  return 0.17 * step;
}
std::vector<TerrainPathPose> stairPath()
{
  std::vector<TerrainPathPose> out;
  for (double x = -1.9; x < 1.9; x += 0.1) {
    // A straight stair link: the planner's path climbs the flight on a line.
    const double z = x <= 0.0 ? 0.0 : (x >= 1.4 ? 0.85 : 0.85 * x / 1.4);
    out.push_back({x, 0.0, z});
  }
  return out;
}
}  // namespace

TEST(TerrainGrid, StairIsLethalWithOrWithoutAPathClimbingIt)
{
  // The grid opens no stair (2026-10-20): the controllers' stair mode climbs
  // a flight without reading the costmap. Lethal is what keeps MPPI from
  // walking onto one on its own, outside the stair mode.
  Scene s;
  s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return stairZ(x); });
  s.classify();
  EXPECT_GT(s.share(0.0, 1.4, -1.0, 1.0, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(-1.9, -0.3, -1.9, 1.9, FREE_SPACE), 0.98);

  // A path climbing it (Octo3D's stair link) seeds the reach flood and
  // relaxes nothing: the flight stays as lethal as without it.
  s.grid.setPath(stairPath());
  s.classify();
  EXPECT_GT(s.share(0.0, 1.4, -1.0, 1.0, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(0.1, 1.3, -0.4, 0.4, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(-1.9, -0.3, -1.9, 1.9, FREE_SPACE), 0.98);
}

TEST(TerrainGrid, PathHeightsConnectAnUnseenRampAndIsolateABoxOnIt)
{
  // A 15 deg ramp from x = 0.5, its foot (x < 1.5) never seen: the first
  // seen ramp cells sit 0.27 m over the robot's ground with nothing known
  // between. Without the path they are an island; with the path's heights
  // along the ramp they are the floor the path says they are.
  Scene s;
  const double k = std::tan(15.0 * M_PI / 180.0);
  auto ramp = [k](double x, double) { return x > 0.5 ? k * (x - 0.5) : 0.0; };
  s.surface(1.5, -kOrigin, kOrigin, -kOrigin, ramp);
  s.classify();
  EXPECT_GT(s.share(1.6, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);

  std::vector<TerrainPathPose> path;
  for (double x = -1.9; x < 1.9; x += 0.1) {
    path.push_back({x, 0.0, ramp(x, 0.0)});
  }
  s.grid.setPath(path);
  s.classify();
  EXPECT_GT(s.share(1.6, 1.9, -1.9, 1.9, FREE_SPACE), 0.95);

  // A box on the seen part of the ramp is still an island.
  s.surface(1.6, 1.8, -0.1, 0.1, [&](double x, double y) { return ramp(x, y) + 0.3; });
  const auto st = s.classify();
  EXPECT_GT(st.lethal_island + st.lethal_obstacle, 0u);
  EXPECT_GT(s.share(1.6, 1.8, -0.1, 0.1, LETHAL_OBSTACLE), 0.9);
}

TEST(TerrainGrid, WindowMovesWithTheSamples)
{
  Scene s;
  s.floor();
  s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; });
  s.classify();
  EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
  // The robot walks 1 m in +x: the box is now at 0 in the window's terms,
  // and the far edge of the old window is unknown.
  s.grid.updateOrigin(kOrigin + 1.0, kOrigin);
  s.classify(0.0, 1.0, 0.0);
  EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
  EXPECT_GT(s.share(2.2, 2.9, -1.9, 1.9, NO_INFORMATION), 0.98);
  EXPECT_GT(s.share(-0.9, 0.5, -1.9, 1.9, FREE_SPACE), 0.98);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
