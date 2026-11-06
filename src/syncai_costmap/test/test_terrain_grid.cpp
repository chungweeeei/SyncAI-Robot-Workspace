// TerrainGrid on synthetic scenes. Each scene is a 4 x 4 m window at 0.05 m
// (the local costmap's) with the robot at its centre on a floor at z = 0,
// sampled the way the lidar samples it -- a few returns per cell, with a
// little noise -- and one feature; the cost expected at the feature and on
// the plain floor beside it is what each test checks. The ROS side
// (TerrainLayer) is the thin part: subscriptions, TF, the costmap write.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "syncai_costmap/cost_values.hpp"
#include "syncai_costmap/terrain_grid.hpp"

using syncai_costmap::FREE_SPACE;
using syncai_costmap::LETHAL_OBSTACLE;
using syncai_costmap::NO_INFORMATION;
using syncai_costmap::TerrainGrid;
using syncai_costmap::TerrainPathPose;
using syncai_costmap::TerrainStats;

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
  // The rule, not the view: the scene samples the lower floor right up to
  // the edge, which the robot's own lidar never sees (next test).
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

// The same 1 m drop as the robot's lidar sees it (2026-11): from 0.481 m up
// with a -7 deg lower FOV the lower floor is in view only from d / tan 7 deg
// (~8.1 m here) past the edge, outside this 4 m window, so only the upper
// floor has returns. The edge has no lower neighbour, so it is not lethal,
// and past it is unknown -- FREE to MPPI. A known limit (README, "A drop-off
// is mostly unseen"): this test pins it, so a change that fixes it shows.
TEST(TerrainGrid, DropOffInItsOwnShadowIsNotSeen)
{
  Scene s;
  s.surface(kOrigin, 0.5, kOrigin, -kOrigin, [](double, double) { return 0.0; });
  const auto st = s.classify();
  EXPECT_EQ(st.lethal_step, 0u);
  EXPECT_LT(s.share(0.35, 0.5, -1.9, 1.9, LETHAL_OBSTACLE), 0.05);
  EXPECT_GT(s.share(0.6, 1.9, -1.9, 1.9, NO_INFORMATION), 0.98);
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
  old.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
  for (double z = 0.1; z < 0.9; z += 0.03) {
    old.grid.addSample(1.0, 0.0, z, kNow - 3.0);
  }
  old.classify();
  EXPECT_EQ(old.at(1.0, 0.0), FREE_SPACE);
  EXPECT_GT(old.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  // 12 s ago: the floor is forgotten too.
  Scene gone;
  gone.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 12.0);
  gone.classify();
  EXPECT_GT(gone.share(-1.9, 1.9, -1.9, 1.9, NO_INFORMATION), 0.98);
}

TEST(TerrainGrid, WallFootIsLethalWhetherOrNotTheFloorUnderItWasSeen)
{
  Scene s;
  s.floor(kOrigin, 1.0, kOrigin, -kOrigin);
  s.wall(1.0, kOrigin, -kOrigin, 0.0, 1.4);
  s.classify();
  // Lethal from the wall's own cell; the floor is free up to its face. Until
  // 2026-11-03 the 0.15 m before it was slope-lethal too, its grounds
  // borrowed from the wall's returns -- the band the slope fit's own-support
  // rule ended.
  EXPECT_GT(s.share(1.0, 1.05, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
  EXPECT_GT(s.share(-1.9, 0.95, -1.9, 1.9, FREE_SPACE), 0.98);
  // Only the wall's upper part seen (the foot hidden, the floor to 0.9):
  // the wall and the unseen strip before it are lethal.
  Scene hidden;
  hidden.floor(kOrigin, 0.9, kOrigin, -kOrigin);
  hidden.wall(1.0, kOrigin, -kOrigin, 0.4, 1.4);
  hidden.classify();
  EXPECT_GT(hidden.share(0.95, 1.05, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
}

TEST(TerrainGrid, ADoorsJambsDoNotTiltTheFloorBetweenThem)
{
  // A wall at x = 1 with a 0.75 m door (dp's 2F lab door), the floor beyond
  // 5 cm higher, and the wall's and the jambs' feet hidden below 0.15 m, as
  // the lower FOV leaves them. The jambs are lethal; the lane between them
  // must not be walled off by the slope rule fitting their returns as
  // ground. With the jambs in the fit (before 2026-11-02): 0.958 of a 0.6 m
  // lane free and 0.345 of the strip in front of the wall; with the
  // obstacle cells out, 1.000 and 0.667; with the unsupported grounds out
  // too (2026-11-03), the door's whole width and the strip.
  constexpr double kHalf = 0.375;
  Scene s;
  const auto floor_z = [](double x, double) { return x > 1.0 ? 0.05 : 0.0; };
  s.surface(kOrigin, 0.95, kOrigin, -kOrigin, floor_z);
  s.surface(0.95, 1.2, -kHalf, kHalf, floor_z);  // the doorway
  s.surface(1.2, -kOrigin, kOrigin, -kOrigin, floor_z);
  s.wall(1.0, kOrigin, -kHalf, 0.15, 1.4);
  s.wall(1.0, kHalf, -kOrigin, 0.15, 1.4);
  for (double x = 1.0; x < 1.15; x += kRes / 2) {
    for (const double y : {-kHalf, kHalf}) {
      for (double z = 0.15; z < 1.4; z += 0.03) {
        s.grid.addSample(x, y, z + s.noise(s.rng), kNow - 1.0);
      }
    }
  }
  s.classify();
  EXPECT_GT(s.share(1.0, 1.05, 0.45, 0.6, LETHAL_OBSTACLE), 0.9);  // the wall either side
  EXPECT_GT(s.share(1.0, 1.05, -0.6, -0.45, LETHAL_OBSTACLE), 0.9);
  EXPECT_GT(s.share(0.9, 1.1, -0.35, 0.35, FREE_SPACE), 0.98);  // the door's width, to a cell
  EXPECT_GT(s.share(0.75, 0.95, 0.5, 1.9, FREE_SPACE), 0.95);   // in front of the wall
}

TEST(TerrainGrid, OverheadUndersideIsNeitherGroundNorObstacle)
{
  // dp's lobby stair on robot01, 2026-10-22: the robot walked under its
  // underside, 1.0-1.4 m over the floor. The floor was seen seconds before,
  // from further off, a few returns a cell; the underside from right below,
  // more returns a cell than a ring holds. They pushed the floor out of the
  // rings and became the ground there: islands and steps round the robot,
  // and MPPI with every candidate colliding.
  auto underside = [](double x, double) { return 1.0 + 0.4 * (x + 1.0) / 2.9; };
  {
    Scene s;
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 5.0);
    s.surface(-1.0, -kOrigin, kOrigin, -kOrigin, underside, 40, 0.5);
    auto st = s.classify();
    EXPECT_EQ(st.lethal_island + st.lethal_step + st.lethal_slope + st.lethal_obstacle, 0u);
    EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
    int ix, iy;
    ASSERT_TRUE(s.grid.worldToCell(1.0, 0.0, ix, iy));
    EXPECT_NEAR(s.grid.lastGround()[s.grid.index(ix, iy)], 0.0, 0.05);

    // Under it, a box is still a box.
    s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; }, 4, 0.5);
    s.wall(0.8, -0.2, 0.2, 0.0, 0.30, 0.5);
    st = s.classify();
    EXPECT_GT(s.share(0.8, 1.2, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);
    EXPECT_GT(s.share(-1.9, 0.5, -1.9, 1.9, FREE_SPACE), 0.98);
  }
  {
    // Seen the other way round -- the underside first, so it went into the
    // rings, then the floor: the estimate leaves it out, and the ground is
    // the floor's, not lifted by max_ground_lift.
    Scene s;
    s.surface(-1.0, -kOrigin, kOrigin, -kOrigin, underside, 40, 3.0);
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 1.0);
    s.classify();
    EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
    int ix, iy;
    ASSERT_TRUE(s.grid.worldToCell(1.0, 0.0, ix, iy));
    EXPECT_NEAR(s.grid.lastGround()[s.grid.index(ix, iy)], 0.0, 0.05);
  }
  {
    // An underside lower than body_clearance -- a stair's foot, 0.5 m up --
    // is not overhead: the robot does not fit under it.
    Scene s;
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 5.0);
    s.surface(1.0, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.5; }, 40, 0.5);
    s.classify();
    EXPECT_GT(s.share(1.05, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
    EXPECT_GT(s.share(-1.9, 0.5, -1.9, 1.9, FREE_SPACE), 0.98);
  }
}

TEST(TerrainGrid, OverheadWithNoFloorSeenUnderItIsStillASurface)
{
  // The limit: overhead is judged against a floor return of the same
  // neighbourhood younger than ground_memory. With none, the underside is
  // the lowest thing there, a surface 1.2 m up and an island; the robot's
  // own floor beside it stays free.
  Scene s;
  s.floor(kOrigin, -1.0, kOrigin, -kOrigin);
  s.surface(-0.5, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 1.2; }, 40, 0.5);
  const auto st = s.classify(0.0, -1.5, 0.0);
  EXPECT_GT(st.lethal_island, 0u);
  EXPECT_GT(s.share(-0.4, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.98);
  EXPECT_GT(s.share(-1.9, -1.2, -1.9, 1.9, FREE_SPACE), 0.98);
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
// The planner's path up it, marked as its stair-gait span (`stair`) or not.
std::vector<TerrainPathPose> stairPath(bool stair = true)
{
  std::vector<TerrainPathPose> out;
  for (double x = -1.9; x < 1.9; x += 0.1) {
    // A straight stair link: the planner's path climbs the flight on a line.
    const double z = x <= 0.0 ? 0.0 : (x >= 1.4 ? 0.85 : 0.85 * x / 1.4);
    out.push_back({x, 0.0, z, stair});
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

  // A path climbing it (Octo3D's stair link), with the planner's word that
  // it is a stair: seeds the reach flood and opens nothing, the flight stays
  // as lethal as without it. And a path the planner has not spoken for (no
  // corridor) opens nothing either, whatever its poses are marked.
  s.grid.setPath(stairPath());
  auto st = s.classify();
  EXPECT_GT(s.share(0.0, 1.4, -1.0, 1.0, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(0.1, 1.3, -0.4, 0.4, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(-1.9, -0.3, -1.9, 1.9, FREE_SPACE), 0.98);
  EXPECT_EQ(st.on_plan, 0u);
  s.grid.setPath(stairPath(false), false);
  st = s.classify();
  EXPECT_GT(s.share(0.0, 1.4, -1.0, 1.0, LETHAL_OBSTACLE), 0.5);
  EXPECT_GT(s.share(0.1, 1.3, -0.4, 0.4, LETHAL_OBSTACLE), 0.5);
  EXPECT_EQ(st.on_plan, 0u);
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

// The shift swaps cells through a second buffer (2026-11) instead of
// building a new grid each time: a cell that left the window must come back
// empty, never with the samples the buffer last held.
TEST(TerrainGrid, WindowShiftedCellByCellAndBackForgetsWhatLeft)
{
  Scene s;
  s.floor();
  s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; });
  s.surface(-1.9, -1.5, -0.2, 0.2, [](double, double) { return 0.30; });
  s.classify();
  EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
  EXPECT_EQ(s.at(-1.7, 0.0), LETHAL_OBSTACLE);
  // 1 m in +x one cell at a time, diagonally for the last half, then back.
  double ox = kOrigin, oy = kOrigin;
  for (int k = 0; k < 20; ++k) {
    ox += kRes;
    oy += k >= 10 ? kRes : 0.0;
    s.grid.updateOrigin(ox, oy);
  }
  for (int k = 0; k < 20; ++k) {
    ox -= kRes;
    oy -= k < 10 ? kRes : 0.0;
    s.grid.updateOrigin(ox, oy);
  }
  s.classify();
  // The box that stayed in the window is where it was; the one that left
  // (x < -1 m after the first leg) and the floor round it came back unknown.
  EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
  EXPECT_GT(s.share(-1.95, -1.05, -1.45, 1.45, NO_INFORMATION), 0.98);
  EXPECT_GT(s.share(-0.9, 0.5, -1.4, 1.4, FREE_SPACE), 0.98);
}

// Ray clearing (2026-10-23). The lidar is at the robot's centre, 0.481 m up
// (TerrainLayer's sensor_height); a "scan" is a fan of rays from it to the
// given returns, each clearing along its way, at kNow.
namespace
{
constexpr double kLidarZ = 0.481;
// Cell centres: the legs stand on them, and the rays aimed through them
// cross those cells and no other.
constexpr double kLegX = 1.025;
constexpr double kLegY = 0.125;

void ray(Scene & s, double x, double y, double z, double age = 0.0)
{
  s.grid.clearRay(0.0, 0.0, kLidarZ, x, y, z, kNow - age);
}
// A person's two legs at (kLegX, +-kLegY), returns from z0 up, `n` of them
// per leg, `age` seconds old.
void legs(Scene & s, double age, double z0 = 0.30, int n = 20)
{
  for (int k = 0; k < n; ++k) {
    const double z = z0 + 0.03 * k;
    s.grid.addSample(kLegX, kLegY, z, kNow - age);
    s.grid.addSample(kLegX, -kLegY, z, kNow - age);
  }
}
// Rays through both leg cells to a wall at x = 1.9 (0.875 m past them, more
// than the end margin), at every height from the floor to 1.4 m: what the
// lidar sees once the person has walked on.
void seeThroughLegs(Scene & s)
{
  for (double ze = 0.0; ze < 1.4; ze += 0.02) {
    ray(s, 1.9, kLegY * 1.9 / kLegX, ze);
    ray(s, 1.9, -kLegY * 1.9 / kLegX, ze);
  }
}
}  // namespace

TEST(TerrainGrid, RayClearingForgetsAPersonWhoWalkedOnAtTheNextScan)
{
  // The floor seen 6 s ago; the person stood here half a second ago, well
  // inside obstacle_memory.
  Scene s;
  s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
  legs(s, 0.5);
  s.classify();
  EXPECT_EQ(s.at(kLegX, kLegY), LETHAL_OBSTACLE);
  EXPECT_EQ(s.at(kLegX, -kLegY), LETHAL_OBSTACLE);
  const int before = s.grid.sampleCount(
    static_cast<int>((kLegX - kOrigin) / kRes), static_cast<int>((kLegY - kOrigin) / kRes));
  // The floor's returns are in the ring too, and the leg's above 0.75 m
  // over them are overhead, not stored.
  EXPECT_GE(before, 16);

  // The next scan sees through to the wall behind: free at once, the floor
  // round them untouched.
  seeThroughLegs(s);
  s.classify();
  EXPECT_EQ(s.at(kLegX, kLegY), FREE_SPACE);
  EXPECT_EQ(s.at(kLegX, -kLegY), FREE_SPACE);
  EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  const int after = s.grid.sampleCount(
    static_cast<int>((kLegX - kOrigin) / kRes), static_cast<int>((kLegY - kOrigin) / kRes));
  EXPECT_LE(after, 4);
  EXPECT_GE(s.grid.clearedSamples(), 30u);

  // Clearing off (tolerance 0): the memory alone, as before.
  Scene off;
  off.grid.params.ray_clear_tolerance = 0.0;
  off.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
  legs(off, 0.5);
  seeThroughLegs(off);
  off.classify();
  EXPECT_EQ(off.at(kLegX, kLegY), LETHAL_OBSTACLE);
  EXPECT_EQ(off.grid.clearedSamples(), 0u);
}

TEST(TerrainGrid, RayClearingRemovesAPersonsGhostFromTheGround)
{
  // The robot stood in front of the person for longer than the floor memory
  // round them lasted: no floor within reach of the window any more, their
  // legs' returns -- the lowest 0.30 m up, the lidar's lower FOV edge at
  // 1.5 m -- the only ground evidence there, 3 s old (past obstacle_memory).
  // Without clearing that is an island 0.45 m up, lethal for ground_memory.
  Scene s;
  legs(s, 3.0);
  const auto st = s.classify();
  EXPECT_EQ(s.at(kLegX, kLegY), LETHAL_OBSTACLE);
  EXPECT_GT(st.lethal_island, 0u);

  // The scan that sees through where they stood takes the ghost with it:
  // the cells are unseen floor again.
  seeThroughLegs(s);
  s.classify();
  EXPECT_EQ(s.at(kLegX, kLegY), NO_INFORMATION);
  EXPECT_EQ(s.at(kLegX, -kLegY), NO_INFORMATION);
}

TEST(TerrainGrid, RayClearingSparesTheSurfacesARayPassesOver)
{
  // Rays to the floor itself, from the lidar's height, at every endpoint
  // the window holds: every one grazes the floor before it ends on it, and
  // none of them is evidence against it.
  {
    Scene s;
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
    s.classify();
    for (double ye = -1.9; ye < 1.9; ye += 0.05) {
      for (double xe = 0.5; xe < 1.95; xe += 0.05) {
        ray(s, xe, ye, 0.0);
      }
    }
    EXPECT_EQ(s.grid.clearedSamples(), 0u);
    s.classify();
    EXPECT_GT(s.share(-1.9, 1.9, -1.9, 1.9, FREE_SPACE), 0.98);
  }
  {
    // A 0.2 m drop-off at x = 1: the rays to the lower floor pass just over
    // the upper floor's edge (from 0.481 m up, the lower floor is in view
    // from 1.42 m on). The edge keeps its floor and stays lethal.
    Scene s;
    s.surface(
      kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x >= 1.0 ? -0.2 : 0.0; },
      4, 6.0);
    s.classify();
    EXPECT_GT(s.share(0.9, 1.0, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
    for (double ye = -1.9; ye < 1.9; ye += 0.05) {
      for (double xe = 1.45; xe < 1.95; xe += 0.05) {
        ray(s, xe, ye, -0.2);
      }
    }
    // The rays that just clear the edge pass under the edge's returns with
    // positive noise and take those: a couple per row of 76, never a cell's
    // worth. The edge keeps a ground estimate, and the step with it.
    EXPECT_LT(s.grid.clearedSamples(), 4u * 76u);
    s.classify();
    EXPECT_GT(s.share(0.9, 1.0, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
    EXPECT_GT(s.share(-1.9, 0.8, -1.9, 1.9, FREE_SPACE), 0.98);
    EXPECT_GT(s.share(1.0, 1.9, -1.9, 1.9, LETHAL_OBSTACLE), 0.95);
  }
  {
    // A ramp from x = 0.5 up to 0.4 m at x = 1.9, seen along it: the rays to
    // its upper part run just over its lower part. Still free, still a ramp.
    auto ramp = [](double x, double) { return x > 0.5 ? (x - 0.5) * 0.4 / 1.4 : 0.0; };
    Scene s;
    s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, ramp, 4, 6.0);
    s.classify();
    for (double ye = -0.5; ye < 0.5; ye += 0.05) {
      for (double xe = 0.6; xe < 1.95; xe += 0.05) {
        ray(s, xe, ye, ramp(xe, ye));
      }
    }
    EXPECT_EQ(s.grid.clearedSamples(), 0u);
    s.classify();
    EXPECT_GT(s.share(0.6, 1.9, -0.5, 0.5, FREE_SPACE), 0.95);
  }
  {
    // A box top at 0.30 m with rays passing over it to the wall behind:
    // the top is its ground, passed over, kept; the box stays lethal.
    Scene s;
    s.floor();
    s.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; }, 4, 0.5);
    s.classify();
    EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
    for (double ye = -0.2; ye < 0.2; ye += 0.025) {
      for (double ze = 0.35; ze < 1.4; ze += 0.02) {
        ray(s, 1.9, ye * 1.9, ze);
      }
    }
    EXPECT_EQ(s.grid.clearedSamples(), 0u);
    s.classify();
    EXPECT_EQ(s.at(1.0, 0.0), LETHAL_OBSTACLE);
  }
}

TEST(TerrainGrid, RayClearingStopsShortOfTheRaysEnd)
{
  // A person 0.3 m in front of the wall they walked away from: the rays
  // through where they stood end inside the margin, and clear nothing --
  // obstacle_memory is what forgets them.
  Scene s;
  s.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; }, 4, 6.0);
  legs(s, 0.5);
  for (double ze = 0.0; ze < 1.4; ze += 0.02) {
    ray(s, 1.325, kLegY * 1.325 / kLegX, ze);
  }
  EXPECT_EQ(s.grid.clearedSamples(), 0u);
  s.classify();
  EXPECT_EQ(s.at(kLegX, kLegY), LETHAL_OBSTACLE);
}

namespace
{
// One return in every `every` cells over [x0, x1] x [y0, y1]: the cells the
// lidar has only just begun to see, too few for a ground of their own.
template <typename F>
void sparse(Scene & s, double x0, double x1, double y0, double y1, F f, int every)
{
  std::uniform_real_distribution<double> jitter(0.0, kRes);
  int n = 0;
  for (double y = y0; y < y1; y += kRes) {
    for (double x = x0; x < x1; x += kRes) {
      if (n++ % every == 0) {
        const double px = x + jitter(s.rng), py = y + jitter(s.rng);
        s.grid.addSample(px, py, f(px, py) + s.noise(s.rng), kNow - 1.0);
      }
    }
  }
}

// dp's arched ramp as robot01 came to it (2026-10-24): the floor and the
// ramp's lower part seen, the next 0.3 m of it only just coming into view,
// nothing beyond yet.
template <typename F>
void rampScene(Scene & s, F ramp)
{
  s.surface(kOrigin, 1.2, kOrigin, -kOrigin, ramp);
  sparse(s, 1.2, 1.5, kOrigin, -kOrigin, ramp, 3);
}
}  // namespace

TEST(TerrainGrid, SparseCellsOnARampAreJudgedOnTheWalkableGroundBesideThem)
{
  // 0.25 m over 1 m from x = 0.5 (14 deg), level after. Each sparse cell,
  // judged against the robot's ground, has its returns 0.17-0.25 m up: the
  // lethal band robot01's replay showed. Judged against the ramp cell beside
  // it, which the robot walks onto, nothing is in the band.
  auto ramp = [](double x, double) { return std::clamp(0.25 * (x - 0.5), 0.0, 0.25); };
  {
    Scene s;
    s.grid.params.sparse_ground_radius = 0.0;  // as before
    rampScene(s, ramp);
    s.classify();
    // Every cell with a return (one in three) is lethal.
    EXPECT_GT(s.share(1.2, 1.5, -1.9, 1.9, LETHAL_OBSTACLE), 0.1);
  }
  Scene s;
  rampScene(s, ramp);
  const auto st = s.classify();
  EXPECT_EQ(s.share(1.2, 1.5, -1.9, 1.9, LETHAL_OBSTACLE), 0.0);
  EXPECT_EQ(st.lethal_obstacle, 0u);
}

TEST(TerrainGrid, SparseCellsAreNeverJudgedOnABoxTopOrAStairTread)
{
  // The reference is ground the robot can walk onto from its own cell, so a
  // box top (an island) or a flight's treads (reached only over the path's
  // seeds) never is one: each scene costs exactly what it did with the
  // reference off. Estimating a sparse cell's ground from a wider
  // neighbourhood instead took 78 % of the box and 33 % of the flight off
  // lethal.
  const auto costs = [](double radius, auto build) {
    Scene s;
    s.grid.params.sparse_ground_radius = radius;
    build(s);
    s.classify();
    return s.cost;
  };
  // A 0.25 m box on floor never seen, its top sparse.
  const auto box = [](Scene & s) {
    sparse(s, 0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.25; }, 2);
  };
  // The same box on a seen floor.
  const auto box_on_floor = [](Scene & s) {
    s.floor();
    sparse(s, 0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.25; }, 3);
  };
  // A flight seen from its foot, its first treads sparse, with and without
  // the stair link's path climbing it.
  const auto flight = [](Scene & s) {
    sparse(s, -0.2, 0.56, kOrigin, -kOrigin, [](double x, double) { return stairZ(x); }, 3);
    s.surface(0.56, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return stairZ(x); });
  };
  const auto flight_with_path = [&](Scene & s) {
    flight(s);
    s.grid.setPath(stairPath());
  };
  const double r = syncai_costmap::TerrainParams().sparse_ground_radius;
  ASSERT_GT(r, 0.0);
  EXPECT_EQ(costs(r, box), costs(0.0, box));
  EXPECT_EQ(costs(r, box_on_floor), costs(0.0, box_on_floor));
  EXPECT_EQ(costs(r, flight), costs(0.0, flight));
  EXPECT_EQ(costs(r, flight_with_path), costs(0.0, flight_with_path));
  // And they are lethal to begin with.
  Scene s;
  box(s);
  s.classify();
  EXPECT_GT(s.share(0.8, 1.2, -0.2, 0.2, LETHAL_OBSTACLE), 0.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TerrainGrid, ThePlansCorridorOpensItsSillButNotAPersonOrABoxOnIt)
{
  // A 0.3 m sill across the window at x = 0 (dp's lab door as the map read
  // it): the step rule walls it off. The plan over it at y = 0, its poses
  // along x at the floors' heights (setPath()), opens a corridor of
  // plan_radius either side once the planner has spoken for it (the
  // corridor flag); the step beyond the corridor stays lethal, a person
  // standing on the corridor is lethal all the same, a box on the sill's
  // upper landing is lethal -- the band there starts at that landing's
  // floor, not the sill's rise over the lower one, which the crossing
  // waiver of 2026-11-02 lifted it by -- and a plan on a floor far above
  // opens nothing here.
  Scene s;
  s.surface(
    kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x > 0.0 ? 0.30 : 0.0; });
  std::vector<TerrainPathPose> lane;
  for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.1) {
    lane.push_back({x, 0.0, x > 0.0 ? 0.30 : 0.0, false});
  }
  s.grid.setPath(lane, false);  // seeds the far floor's reach; no word from the planner yet
  auto st = s.classify(0.0, -1.0, 0.0);
  EXPECT_GT(s.share(-0.1, 0.1, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);
  EXPECT_EQ(st.on_plan, 0u);

  s.grid.setPath(lane);
  st = s.classify(0.0, -1.0, 0.0);
  EXPECT_GT(st.on_plan, 0u);
  EXPECT_GT(s.share(-0.1, 0.1, -0.3, 0.3, FREE_SPACE), 0.98);     // the corridor over the sill
  EXPECT_GT(s.share(-0.1, 0.1, 0.6, 1.9, LETHAL_OBSTACLE), 0.9);  // the step beside it
  EXPECT_GT(s.share(-0.1, 0.1, -1.9, -0.6, LETHAL_OBSTACLE), 0.9);

  // A person's legs on the corridor: the obstacle rule is not waived.
  for (double z = 0.35; z < 1.0; z += 0.05) {
    for (const double ly : {-0.1, 0.1}) {
      s.grid.addSample(0.6, ly, z, kNow - 0.5);
    }
  }
  s.classify(0.0, -1.0, 0.0);
  EXPECT_EQ(s.at(0.6, 0.1), LETHAL_OBSTACLE);

  // A 0.35 m box on the upper landing, on the corridor: 0.35 over the
  // landing's floor, an obstacle (under a band lifted by the sill's 0.30 m
  // rise it stood 0.05 m short of one).
  s.surface(0.8, 1.0, -0.1, 0.1, [](double, double) { return 0.30 + 0.35; });
  s.classify(0.0, -1.0, 0.0);
  EXPECT_GT(s.share(0.8, 1.0, -0.1, 0.1, LETHAL_OBSTACLE), 0.9);

  // The same plan 3 m up (a floor above in the same columns): not this one.
  Scene up;
  up.surface(
    kOrigin, -kOrigin, kOrigin, -kOrigin, [](double x, double) { return x > 0.0 ? 0.30 : 0.0; });
  std::vector<TerrainPathPose> high = lane;
  for (auto & p : high) {
    p.z += 3.0;
  }
  up.grid.setPath(high);
  st = up.classify(0.0, -1.0, 0.0);
  EXPECT_EQ(st.on_plan, 0u);
  EXPECT_GT(up.share(-0.1, 0.1, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);

  // A 15 deg ramp the plan runs up, seen whole: the corridor is free along
  // it, and the band on it starts at the plan's height within two cells of
  // the nearest pose -- not the highest within plan_radius, 0.1 m further
  // up the ramp -- so a 0.2 m box a metre up is still an obstacle.
  Scene ramp;
  const double k = std::tan(15.0 * M_PI / 180.0);
  auto rz = [k](double x, double) { return x > 0.0 ? k * x : 0.0; };
  ramp.surface(kOrigin, -kOrigin, kOrigin, -kOrigin, rz);
  std::vector<TerrainPathPose> up_the_ramp;
  for (double x = -1.9; x < 1.9; x += 0.1) {
    up_the_ramp.push_back({x, 0.0, rz(x, 0.0), false});
  }
  ramp.grid.setPath(up_the_ramp);
  ramp.classify(0.0, -1.0, 0.0);
  EXPECT_GT(ramp.share(-0.1, 1.5, -0.3, 0.3, FREE_SPACE), 0.95);
  ramp.surface(1.0, 1.2, -0.1, 0.1, [&](double x, double y) { return rz(x, y) + 0.20; });
  ramp.classify(0.0, -1.0, 0.0);
  EXPECT_GT(ramp.share(1.0, 1.2, -0.1, 0.1, LETHAL_OBSTACLE), 0.9);
}

TEST(TerrainGrid, ARayThroughACellBoundsItsGround)
{
  // The hidden wall foot again (floor to 0.9, the wall from 0.4 up), with
  // the scan's rays onto the wall from the lidar at the robot's centre: the
  // cell before the wall, whose ground was the wall's returns next door, is
  // crossed 0.4 m up -- empty there -- so it holds no ground at all now
  // (unknown, not lethal), and the wall's own cell, where the rays end,
  // keeps its returns.
  Scene s;
  s.floor(kOrigin, 0.9, kOrigin, -kOrigin);
  s.wall(1.0, kOrigin, -kOrigin, 0.4, 1.4);
  for (double y = -1.9; y < 1.9; y += 0.025) {
    for (double z = 0.4; z < 0.8; z += 0.05) {
      ray(s, 1.0, y, z);
    }
  }
  s.classify();
  EXPECT_GT(s.share(1.0, 1.05, -1.8, 1.8, LETHAL_OBSTACLE), 0.95);
  EXPECT_GT(s.share(0.95, 1.0, -1.8, 1.8, NO_INFORMATION), 0.9);
  EXPECT_GT(s.share(-1.8, 0.95, -1.8, 1.8, FREE_SPACE), 0.98);

  // The lane beside a jamb whose foot is hidden, the doorway's floor unseen
  // (dp's lab door as the lidar sees it from a metre): the lane cells next
  // to the jamb are islands on its returns until the rays through the door
  // -- to the wall and the floor beyond -- cross them low.
  constexpr double kHalf = 0.375;
  const auto door = [](Scene & d) {
    d.surface(kOrigin, 0.85, kOrigin, -kOrigin, [](double, double) { return 0.0; });
    d.surface(1.15, -kOrigin, kOrigin, -kOrigin, [](double, double) { return 0.0; });
    d.wall(1.0, kOrigin, -kHalf, 0.15, 1.4);
    d.wall(1.0, kHalf, -kOrigin, 0.15, 1.4);
    for (double x = 1.0; x < 1.15; x += kRes / 2) {
      for (const double y : {-kHalf, kHalf}) {
        for (double z = 0.15; z < 1.4; z += 0.03) {
          d.grid.addSample(x, y, z + d.noise(d.rng), kNow - 1.0);
        }
      }
    }
  };
  Scene blind;
  door(blind);
  blind.classify();
  EXPECT_GT(blind.share(1.0, 1.1, 0.3, 0.35, LETHAL_OBSTACLE), 0.9);
  Scene seen;
  door(seen);
  for (double y = -0.3; y <= 0.3; y += 0.025) {
    for (double z = 0.2; z < 0.6; z += 0.05) {
      ray(seen, 1.8, y * 1.8, z);
    }
    for (double xf = 1.3; xf < 1.9; xf += 0.1) {
      ray(seen, xf, y * xf, 0.0);
    }
  }
  seen.classify();
  EXPECT_LT(seen.share(1.0, 1.1, 0.3, 0.35, LETHAL_OBSTACLE), 0.1);
  EXPECT_GT(seen.share(1.0, 1.05, 0.35, 0.4, LETHAL_OBSTACLE), 0.9);  // the jamb itself

  // Nothing a ray passes over is touched: a box under the rays to a wall
  // behind it keeps its top, and is lethal as before.
  Scene box;
  box.floor();
  box.surface(0.8, 1.2, -0.2, 0.2, [](double, double) { return 0.30; });
  box.wall(0.8, -0.2, 0.2, 0.0, 0.30);
  for (double y = -0.3; y <= 0.3; y += 0.025) {
    for (double z = 0.35; z < 0.7; z += 0.05) {
      ray(box, 1.8, y * 1.8, z);
    }
  }
  box.classify();
  EXPECT_GT(box.share(0.8, 1.2, -0.2, 0.2, LETHAL_OBSTACLE), 0.9);

  // The cell a ray ends in is left alone whichever way the ray comes
  // (2026-11-07). A post of returns 0.4-0.75 m up in the one cell at
  // (1.0-1.05, 1.0-1.05), nothing seen round it, and a single ray at 45 deg
  // ending on its lowest return in the far half of that cell: the ray
  // enters the cell res * sqrt 2 before its end, and the walk used to stop
  // a distance res short of the end rather than at the end cell, so the
  // bound was written into the cell the return landed in -- every return
  // above the ray's own height left the estimate, and the post had no ground
  // of its own (the robot's, 0). Its ground is its lowest return plus
  // max_ground_lift now, as a wall's is.
  Scene post;
  post.floor(kOrigin, 0.9, kOrigin, -kOrigin);
  for (double z = 0.4; z < 0.75; z += 0.03) {
    post.grid.addSample(1.045, 1.045, z, kNow - 1.0);
  }
  ray(post, 1.045, 1.045, 0.4);
  post.classify();
  int ix, iy;
  ASSERT_TRUE(post.grid.worldToCell(1.02, 1.02, ix, iy));
  EXPECT_NEAR(post.grid.lastGround()[post.grid.index(ix, iy)], 0.49, 0.07);
}
