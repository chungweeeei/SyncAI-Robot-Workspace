// PathProgress / firstLocalMinimum on synthetic patrol routes, 0.1 m between
// poses: a straight line, an out-and-back whose two legs lie on top of each
// other, a closed loop ending on its start, and a stair route whose upper floor
// is over the lower one. Each case walks a robot along the route the way the
// controller would see it and checks the place never jumps to a later leg.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "syncai_util/path_progress.hpp"

using syncai_util::PathProgress;

namespace
{

constexpr double kStep = 0.1;

geometry_msgs::msg::PoseStamped poseAt(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  ps.pose.orientation.w = 1.0;
  return ps;
}

geometry_msgs::msg::Pose robotAt(double x, double y, double z = 0.0)
{
  return poseAt(x, y, z).pose;
}

// Straight segments through `corners`, sampled every kStep, corners included
// once.
nav_msgs::msg::Path polyline(const std::vector<std::vector<double>> & corners, int stamp_sec = 1)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.header.stamp.sec = stamp_sec;
  for (size_t c = 0; c + 1 < corners.size(); ++c) {
    const auto & a = corners[c];
    const auto & b = corners[c + 1];
    const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
    const int n = static_cast<int>(std::round(len / kStep));
    for (int i = (c == 0 ? 0 : 1); i <= n; ++i) {
      const double t = n == 0 ? 0.0 : static_cast<double>(i) / n;
      const double za = a.size() > 2 ? a[2] : 0.0;
      const double zb = b.size() > 2 ? b[2] : 0.0;
      p.poses.push_back(
        poseAt(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), za + t * (zb - za)));
    }
  }
  return p;
}

}  // namespace

TEST(PathProgress, FollowsAStraightLine)
{
  const auto path = polyline({{0, 0}, {10, 0}});
  PathProgress progress;
  for (int i = 0; i <= 40; ++i) {
    const double x = 0.25 * i;
    const size_t idx = progress.update(path, robotAt(x, 0.2), false);
    EXPECT_NEAR(path.poses[idx].pose.position.x, x, 0.051) << "robot at x " << x;
  }
}

TEST(PathProgress, CatchesUpWithARobotOffToOneSide)
{
  // An MPPI detour: 1 m to the side and 3 m along between two updates.
  const auto path = polyline({{0, 0}, {10, 0}});
  PathProgress progress;
  progress.update(path, robotAt(0, 0), false);
  const size_t idx = progress.update(path, robotAt(3.0, 1.0), false);
  EXPECT_NEAR(path.poses[idx].pose.position.x, 3.0, 0.051);
}

TEST(PathProgress, OutAndBackDoesNotJumpToTheReturnLeg)
{
  // A -> B -> A along y = 0: the return leg lies exactly on the outgoing one.
  // The whole-path nearest pose of a robot 2 m short of B is ambiguous; the
  // place must stay on the outgoing leg until the robot is at the tip.
  const auto path = polyline({{0, 0}, {6, 0}, {0, 0}});
  const size_t tip = 60;
  PathProgress progress;
  for (int i = 0; i <= 27; ++i) {
    const double x = 0.2 * i;
    const size_t idx = progress.update(path, robotAt(x, 0.05), false);
    EXPECT_LE(idx, tip) << "robot at x " << x << " went onto the return leg";
    EXPECT_NEAR(path.poses[idx].pose.position.x, x, 0.051);
  }
  // Round the tip and back: now on the return leg, and it stays there.
  for (int i = 30; i >= 0; --i) {
    progress.update(path, robotAt(0.2 * i, -0.05), false);
  }
  EXPECT_GE(progress.index(), path.poses.size() - 3);
}

TEST(PathProgress, ClosedLoopStartsAtTheStartAndEndsAtTheEnd)
{
  // A -> B -> C -> A, a 4 x 4 m square: the robot at A is on both the first
  // and the last pose.
  const auto path = polyline({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0, 0}});
  PathProgress progress;
  EXPECT_EQ(progress.update(path, robotAt(0, 0), false), 0u);
  for (int i = 0; i <= 20; ++i) {
    progress.update(path, robotAt(0.2 * i, 0), false);
  }
  for (int i = 0; i <= 20; ++i) {
    progress.update(path, robotAt(4, 0.2 * i), false);
  }
  for (int i = 20; i >= 0; --i) {
    progress.update(path, robotAt(0.2 * i, 4), false);
  }
  for (int i = 20; i >= 0; --i) {
    progress.update(path, robotAt(0, 0.2 * i), false);
  }
  EXPECT_EQ(progress.index(), path.poses.size() - 1);
}

TEST(PathProgress, UpperFloorOverTheLowerOneIsToldApartIn3D)
{
  // 1F east 5 m, a 2 m stair rising 2.5 m, then 2F back west over 1F.
  const auto path = polyline({{0, 0, 0}, {5, 0, 0}, {5, 2, 2.5}, {0, 2.0, 2.5}});
  PathProgress progress;
  for (int i = 0; i <= 20; ++i) {
    const double x = 0.2 * i;
    const size_t idx = progress.update(path, robotAt(x, 0.0, 0.0), true);
    EXPECT_NEAR(path.poses[idx].pose.position.z, 0.0, 1e-9) << "robot at x " << x;
  }
}

TEST(PathProgress, ANewPathStartsOver)
{
  const auto a = polyline({{0, 0}, {10, 0}}, 1);
  const auto b = polyline({{0, 0}, {10, 0}}, 2);
  PathProgress progress;
  for (int i = 0; i <= 16; ++i) {
    progress.update(a, robotAt(0.5 * i, 0), false);
  }
  EXPECT_GT(progress.index(), 70u);
  // A replan from where the robot stood: same geometry, new stamp. The robot is
  // at x = 1 on it, behind where it was on the old path.
  EXPECT_NEAR(b.poses[progress.update(b, robotAt(1.0, 0), false)].pose.position.x, 1.0, 0.051);
}

TEST(FirstLocalMinimum, EmptyRangeIsEnd)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  auto it = syncai_util::firstLocalMinimum(
    poses.begin(), poses.end(), [](const geometry_msgs::msg::PoseStamped &) { return 0.0; }, 0.3);
  EXPECT_EQ(it, poses.end());
}
