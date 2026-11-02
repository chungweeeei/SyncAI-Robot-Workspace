// findCusps / splitAtCusps on synthetic routes, 0.1 m between poses: the
// shapes a patrol route takes (out and back, out and back one cell aside,
// robot01's P1 -> P2 -> P3 -> P2 -> P4 of 2026-11-02, a loop) and the ones
// that must not be cut (a straight line, a 90 deg corner, a zig-zag of
// smoothing noise, a switchback stair whose landing turns 180 deg).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "syncai_util/path_cusps.hpp"

using syncai_util::CuspParams;
using syncai_util::findCusps;
using syncai_util::splitAtCusps;

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

// Straight segments through `corners`, sampled every kStep, corners included
// once (test_path_progress.cpp's).
nav_msgs::msg::Path polyline(const std::vector<std::vector<double>> & corners)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.header.stamp.sec = 1;
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

double yawOf(const geometry_msgs::msg::PoseStamped & ps)
{
  const auto & q = ps.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double distTo(const geometry_msgs::msg::PoseStamped & ps, double x, double y)
{
  return std::hypot(ps.pose.position.x - x, ps.pose.position.y - y);
}

}  // namespace

TEST(PathCusps, StraightLineHasNone)
{
  const auto path = polyline({{0, 0}, {10, 0}});
  EXPECT_TRUE(findCusps(path, CuspParams{}).empty());
  const auto legs = splitAtCusps(path, {}, CuspParams{}.window);
  ASSERT_EQ(legs.size(), 1u);
  EXPECT_EQ(legs[0].poses.size(), path.poses.size());
}

TEST(PathCusps, CornerOf90DegreesIsDrivenThrough)
{
  EXPECT_TRUE(findCusps(polyline({{0, 0}, {5, 0}, {5, 5}}), CuspParams{}).empty());
  // A loop back onto its start: four corners, no reversal.
  EXPECT_TRUE(findCusps(polyline({{0, 0}, {5, 0}, {5, 5}, {0, 5}, {0, 0}}), CuspParams{}).empty());
}

TEST(PathCusps, ZigZagNoiseIsNotACusp)
{
  auto path = polyline({{0, 0}, {10, 0}});
  for (size_t i = 0; i < path.poses.size(); ++i) {
    path.poses[i].pose.position.y = (i % 2 == 0) ? 0.05 : -0.05;
  }
  EXPECT_TRUE(findCusps(path, CuspParams{}).empty());
}

TEST(PathCusps, OutAndBackOnTopOfItselfIsCutAtTheTip)
{
  const auto path = polyline({{0, 0}, {0, 5}, {0, 0}});
  const auto cusps = findCusps(path, CuspParams{});
  ASSERT_EQ(cusps.size(), 1u);
  EXPECT_LT(distTo(path.poses[cusps[0]], 0, 5), 1e-6);

  const auto legs = splitAtCusps(path, cusps, CuspParams{}.window);
  ASSERT_EQ(legs.size(), 2u);
  // The legs share the tip, and together hold every pose once more.
  EXPECT_EQ(legs[0].poses.size() + legs[1].poses.size(), path.poses.size() + 1);
  EXPECT_LT(distTo(legs[0].poses.back(), 0, 5), 1e-6);
  EXPECT_LT(distTo(legs[1].poses.front(), 0, 5), 1e-6);
  EXPECT_LT(distTo(legs[1].poses.back(), 0, 0), 1e-6);
  // The first leg ends facing back down the corridor, where the final
  // rotation turns the robot before the second leg is handed over.
  EXPECT_NEAR(yawOf(legs[0].poses.back()), -M_PI_2, 1e-6);
  EXPECT_EQ(legs[0].header.frame_id, "map");
  EXPECT_EQ(legs[1].header.stamp.sec, 1);
}

TEST(PathCusps, OutAndBackOneCellAsideIsOneCusp)
{
  // The A* return leg a cell beside the outgoing one: two 90 deg corners,
  // which a consecutive-step dot product (upstream MPPI's test) reads as zero.
  const auto path = polyline({{0, 0}, {0, 5}, {0.1, 5}, {0.1, 0}});
  const auto cusps = findCusps(path, CuspParams{});
  ASSERT_EQ(cusps.size(), 1u);
  EXPECT_LT(distTo(path.poses[cusps[0]], 0.05, 5), 0.06);

  const auto legs = splitAtCusps(path, cusps, CuspParams{}.window);
  ASSERT_EQ(legs.size(), 2u);
  // Facing the next leg 0.4 m on, mostly south -- not the planner's heading
  // at the tip, which points at the next cell, east.
  EXPECT_LT(yawOf(legs[0].poses.back()), -M_PI_2 + 0.4);
  EXPECT_GT(yawOf(legs[0].poses.back()), -M_PI_2 - 0.4);
}

TEST(PathCusps, Robot01PatrolOfNovember2)
{
  // P1 -> P2 -> P3 -> P2 -> P4 on map dp: out to P3 and back past P1 to P4.
  const std::vector<double> p1{-10.565, -3.446};
  const std::vector<double> p2{-10.778, 2.293};
  const std::vector<double> p3{-10.980, 7.008};
  const std::vector<double> p4{-10.336, -4.278};
  const auto path = polyline({{-9.15, -4.49}, p1, p2, p3, p2, p4});
  const auto cusps = findCusps(path, CuspParams{});
  ASSERT_EQ(cusps.size(), 1u);
  EXPECT_LT(distTo(path.poses[cusps[0]], p3[0], p3[1]), 0.06);
  const auto legs = splitAtCusps(path, cusps, CuspParams{}.window);
  ASSERT_EQ(legs.size(), 2u);
  EXPECT_NEAR(yawOf(legs[0].poses.back()), std::atan2(p2[1] - p3[1], p2[0] - p3[0]), 0.01);
}

TEST(PathCusps, BackAndForthTwiceIsTwoCusps)
{
  const auto path = polyline({{0, 0}, {4, 0}, {0, 0}, {4, 0}});
  const auto cusps = findCusps(path, CuspParams{});
  ASSERT_EQ(cusps.size(), 2u);
  EXPECT_LT(distTo(path.poses[cusps[0]], 4, 0), 1e-6);
  EXPECT_LT(distTo(path.poses[cusps[1]], 0, 0), 1e-6);
  const auto legs = splitAtCusps(path, cusps, CuspParams{}.window);
  ASSERT_EQ(legs.size(), 3u);
  EXPECT_NEAR(yawOf(legs[0].poses.back()), M_PI, 1e-6);
  EXPECT_NEAR(yawOf(legs[1].poses.back()), 0.0, 1e-6);
  EXPECT_LT(distTo(legs[2].poses.back(), 4, 0), 1e-6);
}

TEST(PathCusps, SwitchbackStairLandingIsLeftToStairMode)
{
  // Two flights rising 1.5 m each, joined by a landing 0.3 m wide -- narrower
  // than the window, so the landing reads as a 180 deg turn.
  const auto path =
    polyline({{-2, 0, 0}, {0, 0, 0}, {3, 0, 1.5}, {3, 0.3, 1.5}, {0, 0.3, 3.0}, {-2, 0.3, 3.0}});
  EXPECT_TRUE(findCusps(path, CuspParams{}).empty());
  // It is the height that spares it: flattened, the landing is a cusp.
  CuspParams flat;
  flat.max_z_span = 100.0;
  EXPECT_EQ(findCusps(path, flat).size(), 1u);
}

TEST(PathCusps, TooNearAnEndOrDisabledIsNone)
{
  // Back 0.3 m after the tip: no direction `window` after it, and the path
  // ends there anyway.
  EXPECT_TRUE(findCusps(polyline({{0, 0}, {0, 5}, {0, 4.7}}), CuspParams{}).empty());
  CuspParams off;
  off.min_angle = 0.0;
  EXPECT_TRUE(findCusps(polyline({{0, 0}, {0, 5}, {0, 0}}), off).empty());
  EXPECT_TRUE(findCusps(nav_msgs::msg::Path{}, CuspParams{}).empty());
}
