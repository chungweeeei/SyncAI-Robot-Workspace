// TrackTargetFilter, ROS-free: the targets track_pose holds in odom, the one
// it drives to, and when they are no target at all.

#include <gtest/gtest.h>

#include <cmath>

#include "syncai_controller/track_target_filter.hpp"

using syncai_controller::Pose2;
using syncai_controller::TrackTargetFilter;
using syncai_controller::TrackTargetFilterParams;

namespace
{

TrackTargetFilterParams window(double seconds)
{
  TrackTargetFilterParams p;
  p.window = seconds;
  return p;
}

}  // namespace

TEST(TrackTargetFilter, WithoutAWindowTheNewestPoseIsTheTarget)
{
  TrackTargetFilter filter;
  filter.configure(window(0.0));
  EXPECT_FALSE(filter.hasTarget());
  filter.add(1.0, Pose2{1.0, 0.0, 0.0});
  ASSERT_TRUE(filter.hasTarget());
  filter.add(1.1, Pose2{5.0, 2.0, 1.0});
  EXPECT_EQ(filter.size(), 1u);
  EXPECT_DOUBLE_EQ(filter.target().x, 5.0);
  // A tracked thing that moves is never "unstable".
  EXPECT_TRUE(filter.stable());
  // A pose that arrives late does not replace a newer one.
  filter.add(1.05, Pose2{9.0, 9.0, 0.0});
  EXPECT_DOUBLE_EQ(filter.target().x, 5.0);
  EXPECT_DOUBLE_EQ(filter.newestStamp(), 1.1);
}

TEST(TrackTargetFilter, ATargetNeedsMinPoses)
{
  TrackTargetFilter filter;
  filter.configure(window(1.0));
  filter.add(0.0, Pose2{1.0, 0.0, 0.0});
  filter.add(0.1, Pose2{1.0, 0.0, 0.0});
  EXPECT_FALSE(filter.hasTarget());
  filter.add(0.2, Pose2{1.0, 0.0, 0.0});
  EXPECT_TRUE(filter.hasTarget());
}

TEST(TrackTargetFilter, TheMedianIgnoresAStraySighting)
{
  // Ten sightings of a charger, one of them the tag's mirrored solution.
  TrackTargetFilter filter;
  filter.configure(window(1.0));
  for (int i = 0; i < 10; ++i) {
    const double jitter = 0.002 * (i % 3 - 1);
    filter.add(0.1 * i, Pose2{2.0 + jitter, 1.0 - jitter, 0.5 + 0.01 * (i % 2)});
  }
  filter.add(0.95, Pose2{2.3, 0.6, -0.4});
  const Pose2 target = filter.target();
  EXPECT_NEAR(target.x, 2.0, 0.003);
  EXPECT_NEAR(target.y, 1.0, 0.003);
  EXPECT_NEAR(target.yaw, 0.505, 0.006);
  // One in eleven is under the spread's quantile: still one target.
  EXPECT_TRUE(filter.stable());
}

TEST(TrackTargetFilter, TargetsThatScatterAreUnstable)
{
  // The detector alternating between two solutions 0.3 m apart.
  TrackTargetFilter flip;
  flip.configure(window(1.0));
  for (int i = 0; i < 10; ++i) {
    flip.add(0.1 * i, i % 2 ? Pose2{2.0, 1.0, 0.5} : Pose2{2.3, 1.0, 0.5});
  }
  EXPECT_FALSE(flip.stable());

  // The odometry running away: every sighting of a charger that stands
  // still lands 0.2 m further on.
  TrackTargetFilter runaway;
  runaway.configure(window(1.0));
  for (int i = 0; i < 10; ++i) {
    runaway.add(0.1 * i, Pose2{2.0 + 0.2 * i, 1.0, 0.5});
  }
  EXPECT_FALSE(runaway.stable());
  double xy = 0.0;
  double yaw = 0.0;
  runaway.spread(xy, yaw);
  EXPECT_GT(xy, 0.5);

  // A slow drift, 5 mm a sighting, is what every new pose corrects.
  TrackTargetFilter drift;
  drift.configure(window(1.0));
  for (int i = 0; i < 10; ++i) {
    drift.add(0.1 * i, Pose2{2.0 + 0.005 * i, 1.0, 0.5});
  }
  EXPECT_TRUE(drift.stable());

  // The yaw alone scattering is as unstable.
  TrackTargetFilter turning;
  turning.configure(window(1.0));
  for (int i = 0; i < 10; ++i) {
    turning.add(0.1 * i, Pose2{2.0, 1.0, i % 2 ? 0.5 : -0.5});
  }
  EXPECT_FALSE(turning.stable());
}

TEST(TrackTargetFilter, TheWindowIsBackFromTheNewestStamp)
{
  TrackTargetFilter filter;
  filter.configure(window(1.0));
  for (int i = 0; i < 30; ++i) {
    filter.add(0.1 * i, Pose2{i < 20 ? 1.0 : 2.0, 0.0, 0.0});
  }
  // 2.9 back to 1.9: the old target has aged out (ten or eleven left, as
  // the 1.9 one rounds).
  EXPECT_GE(filter.size(), 10u);
  EXPECT_LE(filter.size(), 11u);
  EXPECT_NEAR(filter.target().x, 2.0, 1e-9);
  // No clock: when sightings stop, the last window stays as it is.
  EXPECT_DOUBLE_EQ(filter.newestStamp(), 2.9);
}

TEST(TrackTargetFilter, TheYawMedianCrossesPi)
{
  TrackTargetFilter filter;
  filter.configure(window(1.0));
  filter.add(0.0, Pose2{0.0, 0.0, M_PI - 0.02});
  filter.add(0.1, Pose2{0.0, 0.0, -M_PI + 0.02});
  filter.add(0.2, Pose2{0.0, 0.0, M_PI - 0.01});
  const double yaw = filter.target().yaw;
  EXPECT_NEAR(std::fabs(yaw), M_PI, 0.011);
  EXPECT_TRUE(filter.stable());
}

TEST(TrackTargetFilter, AFrozenTargetIgnoresNewSightings)
{
  TrackTargetFilter filter;
  filter.configure(window(1.0));
  for (int i = 0; i < 5; ++i) {
    filter.add(0.1 * i, Pose2{2.0, 1.0, 0.5});
  }
  filter.freeze();
  ASSERT_TRUE(filter.frozen());
  for (int i = 5; i < 30; ++i) {
    filter.add(0.1 * i, Pose2{2.1, 1.0, 0.5});
  }
  EXPECT_DOUBLE_EQ(filter.target().x, 2.0);
  // Let go, it is on the current window at once.
  filter.unfreeze();
  EXPECT_NEAR(filter.target().x, 2.1, 1e-9);

  // Nothing to freeze, nothing frozen.
  TrackTargetFilter empty;
  empty.configure(window(1.0));
  empty.freeze();
  EXPECT_FALSE(empty.frozen());
  EXPECT_FALSE(empty.hasTarget());
}
