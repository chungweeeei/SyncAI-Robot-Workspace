// StairMode on a synthetic path: 3 m level, a 2 m flight rising 1.36 m (0.68 m
// per metre, dp1f_1012's spiral stair), a 2 m landing, 3 m level, 0.1 m between
// poses. With the default windows the robot enters 0.5 m before the flight
// (idx 25) and leaves 0.4 m past its top (idx 54), where the 0.8 m window
// behind holds less than 0.3 m of rise.
// The controllers' part (braking, the climb itself) is not here; what is
// tested is when the class asks for stairs, when it holds, what it sends and
// what it reports, with the ROS side effects replaced by callbacks.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "syncai_util/stair_mode.hpp"

using syncai_util::StairMode;

namespace
{

constexpr double kStep = 0.1;
constexpr size_t kLevelIn = 30;   // 3 m
constexpr size_t kFlight = 20;    // 2 m, +1.36 m
constexpr size_t kLanding = 20;   // 2 m
constexpr size_t kLevelOut = 30;  // 3 m
constexpr double kRise = 1.36;

nav_msgs::msg::Path stairPath()
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  double z = 0.0;
  const size_t n = kLevelIn + kFlight + kLanding + kLevelOut;
  for (size_t i = 0; i < n; ++i) {
    if (i > kLevelIn && i <= kLevelIn + kFlight) {
      z += kRise / kFlight;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = i * kStep;
    ps.pose.position.z = z;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  return p;
}

nav_msgs::msg::Path levelPath()
{
  auto p = stairPath();
  for (auto & ps : p.poses) {
    ps.pose.position.z = 0.0;
  }
  return p;
}

rclcpp::Time at(double sec)
{
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

struct Harness
{
  StairMode sm;
  std::vector<int> sent;
  std::vector<bool> published;
  bool ready{true};

  explicit Harness(StairMode::Params params = StairMode::Params())
  {
    sm.configure(
      params, [this](int p) { sent.push_back(p); }, [this]() { return ready; },
      [this](bool b) { published.push_back(b); });
  }
  // The controller's loop: remaining poses = everything from `idx` on.
  StairMode::Decision tick(const nav_msgs::msg::Path & path, size_t idx, double sec)
  {
    return sm.update(at(sec), path.poses.size() - idx);
  }
};

}  // namespace

TEST(StairRise, LevelIsZeroAndFlightRises)
{
  const auto path = stairPath();
  EXPECT_DOUBLE_EQ(StairMode::stairRise(path, 5, 0.0, 1.0), 0.0);
  // 1 m before the flight: the window reaches the first step only.
  EXPECT_LT(StairMode::stairRise(path, kLevelIn - 10, 0.0, 1.0), 0.05);
  // On the flight, 1 m ahead is 10 steps of 0.068.
  EXPECT_NEAR(StairMode::stairRise(path, kLevelIn + 5, 0.0, 1.0), 10 * kRise / kFlight, 1e-9);
  // Past the end of the path.
  EXPECT_DOUBLE_EQ(StairMode::stairRise(path, path.poses.size() + 3, 0.0, 1.0), 0.0);
}

TEST(StairMode, DisabledNeverHolds)
{
  StairMode::Params p;
  p.enabled = false;
  Harness h(p);
  const auto path = stairPath();
  h.sm.setPlan(path);
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto d = h.tick(path, i, i * 0.05);
    EXPECT_FALSE(d.hold);
    EXPECT_FALSE(d.on_stairs);
    EXPECT_TRUE(d.refuse.empty());
  }
  EXPECT_TRUE(h.sent.empty());
}

TEST(StairMode, NoPolicySwitchChangesModeWithoutStopping)
{
  StairMode::Params p;
  p.stair_policy = -1;
  Harness h(p);
  const auto path = stairPath();
  h.sm.setPlan(path);
  // Level approach: not yet.
  auto d = h.tick(path, 0, 0.0);
  EXPECT_FALSE(d.on_stairs);
  EXPECT_FALSE(d.hold);
  // Within 1 m of the flight the window rises 0.3: enter, no hold.
  bool entered = false;
  size_t enter_idx = 0;
  for (size_t i = 0; i < kLevelIn + kFlight; ++i) {
    d = h.tick(path, i, 0.0);
    EXPECT_FALSE(d.hold);
    if (d.on_stairs && !entered) {
      entered = true;
      enter_idx = i;
    }
  }
  ASSERT_TRUE(entered);
  // Entered before the flight (the enter window looks 1 m ahead) and not
  // absurdly early.
  EXPECT_LT(enter_idx, kLevelIn);
  EXPECT_GT(enter_idx, kLevelIn - 15);
  // Just past the top the 0.8 m window behind still holds 0.4 m of rise: stay.
  d = h.tick(path, kLevelIn + kFlight + 2, 0.0);
  EXPECT_TRUE(d.on_stairs);
  // Well onto the landing the window is level: leave.
  d = h.tick(path, kLevelIn + kFlight + kLanding + 12, 0.0);
  EXPECT_FALSE(d.on_stairs);
  EXPECT_FALSE(d.hold);
  EXPECT_TRUE(h.sent.empty());
  ASSERT_EQ(h.published.size(), 3u);  // configure(), enter, leave
  EXPECT_FALSE(h.published[0]);
  EXPECT_TRUE(h.published[1]);
  EXPECT_FALSE(h.published[2]);
}

TEST(StairMode, PolicySwitchHoldsSendsSettlesAndRestores)
{
  Harness h;
  h.sm.setReportedPolicy(0);  // PPO on the flat
  const auto path = stairPath();
  h.sm.setPlan(path);
  const size_t idx = kLevelIn - 5;  // 0.5 m before the flight
  auto d = h.tick(path, idx, 0.0);
  EXPECT_TRUE(d.hold);
  EXPECT_FALSE(d.on_stairs);
  EXPECT_TRUE(h.sent.empty());
  // Still braking: nothing sent until the controller reports standing.
  d = h.tick(path, idx, 0.05);
  EXPECT_TRUE(d.hold);
  h.sm.standing(at(0.1));
  ASSERT_EQ(h.sent.size(), 1u);
  EXPECT_EQ(h.sent[0], 1);
  // Settling: hold continues, no second send.
  d = h.tick(path, idx, 1.0);
  EXPECT_TRUE(d.hold);
  h.sm.standing(at(1.0));
  EXPECT_EQ(h.sent.size(), 1u);
  EXPECT_FALSE(h.sm.onStairs());
  // 3 s after the send the switch is done.
  h.sm.standing(at(3.2));
  EXPECT_TRUE(h.sm.onStairs());
  d = h.tick(path, idx, 3.25);
  EXPECT_FALSE(d.hold);
  EXPECT_TRUE(d.on_stairs);

  // Up the flight and just past its top: on stairs, no hold.
  for (size_t i = idx; i < kLevelIn + kFlight + 3; ++i) {
    d = h.tick(path, i, 4.0);
    EXPECT_FALSE(d.hold);
    EXPECT_TRUE(d.on_stairs);
  }
  // Onto the landing: leave, which is a second stop and the restore.
  const size_t out = kLevelIn + kFlight + kLanding + 12;
  d = h.tick(path, out, 5.0);
  EXPECT_TRUE(d.hold);
  EXPECT_TRUE(d.on_stairs);  // still set up for stairs until it settles
  h.sm.standing(at(5.1));
  ASSERT_EQ(h.sent.size(), 2u);
  EXPECT_EQ(h.sent[1], 0);  // back to what was reported
  h.sm.standing(at(8.2));
  EXPECT_FALSE(h.sm.onStairs());
  d = h.tick(path, out, 8.25);
  EXPECT_FALSE(d.hold);
  EXPECT_FALSE(d.on_stairs);
}

TEST(StairMode, FlatPolicyNamedIsRestoredAndUnknownIsKept)
{
  {
    StairMode::Params p;
    p.flat_policy = 2;
    Harness h(p);
    h.sm.setReportedPolicy(0);
    const auto path = stairPath();
    h.sm.setPlan(path);
    h.tick(path, kLevelIn - 5, 0.0);
    h.sm.standing(at(0.0));
    h.sm.standing(at(3.5));
    ASSERT_TRUE(h.sm.onStairs());
    const size_t out = kLevelIn + kFlight + kLanding + 12;
    h.tick(path, out, 5.0);
    h.sm.standing(at(5.0));
    ASSERT_EQ(h.sent.size(), 2u);
    EXPECT_EQ(h.sent[1], 2);
  }
  {
    // Nothing reported, flat_policy -1: leaving sends nothing and does not stop.
    Harness h;
    const auto path = stairPath();
    h.sm.setPlan(path);
    h.tick(path, kLevelIn - 5, 0.0);
    h.sm.standing(at(0.0));
    h.sm.standing(at(3.5));
    ASSERT_TRUE(h.sm.onStairs());
    const size_t out = kLevelIn + kFlight + kLanding + 12;
    const auto d = h.tick(path, out, 5.0);
    EXPECT_FALSE(d.hold);
    EXPECT_FALSE(d.on_stairs);
    EXPECT_EQ(h.sent.size(), 1u);
  }
}

TEST(StairMode, AlreadyOnStairPolicyNeedsNoStop)
{
  Harness h;
  h.sm.setReportedPolicy(1);
  const auto path = stairPath();
  h.sm.setPlan(path);
  const auto d = h.tick(path, kLevelIn - 5, 0.0);
  EXPECT_FALSE(d.hold);
  EXPECT_TRUE(d.on_stairs);
  EXPECT_TRUE(h.sent.empty());
}

TEST(StairMode, ServiceMissingRefusesEnteringOnly)
{
  Harness h;
  h.ready = false;
  h.sm.setReportedPolicy(0);
  const auto path = stairPath();
  h.sm.setPlan(path);
  auto d = h.tick(path, kLevelIn - 5, 0.0);
  EXPECT_FALSE(d.refuse.empty());
  EXPECT_FALSE(d.hold);
  EXPECT_FALSE(h.sm.onStairs());
  // Service back: the next cycle enters.
  h.ready = true;
  d = h.tick(path, kLevelIn - 5, 0.1);
  EXPECT_TRUE(d.refuse.empty());
  EXPECT_TRUE(d.hold);
  h.sm.standing(at(0.2));
  h.sm.standing(at(3.5));
  ASSERT_TRUE(h.sm.onStairs());
  // Service gone again on the way out: no refusal, the policy is just not sent.
  h.ready = false;
  const size_t out = kLevelIn + kFlight + kLanding + 12;
  d = h.tick(path, out, 5.0);
  EXPECT_TRUE(d.refuse.empty());
  EXPECT_TRUE(d.hold);
  h.sm.standing(at(5.0));
  EXPECT_EQ(h.sent.size(), 1u);
  h.sm.standing(at(8.5));
  EXPECT_FALSE(h.sm.onStairs());
}

TEST(StairMode, ReplanBeforeSendCancelsAndResetAfterSendCompletes)
{
  {
    Harness h;
    h.sm.setReportedPolicy(0);
    const auto path = stairPath();
    h.sm.setPlan(path);
    auto d = h.tick(path, kLevelIn - 5, 0.0);
    EXPECT_TRUE(d.hold);
    // A replan round the stairs before the policy went out: dropped.
    const auto level = levelPath();
    h.sm.setPlan(level);
    d = h.tick(level, kLevelIn - 5, 0.1);
    EXPECT_FALSE(d.hold);
    EXPECT_FALSE(d.on_stairs);
    EXPECT_TRUE(h.sent.empty());
  }
  {
    Harness h;
    h.sm.setReportedPolicy(0);
    const auto path = stairPath();
    h.sm.setPlan(path);
    h.tick(path, kLevelIn - 5, 0.0);
    h.sm.standing(at(0.0));
    ASSERT_EQ(h.sent.size(), 1u);
    // The goal ends mid-settle: the switch counts as done.
    h.sm.reset();
    EXPECT_TRUE(h.sm.onStairs());
    // on_stairs survives into the next goal; a level plan takes it back out.
    const auto level = levelPath();
    h.sm.setPlan(level);
    const auto d = h.tick(level, 0, 1.0);
    EXPECT_TRUE(d.hold);  // leaving: stop and restore
    h.sm.standing(at(1.0));
    ASSERT_EQ(h.sent.size(), 2u);
    EXPECT_EQ(h.sent[1], 0);
  }
  {
    // Reset before the send: never started.
    Harness h;
    h.sm.setReportedPolicy(0);
    const auto path = stairPath();
    h.sm.setPlan(path);
    h.tick(path, kLevelIn - 5, 0.0);
    h.sm.reset();
    EXPECT_FALSE(h.sm.onStairs());
    EXPECT_TRUE(h.sent.empty());
  }
}

TEST(StairMode, LevelPathNeverAsks)
{
  Harness h;
  h.sm.setReportedPolicy(0);
  const auto level = levelPath();
  h.sm.setPlan(level);
  for (size_t i = 0; i < level.poses.size(); ++i) {
    const auto d = h.tick(level, i, 0.0);
    EXPECT_FALSE(d.hold);
    EXPECT_FALSE(d.on_stairs);
  }
  EXPECT_TRUE(h.sent.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
