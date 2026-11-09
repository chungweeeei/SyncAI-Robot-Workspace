// MapInsertGate: which scans go into the local map. A scan placed by a pose
// that is running away must not, or it builds the copy of the map that the
// next scans match and that holds the pose wrong (robot01, 2026-11-08 16:16:
// 37 m run standing still, ~400 grids of a shifted corridor). What is pinned:
//  - the rules: a scan fails over the speed or under the effective / near
//    share, strictly; the share is over near points, not all, and not judged
//    on too few; a failing scan is never inserted; failures in a row close the
//    gate to every scan, passes in a row reopen it;
//  - robot01's health log of 2026-11-08, second by second through the default
//    thresholds: the run-away is shut out from the second its speed passes
//    1.5 m/s and stays out, the fastest healthy walking of the 11 h before is
//    never kept out, and the 16:05 slide is caught only in part (the gate is
//    not a slide detector, see map_insert_gate.h).
// Each logged second is ten scans: one at the second's top speed and worst
// effective share, nine at its last speed and the share that makes the mean.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "syncai_pointlio/map_builder/map_insert_gate.h"

namespace
{

constexpr int kNear = 500;

// Defaults as in params/pointlio_params.yaml.
Config defaultConfig()
{
  return Config();
}

// One scan with kNear near points and `ratio` of them effective.
MapInsertGate::Verdict judgeRatio(MapInsertGate & gate, double speed, double ratio)
{
  return gate.judge(speed, kNear, static_cast<int>(std::lround(ratio * kNear)));
}

// One `lio health:` second: the last scan's speed, the window's top speed,
// the mean and the worst scan's effective share (%; near was 100 %).
struct LoggedSecond
{
  const char * time;
  double v;
  double v_max;
  int eff_pct;
  int eff_min_pct;
};

struct SecondResult
{
  std::string time;
  int inserted = 0;
  bool closed = false;
  bool opened = false;
};

std::vector<SecondResult> replay(MapInsertGate & gate, const std::vector<LoggedSecond> & log)
{
  std::vector<SecondResult> out;
  for (const LoggedSecond & s : log) {
    const int n = 10;
    const double worst = s.eff_min_pct / 100.0;
    const double rest = std::min(1.0, std::max(0.0, (n * s.eff_pct / 100.0 - worst) / (n - 1)));
    SecondResult r;
    r.time = s.time;
    for (int i = 0; i < n; ++i) {
      const MapInsertGate::Verdict v =
        i == 0 ? judgeRatio(gate, s.v_max, worst) : judgeRatio(gate, std::min(s.v, s.v_max), rest);
      r.inserted += v.insert ? 1 : 0;
      r.closed = r.closed || v.closed_now;
      r.opened = r.opened || v.opened_now;
    }
    out.push_back(r);
  }
  return out;
}

}  // namespace

TEST(MapInsertGate, HealthyScanIsInserted)
{
  MapInsertGate gate(defaultConfig());
  const MapInsertGate::Verdict v = judgeRatio(gate, 0.7, 0.97);
  EXPECT_TRUE(v.insert);
  EXPECT_FALSE(v.speed_failed);
  EXPECT_FALSE(v.ratio_failed);
  EXPECT_NEAR(v.ratio, 0.97, 1e-3);
  EXPECT_TRUE(gate.isOpen());
}

// A failing scan is kept out on its own; fewer failures in a row than
// close_after leave the gate open and the next good scan goes in.
TEST(MapInsertGate, OneFailingScanIsKeptOutWithoutClosing)
{
  MapInsertGate gate(defaultConfig());
  const MapInsertGate::Verdict slow = judgeRatio(gate, 0.5, 0.60);
  EXPECT_FALSE(slow.insert);
  EXPECT_TRUE(slow.ratio_failed);
  EXPECT_FALSE(slow.speed_failed);
  const MapInsertGate::Verdict fast = judgeRatio(gate, 1.8, 0.97);
  EXPECT_FALSE(fast.insert);
  EXPECT_TRUE(fast.speed_failed);
  EXPECT_FALSE(fast.closed_now);
  EXPECT_TRUE(gate.isOpen());
  EXPECT_TRUE(judgeRatio(gate, 0.7, 0.97).insert);
  // The pass broke the streak: two more failures do not close it.
  EXPECT_FALSE(judgeRatio(gate, 1.8, 0.97).closed_now);
  EXPECT_FALSE(judgeRatio(gate, 1.8, 0.97).closed_now);
  EXPECT_TRUE(gate.isOpen());
}

TEST(MapInsertGate, ClosesAfterCloseAfterFailuresAndReopensAfterOpenAfterPasses)
{
  Config c = defaultConfig();
  c.map_gate_close_after = 3;
  c.map_gate_open_after = 5;
  MapInsertGate gate(c);
  EXPECT_FALSE(judgeRatio(gate, 2.0, 0.5).closed_now);
  EXPECT_FALSE(judgeRatio(gate, 2.0, 0.5).closed_now);
  const MapInsertGate::Verdict third = judgeRatio(gate, 2.0, 0.5);
  EXPECT_TRUE(third.closed_now);
  EXPECT_FALSE(third.insert);
  EXPECT_FALSE(gate.isOpen());

  // Closed: good scans stay out until five in a row; a failure starts over.
  for (int i = 0; i < 4; ++i) EXPECT_FALSE(judgeRatio(gate, 0.5, 0.97).insert);
  EXPECT_FALSE(judgeRatio(gate, 0.5, 0.60).insert);
  for (int i = 0; i < 4; ++i) {
    const MapInsertGate::Verdict v = judgeRatio(gate, 0.5, 0.97);
    EXPECT_FALSE(v.insert);
    EXPECT_FALSE(v.opened_now);
  }
  // The fifth in a row reopens the gate and is itself inserted.
  const MapInsertGate::Verdict fifth = judgeRatio(gate, 0.5, 0.97);
  EXPECT_TRUE(fifth.opened_now);
  EXPECT_TRUE(fifth.insert);
  EXPECT_TRUE(gate.isOpen());
  EXPECT_TRUE(judgeRatio(gate, 0.5, 0.97).insert);
}

// Over / under, strictly: a scan exactly at a threshold passes.
TEST(MapInsertGate, ThresholdsAreStrict)
{
  const Config c = defaultConfig();
  MapInsertGate gate(c);
  EXPECT_TRUE(gate.judge(c.map_gate_max_speed, 100, 85).insert);
  EXPECT_FALSE(gate.judge(c.map_gate_max_speed + 1e-6, 100, 85).insert);
  EXPECT_FALSE(gate.judge(0.5, 100, 84).insert);
}

// The share is effective over near points. A scan entering a place the map
// has not seen has few of those; its share of all points is low, but it must
// go in, or the place is never mapped.
TEST(MapInsertGate, ShareIsOverNearPointsSoANewPlaceIsInserted)
{
  MapInsertGate gate(defaultConfig());
  // 500 points, 100 near (the rest new ground), 95 of those on a plane: 19 %
  // of all points, 95 % of the near ones.
  const MapInsertGate::Verdict v = gate.judge(0.7, 100, 95);
  EXPECT_TRUE(v.insert);
  EXPECT_NEAR(v.ratio, 0.95, 1e-9);
}

// Under min_near near points the share is not judged (one standard error is
// as large as the margin it would be judged on); the speed still is.
TEST(MapInsertGate, FewNearPointsAreNotJudgedOnTheShare)
{
  const Config c = defaultConfig();
  MapInsertGate gate(c);
  const MapInsertGate::Verdict few = gate.judge(0.7, c.map_gate_min_near - 1, 0);
  EXPECT_TRUE(few.insert);
  EXPECT_LT(few.ratio, 0.0);
  EXPECT_FALSE(few.ratio_failed);
  const MapInsertGate::Verdict none = gate.judge(0.7, 0, 0);
  EXPECT_TRUE(none.insert);
  EXPECT_LT(none.ratio, 0.0);
  const MapInsertGate::Verdict fast = gate.judge(2.0, 0, 0);
  EXPECT_FALSE(fast.insert);
  EXPECT_TRUE(fast.speed_failed);
  // At min_near it is judged.
  EXPECT_TRUE(gate.judge(0.7, c.map_gate_min_near, 0).ratio_failed);
}

TEST(MapInsertGate, DisabledInsertsEverything)
{
  Config c = defaultConfig();
  c.map_gate_enabled = false;
  MapInsertGate gate(c);
  for (int i = 0; i < 20; ++i) {
    const MapInsertGate::Verdict v = judgeRatio(gate, 3.0, 0.1);
    EXPECT_TRUE(v.insert);
    EXPECT_FALSE(v.closed_now);
  }
  EXPECT_TRUE(gate.isOpen());
}

// 2026-11-08 16:15:58 - 16:16:45: walking south at P3 -> P4, Point-LIO's speed
// climbing past the robot's; from 16:16:09 the robot stood still while the
// filter ran north at ~2 m/s. On the robot every one of these scans was
// inserted, and 16:16:04 onwards built the copy (dg 3 -> 28 grids a second).
// Through the gate everything up to 16:16:02 goes in, 16:16:03 (1.81 m/s)
// closes it, and nothing is inserted after.
TEST(MapInsertGate, TheRunAwayOf2026_11_08IsShutOutAndStaysOut)
{
  const std::vector<LoggedSecond> log = {
    {"16:15:58", 0.39, 0.42, 88, 86}, {"16:15:59", 0.36, 0.54, 90, 89},
    {"16:16:00", 0.62, 0.62, 89, 88}, {"16:16:01", 1.04, 1.12, 89, 88},
    {"16:16:02", 1.25, 1.25, 87, 85}, {"16:16:03", 1.81, 1.81, 87, 86},
    {"16:16:04", 1.81, 1.88, 85, 81}, {"16:16:05", 1.92, 1.92, 82, 81},
    {"16:16:06", 1.71, 1.89, 80, 79}, {"16:16:07", 1.92, 1.92, 74, 71},
    {"16:16:08", 2.13, 2.13, 62, 57}, {"16:16:09", 2.14, 2.16, 52, 47},
    {"16:16:10", 1.81, 2.11, 51, 49}, {"16:16:11", 1.73, 1.79, 53, 50},
    {"16:16:12", 1.69, 1.84, 50, 47}, {"16:16:13", 1.88, 1.88, 50, 48},
    {"16:16:14", 2.01, 2.01, 47, 42}, {"16:16:15", 2.10, 2.11, 47, 41},
    {"16:16:16", 2.17, 2.18, 48, 45}, {"16:16:17", 2.22, 2.23, 42, 39},
    {"16:16:18", 2.17, 2.24, 45, 42}, {"16:16:19", 2.20, 2.20, 47, 44},
    {"16:16:20", 2.10, 2.19, 47, 44}, {"16:16:21", 1.94, 2.10, 46, 43},
    {"16:16:22", 1.79, 1.93, 45, 44}, {"16:16:23", 1.46, 1.73, 45, 42},
    {"16:16:24", 1.04, 1.39, 47, 44}, {"16:16:25", 0.61, 1.00, 51, 47},
    {"16:16:26", 0.17, 0.54, 55, 52}, {"16:16:27", 0.33, 0.33, 58, 55},
    {"16:16:28", 0.40, 0.40, 60, 56}, {"16:16:29", 0.30, 0.36, 62, 60},
    {"16:16:30", 0.22, 0.29, 63, 60}, {"16:16:31", 0.15, 0.21, 64, 62},
    {"16:16:32", 0.06, 0.14, 67, 65}, {"16:16:33", 0.03, 0.05, 70, 67},
    {"16:16:34", 0.03, 0.04, 72, 69}, {"16:16:35", 0.03, 0.03, 73, 71},
    {"16:16:36", 0.03, 0.04, 74, 71}, {"16:16:37", 0.02, 0.02, 74, 72},
    {"16:16:38", 0.01, 0.02, 75, 73}, {"16:16:39", 0.02, 0.03, 76, 73},
    {"16:16:40", 0.03, 0.03, 76, 71}, {"16:16:41", 0.18, 0.18, 76, 75},
    {"16:16:42", 0.15, 0.26, 76, 75}, {"16:16:43", 0.22, 0.22, 75, 73},
    {"16:16:44", 0.29, 0.29, 74, 72}, {"16:16:45", 0.09, 0.27, 74, 72},
  };
  MapInsertGate gate(defaultConfig());
  const std::vector<SecondResult> result = replay(gate, log);
  bool closed = false;
  for (const SecondResult & r : result) {
    EXPECT_FALSE(r.opened) << r.time;
    if (r.time < "16:16:03") {
      EXPECT_EQ(r.inserted, 10) << r.time;
      EXPECT_FALSE(r.closed) << r.time;
    } else {
      EXPECT_EQ(r.inserted, 0) << r.time;
    }
    if (r.time == "16:16:03") EXPECT_TRUE(r.closed);
    closed = closed || r.closed;
  }
  EXPECT_TRUE(closed);
  EXPECT_FALSE(gate.isOpen());
}

// 2026-11-08 16:05:28 - 16:05:52: the slide on the way into P3 that the
// run-away followed (4 m in an in-place turn). The gate closes as the share
// drops (16:05:32) and again at 16:05:48, but between them the share sat at
// 87-91 %, over the threshold, and 16:05:41 - :47 go in. Pinned so that a
// change to that, either way, is a decision and not an accident.
TEST(MapInsertGate, TheSlideOf2026_11_08IsCaughtOnlyInPart)
{
  const std::vector<LoggedSecond> log = {
    {"16:05:28", 0.90, 0.90, 97, 97}, {"16:05:29", 1.09, 1.23, 97, 96},
    {"16:05:30", 0.87, 1.11, 93, 91}, {"16:05:31", 1.07, 1.15, 89, 88},
    {"16:05:32", 0.37, 0.96, 83, 81}, {"16:05:33", 0.25, 0.36, 84, 83},
    {"16:05:34", 0.57, 0.57, 85, 84}, {"16:05:35", 1.20, 1.20, 91, 89},
    {"16:05:36", 1.35, 1.65, 95, 93}, {"16:05:37", 1.04, 1.40, 89, 88},
    {"16:05:38", 0.19, 0.96, 88, 87}, {"16:05:39", 0.24, 0.30, 90, 88},
    {"16:05:40", 0.48, 0.48, 88, 87}, {"16:05:41", 0.40, 0.56, 88, 87},
    {"16:05:42", 0.48, 0.48, 89, 88}, {"16:05:44", 0.13, 0.40, 96, 91},
    {"16:05:45", 0.41, 0.58, 90, 88}, {"16:05:46", 0.32, 0.67, 88, 85},
    {"16:05:47", 0.31, 0.43, 85, 83}, {"16:05:48", 0.35, 0.43, 84, 81},
    {"16:05:49", 1.13, 1.13, 83, 81}, {"16:05:50", 0.33, 1.17, 82, 79},
    {"16:05:51", 0.98, 0.98, 83, 82}, {"16:05:52", 0.71, 1.04, 85, 84},
  };
  MapInsertGate gate(defaultConfig());
  const std::vector<SecondResult> result = replay(gate, log);
  std::vector<std::string> closes, opens;
  int inserted_after_first_close = 0;
  for (const SecondResult & r : result) {
    if (r.closed) closes.push_back(r.time);
    if (r.opened) opens.push_back(r.time);
    if (!closes.empty() && r.time > closes.front()) inserted_after_first_close += r.inserted;
  }
  EXPECT_EQ(closes, (std::vector<std::string>{"16:05:32", "16:05:48"}));
  EXPECT_EQ(opens, (std::vector<std::string>{"16:05:41"}));
  EXPECT_EQ(inserted_after_first_close, 59);
  EXPECT_FALSE(gate.isOpen());
}

// The fastest walking seconds of the 11 h of 2026-11-08 before the slides
// (every one with a top speed over 1.3 m/s, 1.45 the largest): none is kept
// out. The speed threshold sits 0.05 m/s over the largest.
TEST(MapInsertGate, TheFastestHealthyWalkingOf2026_11_08IsNeverKeptOut)
{
  const std::vector<LoggedSecond> log = {
    {"04:45:54", 1.12, 1.32, 99, 98}, {"05:25:38", 0.87, 1.32, 96, 96},
    {"06:15:16", 0.96, 1.35, 98, 97}, {"07:45:38", 1.40, 1.40, 98, 97},
    {"08:25:41", 0.94, 1.31, 98, 98}, {"08:35:36", 0.78, 1.31, 96, 95},
    {"09:55:28", 0.85, 1.31, 98, 97}, {"10:16:51", 1.38, 1.38, 99, 98},
    {"10:16:56", 0.80, 1.33, 98, 98}, {"11:35:18", 1.01, 1.40, 98, 97},
    {"12:35:37", 1.19, 1.32, 97, 95}, {"12:45:40", 1.00, 1.33, 97, 95},
    {"14:55:27", 1.18, 1.32, 97, 95}, {"15:36:38", 0.70, 1.32, 98, 97},
    {"15:55:21", 0.85, 1.45, 95, 93},
  };
  MapInsertGate gate(defaultConfig());
  for (const SecondResult & r : replay(gate, log)) EXPECT_EQ(r.inserted, 10) << r.time;
  EXPECT_TRUE(gate.isOpen());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
