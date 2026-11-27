// ScanMerger against the four shapes the Lynx M20's merged cloud takes
// (robot01's bag of 2026-11-24): both lidars' sweeps in one message, the rear
// one a sweep behind, absent, or two sweeps at once. What a change to it must
// not move:
//  - every scan it hands on is one window of the time grid, holding every
//    point of that window from whichever message brought it, with each
//    point's curvature its own offset into the window in ms;
//  - scans come out in time order and never overlap, because the filter only
//    steps forward;
//  - nothing is handed on before it is `holdback` old -- or, with the lidars
//    told apart by ring, before every live one has passed its end, never
//    later than `holdback` -- and a point that arrives after its window went
//    is dropped and counted, never moved into a later scan;
//  - a stream that starts again (a bag replayed) or stops for a while does not
//    wedge it, and a lidar that stops is reported once, as is its return.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "syncai_pointlio/scan_merger.hpp"

using syncai_pointlio::ScanMerger;

namespace
{

const float kFront = 1.0f;
const float kRear = 2.0f;
const float kStray = 3.0f;  // a point whose ring is in no lidar's range

// One lidar's sweep: `n` points evenly over [start, start + 0.1), each half a
// step in so that none sits on a window's edge, marked with `tag` in x so a
// test can tell the sources apart, and given a ring in that lidar's range of
// the M20's (front 0-95, rear 96-191) for the tests that tell them apart.
struct Message
{
  ScanMerger::Cloud::Ptr cloud{new ScanMerger::Cloud};
  std::vector<double> times;
  std::vector<long> rings;

  void sweep(double start, float tag, int n = 10)
  {
    for (int i = 0; i < n; ++i) {
      ScanMerger::Point p;
      p.x = tag;
      p.y = static_cast<float>(i);
      cloud->push_back(p);
      times.push_back(start + 0.1 * (i + 0.5) / n);
      rings.push_back(tag == kFront ? 10 + i : tag == kRear ? 100 + i : 250);
    }
  }

  ScanMerger::Timed timed() const { return {cloud, &times}; }
  ScanMerger::Timed timedWithRings() const { return {cloud, &times, &rings}; }
};

int countTag(const ScanMerger::Cloud & cloud, float tag)
{
  int n = 0;
  for (const auto & p : cloud) n += p.x == tag;
  return n;
}

ScanMerger::Config m20Config()
{
  ScanMerger::Config config;
  config.holdback = 0.12;
  config.scan_period = 0.1;
  return config;
}

// Feeds messages and collects every scan handed on, and the sources' events
struct Harness
{
  explicit Harness(const ScanMerger::Config & config) : merger(config) {}

  void feed(const Message & m)
  {
    merger.add(
      merger.numSources() > 0 ? m.timedWithRings() : m.timed(), ScanMerger::Timed(), scans);
    for (const auto & e : merger.takeEvents()) events.push_back(e);
  }

  ScanMerger merger;
  std::vector<ScanMerger::Scan> scans;
  std::vector<ScanMerger::SourceEvent> events;
};

// The M20's, with the two lidars told apart by ring
ScanMerger::Config m20SourcesConfig()
{
  ScanMerger::Config config = m20Config();
  config.source_bounds = {0, 96, 192};
  config.source_timeout = 0.5;
  return config;
}

void expectOrderedWindows(const std::vector<ScanMerger::Scan> & scans, double period)
{
  for (std::size_t i = 0; i < scans.size(); ++i) {
    for (const auto & p : *scans[i].cloud) {
      EXPECT_GE(p.curvature, 0.0f);
      EXPECT_LT(p.curvature, period * 1000.0 + 1e-3);
    }
    if (i > 0) {
      EXPECT_GE(scans[i].start_time, scans[i - 1].start_time + period - 1e-9);
    }
  }
}

}  // namespace

// Both lidars in the same window, message after message: one scan a message,
// both sources whole in it, a holdback behind.
TEST(ScanMerger, AlignedSweepsBecomeOneScanEach)
{
  Harness run(m20Config());
  for (int k = 0; k < 6; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    run.feed(m);
  }
  // Newest point 100.595; windows ending at or before 100.475 have gone: 4
  ASSERT_EQ(run.scans.size(), 4u);
  for (std::size_t i = 0; i < run.scans.size(); ++i) {
    EXPECT_NEAR(run.scans[i].start_time, 100.0 + 0.1 * i, 1e-9);
    EXPECT_EQ(countTag(*run.scans[i].cloud, kFront), 10);
    EXPECT_EQ(countTag(*run.scans[i].cloud, kRear), 10);
  }
  expectOrderedWindows(run.scans, 0.1);
  const auto stats = run.merger.takeStats();
  EXPECT_EQ(stats.late_points, 0u);
  EXPECT_EQ(stats.points, 120u);
}

// The rear sweep comes one message late (the bag's first minute): it still
// lands in the scan of its own time, beside the front sweep that waited.
TEST(ScanMerger, RearOneSweepBehindJoinsItsOwnWindow)
{
  Harness run(m20Config());
  for (int k = 0; k < 7; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k > 0) m.sweep(100.0 + 0.1 * (k - 1), kRear);
    run.feed(m);
  }
  ASSERT_GE(run.scans.size(), 4u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// A message with no rear sweep, then one with two (16.5 % of the bag each):
// two scans come of the second, each with both sources.
TEST(ScanMerger, MissingThenDoubleRearSweep)
{
  Harness run(m20Config());
  auto both = [&](int k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    run.feed(m);
  };
  both(0);
  both(1);
  {
    Message m;  // no rear
    m.sweep(100.2, kFront);
    run.feed(m);
  }
  {
    Message m;  // the rear's two sweeps
    m.sweep(100.3, kFront);
    m.sweep(100.2, kRear);
    m.sweep(100.3, kRear);
    run.feed(m);
  }
  both(4);
  both(5);
  ASSERT_EQ(run.scans.size(), 4u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// Each point keeps its own time: curvature is the offset into the window in
// ms, whichever message carried the point.
TEST(ScanMerger, CurvatureIsOffsetIntoTheWindow)
{
  Harness run(m20Config());
  for (int k = 0; k < 4; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    run.feed(m);
  }
  ASSERT_FALSE(run.scans.empty());
  const auto & cloud = *run.scans.front().cloud;
  ASSERT_EQ(cloud.size(), 10u);
  for (const auto & p : cloud) EXPECT_NEAR(p.curvature, p.y * 10.0f + 5.0f, 1e-3);
}

// With no holdback a sweep two messages late finds its window gone: its points
// are dropped and counted, with how far behind they were, and are in no later
// scan. (One message late still makes it: a window is handed on by the message
// that brings the next sweep, after that message's own points are taken in.)
TEST(ScanMerger, LatePointsAreDroppedAndCounted)
{
  ScanMerger::Config config = m20Config();
  config.holdback = 0.0;
  Harness run(config);
  for (int k = 0; k < 6; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k > 1) m.sweep(100.0 + 0.1 * (k - 2), kRear);
    run.feed(m);
  }
  int rear = 0;
  for (const auto & scan : run.scans) rear += countTag(*scan.cloud, kRear);
  EXPECT_EQ(rear, 0);
  const auto stats = run.merger.takeStats();
  EXPECT_EQ(stats.late_points, 40u);
  // Its first point, 5 ms into a window that ended 95 ms later
  EXPECT_NEAR(stats.max_late, 0.095, 1e-6);
  expectOrderedWindows(run.scans, 0.1);
}

// The same stream with the M20's holdback: the windows wait, and nothing is
// late.
TEST(ScanMerger, HoldbackKeepsTheLateSweep)
{
  Harness run(m20Config());
  for (int k = 0; k < 8; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k > 1) m.sweep(100.0 + 0.1 * (k - 2), kRear);
    run.feed(m);
  }
  ASSERT_GE(run.scans.size(), 4u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// The dense cloud is cut into the same windows as the filter's.
TEST(ScanMerger, DenseCloudFollowsTheSameWindows)
{
  ScanMerger merger(m20Config());
  std::vector<ScanMerger::Scan> scans;
  for (int k = 0; k < 5; ++k) {
    Message sparse, dense;
    sparse.sweep(100.0 + 0.1 * k, kFront, 10);
    dense.sweep(100.0 + 0.1 * k, kFront, 40);
    merger.add(sparse.timed(), dense.timed(), scans);
  }
  ASSERT_FALSE(scans.empty());
  for (const auto & scan : scans) {
    EXPECT_EQ(scan.cloud->size(), 10u);
    ASSERT_NE(scan.dense, scan.cloud);
    EXPECT_EQ(scan.dense->size(), 40u);
  }
}

// A bag played again: time jumps back by more than restart_gap. The merger
// starts over on the new stream instead of dropping it all as late.
TEST(ScanMerger, RestartedStreamStartsOver)
{
  Harness run(m20Config());
  for (int k = 0; k < 5; ++k) {
    Message m;
    m.sweep(500.0 + 0.1 * k, kFront);
    run.feed(m);
  }
  run.scans.clear();
  run.merger.takeStats();
  for (int k = 0; k < 5; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    run.feed(m);
  }
  ASSERT_FALSE(run.scans.empty());
  EXPECT_NEAR(run.scans.front().start_time, 100.0, 1e-9);
  const auto stats = run.merger.takeStats();
  EXPECT_EQ(stats.restarts, 1u);
  EXPECT_EQ(stats.late_points, 0u);
}

// A long silence (a paused bag, a lidar that dropped out): the grid goes to
// where the points are, and the scans after it are whole.
TEST(ScanMerger, GapInTheStreamIsSteppedOver)
{
  Harness run(m20Config());
  for (int k = 0; k < 4; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    run.feed(m);
  }
  for (int k = 0; k < 4; ++k) {
    Message m;
    m.sweep(100.9 + 0.1 * k, kFront);  // 0.5 s of nothing before it
    run.feed(m);
  }
  for (const auto & scan : run.scans) EXPECT_EQ(scan.cloud->size(), 10u);
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// ---- The lidars told apart by ring (2026-11-25) ----------------------------

// Both lidars in the same window: a window goes as soon as both have passed
// its end, one message sooner than the fixed holdback lets it, and the wait
// stays under a sweep where the holdback's is at least its 120 ms.
TEST(ScanMergerSources, AlignedSweepsDoNotWaitTheHoldback)
{
  Harness fixed(m20Config());
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 6; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    fixed.feed(m);
    run.feed(m);
  }
  // Newest point 100.595 from both: every window ending by then has gone, 5
  ASSERT_EQ(fixed.scans.size(), 4u);
  ASSERT_EQ(run.scans.size(), 5u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  const auto wait = run.merger.takeWait();
  EXPECT_EQ(wait.scans, 5u);
  EXPECT_LT(wait.max, 0.1);
  EXPECT_GE(fixed.merger.takeWait().max, 0.12 - 1e-9);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
  EXPECT_TRUE(run.events.empty());
}

// The rear sweep one message late: the windows wait for it, no longer, and
// every point lands in its own window.
TEST(ScanMergerSources, RearSweepBehindIsWaitedFor)
{
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 7; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k > 0) m.sweep(100.0 + 0.1 * (k - 1), kRear);
    run.feed(m);
  }
  // Rear newest 100.595: the windows ending by then, 5 (the holdback alone
  // lets 100.575 go, 5 as well -- the rear lidar is the one waited for)
  ASSERT_EQ(run.scans.size(), 5u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
  EXPECT_TRUE(run.events.empty());
}

// No rear sweep, then two (16.5 % of the 11-24 bag each): the window without
// its rear sweep waits for the message that brings it.
TEST(ScanMergerSources, MissingThenDoubleRearSweep)
{
  Harness run(m20SourcesConfig());
  auto both = [&](int k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    run.feed(m);
  };
  both(0);
  both(1);
  {
    Message m;  // no rear
    m.sweep(100.2, kFront);
    run.feed(m);
  }
  {
    Message m;  // the rear's two sweeps
    m.sweep(100.3, kFront);
    m.sweep(100.2, kRear);
    m.sweep(100.3, kRear);
    run.feed(m);
  }
  both(4);
  both(5);
  ASSERT_EQ(run.scans.size(), 5u);
  for (const auto & scan : run.scans) {
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), 10) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// A rear lidar three sweeps behind, still under the timeout: the holdback
// caps the wait, so the scans go when they would without the rings, and the
// rear's points that come after them are dropped and counted as before.
TEST(ScanMergerSources, HoldbackCapsTheWait)
{
  Harness fixed(m20Config());
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 8; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k > 2) m.sweep(100.0 + 0.1 * (k - 3), kRear);
    fixed.feed(m);
    run.feed(m);
  }
  ASSERT_EQ(run.scans.size(), fixed.scans.size());
  for (std::size_t i = 0; i < run.scans.size(); ++i) {
    EXPECT_NEAR(run.scans[i].start_time, fixed.scans[i].start_time, 1e-9);
    EXPECT_EQ(run.scans[i].cloud->size(), fixed.scans[i].cloud->size());
  }
  const auto stats = run.merger.takeStats();
  EXPECT_GT(stats.late_points, 0u);
  EXPECT_EQ(stats.late_points, fixed.merger.takeStats().late_points);
  EXPECT_NEAR(run.merger.takeWait().max, fixed.merger.takeWait().max, 1e-9);
  EXPECT_TRUE(run.events.empty());  // 0.3 s behind is not down
}

// The rear lidar stops for a second and comes back: one "down" once it is
// more than the timeout behind, the front lidar's scans no longer wait for it,
// one "up" when its points return, and nothing is lost on either side.
TEST(ScanMergerSources, StoppedLidarIsReportedOnceAndNotWaitedFor)
{
  // 0.55, not the M20's 0.5: the rear is 0.5 s behind at k 9 to the last
  // bit, and which side of 0.5 that lands on is the floating point's
  ScanMerger::Config config = m20SourcesConfig();
  config.source_timeout = 0.55;
  Harness run(config);
  for (int k = 0; k < 20; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    if (k < 5 || k >= 15) m.sweep(100.0 + 0.1 * k, kRear);
    run.feed(m);
    if (k == 9) {
      EXPECT_TRUE(run.events.empty()) << "0.5 s behind is not yet down";
    }
    if (k == 14) {
      // Down since k 10: only the front lidar to wait for, so every window
      // ending by its newest point (101.495) has gone
      ASSERT_FALSE(run.scans.empty());
      EXPECT_NEAR(run.scans.back().start_time, 101.3, 1e-9);
    }
  }
  ASSERT_EQ(run.events.size(), 2u);
  EXPECT_EQ(run.events[0].source, 1u);
  EXPECT_FALSE(run.events[0].up);
  EXPECT_NEAR(run.events[0].seconds, 101.095 - 100.495, 1e-6);
  EXPECT_EQ(run.events[1].source, 1u);
  EXPECT_TRUE(run.events[1].up);
  EXPECT_NEAR(run.events[1].seconds, 101.595 - 100.495, 1e-6);
  for (const auto & scan : run.scans) {
    const bool rear_on = scan.start_time < 100.45 || scan.start_time > 101.45;
    EXPECT_EQ(countTag(*scan.cloud, kFront), 10) << "window " << scan.start_time;
    EXPECT_EQ(countTag(*scan.cloud, kRear), rear_on ? 10 : 0) << "window " << scan.start_time;
  }
  expectOrderedWindows(run.scans, 0.1);
  EXPECT_EQ(run.merger.takeStats().late_points, 0u);
}

// A lidar that never sends a point is down `source_timeout` into the stream,
// as one that stops is; the other is not reported at all.
TEST(ScanMergerSources, NeverSeenLidarGoesDown)
{
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 8; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    run.feed(m);
  }
  ASSERT_EQ(run.events.size(), 1u);
  EXPECT_EQ(run.events[0].source, 1u);
  EXPECT_FALSE(run.events[0].up);
}

// A ring in no lidar's range: the point is in its scan as any other, counted,
// and moves no lidar's progress.
TEST(ScanMergerSources, UnmatchedRingIsKeptAndCounted)
{
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 6; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    m.sweep(100.0 + 0.1 * k, kStray, 4);
    run.feed(m);
  }
  ASSERT_EQ(run.scans.size(), 5u);
  for (const auto & scan : run.scans) EXPECT_EQ(countTag(*scan.cloud, kStray), 4);
  EXPECT_EQ(run.merger.takeStats().unmatched_points, 24u);
  EXPECT_TRUE(run.events.empty());
}

// A bag played again: the lidars' progress starts over with the stream, so
// the old, later times do not leave one of them "down" against the new ones.
TEST(ScanMergerSources, RestartResetsTheLidars)
{
  Harness run(m20SourcesConfig());
  for (int k = 0; k < 5; ++k) {
    Message m;
    m.sweep(500.0 + 0.1 * k, kFront);
    m.sweep(500.0 + 0.1 * k, kRear);
    run.feed(m);
  }
  run.scans.clear();
  for (int k = 0; k < 6; ++k) {
    Message m;
    m.sweep(100.0 + 0.1 * k, kFront);
    m.sweep(100.0 + 0.1 * k, kRear);
    run.feed(m);
  }
  EXPECT_TRUE(run.events.empty());
  ASSERT_EQ(run.scans.size(), 5u);
  EXPECT_NEAR(run.scans.front().start_time, 100.0, 1e-9);
  EXPECT_EQ(run.merger.takeStats().restarts, 1u);
}

// ---- Sweeps that end just short of the grid line (2026-11-27) ---------------

namespace
{

// One lidar's sweep as the M20 sends it (robot01, 2026-11-27): starting on
// the 100 ms grid line and ending 0.1 ms before the next, [start, start +
// 0.0999], `n` points evenly over it.
void m20Sweep(Message & m, double start, float tag, int n = 100)
{
  for (int i = 0; i < n; ++i) {
    ScanMerger::Point p;
    p.x = tag;
    p.y = static_cast<float>(i);
    m.cloud->push_back(p);
    m.times.push_back(start + 0.0999 * i / (n - 1));
    m.rings.push_back(tag == kFront ? 10 + i % 80 : 100 + i % 80);
  }
}

}  // namespace

// The newest point of a message is 0.1 ms short of its window's end. With no
// tolerance the window waits for the next message, a whole sweep; with the
// M20's 1 ms each message hands on its own scan, whole, as it arrives.
TEST(ScanMergerSources, EndToleranceHandsOnTheMessagesOwnScan)
{
  ScanMerger::Config tolerant_config = m20SourcesConfig();
  tolerant_config.end_tolerance = 0.001;
  Harness strict(m20SourcesConfig());
  Harness tolerant(tolerant_config);
  for (int k = 0; k < 6; ++k) {
    Message m;
    m20Sweep(m, 100.0 + 0.1 * k, kFront);
    m20Sweep(m, 100.0 + 0.1 * k, kRear);
    strict.feed(m);
    tolerant.feed(m);
    EXPECT_EQ(strict.scans.size(), static_cast<std::size_t>(k)) << "message " << k;
    ASSERT_EQ(tolerant.scans.size(), static_cast<std::size_t>(k + 1)) << "message " << k;
    EXPECT_NEAR(tolerant.scans.back().start_time, 100.0 + 0.1 * k, 1e-9);
    EXPECT_EQ(countTag(*tolerant.scans.back().cloud, kFront), 100);
    EXPECT_EQ(countTag(*tolerant.scans.back().cloud, kRear), 100);
  }
  expectOrderedWindows(tolerant.scans, 0.1);
  // What the filter's newest point costs: a sweep at 0, nothing with it
  EXPECT_GT(strict.merger.takeWait().max, 0.099);
  EXPECT_LT(tolerant.merger.takeWait().max, 0.001);
  EXPECT_EQ(tolerant.merger.takeStats().late_points, 0u);
  EXPECT_TRUE(tolerant.events.empty());
}

// The price: a point for the last millisecond of a window that comes after
// the window went is late -- dropped and counted, never moved into the next
// scan, whose own points are untouched.
TEST(ScanMergerSources, PointInsideTheToleranceAfterItsScanIsLate)
{
  ScanMerger::Config config = m20SourcesConfig();
  config.end_tolerance = 0.001;
  Harness run(config);
  Message first;
  m20Sweep(first, 100.0, kFront);
  m20Sweep(first, 100.0, kRear);
  run.feed(first);
  ASSERT_EQ(run.scans.size(), 1u);

  Message second;
  m20Sweep(second, 100.1, kFront);
  m20Sweep(second, 100.1, kRear);
  ScanMerger::Point straggler;
  straggler.x = kRear;
  second.cloud->push_back(straggler);
  second.times.push_back(100.0995);  // window [100.0, 100.1), already gone
  second.rings.push_back(150);
  run.feed(second);

  ASSERT_EQ(run.scans.size(), 2u);
  EXPECT_EQ(countTag(*run.scans[1].cloud, kRear), 100);
  const auto stats = run.merger.takeStats();
  EXPECT_EQ(stats.late_points, 1u);
  EXPECT_NEAR(stats.max_late, 0.0005, 1e-6);
}
