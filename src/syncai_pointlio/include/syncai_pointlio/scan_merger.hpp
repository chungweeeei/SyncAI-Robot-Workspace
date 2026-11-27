#ifndef SYNCAI_POINTLIO__SCAN_MERGER_HPP_
#define SYNCAI_POINTLIO__SCAN_MERGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace syncai_pointlio
{

// Turns lidar messages whose points carry their own absolute time into scans
// the filter can run through in time order (2026-11-25). ROS-free;
// test/test_scan_merger.cpp.
//
// Why it exists: the Lynx M20's /LIDAR/POINTS_UDP is two lidars merged by the
// vendor, and a message is not one moment or even one sweep. Over an 816 s bag
// (robot01, 2026-11-24, 8156 messages) the front lidar's sweep always ended
// ~22 ms before the header stamp, and the rear lidar's was in the same 100 ms
// window in 52 % of the messages, one sweep older in 15 %, absent in 16.5 %
// and two sweeps long in the 16.5 % that followed those. Taken whole at the
// header stamp, as the PointCloud2 path did, a front point was on average
// 73 ms older than the time it was given and a rear point 106 ms (up to
// 316 ms), with nothing to deskew by: walls came out about twice as thick as a
// G23 map's.
//
// Giving each point its own time is not enough by itself. The filter only
// steps forward (PointEKF::predictState returns on dt <= 0), so a point older
// than the filter's time is projected with the pose of now -- the same error
// under another name -- and a rear sweep that arrives one message late is
// 100 ms older than the front sweep already run. So the points wait here:
// a scan is handed on only once the late lidar's points for it have arrived
// -- `holdback` older than the newest point seen, or sooner when the sources
// are told apart (below) -- and what still arrives later than that is dropped
// and counted (takeStats()).
//
// The scans are cut on a time grid of our own (`scan_period`), not on either
// lidar's sweep or the message: a message holding two sweeps becomes two
// scans, and two lidars turning out of phase would otherwise hand the filter
// scans of alternating odd lengths. Each scan is every point of every source
// with a time in [start, start + scan_period), its curvature the offset from
// `start` in ms -- what MapBuilder and deskewToEndBody read as per-point time.
//
// Waiting for the lidar that is behind (2026-11-25). With no sources the wait
// is fixed: a scan goes once it is `holdback` behind the newest point, 120 ms
// on the M20, though in half its messages both lidars are already in. Given
// `source_bounds`, every point is put down to a source by an integer field of
// the cloud (the M20's ring: 0-95 the front lidar, 96-191 the rear) and each
// source's newest point is tracked, so a window goes as soon as every live
// source has passed its end -- the wait the late lidar needs, and no more --
// and still at the latest `holdback` behind the newest point, so it never
// waits longer than without sources. A source whose newest point falls more
// than `source_timeout` behind the newest of all is down: no longer waited
// for, and reported (takeEvents()) once, as is its return. Point time, not the
// wall clock, decides it, so a bag replayed at any rate reads the same. A point
// whose field value is in no source's range is kept and counted
// (Stats::unmatched_points) but moves no source. This assumes each lidar's own
// points arrive in time order; one that came back in time would be late for
// its window, dropped and counted as any late point is.
class ScanMerger
{
public:
  using Point = pcl::PointXYZINormal;
  using Cloud = pcl::PointCloud<Point>;

  struct Config
  {
    // How far behind the newest point a scan's end must be before it is handed
    // on, s -- with sources, the longest it waits for one that is late. 0 hands
    // every window on as soon as a newer point is seen (or, with sources, as
    // soon as they have all passed it), and drops whatever comes for it
    // afterwards.
    double holdback = 0.0;
    // Length of a scan, s
    double scan_period = 0.1;
    // A message whose newest point is this much older than the newest seen is
    // a restarted stream (a bag played again, a driver restart): start over
    // rather than drop everything as late until the old time is reached.
    double restart_gap = 1.0;
    // The sources a point is put down to by its field value (Timed::sources):
    // source i is [source_bounds[i], source_bounds[i + 1]). Empty (or fewer
    // than two bounds): no sources, the fixed holdback alone. The caller checks
    // that the bounds increase.
    std::vector<long> source_bounds;
    // A source this far behind the newest point of all is down, s
    double source_timeout = 0.5;
    // How close to a window's end the watermark must come for the window to
    // be complete, s: it goes once the watermark is at end - end_tolerance.
    // 0 = at the end itself. A point that comes for [end - end_tolerance, end)
    // after its window went is late, dropped and counted like any other. The
    // caller keeps it under scan_period.
    //
    // Why (2026-11-27): the M20's two lidars are synchronised to whole tenths
    // of a second, so a message is one full sweep of each, [g, g + 99.9 ms],
    // on exactly this grid -- its newest point 0.1 ms short of the window's
    // end. At 0 no message completed its own window: every scan waited for
    // the next message, a whole sweep (robot01: `merge wait` 56-91 ms mean,
    // lio_odom ~200 ms old when received). The next sweep starts on the grid
    // line, so a tolerance well under a sweep takes nothing of it.
    double end_tolerance = 0.0;
  };

  // One message's points and their absolute times, index for index
  struct Timed
  {
    Cloud::Ptr cloud;
    const std::vector<double> * times = nullptr;
    // Each point's source field value (the M20's ring), index for index; null
    // or of another length: no point moves a source. Read for the filter's
    // cloud only -- the dense one follows its windows.
    const std::vector<long> * sources = nullptr;
  };

  struct Scan
  {
    double start_time = 0.0;
    Cloud::Ptr cloud;
    // The same window of the dense input; `cloud` itself when there is none
    Cloud::Ptr dense;
  };

  // Since the last takeStats()
  struct Stats
  {
    std::size_t points = 0;       // points given to add(), the filter's cloud only
    std::size_t late_points = 0;  // of those, dropped: their scan had been handed on
    double max_late = 0.0;        // the furthest one was behind its scan's end, s
    std::size_t scans = 0;
    std::size_t empty_windows = 0;  // grid windows with no point, not handed on
    std::size_t restarts = 0;
    std::size_t unmatched_points = 0;  // with sources: points in no source's range
  };

  // How long the scans handed on waited, in point time: the newest point seen
  // minus the scan's end, when it went. With holdback alone this is at least
  // the holdback; with sources it is what the late lidar cost.
  struct Wait
  {
    std::size_t scans = 0;
    double sum = 0.0;
    double max = 0.0;
  };

  // A source going down or coming back, in the order they happened
  struct SourceEvent
  {
    std::size_t source = 0;
    bool up = false;
    // Down: how far its newest point was behind the newest of all. Up: how
    // long it had been down, both in point time, s.
    double seconds = 0.0;
  };

  explicit ScanMerger(const Config & config) : m_config(config)
  {
    if (m_config.source_bounds.size() >= 2) m_sources.resize(m_config.source_bounds.size() - 1);
  }

  const Config & config() const { return m_config; }

  std::size_t numSources() const { return m_sources.size(); }

  // Source i's field values, [first, second)
  std::pair<long, long> sourceRange(std::size_t i) const
  {
    return {m_config.source_bounds[i], m_config.source_bounds[i + 1]};
  }

  // Leaves the events not yet taken: a source that went down before a reset
  // did go down.
  void reset()
  {
    m_pending.clear();
    m_pending_dense.clear();
    m_started = false;
    m_newest = -std::numeric_limits<double>::infinity();
    for (auto & s : m_sources) s = Source();
  }

  Stats takeStats()
  {
    Stats out = m_stats;
    m_stats = Stats();
    return out;
  }

  Wait takeWait()
  {
    Wait out = m_wait;
    m_wait = Wait();
    return out;
  }

  std::vector<SourceEvent> takeEvents()
  {
    std::vector<SourceEvent> out;
    out.swap(m_events);
    return out;
  }

  // Takes one message and appends to `out` every scan that is now complete, in
  // time order. `dense` is the same message at another decimation (may be
  // empty: Timed()), cut into the same windows.
  void add(const Timed & sparse, const Timed & dense, std::vector<Scan> & out)
  {
    if (!sparse.cloud || !sparse.times || sparse.cloud->size() != sparse.times->size()) return;
    if (sparse.cloud->empty()) return;
    const bool has_dense = dense.cloud && dense.times && dense.cloud != sparse.cloud &&
                           dense.cloud->size() == dense.times->size();

    const auto [min_it, max_it] = std::minmax_element(sparse.times->begin(), sparse.times->end());
    if (m_started && *max_it < m_newest - m_config.restart_gap) {
      reset();
      ++m_stats.restarts;
    }
    if (!m_started) {
      // The grid is whole multiples of scan_period, so where its lines fall
      // does not depend on which point happened to come first. A scan need
      // not be one sweep: any scan_period of a spinning lidar is a full turn.
      m_window_start = std::floor(*min_it / m_config.scan_period) * m_config.scan_period;
      m_started = true;
      // Every source starts at the stream's start, so one that never sends a
      // point is down `source_timeout` in, as one that stops is.
      for (auto & s : m_sources) {
        s.newest = *min_it;
        s.down = false;
      }
    }
    m_newest = std::max(m_newest, *max_it);
    m_has_dense = has_dense;

    m_stats.points += sparse.cloud->size();
    append(sparse, m_pending, true);
    if (has_dense) append(dense, m_pending_dense, false);
    if (!m_sources.empty()) trackSources(sparse);

    const double watermark = this->watermark();
    while (m_window_start + m_config.scan_period - m_config.end_tolerance <= watermark) {
      // A stream that stopped for a while (a pause in a bag, a dead lidar)
      // leaves windows with nothing in them: go to the one the oldest pending
      // point is in, instead of walking the gap a window at a time.
      if (m_pending.empty()) break;
      double oldest = std::numeric_limits<double>::infinity();
      for (const auto & p : m_pending) oldest = std::min(oldest, p.time);
      if (oldest >= m_window_start + m_config.scan_period) {
        const double skip = std::floor((oldest - m_window_start) / m_config.scan_period);
        m_stats.empty_windows += static_cast<std::size_t>(skip);
        m_window_start += skip * m_config.scan_period;
        continue;
      }

      Scan scan;
      scan.start_time = m_window_start;
      scan.cloud = cut(m_pending);
      scan.dense = m_has_dense ? cut(m_pending_dense) : scan.cloud;
      m_window_start += m_config.scan_period;
      if (scan.cloud->empty()) {
        ++m_stats.empty_windows;
        continue;
      }
      ++m_stats.scans;
      const double wait = m_newest - (scan.start_time + m_config.scan_period);
      ++m_wait.scans;
      m_wait.sum += wait;
      m_wait.max = std::max(m_wait.max, wait);
      out.push_back(std::move(scan));
    }
  }

private:
  struct Source
  {
    double newest = -std::numeric_limits<double>::infinity();
    bool down = false;
    double down_since = 0.0;  // its newest point when it went down
  };

  // The time every window ending at or before it is complete: the newest
  // point less the holdback, or later -- when every live source has passed it.
  double watermark() const
  {
    const double fixed = m_newest - m_config.holdback;
    double slowest = std::numeric_limits<double>::infinity();
    for (const auto & s : m_sources) {
      if (!s.down) slowest = std::min(slowest, s.newest);
    }
    if (!std::isfinite(slowest)) return fixed;
    return std::max(fixed, slowest);
  }

  // Moves each source's newest point, then marks the sources down or up again
  void trackSources(const Timed & in)
  {
    const auto & times = *in.times;
    const bool has_sources = in.sources && in.sources->size() == times.size();
    if (has_sources) {
      const auto & values = *in.sources;
      const auto & bounds = m_config.source_bounds;
      for (std::size_t i = 0; i < times.size(); ++i) {
        // upper_bound: the first bound above the value; the source is the one
        // before it, if the value is not below the first bound or at the last
        const auto it = std::upper_bound(bounds.begin(), bounds.end(), values[i]);
        if (it == bounds.begin() || it == bounds.end()) {
          ++m_stats.unmatched_points;
          continue;
        }
        Source & s = m_sources[static_cast<std::size_t>(it - bounds.begin()) - 1];
        s.newest = std::max(s.newest, times[i]);
      }
    }
    // No field values at all (the cloud lacks the field) moves no source
    // either; the caller says so once, rather than counting every point.
    for (std::size_t i = 0; i < m_sources.size(); ++i) {
      Source & s = m_sources[i];
      const double behind = m_newest - s.newest;
      const bool late = behind > m_config.source_timeout;
      if (late && !s.down) {
        s.down = true;
        s.down_since = s.newest;
        m_events.push_back({i, false, behind});
      } else if (!late && s.down) {
        s.down = false;
        m_events.push_back({i, true, s.newest - s.down_since});
      }
    }
  }

  struct Pending
  {
    double time;
    Point point;
  };

  void append(const Timed & in, std::vector<Pending> & pending, bool count)
  {
    pending.reserve(pending.size() + in.cloud->size());
    for (std::size_t i = 0; i < in.cloud->size(); ++i) {
      const double t = (*in.times)[i];
      if (t < m_window_start) {
        // Its scan has gone to the filter, which cannot step back to it
        if (count) {
          ++m_stats.late_points;
          m_stats.max_late = std::max(m_stats.max_late, m_window_start - t);
        }
        continue;
      }
      pending.push_back({t, in.cloud->points[i]});
    }
  }

  // Moves the points of [m_window_start, m_window_start + scan_period) out of
  // `pending` into a cloud, their curvature the offset from the start in ms.
  Cloud::Ptr cut(std::vector<Pending> & pending) const
  {
    const double end = m_window_start + m_config.scan_period;
    Cloud::Ptr cloud(new Cloud);
    std::size_t kept = 0;
    for (std::size_t i = 0; i < pending.size(); ++i) {
      if (pending[i].time < end) {
        Point p = pending[i].point;
        p.curvature = static_cast<float>((pending[i].time - m_window_start) * 1000.0);
        cloud->push_back(p);
      } else {
        pending[kept++] = pending[i];
      }
    }
    pending.resize(kept);
    return cloud;
  }

  Config m_config;
  std::vector<Pending> m_pending;
  std::vector<Pending> m_pending_dense;
  bool m_started = false;
  bool m_has_dense = false;
  double m_window_start = 0.0;
  double m_newest = -std::numeric_limits<double>::infinity();
  Stats m_stats;
  Wait m_wait;
  std::vector<Source> m_sources;
  std::vector<SourceEvent> m_events;
};

}  // namespace syncai_pointlio

#endif  // SYNCAI_POINTLIO__SCAN_MERGER_HPP_
