#ifndef SYNCAI_CONTROLLER__TRACK_TARGET_FILTER_HPP_
#define SYNCAI_CONTROLLER__TRACK_TARGET_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

#include "syncai_controller/planar_pose.hpp"

namespace syncai_controller
{

struct TrackTargetFilterParams
{
  // Seconds of targets kept, back from the newest one's stamp. 0: no filter
  // -- the newest target is the target, one is enough, and nothing is ever
  // unstable (TrackPose.target_filter_window).
  double window{0.0};
  // With a window: fewer targets than this in it is no target yet. A median
  // of one or two is whichever came first.
  std::size_t min_poses{3};
  // With a window: the targets are "one target" while kSpreadQuantile of them
  // lie within these of the median.
  double max_spread_xy{0.08};
  double max_spread_yaw{0.2};
};

/**
 * @class syncai_controller::TrackTargetFilter
 * @brief The targets track_pose has in odom, and the one it drives to.
 * ROS-free (test/test_track_target_filter.cpp).
 *
 * A charger stands still, so every sighting of it, taken into odom at its own
 * stamp, should land on the same point. Two things follow, and both need the
 * target to be held in a fixed frame (which is why track_pose tracks in odom
 * and not in the robot's own frame):
 *  - the MEDIAN of the last `window` seconds is a better target than the
 *    newest sighting. A tag's yaw is its noisiest reading and the offset from
 *    the tag turns it into centimetres across the target's axis -- 1 deg at
 *    1 m is 2.3 cm, more than the ApproachLaw's lateral band, so unfiltered
 *    the strafe starts and stops on noise;
 *  - when the sightings do NOT land on one point, something a blind approach
 *    must not drive on is wrong: the odometry is diverging or was reset under
 *    the goal (the robot's pose, and with it every new target, runs away), or
 *    the publisher flips between two solutions (a tag's mirrored pose). That
 *    is stable() == false, on which track_pose stops the robot.
 * The spread is the kSpreadQuantile-th deviation from the median, not the
 * largest: one stray sighting in ten is exactly what a median ignores, and
 * stopping for a window's length on each would make the filter worse than
 * none.
 *
 * freeze() holds the current target against further sightings (track_pose:
 * while the robot settles inside the goal checker's tolerances). Sightings
 * keep filling the window meanwhile, so unfreezing resumes on a current one.
 */
class TrackTargetFilter
{
public:
  static constexpr double kSpreadQuantile = 0.8;

  void configure(const TrackTargetFilterParams & params)
  {
    params_ = params;
    clear();
  }

  void clear()
  {
    samples_.clear();
    frozen_ = false;
  }

  /** @brief A target in odom, true at `stamp` (seconds). Any order. */
  void add(double stamp, const Pose2 & target)
  {
    samples_.push_back(Sample{stamp, target});
    // Back from the newest stamp, not from the clock: when the sightings
    // stop, the last window stays (how old it is is track_pose's
    // pose_timeout to judge). Without a window that leaves the newest alone,
    // whatever order they came in.
    const double oldest_kept = newestStamp() - std::max(params_.window, 0.0);
    samples_.erase(
      std::remove_if(
        samples_.begin(), samples_.end(),
        [oldest_kept](const Sample & s) { return s.stamp < oldest_kept; }),
      samples_.end());
  }

  std::size_t size() const { return samples_.size(); }

  /** @brief Stamp of the newest target held; -1 with none. */
  double newestStamp() const
  {
    double newest = -1.0;
    for (const auto & s : samples_) {
      newest = std::max(newest, s.stamp);
    }
    return newest;
  }

  /** @brief Enough targets to name one (or a frozen one is held). */
  bool hasTarget() const
  {
    if (frozen_) {
      return true;
    }
    return params_.window > 0.0 ? samples_.size() >= params_.min_poses : !samples_.empty();
  }

  /** @brief The target to drive to. Meaningful only with hasTarget(). */
  Pose2 target() const { return frozen_ ? frozen_target_ : median(); }

  /** @brief The window's spread about its median (0 without a window). */
  void spread(double & xy, double & yaw) const
  {
    xy = 0.0;
    yaw = 0.0;
    if (params_.window <= 0.0 || samples_.empty()) {
      return;
    }
    const Pose2 m = median();
    std::vector<double> dxy;
    std::vector<double> dyaw;
    for (const auto & s : samples_) {
      dxy.push_back(std::hypot(s.pose.x - m.x, s.pose.y - m.y));
      dyaw.push_back(std::fabs(normalizeAngle(s.pose.yaw - m.yaw)));
    }
    xy = quantile(dxy, kSpreadQuantile);
    yaw = quantile(dyaw, kSpreadQuantile);
  }

  /** @brief The window's targets agree on one. Always true without a window. */
  bool stable() const
  {
    double xy = 0.0;
    double yaw = 0.0;
    spread(xy, yaw);
    return xy <= params_.max_spread_xy && yaw <= params_.max_spread_yaw;
  }

  /** @brief Hold the current target (needs hasTarget()); no-op when frozen. */
  void freeze()
  {
    if (!frozen_ && hasTarget()) {
      frozen_target_ = median();
      frozen_ = true;
    }
  }
  void unfreeze() { frozen_ = false; }
  bool frozen() const { return frozen_; }

  const TrackTargetFilterParams & params() const { return params_; }

private:
  struct Sample
  {
    double stamp;
    Pose2 pose;
  };

  static double quantile(std::vector<double> values, double q)
  {
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    const std::size_t k =
      std::min(n - 1, static_cast<std::size_t>(std::ceil(q * static_cast<double>(n))) - 1);
    return values[k];
  }

  static double middle(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
  }

  // x and y each on their own; the yaw as the median of the angles about
  // their circular mean, so a window straddling +-pi has one.
  Pose2 median() const
  {
    if (samples_.empty()) {
      return Pose2{};
    }
    if (samples_.size() == 1) {
      return samples_.front().pose;
    }
    std::vector<double> xs;
    std::vector<double> ys;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const auto & s : samples_) {
      xs.push_back(s.pose.x);
      ys.push_back(s.pose.y);
      sum_sin += std::sin(s.pose.yaw);
      sum_cos += std::cos(s.pose.yaw);
    }
    const double mean_yaw = std::atan2(sum_sin, sum_cos);
    std::vector<double> offsets;
    for (const auto & s : samples_) {
      offsets.push_back(normalizeAngle(s.pose.yaw - mean_yaw));
    }
    return Pose2{middle(xs), middle(ys), normalizeAngle(mean_yaw + middle(offsets))};
  }

  TrackTargetFilterParams params_;
  std::deque<Sample> samples_;
  bool frozen_{false};
  Pose2 frozen_target_;
};

}  // namespace syncai_controller

#endif  // SYNCAI_CONTROLLER__TRACK_TARGET_FILTER_HPP_
