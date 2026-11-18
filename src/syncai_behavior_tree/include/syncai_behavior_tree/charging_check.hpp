#ifndef SYNCAI_BEHAVIOR_TREE__CHARGING_CHECK_HPP_
#define SYNCAI_BEHAVIOR_TREE__CHARGING_CHECK_HPP_

#include <cstdint>
#include <optional>

namespace syncai_behavior_tree
{

/**
 * @brief Whether a run of battery samples says the robot is charging: the
 * judgement of WaitForCharging, ROS-free so that test_charging_check can
 * hold it to its rules.
 *
 * The source is syncai_driver_manager's `battery_state`, whose
 * power_supply_status is derived from the sign of the pack's current with a
 * 0.1 A deadband (protocol.rs, ChargeState::from_current) -- not from the
 * BMS's mode field, whose CHG FET bit also reads set while discharging. The
 * current is the pack's NET current: the charger's minus what the robot
 * draws lying down (the computer, the lidar), so "charging" means the
 * charger delivers more than that load.
 *
 * Rules, each for a case seen or expected on a charger:
 * - Only samples given after reset() count. The one before is from the
 *   walk onto the charger, discharging, and says nothing about the contacts.
 * - A sample qualifies when it reads CHARGING, or -- at or above
 *   full_percentage -- anything but DISCHARGING: a nearly full pack takes a
 *   current that tapers into the deadband, and a robot whose contacts are
 *   made would otherwise fail for being full. The price: above
 *   full_percentage a robot that is merely not discharging passes, and that
 *   is not proof the contacts are made.
 * - Qualifying samples must span hold_time without a sample between them
 *   that does not qualify: one sample of current is a contact touching once
 *   as the body settles. The span is measured between samples, not to the
 *   clock, so a BMS that falls silent after one good sample does not pass by
 *   the passing of time.
 *
 * Times are seconds on any monotonic clock.
 */
class ChargingCheck
{
public:
  // BatteryState's power_supply_status values (sensor_msgs), copied so that
  // this header needs no message package.
  static constexpr uint8_t kStatusCharging = 1;
  static constexpr uint8_t kStatusDischarging = 2;

  ChargingCheck(double hold_time, double full_percentage)
  : hold_time_(hold_time), full_percentage_(full_percentage)
  {
  }

  /** @brief Forget every sample; the next one starts the run */
  void reset() { run_start_.reset(); }

  /**
   * @brief Take one sample, given in arrival order
   * @param percentage 0-1, BatteryState's; NaN when unmeasured (never "full")
   * @return true once the run of qualifying samples spans hold_time
   */
  bool add(double t, uint8_t status, double percentage)
  {
    if (!qualifies(status, percentage)) {
      run_start_.reset();
      return false;
    }
    if (!run_start_) {
      run_start_ = t;
    }
    return t - *run_start_ >= hold_time_;
  }

  bool qualifies(uint8_t status, double percentage) const
  {
    if (status == kStatusCharging) {
      return true;
    }
    // NaN compares false: an unmeasured charge is not a full one.
    return percentage >= full_percentage_ && status != kStatusDischarging;
  }

private:
  double hold_time_;
  double full_percentage_;
  std::optional<double> run_start_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__CHARGING_CHECK_HPP_
