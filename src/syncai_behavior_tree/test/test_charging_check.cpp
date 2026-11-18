// ChargingCheck: when the battery samples say the robot is charging on its
// charger. ROS-free.

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "syncai_behavior_tree/charging_check.hpp"

namespace
{

using syncai_behavior_tree::ChargingCheck;

constexpr uint8_t kCharging = ChargingCheck::kStatusCharging;
constexpr uint8_t kDischarging = ChargingCheck::kStatusDischarging;
constexpr uint8_t kNotCharging = 3;  // BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

TEST(ChargingCheck, ChargingHeldForHoldTimePasses)
{
  ChargingCheck check(2.0, 0.95);
  EXPECT_FALSE(check.add(0.0, kCharging, 0.5));
  EXPECT_FALSE(check.add(1.0, kCharging, 0.5));
  EXPECT_FALSE(check.add(1.9, kCharging, 0.5));
  EXPECT_TRUE(check.add(2.0, kCharging, 0.5));
}

TEST(ChargingCheck, ASampleOutsideTheRunRestartsIt)
{
  // A contact touching once as the body settles is not a charge.
  ChargingCheck check(2.0, 0.95);
  EXPECT_FALSE(check.add(0.0, kCharging, 0.5));
  EXPECT_FALSE(check.add(1.5, kNotCharging, 0.5));
  EXPECT_FALSE(check.add(2.0, kCharging, 0.5));
  EXPECT_FALSE(check.add(3.5, kCharging, 0.5));
  EXPECT_TRUE(check.add(4.0, kCharging, 0.5));
}

TEST(ChargingCheck, OneSampleIsNotARunHowEverLongAgo)
{
  // Measured between samples: a BMS that went quiet after one good sample
  // never passes; the node's timeout ends it.
  ChargingCheck check(2.0, 0.95);
  EXPECT_FALSE(check.add(0.0, kCharging, 0.5));
  EXPECT_TRUE(check.add(30.0, kCharging, 0.5));  // a second sample, much later, does
  ChargingCheck single(2.0, 0.95);
  EXPECT_FALSE(single.add(5.0, kCharging, 0.5));
}

TEST(ChargingCheck, ZeroHoldPassesOnTheFirstSample)
{
  ChargingCheck check(0.0, 0.95);
  EXPECT_TRUE(check.add(0.3, kCharging, 0.5));
}

TEST(ChargingCheck, AFullPackNeedsOnlyNotDischarging)
{
  // The current of a nearly full pack tapers into driver_manager's 0.1 A
  // deadband (NOT_CHARGING) with the contacts made.
  ChargingCheck check(1.0, 0.95);
  EXPECT_FALSE(check.add(0.0, kNotCharging, 0.97));
  EXPECT_TRUE(check.add(1.0, kNotCharging, 0.97));
  EXPECT_TRUE(check.qualifies(kNotCharging, 0.95));
  EXPECT_TRUE(check.qualifies(kCharging, 0.99));
}

TEST(ChargingCheck, DischargingNeverPasses)
{
  ChargingCheck check(1.0, 0.95);
  EXPECT_FALSE(check.qualifies(kDischarging, 0.5));
  EXPECT_FALSE(check.qualifies(kDischarging, 1.0));
  EXPECT_FALSE(check.add(0.0, kDischarging, 1.0));
  EXPECT_FALSE(check.add(5.0, kDischarging, 1.0));
}

TEST(ChargingCheck, NotChargingBelowFullFails)
{
  ChargingCheck check(1.0, 0.95);
  EXPECT_FALSE(check.qualifies(kNotCharging, 0.94));
  EXPECT_FALSE(check.add(0.0, kNotCharging, 0.6));
  EXPECT_FALSE(check.add(5.0, kNotCharging, 0.6));
}

TEST(ChargingCheck, AnUnmeasuredChargeIsNotFull)
{
  ChargingCheck check(1.0, 0.95);
  EXPECT_FALSE(check.qualifies(kNotCharging, kNaN));
  EXPECT_TRUE(check.qualifies(kCharging, kNaN));
}

TEST(ChargingCheck, ResetForgetsTheRun)
{
  ChargingCheck check(2.0, 0.95);
  EXPECT_FALSE(check.add(0.0, kCharging, 0.5));
  check.reset();
  EXPECT_FALSE(check.add(2.5, kCharging, 0.5));
  EXPECT_TRUE(check.add(4.5, kCharging, 0.5));
}

}  // namespace
