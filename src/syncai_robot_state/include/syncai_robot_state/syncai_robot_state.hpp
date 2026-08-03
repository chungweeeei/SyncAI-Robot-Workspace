#ifndef SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_
#define SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "syncai_common/msg/motor_states.hpp"
#include "syncai_common/msg/robot_state.hpp"
#include "syncai_common/msg/wifi_status.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_robot_state
{
// Aggregates the sensor topics into one syncai_common/RobotState, published at
// 10 Hz on the relative topic `robot_state` (so it lands on
// <robot_id>/robot_state and stays inside this robot's namespace, like every
// other topic in the stack).
//
// One publisher, one timer, one consumer: syncai_backend, which re-serialises a
// subset for GET /api/v1/robot/state. The message carries operator-facing detail
// (per-joint temperatures, motor error codes) that must NOT reach that REST
// payload — nothing but the router's explicit field list stops it.
//
// The node also derives the `state` field from what it sees. Three of the six
// RobotStatus values are emitted today, in this precedence:
//
//   UNINITIALIZED   the map -> base_link TF is unavailable
//   WARNING         battery below the low-battery threshold (latched)
//   IDLE            otherwise
//
// It also carries the gait controller's own low-level state machine through from
// the `mode` topic onto RobotState.low_level_mode, and NOTHING ELSE: no staleness
// verdict, no latch, no contribution to `state`, and no freshness field. That
// field is the stack's only measured counterpart to the one-way SetPolicyMode /
// SetMotionKey commands, and keeping it dumb is what keeps this node's "holds no
// state beyond the latest sample" property true.
//
// It only ever reports. No threshold here commands the robot to do anything.
class RobotStateNode : public rclcpp::Node
{
public:
  RobotStateNode();
  ~RobotStateNode();

private:
  void initParameters();

  // Advances the health latches from the cached samples. Called from onTimer()
  // before buildState(), so the latches and the message it produces always come
  // from the same tick.
  void updateHealthLatches();

  // Builds the message from the current TF and cached samples.
  //
  // Keep this a pure read of the health latches — advancing them is
  // updateHealthLatches()' job. Splitting it that way means a dwell counter or
  // rate-limited transition added later has exactly one place it can live, and
  // that place ticks once per publish.
  syncai_common::msg::RobotState buildState();

  // Periodic build-and-publish.
  void onTimer();
  // Caches the latest forward velocity from the odom topic.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Caches the latest battery state from the battery_state topic.
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // Caches the latest WiFi status from the wifi_status topic.
  void wifiStatusCallback(const syncai_common::msg::WifiStatus::SharedPtr msg);

  // Caches the latest per-joint sample (temperatures / torques / error codes)
  // from the motor_states topic.
  void motorStatesCallback(const syncai_common::msg::MotorStates::SharedPtr msg);

  // Caches the gait controller's low-level state machine from the `mode` topic
  // (data[0] = policy state, data[1] = motion state). Samples with fewer than two
  // elements are REJECTED here, not padded.
  void lowLevelModeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<rclcpp::TimerBase> timer_;
  // Dedicated group so the timer's latch/TF/build/publish work runs
  // independently of the (lightweight) sensor subscription callbacks under a
  // multi-threaded executor.
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  // Relative topic name, so it inherits the robot_id namespace.
  std::shared_ptr<rclcpp::Publisher<syncai_common::msg::RobotState>> robot_state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<syncai_common::msg::WifiStatus>::SharedPtr wifi_sub_;
  rclcpp::Subscription<syncai_common::msg::MotorStates>::SharedPtr motor_states_sub_;
  // The TOPIC is `mode`; the FIELD it feeds is `low_level_mode`. RobotState.mode
  // is a different thing entirely (RobotMode, hardcoded to AUTO).
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr low_level_mode_sub_;

  // Latest odom / battery / wifi / motor / low-level-mode samples, guarded because
  // they are written from their subscription callbacks and read from the timer
  // callback.
  std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;
  syncai_common::msg::WifiStatus::SharedPtr latest_wifi_status_;
  syncai_common::msg::MotorStates::SharedPtr latest_motor_states_;

  // Decoded, rather than cached as a SharedPtr like the four above: the payload is
  // two ints, so there is nothing to keep alive by holding the message, and
  // validating in the callback means a short data[] never lands here — buildState()
  // is a two-line copy with no branch and no bounds check.
  //
  // Both are written together under mutex_ and read together under mutex_, so the
  // pair is always internally consistent.
  //
  // These carry NO indication of whether anything has ever been heard: 0 / 0 is
  // both the initial value and a genuine "PPO / Stand". A receipt timestamp used
  // to sit here and make the difference visible; it was removed on request. If it
  // comes back, the one invariant that mattered was that a REJECTED sample must
  // not refresh it, or a stream of malformed messages makes a frozen reading look
  // live.
  int32_t low_level_policy_state_{0};
  int32_t low_level_motion_state_{0};

  // Health latches. Written ONLY by updateHealthLatches(), read by buildState().
  //
  // Not under mutex_: they are touched exclusively from the timer, which has its
  // own MutuallyExclusive callback group. mutex_ guards the sample caches above,
  // which the subscription callbacks write from other threads — a different
  // problem.
  bool low_battery_latched_{false};

  // Parameters.
  std::string robot_id_;
  std::string map_name_;
  std::string global_frame_, base_frame_;
  double publish_rate_;
  double transform_tolerance_;
  // Hysteresis pair for low_battery_latched_, in percent (0-100). Load-time
  // only: this node has no on-set-parameters callback.
  double low_battery_warn_percentage_, low_battery_clear_percentage_;
};
}  // namespace syncai_robot_state

#endif  // SYNCAI_ROBOT_STATE__SYNCAI_ROBOT_STATE_HPP_
