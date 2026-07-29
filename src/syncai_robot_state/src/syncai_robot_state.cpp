#include "syncai_robot_state/syncai_robot_state.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <functional>

#include "syncai_common/msg/robot_mode.hpp"
#include "syncai_common/msg/robot_status.hpp"
#include "syncai_util/robot_utils.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"

namespace syncai_robot_state
{
using std::placeholders::_1;

namespace
{
// Hz -> wall-timer period. A non-positive rate would make create_wall_timer
// either throw or spin as fast as the executor allows, so it is clamped to a
// slow but harmless 1 Hz rather than trusted.
std::chrono::nanoseconds periodFromRate(double rate_hz)
{
  const double safe_rate = (rate_hz > 0.0) ? rate_hz : 1.0;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_rate));
}
}  // namespace

RobotStateNode::RobotStateNode() : Node("syncai_robot_state")
{
  initParameters();

  // register tf buffer and listener
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // Relative name, so it lands on <robot_id>/robot_state and stays inside this
  // robot's namespace like every other topic in the stack.
  //
  // Single publisher with latest-value semantics, so depth 1 is enough.
  // BEST_EFFORT matches odom / battery_state / wifi_status and the nature of a
  // periodic snapshot — but note the consequence for anyone writing a new
  // subscriber: a best-effort publisher cannot satisfy a RELIABLE subscriber, so
  // subscribing with the rclcpp/rclpy default QoS receives NOTHING.
  robot_state_pub_ = this->create_publisher<syncai_common::msg::RobotState>(
    "robot_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&RobotStateNode::odomCallback, this, _1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS(),
    std::bind(&RobotStateNode::batteryCallback, this, _1));

  wifi_sub_ = this->create_subscription<syncai_common::msg::WifiStatus>(
    "wifi_status", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    std::bind(&RobotStateNode::wifiStatusCallback, this, _1));

  // SensorDataQoS to match the publisher in syncai_driver_manager; a mismatched
  // reliability here would silently receive nothing.
  motor_states_sub_ = this->create_subscription<syncai_common::msg::MotorStates>(
    "motor_states", rclcpp::SensorDataQoS(),
    std::bind(&RobotStateNode::motorStatesCallback, this, _1));

  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
    periodFromRate(publish_rate_), std::bind(&RobotStateNode::onTimer, this), timer_cb_group_);
}

RobotStateNode::~RobotStateNode()
{
}

void RobotStateNode::initParameters()
{
  this->declare_parameter("robot_id", std::string(""));
  this->get_parameter("robot_id", robot_id_);
  RCLCPP_INFO(this->get_logger(), "[RobotStateNode][%s] robot_id: %s", __func__, robot_id_.c_str());

  this->declare_parameter("map", std::string(""));
  this->get_parameter("map", map_name_);
  RCLCPP_INFO(this->get_logger(), "[RobotStateNode][%s] map: %s", __func__, map_name_.c_str());

  this->declare_parameter("global_frame", std::string("map"));
  this->get_parameter("global_frame", global_frame_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] global_frame: %s", __func__, global_frame_.c_str());

  this->declare_parameter("base_frame", std::string("base_link"));
  this->get_parameter("base_frame", base_frame_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] base_frame: %s", __func__, base_frame_.c_str());

  this->declare_parameter("transform_tolerance", 0.1);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] transform_tolerance: %f", __func__,
    transform_tolerance_);

  // 10 Hz: the readers are operators watching joint temperatures or chasing a
  // localization dropout, so the channel should react. Note that `timestamp` has
  // whole-second resolution, so raising this does not make the message any more
  // orderable — see RobotState.msg.
  this->declare_parameter("publish_rate", 10.0);
  this->get_parameter("publish_rate", publish_rate_);
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] publish_rate: %f Hz", __func__, publish_rate_);

  // Low-battery hysteresis, in percent. 20% is not a new number: it is the
  // threshold in syncai_driver_manager's unwired
  // "TODO: trigger safety shutdown when soc < 20%", the one the reference
  // implementation (GaitMPC udp_ros_bridge) acts on, and the one the frontend's
  // status strip already hardcodes for its battery colour.
  this->declare_parameter("low_battery_warn_percentage", 20.0);
  this->get_parameter("low_battery_warn_percentage", low_battery_warn_percentage_);
  this->declare_parameter("low_battery_clear_percentage", 25.0);
  this->get_parameter("low_battery_clear_percentage", low_battery_clear_percentage_);

  // A clear threshold at or below the warn threshold is not a usable hysteresis
  // band — equal means no hysteresis at all (the state flaps on sensor noise at
  // 10 Hz), lower means the latch can never clear. Neither is worth honouring
  // silently, so fall back to the defaults as a pair.
  if (low_battery_clear_percentage_ <= low_battery_warn_percentage_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[RobotStateNode][%s] low_battery_clear_percentage (%f) must exceed "
      "low_battery_warn_percentage (%f); falling back to 20/25",
      __func__, low_battery_clear_percentage_, low_battery_warn_percentage_);
    low_battery_warn_percentage_ = 20.0;
    low_battery_clear_percentage_ = 25.0;
  }
  RCLCPP_INFO(
    this->get_logger(), "[RobotStateNode][%s] low battery: warn below %f%%, clear above %f%%",
    __func__, low_battery_warn_percentage_, low_battery_clear_percentage_);
}

void RobotStateNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_ = msg;
}

void RobotStateNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_battery_ = msg;
}

void RobotStateNode::wifiStatusCallback(const syncai_common::msg::WifiStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_wifi_status_ = msg;
}

void RobotStateNode::motorStatesCallback(const syncai_common::msg::MotorStates::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_motor_states_ = msg;
}

void RobotStateNode::updateHealthLatches()
{
  // Hysteresis rather than a bare comparison: at the 10 Hz publish rate a pack
  // sitting on the threshold would flip the state ten times a second, and a state
  // that flaps is a state nobody can act on. Entering at 20% and only clearing
  // above 25% costs one bool.
  //
  // This node REPORTS ONLY. Crossing the threshold does not lie the robot down
  // or block cmd_vel — syncai_driver_manager's triggerSafeShutdown() still has
  // zero call sites, and who owns that actuation is deliberately still open.
  double percentage = 0.0;
  bool have_sample = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_battery_) {
      percentage = latest_battery_->percentage * 100.0;
      have_sample = true;
    }
  }

  // Two ways to read a low battery that isn't one, both of which would latch
  // WARNING on a healthy robot:
  //
  // 1. No sample at all. buildState() reports battery_percentage as 0.0 when
  //    latest_battery_ is null, so a robot whose driver_manager simply has not
  //    started would look completely flat.
  // 2. percentage == 0. The driver parses the BMS section with a bare
  //    strtod lambda instead of the validating parseFloatToken it uses for every
  //    other section, so a non-numeric or empty token silently publishes 0.0. A
  //    robot at a genuine 0% is not powered on to be asked about.
  //
  // Neither case is evidence either way, so the latch holds its current value.
  if (!have_sample || percentage <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No usable battery sample (have_sample=%d, percentage=%f); holding low_battery latch at %d",
      static_cast<int>(have_sample), percentage, static_cast<int>(low_battery_latched_));
    return;
  }

  if (!low_battery_latched_ && percentage < low_battery_warn_percentage_) {
    low_battery_latched_ = true;
    RCLCPP_WARN(
      this->get_logger(), "[RobotStateNode][%s] battery %.1f%% below %.1f%%; state -> WARNING",
      __func__, percentage, low_battery_warn_percentage_);
  } else if (low_battery_latched_ && percentage > low_battery_clear_percentage_) {
    low_battery_latched_ = false;
    RCLCPP_INFO(
      this->get_logger(), "[RobotStateNode][%s] battery recovered to %.1f%% (above %.1f%%)",
      __func__, percentage, low_battery_clear_percentage_);
  }
  // Between the two thresholds the latch is intentionally left alone — that gap
  // is the hysteresis band.
}

syncai_common::msg::RobotState RobotStateNode::buildState()
{
  syncai_common::msg::RobotState msg;
  // Whole SECONDS, because this field is passed verbatim to
  // GET /api/v1/robot/state. At 10 Hz that means ten consecutive messages carry
  // the same value — it is a wall clock for the UI, not a sequence number, and
  // cannot be used to order or rate-check samples. Use motor_timestamp
  // (nanoseconds) if you need sub-second resolution.
  msg.timestamp = static_cast<uint64_t>(this->now().seconds());
  msg.robot_id = robot_id_;
  msg.map = map_name_;
  // {TODO} mode is currently hardcoded to AUTO
  msg.mode = syncai_common::msg::RobotMode::AUTO;  // default for now

  // position from TF (global_frame -> base_frame)
  //
  // A failed lookup used to abort the whole tick, which meant that before the
  // localizer had been relocalized nothing was published at all — no battery,
  // no wifi, no joint temperatures, precisely when an operator is trying to
  // work out why the robot will not localize. The message now goes out
  // regardless, carrying an explicit "the pose is not usable" marker instead.
  //
  // The non-blocking canTransform() gate in front of getCurrentPose() is load
  // bearing at 10 Hz. getCurrentPose() ends in
  // tf_buffer.transform(..., transform_timeout), which BLOCKS for the whole
  // timeout when the transform is absent — and it is absent for as long as the
  // localizer has not been relocalized. Ten builds a second, each stalling
  // 0.1 s, would saturate the timer's callback group indefinitely. Worse,
  // syncai_util::transformPoseInTargetFrame logs its failure with an
  // *unthrottled* RCLCPP_ERROR, so the pre-relocalize state would flood the
  // byobu multilog capture at 10 Hz.
  //
  // With the gate, the common failure costs a lock and a map lookup, and
  // transform_tolerance only ever applies to the rare race where the transform
  // disappears between the check and the call.
  //
  // The zero timeout is what makes canTransform() non-blocking, and it is not
  // optional stylistically: tf2_ros::Buffer's own overloads hide
  // tf2::BufferCore's three-argument canTransform(), so the duration has to be
  // passed explicitly anyway.
  geometry_msgs::msg::PoseStamped pose;
  if (tf_->canTransform(
        global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.0))) {
    msg.localization_valid =
      syncai_util::getCurrentPose(pose, *tf_, global_frame_, base_frame_, transform_tolerance_);
  } else {
    msg.localization_valid = false;
  }

  if (msg.localization_valid) {
    msg.localization_status.position.x = pose.pose.position.x;
    msg.localization_status.position.y = pose.pose.position.y;
    msg.localization_status.position.z = pose.pose.position.z;
    msg.localization_status.position.yaw = tf2::getYaw(pose.pose.orientation);
  } else {
    // localization_status stays zero-initialised. Deliberately NOT the last
    // known pose: a stale pose with no age attached reads as a live one,
    // whereas the map origin is an obviously suspicious value.
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF %s->%s unavailable; publishing robot_state with localization_valid=false",
      global_frame_.c_str(), base_frame_.c_str());
  }

  // State derivation, most-severe-first. UNINITIALIZED wins over WARNING because
  // "we do not know where the robot is" is the more fundamental fact — a low
  // battery is worth reporting, but not at the cost of hiding that the pose in
  // this very message is a zero placeholder. Consumers that need the precise
  // answer read localization_valid; state is the coarse rollup of it.
  //
  // {TODO} RUNNING and ERROR are not derived yet. CHARGING cannot be: the driver
  // hardcodes BatteryState.power_supply_status to UNKNOWN, and the only other
  // candidate is the sign of `current`, whose convention is undocumented in both
  // this port and the reference implementation.
  if (!msg.localization_valid) {
    msg.state = syncai_common::msg::RobotStatus::UNINITIALIZED;
  } else if (low_battery_latched_) {
    msg.state = syncai_common::msg::RobotStatus::WARNING;
  } else {
    msg.state = syncai_common::msg::RobotStatus::IDLE;
  }

  // velocity from odom (forward linear speed), battery level from
  // battery_state, per-joint detail from motor_states
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.localization_status.velocity = latest_odom_ ? latest_odom_->twist.twist.linear.x : 0.0;
    msg.battery_status.battery_percentage =
      latest_battery_ ? latest_battery_->percentage * 100.0 : 0.0;

    // Flatten the latest WifiStatus into a JSON string. Until the first
    // wifi_status message arrives the empty json dumps to the literal string
    // "null", which is what the backend parses defensively against.
    nlohmann::json wifi_json;
    if (latest_wifi_status_) {
      wifi_json = {
        {"ssid", latest_wifi_status_->ssid},
        {"bssid", latest_wifi_status_->bssid},
        {"rssi", latest_wifi_status_->rssi},
        {"ip_address", latest_wifi_status_->ip_address},
        {"mac_address", latest_wifi_status_->mac_address},
      };
    }
    msg.network_status.wifi_info = wifi_json.dump();

    // Operator-facing detail; must not be forwarded into the REST payload — see
    // RobotState.msg. Left empty (and motor_timestamp at 0) while
    // syncai_driver_manager is down, which is itself the useful signal.
    if (latest_motor_states_) {
      msg.motor_status = latest_motor_states_->states;
      msg.motor_timestamp = latest_motor_states_->timestamp;
    }
  }

  return msg;
}

void RobotStateNode::onTimer()
{
  // Latches first, then the build that reads them, so both describe the same
  // tick. This is the only place the latches advance.
  updateHealthLatches();
  robot_state_pub_->publish(buildState());
}
}  // namespace syncai_robot_state
