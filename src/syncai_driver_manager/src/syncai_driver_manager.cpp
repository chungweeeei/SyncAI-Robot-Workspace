#include "syncai_driver_manager/syncai_driver_manager.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
// Number of actuated joints reported per JOINT_* section (quadruped: 4 legs x
// 3 DOF).
constexpr std::size_t kNumDof = 12;

// Value order within each JOINT_* telemetry section, using the G23 URDF's
// actuated joint names (Ankle joints are fixed and not reported).
// TODO: confirm this matches the controller's joint ordering.
constexpr std::array<const char *, kNumDof> kJointNames = {
  "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint", "FR_HipX_joint",
  "FR_HipY_joint", "FR_Knee_joint", "HL_HipX_joint", "HL_HipY_joint",
  "HL_Knee_joint", "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"};

// Splits a line on runs of whitespace, dropping empty tokens.
std::vector<std::string> splitWhitespace(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) {
    out.push_back(cur);
  }
  return out;
}

// True if the token is a section header keyword. Used to bound how far a
// section's values extend before the next section begins.
bool isPacketKeyword(const std::string & s)
{
  static constexpr const char * keys[] = {"BMS_V2",    "IMU_RPY",   "ACC",       "OMEGA",
                                          "JOINT_POS", "JOINT_VEL", "JOINT_TAU", "JOINT_TEMP",
                                          "JOINT_ERR", "MODE_STATE"};
  return std::find(std::begin(keys), std::end(keys), s) != std::end(keys);
}
}  // namespace

namespace syncai_driver_manager
{

DriverManagerNode::DriverManagerNode() : Node("driver_manager")
{
  initParameters();
  initPubSub();

  if (!openSendSocket()) {
    throw std::runtime_error("failed to open UDP command socket");
  }
  if (!openRecvSocket()) {
    throw std::runtime_error("failed to open UDP telemetry socket");
  }

  running_.store(true);
  recv_thread_ = std::thread(&DriverManagerNode::receiveLoop, this);

  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] Driver Manager initialized successfully",
    __func__);
}

DriverManagerNode::~DriverManagerNode()
{
  running_.store(false);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  closeSockets();
}

void DriverManagerNode::initParameters()
{
  telemetry_recv_ip_ = this->declare_parameter<std::string>("telemetry_recv_ip", "0.0.0.0");
  telemetry_recv_port_ = this->declare_parameter<int>("telemetry_recv_port", 50012);
  command_target_ip_ = this->declare_parameter<std::string>("command_target_ip", "192.168.1.120");
  command_target_port_ = this->declare_parameter<int>("command_target_port", 50051);
}

void DriverManagerNode::initPubSub()
{
  imu_state_pub_ =
    this->create_publisher<syncai_common::msg::IMUState>("imu/state", rclcpp::SensorDataQoS());
  motors_state_pub_ = this->create_publisher<syncai_common::msg::MotorStates>(
    "motor_states", rclcpp::SensorDataQoS());
  battery_state_pub_ =
    this->create_publisher<sensor_msgs::msg::BatteryState>("battery/state", rclcpp::QoS(10));
  mode_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mode_state", rclcpp::QoS(10));

  cmd_vel_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions cmd_vel_sub_options;
  cmd_vel_sub_options.callback_group = cmd_vel_cb_group_;
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&DriverManagerNode::cmdVelCallback, this, std::placeholders::_1),
    cmd_vel_sub_options);

  services_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  set_policy_mode_srv_ = this->create_service<syncai_common::srv::SetPolicyMode>(
    "set_policy_mode",
    std::bind(
      &DriverManagerNode::setPolicyModeCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, services_cb_group_);
  reset_safety_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_safety",
    std::bind(
      &DriverManagerNode::resetSafetyCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, services_cb_group_);
  set_motion_key_srv_ = this->create_service<syncai_common::srv::SetMotionKey>(
    "set_motion_key",
    std::bind(
      &DriverManagerNode::setMotionKeyCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, services_cb_group_);
  set_speed_scale_srv_ = this->create_service<syncai_common::srv::SetSpeedScale>(
    "set_speed_scale",
    std::bind(
      &DriverManagerNode::setSpeedScaleCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, services_cb_group_);
}

bool DriverManagerNode::openSendSocket()
{
  // send socket file descriptor
  send_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Failed to create command socket: %s", __func__,
      std::strerror(errno));
    return false;
  }

  command_addr_ = sockaddr_in{};
  command_addr_.sin_family = AF_INET;
  command_addr_.sin_port = htons(static_cast<uint16_t>(command_target_port_));

  if (inet_aton(command_target_ip_.c_str(), &command_addr_.sin_addr) == 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Invalid command target IP '%s'", __func__,
      command_target_ip_.c_str());
    close(send_fd_);
    send_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] Command socket targeting %s:%d", __func__,
    command_target_ip_.c_str(), command_target_port_);
  return true;
}

bool DriverManagerNode::openRecvSocket()
{
  recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Failed to create telemetry socket: %s", __func__,
      std::strerror(errno));
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(telemetry_recv_port_));

  if (inet_aton(telemetry_recv_ip_.c_str(), &addr.sin_addr) == 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Invalid telemetry bind IP '%s'", __func__,
      telemetry_recv_ip_.c_str());
    close(recv_fd_);
    recv_fd_ = -1;
    return false;
  }

  if (bind(recv_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Failed to bind telemetry port %d: %s", __func__,
      telemetry_recv_port_, std::strerror(errno));
    close(recv_fd_);
    recv_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "[DriverManagerNode][%s] Listening for telemetry on %s:%d", __func__,
    telemetry_recv_ip_.c_str(), telemetry_recv_port_);
  return true;
}

void DriverManagerNode::closeSockets()
{
  if (recv_fd_ >= 0) {
    close(recv_fd_);
    recv_fd_ = -1;
  }
  if (send_fd_ >= 0) {
    close(send_fd_);
    send_fd_ = -1;
  }
}

void DriverManagerNode::receiveLoop()
{
  // The socket stays blocking, so the thread sleeps inside the kernel while
  // idle (near-zero CPU) and is woken the instant a datagram arrives (low
  // latency)

  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100 ms
  setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  char buf[4096];
  while (rclcpp::ok() && running_.load()) {
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    ssize_t n =
      recvfrom(recv_fd_, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&src), &slen);
    if (n > 0) {
      buf[n] = '\0';
      parseLine(std::string(buf, static_cast<std::size_t>(n)));
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[DriverManagerNode][%s] Telemetry recv error: %s", __func__, std::strerror(errno));
    }
  }
}

void DriverManagerNode::parseLine(const std::string & line)
{
  // The controller sends whitespace-separated ASCII sections on one line, e.g.
  // IMU_RPY r p y ACC ax ay az OMEGA wx wy wz JOINT_POS q0 ... q11 MODE_STATE pol mot
  // Each section keyword is followed by its numeric values, running until the
  // next known keyword. A single datagram may carry any subset of sections.
  auto tok = splitWhitespace(line);
  if (tok.empty()) {
    return;
  }

  // --- Battery (BMS_V2): voltage current soc ... temps ... cell[8] ---
  if (tok[0] == "BMS_V2") {
    if (static_cast<int>(tok.size()) < 9) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[DriverManagerNode][%s] Skipping BMS_V2: expected at least 8 values, got %zu", __func__,
        tok.size() - 1);
      return;
    }
    auto getf = [&](int i) {
      return (i < static_cast<int>(tok.size())) ? std::strtod(tok[i].c_str(), nullptr) : 0.0;
    };
    float soc = static_cast<float>(getf(3));
    float voltage = static_cast<float>(getf(1));
    float current = static_cast<float>(getf(2));
    float temperature = (static_cast<float>(getf(7)) + static_cast<float>(getf(8))) / 2.0f;

    sensor_msgs::msg::BatteryState battery_state;
    battery_state.header.stamp = this->now();
    battery_state.voltage = voltage;
    battery_state.current = current;
    // BMS reports soc as 0-100; BatteryState.percentage is defined on 0-1.
    battery_state.percentage = soc / 100.0f;
    battery_state.temperature = temperature;
    battery_state.charge = NAN;
    battery_state.capacity = NAN;
    battery_state.design_capacity = NAN;
    battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_state.present = true;
    battery_state_pub_->publish(battery_state);
    // TODO: trigger safety shutdown when soc < 20%.

    RCLCPP_DEBUG(
      this->get_logger(), "[DriverManagerNode][%s] BMS soc=%.1f%% V=%.2f I=%.2f T=%.1f", __func__,
      soc, voltage, current, temperature);
    return;
  }

  auto findKeyword = [&](const char * key) -> int {
    for (int i = 0; i < static_cast<int>(tok.size()); ++i) {
      if (tok[i] == key) {
        return i;
      }
    }
    return -1;
  };

  // IMU
  const int i_rpy = findKeyword("IMU_RPY");
  const int i_acc = findKeyword("ACC");
  const int i_omega = findKeyword("OMEGA");

  // MOTOR
  const int i_pos = findKeyword("JOINT_POS");
  const int i_vel = findKeyword("JOINT_VEL");
  const int i_tau = findKeyword("JOINT_TAU");
  const int i_temp = findKeyword("JOINT_TEMP");
  const int i_err = findKeyword("JOINT_ERR");

  // MODE
  const int i_mode = findKeyword("MODE_STATE");

  // How many value tokens follow a section keyword before the next keyword.
  auto sectionCount = [&](int kw) -> int {
    if (kw < 0) {
      return 0;
    }
    int count = 0;
    for (int i = kw + 1; i < static_cast<int>(tok.size()); ++i) {
      if (isPacketKeyword(tok[i])) {
        break;
      }
      ++count;
    }
    return count;
  };

  auto parseFloatToken = [&](int idx, float & out) -> bool {
    if (idx < 0 || idx >= static_cast<int>(tok.size())) {
      return false;
    }
    char * end = nullptr;
    errno = 0;
    float value = std::strtof(tok[idx].c_str(), &end);
    if (end == tok[idx].c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(value)) {
      return false;
    }
    out = value;
    return true;
  };

  auto parseIntToken = [&](int idx, int & out) -> bool {
    if (idx < 0 || idx >= static_cast<int>(tok.size())) {
      return false;
    }
    char * end = nullptr;
    errno = 0;
    long value = std::strtol(tok[idx].c_str(), &end, 10);
    if (end == tok[idx].c_str() || *end != '\0' || errno == ERANGE) {
      return false;
    }
    out = static_cast<int>(value);
    return true;
  };

  auto parseFloatValues =
    [&](const char * key, int kw, int expected, std::vector<float> & values) -> bool {
    values.clear();
    if (kw < 0) {
      return false;
    }
    if (sectionCount(kw) < expected) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[DriverManagerNode][%s] Skipping %s: expected %d values, got %d", __func__, key, expected,
        sectionCount(kw));
      return false;
    }
    values.reserve(expected);
    for (int i = 0; i < expected; ++i) {
      float value = 0.0f;
      if (!parseFloatToken(kw + 1 + i, value)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[DriverManagerNode][%s] Skipping %s: invalid token '%s'", __func__, key,
          tok[kw + 1 + i].c_str());
        values.clear();
        return false;
      }
      values.push_back(value);
    }
    return true;
  };

  auto parseIntValues =
    [&](const char * key, int kw, int expected, std::vector<int> & values) -> bool {
    values.clear();
    if (kw < 0) {
      return false;
    }
    if (sectionCount(kw) < expected) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[DriverManagerNode][%s] Skipping %s: expected %d values, got %d", __func__, key, expected,
        sectionCount(kw));
      return false;
    }
    values.reserve(expected);
    for (int i = 0; i < expected; ++i) {
      int value = 0;
      if (!parseIntToken(kw + 1 + i, value)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[DriverManagerNode][%s] Skipping %s: invalid token '%s'", __func__, key,
          tok[kw + 1 + i].c_str());
        values.clear();
        return false;
      }
      values.push_back(value);
    }
    return true;
  };

  std::vector<float> rpy, acc, omega, pos, vel, tau, temp;
  std::vector<int> err, mode;
  const bool rpy_ok = parseFloatValues("IMU_RPY", i_rpy, 3, rpy);
  const bool acc_ok = parseFloatValues("ACC", i_acc, 3, acc);
  const bool omega_ok = parseFloatValues("OMEGA", i_omega, 3, omega);
  const bool pos_ok = parseFloatValues("JOINT_POS", i_pos, static_cast<int>(kNumDof), pos);
  const bool vel_ok = parseFloatValues("JOINT_VEL", i_vel, static_cast<int>(kNumDof), vel);
  const bool tau_ok = parseFloatValues("JOINT_TAU", i_tau, static_cast<int>(kNumDof), tau);
  const bool temp_ok = parseFloatValues("JOINT_TEMP", i_temp, static_cast<int>(kNumDof), temp);
  const bool err_ok = parseIntValues("JOINT_ERR", i_err, static_cast<int>(kNumDof), err);

  // TODO: monitor JOINT_TEMP for overheat and trigger safety shutdown.

  if (rpy_ok || acc_ok || omega_ok) {
    syncai_common::msg::IMUState imu_state;
    imu_state.timestamp = static_cast<uint64_t>(this->now().nanoseconds());
    std::copy(rpy.begin(), rpy.end(), imu_state.rpy.begin());
    std::copy(omega.begin(), omega.end(), imu_state.gyroscope.begin());
    std::copy(acc.begin(), acc.end(), imu_state.accelerometer.begin());
    // Telemetry carries no orientation quaternion; derive it from RPY (ZYX
    // convention, radians assumed). Order: w, x, y, z. Falls back to identity
    // when this datagram has no IMU_RPY section (rpy is empty then).
    // TODO: confirm IMU_RPY units are radians, not degrees.
    if (rpy_ok) {
      const float cr = std::cos(rpy[0] * 0.5f), sr = std::sin(rpy[0] * 0.5f);
      const float cp = std::cos(rpy[1] * 0.5f), sp = std::sin(rpy[1] * 0.5f);
      const float cy = std::cos(rpy[2] * 0.5f), sy = std::sin(rpy[2] * 0.5f);
      imu_state.quaternion[0] = cr * cp * cy + sr * sp * sy;
      imu_state.quaternion[1] = sr * cp * cy - cr * sp * sy;
      imu_state.quaternion[2] = cr * sp * cy + sr * cp * sy;
      imu_state.quaternion[3] = cr * cp * sy - sr * sp * cy;
    } else {
      imu_state.quaternion[0] = 1.0f;
    }
    // IMU temperature is not part of the telemetry stream; left at 0.
    imu_state_pub_->publish(imu_state);
  }

  if (pos_ok || vel_ok || tau_ok || temp_ok || err_ok) {
    syncai_common::msg::MotorStates motor_states;
    motor_states.timestamp = static_cast<uint64_t>(this->now().nanoseconds());
    motor_states.states.resize(kNumDof);
    for (std::size_t i = 0; i < kNumDof; ++i) {
      auto & m = motor_states.states[i];
      m.name = kJointNames[i];
      m.q = pos_ok ? pos[i] : 0.0f;
      m.dq = vel_ok ? vel[i] : 0.0f;
      // ddq: telemetry has no acceleration section; left at 0.
      m.tau_est = tau_ok ? tau[i] : 0.0f;
      m.temperature = temp_ok ? static_cast<int8_t>(temp[i]) : 0;
      m.error = err_ok ? static_cast<uint16_t>(err[i]) : 0;
    }
    motors_state_pub_->publish(motor_states);
  }

  if (parseIntValues("MODE_STATE", i_mode, 2, mode)) {
    std_msgs::msg::Int32MultiArray mode_state;
    // data[0] = policy state, data[1] = motion state.
    mode_state.data.assign(mode.begin(), mode.end());
    mode_pub_->publish(mode_state);
  }
}

void DriverManagerNode::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Quadruped planar command: forward velocity, lateral velocity, yaw rate.
  // The gait controller tracks each direction differently, so the sign of the
  // command picks that direction's correction gain.
  const double vx =
    (msg->linear.x >= 0) ? (msg->linear.x * scale_fwd_) : (msg->linear.x * scale_back_);
  const double vy =
    (msg->linear.y >= 0) ? (msg->linear.y * scale_left_) : (msg->linear.y * scale_right_);

  // The controller's turning sign convention is opposite to REP 103 (+z =
  // counter-clockwise), so negate before picking the turn gain.
  const double wz_raw = -msg->angular.z;
  const double wz = (wz_raw >= 0) ? (wz_raw * scale_turn_l_) : (wz_raw * scale_turn_r_);

  char buf[128];
  int len = std::snprintf(buf, sizeof(buf), "AXES %.6f %.6f %.6f\n", vx, vy, wz);
  if (len > 0 && len < static_cast<int>(sizeof(buf))) {
    udpSend(buf, static_cast<std::size_t>(len));
  }
}

void DriverManagerNode::setPolicyModeCallback(
  const std::shared_ptr<syncai_common::srv::SetPolicyMode::Request> request,
  std::shared_ptr<syncai_common::srv::SetPolicyMode::Response> response)
{
  char buf[32];
  int len = std::snprintf(buf, sizeof(buf), "MODE %u\n", static_cast<unsigned>(request->mode));
  if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
    response->success = false;
    response->message = "Failed to format MODE command";
    return;
  }
  udpSend(buf, static_cast<std::size_t>(len));
  response->success = true;
  response->message = "Policy updated";
}

void DriverManagerNode::resetSafetyCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (safe_lock_.load()) {
    safe_lock_.store(false);
    RCLCPP_INFO(
      this->get_logger(), "[DriverManagerNode][%s] Safety lock released; control restored",
      __func__);
    response->success = true;
    response->message = "Safety lock released. Remote control restored.";
  } else {
    response->success = true;
    response->message = "System was not locked.";
  }
}

void DriverManagerNode::setMotionKeyCallback(
  const std::shared_ptr<syncai_common::srv::SetMotionKey::Request> request,
  std::shared_ptr<syncai_common::srv::SetMotionKey::Response> response)
{
  // While the safety lock is engaged, only the recover key ("4") may pass.
  if (safe_lock_.load() && request->key != "4") {
    response->success = false;
    response->message = "LOCKED";
    return;
  }

  // Emergency stop bypasses the MODE keymap.
  if (request->key == "5") {
    udpSend("ESTOP\n", 6);
    response->success = true;
    response->message = "Emergency stop sent";
    return;
  }

  // Map the motion key to the controller's MODE character.
  const char motion_key = (request->key == "0")   ? 'Z'
                          : (request->key == "1") ? 'C'
                          : (request->key == "2") ? 'X'
                          : (request->key == "3") ? 'R'
                          : (request->key == "4") ? 'V'
                                                  : ' ';
  if (motion_key == ' ') {
    response->success = false;
    response->message = "Unknown motion key '" + request->key + "'";
    return;
  }

  char buf[16];
  int len = std::snprintf(buf, sizeof(buf), "MODE %c\n", motion_key);
  if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
    response->success = false;
    response->message = "Failed to format MODE command";
    return;
  }
  udpSend(buf, static_cast<std::size_t>(len));
  response->success = true;
  response->message = "Motion key sent";
}

void DriverManagerNode::setSpeedScaleCallback(
  const std::shared_ptr<syncai_common::srv::SetSpeedScale::Request> request,
  std::shared_ptr<syncai_common::srv::SetSpeedScale::Response> response)
{
  scale_fwd_.store(request->fwd_scale);
  scale_back_.store(request->back_scale);
  scale_left_.store(request->left_scale);
  scale_right_.store(request->right_scale);
  scale_turn_l_.store(request->turn_l_scale);
  scale_turn_r_.store(request->turn_r_scale);

  RCLCPP_INFO(
    this->get_logger(),
    "[DriverManagerNode][%s] Speed scales updated: F:%.2f, B:%.2f, L:%.2f, R:%.2f, TL:%.2f, "
    "TR:%.2f",
    __func__, scale_fwd_.load(), scale_back_.load(), scale_left_.load(), scale_right_.load(),
    scale_turn_l_.load(), scale_turn_r_.load());
  response->success = true;
}

void DriverManagerNode::triggerSafeShutdown(const std::string & reason)
{
  // exchange() makes the check-and-set atomic, so concurrent triggers from
  // the telemetry thread and executor threads act exactly once.
  if (!safe_lock_.exchange(true)) {
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] !!! SAFETY TRIGGERED: %s !!!", __func__,
      reason.c_str());
    RCLCPP_ERROR(
      this->get_logger(), "[DriverManagerNode][%s] Executing LieDown [MODE X] and blocking control",
      __func__);
    udpSend("MODE X\n", 7);
  }
}

void DriverManagerNode::udpSend(const char * buf, std::size_t len)
{
  if (send_fd_ < 0) {
    return;
  }
  sendto(
    send_fd_, buf, len, 0, reinterpret_cast<sockaddr *>(&command_addr_), sizeof(command_addr_));
}

}  // namespace syncai_driver_manager
