#ifndef SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_
#define SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_

#include <netinet/in.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "syncai_common/msg/imu_state.hpp"
#include "syncai_common/msg/motor_states.hpp"
#include "syncai_common/srv/set_motion_key.hpp"
#include "syncai_common/srv/set_policy_mode.hpp"
#include "syncai_common/srv/set_speed_scale.hpp"

namespace syncai_driver_manager
{
class DriverManagerNode : public rclcpp::Node
{
public:
  DriverManagerNode();
  ~DriverManagerNode();

private:
  void initParameters();
  void initPubSub();

  // --- UDP session lifecycle ---
  // Opens the outbound command socket toward the robot controller.
  bool openSendSocket();
  // Opens and binds the inbound telemetry socket. Throws on fatal failure so
  // the node does not come up half-connected.
  bool openRecvSocket();
  // Closes both sockets if open. Safe to call multiple times.
  void closeSockets();
  // Body of recv_thread_: blocks on the telemetry socket and drains datagrams
  // until running_ is cleared.
  void receiveLoop();

  // Decodes one ASCII telemetry datagram from the controller. Parsing only for
  // now; publishing and safety handling are left as TODOs.
  void parseLine(const std::string & line);

  // Sends a raw datagram to the configured command target.
  void udpSend(const char * buf, std::size_t len);

  // Engages the safety lock and commands the controller to lie down
  // (MODE X). Idempotent: only the first trigger acts until reset_safety
  // releases the lock. Safe to call from the telemetry thread.
  void triggerSafeShutdown(const std::string & reason);

  // Connection parameters (populated in initParameters()).
  std::string telemetry_recv_ip_;
  int telemetry_recv_port_{0};
  std::string command_target_ip_;
  int command_target_port_{0};

  // Per-direction command-to-actual velocity correction gains. The gait
  // controller tracks commands asymmetrically (forward vs backward, etc.), so
  // each direction gets its own empirically tuned scale. Atomic because the
  // set_speed_scale service writes them while cmd_vel reads them from a
  // different callback group.
  std::atomic<double> scale_fwd_{1.0};
  std::atomic<double> scale_back_{1.0};
  std::atomic<double> scale_left_{1.0};
  std::atomic<double> scale_right_{1.0};
  std::atomic<double> scale_turn_l_{1.0};
  std::atomic<double> scale_turn_r_{1.0};

  // Socket state.
  int send_fd_{-1};
  int recv_fd_{-1};
  sockaddr_in command_addr_{};

  // Telemetry receive thread.
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  // Safety lock: while set, motion commands are blocked until the
  // reset_safety service releases it. Atomic because it is shared between the
  // executor callbacks and the telemetry thread (future safety triggers).
  std::atomic<bool> safe_lock_{false};

  // Publishers
  std::shared_ptr<rclcpp::Publisher<syncai_common::msg::IMUState>> imu_pub_;
  std::shared_ptr<rclcpp::Publisher<syncai_common::msg::MotorStates>> motors_state_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_state_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>> mode_pub_;

  // Callback groups: cmd_vel gets its own so the high-rate command stream can
  // run concurrently with service handling under the MultiThreadedExecutor;
  // all services share one group and are serialized among themselves.
  std::shared_ptr<rclcpp::CallbackGroup> cmd_vel_cb_group_;
  std::shared_ptr<rclcpp::CallbackGroup> services_cb_group_;

  // Subscribers
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
  // Formats an ASCII velocity command from a Twist and forwards it to the
  // controller over the command socket.
  void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);

  // Services
  std::shared_ptr<rclcpp::Service<syncai_common::srv::SetPolicyMode>> set_policy_mode_srv_;
  // Forwards a policy-mode change to the controller as a MODE command.
  void setPolicyModeCallback(
    const std::shared_ptr<syncai_common::srv::SetPolicyMode::Request> request,
    std::shared_ptr<syncai_common::srv::SetPolicyMode::Response> response);

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_safety_srv_;
  // Releases the safety lock so motion commands flow to the controller again.
  void resetSafetyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<rclcpp::Service<syncai_common::srv::SetMotionKey>> set_motion_key_srv_;
  // Maps a motion key to the controller's MODE character command (or ESTOP).
  void setMotionKeyCallback(
    const std::shared_ptr<syncai_common::srv::SetMotionKey::Request> request,
    std::shared_ptr<syncai_common::srv::SetMotionKey::Response> response);

  std::shared_ptr<rclcpp::Service<syncai_common::srv::SetSpeedScale>> set_speed_scale_srv_;
  // Updates the per-direction velocity correction gains at runtime.
  void setSpeedScaleCallback(
    const std::shared_ptr<syncai_common::srv::SetSpeedScale::Request> request,
    std::shared_ptr<syncai_common::srv::SetSpeedScale::Response> response);
};
}  // namespace syncai_driver_manager

#endif  // SYNCAI_DRIVER_MANAGER__SYNCAI_DRIVER_MANAGER_HPP_
