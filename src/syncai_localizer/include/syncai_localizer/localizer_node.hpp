#ifndef SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_
#define SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_

// The ROS shell around the two-stage GICP relocalizer. The registration
// itself lives in localizers/ (icp_localizer.*, small_gicp's RegistrationPCL
// under a rough + refine schedule); this class owns the cloud/odom sync, the
// timer that re-registers and rebroadcasts map -> local_frame, the motion
// gate, and the three ways a guess reaches it (relocalize, initialpose, the
// INI's [initial_pose]).
//
// Ported into the workspace from SyncAI-Fast-LIO2's `localizer` package in
// 2026-09, the last node in that fork, with the ROS surface unchanged: node
// name `localizer_node`, namespace /<robot_id> (set by the launch), services
// `relocalize` / `relocalize_check`, topics `initialpose` (in) and `map_cloud`
// (out, latched), TF map -> local_frame where local_frame is adopted from the
// first odom message. The service types were already syncai_common's.
//
// THE NAMESPACE IS /<robot_id>, NOT /<robot_id>/localizer. Every doc used to
// say the latter (the fork's own README, the workspace CLAUDE.md, the srv
// comments in SyncAI-Robot-Interface, the rviz config) because the launch
// once set it, but fork commit 3f5f01b (2026-07-29) dropped the `/localizer`
// segment, and the backend has been calling the bare `relocalize` /
// `relocalize_check` and publishing the bare `initialpose` in the robot's
// namespace ever since -- its map gateway records that writing
// `localizer/relocalize` cost it a stack_not_ready refusal against a healthy
// stack. So the resolved names are /<robot_id>/relocalize,
// /<robot_id>/relocalize_check, /<robot_id>/initialpose, /<robot_id>/map_cloud
// and the node is /<robot_id>/localizer_node. The port keeps that, because
// moving the namespace is a cross-repository change the backend would have
// to follow; the docs were corrected instead.

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>

#include "syncai_common/srv/is_valid.hpp"
#include "syncai_common/srv/relocalize.hpp"
#include "syncai_localizer/localizers/commons.h"
#include "syncai_localizer/localizers/icp_localizer.h"

namespace syncai_localizer
{

struct NodeConfig
{
  // Absolute names into pointlio's namespace, set as parameters rather than
  // remapped: they are the contract with syncai_pointlio/launch/pointlio.launch.py
  // and the launch here overrides both with the robot_id prefix. The values
  // below are only the fallback for running the node bare.
  std::string cloud_topic = "/pointlio/body_cloud";
  std::string odom_topic = "/pointlio/lio_odom";
  std::string map_frame = "map";
  std::string local_frame = "lidar";
  // The [map] pcd from system.ini, passed in by the launch file as a parameter override.
  // initialpose uses it to loadMap automatically when no map is loaded yet (the localizer was
  // restarted and no relocalize has run since); an empty string means it is not set, in which
  // case initialpose can only be used after a relocalize.
  std::string map_path = "";
  // The [initial_pose] from system.ini: the robot's known start pose at boot (map frame). When
  // set, it is applied once as the ICP initial guess as soon as the first odom arrives, which is
  // equivalent to an automatic relocalize with no manual service call.
  // Only x / y / yaw: z / roll / pitch are always taken from the current estimate, for the
  // reasons given at applyPlanarGuess.
  bool set_initial_pose = false;
  double initial_pose_x = 0.0;
  double initial_pose_y = 0.0;
  double initial_pose_yaw = 0.0;
  double update_hz = 1.0;

  // Motion gate. update_hz is a *ceiling* on re-registration, not a schedule:
  // below these thresholds the odom pose has not moved enough to justify a new
  // scan match, so the previous offset is rebroadcast and GICP is skipped
  // entirely. For a parked robot the correct correction is a constant, and
  // re-solving it 5 times a second only resamples GICP's own noise floor
  // (measured on robot01 2026-09-21: 8.9 mm median / 24.9 mm max per update,
  // 193 of 241 updates > 5 mm, while Point-LIO itself moved 0.87 mm/frame).
  double min_update_trans = 0.05;
  double min_update_rot = 0.02;
  // Backstop: re-register this often even when parked, so a slow true drift is
  // still corrected. Kept short on purpose — the failure direction we want is
  // "jitter only partly suppressed", never "frozen on a stale pose".
  double max_update_interval = 2.0;
  // EMA weight applied to a backstop refresh (a re-registration that the motion
  // gate would otherwise have skipped). Motion-triggered updates always use 1.0,
  // so driving is bit-for-bit unchanged and picks up no lag. 1.0 here disables
  // blending and leaves only the gate.
  double static_blend_alpha = 0.1;
  // After a relocalize / initial-pose guess, register at full rate and full
  // alpha for this long. A relocalize returning success is a receipt, not a
  // result: the pose converges over the following rounds. Gating right after
  // the first successful align would leave a parked robot refining at one
  // alpha=0.1 step per max_update_interval, i.e. tens of seconds to converge.
  double post_reloc_settle = 3.0;
};

struct NodeState
{
  std::mutex message_mutex;
  std::mutex service_mutex;

  bool message_received = false;
  bool service_received = false;
  bool localize_success = false;
  // The start pose from config has not been applied yet. Only the timer thread touches it (set
  // during construction, then read and written only in timerCB once spin has started), so it
  // needs no lock.
  bool initial_pose_pending = false;
  rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
  builtin_interfaces::msg::Time last_message_time;
  CloudType::Ptr last_cloud = std::make_shared<CloudType>();
  M3D last_r;                           // localmap_body_r
  V3D last_t;                           // localmap_body_t
  M3D last_offset_r = M3D::Identity();  // map_localmap_r
  V3D last_offset_t = V3D::Zero();      // map_localmap_t
  M4F initial_guess = M4F::Identity();

  // Odom pose at the last *accepted* registration — the reference the motion
  // gate measures against. Only the timer thread touches these.
  M3D last_reg_r = M3D::Identity();
  V3D last_reg_t = V3D::Zero();
  bool has_reg = false;
  rclcpp::Time last_reg_time{0, 0, RCL_SYSTEM_TIME};
  // Full-rate window after a relocalize; see NodeConfig::post_reloc_settle.
  rclcpp::Time settle_until{0, 0, RCL_SYSTEM_TIME};
};

class LocalizerNode : public rclcpp::Node
{
public:
  LocalizerNode();

  // Declares every parameter with the struct default as its default. See the
  // definition for why these are ROS parameters and not a hand-parsed YAML.
  void loadParameters();

  // loadMap() of map_path during construction, so relocalize / initialpose
  // only ever have to deal with the guess. See the definition.
  void loadInitialMap();

  // 10 ms wall timer: apply a pending guess, rebroadcast the last offset, and
  // re-register at most update_hz (motion-gated).
  void timerCB();

  // ApproximateTime-synchronised body cloud + LIO odom: buffer only.
  void syncCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);

  void sendBroadCastTF(builtin_interfaces::msg::Time & time);

  // `relocalize`: (re)load a map and store the request's RAW 6-DOF pose as the
  // next guess. Runs on the services' callback group.
  void relocCB(
    const std::shared_ptr<syncai_common::srv::Relocalize::Request> request,
    std::shared_ptr<syncai_common::srv::Relocalize::Response> response);

  // `initialpose` (rviz "2D Pose Estimate", the console): x / y / yaw through
  // applyPlanarGuess. Same callback group as the services.
  void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Build the ICP initial guess from a 2D start pose, keeping the current
  // estimate's roll / pitch / z. See the definition for why a flat guess
  // never converges on this robot.
  void applyPlanarGuess(double x, double y, double yaw, const char * source);

  // `relocalize_check`: did the first registration after the last guess succeed?
  void relocCheckCB(
    const std::shared_ptr<syncai_common::srv::IsValid::Request> request,
    std::shared_ptr<syncai_common::srv::IsValid::Response> response);

  void publishMapCloud(builtin_interfaces::msg::Time & time);

private:
  NodeConfig m_config;
  NodeState m_state;

  ICPConfig m_localizer_config;
  std::shared_ptr<ICPLocalizer> m_localizer;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  // The services and initialpose live here, apart from the timer/subscriber
  // group, paired with the 2-thread MultiThreadedExecutor in main.cpp: a
  // multi-second loadMap inside relocCB must not gap the TF rebroadcast.
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::Service<syncai_common::srv::Relocalize>::SharedPtr m_reloc_srv;
  rclcpp::Service<syncai_common::srv::IsValid>::SharedPtr m_reloc_check_srv;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};

}  // namespace syncai_localizer

#endif  // SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_
