#ifndef SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_
#define SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_

// The ROS shell around the two-stage GICP relocalizer. The registration
// itself lives in localizers/ (icp_localizer.*, small_gicp's RegistrationPCL
// under a rough + refine schedule); this class owns the cloud/odom sync, the
// timer that re-registers and rebroadcasts map -> local_frame, the motion
// gate, and the four ways a guess reaches it (relocalize, initialpose, the
// INI's [initial_pose], and auto_localize's global search -- the one that
// needs no pose from anyone; localizers/global_localizer.* does the search).
// reset_odometry (2026-10) is the fifth, and the only one that also changes
// the odometry under it: it zeroes Point-LIO and carries the map pose across.
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
// and the node is /<robot_id>/localizer_node. auto_localize (2026-10, the
// kidnap recovery) followed the same rule: /<robot_id>/auto_localize, and so
// did reset_odometry: /<robot_id>/reset_odometry. The port keeps that, because
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

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "syncai_common/srv/auto_localize.hpp"
#include "syncai_common/srv/is_valid.hpp"
#include "syncai_common/srv/relocalize.hpp"
#include "syncai_common/srv/reset_lio.hpp"
#include "syncai_common/srv/reset_odometry.hpp"
#include "syncai_localizer/localizers/commons.h"
#include "syncai_localizer/localizers/global_localizer.h"
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

  // reset_odometry. The service name is relative, so from this node's
  // /<robot_id> namespace it resolves to pointlio's /<robot_id>/pointlio/reset.
  std::string lio_reset_service = "pointlio/reset";
  // How long to wait for pointlio's reset response (it swaps the builder and
  // joins the old ikd-tree's rebuild thread -- well under a second normally).
  double lio_reset_timeout_sec = 2.0;
  // How long to wait for the first odometry of the new run: pointlio re-runs
  // its static IMU init (imu_init_num samples at 200 Hz) and publishes nothing
  // until it is done.
  double wait_lio_timeout_sec = 10.0;
  // How long the timer gets to register against the seeded pose.
  double converge_timeout_sec = 5.0;
  // A registration only becomes the seed for a later reset_odometry when the
  // odometry it was chained to moved slower than this since the previous one.
  // A diverged Point-LIO reports 10-10^4 m/s (robot01 2026-10-13); the robot
  // tops out at 0.6. Generous on purpose: this only decides what is
  // remembered, never what is broadcast.
  double max_lio_speed = 3.0;
};

// One synchronised body cloud with the odom pose it was taken at, for
// auto_localize's accumulation.
struct ScanRecord
{
  CloudType::Ptr cloud;
  M3D r;  // localmap_body_r
  V3D t;  // localmap_body_t
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
  // The last GlobalLocConfig::accum_scans synchronised clouds, newest at the
  // back (message_mutex). Each is its own allocation, so a snapshot of the
  // pointers stays valid while syncCB keeps going.
  std::deque<ScanRecord> recent_scans;
  // The PCD the localizer currently registers against, set on every
  // successful loadMap; auto_localize reads poses.txt beside it. Only written
  // during construction and on the services' callback group, and only read on
  // that group, so it needs no lock.
  std::string loaded_map_path;
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

  // The last TRUSTED registration, map_T_body: one that converged while the
  // odometry moved plausibly (NodeConfig::max_lio_speed). reset_odometry's
  // default seed. Written by the timer, read by the service (service_mutex).
  bool has_good = false;
  M3D good_map_body_r = M3D::Identity();
  V3D good_map_body_t = V3D::Zero();

  // reset_odometry's hold on the pipeline while Point-LIO restarts.
  //  - lio_resetting: the timer only rebroadcasts the TF, no registration,
  //    no write to last_offset_* (which the service writes meanwhile).
  //  - intake_paused (message_mutex): syncCB drops everything -- the old
  //    run's tail is still in flight when the reset lands.
  //  - lio_boundary (message_mutex): afterwards, syncCB drops every pair
  //    stamped at or before ResetLIO's last_odom_time, the same gate pgo_node
  //    uses. -1 until the first reset, i.e. nothing is dropped.
  //  - scan_seq (message_mutex): pairs accepted so far, so the service can
  //    wait for the first one of the new run.
  //  - reset_epoch (message_mutex): bumped when a reset starts; a timer round
  //    that snapshotted an older epoch drops its result instead of writing
  //    an old-run offset over the new run's seed.
  //  - skip_speed_check: the next registration straddles the odometry jump;
  //    measuring LIO speed across it would read the jump as divergence.
  std::atomic<bool> lio_resetting{false};
  bool intake_paused = false;
  double lio_boundary = -1.0;
  uint64_t scan_seq = 0;
  uint64_t reset_epoch = 0;
  std::atomic<bool> skip_speed_check{false};
};

// What globalSearch() hands back: the search result, the pose it found at the
// reference scan, and that pose re-applied to the newest odometry -- the
// guess to give the timer.
struct GlobalSearchOutcome
{
  GlobalLocResult found;
  M3D map_body_r = M3D::Identity();
  V3D map_body_t = V3D::Zero();
  M4F guess = M4F::Identity();
};

enum class Convergence { kConverged, kSuperseded, kTimedOut };

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

  // `auto_localize`: hint-free global search, then a wait for the timer to
  // register against the winner. Both halves have a deadline, and a failure in
  // either leaves localization as it was. Same callback group as the services.
  void autoLocCB(
    const std::shared_ptr<syncai_common::srv::AutoLocalize::Request> request,
    std::shared_ptr<syncai_common::srv::AutoLocalize::Response> response);

  // `reset_odometry`: zero Point-LIO (pointlio/reset) and carry the map pose
  // across the jump, then wait for the timer to confirm it -- optionally
  // falling back to auto_localize's search. The robot must be standing still.
  // Same callback group as the services; ResetOdometry.srv is the contract.
  void resetOdomCB(
    const std::shared_ptr<syncai_common::srv::ResetOdometry::Request> request,
    std::shared_ptr<syncai_common::srv::ResetOdometry::Response> response);

  // `relocalize_check`: did the first registration after the last guess succeed?
  void relocCheckCB(
    const std::shared_ptr<syncai_common::srv::IsValid::Request> request,
    std::shared_ptr<syncai_common::srv::IsValid::Response> response);

  void publishMapCloud(builtin_interfaces::msg::Time & time);

private:
  // auto_localize's search half, shared with reset_odometry's fallback:
  // accumulate recent_scans, search poses.txt, re-apply the winner to the
  // newest odom. False with `error` set when it could not run or found nothing
  // acceptable; `out.found` is filled whenever the search ran, for diagnosis.
  bool globalSearch(
    GlobalLocalizer::Clock::time_point deadline, GlobalSearchOutcome & out, std::string & error);

  // Hand `guess` (map_T_body) to the timer as a service guess and wait for
  // its converged registration. On timeout the guess is withdrawn and the
  // timer goes back to its offset + odometry. timeout_sec <= 0 waits forever.
  Convergence handOverAndAwait(const M4F & guess, double timeout_sec);

  // map_T_body at the newest odometry, from the current offset.
  void currentMapPose(M3D & r, V3D & t);

  NodeConfig m_config;
  NodeState m_state;

  ICPConfig m_localizer_config;
  std::shared_ptr<ICPLocalizer> m_localizer;
  GlobalLocConfig m_global_config;
  // Only ever used from the services' group (auto_localize, reset_odometry;
  // mutually exclusive), so one search at a time and no lock of its own.
  std::shared_ptr<GlobalLocalizer> m_global_localizer;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  // The services and initialpose live here, apart from the timer/subscriber
  // group, paired with the 3-thread MultiThreadedExecutor in main.cpp: a
  // multi-second loadMap inside relocCB must not gap the TF rebroadcast.
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::Service<syncai_common::srv::Relocalize>::SharedPtr m_reloc_srv;
  rclcpp::Service<syncai_common::srv::IsValid>::SharedPtr m_reloc_check_srv;
  rclcpp::Service<syncai_common::srv::AutoLocalize>::SharedPtr m_auto_loc_srv;
  rclcpp::Service<syncai_common::srv::ResetOdometry>::SharedPtr m_reset_odom_srv;
  // reset_odometry blocks on this client's future, so its response must be
  // delivered on a different executor thread -- a group of its own, and the
  // third thread in main.cpp. Same arrangement as pgo_node's ResetLIO client.
  rclcpp::CallbackGroup::SharedPtr m_cli_cb_group;
  rclcpp::Client<syncai_common::srv::ResetLIO>::SharedPtr m_lio_reset_cli;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};

}  // namespace syncai_localizer

#endif  // SYNCAI_LOCALIZER__LOCALIZER_NODE_HPP_
