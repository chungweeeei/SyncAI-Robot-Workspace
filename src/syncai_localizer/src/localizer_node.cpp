#include "syncai_localizer/localizer_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <future>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace syncai_localizer
{

namespace
{
// The cell of PoseWithCovariance's row-major 6x6 that marks an initialpose's z as meant: the z
// variance, row 2 column 2. RViz's "2D Pose Estimate" and every sender from before leave it 0
// with position.z 0, and taking that 0 would pull each of their guesses to the ground floor.
constexpr std::size_t kInitialPoseZVarianceIndex = 2 * 6 + 2;
}  // namespace

LocalizerNode::LocalizerNode() : Node("localizer_node")
{
  RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
  loadParameters();

  // message filter subscribe body pointcloud & lio_odom topic
  rclcpp::QoS qos = rclcpp::QoS(10);
  m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
  m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

  // transform broadcaster
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  m_sync =
    std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
  m_sync->setAgePenalty(0.1);
  m_sync->registerCallback(
    std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));

  // localizer
  m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
  m_global_localizer = std::make_shared<GlobalLocalizer>(m_global_config, m_localizer_config);

  // The services live in their own callback group (paired with the MultiThreadedExecutor in
  // main), so the slow loadMap inside relocCB cannot stall the TF rebroadcast running in the
  // timer/subscriber group.
  m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Bare relative names. The node runs in the /<robot_id> namespace (see the header), so these
  // land on /<robot_id>/relocalize and /<robot_id>/relocalize_check -- which is what the backend
  // calls. Do not prefix them with "localizer/" to make them match the docs that used to say so;
  // fix the doc.
  m_reloc_srv = this->create_service<syncai_common::srv::Relocalize>(
    "relocalize",
    std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);

  m_reloc_check_srv = this->create_service<syncai_common::srv::IsValid>(
    "relocalize_check",
    std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);

  // Same group as relocalize on purpose: the two are mutually exclusive, so the
  // map cannot be swapped in the middle of a search, and the search (seconds)
  // runs on the group's own thread while the timer keeps the TF going.
  m_auto_loc_srv = this->create_service<syncai_common::srv::AutoLocalize>(
    "auto_localize",
    std::bind(&LocalizerNode::autoLocCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);

  // Same group again: a reset must not interleave with a relocalize or a
  // search, and it ends in one when asked to (global_fallback). Its ResetLIO
  // client gets a group of its own -- the handler blocks on the client's
  // future, and the response can only arrive on another thread (main.cpp runs
  // three). Never spin_until_future_complete from inside a callback.
  m_reset_odom_srv = this->create_service<syncai_common::srv::ResetOdometry>(
    "reset_odometry",
    std::bind(&LocalizerNode::resetOdomCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);
  m_cli_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_lio_reset_cli = this->create_client<syncai_common::srv::ResetLIO>(
    m_config.lio_reset_service, rmw_qos_profile_services_default, m_cli_cb_group);

  // Initial guesses from sources such as RViz's "2D Pose Estimate": they take the same
  // initial_guess path as relocalize (the next timerCB round feeds it to ICP). Normally the map
  // is already loaded during construction (loadInitialMap), so all that is left in the callback
  // is the fallback loadMap branch. That branch can take several seconds, which is why it shares
  // the services' callback group — so it cannot stall the TF rebroadcast in the timer group.
  rclcpp::SubscriptionOptions initialpose_options;
  initialpose_options.callback_group = m_srv_cb_group;
  m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10, std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1),
    initialpose_options);

  // The map is static: transient_local (latched) plus one publish after a successful loadMap, so
  // late-joining subscribers still receive it (the RViz side must set its QoS to transient_local
  // too).
  m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "map_cloud", rclcpp::QoS(1).transient_local());

  // Load the map during construction. Previously only relocalize called loadMap, so an
  // initialpose sent first after a restart hit an empty target (align() returned false
  // outright); and having every entry point do its own catch-up loadMap turned "is the map
  // loaded" into hidden state. The launch file now guarantees map_path exists (the node is not
  // started when the file is missing), so the map is loaded once here and relocalize /
  // initialpose only have to deal with the guess.
  // This must come after m_map_cloud_pub (the loaded map is published once, latched), and spin
  // has not started yet, so a synchronous load of a few seconds blocks no callback.
  loadInitialMap();

  // The start pose can only be applied after the first odom (applyPlanarGuess needs the current
  // estimate to fill in roll/pitch/z), so the constructor only raises the flag and the actual
  // application happens in timerCB.
  m_state.initial_pose_pending = m_config.set_initial_pose;
  if (m_config.set_initial_pose) {
    RCLCPP_INFO(
      this->get_logger(),
      "initial pose from config: x=%.3f y=%.3f yaw=%.3f (applied on first odom)",
      m_config.initial_pose_x, m_config.initial_pose_y, m_config.initial_pose_yaw);
  }

  m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
}

void LocalizerNode::loadInitialMap()
{
  if (m_config.map_path.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "map_path is empty: starting with no map — localization stays idle until "
      "the relocalize service loads one");
    return;
  }
  if (!std::filesystem::exists(m_config.map_path)) {
    RCLCPP_ERROR(
      this->get_logger(), "map_path '%s' does not exist: starting with no map",
      m_config.map_path.c_str());
    return;
  }
  if (!m_localizer->loadMap(m_config.map_path)) {
    RCLCPP_ERROR(
      this->get_logger(), "failed to load map from '%s': starting with no map",
      m_config.map_path.c_str());
    return;
  }
  m_state.loaded_map_path = m_config.map_path;
  RCLCPP_INFO(this->get_logger(), "map preloaded from %s", m_config.map_path.c_str());
  builtin_interfaces::msg::Time stamp = this->now();
  publishMapCloud(stamp);
}

void LocalizerNode::loadParameters()
{
  // Standard ROS 2 parameters (params/localizer_params.yaml is the /**/localizer_node params
  // file; the launch file layers the robot_id-prefixed topic / map_path overrides on top). The
  // old version parsed a flat YAML at config_path with yaml-cpp itself: `ros2 param` could only
  // see config_path, --params-file could not be used, and when config_path was not given
  // YAML::LoadFile threw, while a single missing required key raised InvalidNode — the node
  // died during construction either way. Now every parameter takes the struct's value as its
  // default, so a missing entry simply keeps that default.
  m_config.cloud_topic = declare_parameter<std::string>("cloud_topic", m_config.cloud_topic);
  m_config.odom_topic = declare_parameter<std::string>("odom_topic", m_config.odom_topic);
  m_config.map_frame = declare_parameter<std::string>("map_frame", m_config.map_frame);
  m_config.local_frame = declare_parameter<std::string>("local_frame", m_config.local_frame);
  m_config.map_path = declare_parameter<std::string>("map_path", m_config.map_path);
  m_config.update_hz = declare_parameter<double>("update_hz", m_config.update_hz);

  // Motion gate; see NodeConfig for what each one buys.
  m_config.min_update_trans =
    declare_parameter<double>("min_update_trans", m_config.min_update_trans);
  m_config.min_update_rot = declare_parameter<double>("min_update_rot", m_config.min_update_rot);
  m_config.max_update_interval =
    declare_parameter<double>("max_update_interval", m_config.max_update_interval);
  m_config.static_blend_alpha =
    declare_parameter<double>("static_blend_alpha", m_config.static_blend_alpha);
  m_config.post_reloc_settle =
    declare_parameter<double>("post_reloc_settle", m_config.post_reloc_settle);

  // reset_odometry; see NodeConfig.
  m_config.lio_reset_service =
    declare_parameter<std::string>("reset_odom.lio_reset_service", m_config.lio_reset_service);
  m_config.lio_reset_timeout_sec =
    declare_parameter<double>("reset_odom.lio_reset_timeout_sec", m_config.lio_reset_timeout_sec);
  m_config.wait_lio_timeout_sec =
    declare_parameter<double>("reset_odom.wait_lio_timeout_sec", m_config.wait_lio_timeout_sec);
  m_config.converge_timeout_sec =
    declare_parameter<double>("reset_odom.converge_timeout_sec", m_config.converge_timeout_sec);
  m_config.max_lio_speed =
    declare_parameter<double>("reset_odom.max_lio_speed", m_config.max_lio_speed);

  // The nested names (initial_pose.x) line up with the retired syncai_amcl's set_initial_pose /
  // initial_pose.*; both are overridden by the launch file from the [initial_pose] section of
  // system.ini. z is deliberately not accepted here: see applyPlanarGuess.
  m_config.set_initial_pose =
    declare_parameter<bool>("set_initial_pose", m_config.set_initial_pose);
  m_config.initial_pose_x = declare_parameter<double>("initial_pose.x", m_config.initial_pose_x);
  m_config.initial_pose_y = declare_parameter<double>("initial_pose.y", m_config.initial_pose_y);
  m_config.initial_pose_yaw =
    declare_parameter<double>("initial_pose.yaw", m_config.initial_pose_yaw);
  m_config.lidar_height = declare_parameter<double>("lidar_height", m_config.lidar_height);

  // Common parameters of the small_gicp backend, shared by both registration stages.
  m_localizer_config.num_threads =
    declare_parameter<int>("num_threads", m_localizer_config.num_threads);
  m_localizer_config.num_neighbors =
    declare_parameter<int>("num_neighbors", m_localizer_config.num_neighbors);

  m_localizer_config.rough_scan_resolution =
    declare_parameter<double>("rough_scan_resolution", m_localizer_config.rough_scan_resolution);
  m_localizer_config.rough_map_resolution =
    declare_parameter<double>("rough_map_resolution", m_localizer_config.rough_map_resolution);
  m_localizer_config.rough_max_iteration =
    declare_parameter<int>("rough_max_iteration", m_localizer_config.rough_max_iteration);
  m_localizer_config.rough_score_thresh =
    declare_parameter<double>("rough_score_thresh", m_localizer_config.rough_score_thresh);
  m_localizer_config.rough_rotation_eps =
    declare_parameter<double>("rough_rotation_eps", m_localizer_config.rough_rotation_eps);
  m_localizer_config.rough_translation_eps =
    declare_parameter<double>("rough_translation_eps", m_localizer_config.rough_translation_eps);
  m_localizer_config.rough_max_corr_dist =
    declare_parameter<double>("rough_max_corr_dist", m_localizer_config.rough_max_corr_dist);
  m_localizer_config.rough_registration_type = declare_parameter<std::string>(
    "rough_registration_type", m_localizer_config.rough_registration_type);
  m_localizer_config.rough_voxel_resolution =
    declare_parameter<double>("rough_voxel_resolution", m_localizer_config.rough_voxel_resolution);

  m_localizer_config.refine_scan_resolution =
    declare_parameter<double>("refine_scan_resolution", m_localizer_config.refine_scan_resolution);
  m_localizer_config.refine_map_resolution =
    declare_parameter<double>("refine_map_resolution", m_localizer_config.refine_map_resolution);
  m_localizer_config.refine_max_iteration =
    declare_parameter<int>("refine_max_iteration", m_localizer_config.refine_max_iteration);
  m_localizer_config.refine_score_thresh =
    declare_parameter<double>("refine_score_thresh", m_localizer_config.refine_score_thresh);
  m_localizer_config.refine_rotation_eps =
    declare_parameter<double>("refine_rotation_eps", m_localizer_config.refine_rotation_eps);
  m_localizer_config.refine_translation_eps =
    declare_parameter<double>("refine_translation_eps", m_localizer_config.refine_translation_eps);
  m_localizer_config.refine_max_corr_dist =
    declare_parameter<double>("refine_max_corr_dist", m_localizer_config.refine_max_corr_dist);
  m_localizer_config.refine_registration_type = declare_parameter<std::string>(
    "refine_registration_type", m_localizer_config.refine_registration_type);
  m_localizer_config.refine_voxel_resolution = declare_parameter<double>(
    "refine_voxel_resolution", m_localizer_config.refine_voxel_resolution);

  // auto_localize's global search; see GlobalLocConfig for what each one does.
  m_global_config.accum_scans =
    declare_parameter<int>("auto_loc.accum_scans", m_global_config.accum_scans);
  m_global_config.candidate_spacing =
    declare_parameter<double>("auto_loc.candidate_spacing", m_global_config.candidate_spacing);
  m_global_config.yaw_step_deg =
    declare_parameter<double>("auto_loc.yaw_step_deg", m_global_config.yaw_step_deg);
  m_global_config.coarse_scan_resolution = declare_parameter<double>(
    "auto_loc.coarse_scan_resolution", m_global_config.coarse_scan_resolution);
  m_global_config.coarse_max_points =
    declare_parameter<int>("auto_loc.coarse_max_points", m_global_config.coarse_max_points);
  m_global_config.coarse_inlier_dist =
    declare_parameter<double>("auto_loc.coarse_inlier_dist", m_global_config.coarse_inlier_dist);
  m_global_config.top_k = declare_parameter<int>("auto_loc.top_k", m_global_config.top_k);
  m_global_config.rough_max_corr_dist =
    declare_parameter<double>("auto_loc.rough_max_corr_dist", m_global_config.rough_max_corr_dist);
  m_global_config.max_iteration =
    declare_parameter<int>("auto_loc.max_iteration", m_global_config.max_iteration);
  m_global_config.refine_inlier_dist =
    declare_parameter<double>("auto_loc.refine_inlier_dist", m_global_config.refine_inlier_dist);
  m_global_config.min_inlier_ratio =
    declare_parameter<double>("auto_loc.min_inlier_ratio", m_global_config.min_inlier_ratio);
  m_global_config.ambiguity_ratio =
    declare_parameter<double>("auto_loc.ambiguity_ratio", m_global_config.ambiguity_ratio);
  m_global_config.ambiguity_trans =
    declare_parameter<double>("auto_loc.ambiguity_trans", m_global_config.ambiguity_trans);
  m_global_config.ambiguity_yaw_deg =
    declare_parameter<double>("auto_loc.ambiguity_yaw_deg", m_global_config.ambiguity_yaw_deg);
  m_global_config.search_timeout_sec =
    declare_parameter<double>("auto_loc.search_timeout_sec", m_global_config.search_timeout_sec);
  m_global_config.converge_timeout_sec = declare_parameter<double>(
    "auto_loc.converge_timeout_sec", m_global_config.converge_timeout_sec);
  m_global_config.accum_scans = std::max(1, m_global_config.accum_scans);

  // Log the instance values, so a bare run (subscribing to /pointlio/..., which nothing
  // publishes) shows up in the log as the wrong names rather than as silence.
  RCLCPP_INFO(
    this->get_logger(), "params: cloud_topic=%s odom_topic=%s map_frame=%s update_hz=%.2f",
    m_config.cloud_topic.c_str(), m_config.odom_topic.c_str(), m_config.map_frame.c_str(),
    m_config.update_hz);
}

void LocalizerNode::timerCB()
{
  if (!m_state.message_received) return;

  // The known start pose from config: only once the first odom has arrived is there a "current
  // estimate" to fill in roll/pitch/z, so it is applied here rather than in the constructor. It
  // gets exactly one chance — the flag is cleared first, so even if it is skipped because no
  // map is loaded, it does not linger until some later relocalize finishes loading a map and
  // then suddenly overwrite that call's guess.
  if (m_state.initial_pose_pending) {
    m_state.initial_pose_pending = false;
    if (m_localizer->isMapLoaded()) {
      applyPlanarGuess(
        m_config.initial_pose_x, m_config.initial_pose_y, m_config.initial_pose_yaw, "config");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "initial pose from config skipped: no map loaded — use the relocalize service");
    }
  }

  // reset_odometry owns last_offset_* while Point-LIO restarts: keep the TF
  // alive (under message_mutex, which the service writes the offset under)
  // and register nothing -- the odometry is either the diverged old run or
  // not there yet.
  if (m_state.lio_resetting.load()) {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    sendBroadCastTF(m_state.last_message_time);
    return;
  }

  rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

  // Check the time since the last TF publish; if it is under 1 / update_hz, republish the
  // previous TF.
  bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

  if (!update_tf) {
    sendBroadCastTF(m_state.last_message_time);
    return;
  }

  m_state.last_send_tf_time = rclcpp::Clock().now();

  // Run rough + refine ICP and update the offset on success.
  M4F initial_guess = M4F::Identity();
  bool service_pending = false;
  {
    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    service_pending = m_state.service_received;
    if (service_pending) initial_guess = m_state.initial_guess;
  }

  // Snapshot the odom pose before deciding whether to register: the motion
  // gate measures against it. setInput() stays on the align path — copying
  // the cloud into the localizer is the expensive part and a gated tick must
  // not pay it.
  M3D current_local_r;
  V3D current_local_t;
  builtin_interfaces::msg::Time current_time;
  uint64_t reset_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    reset_epoch = m_state.reset_epoch;
    current_local_r = m_state.last_r;
    current_local_t = m_state.last_t;
    current_time = m_state.last_message_time;
    if (!service_pending) {
      initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
      initial_guess.block<3, 1>(0, 3) =
        (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
    }
  }

  // A relocalize (or the config initial pose, which reaches us the same way)
  // must bypass the gate *and* the blend: it is the first step of a live map
  // switch, and easing 10% toward the requested pose would be worse than
  // useless. It also opens a full-rate settle window, because the pose only
  // converges over the rounds that follow.
  const rclcpp::Time now = rclcpp::Clock().now();
  if (service_pending) {
    m_state.settle_until = now + rclcpp::Duration::from_seconds(m_config.post_reloc_settle);
  }
  const bool settling = now < m_state.settle_until;

  double alpha = 1.0;
  if (!service_pending && !settling) {
    const double moved_trans = (current_local_t - m_state.last_reg_t).norm();
    // Rotation angle of last_reg_r -> current_local_r, via the trace.
    const M3D dr = current_local_r * m_state.last_reg_r.transpose();
    const double moved_rot = std::acos(std::clamp((dr.trace() - 1.0) * 0.5, -1.0, 1.0));
    const bool moved =
      moved_trans > m_config.min_update_trans || moved_rot > m_config.min_update_rot;
    const bool stale = (now - m_state.last_reg_time).seconds() > m_config.max_update_interval;

    if (m_state.has_reg && !moved && !stale) {
      // Parked and recently registered: the correct offset is whatever we
      // already hold. Rebroadcast it and skip GICP entirely.
      sendBroadCastTF(current_time);
      return;
    }
    if (!moved) alpha = m_config.static_blend_alpha;
  }

  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    m_localizer->setInput(m_state.last_cloud);
  }

  // With no map loaded the target cloud is empty and align() returns false outright, before
  // touching the guess. Normally loadInitialMap() has loaded it in the constructor; this only
  // happens when map_path was unset or unreadable and no relocalize / initialpose has loaded one
  // since.
  bool result = m_localizer->align(initial_guess);
  if (result) {
    M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
    V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
    // Re-orthonormalize: the ICP solution and the odom rotation are chained float products (and
    // are fed back into initial_guess on the next round), so accumulated error drifts R off
    // SO(3), and the norm of the quaternion it is converted to at broadcast time drifts with it
    // (tf2 denormalized-quaternion warning, and the skew/scale gets read as a spurious
    // displacement). Pull it back to a valid rotation. The blending path adds one more product
    // to the chain, which makes this step more important there; slerp itself returns a unit
    // quaternion.
    const M3D raw_offset_r = map_body_r * current_local_r.transpose();
    Eigen::Quaterniond cand_q(raw_offset_r);
    cand_q.normalize();
    M3D cand_r = cand_q.toRotationMatrix();
    // Use the same orthonormalized rotation for the translation so rotation and translation
    // stay consistent.
    V3D cand_t = -cand_r * current_local_t + map_body_t;

    // Build the candidate in locals and blend afterwards: writing
    // last_offset_r first and then deriving last_offset_t from it (as this
    // used to) would mix a blended rotation into an unblended translation.
    // Eigen's slerp picks the short arc itself, so no sign fix-up is needed.
    if (alpha < 1.0 && m_state.has_reg) {
      cand_q = Eigen::Quaterniond(m_state.last_offset_r).slerp(alpha, cand_q);
      cand_r = cand_q.toRotationMatrix();
      cand_t = (1.0 - alpha) * m_state.last_offset_t + alpha * cand_t;
    }
    {
      // Locked since reset_odometry: the services' thread now reads the
      // offset (currentMapPose) and writes it (the seed), so a torn Eigen
      // read is no longer only a cosmetic risk.
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      // A reset_odometry began while this round was registering: its pose is
      // in the old odometry, and the service may already have written the
      // new run's seed. Drop the round rather than overwrite it.
      if (m_state.reset_epoch != reset_epoch) return;
      m_state.last_offset_r = cand_r;
      m_state.last_offset_t = cand_t;
    }

    // Is this registration worth remembering as reset_odometry's seed? Only
    // when the odometry it is chained to moved plausibly since the last one:
    // a diverging Point-LIO can still drag a round or two of ICP to
    // "converged" (robot01 2026-10-13: fitness 0.004 while the guess jumped
    // 0.3-0.6 m every 0.2 s). A service guess is a discontinuity by design and
    // is trusted on convergence alone, and so is the first round after a
    // reset_odometry, whose odometry jumped back to the origin.
    bool trusted = true;
    const bool skip_speed = m_state.skip_speed_check.exchange(false);
    if (m_state.has_reg && !service_pending && !skip_speed) {
      const double dt = std::max((now - m_state.last_reg_time).seconds(), 0.05);
      const double speed = (current_local_t - m_state.last_reg_t).norm() / dt;
      if (speed > m_config.max_lio_speed) {
        trusted = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "LIO moved %.2f m in %.2f s (%.1f m/s > reset_odom.max_lio_speed %.1f): Point-LIO may "
          "be diverging; this registration will not seed reset_odometry",
          (current_local_t - m_state.last_reg_t).norm(), dt, speed, m_config.max_lio_speed);
      }
    }

    m_state.last_reg_r = current_local_r;
    m_state.last_reg_t = current_local_t;
    m_state.last_reg_time = now;
    m_state.has_reg = true;

    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    if (trusted) {
      m_state.has_good = true;
      m_state.good_map_body_r = cand_r * current_local_r;
      m_state.good_map_body_t = cand_r * current_local_t + cand_t;
    }
    if (!m_state.localize_success && m_state.service_received) {
      m_state.localize_success = true;
      m_state.service_received = false;
    }
  }

  sendBroadCastTF(current_time);
}

void LocalizerNode::syncCB(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  // A fresh cloud per message rather than refilling last_cloud in place:
  // recent_scans keeps the previous ones by pointer for auto_localize, and a
  // snapshot of those pointers must not change under the search. Converted
  // before taking the lock -- it is the expensive part.
  CloudType::Ptr cloud = std::make_shared<CloudType>();
  pcl::fromROSMsg(*cloud_msg, *cloud);

  std::lock_guard<std::mutex> lock(m_state.message_mutex);

  // reset_odometry's gate: nothing while Point-LIO restarts, then only the
  // new run -- strictly after ResetLIO's last_odom_time. Both stamps, because
  // ApproximateTime could pair an old-run cloud with a new-run odom. Same
  // formula as pointlio's Utils::getSec, so the comparison is exact.
  if (m_state.intake_paused) return;
  auto sec = [](const builtin_interfaces::msg::Time & t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };
  if (
    sec(odom_msg->header.stamp) <= m_state.lio_boundary ||
    sec(cloud_msg->header.stamp) <= m_state.lio_boundary) {
    return;
  }
  ++m_state.scan_seq;

  m_state.last_cloud = cloud;

  m_state.last_r = Eigen::Quaterniond(
                     odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                     .toRotationMatrix();
  m_state.last_t = V3D(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  m_state.last_message_time = cloud_msg->header.stamp;
  m_state.recent_scans.push_back({cloud, m_state.last_r, m_state.last_t});
  while (m_state.recent_scans.size() > static_cast<size_t>(m_global_config.accum_scans)) {
    m_state.recent_scans.pop_front();
  }
  if (!m_state.message_received) {
    m_state.message_received = true;
    // Adopt the LIO's odom frame from the message rather than trusting the local_frame
    // parameter: the correction has to land on whatever frame pointlio actually broadcasts
    // (<robot_id>/pointlio_odom), and the YAML value is only a placeholder. pgo_node in
    // syncai_mapping does NOT do this and needs its local_frame override to match instead.
    m_config.local_frame = odom_msg->header.frame_id;
  }
}

void LocalizerNode::sendBroadCastTF(builtin_interfaces::msg::Time & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_config.map_frame;
  transformStamped.child_frame_id = m_config.local_frame;
  transformStamped.header.stamp = time;
  Eigen::Quaterniond q(m_state.last_offset_r);
  V3D t = m_state.last_offset_t;
  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  m_tf_broadcaster->sendTransform(transformStamped);
}

void LocalizerNode::relocCB(
  const std::shared_ptr<syncai_common::srv::Relocalize::Request> request,
  std::shared_ptr<syncai_common::srv::Relocalize::Response> response)
{
  std::string pcd_path = request->pcd_path;
  float x = request->x;
  float y = request->y;
  float z = request->z;
  float yaw = request->yaw;
  float roll = request->roll;
  float pitch = request->pitch;

  if (!std::filesystem::exists(pcd_path)) {
    response->success = false;
    response->message = "pcd file not found";
    return;
  }

  Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  bool load_flag = m_localizer->loadMap(pcd_path);
  if (!load_flag) {
    response->success = false;
    response->message = "load map failed";
    return;
  }
  m_state.loaded_map_path = pcd_path;
  // The request's RAW 6-DOF pose, deliberately NOT through applyPlanarGuess: a caller that
  // knows the full pose (a bag, a test) must be able to hand it over. The price is that a flat
  // guess from a 2D source never converges on this robot's tilted mount, which is why the
  // backend's map switch follows this call with an initialpose publish. Keep both paths.
  {
    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    m_state.initial_guess.setIdentity();
    m_state.initial_guess.block<3, 3>(0, 0) =
      (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
    m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
    m_state.service_received = true;
    m_state.localize_success = false;
  }

  // This is the only place the map changes at runtime; publish it once, latched, for RViz and
  // any other subscriber.
  builtin_interfaces::msg::Time stamp = this->now();
  publishMapCloud(stamp);

  // A receipt, not a result: registration against the new guess runs on the timer, and
  // relocalize_check is what reports whether it converged.
  response->success = true;
  response->message = "relocalize success";
  return;
}

void LocalizerNode::initialPoseCB(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // On the normal path the map was already loaded during construction (loadInitialMap); this
  // block is only reached when map_path was not set or that load failed. It is kept because,
  // with no map loaded, align() returns false outright on the empty target (the guess is not
  // even logged), so initialpose looks like it did nothing at all — which is exactly how it bit
  // us on the robot. Here we either load the map successfully or warn explicitly and ignore the
  // message, never leaving behind a stale guess that only takes effect once a map is loaded.
  if (!m_localizer->isMapLoaded()) {
    if (m_config.map_path.empty() || !std::filesystem::exists(m_config.map_path)) {
      RCLCPP_WARN(
        this->get_logger(),
        "initialpose ignored: map not loaded and map_path '%s' not usable — "
        "call the relocalize service first",
        m_config.map_path.c_str());
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "initialpose with no map loaded: loading map from %s",
      m_config.map_path.c_str());
    if (!m_localizer->loadMap(m_config.map_path)) {
      RCLCPP_WARN(
        this->get_logger(), "initialpose ignored: failed to load map from %s",
        m_config.map_path.c_str());
      return;
    }
    m_state.loaded_map_path = m_config.map_path;
    builtin_interfaces::msg::Time stamp = this->now();
    publishMapCloud(stamp);
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != m_config.map_frame) {
    RCLCPP_WARN(
      this->get_logger(), "initialpose frame_id '%s' != map_frame '%s', treating it as %s",
      msg->header.frame_id.c_str(), m_config.map_frame.c_str(), m_config.map_frame.c_str());
  }

  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  q.normalize();
  M3D msg_r = q.toRotationMatrix();

  // position.z is the ground under the pose, and taken only when the sender marks it with a
  // positive z variance (the backend's set_initial_pose with a z does). Unmarked, it is RViz's
  // or an older sender's 0, which says nothing about the storey.
  std::optional<double> ground_z;
  const double z_variance = msg->pose.covariance[kInitialPoseZVarianceIndex];
  if (z_variance > 0.0 && std::isfinite(msg->pose.pose.position.z)) {
    ground_z = msg->pose.pose.position.z;
  }
  applyPlanarGuess(
    msg->pose.pose.position.x, msg->pose.pose.position.y, std::atan2(msg_r(1, 0), msg_r(0, 0)),
    "initialpose", ground_z);
}

// Build the ICP initial guess from a 2D start pose (x, y, yaw). Shared by two sources: the
// initialpose from RViz / the console's gridmap, and the boot start pose from system.ini
// [initial_pose].
//
// A 2D source can only yield x, y, yaw (z=0, roll=pitch=0). But with a tilted lidar mount and a
// gravity-aligned map, map_T_body always carries the ~15° mount pitch: using the 2D pose
// directly as the guess tilts the whole scan relative to the map, the error of far-away points
// exceeds rough_max_corr_dist, and rough ICP never passes the score threshold — and while the
// guess is pending, timerCB also skips the odom feedback path, so localization freezes on the
// old offset and retries forever (measured on the robot: an initialpose at (5.0, 0.982) hung;
// with the 14.6° pitch filled in, the same spot converged on the first round).
// So only x, y, yaw are taken; roll/pitch are always filled in from the current estimate.
//
// z is too, unless the sender names the storey with `ground_z` (an initialpose with its z
// variance set): the current estimate's z is the storey the localizer thinks it is on, and a
// guess on another one starts a whole storey out, beyond rough_max_corr_dist, and hangs the
// same way. `ground_z` is the ground's height, so the body sits lidar_height above it. It only
// has to land the rough stage inside its reach, so a storey's ground half a metre out (the
// console's "0 is the ground floor") still converges.
void LocalizerNode::applyPlanarGuess(
  double x, double y, double yaw, const char * source, std::optional<double> ground_z)
{
  M3D guess_r = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
  V3D guess_t(x, y, 0.0);
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    if (m_state.message_received) {
      M3D cur_r = m_state.last_offset_r * m_state.last_r;
      V3D cur_t = m_state.last_offset_r * m_state.last_t + m_state.last_offset_t;
      double cur_yaw = std::atan2(cur_r(1, 0), cur_r(0, 0));
      // Keep the gravity tilt of the current pose and only turn the heading to the requested yaw.
      guess_r = Eigen::AngleAxisd(yaw - cur_yaw, V3D::UnitZ()).toRotationMatrix() * cur_r;
      guess_t.z() = cur_t.z();
    }
    // Early at boot, before any odom has arrived, there is no estimate to fill in from, so the
    // pure 2D pose is used (the config start pose waits for the first odom and never gets here).
  }
  if (ground_z) {
    guess_t.z() = *ground_z + m_config.lidar_height;
  }

  std::lock_guard<std::mutex> lock(m_state.service_mutex);
  m_state.initial_guess.setIdentity();
  m_state.initial_guess.block<3, 3>(0, 0) = guess_r.cast<float>();
  m_state.initial_guess.block<3, 1>(0, 3) = guess_t.cast<float>();
  m_state.service_received = true;
  m_state.localize_success = false;
  const char * filled_in = "roll/pitch/z from current estimate";
  if (ground_z) {
    filled_in = "z from the given ground, roll/pitch from current estimate";
  }
  RCLCPP_INFO(
    this->get_logger(), "guess from %s applied: x=%.3f y=%.3f z=%.3f yaw=%.3f (%s)", source,
    guess_t.x(), guess_t.y(), guess_t.z(), yaw, filled_in);
}

bool LocalizerNode::globalSearch(
  GlobalLocalizer::Clock::time_point deadline, GlobalSearchOutcome & out, std::string & error)
{
  if (!m_localizer->isMapLoaded() || m_state.loaded_map_path.empty()) {
    error = "no map loaded";
    return false;
  }
  std::vector<ScanRecord> scans;
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    scans.assign(m_state.recent_scans.begin(), m_state.recent_scans.end());
  }
  if (scans.empty()) {
    error = "no scan received yet on " + m_config.cloud_topic;
    return false;
  }

  // poses.txt sits beside map.pcd (syncai_mapping's save_maps with
  // save_patches writes both into map/<name>/).
  const std::string poses_path =
    (std::filesystem::path(m_state.loaded_map_path).parent_path() / "poses.txt").string();
  std::vector<V3D> positions;
  if (!m_global_localizer->loadCandidatePositions(poses_path, positions, error)) {
    return false;
  }

  // Accumulate the recent scans into the newest one's gravity-level, yaw-zero
  // frame ("level"): odom (pointlio_odom) is gravity-aligned and so is the
  // map, so only x / y / z / yaw are left for the search -- the tilt of the
  // mount is carried in level_body_r and never searched. The odom poses
  // compensate any motion between the scans.
  const ScanRecord & ref = scans.back();
  const double ref_yaw = std::atan2(ref.r(1, 0), ref.r(0, 0));
  const M3D level_local_r = Eigen::AngleAxisd(-ref_yaw, V3D::UnitZ()).toRotationMatrix();
  const M3D level_body_r = level_local_r * ref.r;
  CloudType::Ptr accumulated = std::make_shared<CloudType>();
  for (const ScanRecord & rec : scans) {
    const M3D R = level_local_r * rec.r;
    const V3D t = level_local_r * (rec.t - ref.t);
    for (const PointType & p : *rec.cloud) {
      const V3D q = R * V3D(p.x, p.y, p.z) + t;
      PointType o = p;
      o.x = static_cast<float>(q.x());
      o.y = static_cast<float>(q.y());
      o.z = static_cast<float>(q.z());
      accumulated->push_back(o);
    }
  }
  RCLCPP_INFO(
    this->get_logger(), "global search: %zu scans, %zu points, %zu candidate positions",
    scans.size(), accumulated->size(), positions.size());

  out.found = m_global_localizer->search(
    accumulated, positions, m_localizer->roughMap(), m_localizer->refineMap(), deadline);

  // map_T_body at the reference scan = map_T_level * level_T_body.
  const M3D map_level_r = out.found.pose.block<3, 3>(0, 0).cast<double>();
  out.map_body_r = map_level_r * level_body_r;
  out.map_body_t = out.found.pose.block<3, 1>(0, 3).cast<double>();
  if (!out.found.success) {
    error = out.found.message;
    return false;
  }

  // Hand over as map_T_localmap, then re-apply it to the newest odom pose: the
  // search took seconds and the robot may have moved since the reference
  // scan.
  const M3D offset_r = out.map_body_r * ref.r.transpose();
  const V3D offset_t = out.map_body_t - offset_r * ref.t;
  std::lock_guard<std::mutex> lock(m_state.message_mutex);
  out.guess.setIdentity();
  out.guess.block<3, 3>(0, 0) = (offset_r * m_state.last_r).cast<float>();
  out.guess.block<3, 1>(0, 3) = (offset_r * m_state.last_t + offset_t).cast<float>();
  return true;
}

Convergence LocalizerNode::handOverAndAwait(const M4F & guess, double timeout_sec)
{
  // From here on it is relocalize's path exactly -- the timer takes
  // initial_guess, bypasses the motion gate and opens the settle window.
  {
    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    m_state.initial_guess = guess;
    m_state.service_received = true;
    m_state.localize_success = false;
  }

  // Wait for the timer's own (converged-gated) registration against a fresh
  // scan. This is the timeout that matters most: a guess the timer cannot
  // converge on is retried by it every round, forever -- relocalize and
  // initialpose have exactly that failure mode, and nothing ever withdraws
  // their guess. Here a non-converging candidate is withdrawn instead, the
  // timer falls back to its previous offset + odometry (last_offset_* is only
  // written by a successful align -- or by reset_odometry's seed, before this
  // is called), and the caller gets a definite failure.
  using Clock = GlobalLocalizer::Clock;
  const bool bounded = timeout_sec > 0;
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                         std::chrono::duration<double>(timeout_sec));
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      if (m_state.localize_success) return Convergence::kConverged;
      // Nothing on this callback group can replace the guess meanwhile, but
      // the timer applies the INI's [initial_pose] once on its first odom --
      // if that lands now, it is the newer guess and is left alone.
      if (m_state.initial_guess != guess) return Convergence::kSuperseded;
    }
    if (bounded && Clock::now() > deadline) break;
    std::this_thread::sleep_for(50ms);
  }

  std::lock_guard<std::mutex> lock(m_state.service_mutex);
  m_state.service_received = false;
  return Convergence::kTimedOut;
}

void LocalizerNode::currentMapPose(M3D & r, V3D & t)
{
  std::lock_guard<std::mutex> lock(m_state.message_mutex);
  r = m_state.last_offset_r * m_state.last_r;
  t = m_state.last_offset_r * m_state.last_t + m_state.last_offset_t;
}

void LocalizerNode::autoLocCB(
  const std::shared_ptr<syncai_common::srv::AutoLocalize::Request> /*request*/,
  std::shared_ptr<syncai_common::srv::AutoLocalize::Response> response)
{
  using Clock = GlobalLocalizer::Clock;
  const auto t_start = Clock::now();
  auto seconds = [](double s) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
  };
  auto fail = [&](const std::string & message) {
    response->success = false;
    response->message = message;
    RCLCPP_WARN(this->get_logger(), "auto_localize failed: %s", message.c_str());
  };

  GlobalSearchOutcome out;
  std::string error;
  const bool found_ok =
    globalSearch(t_start + seconds(m_global_config.search_timeout_sec), out, error);
  response->candidates = out.found.candidates;
  if (out.found.has_pose) {
    response->x = static_cast<float>(out.map_body_t.x());
    response->y = static_cast<float>(out.map_body_t.y());
    response->z = static_cast<float>(out.map_body_t.z());
    response->yaw = static_cast<float>(std::atan2(out.map_body_r(1, 0), out.map_body_r(0, 0)));
    response->fitness = static_cast<float>(out.found.fitness);
    response->inlier_ratio = static_cast<float>(out.found.inlier_ratio);
  }
  if (!found_ok) {
    fail(error);
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "auto_localize: search done in %.2f s, handing to the timer: %s",
    std::chrono::duration<double>(Clock::now() - t_start).count(), out.found.message.c_str());

  const Convergence result = handOverAndAwait(out.guess, m_global_config.converge_timeout_sec);
  if (result != Convergence::kConverged) {
    char buf[160];
    std::snprintf(
      buf, sizeof(buf), "found (%.2f, %.2f, %.1f deg) but ", response->x, response->y,
      response->yaw * 180.0 / M_PI);
    fail(
      std::string(buf) +
      (result == Convergence::kSuperseded
         ? "another guess replaced it before the timer converged"
         : "the timer did not converge on it within " +
             std::to_string(m_global_config.converge_timeout_sec) +
             " s (auto_loc.converge_timeout_sec); withdrawn, localization unchanged"));
    return;
  }

  response->success = true;
  response->message = out.found.message;
  RCLCPP_INFO(
    this->get_logger(), "auto_localize succeeded in %.2f s: %s",
    std::chrono::duration<double>(Clock::now() - t_start).count(), out.found.message.c_str());
}

// Zero the odometry and keep the map pose -- what resetting a wheel encoder is
// on a wheeled robot. ResetOdometry.srv is the contract; the why is there.
//
// The order is the whole design, as in pgo_node's reset_mapping:
//   1. pick the seed first, while the last trusted registration is still the
//      pre-reset one, and fail before touching anything if there is none;
//   2. pause intake and the timer's registration, THEN reset Point-LIO, so no
//      pair of the old run lands after the boundary and nothing registers a
//      new-run scan against the old offset;
//   3. gate on ResetLIO's last_odom_time and wait for the first new-run pair;
//   4. write the seeded offset directly, so map -> body is continuous the
//      moment the new odometry appears, and only then let the timer refine it.
// Everything after step 2's ResetLIO is not undone on a later failure: the
// old odometry was the problem, there is nothing to go back to.
void LocalizerNode::resetOdomCB(
  const std::shared_ptr<syncai_common::srv::ResetOdometry::Request> request,
  std::shared_ptr<syncai_common::srv::ResetOdometry::Response> response)
{
  using Clock = GlobalLocalizer::Clock;
  const auto t_start = Clock::now();
  auto seconds = [](double s) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
  };
  auto fail = [&](const std::string & message) {
    response->success = false;
    response->message = message;
    RCLCPP_WARN(this->get_logger(), "reset_odometry failed: %s", message.c_str());
  };
  auto report = [&](const M3D & r, const V3D & t) {
    response->x = static_cast<float>(t.x());
    response->y = static_cast<float>(t.y());
    response->yaw = static_cast<float>(std::atan2(r(1, 0), r(0, 0)));
  };

  if (!m_localizer->isMapLoaded()) {
    fail("no map loaded; nothing was reset");
    return;
  }

  // ---- 1. The seed: x / y / yaw (map) and z. Roll / pitch come later, from
  // the new run's own gravity alignment. ----
  //
  // skip_seed (the kidnap case) tries no seed at all: the last trusted
  // registration is where the robot was before it was carried, and right
  // after boot there is none. The map pose the TF shows right now is still
  // written into the new run's offset in step 4, but only so map -> body
  // stays continuous while the search runs -- it is never handed to the timer.
  const bool skip_seed = request->skip_seed;
  double seed_x = 0.0, seed_y = 0.0, seed_yaw = 0.0, seed_z = 0.0;
  if (skip_seed) {
    M3D r;
    V3D t;
    currentMapPose(r, t);
    seed_x = t.x();
    seed_y = t.y();
    seed_z = t.z();
    seed_yaw = std::atan2(r(1, 0), r(0, 0));
  } else {
    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    if (m_state.has_good) seed_z = m_state.good_map_body_t.z();
    if (request->use_hint) {
      seed_x = request->x;
      seed_y = request->y;
      seed_yaw = request->yaw;
      response->method = "hint";
    } else if (m_state.has_good) {
      seed_x = m_state.good_map_body_t.x();
      seed_y = m_state.good_map_body_t.y();
      seed_yaw = std::atan2(m_state.good_map_body_r(1, 0), m_state.good_map_body_r(0, 0));
      response->method = "last_good";
    } else {
      fail(
        "no trusted registration to seed from since this node started; nothing was reset -- "
        "call again with use_hint and the robot's pose on the map, or with skip_seed");
      return;
    }
  }
  if (skip_seed) {
    RCLCPP_WARN(
      this->get_logger(),
      "reset_odometry: no seed (skip_seed), global search after the reset; resetting Point-LIO "
      "(the robot must be standing still)");
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "reset_odometry: seed from %s x=%.3f y=%.3f yaw=%.3f z=%.3f; resetting Point-LIO (the "
      "robot must be standing still)",
      response->method.c_str(), seed_x, seed_y, seed_yaw, seed_z);
  }

  // ---- 2. Pause, then reset the front end. ----
  //
  // The guard releases the timer on every exit. Intake is reopened by step 3
  // on success; on a failure before that, it is reopened here with the
  // boundary unchanged -- pointlio either did not reset, or did and will only
  // publish new-run pairs, which pass.
  struct Resume
  {
    NodeState & state;
    ~Resume()
    {
      {
        std::lock_guard<std::mutex> lock(state.message_mutex);
        state.intake_paused = false;
      }
      state.lio_resetting.store(false);
    }
  } resume{m_state};
  m_state.lio_resetting.store(true);
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    m_state.intake_paused = true;
    ++m_state.reset_epoch;
  }

  if (!m_lio_reset_cli->wait_for_service(std::chrono::seconds(2))) {
    response->method.clear();
    fail(
      "LIO reset service " + std::string(m_lio_reset_cli->get_service_name()) +
      " is not available; nothing was reset");
    return;
  }
  auto future =
    m_lio_reset_cli->async_send_request(std::make_shared<syncai_common::srv::ResetLIO::Request>());
  if (future.wait_for(seconds(m_config.lio_reset_timeout_sec)) != std::future_status::ready) {
    response->method.clear();
    fail(
      "timed out after " + std::to_string(m_config.lio_reset_timeout_sec) +
      " s (reset_odom.lio_reset_timeout_sec) waiting for the LIO reset; it may still land -- "
      "check the pointlio log before calling again");
    return;
  }
  auto lio_response = future.get();
  if (!lio_response->success) {
    response->method.clear();
    fail("LIO refused the reset (" + lio_response->message + "); nothing was reset");
    return;
  }

  // ---- 3. Gate on the boundary, reopen intake, wait for the new run. ----
  uint64_t seq_at_reset = 0;
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    m_state.lio_boundary = lio_response->last_odom_time;
    // Posed by the old (diverged) odometry: a global search over them would
    // accumulate garbage, which is why auto_localize alone cannot recover
    // from a diverged Point-LIO.
    m_state.recent_scans.clear();
    m_state.intake_paused = false;
    seq_at_reset = m_state.scan_seq;
  }
  const auto odom_deadline = Clock::now() + seconds(m_config.wait_lio_timeout_sec);
  bool new_run = false;
  while (rclcpp::ok() && Clock::now() < odom_deadline) {
    {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      if (m_state.scan_seq > seq_at_reset) {
        new_run = true;
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  if (!new_run) {
    fail(
      "Point-LIO was reset but published no odometry within " +
      std::to_string(m_config.wait_lio_timeout_sec) +
      " s (reset_odom.wait_lio_timeout_sec) -- still in IMU init? check the pointlio log, then "
      "relocalize or call again");
    return;
  }
  // ---- 4. Seed the new run's offset and let the timer refine it. ----
  //
  // With skip_seed the "seed" is the pre-reset map pose and the timer is not
  // released: the offset is written only so map -> body stays continuous from
  // the first new odometry on, while the search below runs.
  //
  // applyPlanarGuess's construction: keep the new odometry's gravity tilt
  // (pointlio_odom is gravity-aligned from the IMU init that just ran) and
  // only turn its heading to the seed's yaw. A flat guess never converges on
  // the tilted mount.
  M4F guess = M4F::Identity();
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    const M3D & new_r = m_state.last_r;
    const V3D & new_t = m_state.last_t;
    const double new_yaw = std::atan2(new_r(1, 0), new_r(0, 0));
    const M3D guess_r =
      Eigen::AngleAxisd(seed_yaw - new_yaw, V3D::UnitZ()).toRotationMatrix() * new_r;
    const V3D guess_t(seed_x, seed_y, seed_z);
    m_state.last_offset_r = guess_r * new_r.transpose();
    m_state.last_offset_t = guess_t - m_state.last_offset_r * new_t;
    guess.block<3, 3>(0, 0) = guess_r.cast<float>();
    guess.block<3, 1>(0, 3) = guess_t.cast<float>();
    if (!skip_seed) report(guess_r, guess_t);
  }

  // ---- 5. The hint-free search, on new-run scans only: the fallback of a
  // seed that did not converge, or with skip_seed the only attempt. Fills the
  // AutoLocalize-style measurements whenever the search produced a pose. ----
  auto global = [&](std::string & why) -> Convergence {
    GlobalSearchOutcome out;
    std::string error;
    const bool found_ok =
      globalSearch(Clock::now() + seconds(m_global_config.search_timeout_sec), out, error);
    response->candidates = out.found.candidates;
    if (out.found.has_pose) {
      response->z = static_cast<float>(out.map_body_t.z());
      response->fitness = static_cast<float>(out.found.fitness);
      response->inlier_ratio = static_cast<float>(out.found.inlier_ratio);
    }
    if (!found_ok) {
      why +=
        (why.empty() ? "the global search failed: " : "; the global search failed too: ") + error;
      return Convergence::kTimedOut;
    }
    response->method = "global";
    report(out.map_body_r, out.map_body_t);
    // A skip_seed call still holds the timer here (the search must not race a
    // registration around the continuity offset); release it now, with the
    // jump to the new origin exempt from the speed check, exactly as a seed is.
    m_state.skip_speed_check.store(true);
    m_state.lio_resetting.store(false);
    const Convergence result = handOverAndAwait(out.guess, m_global_config.converge_timeout_sec);
    if (result != Convergence::kConverged) {
      why += std::string(why.empty() ? "" : "; ") +
             "the global search found a pose but the timer did not converge on it";
    }
    return result;
  };

  Convergence result = Convergence::kTimedOut;
  std::string why;
  if (skip_seed) {
    // The search runs straight away, so wait for the whole scan window
    // (auto_loc.accum_scans), not just the first pair: the seeded path
    // refills it during its converge wait, this one has none. Same deadline
    // as the first pair's; recent_scans was cleared at the boundary, so its
    // size counts new-run scans only.
    size_t window = 0;
    while (rclcpp::ok() && Clock::now() < odom_deadline) {
      {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        window = m_state.recent_scans.size();
      }
      if (window >= static_cast<size_t>(m_global_config.accum_scans)) break;
      std::this_thread::sleep_for(50ms);
    }
    if (window < static_cast<size_t>(m_global_config.accum_scans)) {
      // Not a failure: the search can run on fewer scans, only less reliably.
      RCLCPP_WARN(
        this->get_logger(),
        "reset_odometry: only %zu of %d new-run scans within %.1f s; searching on those", window,
        m_global_config.accum_scans, m_config.wait_lio_timeout_sec);
    }
    RCLCPP_INFO(
      this->get_logger(), "reset_odometry: %.2f s after the reset; running the global search",
      std::chrono::duration<double>(Clock::now() - t_start).count());
    result = global(why);
  } else {
    // The guess goes in BEFORE the timer is released, so its first round is a
    // service round (no motion gate, no blend, settle window) rather than one
    // that measures the jump to the new origin as motion.
    {
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      m_state.initial_guess = guess;
      m_state.service_received = true;
      m_state.localize_success = false;
    }
    m_state.skip_speed_check.store(true);
    m_state.lio_resetting.store(false);
    RCLCPP_INFO(
      this->get_logger(),
      "reset_odometry: new odometry after %.2f s; seeded, waiting for the timer",
      std::chrono::duration<double>(Clock::now() - t_start).count());

    result = handOverAndAwait(guess, m_config.converge_timeout_sec);
    if (result == Convergence::kSuperseded) {
      why = "another guess replaced the seed before the timer converged";
    } else if (result == Convergence::kTimedOut) {
      why = "the timer did not converge on the " + response->method + " seed within " +
            std::to_string(m_config.converge_timeout_sec) + " s (reset_odom.converge_timeout_sec)";
    }

    // By now recent_scans has refilled (10 Hz for converge_timeout_sec).
    if (result == Convergence::kTimedOut && request->global_fallback) {
      RCLCPP_WARN(
        this->get_logger(), "reset_odometry: %s; falling back to the global search", why.c_str());
      result = global(why);
    }
  }

  if (result != Convergence::kConverged) {
    if (skip_seed) {
      fail(
        "Point-LIO was reset, but " + why +
        "; the offset is left where the map pose was before the reset -- relocalize, or call "
        "again with use_hint");
    } else {
      fail(
        "Point-LIO was reset and the map pose seeded, but " + why +
        "; the offset is left at the seed -- relocalize, or call again with use_hint");
    }
    return;
  }

  M3D r;
  V3D t;
  currentMapPose(r, t);
  report(r, t);
  response->success = true;
  char buf[200];
  std::snprintf(
    buf, sizeof(buf), "odometry reset; localized from %s at (%.2f, %.2f, %.1f deg) in %.2f s",
    response->method.c_str(), response->x, response->y, response->yaw * 180.0 / M_PI,
    std::chrono::duration<double>(Clock::now() - t_start).count());
  response->message = buf;
  RCLCPP_INFO(this->get_logger(), "reset_odometry: %s", buf);
}

void LocalizerNode::relocCheckCB(
  const std::shared_ptr<syncai_common::srv::IsValid::Request> request,
  std::shared_ptr<syncai_common::srv::IsValid::Response> response)
{
  // code 1 is a liveness probe ("is the service up?") and always answers true; anything else
  // asks the real question. This is the stack's ONLY localization-quality signal --
  // RobotState.localization_valid is TF presence, nothing more.
  std::lock_guard<std::mutex> lock(m_state.service_mutex);
  if (request->code == 1)
    response->valid = true;
  else
    response->valid = m_state.localize_success;
  return;
}

void LocalizerNode::publishMapCloud(builtin_interfaces::msg::Time & time)
{
  // Latched: publish even when there is no subscriber right now; DDS caches it for late joiners.
  CloudType::Ptr map_cloud = m_localizer->refineMap();
  if (map_cloud->size() < 1) return;
  sensor_msgs::msg::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(*map_cloud, map_cloud_msg);
  map_cloud_msg.header.frame_id = m_config.map_frame;
  map_cloud_msg.header.stamp = time;
  m_map_cloud_pub->publish(map_cloud_msg);
}

}  // namespace syncai_localizer
