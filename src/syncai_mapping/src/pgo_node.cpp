#include "syncai_mapping/pgo_node.hpp"

#include <pcl/common/io.h>
#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "syncai_mapping/map_clean_recipe.hpp"

extern char ** environ;

namespace syncai_mapping
{

using namespace std::chrono_literals;

PGONode::PGONode() : Node("pgo_node")
{
  RCLCPP_INFO(this->get_logger(), "PGO node started");
  loadParameters();
  m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
  rclcpp::QoS qos = rclcpp::QoS(10);
  m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
  m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
  // Relative name, was the absolute "/pgo/loop_markers": the launch runs this
  // node at /<robot_id>/pgo, and an absolute name ignores that namespace, so
  // two robots in one DDS domain would publish onto the same topic. The
  // inputs above stay absolute on purpose — they live in pointlio's
  // namespace, which a relative name cannot reach.
  m_loop_marker_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_markers", 10000);
  // Relative like loop_markers, so it lands on /<robot_id>/pgo/map_cloud.
  // Depth 1: each message is a multi-MB full-map merge and only the latest
  // matters — queueing old merges would just hold memory.
  //
  // Kept for rviz (pgo.rviz) and `ros2 topic echo`; the operator console's
  // backend no longer reads it. A large site's merge is 16-45 MB, and over
  // CycloneDDS/UDP on `lo` that is tens of thousands of datagrams in one
  // burst into a socket buffer capped by net.core.rmem_max (208 KB by
  // default) -- fragments drop, and a BEST_EFFORT reader loses the whole
  // sample, so the preview simply stopped once the map grew. Hence the
  // second output below.
  m_map_cloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", rclcpp::QoS(1));
  // The file hand-off: the merge is written as a binary PCD into
  // map_cloud_dir and this ~200 B JSON notice names it (see
  // mapCloudNoticeJson for the fields). RELIABLE + TRANSIENT_LOCAL depth 1:
  // latching a notice is free, and it means a backend (re)started
  // mid-mapping gets the current map at once instead of waiting for the next
  // keyframe. The reader must request TRANSIENT_LOCAL too, or nothing is
  // replayed. An empty-map notice (reset) replaces the latched one, so a late
  // joiner never learns about a file that has been deleted.
  m_map_cloud_file_pub = this->create_publisher<std_msgs::msg::String>(
    "map_cloud_file", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  // The run state (IDLE / MAPPING / RESETTING, see NodeState::phase). Latched
  // for the same reason as the file notice: the backend's own process may
  // (re)start at any point in a session and has to know whether a Start is
  // needed without waiting for the next transition. Republished at 1 Hz by
  // m_status_timer so the keyframe count moves and so a consumer can age a
  // sample out once this node is gone.
  m_status_pub = this->create_publisher<syncai_common::msg::MappingStatus>(
    "mapping_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  setupMapCloudDir();
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  m_sync =
    std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
  m_sync->setAgePenalty(0.1);
  m_sync->registerCallback(
    std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
  m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
  m_status_timer = this->create_wall_timer(1s, std::bind(&PGONode::statusTimerCB, this));
  // Relative for the same reason as loop_markers — the service is now
  // /<robot_id>/pgo/save_maps. The type is syncai_common's since 2026-09: the
  // backend (its only caller, from its own container) builds against that
  // package, and it went there with this node when pgo left SyncAI-Fast-LIO2.
  m_save_map_srv = this->create_service<syncai_common::srv::SaveMaps>(
    "save_maps",
    std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

  // Two dedicated MutuallyExclusive groups, same shape (and the same reason)
  // as localizer_node's: a handler that blocks must not sit on the group that
  // owns the 50 ms TF broadcast.
  //
  // The client's group is not a nicety, it is what stops a deadlock.
  // resetMappingCB blocks on the ResetLIO future, so the response has to be
  // delivered by a DIFFERENT executor thread, which means a different
  // callback group. (And never spin_until_future_complete from inside a
  // callback -- that re-enters the executor that is already busy running us.)
  m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_cli_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  m_reset_srv = this->create_service<syncai_common::srv::ResetMapping>(
    "reset_mapping",
    std::bind(&PGONode::resetMappingCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);
  // Same group as the reset, deliberately: the two run the same blocking
  // sequence (beginRun) and a MutuallyExclusive group serialises them.
  m_start_srv = this->create_service<syncai_common::srv::StartMapping>(
    "start_mapping",
    std::bind(&PGONode::startMappingCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);

  m_lio_reset_cli = this->create_client<syncai_common::srv::ResetLIO>(
    m_node_config.lio_reset_service, rmw_qos_profile_services_default, m_cli_cb_group);

  // The session comes up IDLE (the NodeState default) and nothing reaches
  // the graph until start_mapping -- said out loud, because the symptom of a
  // forgotten Start is silence. start_on_launch is the replay escape hatch.
  if (m_node_config.start_on_launch) {
    m_state.phase.store(syncai_common::msg::MappingStatus::MAPPING);
    RCLCPP_WARN(
      this->get_logger(),
      "[PGONode] start_on_launch is set: mapping from the first pair, no start_mapping needed");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode] idle until start_mapping is called; pairs are dropped and only an identity "
      "map -> %s is broadcast meanwhile",
      m_node_config.local_frame.c_str());
  }
  publishStatus();
}

PGONode::~PGONode()
{
  // A merge may still be running on the worker; joining here keeps shutdown
  // from tearing the publisher down under it.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
  // Best effort: the files are ours and nobody else will ever remove them
  // (the dir is the host's tmpfs, so it outlives this container). A stale
  // notice pointing here after we are gone is the reader's ENOENT to skip.
  clearMapCloudDir();
  // Neither killed nor waited for, on purpose: the clean is detached so that
  // it survives the mapping session going away (a mode switch right after
  // the save is the normal operator flow), and this is the one shutdown path
  // -- Ctrl-C in the pane -- where killing it would even be possible. Its
  // outcome lands in the sidecar either way.
  for (const auto & child : m_map_clean_children) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PGONode][map_clean] clean_map pid %d for %s continues detached; its outcome lands in %s",
      child.pid, child.dir.c_str(), map_clean_recipe::kSidecarFile);
  }
}

// ---- The file hand-off (see the map_cloud_file publisher) ----------------

void PGONode::setupMapCloudDir()
{
  m_map_cloud_dir = m_node_config.map_cloud_dir;
  std::error_code ec;
  std::filesystem::create_directories(m_map_cloud_dir, ec);
  if (ec) {
    // Not fatal on purpose: this node also owns the map->local_frame TF
    // broadcast, and a preview must never take that down. The PointCloud2
    // path keeps working; the file path is simply disabled for this run.
    RCLCPP_ERROR(
      this->get_logger(),
      "[PGONode] cannot create map_cloud_dir %s (%s); the map_cloud_file "
      "hand-off is disabled for this run",
      m_map_cloud_dir.c_str(), ec.message().c_str());
    m_map_cloud_dir_ok = false;
    return;
  }
  m_map_cloud_dir_ok = true;
  // A crash leaves files behind, and they would otherwise accumulate across
  // runs (nothing but this node ever deletes them). Ours only: files this
  // node's naming produced, never the directory itself.
  clearMapCloudDir();
  RCLCPP_INFO(
    this->get_logger(), "[PGONode] map_cloud merges are handed off as PCD files under %s",
    m_map_cloud_dir.c_str());
}

bool PGONode::isMapCloudFile(const std::filesystem::directory_entry & entry)
{
  // error_code overload: this runs from the destructor too, where a throw
  // out of a stat() would be fatal rather than merely a file left behind.
  std::error_code ec;
  if (!entry.is_regular_file(ec) || ec) return false;
  const std::string name = entry.path().filename().string();
  if (name.rfind("map_cloud_", 0) != 0) return false;
  const std::string ext = entry.path().extension().string();
  return ext == ".pcd" || ext == ".tmp";
}

// Remove every file this node wrote. Errors are ignored: the caller is a
// reset or shutdown, and there is nothing useful to do about a file that
// will not go away except log it.
void PGONode::clearMapCloudDir()
{
  if (!m_map_cloud_dir_ok) return;
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(m_map_cloud_dir, ec)) {
    if (!isMapCloudFile(entry)) continue;
    std::error_code rm_ec;
    std::filesystem::remove(entry.path(), rm_ec);
    if (rm_ec) {
      RCLCPP_WARN(
        this->get_logger(), "[PGONode] could not remove %s: %s", entry.path().c_str(),
        rm_ec.message().c_str());
    }
  }
}

// Keep the newest two `map_cloud_<seq>.pcd` by seq, delete the rest, and
// delete any stray `.tmp` (a write that never reached rename). Two, not one:
// the backend's reader is KEEP_LAST 1, so the notice it is acting on can be
// at most one behind the one just published, and the file it names must
// still exist when it opens it. (An already-open file survives unlink on
// Linux, so mid-read is safe regardless.) By seq rather than "seq - 2":
// the reset's empty notice consumes a seq too, so a literal offset would
// leak a file after every reset.
void PGONode::pruneMapCloudFiles()
{
  std::vector<std::pair<uint64_t, std::filesystem::path>> pcds;
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(m_map_cloud_dir, ec)) {
    if (!isMapCloudFile(entry)) continue;
    if (entry.path().extension() == ".tmp") {
      std::error_code rm_ec;
      std::filesystem::remove(entry.path(), rm_ec);
      continue;
    }
    const std::string stem = entry.path().stem().string();  // map_cloud_<seq>
    try {
      pcds.emplace_back(std::stoull(stem.substr(std::string("map_cloud_").size())), entry.path());
    } catch (const std::exception &) {
      // Not ours after all (or hand-made); leave it alone.
    }
  }
  std::sort(
    pcds.begin(), pcds.end(), [](const auto & a, const auto & b) { return a.first > b.first; });
  for (size_t i = 2; i < pcds.size(); ++i) {
    std::error_code rm_ec;
    std::filesystem::remove(pcds[i].second, rm_ec);
  }
}

std::string PGONode::jsonEscape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  for (const unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out;
}

// The notice the backend parses. Five fixed fields, hand-formatted so this
// node grows no JSON dependency:
//   seq       monotonic within this process; informational for the reader
//             (a restart begins again at 1, and the reader applies it anyway)
//   path      absolute path of the PCD, "" when the map is empty
//   points    point count after the voxel filter, 0 when the map is empty
//   frame_id  map_frame -- the points are already global
//   stamp     the keyframe's time, same as the PointCloud2 header
std::string PGONode::mapCloudNoticeJson(
  uint64_t seq, const std::string & path, size_t points,
  const builtin_interfaces::msg::Time & time) const
{
  std::ostringstream os;
  os << "{\"seq\":" << seq << ",\"path\":\"" << jsonEscape(path) << "\",\"points\":" << points
     << ",\"frame_id\":\"" << jsonEscape(m_node_config.map_frame)
     << "\",\"stamp\":{\"sec\":" << time.sec << ",\"nanosec\":" << time.nanosec << "}}";
  return os.str();
}

void PGONode::publishMapCloudNotice(
  uint64_t seq, const std::string & path, size_t points, const builtin_interfaces::msg::Time & time)
{
  std_msgs::msg::String msg;
  msg.data = mapCloudNoticeJson(seq, path, points, time);
  m_map_cloud_file_pub->publish(msg);
}

// Write the merge as `map_cloud_<seq>.pcd` (via .tmp + rename, so a reader
// never sees a partial file) and announce it. Every failure is caught HERE:
// this runs on m_map_cloud_thread, and an exception escaping a std::thread
// is std::terminate -- pgo gone, TF gone, mid-run. savePCDFileBinary does
// throw (pcl::IOException on an empty cloud and on a failed write, ENOSPC
// included -- exactly what a 64 MB private /dev/shm produces at a large
// site), so the empty case is handled before the writer is ever called.
void PGONode::writeAndAnnounceMapCloud(
  const CloudType & merged, uint64_t seq, const builtin_interfaces::msg::Time & time)
{
  if (merged.empty()) {
    publishMapCloudNotice(seq, "", 0, time);
    return;
  }
  const std::filesystem::path final_path =
    std::filesystem::path(m_map_cloud_dir) / ("map_cloud_" + std::to_string(seq) + ".pcd");
  const std::filesystem::path tmp_path = final_path.string() + ".tmp";
  try {
    if (pcl::io::savePCDFileBinary(tmp_path.string(), merged) != 0) {
      throw std::runtime_error("savePCDFileBinary returned non-zero");
    }
    std::filesystem::rename(tmp_path, final_path);  // throws filesystem_error
    publishMapCloudNotice(seq, final_path.string(), merged.size(), time);
    pruneMapCloudFiles();
  } catch (const pcl::IOException & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode] map_cloud file hand-off failed writing %s: %s",
      tmp_path.c_str(), e.what());
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode] map_cloud file hand-off failed on %s: %s", tmp_path.c_str(),
      e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "[PGONode] map_cloud file hand-off failed: %s", e.what());
  }
  // Whatever happened, do not leave a half-written .tmp behind.
  std::error_code ec;
  std::filesystem::remove(tmp_path, ec);
}

// Everything is a declared ROS parameter, fed by params/mapping_params.yaml
// through the launch file's `parameters=[file, overrides]`. Until 2026-09 the
// node took a single `config_path` parameter and parsed that YAML itself with
// yaml-cpp, which put it outside every ROS tool (`ros2 param list` showed only
// config_path, a launch-level override could not touch a single value) and
// forced the launch file to rewrite the whole YAML into
// /tmp/syncai_pgo/pgo_<robot_id>.yaml just to inject the robot_id prefix.
// Declaring them properly is what lets the launch layer the five
// robot_id-dependent values on top of the shared file, the same way
// syncai_pointlio's launch layers its two frame names.
//
// The defaults repeat the struct defaults (NodeConfig here, Config in
// pgos/simple_pgo.h), so a missing key degrades to running the node bare
// instead of throwing -- a deliberate change from the yaml-cpp loader, where
// twelve of these keys were required and a missing one was an InvalidNode
// exception at construction. The trade is that a misspelt key silently uses
// its default; the log of the resolved instance values below is the
// counterweight, because a bare-run pgo subscribes to a topic nothing
// publishes and would otherwise just be silent.
//
// Read once. m_pgo_config is copied into SimplePGO at construction and again
// on every reset, and the message_filters subscribers take their topic names
// at construction, so `ros2 param set` after startup changes nothing.
void PGONode::loadParameters()
{
  // Inputs, frames and the LIO reset service. Absolute topic / service names
  // on purpose: they live in pointlio's namespace, which a relative name from
  // inside /<robot_id>/pgo cannot reach, so the launch overrides all of them
  // per robot_id. The fallbacks here must never contain a robot_id.
  m_node_config.cloud_topic = this->declare_parameter("cloud_topic", m_node_config.cloud_topic);
  m_node_config.odom_topic = this->declare_parameter("odom_topic", m_node_config.odom_topic);
  m_node_config.map_frame = this->declare_parameter("map_frame", m_node_config.map_frame);
  // Must equal syncai_pointlio's world_frame override (<robot_id>/pointlio_odom):
  // this node broadcasts map -> local_frame and, unlike the localizer, does NOT
  // adopt the frame from the incoming odometry header, so a mismatch lands
  // the correction on a frame nobody looks up.
  m_node_config.local_frame = this->declare_parameter("local_frame", m_node_config.local_frame);
  m_node_config.lio_reset_service =
    this->declare_parameter("lio_reset_service", m_node_config.lio_reset_service);
  // The launch passes the per-robot subdirectory (<base>/<robot_id>) so two
  // robots on one host never prune each other's files.
  m_node_config.map_cloud_dir =
    this->declare_parameter("map_cloud_dir", m_node_config.map_cloud_dir);
  m_node_config.map_cloud_resolution =
    this->declare_parameter("map_cloud_resolution", m_node_config.map_cloud_resolution);
  m_node_config.map_cloud_pub_period =
    this->declare_parameter("map_cloud_pub_period", m_node_config.map_cloud_pub_period);
  // Replay escape hatch; see NodeConfig. Read after the hand-off settings so
  // the startup log below can report it in one place.
  m_node_config.start_on_launch =
    this->declare_parameter("start_on_launch", m_node_config.start_on_launch);

  // PGO math (pgos/simple_pgo.h Config). All doubles except
  // loop_submap_half_range, which is an int -- and the params file has to
  // match: a bare `10` for a double key is an int64 override that throws
  // InvalidParameterTypeException at startup, and `5.0` for the int key is the
  // mirror-image failure.
  m_pgo_config.key_pose_delta_deg =
    this->declare_parameter("key_pose_delta_deg", m_pgo_config.key_pose_delta_deg);
  m_pgo_config.key_pose_delta_trans =
    this->declare_parameter("key_pose_delta_trans", m_pgo_config.key_pose_delta_trans);
  m_pgo_config.loop_search_radius =
    this->declare_parameter("loop_search_radius", m_pgo_config.loop_search_radius);
  m_pgo_config.loop_time_tresh =
    this->declare_parameter("loop_time_tresh", m_pgo_config.loop_time_tresh);
  m_pgo_config.loop_score_tresh =
    this->declare_parameter("loop_score_tresh", m_pgo_config.loop_score_tresh);
  m_pgo_config.loop_icp_max_corr_dist =
    this->declare_parameter("loop_icp_max_corr_dist", m_pgo_config.loop_icp_max_corr_dist);
  m_pgo_config.loop_registration =
    this->declare_parameter("loop_registration", m_pgo_config.loop_registration);
  m_pgo_config.loop_gicp_num_threads =
    this->declare_parameter("loop_gicp_num_threads", m_pgo_config.loop_gicp_num_threads);
  m_pgo_config.loop_gicp_num_neighbors =
    this->declare_parameter("loop_gicp_num_neighbors", m_pgo_config.loop_gicp_num_neighbors);
  m_pgo_config.loop_planar_correction =
    this->declare_parameter("loop_planar_correction", m_pgo_config.loop_planar_correction);
  m_pgo_config.loop_noise_var_roll_pitch_z = this->declare_parameter(
    "loop_noise_var_roll_pitch_z", m_pgo_config.loop_noise_var_roll_pitch_z);
  m_pgo_config.loop_noise_yaw_sigma_deg =
    this->declare_parameter("loop_noise_yaw_sigma_deg", m_pgo_config.loop_noise_yaw_sigma_deg);
  m_pgo_config.loop_noise_xy_sigma_m =
    this->declare_parameter("loop_noise_xy_sigma_m", m_pgo_config.loop_noise_xy_sigma_m);
  m_pgo_config.keyframe_tilt_sigma_deg =
    this->declare_parameter("keyframe_tilt_sigma_deg", m_pgo_config.keyframe_tilt_sigma_deg);
  if (m_pgo_config.loop_registration != "gicp" && m_pgo_config.loop_registration != "icp") {
    RCLCPP_WARN(
      this->get_logger(), "[PGONode] loop_registration '%s' unknown, using 'gicp'",
      m_pgo_config.loop_registration.c_str());
    m_pgo_config.loop_registration = "gicp";
  }
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] loop verification: %s, max corr dist %.2f m, fitness gate %.3f, planar "
    "correction %s, roll/pitch/z variance %.1e, planar xy sigma %.3f m%s, planar yaw sigma "
    "%.2f deg%s, keyframe tilt sigma %.2f deg%s",
    m_pgo_config.loop_registration.c_str(), m_pgo_config.loop_icp_max_corr_dist,
    m_pgo_config.loop_score_tresh, m_pgo_config.loop_planar_correction ? "on" : "off",
    m_pgo_config.loop_noise_var_roll_pitch_z, m_pgo_config.loop_noise_xy_sigma_m,
    m_pgo_config.loop_noise_xy_sigma_m > 0.0 ? "" : " (<= 0: variance = fitness)",
    m_pgo_config.loop_noise_yaw_sigma_deg,
    m_pgo_config.loop_noise_yaw_sigma_deg > 0.0 ? "" : " (<= 0: variance = fitness)",
    m_pgo_config.keyframe_tilt_sigma_deg,
    m_pgo_config.keyframe_tilt_sigma_deg > 0.0 ? "" : " (<= 0: off)");
  m_pgo_config.loop_submap_half_range =
    this->declare_parameter("loop_submap_half_range", m_pgo_config.loop_submap_half_range);
  m_pgo_config.submap_resolution =
    this->declare_parameter("submap_resolution", m_pgo_config.submap_resolution);
  m_pgo_config.min_loop_detect_duration =
    this->declare_parameter("min_loop_detect_duration", m_pgo_config.min_loop_detect_duration);

  // The post-save map cleaning. Handed to clean_map on its command line and
  // recorded in the sidecar, so what a map was cleaned with is on disk beside
  // it. All doubles except map_clean_nice, map_clean_min_hits and
  // map_clean_dynamic_min_miss (ints) and map_clean_clip_below_floor (bool).
  m_node_config.map_clean_enabled =
    this->declare_parameter("map_clean_enabled", m_node_config.map_clean_enabled);
  m_node_config.map_clean_nice =
    this->declare_parameter("map_clean_nice", m_node_config.map_clean_nice);
  auto & mc = m_node_config.map_clean;
  mc.resolution = this->declare_parameter("map_clean_resolution", mc.resolution);
  mc.max_range = this->declare_parameter("map_clean_max_range", mc.max_range);
  mc.min_range = this->declare_parameter("map_clean_min_range", mc.min_range);
  mc.lidar_height = this->declare_parameter("map_clean_lidar_height", mc.lidar_height);
  mc.floor_band = this->declare_parameter("map_clean_floor_band", mc.floor_band);
  mc.max_height = this->declare_parameter("map_clean_max_height", mc.max_height);
  mc.floor_radius = this->declare_parameter("map_clean_floor_radius", mc.floor_radius);
  mc.min_hits = this->declare_parameter("map_clean_min_hits", mc.min_hits);
  mc.dynamic_miss_ratio =
    this->declare_parameter("map_clean_dynamic_miss_ratio", mc.dynamic_miss_ratio);
  mc.dynamic_min_miss = this->declare_parameter("map_clean_dynamic_min_miss", mc.dynamic_min_miss);
  mc.clip_below_floor = this->declare_parameter("map_clean_clip_below_floor", mc.clip_below_floor);
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] map cleaning after save: %s | %.3f m, range %.1f-%.1f m, lidar height %.3f m, "
    "floor band %.2f m, max height %.1f m, floor radius %.1f m, min hits %d, dynamic miss ratio "
    "%.2f, dynamic min miss %d, clip below floor %s, nice %d",
    m_node_config.map_clean_enabled ? "on" : "off", mc.resolution, mc.min_range, mc.max_range,
    mc.lidar_height, mc.floor_band, mc.max_height, mc.floor_radius, mc.min_hits,
    mc.dynamic_miss_ratio, mc.dynamic_min_miss, mc.clip_below_floor ? "on" : "off",
    m_node_config.map_clean_nice);

  // The five values the launch is expected to have overridden. A value here
  // without a robot_id prefix means the node was started bare, or the
  // overrides were passed before the params file (later entries win).
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] inputs: %s + %s | TF %s -> %s | LIO reset: %s | map_cloud_dir: %s | "
    "start_on_launch: %s",
    m_node_config.cloud_topic.c_str(), m_node_config.odom_topic.c_str(),
    m_node_config.map_frame.c_str(), m_node_config.local_frame.c_str(),
    m_node_config.lio_reset_service.c_str(), m_node_config.map_cloud_dir.c_str(),
    m_node_config.start_on_launch ? "true" : "false");
}

void PGONode::syncCB(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  /**
   * cloud_msg: point cloud in the LIO odom frame
   * odom_msg: LIO odom -> robot pose
   */

  CloudWithPose cp;
  cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
  // Recorded for every pair, taken or not: a start_mapping with reset_lio
  // false gates the new run on "after this call" and this is its boundary.
  m_state.last_seen_time.store(cp.pose.second);

  // Gate before the expensive part: a pair dropped here costs nothing, while
  // pcl::fromROSMsg below is a full copy of a lidar frame.
  //
  // Anything but MAPPING drops the pair -- see NodeState::phase. Dropping
  // rather than buffering is the point: a pair produced before a Start or
  // while the front end is being reset belongs to no run. IDLE additionally
  // keeps the map -> local_frame TF alive as identity, so the console's live
  // scan (which looks that transform up) stays visible while the operator
  // positions the robot; RESETTING stays silent, as the reset always has.
  const uint8_t phase = m_state.phase.load();
  if (phase != syncai_common::msg::MappingStatus::MAPPING) {
    if (phase == syncai_common::msg::MappingStatus::IDLE) sendIdentityTF(cloud_msg->header.stamp);
    return;
  }

  // The old run's tail. pointlio reported this boundary as the stamp of the
  // last odometry it published before resetting, so <= is the whole of the
  // old run and > is the whole of the new one. Without this, a single
  // straddling frame anchors a BetweenFactor carrying the entire accumulated
  // drift at 1e-6 variance, and the graph never recovers.
  if (cp.pose.second <= m_state.accept_after_time.load()) return;

  if (cp.pose.second < m_state.last_message_time) {
    RCLCPP_WARN(this->get_logger(), "Received out of order message");
    return;
  }

  // Was `std::lock_guard<std::mutex>(m_state.message_mutex);` -- with no
  // declarator-id that parses as a functional cast, so it built a temporary
  // and released the mutex at the end of that very statement. The buffer was
  // unguarded, and got away with it only because rclcpp::spin() serialised
  // syncCB against timerCB. main() now runs a MultiThreadedExecutor, so it
  // would not get away with it any more.
  std::lock_guard<std::mutex> lock(m_state.message_mutex);
  m_state.last_message_time = cp.pose.second;

  cp.pose.r = Eigen::Quaterniond(
                odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                .toRotationMatrix();
  cp.pose.t = V3D(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  cp.cloud = CloudType::Ptr(new CloudType);
  pcl::fromROSMsg(*cloud_msg, *cp.cloud);
  m_state.cloud_buffer.push(cp);
}

void PGONode::sendBroadCastTF(builtin_interfaces::msg::Time & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_node_config.map_frame;
  transformStamped.child_frame_id = m_node_config.local_frame;
  transformStamped.header.stamp = time;
  Eigen::Quaterniond q(m_pgo->offsetR());
  V3D t = m_pgo->offsetT();
  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  m_tf_broadcaster->sendTransform(transformStamped);
}

// Identity is also what a fresh SimplePGO's offset is, so the first keyframe
// after a Start causes no jump. No m_pgo access, hence no m_pgo_mutex: this
// runs on the intake thread while a Start may be replacing m_pgo.
void PGONode::sendIdentityTF(const builtin_interfaces::msg::Time & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_node_config.map_frame;
  transformStamped.child_frame_id = m_node_config.local_frame;
  transformStamped.header.stamp = time;
  transformStamped.transform.rotation.w = 1.0;
  m_tf_broadcaster->sendTransform(transformStamped);
}

void PGONode::publishStatus()
{
  syncai_common::msg::MappingStatus msg;
  msg.state = m_state.phase.load();
  msg.key_poses = m_key_pose_count.load();
  msg.loop_closures = m_loop_count.load();
  msg.stamp = this->get_clock()->now();
  m_status_pub->publish(msg);
}

void PGONode::statusTimerCB()
{
  publishStatus();
  // Here because this timer shares the default group with saveMapsCB, the
  // only other writer of m_map_clean_children; 1 Hz is plenty for a job that
  // takes minutes.
  reapMapCleans();
}

void PGONode::publishLoopMarkers(builtin_interfaces::msg::Time & time)
{
  if (m_loop_marker_pub->get_subscription_count() == 0) return;
  if (m_pgo->historyPairs().size() == 0) return;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker nodes_marker;
  visualization_msgs::msg::Marker edges_marker;
  nodes_marker.header.frame_id = m_node_config.map_frame;
  nodes_marker.header.stamp = time;
  nodes_marker.ns = "pgo_nodes";
  nodes_marker.id = 0;
  nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker.action = visualization_msgs::msg::Marker::ADD;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.scale.x = 0.3;
  nodes_marker.scale.y = 0.3;
  nodes_marker.scale.z = 0.3;
  nodes_marker.color.r = 1.0;
  nodes_marker.color.g = 0.8;
  nodes_marker.color.b = 0.0;
  nodes_marker.color.a = 1.0;

  edges_marker.header.frame_id = m_node_config.map_frame;
  edges_marker.header.stamp = time;
  edges_marker.ns = "pgo_edges";
  edges_marker.id = 1;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.pose.orientation.w = 1.0;
  edges_marker.scale.x = 0.1;
  edges_marker.color.r = 0.0;
  edges_marker.color.g = 0.8;
  edges_marker.color.b = 0.0;
  edges_marker.color.a = 1.0;

  std::vector<KeyPoseWithCloud> & poses = m_pgo->keyPoses();
  std::vector<std::pair<size_t, size_t>> & pairs = m_pgo->historyPairs();
  for (size_t i = 0; i < pairs.size(); i++) {
    size_t i1 = pairs[i].first;
    size_t i2 = pairs[i].second;
    geometry_msgs::msg::Point p1, p2;
    p1.x = poses[i1].t_global.x();
    p1.y = poses[i1].t_global.y();
    p1.z = poses[i1].t_global.z();

    p2.x = poses[i2].t_global.x();
    p2.y = poses[i2].t_global.y();
    p2.z = poses[i2].t_global.z();

    nodes_marker.points.push_back(p1);
    nodes_marker.points.push_back(p2);
    edges_marker.points.push_back(p1);
    edges_marker.points.push_back(p2);
  }

  marker_array.markers.push_back(nodes_marker);
  marker_array.markers.push_back(edges_marker);
  m_loop_marker_pub->publish(marker_array);
}

void PGONode::timerCB()
{
  // Held for the whole body: everything below reads or mutates m_pgo, which
  // resetMappingCB replaces wholesale. See the m_pgo_mutex declaration for the
  // full discipline.
  std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

  CloudWithPose cp;
  {
    // front() used to be read OUTSIDE this block, and the block itself used to
    // be `std::lock_guard<std::mutex>(m_state.message_mutex);` -- a temporary,
    // so nothing was ever locked. Both are fixed together: the read and the
    // drain are one critical section against syncCB, which now runs on a
    // different executor thread.
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    if (m_state.cloud_buffer.empty()) return;
    cp = m_state.cloud_buffer.front();  // Take only the oldest entry
    // Drain the whole queue
    while (!m_state.cloud_buffer.empty()) {
      m_state.cloud_buffer.pop();
    }
  }

  builtin_interfaces::msg::Time cur_time;
  cur_time.sec = cp.pose.sec;
  cur_time.nanosec = cp.pose.nsec;

  if (!m_pgo->addKeyPose(cp)) {  // Keyframe selection
    sendBroadCastTF(cur_time);   // Not a keyframe -> only broadcast the transform
    return;
  }

  // Search for loop closures
  m_pgo->searchForLoopPairs();

  // One line per accepted loop, BEFORE the graph consumes it. This is the only
  // record of what a closure asked for: the ICP fitness that passed
  // loop_score_tresh, and the world-frame transform ICP applied to the current
  // keyframe, split into translation (z separately, because a loop that lifts
  // the robot is the failure that has actually happened -- see the 2026-10
  // dp1f_1002 replay, where seven corridor loops with |dt| < 0.2 m each added
  // ~2 cm of z and the map ended up with a 15 cm double floor) and rotation.
  // The pre-optimisation z of both keyframes is printed next to it so the log
  // alone shows whether the loop moved them closer together or further apart.
  const std::vector<LoopPair> & loops = m_pgo->pendingLoops();
  std::vector<double> src_z_before;
  src_z_before.reserve(loops.size());
  for (const LoopPair & lp : loops) {
    const KeyPoseWithCloud & tgt = m_pgo->keyPoses()[lp.target_id];
    const KeyPoseWithCloud & src = m_pgo->keyPoses()[lp.source_id];
    src_z_before.push_back(src.t_global.z());
    const double rot_deg = Eigen::AngleAxisd(lp.icp_r).angle() * 180.0 / M_PI;
    // Where ICP wants the source keyframe, not the transform's raw translation:
    // with a rotation about the world origin, t alone is meaningless 20 m out.
    const V3D moved = lp.icp_r * src.t_global + lp.icp_t - src.t_global;
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][loop] target %zu -> source %zu | fitness %.4f | ICP moves source by "
      "(%.3f, %.3f, %.3f) m, rot %.2f deg | z before: target %.3f source %.3f (source-target "
      "%+.3f)",
      lp.target_id, lp.source_id, lp.score, moved.x(), moved.y(), moved.z(), rot_deg,
      tgt.t_global.z(), src.t_global.z(), src.t_global.z() - tgt.t_global.z());
  }
  // The ids survive the update (they index m_key_poses); the pairs do not.
  std::vector<std::pair<size_t, size_t>> loop_ids;
  for (const LoopPair & lp : loops) loop_ids.emplace_back(lp.target_id, lp.source_id);

  // Graph optimisation
  m_pgo->smoothAndUpdate();
  // The mirrors mapping_status reports (lock-free readers; see the members).
  m_key_pose_count.store(static_cast<uint32_t>(m_pgo->keyPoses().size()));
  m_loop_count.store(static_cast<uint32_t>(m_pgo->historyPairs().size()));

  // ...and what the optimiser actually did with each loop: the same two
  // keyframes after the update. A source z that moved AWAY from the target's
  // is the smoking gun this log exists for.
  for (size_t i = 0; i < loop_ids.size(); i++) {
    const KeyPoseWithCloud & tgt = m_pgo->keyPoses()[loop_ids[i].first];
    const KeyPoseWithCloud & src = m_pgo->keyPoses()[loop_ids[i].second];
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][loop] target %zu -> source %zu | z after: target %.3f source %.3f "
      "(source-target %+.3f, source moved %+.3f)",
      loop_ids[i].first, loop_ids[i].second, tgt.t_global.z(), src.t_global.z(),
      src.t_global.z() - tgt.t_global.z(), src.t_global.z() - src_z_before[i]);
  }

  // Broadcast the transform
  sendBroadCastTF(cur_time);

  publishLoopMarkers(cur_time);

  // Only a keyframe tick reaches this point -- which is exactly when the merged map has
  // actually changed.
  publishMapCloud(cur_time);
}

void PGONode::publishMapCloud(builtin_interfaces::msg::Time & time)
{
  // Same gate as publishLoopMarkers: with nobody listening on EITHER output
  // (the operator console's backend takes the file notice; rviz takes the
  // PointCloud2) the merge is pure waste.
  const bool want_msg = m_map_cloud_pub->get_subscription_count() > 0;
  const bool want_file = m_map_cloud_dir_ok && m_map_cloud_file_pub->get_subscription_count() > 0;
  if (!want_msg && !want_file) return;
  if (m_pgo->keyPoses().empty()) return;

  // Keyframe-triggered AND rate-floored: keyframes land every ~0.5 m of
  // travel, which early in a run is faster than anyone needs a multi-MB
  // full-map merge. Timestamps are the keyframes' own (sensor time), so the
  // floor also behaves under sim/bag time.
  double now_s = m_pgo->keyPoses().back().time;
  if (now_s - m_last_map_cloud_time < m_node_config.map_cloud_pub_period) return;

  // One worker at a time. exchange() is the claim; the worker releases it as
  // its last act. If the previous merge is still running we simply skip this
  // keyframe — the next one re-triggers, and a merge that cannot keep up
  // with the period degrades to back-to-back merges, never to a queue.
  if (m_map_cloud_busy.exchange(true)) return;
  m_last_map_cloud_time = now_s;

  // busy was false, so a previous thread (if any) has finished executing;
  // join() only reclaims it and returns immediately.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();

  // Snapshot by value, made HERE on the timer thread — the only thread that
  // ever mutates m_key_poses. Poses are copied (~200 B per keyframe);
  // body_cloud Ptrs are shared, which is safe because a keyframe's cloud is
  // never written after insertion (smoothAndUpdate rewrites poses only).
  // This is also why the merge does NOT go through SimplePGO::getSubMap():
  // that reads m_key_poses live and would race the next timer tick.
  std::vector<KeyPoseWithCloud> snapshot = m_pgo->keyPoses();
  // seq is taken here, on the timer thread under m_pgo_mutex -- the same
  // lock the reset's empty notice increments it under -- and passed by
  // value, so the worker never touches the counter.
  const uint64_t seq = ++m_map_cloud_seq;
  m_map_cloud_thread =
    std::thread(&PGONode::mergeAndPublishMapCloud, this, std::move(snapshot), time, seq);
}

// Runs on m_map_cloud_thread. The whole point of the thread: late in a run
// this is a transform+concat over millions of points plus a VoxelGrid —
// hundreds of ms — and timerCB owns the map->local_frame TF broadcast, which
// must not gap for that long. rclcpp publishers are thread-safe.
void PGONode::mergeAndPublishMapCloud(
  std::vector<KeyPoseWithCloud> snapshot, builtin_interfaces::msg::Time time, uint64_t seq)
{
  // Same merge as saveMapsCB: each keyframe is transformed into the map frame with its
  // (r_global, t_global) and stacked.
  CloudType::Ptr merged(new CloudType);
  for (const auto & kp : snapshot) {
    CloudType::Ptr world_cloud(new CloudType);
    pcl::transformPointCloud(
      *kp.body_cloud, *world_cloud, kp.t_global, Eigen::Quaterniond(kp.r_global));
    *merged += *world_cloud;
  }

  if (m_node_config.map_cloud_resolution > 0) {
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(
      m_node_config.map_cloud_resolution, m_node_config.map_cloud_resolution,
      m_node_config.map_cloud_resolution);
    voxel_filter.setInputCloud(merged);
    voxel_filter.filter(*merged);
  }

  // Already global: every point was placed with the snapshot's corrected
  // poses, so downstream needs no TF — and a re-publish after a loop closure
  // moves the whole map into its corrected shape. Both outputs carry the
  // same merge; each is produced only if someone is listening to it (the
  // subscriber counts are re-read here rather than passed in, so a reader
  // that left during a slow merge costs nothing).
  if (m_map_cloud_pub->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    msg.header.frame_id = m_node_config.map_frame;
    msg.header.stamp = time;
    m_map_cloud_pub->publish(msg);
  }
  if (m_map_cloud_dir_ok && m_map_cloud_file_pub->get_subscription_count() > 0) {
    writeAndAnnounceMapCloud(*merged, seq, time);
  }

  // Last, deliberately: this is what lets publishMapCloud spawn the next
  // worker, and everything this thread does must be finished by then.
  m_map_cloud_busy.store(false);
}

// Tell every consumer the map is gone. All of these are published UNGATED,
// unlike their counterparts in the normal path: the subscriber-count check
// exists to skip expensive merges nobody wants, but "the map is empty now" is
// two dozen bytes and is precisely the message a late or idle subscriber
// must not miss. Without these, the last thing rviz and the operator console
// hold is the map the operator was just told had been discarded.
//
// Caller holds m_pgo_mutex and has joined the worker (stopRunLocked, on
// behalf of a start, a reset or a save), so the seq increment races nothing
// and no .tmp is mid-write when the files go. The empty notice is published
// BEFORE the files are removed and, being TRANSIENT_LOCAL depth 1, replaces
// the latched notice that named them.
void PGONode::publishEmptyMapCloud(const builtin_interfaces::msg::Time & time)
{
  CloudType empty;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(empty, msg);
  msg.header.frame_id = m_node_config.map_frame;
  msg.header.stamp = time;
  m_map_cloud_pub->publish(msg);

  publishMapCloudNotice(++m_map_cloud_seq, "", 0, time);
  clearMapCloudDir();
}

void PGONode::publishLoopMarkerDeleteAll()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  m_loop_marker_pub->publish(marker_array);
}

// Everything that turns "a run" into "no run". Caller holds m_pgo_mutex.
void PGONode::stopRunLocked(const builtin_interfaces::msg::Time & now)
{
  // NOT for safety -- mergeAndPublishMapCloud works on a by-value snapshot
  // and never touches m_pgo, so destroying SimplePGO under it was already
  // fine (the keyframe clouds are refcounted). The join is here so an
  // in-flight merge cannot publish the OLD map after we publish the empty
  // one, which would undo the only thing telling consumers the map is gone.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
  m_map_cloud_busy.store(false);
  m_last_map_cloud_time = 0.0;

  // The constructor IS the reset: fresh ISAM2, empty values and graph,
  // identity offsets, and the three keyframe vectors empty by virtue of
  // being a new object. Deliberately not a SimplePGO::reset() method --
  // gtsam::ISAM2 has no clear, so such a method would be a second
  // definition of "empty" that has to stay in sync with this one. After a
  // save this is also what frees the keyframe clouds: hundreds of MB on a
  // long drive, all of it on disk by then.
  m_pgo = std::make_shared<SimplePGO>(m_pgo_config);

  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    // swap, not a pop() loop: pop() leaves the deque's capacity behind, and
    // every entry here drags a full body cloud with it.
    std::queue<CloudWithPose>().swap(m_state.cloud_buffer);
    m_state.last_message_time = -1.0;
  }
  m_key_pose_count.store(0);
  m_loop_count.store(0);

  publishEmptyMapCloud(now);
  publishLoopMarkerDeleteAll();
}

// The sequence a start and a reset share. Four phases, ordered so that the
// only step which can fail happens before anything is destroyed. The single
// reachable partial state is "paused, graph intact, LIO untouched", and every
// failure path exits through the guard that puts the previous phase back --
// so there is no half-reset for a caller to clean up, and no ordering
// exposed to a caller to get wrong.
bool PGONode::beginRun(
  bool reset_lio, uint8_t required_phase, std::string & message, double & last_odom_time,
  uint32_t & dropped)
{
  using syncai_common::msg::MappingStatus;
  const bool is_start = required_phase == MappingStatus::IDLE;
  const char * tag = is_start ? "startMappingCB" : "resetMappingCB";
  // What a failure leaves behind, in the caller's words: a reset keeps the
  // map it was about to discard; a start has started nothing.
  const char * kept = is_start ? "nothing started" : "map kept";
  last_odom_time = 0.0;
  dropped = 0;

  // ---- Phase 1: pause. Reversible, and the whole ordering fix. ----
  //
  // With the phase RESETTING, nothing the front end publishes can reach the
  // graph while pointlio's state changes underneath us. That is why this
  // design needs no sleep and no slack window: the odometry discontinuity
  // has nowhere to land, rather than landing somewhere we hope is harmless.
  if (m_transitioning.exchange(true)) {
    message = "A start or reset is already running";
    return false;
  }
  // RAII so every early return below -- including an exception out of the
  // client -- puts the previous phase back and clears the flag. `armed` is
  // what distinguishes "refused before anything changed" from "paused".
  struct ResumeGuard
  {
    PGONode * self;
    uint8_t prev = MappingStatus::IDLE;
    bool armed = false;
    bool committed = false;
    ~ResumeGuard()
    {
      if (armed && !committed) {
        self->m_state.phase.store(prev);
        self->publishStatus();
      }
      self->m_transitioning.store(false);
    }
  } resume_guard{this};

  {
    // Brief, and the one place the precondition is exact: saveMapsCB stores
    // IDLE under this same mutex at the end of a save, so a reset that was
    // racing it either sees MAPPING and waits behind the save, or sees IDLE
    // and is refused -- never "paused the run the save just ended".
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);
    resume_guard.prev = m_state.phase.load();
    if (resume_guard.prev != required_phase) {
      message = is_start
                  ? "Already mapping: use reset_mapping to start over, or save_maps to finish."
                  : "Not mapping (idle): nothing to discard. Call start_mapping to begin a run.";
      return false;
    }
    m_state.phase.store(MappingStatus::RESETTING);
    resume_guard.armed = true;
  }
  publishStatus();

  // ---- Phase 2: reset the front end. The only fallible step, no mutex. ----
  //
  // Deliberately outside m_pgo_mutex: this blocks for up to five seconds, and
  // holding the lock would stall the 50 ms timer -- and with it the
  // map -> local_frame TF broadcast -- for that whole time.
  if (reset_lio) {
    if (!m_lio_reset_cli->wait_for_service(std::chrono::seconds(2))) {
      message = "LIO reset service " + m_node_config.lio_reset_service + " is not available; " +
                kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }

    auto future = m_lio_reset_cli->async_send_request(
      std::make_shared<syncai_common::srv::ResetLIO::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      message = std::string("Timed out waiting for the LIO reset; ") + kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }

    auto lio_response = future.get();
    if (!lio_response->success) {
      message = "LIO refused the reset (" + lio_response->message + "); " + kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }
    last_odom_time = lio_response->last_odom_time;
  }

  // ---- Phase 3: rebuild the graph. Nothing below can fail. ----
  builtin_interfaces::msg::Time now = this->get_clock()->now();
  {
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);
    dropped = static_cast<uint32_t>(m_pgo->keyPoses().size());
    stopRunLocked(now);
    // The boundary the new run starts after. With a LIO reset it is the stamp
    // pointlio reported (exact: the old stream is <= it, the new one > it).
    // Without one, a start gates on the last pair seen -- the run begins at
    // the call -- while a reset keeps its historical 0.0, which lets the
    // synchroniser's held pairs through as it always has for replays.
    m_state.accept_after_time.store(
      reset_lio ? last_odom_time : (is_start ? m_state.last_seen_time.load() : 0.0));
    m_state.phase.store(MappingStatus::MAPPING);
    resume_guard.committed = true;
  }

  // ---- Phase 4: resume (the guard clears m_transitioning). ----
  publishStatus();
  return true;
}

// Begin a run: IDLE -> MAPPING. The same sequence as the reset from the
// other precondition; see StartMapping.srv for the contract.
void PGONode::startMappingCB(
  const std::shared_ptr<syncai_common::srv::StartMapping::Request> request,
  std::shared_ptr<syncai_common::srv::StartMapping::Response> response)
{
  std::string message;
  double last_odom_time = 0.0;
  uint32_t dropped = 0;
  const bool ok = beginRun(
    request->reset_lio, syncai_common::msg::MappingStatus::IDLE, message, last_odom_time, dropped);
  response->success = ok;
  response->lio_last_odom_time = last_odom_time;
  if (!ok) {
    response->message = message;
    return;
  }
  // Rendered verbatim by the operator console, so it is written as UI copy
  // rather than as a log line -- and it is the LAST place the stillness
  // warning can land: the static IMU initialisation happens in the seconds
  // after this returns.
  response->message =
    request->reset_lio
      ? "Mapping started. The map begins building once the lidar has re-levelled — keep "
        "the robot still until then."
      : "Mapping started over the running odometry; the map origin is where the LIO started.";
  RCLCPP_WARN(this->get_logger(), "[PGONode][startMappingCB] %s", response->message.c_str());
}

// Throw the pose graph away and start a new map, with nothing restarted:
// MAPPING -> MAPPING. Refused while IDLE (there is nothing to discard, and a
// LIO reset as the side effect of a no-op would be a surprise).
void PGONode::resetMappingCB(
  const std::shared_ptr<syncai_common::srv::ResetMapping::Request> request,
  std::shared_ptr<syncai_common::srv::ResetMapping::Response> response)
{
  std::string message;
  double last_odom_time = 0.0;
  uint32_t dropped = 0;
  const bool ok = beginRun(
    request->reset_lio, syncai_common::msg::MappingStatus::MAPPING, message, last_odom_time,
    dropped);
  response->success = ok;
  response->lio_last_odom_time = last_odom_time;
  response->dropped_key_poses = dropped;
  if (!ok) {
    response->message = message;
    return;
  }
  // UI copy, as in startMappingCB.
  response->message = request->reset_lio
                        ? "Map discarded. The new one starts building once the "
                          "lidar has re-levelled — keep the robot still until then."
                        : "Pose graph reset. The LIO front end was left running.";
  RCLCPP_WARN(
    this->get_logger(), "[PGONode][resetMappingCB] %s (dropped %u key poses)",
    response->message.c_str(), dropped);
}

// Serialise the run to disk and END it: MAPPING -> IDLE. The map lives in
// map.pcd from here on (the backend converts it from there), so the
// keyframes are freed, the empty map is published -- which is what clears
// /dev/shm and the console's "map so far" -- and the next run waits for
// start_mapping. A failed save changes nothing.
void PGONode::saveMapsCB(
  const std::shared_ptr<syncai_common::srv::SaveMaps::Request> request,
  std::shared_ptr<syncai_common::srv::SaveMaps::Response> response)
{
  using syncai_common::msg::MappingStatus;
  // Whole body, like timerCB: this iterates m_pgo->keyPoses() and writes a
  // multi-MB PCD out of it. A reset waiting behind a save is the correct
  // outcome -- the save is serialising exactly what the reset is about to
  // destroy, and letting them interleave would write a half-reset map. (It
  // then finds IDLE and is refused, which is also correct.)
  std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

  // Before any filesystem work. RESETTING is observable here: beginRun's
  // phase 2 waits on the LIO without the mutex.
  const uint8_t phase = m_state.phase.load();
  if (phase != MappingStatus::MAPPING) {
    response->success = false;
    response->message = phase == MappingStatus::IDLE
                          ? "Not mapping (idle): nothing to save. Call start_mapping to begin a run."
                          : "A start or reset is in progress; try again in a moment.";
    return;
  }

  if (!std::filesystem::exists(request->file_path)) {
    response->success = false;
    response->message = request->file_path + " IS NOT EXISTS!";
    return;
  }

  if (m_pgo->keyPoses().size() == 0) {
    response->success = false;
    response->message = "NO POSES!";
    return;
  }

  std::filesystem::path p_dir(request->file_path);
  std::filesystem::path patches_dir = p_dir / "patches";       // per-keyframe patch clouds
  std::filesystem::path poses_txt_path = p_dir / "poses.txt";  // per-frame pose list
  std::filesystem::path map_path = p_dir / "map.pcd";          // the merged full map

  // Everything that touches the disk. pcl throws on a write failure (a full
  // disk, a vanished bind mount) and a throw out of a service callback would
  // take the node down -- and, now that the run state hangs off a successful
  // save, leave it undefined. Caught, reported, and the run kept.
  try {
    if (request->save_patches) {
      if (std::filesystem::exists(patches_dir)) {
        std::filesystem::remove_all(patches_dir);
      }

      std::filesystem::create_directories(patches_dir);

      if (std::filesystem::exists(poses_txt_path)) {
        std::filesystem::remove(poses_txt_path);
      }
      // A clean of the previous save into this directory may still be
      // running; its input is being replaced. clean_map's own poses.txt
      // fingerprint is what stops it renaming the old map over the new one.
      stopMapCleanFor(p_dir);
      removeMapCleanOutputs(p_dir);
      RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

    // Opened only when patches are wanted: an unconditional open used to
    // truncate an existing poses.txt on a patch-less save.
    std::ofstream txt_file;
    if (request->save_patches) txt_file.open(poses_txt_path);

    CloudType::Ptr ret(new CloudType);
    for (size_t i = 0; i < m_pgo->keyPoses().size(); i++) {
      CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
      if (request->save_patches) {
        std::string patch_name = std::to_string(i) + ".pcd";
        std::filesystem::path patch_path = patches_dir / patch_name;
        pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
        Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
        V3D t = m_pgo->keyPoses()[i].t_global;
        txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w()
                 << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
      }
      CloudType::Ptr world_cloud(new CloudType);
      pcl::transformPointCloud(
        *body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global,
        Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));

      // Stack into the merged map
      *ret += *world_cloud;
    }
    txt_file.close();

    // Save the full map
    pcl::io::savePCDFileBinary(map_path.string(), *ret);
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("Save failed: ") + e.what() + "; the run is kept.";
    RCLCPP_ERROR(this->get_logger(), "[PGONode][saveMapsCB] %s", response->message.c_str());
    return;
  }

  // ---- The run is on disk: end it. ----
  const size_t saved = m_pgo->keyPoses().size();
  stopRunLocked(this->get_clock()->now());
  m_state.phase.store(MappingStatus::IDLE);
  publishStatus();

  response->success = true;
  // UI copy, rendered verbatim by the console.
  // After the run has ended, not before: the keyframes are already freed, so
  // their RAM is back before the clean starts, and nothing the spawn does can
  // disturb the transition -- a clean that cannot start is reported in the
  // message and the sidecar, never as a failed save.
  const std::string clean_note = startMapClean(p_dir, request->save_patches);
  response->message = "Map saved (" + std::to_string(saved) +
                      " keyframes). Mapping stopped — start it again for another map." + clean_note;
  RCLCPP_WARN(
    this->get_logger(), "[PGONode][saveMapsCB] %s (%s)", response->message.c_str(),
    map_path.string().c_str());
}

// ---- The post-save map cleaning ---------------------------------------------
//
// clean_map rewrites map.pcd without people and one-off returns (see
// map_cleaner.hpp for what and why). A child PROCESS, not a fourth thread,
// for three reasons that each decide it alone:
//   - Memory. A clean holds a hash map of every hit voxel of the site --
//     millions of entries, hundreds of MB. In this process that heap would
//     mostly never go back to the OS, and the next run's keyframes would
//     compete with it; a process returns all of it at exit. It also writes
//     oom_score_adj 500, so if anything is killed for memory it is the clean,
//     not the node that owns the TF and possibly an unsaved run.
//   - Lifetime. switch_mode / restart_mode kill the byobu session, which
//     hangs up the pane's pty: SIGHUP, no destructor, every thread in here
//     gone. Saving and switching straight back to AUTO is the normal operator
//     flow, so the clean has to outlive this process -- POSIX_SPAWN_SETSID
//     puts it in its own session with no controlling terminal.
//   - One entry point. The same executable is the by-hand re-clean of any
//     saved map, so the save path is exactly what a developer reruns.
// posix_spawn rather than fork(): this process has four threads, and a
// fork()ed copy of a multithreaded process may only call async-signal-safe
// functions before exec.
std::string PGONode::startMapClean(const std::filesystem::path & map_dir, bool have_patches)
{
  if (!m_node_config.map_clean_enabled) return "";
  // UI copy, appended to the save's message (rendered verbatim by the
  // console, which never names its internals).
  if (!have_patches) {
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][map_clean] save_patches was false: nothing to ray-cast, %s stays uncleaned",
      map_dir.c_str());
    return " People were not removed from this map: that needs the per-scan data this save "
           "did not keep.";
  }
  const auto & oc = m_node_config.map_clean;
  const std::string started_at = map_clean_recipe::isoUtcNow();
  // Before the spawn, synchronously: the backend acts on this response, and
  // the sidecar must already say "converting" when it looks.
  try {
    map_clean_recipe::writeAtomic(map_dir, map_clean_recipe::converting(oc, started_at));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode][map_clean] %s in %s; no clean", e.what(), map_dir.c_str());
    return " Removing people from the map could not be started; the saved map is unaffected.";
  }

  std::string exe;
  try {
    exe = ament_index_cpp::get_package_prefix("syncai_mapping") + "/lib/syncai_mapping/clean_map";
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return failMapClean(map_dir, started_at, "clean_map executable not found");
  }
  if (access(exe.c_str(), X_OK) != 0) {
    return failMapClean(map_dir, started_at, "clean_map executable not found");
  }

  auto num = [](double v) {
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os << std::setprecision(12) << v;
    return os.str();
  };
  std::vector<std::string> args = {
    exe,
    map_dir.string(),
    "--resolution",
    num(oc.resolution),
    "--max-range",
    num(oc.max_range),
    "--min-range",
    num(oc.min_range),
    "--lidar-height",
    num(oc.lidar_height),
    "--floor-band",
    num(oc.floor_band),
    "--max-height",
    num(oc.max_height),
    "--floor-radius",
    num(oc.floor_radius),
    "--min-hits",
    std::to_string(oc.min_hits),
    "--dynamic-miss-ratio",
    num(oc.dynamic_miss_ratio),
    "--dynamic-min-miss",
    std::to_string(oc.dynamic_min_miss),
    "--clip-below-floor",
    oc.clip_below_floor ? "1" : "0",
    "--nice",
    std::to_string(m_node_config.map_clean_nice),
    "--started-at",
    started_at};
  std::vector<char *> argv;
  for (auto & a : args) argv.push_back(a.data());
  argv.push_back(nullptr);

  // exec already resets handled signals; SETSIGDEF also undoes any that are
  // IGNORED here (an ignored SIGCHLD would make the reaper's waitpid fail),
  // and SETSIGMASK clears whatever this thread happens to block.
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  sigset_t defaults, none;
  sigemptyset(&defaults);
  for (const int sig : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGPIPE, SIGCHLD}) {
    sigaddset(&defaults, sig);
  }
  sigemptyset(&none);
  posix_spawnattr_setsigdefault(&attr, &defaults);
  posix_spawnattr_setsigmask(&attr, &none);
  posix_spawnattr_setflags(
    &attr, POSIX_SPAWN_SETSID | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
  pid_t pid = 0;
  // stdout / stderr are inherited: the clean's progress lands in this pane's
  // log (log/stack/<robot_id>/mapping/pgo/) for as long as the pane lives.
  const int rc = posix_spawn(&pid, exe.c_str(), nullptr, &attr, argv.data(), environ);
  posix_spawnattr_destroy(&attr);
  if (rc != 0) {
    return failMapClean(
      map_dir, started_at, std::string("cannot start clean_map: ") + std::strerror(rc));
  }
  m_map_clean_children.push_back({pid, map_dir, std::chrono::steady_clock::now()});
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode][map_clean] clean_map pid %d started for %s (%.3f m, nice %d); outcome in %s", pid,
    map_dir.c_str(), oc.resolution, m_node_config.map_clean_nice, map_clean_recipe::kSidecarFile);
  return " People and stray points are being removed from the map in the background; it can "
         "take a few minutes.";
}

std::string PGONode::failMapClean(
  const std::filesystem::path & map_dir, const std::string & started_at, const std::string & error)
{
  RCLCPP_ERROR(
    this->get_logger(), "[PGONode][map_clean] %s (for %s)", error.c_str(), map_dir.c_str());
  try {
    map_clean_recipe::writeAtomic(
      map_dir, map_clean_recipe::failed(
                 m_node_config.map_clean, started_at, map_clean_recipe::isoUtcNow(), error));
  } catch (const std::exception & e) {
    // The "converting" written a moment ago stays; a reader ages it out.
    RCLCPP_ERROR(this->get_logger(), "[PGONode][map_clean] %s", e.what());
  }
  return " Removing people from the map could not be started; the saved map is unaffected.";
}

void PGONode::reapMapCleans()
{
  for (auto it = m_map_clean_children.begin(); it != m_map_clean_children.end();) {
    int st = 0;
    const pid_t r = waitpid(it->pid, &st, WNOHANG);
    if (r == 0) {
      ++it;
      continue;
    }
    const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - it->started).count();
    if (r < 0) {
      // ECHILD: only if something set SIGCHLD to SIG_IGN in this process,
      // which reaps children behind our back. The sidecar still has it all.
      RCLCPP_WARN(
        this->get_logger(),
        "[PGONode][map_clean] cannot wait for clean_map pid %d (%s); see %s in %s", it->pid,
        std::strerror(errno), map_clean_recipe::kSidecarFile, it->dir.c_str());
    } else if (WIFEXITED(st) && WEXITSTATUS(st) == 0) {
      RCLCPP_INFO(
        this->get_logger(), "[PGONode][map_clean] clean_map pid %d finished %s in %.0f s", it->pid,
        it->dir.c_str(), secs);
    } else if (WIFEXITED(st)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[PGONode][map_clean] clean_map pid %d exited %d after %.0f s; %s in %s has the reason",
        it->pid, WEXITSTATUS(st), secs, map_clean_recipe::kSidecarFile, it->dir.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[PGONode][map_clean] clean_map pid %d killed by signal %d after %.0f s (OOM?); %s in "
        "%s is left at converting",
        it->pid, WIFSIGNALED(st) ? WTERMSIG(st) : 0, secs, map_clean_recipe::kSidecarFile,
        it->dir.c_str());
    }
    it = m_map_clean_children.erase(it);
  }
}

void PGONode::stopMapCleanFor(const std::filesystem::path & map_dir)
{
  std::error_code ec;
  const auto target = std::filesystem::weakly_canonical(map_dir, ec);
  for (const auto & child : m_map_clean_children) {
    std::error_code cec;
    if (std::filesystem::weakly_canonical(child.dir, cec) != target) continue;
    // Only a child not yet reaped is in the list, so the pid is still ours
    // (a zombie at worst) and cannot have been recycled.
    if (kill(child.pid, SIGTERM) == 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PGONode][map_clean] a new save into %s: stopping clean_map pid %d, which was "
        "cleaning the previous one",
        map_dir.c_str(), child.pid);
    }
  }
}

void PGONode::removeMapCleanOutputs(const std::filesystem::path & map_dir)
{
  std::error_code ec;
  std::filesystem::remove(map_dir / map_clean_recipe::kSidecarFile, ec);
  // The cleaned map.pcd a killed clean may have left half-written. map.pcd
  // itself is the save's own output, written right after this.
  std::filesystem::remove(map_dir / (std::string(map_cleaner::kMapFile) + ".tmp"), ec);
  // And the sidecar's per-process `map_clean.recipe.json.<pid>.tmp`.
  const std::string prefix = std::string(map_clean_recipe::kSidecarFile) + ".";
  for (const auto & entry : std::filesystem::directory_iterator(map_dir, ec)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0 && entry.path().extension() == ".tmp") {
      std::error_code rm_ec;
      std::filesystem::remove(entry.path(), rm_ec);
    }
  }
}

}  // namespace syncai_mapping
