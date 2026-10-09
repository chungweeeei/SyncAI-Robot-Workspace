#ifndef SYNCAI_MAPPING__PGO_NODE_HPP_
#define SYNCAI_MAPPING__PGO_NODE_HPP_

// The ROS shell around the pose graph. The graph itself (keyframe selection,
// radius-search loop detection with ICP verification, GTSAM iSAM2 smoothing)
// is pgos/simple_pgo.*; this class owns the synchronised cloud+odom intake,
// the 50 ms timer that feeds it and broadcasts map -> local_frame, the
// "map so far" merge worker and its file hand-off to the operator console's
// backend, and the three services that bracket a mapping run: start_mapping
// and reset_mapping (both drive syncai_pointlio's reset through a client) and
// save_maps, which ends the run. The run state (idle / mapping / resetting)
// is latched on mapping_status for the backend.
//
// Ported into the workspace from SyncAI-Fast-LIO2's `pgo` package in 2026-09
// with the ROS surface unchanged: node name `pgo_node`, namespace
// /<robot_id>/pgo (set by the launch), services save_maps / reset_mapping,
// topics map_cloud / map_cloud_file / loop_markers, TF map -> local_frame, the
// /dev/shm/syncai_pgo/<robot_id> hand-off directory and the save_maps on-disk
// layout. Two things did change with the port: the service *types* moved from
// that repo's `interface` package to syncai_common (the backend, their only
// caller, builds against that), and configuration became declared ROS
// parameters instead of a yaml-cpp-parsed `config_path`.

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "syncai_common/msg/mapping_status.hpp"
#include "syncai_common/srv/reset_lio.hpp"
#include "syncai_common/srv/reset_mapping.hpp"
#include "syncai_common/srv/save_maps.hpp"
#include "syncai_common/srv/start_mapping.hpp"
#include "syncai_mapping/octomap_builder.hpp"
#include "syncai_mapping/pgos/commons.h"
#include "syncai_mapping/pgos/simple_pgo.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace syncai_mapping
{

struct NodeConfig
{
  // The two inputs and the LIO reset service live in syncai_pointlio's
  // namespace, which a relative name from inside /<robot_id>/pgo cannot
  // reach, so all three are absolute and the launch overrides them per
  // robot_id. These are the bare-run fallbacks and must never carry one.
  std::string cloud_topic = "/pointlio/body_cloud";
  std::string odom_topic = "/pointlio/lio_odom";
  std::string map_frame = "map";
  std::string local_frame = "lidar";
  // The published "map so far" merge (see publishMapCloud). Node-level publish
  // behaviour, not PGO math, so they live here rather than in Config.
  double map_cloud_resolution = 0.2;
  double map_cloud_pub_period = 3.0;
  // Where the merge is handed to the operator console's backend as a FILE
  // (see mergeAndPublishMapCloud). A tmpfs the backend's container shares
  // (`ipc: host` on both compose services -- a private /dev/shm is 64 MB, too
  // small for two merges of a large site). mapping.launch.py appends
  // /<robot_id> so two robots on one host never share a directory.
  std::string map_cloud_dir = "/dev/shm/syncai_pgo";
  // pointlio's reset service. Absolute, and overridden per robot_id by
  // mapping.launch.py for exactly the reason cloud_topic and odom_topic are:
  // it names something in pointlio's namespace, which a relative name from
  // inside /<robot_id>/pgo cannot reach. Keeping it a parameter is what stops
  // this file from ever spelling a robot_id.
  std::string lio_reset_service = "/pointlio/reset";
  // Come up MAPPING instead of IDLE (2026-10, when the idle state arrived).
  // The escape hatch for bag replays and for a backend without a Start
  // control: with it false -- the default, and what the mapping session
  // wants -- nothing reaches the graph until start_mapping is called.
  bool start_on_launch = false;
  // The post-save OctoMap build (see startOctomapBuild): whether a successful
  // save_maps spawns it, the nice value it runs at, and the build parameters
  // it is handed on its command line. octomap_builder::Params carries the
  // defaults and the why of each.
  bool octomap_enabled = true;
  int octomap_nice = 10;
  octomap_builder::Params octomap;
};

struct NodeState
{
  std::mutex message_mutex;
  std::queue<CloudWithPose> cloud_buffer;
  // Was uninitialised, so the out-of-order guard in syncCB compared the very
  // first message against whatever was on the stack. -1.0 is the "no message
  // seen yet" value, and is also what resetMappingCB puts back.
  double last_message_time = -1.0;

  // The intake gate. Atomics rather than fields under message_mutex on purpose:
  // syncCB would otherwise have to take m_pgo_mutex *and* message_mutex in a
  // fixed order on its hot path, and this file has already proved it cannot be
  // trusted with one lock (see the lock_guard note in syncCB). Relaxed loads
  // keep the gate lock-free and remove lock ordering from the design entirely.
  //
  // phase is the run state, one of syncai_common::msg::MappingStatus's
  // constants, and it REPLACED a bool `accepting` when the idle state arrived
  // (2026-10): "accepting" was exactly "phase == MAPPING", and two flags that
  // can disagree are the bug a single one cannot have. Only MAPPING lets a
  // pair into the buffer. IDLE is where a mapping session starts and where a
  // successful save_maps returns to; RESETTING is the window inside a
  // start_mapping / reset_mapping while pointlio's state is changing, which
  // is what makes those transitions ordering-proof rather than
  // timing-dependent: nothing the front end publishes can reach the graph.
  // Every store happens under m_pgo_mutex; the loads here do not need it.
  //
  // accept_after_time then discards the old run's tail: pairs already held by
  // the message_filters synchroniser, which surface after the phase goes back
  // to MAPPING. Both sides of that comparison are the lidar header stamp, so
  // it is exact -- no clock conversion, no tolerance constant. last_seen_time
  // is the stamp of the last pair syncCB saw, accepted or not; a
  // start_mapping with reset_lio false has no LIO boundary to gate on and
  // uses it instead, so only pairs produced after the call are taken.
  std::atomic<uint8_t> phase{syncai_common::msg::MappingStatus::IDLE};
  std::atomic<double> accept_after_time{-1.0};
  std::atomic<double> last_seen_time{-1.0};
};

class PGONode : public rclcpp::Node
{
public:
  PGONode();
  ~PGONode();

  // ---- The file hand-off (see the map_cloud_file publisher) ----------------
  void setupMapCloudDir();
  static bool isMapCloudFile(const std::filesystem::directory_entry & entry);
  void clearMapCloudDir();
  void pruneMapCloudFiles();
  std::string mapCloudNoticeJson(
    uint64_t seq, const std::string & path, size_t points,
    const builtin_interfaces::msg::Time & time) const;
  void publishMapCloudNotice(
    uint64_t seq, const std::string & path, size_t points,
    const builtin_interfaces::msg::Time & time);
  void writeAndAnnounceMapCloud(
    const CloudType & merged, uint64_t seq, const builtin_interfaces::msg::Time & time);

  // Declares every parameter with the struct default as its default. See the
  // definition for why these are ROS parameters and not a hand-parsed YAML.
  void loadParameters();

  // Intake: one synchronised cloud+odom pair, gated by the run state.
  void syncCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);

  void sendBroadCastTF(builtin_interfaces::msg::Time & time);
  // The IDLE stand-in for sendBroadCastTF: identity map -> local_frame, read
  // from nothing (so it needs no m_pgo_mutex). The backend's live-scan
  // subscriber looks that transform up and drops every frame it cannot find,
  // so an idle pgo that simply fell silent would blank the console exactly
  // while the operator is positioning the robot before Start.
  void sendIdentityTF(const builtin_interfaces::msg::Time & time);
  void publishLoopMarkers(builtin_interfaces::msg::Time & time);

  // The latched run state (see the mapping_status publisher). Lock-free --
  // it reads phase and the two counter mirrors -- so it can be called from
  // anywhere, including the RAII guard inside beginRun and the 1 s timer.
  void publishStatus();
  void statusTimerCB();

  // 50 ms timer: keyframe selection, loop search, smoothing, TF, outputs.
  void timerCB();

  // The "map so far" merge: claimed on the timer thread, run on the worker.
  void publishMapCloud(builtin_interfaces::msg::Time & time);
  void mergeAndPublishMapCloud(
    std::vector<KeyPoseWithCloud> snapshot, builtin_interfaces::msg::Time time, uint64_t seq);
  void publishEmptyMapCloud(const builtin_interfaces::msg::Time & time);
  void publishLoopMarkerDeleteAll();

  // The three services. start_mapping and reset_mapping share one callback
  // group (MutuallyExclusive, so they serialise) and block on the ResetLIO
  // client; save_maps shares the timer's group.
  void startMappingCB(
    const std::shared_ptr<syncai_common::srv::StartMapping::Request> request,
    std::shared_ptr<syncai_common::srv::StartMapping::Response> response);
  void resetMappingCB(
    const std::shared_ptr<syncai_common::srv::ResetMapping::Request> request,
    std::shared_ptr<syncai_common::srv::ResetMapping::Response> response);
  void saveMapsCB(
    const std::shared_ptr<syncai_common::srv::SaveMaps::Request> request,
    std::shared_ptr<syncai_common::srv::SaveMaps::Response> response);

  // ---- The post-save OctoMap build ------------------------------------------
  // Spawn `build_octomap <map_dir>` detached, after the save has ended the run.
  // Returns the sentence appended to the save's response message ("" when
  // octomap_enabled is false). Never fails the save: the map is on disk.
  std::string startOctomapBuild(const std::filesystem::path & map_dir, bool have_patches);
  // Record a build that could not start in the sidecar; returns the sentence.
  std::string failOctomapBuild(
    const std::filesystem::path & map_dir, const std::string & started_at,
    const std::string & error);
  // waitpid(WNOHANG) every spawned build; one log line per outcome. 1 Hz.
  void reapOctomapBuilds();
  // SIGTERM any build still running on map_dir, before a new save replaces
  // its input. Not waited for (it stops at its next keyframe, or later in the
  // layer stage); the reaper logs it, and the build's own poses.txt
  // fingerprint stops it from renaming anything over the new save.
  void stopOctomapBuildFor(const std::filesystem::path & map_dir);
  // The OctoMap outputs of an earlier save into the same directory, and their
  // .tmp files (map.pcd.tmp included): their inputs (patches/, poses.txt) are
  // about to be replaced. map.pcd itself is the save's own output, not this.
  static void removeOctomapOutputs(const std::filesystem::path & map_dir);

private:
  // The four-phase sequence start_mapping and reset_mapping share (pause,
  // reset the LIO, rebuild the graph, resume). They differ only in the phase
  // they require on entry -- IDLE for a start, MAPPING for a reset -- and in
  // the words of their messages. Returns false with `message` filled when
  // refused or when the LIO round trip failed; the state is then exactly
  // what it was. On success the node is MAPPING.
  bool beginRun(
    bool reset_lio, uint8_t required_phase, std::string & message, double & last_odom_time,
    uint32_t & dropped);
  // Everything that turns "a run" into "no run", for the caller that already
  // holds m_pgo_mutex: join the merge worker, replace SimplePGO, empty the
  // buffer and the counters, publish the empty map (which clears /dev/shm)
  // and delete the loop markers. Does NOT touch phase -- the caller stores
  // IDLE (save) or MAPPING (start / reset) itself.
  void stopRunLocked(const builtin_interfaces::msg::Time & now);

  NodeConfig m_node_config;
  Config m_pgo_config;
  NodeState m_state;
  std::shared_ptr<SimplePGO> m_pgo;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
  // The map-cloud worker. busy is claimed by publishMapCloud on the timer
  // thread and released by mergeAndPublishMapCloud as its last act; the thread
  // handle is only ever joined while busy is false (or in the destructor).
  std::atomic<bool> m_map_cloud_busy{false};
  double m_last_map_cloud_time = 0.0;
  std::thread m_map_cloud_thread;
  // The file hand-off (see the map_cloud_file publisher and
  // writeAndAnnounceMapCloud). m_map_cloud_seq is only ever touched under
  // m_pgo_mutex (publishMapCloud on the timer thread, publishEmptyMapCloud
  // from the reset); the worker gets its value by copy. dir_ok false means
  // the directory could not be created and the file output is off for this
  // run -- the PointCloud2 output is unaffected.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_map_cloud_file_pub;
  uint64_t m_map_cloud_seq = 0;
  std::string m_map_cloud_dir;
  bool m_map_cloud_dir_ok = false;
  rclcpp::Service<syncai_common::srv::SaveMaps>::SharedPtr m_save_map_srv;
  // Builds spawned by saveMapsCB and not yet reaped. Touched only by
  // saveMapsCB, statusTimerCB (the reaper) and the destructor; the first two
  // share the default MutuallyExclusive group, so -- like
  // m_last_map_cloud_time -- it needs no lock. A child outliving this process
  // is the design (see startOctomapBuild), so the destructor only logs them.
  struct OctomapChild
  {
    pid_t pid;
    std::filesystem::path dir;
    std::chrono::steady_clock::time_point started;
  };
  std::vector<OctomapChild> m_octomap_children;
  // The run state for consumers (see the publisher's comment in the
  // constructor) and the two counters it reports. The counters are mirrors
  // of m_pgo->keyPoses().size() / historyPairs().size(), written where those
  // change (under m_pgo_mutex) so that publishStatus never needs the lock.
  rclcpp::Publisher<syncai_common::msg::MappingStatus>::SharedPtr m_status_pub;
  rclcpp::TimerBase::SharedPtr m_status_timer;
  std::atomic<uint32_t> m_key_pose_count{0};
  std::atomic<uint32_t> m_loop_count{0};
  // The transition surface. m_transitioning rejects a second concurrent
  // start/reset outright rather than queueing it -- two in flight would have
  // the second one reading a boundary timestamp the first had already
  // invalidated. (The two handlers share a MutuallyExclusive group, so this
  // is belt and braces, as it was when only the reset existed.)
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::CallbackGroup::SharedPtr m_cli_cb_group;
  rclcpp::Service<syncai_common::srv::StartMapping>::SharedPtr m_start_srv;
  rclcpp::Service<syncai_common::srv::ResetMapping>::SharedPtr m_reset_srv;
  rclcpp::Client<syncai_common::srv::ResetLIO>::SharedPtr m_lio_reset_cli;
  std::atomic<bool> m_transitioning{false};
  // Guards m_pgo -- which beginRun and stopRunLocked REPLACE rather than
  // mutate, so every reader needs to be excluded, not just every writer.
  // Discipline:
  //
  //   timerCB          whole body (addKeyPose, searchForLoopPairs,
  //                    smoothAndUpdate, the TF broadcast, and the
  //                    m_map_cloud_busy / m_map_cloud_thread handshake)
  //   saveMapsCB       whole body (it serialises what a reset would destroy,
  //                    and the save -> IDLE transition at its end)
  //   beginRun         phase 1 (a few lines: the precondition check and the
  //                    RESETTING store, so a save finishing concurrently
  //                    cannot be overtaken) and phase 3 -- NEVER while
  //                    waiting on the LIO future, which would stall the TF
  //                    broadcast for seconds
  //
  // Every store to m_state.phase happens under it, which is what makes the
  // precondition checks exact.
  //
  // Needed only because main() runs a MultiThreadedExecutor now; under the old
  // rclcpp::spin() the executor itself provided this exclusion.
  std::mutex m_pgo_mutex;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
};

}  // namespace syncai_mapping

#endif  // SYNCAI_MAPPING__PGO_NODE_HPP_
