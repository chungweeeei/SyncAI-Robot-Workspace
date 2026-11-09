#ifndef SYNCAI_POINTLIO__POINTLIO_NODE_HPP_
#define SYNCAI_POINTLIO__POINTLIO_NODE_HPP_

// The ROS shell around Point-LIO. Everything numeric lives in map_builder/
// (point_ekf, imu_initializer, lidar_processor, ivox_map); this class owns the
// subscriptions, the sync of a lidar frame with the IMU samples covering it,
// the publishers / TF broadcaster, and the in-place `reset` service.
//
// Ported into the workspace from SyncAI-Fast-LIO2's `pointlio` package in
// 2026-09 with the ROS surface unchanged: node name `pointlio_node`, namespace
// /<robot_id>/pointlio (set by the launch), topics lio_odom / body_cloud /
// world_cloud (lio_path, an accumulated trajectory nothing subscribed to, was
// dropped on 2026-10-28), service `reset`, TF world_frame -> body_frame. The
// only rename is the service *type*, which moved from that repo's `interface`
// package to syncai_common when server and client stopped sharing a repo.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "syncai_common/srv/reset_lio.hpp"
#include "syncai_pointlio/map_builder/commons.h"
#include "syncai_pointlio/map_builder/map_builder.h"
#include "syncai_pointlio/utils.h"

namespace syncai_pointlio
{

// Input topics are NOT parameters: the node subscribes to the relative names
// "lidar" and "imu" and the launch file remaps them onto the driver's
// /<robot_id>/livox/{lidar,imu} (a relative name alone cannot reach them —
// the node runs in the /<robot_id>/pointlio namespace, so "lidar" would resolve
// to /<robot_id>/pointlio/lidar). Remapping is the ROS-native knob for this and
// keeps `ros2 node info` honest about where the data comes from.
struct NodeConfig
{
  std::string body_frame = "body";
  std::string world_frame = "lidar";
  bool print_time_cost = false;
  int lidar_type = 0;
  // 0: livox_ros_driver2 CustomMsg, 1: sensor_msgs/PointCloud2 (e.g. Isaac Sim)
  // Scale applied to linear_acceleration. Livox IMU reports g-units -> ~10.0 to get m/s^2.
  // Sensors already in m/s^2 (e.g. Isaac Sim, z~9.81 at rest) must use 1.0, since the
  // filter's gravity magnitude is fixed at 9.81 (point_ekf.cpp). satu_acc is compared
  // against the scaled value.
  double imu_acc_scale = 10.0;
  // Decimation of the extra body_cloud_dense output, for mapping only (pgo).
  // The filter and body_cloud keep lidar_filter_num; this one is taken from
  // the same scan in the same pass, so it costs nothing while nobody
  // subscribes. 2 is the floor: at 1 a MID360 scan is ~20k points, ~320 KB as
  // x/y/z/intensity, over CycloneDDS's default 208 KB receive buffer on lo
  // (see CLAUDE.md, "The live merge is a file"). <= 0 or >= lidar_filter_num:
  // body_cloud_dense carries body_cloud's points.
  int dense_filter_num = 2;
  // The scan backlog guard (2026-10-28). timerCB takes one scan per 20 ms
  // tick and lidar_buffer has no bound, so once a scan costs more than the
  // lidar's 100 ms the node falls behind for good: on robot01 on 2026-10-28,
  // with the 8 online cores ~90 % busy, Point-LIO ran at ~0.65x real time,
  // lio_odom / body_cloud fell more than 5 minutes behind the lidar's own
  // stamps, the buffers grew ~125 MB a minute, and every consumer (localizer,
  // lio_bridge, the costmaps) worked on minutes-old data with no error
  // anywhere. Restarting the node was the only way out.
  //
  // When the buffered scans span more than max_lidar_backlog seconds,
  // syncPackage() skips the oldest before taking the next, at most
  // max_lidar_skip in a row. A skipped scan loses its points only: its IMU
  // samples stay in imu_buffer and go into the next package, so the filter
  // predicts and IMU-updates straight through the gap (~0.2 s at 2). It then
  // catches up whenever it runs faster than 1 / (max_lidar_skip + 1) of real
  // time; slower than that the backlog still grows, and the WARN says so.
  // Never one long skip down to the newest scan: seconds of IMU-only
  // propagation on a walking robot would put the next scan's points on the
  // wrong planes, and a diverged solution is worse than a late one. 0.3 s is
  // three scans of slack before anything is skipped. <= 0 disables the guard
  // (every scan processed however late: an offline replay that wants them
  // all).
  double max_lidar_backlog = 0.3;
  int max_lidar_skip = 2;

  // The health log (2026-11-02). On 2026-10-30 robot01's Point-LIO diverged
  // on the dock after hours of slow degradation, and its own log said nothing
  // until the numbers were already 10^6 m: the decline could only be read off
  // the localizer's corrections, and why it happened not at all. Now one
  // `lio health:` line per health_summary_period: how many scans, how well
  // their points matched the map (near / effective / residual), the IMU's
  // saturation, the speed and the covariance -- the numbers an incident review
  // needs, at a resolution that shows a decline building. It is a WARN, with
  // every reason that holds, when
  //  - the speed exceeds health_max_speed anywhere in the window. The robot
  //    tops out at 0.8 m/s; the localizer's health.max_lio_speed (1.5) is the
  //    same check from outside, which stops running once its rounds fail;
  //  - an IMU sample had an axis at or over satu_acc / satu_gyro (dropped
  //    from the update, PointEKF::updateIMU);
  //  - a scan's lidar update had no effective point at all (IMU only);
  //  - health_min_effective_ratio > 0 and a scan's effective / points fell
  //    under it. Off by default: what the ratio is on a healthy robot has not
  //    been measured yet, and a made-up floor would only cry wolf.
  // Log only: nothing here changes the filter or what is published.
  // health_summary_period <= 0 turns the whole log off.
  double health_summary_period = 1.0;
  double health_max_speed = 1.5;
  double health_min_effective_ratio = 0.0;
};

// The health log's tally of one summary window (NodeConfig::health_*), from
// MapBuilder::health() after every MAPPING scan. timerCB / resetCB only.
struct HealthWindow
{
  int scans = 0;
  long points = 0;
  long near = 0;
  long effective = 0;
  double residual_sum = 0.0;
  double effective_ratio_min = 1.0;
  int chunks = 0;
  int chunks_updated = 0;
  int scans_without_update = 0;
  int imu_samples = 0;
  int acc_saturated = 0;
  int gyro_saturated = 0;
  double acc_abs_max = 0.0;
  double gyro_abs_max = 0.0;
  double speed_max = 0.0;
  double cost_ms_sum = 0.0;
  double cost_ms_max = 0.0;
};

struct StateData
{
  bool lidar_pushed = false;

  std::mutex imu_mutex;
  std::mutex lidar_mutex;

  double last_lidar_time = -1.0;
  double last_imu_time = -1.0;

  std::deque<IMUData> imu_buffer;
  std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
  // The dense cloud of each lidar_buffer entry, same index; pushed, popped and
  // cleared together with it.
  std::deque<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> lidar_dense_buffer;
};

class PointLIONode : public rclcpp::Node
{
public:
  PointLIONode();

  // Declares every parameter with the struct default as its default. See the
  // definition for why these are ROS parameters and not a hand-parsed YAML.
  void loadParameters();

  // Subscription callbacks: buffer only, no processing. They run in
  // m_sensor_group, concurrently with timerCB / resetCB, so every touch of
  // StateData's buffers holds that buffer's mutex.
  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void pc2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Assembles m_package from one lidar frame plus the IMU samples that cover
  // it; false while the IMU has not caught up to the end of the scan.
  bool syncPackage();

  void publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
    std::string frame_id, const double & time);
  // x / y / z / intensity only, 16 B a point: pcl::toROSMsg would write the
  // whole PointXYZINormal (48 B), three times the bytes for fields pgo drops.
  void publishCompactCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, const CloudType & cloud,
    const std::string & frame_id, const double & time);
  void publishOdometry(
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
    std::string child_frame, const double & time);
  void broadCastTF(
    std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
    std::string child_frame, const double & time);

  // 20 ms wall timer: sync, process, publish.
  void timerCB();
  // The backlog guard's report, every few seconds from timerCB: a WARN while
  // scans are being skipped, an INFO once they no longer are.
  void reportBacklog();
  // The health log (NodeConfig::health_*): one MAPPING scan into the window,
  // and the `lio health:` line once the window is over, every timerCB tick.
  void recordHealth(double cost_ms);
  void reportHealth();
  // The publish half of timerCB, once the builder is MAPPING: TF, odometry,
  // the deskewed clouds, all for m_package.
  void publishResults();

  // The `reset` service: throw the whole solution away and start over, in
  // place. Takes no lock against timerCB: it relies on sharing the default
  // callback group with it (see main.cpp and the definition's comment).
  void resetCB(
    const std::shared_ptr<syncai_common::srv::ResetLIO::Request> request,
    std::shared_ptr<syncai_common::srv::ResetLIO::Response> response);

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_dense_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

  // The lidar / IMU subscriptions only; timer and reset stay in the default
  // group. See the constructor.
  rclcpp::CallbackGroup::SharedPtr m_sensor_group;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Service<syncai_common::srv::ResetLIO>::SharedPtr m_reset_srv;
  // Stamp of the last odometry sample this node published, in the lidar's clock
  // (the same value that goes into lio_odom's and body_cloud's headers). Kept
  // only so resetCB can hand it to pgo as the boundary between the two runs;
  // 0.0 until MAPPING is first reached, and again after every reset.
  double m_last_odom_time = 0.0;
  // The backlog guard's bookkeeping (NodeConfig::max_lidar_backlog), touched
  // only from timerCB (syncPackage runs inside it): scans skipped and the
  // widest backlog seen since the last report, when that report went out, and
  // whether it was a WARN still waiting for its all-clear.
  int m_backlog_skipped = 0;
  double m_backlog_max_seen = 0.0;
  std::chrono::steady_clock::time_point m_last_backlog_report = std::chrono::steady_clock::now();
  bool m_backlog_warned = false;
  // The health log's window, when it started, whether a scan has been
  // processed in MAPPING since start / the last reset (the no-scan WARN means
  // nothing before that), and whether that WARN is out. timerCB / resetCB
  // only.
  HealthWindow m_health;
  std::chrono::steady_clock::time_point m_last_health_report = std::chrono::steady_clock::now();
  bool m_health_seen_mapping = false;
  bool m_health_idle_warned = false;
  StateData m_state_data;
  SyncPackage m_package;
  NodeConfig m_node_config;
  Config m_builder_config;
  std::shared_ptr<PointEKF> m_kf;
  std::shared_ptr<MapBuilder> m_builder;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

}  // namespace syncai_pointlio

#endif  // SYNCAI_POINTLIO__POINTLIO_NODE_HPP_
