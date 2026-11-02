#include "syncai_pointlio/pointlio_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace syncai_pointlio
{

namespace
{
// See imuCB.
constexpr double kImuGapWarnSec = 0.030;
// See reportBacklog: one line per window, not one per skipped scan.
constexpr double kBacklogReportSec = 5.0;
}  // namespace

PointLIONode::PointLIONode() : Node("pointlio_node")
{
  RCLCPP_INFO(this->get_logger(), "[PointLIONode][%s] Point-LIO Node Started", __func__);
  loadParameters();

  // The sensor callbacks get a callback group of their own, and main() runs
  // a two-thread MultiThreadedExecutor, so ingest never waits for timerCB.
  // Until 2026-10 everything shared the default group under a single-threaded
  // spin(): imuCB could not run while timerCB was busy, the IMU reader held
  // 10 samples (50 ms at 200 Hz), and any timerCB longer than that (an
  // ikd-tree rebuild, the two deskews, a big toROSMsg) let the reader history
  // overwrite its oldest samples -- silently; RELIABLE does not help, the drop
  // is the reader's own. The filter then predicted across the hole with no
  // measurement and drifted with no error anywhere. Suspected replaying
  // dp1f_1012, whose bag itself is gap-free (200 Hz, no hole past 23 ms).
  //
  // MutuallyExclusive, not Reentrant: with two threads, a reentrant sensor
  // group could take both and leave the timer waiting -- the inverse of the
  // bug. timerCB and resetCB stay in the default group, so they still exclude
  // each other; see resetCB for why that is load-bearing.
  m_sensor_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sensor_options;
  sensor_options.callback_group = m_sensor_group;

  // register imu subscriber. Depth 2000 (~10 s at the MID360's 200 Hz) is the
  // backstop now, not the fix: the sensor group has its own thread, but
  // lidarCB's conversion runs in the same group, and a starved reader should
  // cost latency rather than samples. Upstream Point-LIO uses 200000.
  m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(rclcpp::KeepLast(2000)),
    std::bind(&PointLIONode::imuCB, this, std::placeholders::_1), sensor_options);

  // register scan subscriber. Log the resolved name so a missing/incorrect
  // remapping is visible at startup instead of showing up as "no data". Depth
  // 100 (10 s at 10 Hz) for the same reason as the IMU's; 10 was a full second
  // there, so it never bit, but a stalled reader should cost lag, not scans.
  if (m_node_config.lidar_type == 1) {
    m_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar", rclcpp::QoS(rclcpp::KeepLast(100)),
      std::bind(&PointLIONode::pc2CB, this, std::placeholders::_1), sensor_options);
    RCLCPP_INFO(
      this->get_logger(), "[PointLIONode][%s] Lidar input: sensor_msgs/PointCloud2 (%s)", __func__,
      m_pc2_sub->get_topic_name());
  } else {
    m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "lidar", rclcpp::QoS(rclcpp::KeepLast(100)),
      std::bind(&PointLIONode::lidarCB, this, std::placeholders::_1), sensor_options);
    RCLCPP_INFO(
      this->get_logger(), "[PointLIONode][%s] Lidar input: livox_ros_driver2/CustomMsg (%s)",
      __func__, m_lidar_sub->get_topic_name());
  }
  RCLCPP_INFO(
    this->get_logger(), "[PointLIONode][%s] IMU input: %s", __func__, m_imu_sub->get_topic_name());

  // register publisher
  m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
  m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
  // Depth 10, not the 10000 the others carry: ~130 KB a message, and a slow
  // subscriber should lose old scans rather than queue minutes of them.
  m_body_cloud_dense_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud_dense", 10);
  m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);

  // register transform broadcaster
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // initialize EKF & Map builder
  m_kf = std::make_shared<PointEKF>();
  m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

  // register timer
  m_timer = this->create_wall_timer(20ms, std::bind(&PointLIONode::timerCB, this));

  // The node's only service, and its first: throw the solution away and start
  // over. Relative name, so it lands on /<robot_id>/pointlio/reset -- the
  // workspace rule, and what lets pgo_node reach it through a config key
  // instead of spelling a namespace it does not own. The type lives in
  // syncai_common because the client (pgo_node, syncai_mapping) and this server
  // are two packages, and the backend that drives the whole reset builds
  // against that interface package.
  m_reset_srv = this->create_service<syncai_common::srv::ResetLIO>(
    "reset", std::bind(&PointLIONode::resetCB, this, std::placeholders::_1, std::placeholders::_2));
}

// Everything is a declared ROS parameter, fed by params/pointlio_params.yaml
// through the launch file's `parameters=[...]`. The node used to take a single
// `config_path` parameter and parse that YAML itself with yaml-cpp, which put
// it outside every ROS tool: `ros2 param list/get/dump` showed only
// config_path, a launch-level override could not touch a single value, and
// the launch file had to rewrite the whole YAML into a /tmp file just to
// inject the robot_id prefix. Declaring them properly is what lets the launch
// layer the two robot_id-dependent frame names on top of the shared file.
//
// The defaults below deliberately repeat the struct defaults (NodeConfig here,
// Config in map_builder/commons.h) so a missing key degrades exactly the way
// running the node without any params file does, instead of throwing.
void PointLIONode::loadParameters()
{
  // Frame names. TF frame ids are NOT namespaced by ROS, so the launch file
  // overrides both with the <robot_id>/ prefix; these are only the fallbacks
  // for running pointlio_node bare.
  m_node_config.body_frame = this->declare_parameter("body_frame", m_node_config.body_frame);
  m_node_config.world_frame = this->declare_parameter("world_frame", m_node_config.world_frame);
  m_node_config.print_time_cost =
    this->declare_parameter("print_time_cost", m_node_config.print_time_cost);
  m_node_config.lidar_type = this->declare_parameter("lidar_type", m_node_config.lidar_type);
  m_node_config.imu_acc_scale =
    this->declare_parameter("imu_acc_scale", m_node_config.imu_acc_scale);
  // The backlog guard; NodeConfig has the argument.
  m_node_config.max_lidar_backlog =
    this->declare_parameter("max_lidar_backlog", m_node_config.max_lidar_backlog);
  m_node_config.max_lidar_skip =
    this->declare_parameter("max_lidar_skip", m_node_config.max_lidar_skip);
  // The health log; NodeConfig has what each one decides.
  m_node_config.health_summary_period =
    this->declare_parameter("health.summary_period", m_node_config.health_summary_period);
  m_node_config.health_max_speed =
    this->declare_parameter("health.max_speed", m_node_config.health_max_speed);
  m_node_config.health_min_effective_ratio =
    this->declare_parameter("health.min_effective_ratio", m_node_config.health_min_effective_ratio);

  // config for map builder
  m_builder_config.lidar_filter_num =
    this->declare_parameter("lidar_filter_num", m_builder_config.lidar_filter_num);
  m_node_config.dense_filter_num =
    this->declare_parameter("dense_filter_num", m_node_config.dense_filter_num);
  if (
    m_node_config.dense_filter_num > 0 &&
    m_node_config.dense_filter_num < m_builder_config.lidar_filter_num &&
    m_builder_config.lidar_filter_num % m_node_config.dense_filter_num != 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PointLIONode] dense_filter_num %d does not divide lidar_filter_num %d: body_cloud_dense "
      "will not contain every body_cloud point",
      m_node_config.dense_filter_num, m_builder_config.lidar_filter_num);
  }
  m_builder_config.lidar_min_range =
    this->declare_parameter("lidar_min_range", m_builder_config.lidar_min_range);
  m_builder_config.lidar_max_range =
    this->declare_parameter("lidar_max_range", m_builder_config.lidar_max_range);
  m_builder_config.scan_resolution =
    this->declare_parameter("scan_resolution", m_builder_config.scan_resolution);
  m_builder_config.map_resolution =
    this->declare_parameter("map_resolution", m_builder_config.map_resolution);
  m_builder_config.cube_len = this->declare_parameter("cube_len", m_builder_config.cube_len);
  m_builder_config.det_range = this->declare_parameter("det_range", m_builder_config.det_range);
  m_builder_config.move_thresh =
    this->declare_parameter("move_thresh", m_builder_config.move_thresh);

  // Point-LIO output model
  m_builder_config.gyr_cov_output =
    this->declare_parameter("gyr_cov_output", m_builder_config.gyr_cov_output);
  m_builder_config.acc_cov_output =
    this->declare_parameter("acc_cov_output", m_builder_config.acc_cov_output);
  m_builder_config.b_gyr_cov = this->declare_parameter("b_gyr_cov", m_builder_config.b_gyr_cov);
  m_builder_config.b_acc_cov = this->declare_parameter("b_acc_cov", m_builder_config.b_acc_cov);
  m_builder_config.imu_meas_omg_cov =
    this->declare_parameter("imu_meas_omg_cov", m_builder_config.imu_meas_omg_cov);
  m_builder_config.imu_meas_acc_cov =
    this->declare_parameter("imu_meas_acc_cov", m_builder_config.imu_meas_acc_cov);
  m_builder_config.satu_gyro = this->declare_parameter("satu_gyro", m_builder_config.satu_gyro);
  m_builder_config.satu_acc = this->declare_parameter("satu_acc", m_builder_config.satu_acc);
  m_builder_config.lidar_meas_cov =
    this->declare_parameter("lidar_meas_cov", m_builder_config.lidar_meas_cov);
  m_builder_config.plane_thr = this->declare_parameter("plane_thr", m_builder_config.plane_thr);
  m_builder_config.batch_max_points =
    this->declare_parameter("batch_max_points", m_builder_config.batch_max_points);

  m_builder_config.imu_init_num =
    this->declare_parameter("imu_init_num", m_builder_config.imu_init_num);
  m_builder_config.near_search_num =
    this->declare_parameter("near_search_num", m_builder_config.near_search_num);
  m_builder_config.gravity_align =
    this->declare_parameter("gravity_align", m_builder_config.gravity_align);
  m_builder_config.esti_il = this->declare_parameter("esti_il", m_builder_config.esti_il);

  // IMU <- lidar extrinsic. Flat double arrays because ROS parameters have no
  // nested/matrix type; the size checks matter because the old yaml-cpp path
  // indexed r_il_vec[8] with no validation, i.e. a short list in the config
  // was undefined behaviour rather than an error message.
  std::vector<double> t_il_vec = this->declare_parameter<std::vector<double>>(
    "t_il", {m_builder_config.t_il.x(), m_builder_config.t_il.y(), m_builder_config.t_il.z()});
  std::vector<double> r_il_vec = this->declare_parameter<std::vector<double>>(
    "r_il", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (t_il_vec.size() == 3) {
    m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "[PointLIONode][%s] t_il needs 3 elements, got %zu; keeping default",
      __func__, t_il_vec.size());
  }
  if (r_il_vec.size() == 9) {
    m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4],
      r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "[PointLIONode][%s] r_il needs 9 elements (row-major 3x3), got %zu; keeping identity",
      __func__, r_il_vec.size());
  }
}

void PointLIONode::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // only use to store imu data into data structure
  std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_imu_time) {
    RCLCPP_WARN(this->get_logger(), "[PointLIONode][%s] IMU Message is out of order", __func__);
    std::deque<IMUData>().swap(m_state_data.imu_buffer);
  } else if (
    m_state_data.last_imu_time >= 0.0 && timestamp - m_state_data.last_imu_time > kImuGapWarnSec) {
    // A hole in the IMU stream is otherwise invisible: processIMU just
    // predicts across a longer dt with no measurement. 30 ms, not "one missed
    // 5 ms period", because the MID360 itself stamps up to ~23 ms apart a few
    // times an hour (3 in the 30 min of dp1f_1012); a reader-history overflow
    // loses whole bursts, well past that.
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[PointLIONode][%s] IMU gap of %.1f ms (samples lost before this node?)", __func__,
      (timestamp - m_state_data.last_imu_time) * 1000.0);
  }

  // push latest imu data into imu_buffer
  m_state_data.imu_buffer.emplace_back(
    V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) *
      m_node_config.imu_acc_scale,
    V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), timestamp);
  m_state_data.last_imu_time = timestamp;
}

void PointLIONode::lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  // only use to store lidar data into data structure
  CloudType::Ptr dense;
  CloudType::Ptr cloud = Utils::livox2PCL(
    msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
    m_builder_config.lidar_max_range, m_node_config.dense_filter_num, &dense);
  std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_lidar_time) {
    RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(
      m_state_data.lidar_buffer);
    m_state_data.lidar_dense_buffer.clear();
  }
  m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
  m_state_data.lidar_dense_buffer.push_back(dense);
  m_state_data.last_lidar_time = timestamp;
}

void PointLIONode::pc2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // CloudType => pcl::PointCloud<pcl::PointXYZINormal>
  CloudType::Ptr dense;
  CloudType::Ptr cloud = Utils::pc2ToPCL(
    msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
    m_builder_config.lidar_max_range, m_node_config.dense_filter_num, &dense);
  // PointCloud2 carries no per-point time (curvature = 0): every point in the
  // scan shares one timestamp, so the point-by-point update degenerates into a
  // single batch update per scan (no distortion compensation).
  RCLCPP_WARN_ONCE(
    this->get_logger(),
    "[PointLIONode][%s] PointCloud2 input has no per-point time; Point-LIO degrades to "
    "one batch update per scan",
    __func__);
  std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_lidar_time) {
    RCLCPP_WARN(this->get_logger(), "[PointLIONode][%s] Lidar Message is out of order", __func__);
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(
      m_state_data.lidar_buffer);
    m_state_data.lidar_dense_buffer.clear();
  }
  m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
  m_state_data.lidar_dense_buffer.push_back(dense);
  m_state_data.last_lidar_time = timestamp;
}

bool PointLIONode::syncPackage()
{
  // Both buffers, for the whole function: the sensor callbacks append to them
  // from the other executor thread. scoped_lock takes the pair deadlock-free;
  // the callbacks and resetCB only ever hold one at a time. Held across the
  // sort below, which is a few thousand points and cheaper than juggling the
  // cloud out from under the lock.
  std::scoped_lock lock(m_state_data.lidar_mutex, m_state_data.imu_mutex);

  if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty()) return false;

  // A lidar frame is not processed as soon as it arrives; it has to wait until the IMU data
  // has caught up to the end-of-scan time.
  if (!m_state_data.lidar_pushed) {
    // The backlog guard (NodeConfig::max_lidar_backlog): drop the oldest scans
    // while the buffer spans too much, at most max_lidar_skip of them before
    // the next one is processed. Only lidar_buffer is touched -- the skipped
    // scans' IMU samples are drained into this package below, so the filter
    // runs through the gap on the IMU. Measured on the lidar's own stamps,
    // never against the wall clock, which the sensor's need not share.
    if (m_node_config.max_lidar_backlog > 0.0) {
      auto span = [this]() {
        return m_state_data.lidar_buffer.back().first - m_state_data.lidar_buffer.front().first;
      };
      m_backlog_max_seen = std::max(m_backlog_max_seen, span());
      int skipped = 0;
      while (skipped < m_node_config.max_lidar_skip && m_state_data.lidar_buffer.size() > 1 &&
             span() > m_node_config.max_lidar_backlog) {
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_dense_buffer.pop_front();
        ++skipped;
      }
      m_backlog_skipped += skipped;
    }

    m_package.cloud = m_state_data.lidar_buffer.front().second;
    m_package.dense_cloud = m_state_data.lidar_dense_buffer.front();

    // The point cloud's curvature field holds each point's timestamp. Sorting makes the last
    // point of the cloud the latest one; otherwise it is merely the last element of the array,
    // with no relation to time.
    std::sort(
      m_package.cloud->points.begin(), m_package.cloud->points.end(),
      [](PointType & p1, PointType & p2) { return p1.curvature < p2.curvature; });

    m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
    m_package.cloud_end_time =
      m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
    m_state_data.lidar_pushed = true;
  }

  // Wait while the IMU data does not yet cover the whole point cloud frame (the point-by-point
  // update needs the IMU events for the entire frame)
  if (m_state_data.last_imu_time < m_package.cloud_end_time) return false;

  // Clear and release the IMU data held in m_package
  Vec<IMUData>().swap(m_package.imus);

  // Pull out the IMU data this frame needs and drain it from imu_buffer
  while (!m_state_data.imu_buffer.empty() &&
         m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
    // Push the IMU samples this frame needs into m_package.
    m_package.imus.emplace_back(m_state_data.imu_buffer.front());
    m_state_data.imu_buffer.pop_front();
  }

  // Drop the lidar frame that has just been consumed
  m_state_data.lidar_buffer.pop_front();
  m_state_data.lidar_dense_buffer.pop_front();
  m_state_data.lidar_pushed = false;
  return true;
}

void PointLIONode::publishCloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
  std::string frame_id, const double & time)
{
  if (pub->get_subscription_count() <= 0) return;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = Utils::getTime(time);
  pub->publish(cloud_msg);
}

void PointLIONode::publishCompactCloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, const CloudType & cloud,
  const std::string & frame_id, const double & time)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = Utils::getTime(time);
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(cloud.size());
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_dense = true;
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"),
    in(msg, "intensity");
  for (const auto & p : cloud.points) {
    *x = p.x;
    *y = p.y;
    *z = p.z;
    *in = p.intensity;
    ++x;
    ++y;
    ++z;
    ++in;
  }
  pub->publish(msg);
}

void PointLIONode::publishOdometry(
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
  std::string child_frame, const double & time)
{
  if (odom_pub->get_subscription_count() <= 0) return;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame_id;
  odom.header.stamp = Utils::getTime(time);
  odom.child_frame_id = child_frame;
  odom.pose.pose.position.x = m_kf->x().t_wi.x();
  odom.pose.pose.position.y = m_kf->x().t_wi.y();
  odom.pose.pose.position.z = m_kf->x().t_wi.z();
  Eigen::Quaterniond q(m_kf->x().r_wi);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();

  // The output model estimates angular velocity directly, so publish it as well
  odom.twist.twist.angular.x = m_kf->x().omg.x();
  odom.twist.twist.angular.y = m_kf->x().omg.y();
  odom.twist.twist.angular.z = m_kf->x().omg.z();
  odom_pub->publish(odom);
}

void PointLIONode::broadCastTF(
  std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
  std::string child_frame, const double & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame;
  transformStamped.header.stamp = Utils::getTime(time);

  // The subscript wi reads w <- i, right to left: "from i to w". Likewise r_il is i <- l
  // (lidar -> IMU). The full chain: lidar frame --T_il--> IMU frame --T_wi--> world frame
  Eigen::Quaterniond q(m_kf->x().r_wi);
  V3D t = m_kf->x().t_wi;

  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  broad_caster->sendTransform(transformStamped);
}

void PointLIONode::timerCB()
{
  const bool synced = syncPackage();
  // Every tick, not only the ones that process a scan: the all-clear has to
  // go out while the node is idle between scans too.
  reportBacklog();
  reportHealth();
  if (!synced) return;

  auto t1 = std::chrono::steady_clock::now();
  // Read before process(): the frame that reaches MAPPING is MAP_INIT's,
  // which only seeds the map and has no lidar update to report.
  const bool mapping = m_builder->status() == BuilderStatus::MAPPING;
  m_builder->process(m_package);
  auto t2 = std::chrono::steady_clock::now();
  if (m_builder->status() == BuilderStatus::MAPPING) publishResults();
  auto t3 = std::chrono::steady_clock::now();
  if (mapping) {
    recordHealth(std::chrono::duration<double, std::milli>(t3 - t1).count());
  }

  // Both numbers, because the total is the real per-scan budget: a scan
  // every 100 ms, so a total that stays above that means lidar_buffer grows
  // without bound. It used to print process() alone, which hid the deskews
  // and the cloud conversions behind a smaller number.
  if (m_node_config.print_time_cost) {
    using ms = std::chrono::duration<double, std::milli>;
    RCLCPP_WARN(
      this->get_logger(), "Time cost: process %.2f ms, total %.2f ms", ms(t2 - t1).count(),
      ms(t3 - t1).count());
  }
}

void PointLIONode::reportBacklog()
{
  const auto now = std::chrono::steady_clock::now();
  const double window = std::chrono::duration<double>(now - m_last_backlog_report).count();
  if (window < kBacklogReportSec) return;
  // The odometry is late by at least the backlog, and nothing downstream can
  // see that (every stamp is the scan's own), so this line is the only place
  // it shows.
  if (m_backlog_skipped > 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PointLIONode][%s] lidar backlog up to %.2f s (max_lidar_backlog %.2f s): skipped %d "
      "scans in the last %.0f s, the IMU fused across each gap. Point-LIO is not keeping up with "
      "the lidar -- is the CPU saturated?",
      __func__, m_backlog_max_seen, m_node_config.max_lidar_backlog, m_backlog_skipped, window);
    m_backlog_warned = true;
  } else if (m_backlog_warned) {
    RCLCPP_INFO(
      this->get_logger(),
      "[PointLIONode][%s] lidar backlog back under max_lidar_backlog: no scan skipped in the "
      "last %.0f s",
      __func__, window);
    m_backlog_warned = false;
  }
  m_backlog_skipped = 0;
  m_backlog_max_seen = 0.0;
  m_last_backlog_report = now;
}

void PointLIONode::recordHealth(double cost_ms)
{
  const ScanHealth & s = m_builder->health();
  HealthWindow & w = m_health;
  ++w.scans;
  w.points += s.lidar.points;
  w.near += s.lidar.near;
  w.effective += s.lidar.effective;
  w.residual_sum += s.lidar.residual_sum;
  if (s.lidar.points > 0) {
    w.effective_ratio_min =
      std::min(w.effective_ratio_min, static_cast<double>(s.lidar.effective) / s.lidar.points);
  }
  w.chunks += s.lidar.chunks;
  w.chunks_updated += s.lidar.chunks_updated;
  if (s.lidar.chunks_updated == 0) ++w.scans_without_update;
  w.imu_samples += s.imu_samples;
  w.acc_saturated += s.acc_saturated;
  w.gyro_saturated += s.gyro_saturated;
  w.acc_abs_max = std::max(w.acc_abs_max, s.acc_abs_max);
  w.gyro_abs_max = std::max(w.gyro_abs_max, s.gyro_abs_max);
  // At the end of the scan, the stamp lio_odom carries. World frame, so this
  // is the speed over ground a diverging filter runs away with.
  w.speed_max = std::max(w.speed_max, m_kf->x().v.norm());
  w.cost_ms_sum += cost_ms;
  w.cost_ms_max = std::max(w.cost_ms_max, cost_ms);
  m_health_seen_mapping = true;
}

// The `lio health:` line (NodeConfig::health_*). Every number the filter
// already has; nothing here feeds back into it. The reasons a line is a WARN
// are appended after " -- ", each with the number that tripped it, so
// `grep 'lio health'` reads as the node's timeline and `grep -- 'lio health.* -- '`
// as its trouble.
void PointLIONode::reportHealth()
{
  if (m_node_config.health_summary_period <= 0.0) return;
  const auto now = std::chrono::steady_clock::now();
  const double window = std::chrono::duration<double>(now - m_last_health_report).count();
  if (window < m_node_config.health_summary_period) return;
  m_last_health_report = now;
  const HealthWindow w = m_health;
  m_health = HealthWindow();

  // No scan in a whole window. Once, until scans come back: it repeats
  // nothing the first line did not say. The buffers tell the causes apart --
  // no lidar (lidar 0), the IMU not yet covering the scan's end (lidar > 0),
  // a reset re-initialising (IMU_INIT).
  if (w.scans == 0) {
    if (m_health_seen_mapping && !m_health_idle_warned) {
      size_t lidar_n = 0, imu_n = 0;
      {
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        lidar_n = m_state_data.lidar_buffer.size();
      }
      {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        imu_n = m_state_data.imu_buffer.size();
      }
      const BuilderStatus status = m_builder->status();
      RCLCPP_WARN(
        this->get_logger(),
        "lio health: no scan processed in the last %.1f s (status %s, lidar buffer %zu scans, "
        "IMU buffer %zu samples): no lidar arriving, or the IMU not covering the scan's end",
        window,
        status == BuilderStatus::MAPPING    ? "MAPPING"
        : status == BuilderStatus::MAP_INIT ? "MAP_INIT"
                                            : "IMU_INIT",
        lidar_n, imu_n);
      m_health_idle_warned = true;
    }
    return;
  }
  if (m_health_idle_warned) {
    RCLCPP_INFO(this->get_logger(), "lio health: scans processed again");
    m_health_idle_warned = false;
  }

  const double points = std::max<long>(w.points, 1);
  const double near_pct = 100.0 * w.near / points;
  const double eff_pct = 100.0 * w.effective / points;
  const double res_cm = w.effective > 0 ? 100.0 * w.residual_sum / w.effective : 0.0;
  // The 1-sigma of position (cm) and attitude (deg) from P's r_wi / t_wi
  // blocks: how sure the filter is. A collapse towards zero while the
  // residuals grow is an over-confident filter ignoring its measurements.
  const M30D & P = m_kf->P();
  const double std_p_cm = 100.0 * std::sqrt(std::max(0.0, P(3, 3) + P(4, 4) + P(5, 5)));
  const double std_r_deg = std::sqrt(std::max(0.0, P(0, 0) + P(1, 1) + P(2, 2))) * 180.0 / M_PI;
  const State & x = m_kf->x();

  std::string reasons;
  auto add = [&reasons](const std::string & r) {
    reasons += reasons.empty() ? " -- " : "; ";
    reasons += r;
  };
  char buf[256];
  if (w.speed_max > m_node_config.health_max_speed) {
    std::snprintf(
      buf, sizeof(buf), "speed %.2f m/s over health.max_speed %.2f", w.speed_max,
      m_node_config.health_max_speed);
    add(buf);
  }
  if (w.acc_saturated > 0) {
    std::snprintf(
      buf, sizeof(buf), "%d IMU samples with |acc| %.1f >= satu_acc %.1f dropped from the update",
      w.acc_saturated, w.acc_abs_max, m_builder_config.satu_acc);
    add(buf);
  }
  if (w.gyro_saturated > 0) {
    std::snprintf(
      buf, sizeof(buf), "%d IMU samples with |gyro| %.2f >= satu_gyro %.2f dropped from the update",
      w.gyro_saturated, w.gyro_abs_max, m_builder_config.satu_gyro);
    add(buf);
  }
  if (w.scans_without_update > 0) {
    std::snprintf(
      buf, sizeof(buf), "%d of %d scans matched no plane (IMU only)", w.scans_without_update,
      w.scans);
    add(buf);
  }
  if (
    m_node_config.health_min_effective_ratio > 0.0 &&
    w.effective_ratio_min < m_node_config.health_min_effective_ratio) {
    std::snprintf(
      buf, sizeof(buf), "effective ratio down to %.0f%% under health.min_effective_ratio %.0f%%",
      100.0 * w.effective_ratio_min, 100.0 * m_node_config.health_min_effective_ratio);
    add(buf);
  }

  // One line, fields in a fixed order for grep / awk:
  //  scans in the window | mean points per scan, near / effective share of
  //  them, the worst scan's effective share, mean |residual| | update chunks
  //  with a match / all | IMU samples, saturated samples, largest |axis| |
  //  speed now and the window's max | position / attitude 1-sigma | bias and
  //  gravity norms | timerCB cost per scan.
  const double scans = w.scans;
  char line[512];
  std::snprintf(
    line, sizeof(line),
    "lio health: %d scans in %.1f s | pts %.0f near %.0f%% eff %.0f%% (min %.0f%%) res %.2f cm | "
    "chunks %d/%d | imu %d sat acc %d gyro %d max |a| %.1f |w| %.2f | v %.2f max %.2f m/s | "
    "std p %.2f cm r %.3f deg | |ba| %.3f |bg| %.4f |g| %.3f | cost %.1f max %.1f ms",
    w.scans, window, w.points / scans, near_pct, eff_pct, 100.0 * w.effective_ratio_min, res_cm,
    w.chunks_updated, w.chunks, w.imu_samples, w.acc_saturated, w.gyro_saturated, w.acc_abs_max,
    w.gyro_abs_max, x.v.norm(), w.speed_max, std_p_cm, std_r_deg, x.ba.norm(), x.bg.norm(),
    x.g.norm(), w.cost_ms_sum / scans, w.cost_ms_max);
  if (reasons.empty()) {
    RCLCPP_INFO(this->get_logger(), "%s", line);
  } else {
    RCLCPP_WARN(this->get_logger(), "%s%s", line, reasons.c_str());
  }
}

void PointLIONode::publishResults()
{
  // The stamp resetCB hands back to pgo, recorded HERE rather than inside
  // publishOdometry: that one returns early when nothing is subscribed, and
  // the ordering gate must not depend on who happens to be listening. Past
  // this gate the frame is one the solution accepted, which is exactly what
  // "the last sample of the old run" has to mean.
  m_last_odom_time = m_package.cloud_end_time;

  // world frame -> odom3D
  // body_frame -> laser3D
  broadCastTF(
    m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame,
    m_package.cloud_end_time);

  publishOdometry(
    m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

  // Every published cloud is motion-compensated into the body frame at
  // cloud_end_time (MapBuilder::deskewToEndBody), the stamp and frame lio_odom
  // carries, so a consumer that pairs the two gets a scan that agrees with the
  // pose. Each deskew is skipped when nobody takes its output.
  const bool want_body = m_body_cloud_pub->get_subscription_count() > 0 ||
                         m_world_cloud_pub->get_subscription_count() > 0;
  CloudType::Ptr body_cloud;
  if (want_body) {
    body_cloud = m_builder->deskewToEndBody(m_package.cloud, m_package.cloud_start_time);
    publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);
    // world_cloud: the same deskewed scan put in the world with the end pose.
    CloudType::Ptr world_cloud =
      m_builder->lidar_processor()->transformCloud(body_cloud, m_kf->x().r_wi, m_kf->x().t_wi);
    publishCloud(
      m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);
  }
  if (m_body_cloud_dense_pub->get_subscription_count() > 0 && m_package.dense_cloud) {
    CloudType::Ptr dense =
      (m_package.dense_cloud == m_package.cloud && body_cloud)
        ? body_cloud
        : m_builder->deskewToEndBody(m_package.dense_cloud, m_package.cloud_start_time);
    publishCompactCloud(
      m_body_cloud_dense_pub, *dense, m_node_config.body_frame, m_package.cloud_end_time);
  }
}

// Throw the whole solution away and start over, in place.
//
// Everything below mirrors the constructor -- deliberately, because the
// constructor is the only definition of "a node that has not mapped anything
// yet" and a second, subtly different one would drift from it. The order
// matters twice: last_odom_time is read before anything is cleared (it is the
// answer), and the builder is replaced last (dropping it releases the old
// LidarProcessor and its ikd-tree, whose destructor joins the tree's rebuild
// thread -- the one genuinely blocking step here).
//
// NO LOCK against timerCB, and that is load-bearing: this service and the
// timer are both in the node's default MutuallyExclusive group, so timerCB
// cannot be halfway through MapBuilder::process() while the builder is swapped
// out, even under main()'s MultiThreadedExecutor. Moving this service (or the
// timer) into another callback group breaks that and needs a mutex shared with
// timerCB around m_builder / m_kf / m_package. The two sensor mutexes below are
// a different matter: the subscription callbacks have their own group and run
// concurrently with this handler on the other thread (since 2026-10). A
// sample that lands between the buffer clear and the builder swap goes into
// the new run's buffer, which is where it belongs -- the robot is standing
// still for the re-init anyway.
void PointLIONode::resetCB(
  const std::shared_ptr<syncai_common::srv::ResetLIO::Request> request,
  std::shared_ptr<syncai_common::srv::ResetLIO::Response> response)
{
  (void)request;  // empty by design -- a reset is not a reconfigure

  // First, before any of it is invalidated: this is the whole point of the
  // call for pgo, and everything after this line destroys the state that
  // produced it.
  response->last_odom_time = m_last_odom_time;

  {
    std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
    std::deque<IMUData>().swap(m_state_data.imu_buffer);
    m_state_data.last_imu_time = -1.0;
  }

  {
    std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(
      m_state_data.lidar_buffer);
    m_state_data.lidar_dense_buffer.clear();
    m_state_data.last_lidar_time = -1.0;
  }

  // Easy to miss and it silently defeats the timestamp gate: syncPackage()
  // only refills m_package while lidar_pushed is false. A reset landing
  // between "package assembled" and "package consumed" would otherwise leave
  // this true, so the first frame of the NEW run reuses the old package --
  // including its cloud_end_time -- and publishes new-run odometry carrying an
  // old-run stamp, which is precisely the message pgo's gate is meant to drop.
  m_state_data.lidar_pushed = false;
  Vec<IMUData>().swap(m_package.imus);
  m_package.cloud.reset();
  m_package.dense_cloud.reset();
  m_package.cloud_start_time = 0.0;
  m_package.cloud_end_time = 0.0;

  // The reset proper. Same two lines as the constructor: MapBuilder's ctor
  // re-runs setConfig on the EKF (which re-identities its covariance), builds
  // a fresh IMUInitializer and a fresh LidarProcessor -- new ikd-tree -- and
  // puts the status back to IMU_INIT.
  m_kf = std::make_shared<PointEKF>();
  m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

  m_last_odom_time = 0.0;
  // The old run's scans say nothing about the new one, and until it reaches
  // MAPPING an empty window is the re-init, not a lost lidar.
  m_health = HealthWindow();
  m_health_seen_mapping = false;
  m_health_idle_warned = false;

  response->success = true;
  response->message = "LIO reset; re-initialising IMU (robot must be still).";
  RCLCPP_WARN(this->get_logger(), "[PointLIONode][resetCB] %s", response->message.c_str());
}

}  // namespace syncai_pointlio
