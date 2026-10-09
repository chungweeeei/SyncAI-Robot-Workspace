#include "syncai_pointlio/pointlio_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace syncai_pointlio
{

PointLIONode::PointLIONode() : Node("pointlio_node")
{
  RCLCPP_INFO(this->get_logger(), "[PointLIONode][%s] Point-LIO Node Started", __func__);
  loadParameters();

  // register imu subscriber
  m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&PointLIONode::imuCB, this, std::placeholders::_1));

  // register scan subscriber. Log the resolved name so a missing/incorrect
  // remapping is visible at startup instead of showing up as "no data".
  if (m_node_config.lidar_type == 1) {
    m_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar", 10, std::bind(&PointLIONode::pc2CB, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "[PointLIONode][%s] Lidar input: sensor_msgs/PointCloud2 (%s)", __func__,
      m_pc2_sub->get_topic_name());
  } else {
    m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "lidar", 10, std::bind(&PointLIONode::lidarCB, this, std::placeholders::_1));
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
  m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
  m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);

  // register transform broadcaster
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // clear state data
  m_state_data.path.poses.clear();
  m_state_data.path.header.frame_id = m_node_config.world_frame;

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
  if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty()) return false;

  // A lidar frame is not processed as soon as it arrives; it has to wait until the IMU data
  // has caught up to the end-of-scan time.
  if (!m_state_data.lidar_pushed) {
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

void PointLIONode::publishPath(
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id,
  const double & time)
{
  if (path_pub->get_subscription_count() <= 0) return;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = Utils::getTime(time);
  pose.pose.position.x = m_kf->x().t_wi.x();
  pose.pose.position.y = m_kf->x().t_wi.y();
  pose.pose.position.z = m_kf->x().t_wi.z();
  Eigen::Quaterniond q(m_kf->x().r_wi);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  m_state_data.path.poses.push_back(pose);
  path_pub->publish(m_state_data.path);
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
  if (!syncPackage()) return;

  auto t1 = std::chrono::high_resolution_clock::now();
  m_builder->process(m_package);
  auto t2 = std::chrono::high_resolution_clock::now();

  if (m_node_config.print_time_cost) {
    auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
  }

  if (m_builder->status() != BuilderStatus::MAPPING) return;

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

  // odom path
  publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
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
// NO LOCKING against the executor, and that is load-bearing: main() spins this
// node with rclcpp::spin(), so every callback shares the default
// MutuallyExclusive group and timerCB cannot be halfway through
// MapBuilder::process() while the builder is swapped out. Moving this node to
// a MultiThreadedExecutor, or putting this service in its own callback group,
// breaks that and needs a mutex shared with timerCB. (pgo_node does exactly
// that, for a reason that does not apply here: it has to wait on a client
// future from inside its handler.) The two sensor mutexes below are a
// different matter -- they guard against the subscription callbacks, which are
// in the same group but would be a real race the moment anything changes.
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

  // Published unconditionally, unlike publishPath's subscriber-gated send: an
  // empty Path is what makes rviz drop the old trajectory instead of drawing
  // the new run appended to the end of the old one.
  m_state_data.path.poses.clear();
  m_path_pub->publish(m_state_data.path);

  // The reset proper. Same two lines as the constructor: MapBuilder's ctor
  // re-runs setConfig on the EKF (which re-identities its covariance), builds
  // a fresh IMUInitializer and a fresh LidarProcessor -- new ikd-tree -- and
  // puts the status back to IMU_INIT.
  m_kf = std::make_shared<PointEKF>();
  m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

  m_last_odom_time = 0.0;

  response->success = true;
  response->message = "LIO reset; re-initialising IMU (robot must be still).";
  RCLCPP_WARN(this->get_logger(), "[PointLIONode][resetCB] %s", response->message.c_str());
}

}  // namespace syncai_pointlio
