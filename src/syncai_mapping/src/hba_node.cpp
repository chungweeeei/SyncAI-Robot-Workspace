#include "syncai_mapping/hba_node.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace syncai_mapping
{

using namespace std::chrono_literals;

void fromStr(const std::string & str, std::string & file_name, Pose & pose)
{
  std::stringstream ss(str);
  std::vector<std::string> tokens;
  std::string token;
  while (std::getline(ss, token, ' ')) {
    tokens.push_back(token);
  }
  assert(tokens.size() == 8);
  file_name = tokens[0];
  pose.t = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
  pose.r = Eigen::Quaterniond(
             std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]))
             .normalized()
             .toRotationMatrix();
}

HBANode::HBANode() : Node("hba_node")
{
  RCLCPP_INFO(this->get_logger(), "HBA node started");
  loadParameters();
  m_hba = std::make_shared<HBA>(m_hba_config);
  m_voxel_grid.setLeafSize(
    m_node_config.scan_resolution, m_node_config.scan_resolution, m_node_config.scan_resolution);
  // Relative names, so they land on /<robot_id>/hba/{refine_map,save_poses}
  // -- the workspace rule. Upstream ran this node at a bare /hba with no
  // robot_id; the port put it under the same namespacing as every other
  // node, which is why the fork's README used to say `/hba/refine_map`.
  m_refine_map_srv = this->create_service<syncai_common::srv::RefineMap>(
    "refine_map",
    std::bind(&HBANode::refineMapCB, this, std::placeholders::_1, std::placeholders::_2));
  m_save_poses_srv = this->create_service<syncai_common::srv::SavePoses>(
    "save_poses",
    std::bind(&HBANode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));
  m_timer = this->create_wall_timer(100ms, std::bind(&HBANode::mainCB, this));
  m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
}

// Everything is a declared ROS parameter, fed by params/hba_params.yaml
// through the launch file's `parameters=[...]`. Until 2026-09 the node took a
// single `config_path` parameter and parsed that YAML itself with yaml-cpp,
// the same mechanism pgo_node had; it went with pgo's port, so that this
// package has one configuration story and no yaml-cpp dependency.
//
// The defaults repeat the struct defaults (HBANodeConfig here, HBAConfig in
// hba/hba.h) so a missing key degrades to running the node bare instead of
// throwing -- a change from the yaml-cpp loader, where every key was required.
// Two of the HBAConfig fields are size_t; ROS parameters have no unsigned
// type, so they are declared as int and cast, with a floor at zero.
//
// Read once: m_hba_config is copied into HBA at construction, so
// `ros2 param set` after startup changes nothing.
void HBANode::loadParameters()
{
  m_node_config.scan_resolution =
    this->declare_parameter("scan_resolution", m_node_config.scan_resolution);

  m_hba_config.window_size = this->declare_parameter("window_size", m_hba_config.window_size);
  m_hba_config.stride = this->declare_parameter("stride", m_hba_config.stride);
  m_hba_config.voxel_size = this->declare_parameter("voxel_size", m_hba_config.voxel_size);
  m_hba_config.min_point_num = this->declare_parameter("min_point_num", m_hba_config.min_point_num);
  m_hba_config.max_layer = this->declare_parameter("max_layer", m_hba_config.max_layer);
  m_hba_config.plane_thresh = this->declare_parameter("plane_thresh", m_hba_config.plane_thresh);
  const int ba_max_iter =
    this->declare_parameter("ba_max_iter", static_cast<int>(m_hba_config.ba_max_iter));
  const int hba_iter = this->declare_parameter("hba_iter", static_cast<int>(m_hba_config.hba_iter));
  m_hba_config.ba_max_iter = static_cast<size_t>(std::max(ba_max_iter, 0));
  m_hba_config.hba_iter = static_cast<size_t>(std::max(hba_iter, 0));
  m_hba_config.down_sample = this->declare_parameter("down_sample", m_hba_config.down_sample);

  RCLCPP_INFO(
    this->get_logger(),
    "[HBANode] window %d / stride %d, voxel %.2f m, %d layers, %zu BA iters x %zu HBA passes",
    m_hba_config.window_size, m_hba_config.stride, m_hba_config.voxel_size, m_hba_config.max_layer,
    m_hba_config.ba_max_iter, m_hba_config.hba_iter);
}

void HBANode::refineMapCB(
  const std::shared_ptr<syncai_common::srv::RefineMap::Request> request,
  std::shared_ptr<syncai_common::srv::RefineMap::Response> response)
{
  std::lock_guard<std::mutex> lock(m_service_mutex);
  if (!std::filesystem::exists(request->maps_path)) {
    response->success = false;
    response->message = "maps_path not exists";
    return;
  }

  std::filesystem::path p_dir(request->maps_path);
  std::filesystem::path pcd_dir = p_dir / "patches";
  if (!std::filesystem::exists(pcd_dir)) {
    response->success = false;
    response->message = pcd_dir.string() + " not exists";
    return;
  }

  std::filesystem::path txt_file = p_dir / "poses.txt";

  if (!std::filesystem::exists(txt_file)) {
    response->success = false;
    response->message = txt_file.string() + " not exists";
    return;
  }

  std::ifstream ifs(txt_file);
  std::string line;
  std::string file_name;
  Pose pose;
  pcl::PCDReader reader;
  while (std::getline(ifs, line)) {
    fromStr(line, file_name, pose);
    std::filesystem::path pcd_file = p_dir / "patches" / file_name;
    if (!std::filesystem::exists(pcd_file)) {
      std::cerr << "pcd file not found" << std::endl;
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    reader.read(pcd_file, *cloud);
    m_voxel_grid.setInputCloud(cloud);
    m_voxel_grid.filter(*cloud);
    m_hba->insert(cloud, pose);
  }
  RCLCPP_INFO(this->get_logger(), "LOAD POSE %lu;", m_hba->poses().size());
  response->success = true;
  response->message = "load poses success!";
  m_do_optimize = true;
  return;
}

void HBANode::savePosesCB(
  const std::shared_ptr<syncai_common::srv::SavePoses::Request> request,
  std::shared_ptr<syncai_common::srv::SavePoses::Response> response)
{
  std::lock_guard<std::mutex> lock(m_service_mutex);
  std::filesystem::path file_path(request->file_path);
  std::filesystem::path par_path = file_path.parent_path();
  if (!std::filesystem::exists(par_path)) {
    response->success = false;
    response->message = "parent path not exists";
    return;
  }
  if (m_hba->poses().size() < 1) {
    response->success = false;
    response->message = "poses is empty";
    return;
  }

  if (std::filesystem::exists(file_path)) {
    std::filesystem::remove(file_path);
  }

  m_hba->writePoses(file_path);
  response->success = true;
  response->message = "save poses success!";
  return;
}

void HBANode::mainCB()
{
  {
    std::lock_guard<std::mutex> lock(m_service_mutex);
    if (!m_do_optimize) return;
    RCLCPP_WARN(this->get_logger(), "START OPTIMIZE");
    publishMap();
    for (size_t i = 0; i < m_hba_config.hba_iter; i++) {
      RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu START======", i + 1);
      m_hba->optimize();
      publishMap();
      RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu END========", i + 1);
    }

    m_do_optimize = false;
    RCLCPP_WARN(this->get_logger(), "END OPTIMIZE");
  }
}

void HBANode::publishMap()
{
  if (m_cloud_pub->get_subscription_count() < 1) return;
  if (m_hba->poses().size() < 1) return;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_hba->getMapPoints();
  m_voxel_grid.setInputCloud(cloud);
  m_voxel_grid.filter(*cloud);
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = this->now();
  m_cloud_pub->publish(cloud_msg);
}

}  // namespace syncai_mapping
