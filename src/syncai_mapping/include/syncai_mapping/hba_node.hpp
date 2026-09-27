#ifndef SYNCAI_MAPPING__HBA_NODE_HPP_
#define SYNCAI_MAPPING__HBA_NODE_HPP_

// The ROS shell around hierarchical bundle adjustment (HBA / BALM): an
// OFFLINE refinement step over the patches/ + poses.txt pair save_maps
// writes. Not part of any session; an operator runs it by hand after a
// mapping drive, calls refine_map, watches map_points converge, and writes
// the result out with save_poses. The maths is hba/ (blam.*, hba.*, upstream
// code); this class owns the two services, the timer that drives the
// optimisation, and the preview publisher.
//
// Ported into the workspace from SyncAI-Fast-LIO2's `hba` package in 2026-09
// as syncai_mapping's second node, with the same executable, node name,
// services and topic. Three things changed with the port: the node runs at
// /<robot_id>/hba (it never had the robot_id namespace the rest of the stack
// uses), configuration is declared ROS parameters instead of a yaml-cpp
// config_path, and the service types are syncai_common's.

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "syncai_common/srv/refine_map.hpp"
#include "syncai_common/srv/save_poses.hpp"
#include "syncai_mapping/hba/hba.h"

namespace syncai_mapping
{

struct HBANodeConfig
{
  // Voxel leaf applied to every patch on load and to the preview merge.
  double scan_resolution = 0.1;
};

class HBANode : public rclcpp::Node
{
public:
  HBANode();

  // Declares every parameter with the struct default as its default. See the
  // definition for why these are ROS parameters and not a hand-parsed YAML.
  void loadParameters();

  // Load patches/ + poses.txt from a map directory and schedule the
  // optimisation; returns as soon as the patches are in.
  void refineMapCB(
    const std::shared_ptr<syncai_common::srv::RefineMap::Request> request,
    std::shared_ptr<syncai_common::srv::RefineMap::Response> response);
  // Write the refined poses to a file of the caller's choosing.
  void savePosesCB(
    const std::shared_ptr<syncai_common::srv::SavePoses::Request> request,
    std::shared_ptr<syncai_common::srv::SavePoses::Response> response);

  // 100 ms timer: runs hba_iter optimisation passes once refine_map has
  // loaded something, publishing the preview after each.
  void mainCB();
  void publishMap();

private:
  HBANodeConfig m_node_config;
  HBAConfig m_hba_config;
  std::shared_ptr<HBA> m_hba;
  rclcpp::Service<syncai_common::srv::RefineMap>::SharedPtr m_refine_map_srv;
  rclcpp::Service<syncai_common::srv::SavePoses>::SharedPtr m_save_poses_srv;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;

  pcl::VoxelGrid<pcl::PointXYZI> m_voxel_grid;
  // Serialises the two services against the timer. Under the single-threaded
  // spin in hba_main.cpp the executor already does that; the mutex is what
  // keeps it true if the executor ever changes.
  std::mutex m_service_mutex;
  bool m_do_optimize = false;
  rclcpp::TimerBase::SharedPtr m_timer;
};

// Parse one poses.txt line, "<i>.pcd tx ty tz qw qx qy qz" (the format
// pgo_node's save_maps writes), into a file name and a Pose.
void fromStr(const std::string & str, std::string & file_name, Pose & pose);

}  // namespace syncai_mapping

#endif  // SYNCAI_MAPPING__HBA_NODE_HPP_
