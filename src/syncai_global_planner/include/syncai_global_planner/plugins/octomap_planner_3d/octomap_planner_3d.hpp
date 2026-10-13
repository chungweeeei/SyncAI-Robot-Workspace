#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_global_planner
{

/**
 * @class OctomapPlanner3D
 * @brief Ground-constrained 3D global planner for the quadruped, on the
 * OctoMap a map was saved with (map/<name>/octomap.bt, written by
 * syncai_mapping's clean_map).
 *
 * At initialize() the tree is reduced, in a background thread, to a
 * multi-layer 2.5D surface map (surface_map.hpp): every column's floor
 * surfaces, each judged with the robot's geometry (body clearance, footprint
 * radius, step height, slope). createPlan() snaps start and goal onto that
 * ground, runs A* over it (surface_astar.hpp), shortcuts and resamples the
 * result (path_postprocess.hpp) and returns it in the costmap's global frame
 * with z = the floor's top face. The path therefore never leaves the ground,
 * climbs a wall, crosses a stair riser or a kerb, or jumps between floors;
 * it takes ramps. The one way between floors is a stair the robot walked
 * while the map was made: the map's poses.txt is read next to the tree and
 * every flight on that track becomes a stair link (stair_links.hpp), which
 * the search may take in either direction and the path follows point by
 * point.
 *
 * What it takes from the planner server and what it ignores: the 2D costmap
 * is used only for its global frame id. The start pose the server passes
 * comes from the planar map -> base_link TF, whose z is always 0, so the
 * start height is read from TF (map -> <robot_id>/pointlio_body, the LIO
 * chain that keeps z) minus lidar_height, falling back to the request. The
 * goal's z is the request's (goal_z_mode: request) or the start's
 * (goal_z_mode: start, for a 2D backend that can only say z = 0 while the
 * robot is upstairs).
 *
 * A missing or unreadable octomap_file is logged once and makes every plan
 * fail with map_not_loaded; it never throws, so a map without a tree still
 * boots the planner server with the other planner ids intact.
 */
class OctomapPlanner3D : public syncai_nav_core::GlobalPlanner
{
public:
  OctomapPlanner3D() = default;
  ~OctomapPlanner3D() override;

  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // The background thread: load the .bt, build the surface map, swap it in.
  void buildMap();

  // z of the ground under the robot from TF (body_frame_ minus lidar_height_).
  bool startHeightHint(double & z) const;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Weak: the planner server owns the node and us; a strong pointer here
  // would be an ownership cycle (same as SmacPlanner2D).
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("OctomapPlanner3D")};
  std::string global_frame_, name_;

  // Parameters (see planner_server_params.yaml for the reasoning).
  std::string octomap_file_, poses_file_, body_frame_, goal_z_mode_;
  double lidar_height_{0.481}, interpolation_resolution_{0.1}, path_z_offset_{0.0};
  bool shortcut_{true}, publish_raw_plan_{false}, stair_links_{true};
  octomap_planner_3d::SurfaceMapParams map_params_;
  octomap_planner_3d::StairLinkParams stair_params_;
  octomap_planner_3d::SearchParams search_params_;

  // One lock around createPlan, the parameter callback and the map swap.
  std::mutex mutex_;
  std::shared_ptr<const octomap_planner_3d::SurfaceMap> map_;
  std::shared_ptr<const octomap_planner_3d::StairLinks> links_;
  std::unique_ptr<octomap_planner_3d::SurfaceAStar> astar_;
  std::thread build_thread_;
  std::atomic<bool> building_{false};
  std::string build_error_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_plan_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace syncai_global_planner

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
