#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "syncai_util/node_thread.hpp"
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
 *
 * A map switch reloads the tree without a restart (2026-10-19):
 * `<planner_server>/<id>/load_map` (nav2_msgs/srv/LoadMap, map_server's own
 * type) takes the new octomap.bt as map_url, drops the current surface map at
 * once (plans fail with map_loading, never on the old site's map), builds the
 * new one and answers when it is in -- success is a result, not a receipt. A
 * failed build leaves no map at all, not the old one. A missing file is
 * refused before anything is dropped. See loadMapCallback().
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

  // The 2D costmap gives this planner its frame id and nothing else, so a
  // goal must not wait for it: a 3D-only map has no gridmap.yaml, and its
  // global costmap never turns current.
  bool usesCostmap() const override { return false; }

protected:
  // Load the .bt, build the surface map and the stair links, swap them in and
  // clear building_. Runs on the boot thread, or on the load_map service's
  // thread for a reload. On failure sets build_error_ (= error) and leaves no
  // map. The files are passed in rather than read from the members, which a
  // reload changes.
  bool buildMap(
    const std::string & octomap_file, const std::string & poses_file, std::string & error);

  // `<planner_server>/<id>/load_map`: reload from request->map_url (see the
  // class comment). Blocks for the build, on its own executor thread.
  void loadMapCallback(
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  // poses.txt next to the tree: map/<name>/ holds both.
  static std::string posesFileBeside(const std::string & octomap_file);

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

  // load_map: its own callback group on its own executor thread, because a
  // reload blocks for seconds and planner_server's main executor also serves
  // is_path_valid, clear_plan and the plan republish timer. One thread, so two
  // reloads never build at once: the second waits, and the last one wins.
  rclcpp::CallbackGroup::SharedPtr load_map_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr load_map_executor_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  // planner_server's clear_plan, so `plan` stops re-sending the old map's route.
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_plan_client_;
  // True only while loadMapCallback() writes octomap_file / poses_file back
  // into the parameters, which the parameter callback otherwise refuses.
  std::atomic<bool> updating_file_params_{false};
  // Declared last: destroyed first, so the service thread stops before the
  // members it uses go.
  std::unique_ptr<syncai_util::NodeThread> load_map_thread_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_plan_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace syncai_global_planner

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
