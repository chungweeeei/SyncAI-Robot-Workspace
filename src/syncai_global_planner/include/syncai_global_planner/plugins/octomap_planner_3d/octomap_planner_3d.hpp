#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/connection.h"
#include "message_filters/subscriber.h"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "syncai_costmap/costmap_ros.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/keepout_zones.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "syncai_util/node_thread.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"

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
 * refused before anything is dropped. An empty map_url unloads the map
 * (what the backend's switch to a 2D map sends; no 2D map navigates since
 * 2026-10-28). See loadMapCallback().
 *
 * There is no `map_available` topic since 2026-10-28: from 2026-10-19 the
 * task_runner picked the 3D or the 2D trees by it, and the local costmap its
 * sensor layer; both went with 2D navigation and left it with no reader.
 *
 * Dynamic obstacles (2026-10-21): the surface map is static, so on its own
 * a replan returns the route a person is standing on, and the 2D global
 * costmap (layerless since 2026-10-28) could not judge a 3D path. The plugin therefore keeps
 * a dynamic overlay of its own (dynamic_overlay.hpp): the lidar cloud
 * (`dynamic.topic`, pointlio/body_cloud) in the map frame, every return in
 * the band above the surface it lies on a mark on that cell, rays clearing
 * the marks they cross, marks aging out after dynamic.obstacle_memory. A
 * plan reads a snapshot of it (the search and the shortcut keep off the
 * marks and their block_radius ring, and pay for the band beyond as for a
 * wall), and isPathValid() -- which the planner server calls for the
 * planner that made the last plan -- reads the same snapshot for the path
 * ahead, so move.xml's IsPathValid / blocked_time / replan machinery works
 * on it. The overlay has
 * its own lock, and its scans arrive on the global costmap's TF thread (see
 * initialize()); nothing of it waits behind mutex_.
 * `<planner_server>/<id>/clear_overlay` (nav2_msgs/srv/
 * ClearEntireCostmap, an empty request) drops every mark: what the 3D
 * trees' ClearEntireCostmap nodes call where the 2D trees clear the global
 * costmap.
 *
 * Keepout zones (2026-10-23): map/<name>/keepout.json beside the tree --
 * polygons in the map frame with a z band each (keepout_zones.hpp has the
 * format and the rules) -- is read with the tree, at boot and on every
 * load_map, and rasterised onto the surface map as a KeepoutMask: the
 * polygon's cells on the band's floor are the core, dilated by
 * keepout.block_radius (the robot's radius, as the 2D KeepoutFilter inflates
 * its mask) and costed beyond like a wall; a stair link with a point in a
 * core is cut. The mask rides beside the dynamic overlay's in every
 * BlockedView, so a plan, its shortcut, the goal snap and isPathValid()
 * keep off a zone as they keep off a person; a robot standing inside a core
 * is refused a plan (start_in_keepout), and clear_overlay leaves the zones
 * alone. `<planner_server>/<id>/load_keepout` (nav2_msgs/srv/LoadMap,
 * map_url = the file, "" = no zones) reloads an edited file without
 * rebuilding the surface map -- what the backend calls after it writes the
 * file; a map switch needs no extra call, load_map reads the new map's own.
 * An absent file is no zones (one info line); a broken one beside a good
 * tree loads the map without zones and says so in an error, since failing
 * the whole load would leave no map at all. See loadKeepoutCallback().
 */
class OctomapPlanner3D : public syncai_nav_core::GlobalPlanner
{
public:
  OctomapPlanner3D() = default;
  ~OctomapPlanner3D() override;

  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  // The part of `path` ahead of the robot against the dynamic overlay and
  // the keepout zones, on the floor stretch the robot is on (see the .cpp
  // for the stair rule). Takes the overlay's lock only, never mutex_.
  bool isPathValid(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot_pose,
    std::vector<int32_t> & invalid_pose_indices) override;

protected:
  // The lidar cloud, transformed into the map frame and fed to the overlay.
  // Runs on the global costmap's executor thread, the one that fills tf_
  // from /tf: the subscription and the filter's timers are on its callback
  // group, so the filter calls back from that thread whichever way the
  // transform arrives.
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);

  // `<planner_server>/<id>/clear_overlay`: drop every mark.
  void clearOverlayCallback(
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);
  // Load the .bt, build the surface map, the stair links and the keepout
  // mask, swap them in and clear building_. Runs on the boot thread, or on
  // the load_map service's thread for a reload. On failure sets build_error_
  // (= error) and leaves no map. The files are passed in rather than read
  // from the members, which a reload changes.
  bool buildMap(
    const std::string & octomap_file, const std::string & poses_file,
    const std::string & keepout_file, std::string & error);

  // `<planner_server>/<id>/load_map`: reload from request->map_url (see the
  // class comment). Blocks for the build, on its own executor thread.
  void loadMapCallback(
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  // `<planner_server>/<id>/load_keepout`: replace the zones from
  // request->map_url on the loaded map, "" = none (see the class comment).
  // On load_map's thread, so it never interleaves with a reload.
  void loadKeepoutCallback(
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  // Read `file` and rasterise it on `map` / `links` with `block_radius`.
  // Absent: no zones, one info line. Loaded: zones and mask set, the one
  // summary line logged. Failed: error logged with the parser's message,
  // zones and mask untouched. Never takes mutex_: the caller swaps.
  enum class KeepoutLoad { Loaded, Absent, Failed };
  KeepoutLoad loadKeepout(
    const std::string & file, const octomap_planner_3d::SurfaceMap & map,
    const octomap_planner_3d::StairLinks * links, double block_radius,
    std::vector<octomap_planner_3d::KeepoutZone> & zones,
    octomap_planner_3d::KeepoutMaskPtr & mask) const;

  // `name` next to the tree: map/<name>/ holds poses.txt and keepout.json
  // beside octomap.bt.
  static std::string fileBeside(const std::string & octomap_file, const char * name);

  // z of the ground under the robot from TF (body_frame_ minus lidar_height_).
  bool startHeightHint(double & z) const;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Weak: the planner server owns the node and us; a strong pointer here
  // would be an ownership cycle.
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("OctomapPlanner3D")};
  std::string global_frame_, name_;

  // Parameters (see planner_server_params.yaml for the reasoning).
  std::string octomap_file_, poses_file_, body_frame_, goal_z_mode_;
  double lidar_height_{0.481}, interpolation_resolution_{0.1}, path_z_offset_{0.0};
  bool shortcut_{true}, stair_links_{true};
  octomap_planner_3d::SurfaceMapParams map_params_;
  octomap_planner_3d::StairLinkParams stair_params_;
  octomap_planner_3d::SearchParams search_params_;
  // dynamic.*: the overlay's own (dynamic_overlay.hpp), plus whether to
  // subscribe at all, what, and how long a cloud may wait for its transform.
  bool dynamic_enabled_{true};
  std::string dynamic_topic_{"pointlio/body_cloud"};
  double transform_tolerance_{0.3};
  octomap_planner_3d::DynamicOverlayParams dynamic_params_;
  // keepout.*: the file (derived beside octomap_file when empty; written
  // back by load_map / load_keepout, refused to `ros2 param set`) and the
  // ring round a zone's core (dynamic: re-rasterised on change).
  std::string keepout_file_;
  double keepout_block_radius_{0.25};

  // The dynamic overlay: bound to map_ by buildMap(), unbound by
  // loadMapCallback(); read by createPlan() (under mutex_) and by
  // isPathValid() and the cloud callback (never under mutex_). Its own lock
  // orders those; mutex_ -> overlay lock is the one order ever taken. It
  // also carries the keepout mask for isPathValid() (setKeepout).
  std::unique_ptr<octomap_planner_3d::DynamicOverlay> overlay_;

  // One lock around createPlan, the parameter callback and the map swap.
  std::mutex mutex_;
  std::shared_ptr<const octomap_planner_3d::SurfaceMap> map_;
  std::shared_ptr<const octomap_planner_3d::StairLinks> links_;
  std::unique_ptr<octomap_planner_3d::SurfaceAStar> astar_;
  // The zones as parsed (for messages and a re-rasterise) and their mask on
  // map_ (immutable; a plan copies the pointer and reads that).
  std::vector<octomap_planner_3d::KeepoutZone> zones_;
  octomap_planner_3d::KeepoutMaskPtr keepout_;
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
  // load_keepout shares load_map's group and thread: a reload and a keepout
  // load never interleave, and neither stalls the main executor.
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_keepout_service_;
  // planner_server's clear_plan, so `plan` stops re-sending the old map's route.
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_plan_client_;
  // True only while loadMapCallback() / loadKeepoutCallback() write
  // octomap_file / poses_file / keepout.file back into the parameters,
  // which the parameter callback otherwise refuses.
  std::atomic<bool> updating_file_params_{false};
  // Declared last: destroyed first, so the service thread stops before the
  // members it uses go.
  std::unique_ptr<syncai_util::NodeThread> load_map_thread_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // The overlay's input: the cloud through a TF message filter to the map
  // frame (as the costmap's terrain layer takes it), subscribed on the global
  // costmap's node and callback group, so the thread that takes /tf into tf_
  // runs the whole filter (see initialize() for why it has no thread of its
  // own). A plan or a reload still never queues the scans, nor the scans a
  // plan: those run on the action and load_map threads. clear_overlay is on
  // the node's default group (the planner server's main executor, beside
  // is_path_valid): O(marks) under the overlay's lock, and the BT gives it
  // 20 ms. The subscriber takes the costmap node by raw pointer: the planner
  // server owns it and destroys it after the planners.
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_overlay_service_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp::Node>>
    cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;
  // cloudCallback()'s registration on cloud_filter_, which the destructor
  // disconnects first: the costmap's thread is still spinning while this
  // plugin is destroyed, and the disconnect is what waits for a call in
  // progress there (the filter's signal lock) and stops the next one.
  message_filters::Connection cloud_connection_;
};

}  // namespace syncai_global_planner

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__OCTOMAP_PLANNER_3D_HPP_
