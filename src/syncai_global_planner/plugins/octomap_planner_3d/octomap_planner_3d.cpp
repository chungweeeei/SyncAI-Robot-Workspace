#include "syncai_global_planner/plugins/octomap_planner_3d/octomap_planner_3d.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace syncai_global_planner
{

using syncai_util::declare_parameter_if_not_declared;
namespace o3d = octomap_planner_3d;

OctomapPlanner3D::~OctomapPlanner3D()
{
  // The service thread first: a reload in progress joins build_thread_ too,
  // and waits for its build to end before the thread stops.
  load_map_thread_.reset();
  if (build_thread_.joinable()) {
    build_thread_.join();
  }
}

std::string OctomapPlanner3D::posesFileBeside(const std::string & octomap_file)
{
  const auto slash = octomap_file.find_last_of('/');
  return (slash == std::string::npos ? std::string() : octomap_file.substr(0, slash + 1)) +
         "poses.txt";
}

void OctomapPlanner3D::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  tf_ = tf;
  name_ = name;
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  // The only thing taken from the 2D costmap: the frame the path is in. The
  // planner server validates and republishes paths in that frame.
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(
    logger_, "[OctomapPlanner3D][%s] Configuring plugin %s of type OctomapPlanner3D", __func__,
    name_.c_str());

  const auto dp = [&](const std::string & key, const rclcpp::ParameterValue & def) {
    declare_parameter_if_not_declared(node, name + "." + key, def);
  };
  dp("octomap_file", rclcpp::ParameterValue(std::string("")));
  dp("poses_file", rclcpp::ParameterValue(std::string("")));
  dp("body_frame", rclcpp::ParameterValue(std::string("pointlio_body")));
  dp("lidar_height", rclcpp::ParameterValue(lidar_height_));
  dp("goal_z_mode", rclcpp::ParameterValue(std::string("request")));
  dp("interpolation_resolution", rclcpp::ParameterValue(interpolation_resolution_));
  dp("path_z_offset", rclcpp::ParameterValue(path_z_offset_));
  dp("shortcut", rclcpp::ParameterValue(shortcut_));
  dp("publish_raw_plan", rclcpp::ParameterValue(publish_raw_plan_));
  // Surface map (build time).
  dp("robot_radius", rclcpp::ParameterValue(map_params_.robot_radius));
  dp("body_clearance", rclcpp::ParameterValue(map_params_.body_clearance));
  dp("max_step_height", rclcpp::ParameterValue(map_params_.max_step_height));
  dp("max_slope_deg", rclcpp::ParameterValue(map_params_.max_slope_deg));
  dp(
    "max_unknown_clearance_cells", rclcpp::ParameterValue(map_params_.max_unknown_clearance_cells));
  dp("min_known_free_above", rclcpp::ParameterValue(map_params_.min_known_free_above));
  dp("hole_fill_radius", rclcpp::ParameterValue(map_params_.hole_fill_radius));
  dp("hole_fill_min_support", rclcpp::ParameterValue(map_params_.hole_fill_min_support));
  dp("hole_fill_passes", rclcpp::ParameterValue(map_params_.hole_fill_passes));
  dp("clearance_cost_radius", rclcpp::ParameterValue(map_params_.clearance_cost_radius));
  // Stair links (build time).
  dp("stair_links", rclcpp::ParameterValue(stair_links_));
  dp("stair_min_rise", rclcpp::ParameterValue(stair_params_.min_rise));
  dp("stair_max_length", rclcpp::ParameterValue(stair_params_.max_length));
  dp("stair_anchor_z_window", rclcpp::ParameterValue(stair_params_.anchor_z_window));
  dp("stair_anchor_min_area", rclcpp::ParameterValue(stair_params_.anchor_min_area));
  dp("stair_smooth_passes", rclcpp::ParameterValue(stair_params_.smooth_passes));
  // Search (dynamic).
  dp("clearance_cost_weight", rclcpp::ParameterValue(search_params_.clearance_cost_weight));
  dp("clearance_cost_scaling", rclcpp::ParameterValue(search_params_.clearance_cost_scaling));
  dp("inferred_cost_multiplier", rclcpp::ParameterValue(search_params_.inferred_cost_multiplier));
  dp("heuristic_weight", rclcpp::ParameterValue(search_params_.heuristic_weight));
  dp("max_iterations", rclcpp::ParameterValue(static_cast<int>(search_params_.max_iterations)));
  dp("max_planning_time", rclcpp::ParameterValue(search_params_.max_planning_time));
  dp("snap_xy_radius", rclcpp::ParameterValue(search_params_.snap_xy_radius));
  dp("snap_z_window", rclcpp::ParameterValue(search_params_.snap_z_window));
  dp("stair_cost_multiplier", rclcpp::ParameterValue(search_params_.stair_cost_multiplier));

  node->get_parameter(name + ".octomap_file", octomap_file_);
  node->get_parameter(name + ".poses_file", poses_file_);
  node->get_parameter(name + ".body_frame", body_frame_);
  node->get_parameter(name + ".lidar_height", lidar_height_);
  node->get_parameter(name + ".goal_z_mode", goal_z_mode_);
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name + ".path_z_offset", path_z_offset_);
  node->get_parameter(name + ".shortcut", shortcut_);
  node->get_parameter(name + ".publish_raw_plan", publish_raw_plan_);
  node->get_parameter(name + ".robot_radius", map_params_.robot_radius);
  node->get_parameter(name + ".body_clearance", map_params_.body_clearance);
  node->get_parameter(name + ".max_step_height", map_params_.max_step_height);
  node->get_parameter(name + ".max_slope_deg", map_params_.max_slope_deg);
  node->get_parameter(
    name + ".max_unknown_clearance_cells", map_params_.max_unknown_clearance_cells);
  node->get_parameter(name + ".min_known_free_above", map_params_.min_known_free_above);
  node->get_parameter(name + ".hole_fill_radius", map_params_.hole_fill_radius);
  node->get_parameter(name + ".hole_fill_min_support", map_params_.hole_fill_min_support);
  node->get_parameter(name + ".hole_fill_passes", map_params_.hole_fill_passes);
  node->get_parameter(name + ".clearance_cost_radius", map_params_.clearance_cost_radius);
  node->get_parameter(name + ".stair_links", stair_links_);
  node->get_parameter(name + ".stair_min_rise", stair_params_.min_rise);
  node->get_parameter(name + ".stair_max_length", stair_params_.max_length);
  node->get_parameter(name + ".stair_anchor_z_window", stair_params_.anchor_z_window);
  node->get_parameter(name + ".stair_anchor_min_area", stair_params_.anchor_min_area);
  node->get_parameter(name + ".stair_smooth_passes", stair_params_.smooth_passes);
  node->get_parameter(name + ".clearance_cost_weight", search_params_.clearance_cost_weight);
  node->get_parameter(name + ".clearance_cost_scaling", search_params_.clearance_cost_scaling);
  node->get_parameter(name + ".inferred_cost_multiplier", search_params_.inferred_cost_multiplier);
  node->get_parameter(name + ".heuristic_weight", search_params_.heuristic_weight);
  int max_iterations = static_cast<int>(search_params_.max_iterations);
  node->get_parameter(name + ".max_iterations", max_iterations);
  search_params_.max_iterations = static_cast<size_t>(std::max(1, max_iterations));
  node->get_parameter(name + ".max_planning_time", search_params_.max_planning_time);
  node->get_parameter(name + ".snap_xy_radius", search_params_.snap_xy_radius);
  node->get_parameter(name + ".snap_z_window", search_params_.snap_z_window);
  node->get_parameter(name + ".stair_cost_multiplier", search_params_.stair_cost_multiplier);
  // poses.txt sits next to octomap.bt in map/<name>/, so it is derived the
  // way the launch derives the tree from [map] pcd.
  if (poses_file_.empty() && !octomap_file_.empty()) {
    poses_file_ = posesFileBeside(octomap_file_);
  }

  if (goal_z_mode_ != "request" && goal_z_mode_ != "start") {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] goal_z_mode '%s' is not 'request' or 'start'; using 'request'",
      __func__, goal_z_mode_.c_str());
    goal_z_mode_ = "request";
  }
  if (publish_raw_plan_) {
    raw_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/raw_plan", 1);
  }
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] octomap_file '%s', body_frame '%s' (lidar_height %.3f), robot_radius "
    "%.2f, "
    "body_clearance %.2f, max_step_height %.2f, max_slope_deg %.1f, goal_z_mode %s",
    __func__, octomap_file_.c_str(), body_frame_.c_str(), lidar_height_, map_params_.robot_radius,
    map_params_.body_clearance, map_params_.max_step_height, map_params_.max_slope_deg,
    goal_z_mode_.c_str());

  // Registered last, like SmacPlanner2D: nothing above should run under it.
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OctomapPlanner3D::dynamicParametersCallback, this, std::placeholders::_1));

  // The map-switch reload (see the class comment). Named like map_server's
  // `<node>/load_map`, with the plugin id between, so the backend calls
  // /<robot_id>/planner_server/Octo3D/load_map with the same request type.
  // Created before the boot build starts, so a reload that arrives during it
  // waits for it (loadMapCallback joins build_thread_) instead of racing it.
  load_map_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  load_map_service_ = node->create_service<nav2_msgs::srv::LoadMap>(
    std::string(node->get_name()) + "/" + name_ + "/load_map",
    std::bind(
      &OctomapPlanner3D::loadMapCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, load_map_group_);
  clear_plan_client_ =
    node->create_client<std_srvs::srv::Empty>("clear_plan", rmw_qos_profile_services_default);
  load_map_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  load_map_executor_->add_callback_group(load_map_group_, node->get_node_base_interface());
  load_map_thread_ = std::make_unique<syncai_util::NodeThread>(load_map_executor_);

  if (octomap_file_.empty()) {
    RCLCPP_ERROR(
      logger_,
      "[OctomapPlanner3D][%s] no octomap_file set (the launch file derives it from [map] pcd); "
      "every plan with planner_id %s will fail with map_not_loaded until %s/load_map gives one",
      __func__, name_.c_str(), name_.c_str());
    build_error_ = "no octomap_file was given";
    return;
  }
  // The build takes seconds on a site map (the tree alone is 200-300 MB to
  // load); doing it here would hold up planner_server::configure() and with
  // it the other planner ids' action servers. Plans arriving meanwhile fail
  // with map_loading.
  building_ = true;
  build_thread_ = std::thread([this, octomap_file = octomap_file_, poses_file = poses_file_]() {
    std::string error;
    buildMap(octomap_file, poses_file, error);
  });
}

void OctomapPlanner3D::loadMapCallback(
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  using LoadMap = nav2_msgs::srv::LoadMap;
  const std::string octomap_file = request->map_url;
  RCLCPP_INFO(
    logger_, "[OctomapPlanner3D][%s] load_map request for '%s'", __func__, octomap_file.c_str());

  // Refused before anything is dropped: a typo in the request must not cost
  // the robot the map it has. Relative paths resolve against the workspace
  // root, every pane's cwd, as at boot.
  if (octomap_file.empty()) {
    RCLCPP_WARN(logger_, "[OctomapPlanner3D][%s] empty map_url; keeping the current map", __func__);
    response->result = LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return;
  }
  std::error_code ec;
  if (!std::filesystem::is_regular_file(octomap_file, ec)) {
    RCLCPP_WARN(
      logger_, "[OctomapPlanner3D][%s] '%s' does not exist; keeping the current map", __func__,
      octomap_file.c_str());
    response->result = LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return;
  }

  // The boot build may still be running; it would swap its map in after ours.
  if (build_thread_.joinable()) {
    build_thread_.join();
  }

  // On a reload poses.txt always comes from beside the new tree: an explicit
  // poses_file parameter named the boot map's track, not this one's.
  const std::string poses_file = posesFileBeside(octomap_file);

  // Drop the old site's map first. It must not plan once the robot is on
  // another map (a route through the wrong building), and freeing it before
  // the build keeps the peak to one tree. Waits for a plan in progress, which
  // holds mutex_ for its whole search.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.reset();
    links_.reset();
    astar_.reset();
    octomap_file_ = octomap_file;
    poses_file_ = poses_file;
    build_error_.clear();
    building_ = true;
  }

  std::string error;
  const bool ok = buildMap(octomap_file, poses_file, error);

  // Keep `ros2 param get` telling the truth about what is loaded. Outside
  // mutex_: the parameter callback takes it.
  if (auto node = node_.lock()) {
    updating_file_params_ = true;
    node->set_parameters(
      {rclcpp::Parameter(name_ + ".octomap_file", octomap_file),
       rclcpp::Parameter(name_ + ".poses_file", poses_file)});
    updating_file_params_ = false;
  }

  // planner_server re-sends its last plan at 1 Hz; after a map switch that is
  // a route on the old map. Fire and forget: the server's main executor
  // answers it, and a missing service only leaves a stale drawing.
  if (clear_plan_client_->service_is_ready()) {
    clear_plan_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  }

  if (ok) {
    response->result = LoadMap::Response::RESULT_SUCCESS;
  } else {
    // No fallback to the old map, for the reason above: it is gone already.
    response->result = LoadMap::Response::RESULT_INVALID_MAP_DATA;
    RCLCPP_ERROR(
      logger_, "[OctomapPlanner3D][%s] reload from '%s' failed: %s", __func__, octomap_file.c_str(),
      error.c_str());
  }
}

bool OctomapPlanner3D::buildMap(
  const std::string & octomap_file, const std::string & poses_file, std::string & error)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  std::shared_ptr<const o3d::SurfaceMap> map;
  o3d::BuildStats stats;
  error.clear();
  try {
    // Scoped so the tree is freed before the map is handed over: a 7 M node
    // tree is ~300 MB resident, the surface map ~30 MB.
    octomap::OcTree tree(0.1);
    if (o3d::loadOctomapBt(octomap_file, tree, error)) {
      RCLCPP_INFO(
        logger_, "[OctomapPlanner3D][%s] loaded %s: %zu nodes at %.3f m in %.2f s", __func__,
        octomap_file.c_str(), tree.size(), tree.getResolution(),
        std::chrono::duration<double>(Clock::now() - t0).count());
      map = o3d::buildSurfaceMap(tree, map_params_, &stats);
    }
  } catch (const std::exception & e) {
    error = std::string("building the surface map threw: ") + e.what();
    map.reset();
  }
  if (!map) {
    RCLCPP_ERROR(
      logger_,
      "[OctomapPlanner3D][%s] %s; every plan with planner_id %s will fail with map_not_loaded",
      __func__, error.c_str(), name_.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    build_error_ = error;
    building_ = false;
    return false;
  }
  const auto rej = [&](o3d::Reject r) { return stats.rejected[static_cast<size_t>(r)]; };
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] surface map: %d x %d columns at %.2f m, %zu surface cells (%zu "
    "inferred), "
    "%zu traversable in %zu components (largest %zu); rejected clearance_occupied %zu, "
    "no_free_above %zu, "
    "clearance_unknown %zu, step %zu, footprint_blocked %zu, slope %zu, residual %zu; built in "
    "%.2f s",
    __func__, map->nx, map->ny, map->res, map->cells.size(), stats.inferred, stats.traversable,
    stats.components, stats.component_sizes.empty() ? size_t{0} : stats.component_sizes.front(),
    rej(o3d::Reject::ClearanceOccupied), rej(o3d::Reject::NoFreeAbove),
    rej(o3d::Reject::ClearanceUnknown), rej(o3d::Reject::Step), rej(o3d::Reject::FootprintBlocked),
    rej(o3d::Reject::Slope), rej(o3d::Reject::Residual), stats.t_total);
  std::shared_ptr<const o3d::StairLinks> links;
  if (stair_links_) {
    std::vector<o3d::Waypoint> track;
    std::string track_error;
    if (o3d::readTrack(poses_file, lidar_height_, track, track_error)) {
      links = o3d::buildStairLinks(*map, track, stair_params_);
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] %zu stair links from %zu keyframes of %s (min_rise %.2f m)",
        __func__, links->links.size(), track.size(), poses_file.c_str(), stair_params_.min_rise);
      for (const auto & l : links->links) {
        RCLCPP_INFO(
          logger_,
          "[OctomapPlanner3D][%s]   stair link kf %zu -> %zu: z %.2f -> %.2f, %.1f m, %zu points",
          __func__, l.kf_a, l.kf_b, map->zTop(l.a), map->zTop(l.b), l.length, l.pts.size());
      }
      for (const std::string & note : links->notes) {
        RCLCPP_WARN(logger_, "[OctomapPlanner3D][%s] stair links: %s", __func__, note.c_str());
      }
    } else {
      RCLCPP_WARN(
        logger_,
        "[OctomapPlanner3D][%s] %s: no stair links, so the floors of this map do not connect",
        __func__, track_error.c_str());
    }
  }
  if (stats.traversable == 0) {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] the surface map has no traversable cell: check robot_radius / "
      "body_clearance against the map's resolution, or run octomap_planner_3d_cli --stats on it",
      __func__);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = map;
  links_ = links;
  astar_ = std::make_unique<o3d::SurfaceAStar>(map_, links_);
  building_ = false;
  return true;
}

bool OctomapPlanner3D::startHeightHint(double & z) const
{
  if (!tf_ || body_frame_.empty()) {
    return false;
  }
  try {
    const auto t = tf_->lookupTransform(global_frame_, body_frame_, tf2::TimePointZero);
    z = t.transform.translation.z - lidar_height_;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

nav_msgs::msg::Path OctomapPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // An empty path is how the planner server learns of a failure; the reason
  // goes to the log (and, through the BT's failure channel, to
  // NavigateToGoal.error_msg as PLAN_FAILED).
  const auto fail = [&](o3d::PlanFailure f, const std::string & detail) {
    RCLCPP_ERROR(
      logger_, "[OctomapPlanner3D][createPlan] %s: %s", o3d::planFailureName(f), detail.c_str());
    path.poses.clear();
    return path;
  };

  if (!map_) {
    if (building_) {
      return fail(
        o3d::PlanFailure::MapLoading, "the surface map is still being built from " + octomap_file_);
    }
    return fail(o3d::PlanFailure::MapNotLoaded, build_error_);
  }

  // The server's start pose is the planar map -> base_link, z = 0 always.
  double start_z = start.pose.position.z;
  double tf_z = 0.0;
  if (startHeightHint(tf_z)) {
    start_z = tf_z;
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[OctomapPlanner3D][createPlan] no TF %s -> %s; using the request's start z %.2f, which is 0 "
      "for the planar robot pose (fine on the ground floor, wrong upstairs)",
      global_frame_.c_str(), body_frame_.c_str(), start_z);
  }
  const double goal_z = goal_z_mode_ == "start" ? start_z : goal.pose.position.z;

  const o3d::SnapResult s =
    astar_->snap(start.pose.position.x, start.pose.position.y, start_z, search_params_);
  if (s.cell == o3d::kNoCell) {
    return fail(
      s.outside ? o3d::PlanFailure::StartOutsideMap : o3d::PlanFailure::StartUnavailable, s.detail);
  }
  const o3d::SnapResult g = astar_->snap(
    goal.pose.position.x, goal.pose.position.y, goal_z, search_params_,
    map_->cells[s.cell].component);
  if (g.cell == o3d::kNoCell) {
    return fail(
      g.outside ? o3d::PlanFailure::GoalOutsideMap : o3d::PlanFailure::GoalUnavailable, g.detail);
  }

  o3d::PlanResult r = astar_->plan(s.cell, g.cell, search_params_);
  if (r.failure != o3d::PlanFailure::None) {
    char buf[96];
    std::snprintf(buf, sizeof buf, " (%zu iterations, %.3f s)", r.iterations, r.search_s);
    return fail(r.failure, r.detail + buf);
  }

  const auto toPoses = [&](const std::vector<o3d::Waypoint> & wps, nav_msgs::msg::Path & out) {
    out.poses.clear();
    out.poses.reserve(wps.size());
    for (size_t i = 0; i < wps.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = out.header;
      pose.pose.position.x = wps[i].x;
      pose.pose.position.y = wps[i].y;
      pose.pose.position.z = wps[i].z + path_z_offset_;
      if (i + 1 < wps.size()) {
        pose.pose.orientation = syncai_util::geometry_utils::orientationAroundZAxis(
          std::atan2(wps[i + 1].y - wps[i].y, wps[i + 1].x - wps[i].x));
      } else {
        pose.pose.orientation = goal.pose.orientation;
      }
      out.poses.push_back(pose);
    }
  };

  if (raw_plan_pub_) {
    nav_msgs::msg::Path raw;
    raw.header = path.header;
    toPoses(o3d::buildPath(*map_, links_.get(), r.cells, r.links, false, map_->res), raw);
    raw_plan_pub_->publish(raw);
  }
  size_t vertices = 0;
  const auto wps = o3d::buildPath(
    *map_, links_.get(), r.cells, r.links, shortcut_, interpolation_resolution_, &vertices);
  toPoses(wps, path);

  double length = 0.0, zmin = wps.empty() ? 0.0 : wps.front().z, zmax = zmin;
  for (size_t i = 1; i < wps.size(); ++i) {
    length += std::hypot(wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y);
    zmin = std::min(zmin, wps[i].z);
    zmax = std::max(zmax, wps[i].z);
  }
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][createPlan] %zu poses, %.1f m, z %.2f..%.2f (start snapped %.2f m, goal "
    "%.2f m), "
    "%zu cells -> %zu vertices, %zu stair links, %zu iterations, search %.3f s, total %.3f s",
    path.poses.size(), length, zmin, zmax, s.xy_dist, g.xy_dist, r.cells.size(), vertices,
    r.links.size(), r.iterations, r.search_s,
    std::chrono::duration<double>(Clock::now() - t0).count());
  return path;
}

rcl_interfaces::msg::SetParametersResult OctomapPlanner3D::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::string prefix = name_ + ".";
  for (const auto & p : parameters) {
    const std::string & n = p.get_name();
    if (n.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string key = n.substr(prefix.size());
    const auto type = p.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double v = p.as_double();
      if (key == "clearance_cost_weight") {
        search_params_.clearance_cost_weight = v;
      } else if (key == "clearance_cost_scaling") {
        search_params_.clearance_cost_scaling = v;
      } else if (key == "inferred_cost_multiplier") {
        search_params_.inferred_cost_multiplier = v;
      } else if (key == "heuristic_weight") {
        search_params_.heuristic_weight = v;
      } else if (key == "max_planning_time") {
        search_params_.max_planning_time = v;
      } else if (key == "snap_xy_radius") {
        search_params_.snap_xy_radius = v;
      } else if (key == "snap_z_window") {
        search_params_.snap_z_window = v;
      } else if (key == "stair_cost_multiplier") {
        search_params_.stair_cost_multiplier = v;
      } else if (key == "interpolation_resolution") {
        interpolation_resolution_ = v;
      } else if (key == "path_z_offset") {
        path_z_offset_ = v;
      } else if (key == "lidar_height") {
        lidar_height_ = v;
      } else {
        result.successful = false;
        result.reason =
          key + " is baked into the surface map at startup; restart planner_server to apply";
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (key == "max_iterations") {
        search_params_.max_iterations = static_cast<size_t>(std::max<int64_t>(1, p.as_int()));
      } else {
        result.successful = false;
        result.reason =
          key + " is baked into the surface map at startup; restart planner_server to apply";
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (key == "shortcut") {
        shortcut_ = p.as_bool();
      } else {
        result.successful = false;
        result.reason = key + " cannot be changed at runtime";
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_STRING) {
      if (key == "goal_z_mode") {
        if (p.as_string() == "request" || p.as_string() == "start") {
          goal_z_mode_ = p.as_string();
        } else {
          result.successful = false;
          result.reason = "goal_z_mode must be 'request' or 'start'";
        }
      } else if (key == "body_frame") {
        body_frame_ = p.as_string();
      } else if ((key == "octomap_file" || key == "poses_file") && updating_file_params_) {
        // loadMapCallback() writing back what it loaded; the members are set.
      } else if (key == "octomap_file" || key == "poses_file") {
        result.successful = false;
        result.reason = "call " + name_ + "/load_map with the new octomap.bt to switch maps";
      } else {
        result.successful = false;
        result.reason = key + " cannot be changed at runtime";
      }
    }
  }
  return result;
}

}  // namespace syncai_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_global_planner::OctomapPlanner3D, syncai_nav_core::GlobalPlanner)
