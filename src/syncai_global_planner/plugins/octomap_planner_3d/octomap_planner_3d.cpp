#include "syncai_global_planner/plugins/octomap_planner_3d/octomap_planner_3d.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace syncai_global_planner
{

using syncai_util::declare_parameter_if_not_declared;
namespace o3d = octomap_planner_3d;

OctomapPlanner3D::~OctomapPlanner3D()
{
  // The scans first (they read the overlay and tf_). They arrive on the
  // global costmap's executor thread, which the planner server stops only
  // after the planners are gone, so there is no thread here to join:
  // disconnecting cloudCallback() waits for a call in progress and stops the
  // next one. Then the service thread: a reload in progress joins
  // build_thread_ too, and waits for its build to end before the thread
  // stops.
  cloud_connection_.disconnect();
  load_map_thread_.reset();
  if (build_thread_.joinable()) {
    build_thread_.join();
  }
}

void OctomapPlanner3D::publishMapAvailable(bool available)
{
  if (map_available_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = available;
    map_available_pub_->publish(msg);
  }
}

std::string OctomapPlanner3D::fileBeside(const std::string & octomap_file, const char * name)
{
  const auto slash = octomap_file.find_last_of('/');
  return (slash == std::string::npos ? std::string() : octomap_file.substr(0, slash + 1)) + name;
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
  dp("stair_straight", rclcpp::ParameterValue(stair_params_.straight));
  dp("stair_straight_max_flight", rclcpp::ParameterValue(stair_params_.straight_max_flight));
  dp("stair_straight_fit_radius", rclcpp::ParameterValue(stair_params_.straight_fit_radius));
  dp("stair_straight_half_width", rclcpp::ParameterValue(stair_params_.straight_half_width));
  dp("stair_straight_exit_reach", rclcpp::ParameterValue(stair_params_.straight_exit_reach));
  dp("stair_flights", rclcpp::ParameterValue(stair_params_.flights));
  dp(
    "stair_flights_landing_min_len", rclcpp::ParameterValue(stair_params_.flights_landing_min_len));
  dp(
    "stair_flights_landing_slope_deg",
    rclcpp::ParameterValue(stair_params_.flights_landing_slope_deg));
  dp("stair_flights_lead", rclcpp::ParameterValue(stair_params_.flights_lead));
  dp("stair_flights_turn_radius", rclcpp::ParameterValue(stair_params_.flights_turn_radius));
  dp("stair_centre", rclcpp::ParameterValue(stair_params_.centre));
  dp("stair_centre_dz", rclcpp::ParameterValue(stair_params_.centre_dz));
  dp("stair_centre_max_half_width", rclcpp::ParameterValue(stair_params_.centre_max_half_width));
  dp("stair_centre_max_shift", rclcpp::ParameterValue(stair_params_.centre_max_shift));
  dp("stair_detect", rclcpp::ParameterValue(stair_params_.detect));
  dp("stair_detect_floor_band", rclcpp::ParameterValue(stair_params_.detect_floor_band));
  dp("stair_detect_min_width", rclcpp::ParameterValue(stair_params_.detect_min_width));
  dp("stair_detect_min_area", rclcpp::ParameterValue(stair_params_.detect_min_area));
  // Search (dynamic).
  dp("clearance_cost_weight", rclcpp::ParameterValue(search_params_.clearance_cost_weight));
  dp("clearance_cost_scaling", rclcpp::ParameterValue(search_params_.clearance_cost_scaling));
  dp("inferred_cost_multiplier", rclcpp::ParameterValue(search_params_.inferred_cost_multiplier));
  dp("heuristic_weight", rclcpp::ParameterValue(search_params_.heuristic_weight));
  dp("max_iterations", rclcpp::ParameterValue(static_cast<int>(search_params_.max_iterations)));
  dp("max_planning_time", rclcpp::ParameterValue(search_params_.max_planning_time));
  dp("snap_xy_radius", rclcpp::ParameterValue(search_params_.snap_xy_radius));
  dp("snap_z_window", rclcpp::ParameterValue(search_params_.snap_z_window));
  dp("snap_single_layer", rclcpp::ParameterValue(search_params_.snap_single_layer));
  dp("stair_cost_multiplier", rclcpp::ParameterValue(search_params_.stair_cost_multiplier));
  // Dynamic obstacles (the overlay; its six doubles are dynamic).
  dp("dynamic.enabled", rclcpp::ParameterValue(dynamic_enabled_));
  dp("dynamic.topic", rclcpp::ParameterValue(dynamic_topic_));
  dp("dynamic.transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  dp("dynamic.obstacle_max_range", rclcpp::ParameterValue(dynamic_params_.obstacle_max_range));
  dp("dynamic.raytrace_max_range", rclcpp::ParameterValue(dynamic_params_.raytrace_max_range));
  dp("dynamic.self_radius", rclcpp::ParameterValue(dynamic_params_.self_radius));
  dp("dynamic.min_obstacle_height", rclcpp::ParameterValue(dynamic_params_.min_obstacle_height));
  dp("dynamic.obstacle_memory", rclcpp::ParameterValue(dynamic_params_.obstacle_memory));
  dp("dynamic.block_radius", rclcpp::ParameterValue(dynamic_params_.block_radius));
  // Keepout zones (keepout_zones.hpp): the file, and the ring (dynamic).
  dp("keepout.file", rclcpp::ParameterValue(std::string("")));
  dp("keepout.block_radius", rclcpp::ParameterValue(keepout_block_radius_));

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
  node->get_parameter(name + ".stair_straight", stair_params_.straight);
  node->get_parameter(name + ".stair_straight_max_flight", stair_params_.straight_max_flight);
  node->get_parameter(name + ".stair_straight_fit_radius", stair_params_.straight_fit_radius);
  node->get_parameter(name + ".stair_straight_half_width", stair_params_.straight_half_width);
  node->get_parameter(name + ".stair_straight_exit_reach", stair_params_.straight_exit_reach);
  node->get_parameter(name + ".stair_flights", stair_params_.flights);
  node->get_parameter(
    name + ".stair_flights_landing_min_len", stair_params_.flights_landing_min_len);
  node->get_parameter(
    name + ".stair_flights_landing_slope_deg", stair_params_.flights_landing_slope_deg);
  node->get_parameter(name + ".stair_flights_lead", stair_params_.flights_lead);
  node->get_parameter(name + ".stair_flights_turn_radius", stair_params_.flights_turn_radius);
  node->get_parameter(name + ".stair_centre", stair_params_.centre);
  node->get_parameter(name + ".stair_centre_dz", stair_params_.centre_dz);
  node->get_parameter(name + ".stair_centre_max_half_width", stair_params_.centre_max_half_width);
  node->get_parameter(name + ".stair_centre_max_shift", stair_params_.centre_max_shift);
  node->get_parameter(name + ".stair_detect", stair_params_.detect);
  node->get_parameter(name + ".stair_detect_floor_band", stair_params_.detect_floor_band);
  node->get_parameter(name + ".stair_detect_min_width", stair_params_.detect_min_width);
  node->get_parameter(name + ".stair_detect_min_area", stair_params_.detect_min_area);
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
  node->get_parameter(name + ".snap_single_layer", search_params_.snap_single_layer);
  node->get_parameter(name + ".stair_cost_multiplier", search_params_.stair_cost_multiplier);
  node->get_parameter(name + ".dynamic.enabled", dynamic_enabled_);
  node->get_parameter(name + ".dynamic.topic", dynamic_topic_);
  node->get_parameter(name + ".dynamic.transform_tolerance", transform_tolerance_);
  node->get_parameter(name + ".dynamic.obstacle_max_range", dynamic_params_.obstacle_max_range);
  node->get_parameter(name + ".dynamic.raytrace_max_range", dynamic_params_.raytrace_max_range);
  node->get_parameter(name + ".dynamic.self_radius", dynamic_params_.self_radius);
  node->get_parameter(name + ".dynamic.min_obstacle_height", dynamic_params_.min_obstacle_height);
  node->get_parameter(name + ".dynamic.obstacle_memory", dynamic_params_.obstacle_memory);
  node->get_parameter(name + ".dynamic.block_radius", dynamic_params_.block_radius);
  node->get_parameter(name + ".keepout.file", keepout_file_);
  node->get_parameter(name + ".keepout.block_radius", keepout_block_radius_);
  // poses.txt and keepout.json sit next to octomap.bt in map/<name>/, so
  // they are derived the way the launch derives the tree from [map] pcd.
  if (poses_file_.empty() && !octomap_file_.empty()) {
    poses_file_ = fileBeside(octomap_file_, "poses.txt");
  }
  if (keepout_file_.empty() && !octomap_file_.empty()) {
    keepout_file_ = fileBeside(octomap_file_, "keepout.json");
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
  // The dynamic overlay (see the class comment and dynamic_overlay.hpp)
  // exists before anything that can reach it -- the parameter callback
  // (setParams), load_map (setMap) and the boot build -- and with no map,
  // so the subscription and the service below come up once and stay; the
  // cloud callback does nothing until buildMap() binds a map.
  overlay_ = std::make_unique<o3d::DynamicOverlay>(dynamic_params_);
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] octomap_file '%s', body_frame '%s' (lidar_height %.3f), robot_radius "
    "%.2f, "
    "body_clearance %.2f, max_step_height %.2f, max_slope_deg %.1f, goal_z_mode %s",
    __func__, octomap_file_.c_str(), body_frame_.c_str(), lidar_height_, map_params_.robot_radius,
    map_params_.body_clearance, map_params_.max_step_height, map_params_.max_slope_deg,
    goal_z_mode_.c_str());

  // Registered last: nothing above should run under it.
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
  // The keepout reload, on the same group and thread (see the header): the
  // backend calls /<robot_id>/planner_server/Octo3D/load_keepout with the
  // same request type after it writes map/<name>/keepout.json.
  load_keepout_service_ = node->create_service<nav2_msgs::srv::LoadMap>(
    std::string(node->get_name()) + "/" + name_ + "/load_keepout",
    std::bind(
      &OctomapPlanner3D::loadKeepoutCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, load_map_group_);
  clear_plan_client_ =
    node->create_client<std_srvs::srv::Empty>("clear_plan", rmw_qos_profile_services_default);
  // Latched: the task_runner reads the last value whenever it subscribes.
  map_available_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    std::string(node->get_name()) + "/" + name_ + "/map_available",
    rclcpp::QoS(1).reliable().transient_local());
  load_map_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  load_map_executor_->add_callback_group(load_map_group_, node->get_node_base_interface());
  load_map_thread_ = std::make_unique<syncai_util::NodeThread>(load_map_executor_);

  // The overlay's input and clear (the overlay itself was made above).
  clear_overlay_service_ = node->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    std::string(node->get_name()) + "/" + name_ + "/clear_overlay",
    std::bind(
      &OctomapPlanner3D::clearOverlayCallback, this, std::placeholders::_1, std::placeholders::_2));
  if (dynamic_enabled_) {
    // On the global costmap's callback group, the one its executor thread
    // spins for the /tf listener that fills tf_ (costmap_2d_ros.cpp), as the
    // costmap layers' own filters on that buffer are. From 2026-10-21 to
    // 2026-10-22 the subscription had a group and thread of its own, so the
    // filter ran on two threads: add() on this one (registering a
    // transformable request and its timeout timer on tf_), the ready callback
    // -- and with it cloudCallback() -- on whichever thread delivered the
    // transform, the costmap's included. On robot01 (2026-10-22) the
    // costmap's thread stopped for good mid-goal: every thread of
    // planner_server in futex_wait, /tf no longer reaching tf_, the global
    // costmap's footprint stamped with the same second from then on. Since
    // getRobotPose() answers the newest transform in the buffer however old,
    // every later plan started where the robot had been, 20 m from where it
    // stood, and MPPI aborted each goal ("Resulting plan has 0 poses"). Most likely a
    // lock-order deadlock between the filter and tf2's request / timer
    // bookkeeping; no backtrace was taken. On one thread the filter cannot
    // interleave with itself. The price: the thread that takes /tf now also
    // runs addScan(), as the local costmap's terrain layer does with its
    // scans.
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = costmap_ros->getCallbackGroup();
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = 10;
    // The subscription is the costmap node's (a group belongs to one node),
    // whose namespace is /<robot_id>/global_costmap, so the topic is resolved
    // against this node first: relative, it is still
    // /<robot_id>/pointlio/body_cloud, the cloud the costmaps read.
    const std::string cloud_topic =
      node->get_node_topics_interface()->resolve_topic_name(dynamic_topic_);
    cloud_sub_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp::Node>>(
        costmap_ros.get(), cloud_topic, qos, sub_opt);
    cloud_sub_->unsubscribe();
    // The filter holds a cloud until its frame resolves in the map frame
    // (map <- pointlio_odom <- pointlio_body: only after relocalize).
    cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *cloud_sub_, *tf_, global_frame_, 10, node->get_node_logging_interface(),
      node->get_node_clock_interface(), tf2::durationFromSec(transform_tolerance_));
    cloud_connection_ = cloud_filter_->registerCallback(
      std::bind(&OctomapPlanner3D::cloudCallback, this, std::placeholders::_1));
    cloud_sub_->subscribe();
    RCLCPP_INFO(
      logger_,
      "[OctomapPlanner3D][%s] dynamic overlay on %s: marks returns %.2f-%.2f m above the surface "
      "within %.1f m, clears to %.1f m, ignores %.1f m round the sensor, memory %.0f s, block "
      "radius %.2f m",
      __func__, cloud_topic.c_str(), dynamic_params_.min_obstacle_height,
      map_params_.body_clearance, dynamic_params_.obstacle_max_range,
      dynamic_params_.raytrace_max_range, dynamic_params_.self_radius,
      dynamic_params_.obstacle_memory, dynamic_params_.block_radius);
  } else {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] dynamic overlay disabled (dynamic.enabled false): no dynamic "
      "obstacle is ever marked, every path reads valid and no replan detours",
      __func__);
  }

  if (octomap_file_.empty()) {
    RCLCPP_ERROR(
      logger_,
      "[OctomapPlanner3D][%s] no octomap_file set (the launch file derives it from [map] pcd); "
      "every plan with planner_id %s will fail with map_not_loaded until %s/load_map gives one",
      __func__, name_.c_str(), name_.c_str());
    build_error_ = "no octomap_file was given";
    publishMapAvailable(false);
    return;
  }
  // The build takes seconds on a site map (the tree alone is 200-300 MB to
  // load); doing it here would hold up planner_server::configure() and with
  // it the other planner ids' action servers. Plans arriving meanwhile fail
  // with map_loading.
  building_ = true;
  publishMapAvailable(true);
  build_thread_ = std::thread(
    [this, octomap_file = octomap_file_, poses_file = poses_file_, keepout_file = keepout_file_]() {
      std::string error;
      buildMap(octomap_file, poses_file, keepout_file, error);
    });
}

OctomapPlanner3D::KeepoutLoad OctomapPlanner3D::loadKeepout(
  const std::string & file, const o3d::SurfaceMap & map, const o3d::StairLinks * links,
  double block_radius, std::vector<o3d::KeepoutZone> & zones, o3d::KeepoutMaskPtr & mask) const
{
  std::error_code ec;
  if (file.empty() || !std::filesystem::is_regular_file(file, ec)) {
    RCLCPP_INFO(
      logger_, "[OctomapPlanner3D][%s] no keepout.json beside the tree (%s): no keepout zones",
      __func__, file.c_str());
    return KeepoutLoad::Absent;
  }
  std::vector<o3d::KeepoutZone> parsed;
  std::string error;
  if (!o3d::readKeepoutFile(file, parsed, error)) {
    RCLCPP_ERROR(
      logger_, "[OctomapPlanner3D][%s] keepout zones not loaded: %s", __func__, error.c_str());
    return KeepoutLoad::Failed;
  }
  o3d::KeepoutMaskPtr built = o3d::buildKeepoutMask(map, links, parsed, block_radius);
  // One line the smoke test greps for; keep its shape.
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] keepout zones from %s: %zu zones, %zu cells blocked (%zu core), %zu "
    "stair links cut (block radius %.2f m)",
    __func__, file.c_str(), parsed.size(), built ? built->blocked : size_t{0},
    built ? built->core : size_t{0}, built ? built->links_cut : size_t{0}, block_radius);
  for (const o3d::KeepoutZone & z : parsed) {
    RCLCPP_INFO(
      logger_, "[OctomapPlanner3D][%s]   zone %s: %zu vertices, z %.2f..%.2f", __func__,
      o3d::zoneLabel(z).c_str(), z.polygon.size(), z.z_min, z.z_max);
  }
  zones = std::move(parsed);
  mask = std::move(built);
  return KeepoutLoad::Loaded;
}

void OctomapPlanner3D::loadMapCallback(
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  using LoadMap = nav2_msgs::srv::LoadMap;
  const std::string octomap_file = request->map_url;
  RCLCPP_INFO(
    logger_, "[OctomapPlanner3D][%s] load_map request for '%s'", __func__, octomap_file.c_str());

  // An empty map_url unloads (2026-10-19): the switch to a 2D map. Without
  // it the 3D map stayed loaded across that switch, map_available stayed
  // true, and the task_runner kept running the 3D trees on the previous
  // site's octree -- a route through the wrong building, not a failure.
  // Waits for a boot build still running, as a load does.
  if (octomap_file.empty()) {
    if (build_thread_.joinable()) {
      build_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_.reset();
      links_.reset();
      astar_.reset();
      zones_.clear();
      keepout_.reset();
      overlay_->setMap(nullptr);
      octomap_file_.clear();
      poses_file_.clear();
      keepout_file_.clear();
      build_error_ = "no 3D map loaded (unloaded by load_map for a 2D map)";
      building_ = false;
    }
    publishMapAvailable(false);
    if (auto node = node_.lock()) {
      updating_file_params_ = true;
      node->set_parameters(
        {rclcpp::Parameter(name_ + ".octomap_file", std::string()),
         rclcpp::Parameter(name_ + ".poses_file", std::string()),
         rclcpp::Parameter(name_ + ".keepout.file", std::string())});
      updating_file_params_ = false;
    }
    if (clear_plan_client_->service_is_ready()) {
      clear_plan_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    }
    RCLCPP_INFO(
      logger_, "[OctomapPlanner3D][%s] empty map_url: 3D map unloaded (the map is 2D now)",
      __func__);
    response->result = LoadMap::Response::RESULT_SUCCESS;
    return;
  }

  // Refused before anything is dropped: a typo in the request must not cost
  // the robot the map it has. Relative paths resolve against the workspace
  // root, every pane's cwd, as at boot.
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

  // On a reload poses.txt and keepout.json always come from beside the new
  // tree: an explicit poses_file / keepout.file parameter named the boot
  // map's, not this one's.
  const std::string poses_file = fileBeside(octomap_file, "poses.txt");
  const std::string keepout_file = fileBeside(octomap_file, "keepout.json");

  // Drop the old site's map first. It must not plan once the robot is on
  // another map (a route through the wrong building), and freeing it before
  // the build keeps the peak to one tree. Waits for a plan in progress, which
  // holds mutex_ for its whole search.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.reset();
    links_.reset();
    astar_.reset();
    // The zones and the marks are the old site's too; buildMap() binds the
    // new map with its own zones and no marks.
    zones_.clear();
    keepout_.reset();
    overlay_->setMap(nullptr);
    octomap_file_ = octomap_file;
    poses_file_ = poses_file;
    keepout_file_ = keepout_file;
    build_error_.clear();
    building_ = true;
  }
  publishMapAvailable(true);

  std::string error;
  const bool ok = buildMap(octomap_file, poses_file, keepout_file, error);

  // Keep `ros2 param get` telling the truth about what is loaded. Outside
  // mutex_: the parameter callback takes it.
  if (auto node = node_.lock()) {
    updating_file_params_ = true;
    node->set_parameters(
      {rclcpp::Parameter(name_ + ".octomap_file", octomap_file),
       rclcpp::Parameter(name_ + ".poses_file", poses_file),
       rclcpp::Parameter(name_ + ".keepout.file", keepout_file)});
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
  const std::string & octomap_file, const std::string & poses_file,
  const std::string & keepout_file, std::string & error)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  std::shared_ptr<const o3d::SurfaceMap> map;
  o3d::BuildStats stats;
  error.clear();
  double keepout_block_radius;
  {
    // Written by the parameter callback under mutex_; this build is not.
    std::lock_guard<std::mutex> lock(mutex_);
    keepout_block_radius = keepout_block_radius_;
  }
  // Freed once the stair links are built, before the map is handed over: a
  // 7 M node tree is ~300 MB resident, the surface map ~30 MB. The links
  // read it for the walls round the landings' turns.
  auto tree = std::make_unique<octomap::OcTree>(0.1);
  try {
    if (o3d::loadOctomapBt(octomap_file, *tree, error)) {
      RCLCPP_INFO(
        logger_, "[OctomapPlanner3D][%s] loaded %s: %zu nodes at %.3f m in %.2f s", __func__,
        octomap_file.c_str(), tree->size(), tree->getResolution(),
        std::chrono::duration<double>(Clock::now() - t0).count());
      map = o3d::buildSurfaceMap(*tree, map_params_, &stats);
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      build_error_ = error;
      building_ = false;
    }
    publishMapAvailable(false);
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
    const bool have_track = o3d::readTrack(poses_file, lidar_height_, track, track_error);
    if (!have_track) {
      // Without the track only the detected stairs connect the floors.
      track.clear();
      RCLCPP_WARN(
        logger_, "[OctomapPlanner3D][%s] %s: no stair links from the track%s", __func__,
        track_error.c_str(),
        stair_params_.detect ? "; the stairs detected off the map only"
                             : ", so the floors of this map do not connect");
    }
    if (have_track || stair_params_.detect) {
      links = o3d::buildStairLinks(*map, track, stair_params_, tree.get());
      size_t detected = 0;
      for (const auto & l : links->links) {
        detected += l.detected ? 1 : 0;
      }
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] %zu stair links: %zu from %zu keyframes of %s, %zu detected off "
        "the map (min_rise %.2f m)",
        __func__, links->links.size(), links->links.size() - detected, track.size(),
        poses_file.c_str(), detected, stair_params_.min_rise);
      // "kf 2221 -> 2358", or "detected at (6.5, -12.6)" (its first point).
      const auto name = [](const o3d::StairLink & l) {
        char buf[80];
        if (l.detected) {
          std::snprintf(
            buf, sizeof buf, "detected at (%.1f, %.1f)", l.pts.front().x, l.pts.front().y);
        } else {
          std::snprintf(buf, sizeof buf, "kf %zu -> %zu", l.kf_a, l.kf_b);
        }
        return std::string(buf);
      };
      for (const auto & f : links->flights) {
        const auto & l = links->links[f.first];
        RCLCPP_INFO(
          logger_,
          "[OctomapPlanner3D][%s]   stair %s: z %.2f -> %.2f, %u straight links, "
          "uphill %.0f deg, %.1f m wide, %.1f m long",
          __func__, name(l).c_str(), map->zTop(l.a), map->zTop(l.b), f.count, f.heading_deg,
          f.width, l.length);
      }
      for (const auto & l : links->links) {
        if (l.straight) {
          continue;
        }
        RCLCPP_INFO(
          logger_,
          "[OctomapPlanner3D][%s]   stair link %s: z %.2f -> %.2f, %.1f m, %zu points (%s)",
          __func__, name(l).c_str(), map->zTop(l.a), map->zTop(l.b), l.length, l.pts.size(),
          l.flights ? l.geometry.c_str() : "walked track");
        if (l.centred > 0) {
          RCLCPP_INFO(
            logger_,
            "[OctomapPlanner3D][%s]     centred in the stairwell: %zu points moved, mean %.2f m, "
            "max %.2f m; narrowest cross-section %.2f m",
            __func__, l.centred, l.centre_mean_shift, l.centre_max_shift, l.centre_min_width);
        }
      }
      for (const std::string & note : links->notes) {
        RCLCPP_WARN(logger_, "[OctomapPlanner3D][%s] stair links: %s", __func__, note.c_str());
      }
    }
  }
  tree.reset();
  if (stats.traversable == 0) {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] the surface map has no traversable cell: check robot_radius / "
      "body_clearance against the map's resolution, or run octomap_planner_3d_cli --stats on it",
      __func__);
  }
  // The zones beside the tree. A broken file does not fail the build: the
  // old map is gone already (a reload drops it first), so failing here would
  // leave no map at all, which strands the robot worse than missing zones;
  // the error above says the zones protect nothing, and load_keepout takes
  // the fixed file without a rebuild.
  std::vector<o3d::KeepoutZone> zones;
  o3d::KeepoutMaskPtr keepout;
  loadKeepout(keepout_file, *map, links.get(), keepout_block_radius, zones, keepout);
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = map;
  links_ = links;
  astar_ = std::make_unique<o3d::SurfaceAStar>(map_, links_);
  zones_ = std::move(zones);
  keepout_ = std::move(keepout);
  // Bound empty: the scans start marking it from the next cloud on. setMap()
  // resets the overlay's keepout, so the mask goes in after it.
  overlay_->setMap(map_);
  overlay_->setKeepout(keepout_);
  building_ = false;
  return true;
}

void OctomapPlanner3D::loadKeepoutCallback(
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  using LoadMap = nav2_msgs::srv::LoadMap;
  const std::string file = request->map_url;
  RCLCPP_INFO(
    logger_, "[OctomapPlanner3D][%s] load_keepout request for '%s'", __func__, file.c_str());
  const auto writeBack = [&](const std::string & value) {
    if (auto node = node_.lock()) {
      updating_file_params_ = true;
      node->set_parameters({rclcpp::Parameter(name_ + ".keepout.file", value)});
      updating_file_params_ = false;
    }
  };
  // The boot build may still be running; its swap would put the file's
  // zones in after ours. A reload cannot: it runs on this thread.
  if (build_thread_.joinable()) {
    build_thread_.join();
  }

  // An empty map_url is "no zones": the mask goes, the file parameter too.
  if (file.empty()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      zones_.clear();
      keepout_.reset();
      keepout_file_.clear();
      overlay_->setKeepout(nullptr);
    }
    writeBack(std::string());
    RCLCPP_INFO(logger_, "[OctomapPlanner3D][%s] empty map_url: keepout zones cleared", __func__);
    response->result = LoadMap::Response::RESULT_SUCCESS;
    return;
  }
  // Refused before anything changes, as load_map refuses a missing tree.
  std::error_code ec;
  if (!std::filesystem::is_regular_file(file, ec)) {
    RCLCPP_WARN(
      logger_, "[OctomapPlanner3D][%s] '%s' does not exist; keeping the current zones", __func__,
      file.c_str());
    response->result = LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return;
  }
  // The map the mask is rasterised on. Copied out so the parse and the
  // rasterisation run outside mutex_ (a plan may hold it for seconds).
  std::shared_ptr<const o3d::SurfaceMap> map;
  std::shared_ptr<const o3d::StairLinks> links;
  double block_radius;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map = map_;
    links = links_;
    block_radius = keepout_block_radius_;
  }
  if (!map) {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] no 3D map loaded, nothing to rasterise '%s' on; a later load_map "
      "reads the keepout.json beside its tree",
      __func__, file.c_str());
    response->result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return;
  }
  std::vector<o3d::KeepoutZone> zones;
  o3d::KeepoutMaskPtr keepout;
  if (loadKeepout(file, *map, links.get(), block_radius, zones, keepout) != KeepoutLoad::Loaded) {
    // Unlike load_map there is no memory reason to drop first, and the
    // zones in force are safer than none: kept.
    response->result = LoadMap::Response::RESULT_INVALID_MAP_DATA;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_ != map) {
      // Cannot happen (a reload runs on this thread), but a mask indexes one
      // map's cells, so never swap it under another.
      RCLCPP_ERROR(
        logger_, "[OctomapPlanner3D][%s] the map changed during the load; zones not applied",
        __func__);
      response->result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
      return;
    }
    zones_ = std::move(zones);
    keepout_ = std::move(keepout);
    keepout_file_ = file;
    overlay_->setKeepout(keepout_);
  }
  writeBack(file);
  // No clear_plan: the current route is judged against the new zones by the
  // next is_path_valid (1 Hz), and move.xml replans after blocked_time.
  response->result = LoadMap::Response::RESULT_SUCCESS;
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

  // The start is snapped without the overlay: the robot stands where it
  // stands, and a person beside it must not make its own cell unavailable.
  const o3d::SnapResult s =
    astar_->snap(start.pose.position.x, start.pose.position.y, start_z, search_params_);
  if (s.cell == o3d::kNoCell) {
    return fail(
      s.outside ? o3d::PlanFailure::StartOutsideMap : o3d::PlanFailure::StartUnavailable, s.detail);
  }
  // A start inside a keepout zone's core: refused, as the 2D planner refuses
  // a start in a lethal cell. The search would otherwise walk out through
  // the core wherever its edge is within the exempt disc, which is the one
  // thing a zone must never allow; the operator drives the robot out.
  if (keepout_ && keepout_->core_(s.cell)) {
    const o3d::KeepoutZone * z = o3d::zoneAt(zones_, *map_, s.cell);
    return fail(
      o3d::PlanFailure::StartInKeepout, "the robot stands inside keepout zone " +
                                          (z ? o3d::zoneLabel(*z) : std::string("?")) +
                                          "; drive it out before planning");
  }
  // The dynamic obstacles as of now and the keepout zones, for this plan:
  // the goal snap, the search and the shortcut all read the one snapshot
  // and the one mask (keepout_ is immutable; a load_keepout swaps the
  // pointer under mutex_, which this holds). The disc around the start is
  // exempt from the marks' and the zones' dilation (not from a mark or a
  // core), so the first step away from a person standing close, or from a
  // zone edge, is legal.
  const o3d::DynamicMaskPtr mask = overlay_->snapshot(clock_->now().seconds());
  o3d::BlockedView blocked;
  blocked.mask = mask.get();
  blocked.keepout = keepout_.get();
  blocked.exempt = s.cell;
  blocked.exempt_radius =
    std::max(mask ? mask->block_radius : 0.0, keepout_ ? keepout_->block_radius : 0.0);
  const o3d::SnapResult g = astar_->snap(
    goal.pose.position.x, goal.pose.position.y, goal_z, search_params_,
    map_->cells[s.cell].component, &blocked);
  if (g.cell == o3d::kNoCell) {
    return fail(
      g.outside ? o3d::PlanFailure::GoalOutsideMap : o3d::PlanFailure::GoalUnavailable, g.detail);
  }
  // Said every time: a goal whose z is this far off is one the backend or
  // the console placed without the ground's height, and on a map whose
  // floors overlap elsewhere the same goal would have failed.
  if (s.z_fallback || g.z_fallback) {
    const o3d::SnapResult & f = g.z_fallback ? g : s;
    const double hint = g.z_fallback ? goal_z : start_z;
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][createPlan] %s z hint %.2f, ground there at z %.2f (%.2f m off, beyond "
      "snap_z_window %.2f): snapped to it as the only floor within %.2f m (snap_single_layer)",
      g.z_fallback ? "goal" : "start", hint, hint + f.dz, std::fabs(f.dz),
      search_params_.snap_z_window, search_params_.snap_xy_radius);
  }

  o3d::PlanResult r = astar_->plan(s.cell, g.cell, search_params_, blocked);
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
    toPoses(
      o3d::buildPath(*map_, links_.get(), r.cells, r.links, false, map_->res, nullptr, &blocked),
      raw);
    raw_plan_pub_->publish(raw);
  }
  size_t vertices = 0;
  const auto wps = o3d::buildPath(
    *map_, links_.get(), r.cells, r.links, shortcut_, interpolation_resolution_, &vertices,
    &blocked);
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
    "%zu cells -> %zu vertices, %zu stair links, %zu dynamic marks, %zu keepout zones, %zu "
    "iterations, search %.3f s, total %.3f s",
    path.poses.size(), length, zmin, zmax, s.xy_dist, g.xy_dist, r.cells.size(), vertices,
    r.links.size(), mask ? mask->marked : size_t{0}, keepout_ ? keepout_->zones : size_t{0},
    r.iterations, r.search_s, std::chrono::duration<double>(Clock::now() - t0).count());
  return path;
}

bool OctomapPlanner3D::isPathValid(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot_pose,
  std::vector<int32_t> & invalid_pose_indices)
{
  // Never mutex_: a plan in progress holds it for up to max_planning_time
  // and the BT gives this call 100 ms. The overlay carries its own copy of
  // the map pointer under its own lock, which is all this needs. body_frame_
  // and lidar_height_ (startHeightHint) and path_z_offset_ are written by
  // the parameter callback, which runs on the same executor as this
  // service, so they are read unlocked by design.
  const auto [map, keepout] = overlay_->mapAndKeepout();
  if (!map || path.poses.empty()) {
    return true;  // no 3D map: nothing to judge against, keep the path
  }
  const o3d::DynamicMaskPtr mask = overlay_->snapshot(clock_->now().seconds());
  if (!mask && !keepout) {
    return true;  // the fast path: not one mark alive and no zone
  }
  double rz = 0.0;
  if (!startHeightHint(rz)) {
    // The server's rule for "can't tell": valid, so the path is kept.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[OctomapPlanner3D][%s] no TF %s -> %s; cannot place the robot on a floor, keeping the "
      "path unchecked",
      __func__, global_frame_.c_str(), body_frame_.c_str());
    return true;
  }
  const o3d::SurfaceMap & m = *map;
  const auto & poses = path.poses;
  const double rx = robot_pose.pose.position.x, ry = robot_pose.pose.position.y;

  // The pose nearest the robot in 3D (the server's own nearest-pose search
  // is planar, with a robot z of 0: wrong on a stair whose turns lie over
  // each other), then the robot's own cell. No traversable cell under that
  // pose means the robot is on a stair (a flight's cells are all rejected)
  // or off the map's ground: valid, never a replan from there. A replan
  // whose start snaps mid-flight lands on the wrong floor, and a goal that
  // aborts on a flight stops the robot on the stair.
  size_t i0 = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto & p = poses[i].pose.position;
    const double d = (p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry) +
                     (p.z - path_z_offset_ - rz) * (p.z - path_z_offset_ - rz);
    if (d < best) {
      best = d;
      i0 = i;
    }
  }
  const auto cellOf = [&](size_t i) {
    const auto & p = poses[i].pose.position;
    return m.traversableAt(p.x, p.y, p.z - path_z_offset_);
  };
  const o3d::CellId c0 = cellOf(i0);
  if (c0 == o3d::kNoCell) {
    return true;
  }
  // Only the floor stretch the robot is on is judged: the check stops where
  // the path leaves traversable ground (a stair link), and the stretch
  // beyond is judged once the robot is on it. Judging it now would, with
  // blocked_time's 10 s wait, fire the replan while the robot is mid-flight
  // -- the case above. The disc around the robot's own cell is exempt from
  // the marks' and the zones' dilation, as it is for the planner, or a
  // person beside the robot would read "blocked 0.0 m ahead" on every call
  // with nothing a replan could change. A zone matters here only after a
  // load_keepout: a path planned with the zones never crosses one, but one
  // planned before they changed may, and this is what makes it replan.
  o3d::BlockedView blocked;
  blocked.mask = mask.get();
  blocked.keepout = keepout.get();
  blocked.exempt = c0;
  blocked.exempt_radius =
    std::max(mask ? mask->block_radius : 0.0, keepout ? keepout->block_radius : 0.0);
  size_t in_zones = 0;
  for (size_t i = i0; i < poses.size(); ++i) {
    const o3d::CellId c = i == i0 ? c0 : cellOf(i);
    if (c == o3d::kNoCell) {
      break;
    }
    if (blocked.blocked(m, c)) {
      invalid_pose_indices.push_back(static_cast<int32_t>(i));
      in_zones += blocked.blockedByZone(m, c);
    }
  }
  if (!invalid_pose_indices.empty()) {
    const size_t first = static_cast<size_t>(invalid_pose_indices.front());
    double ahead = 0.0;
    for (size_t i = i0; i < first; ++i) {
      ahead += syncai_util::geometry_utils::euclidean_distance(poses[i], poses[i + 1]);
    }
    RCLCPP_INFO(
      logger_,
      "[OctomapPlanner3D][%s] Path blocked: %zu of %zu poses ahead (%zu in keepout zones); first "
      "at index %zu, %.2f m ahead of the robot, (%.2f, %.2f, z %.2f); %zu cells marked by dynamic "
      "obstacles, %zu keepout zones",
      __func__, invalid_pose_indices.size(), poses.size() - i0, in_zones, first, ahead,
      poses[first].pose.position.x, poses[first].pose.position.y, poses[first].pose.position.z,
      mask ? mask->marked : size_t{0}, keepout ? keepout->zones : size_t{0});
    return false;
  }
  return true;
}

void OctomapPlanner3D::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  // No 3D map (a 2D map, or one still building): nothing to judge a return
  // against, and tens of thousands of points a second not to transform.
  if (!overlay_->map()) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(
      global_frame_, message->header.frame_id, message->header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[OctomapPlanner3D][%s] cannot transform %s into %s: %s", __func__,
      message->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  tf2::doTransform(*message, cloud, tf);
  std::vector<o3d::Waypoint> points;
  points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }
  // The cloud's frame is the lidar's own (the LIO body frame, the IMU a few
  // cm from the optical centre -- nothing at a 0.1 m cell), so the
  // transform's translation is the sensor origin the rays start from.
  const o3d::Waypoint origin{
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
  const o3d::ScanStats st =
    overlay_->addScan(points, origin, rclcpp::Time(message->header.stamp).seconds());
  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 5000,
    "[OctomapPlanner3D][%s] scan: %zu returns, %zu in range, %zu marked, %zu cleared; %zu cells "
    "marked in all",
    __func__, st.points, st.in_range, st.marked, st.cleared, overlay_->hotCount());
}

void OctomapPlanner3D::clearOverlayCallback(
  const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> /*request*/,
  std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> /*response*/)
{
  const size_t n = overlay_->clear();
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] dynamic overlay cleared: %zu marks dropped (keepout zones kept)",
    __func__, n);
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
      } else if (key == "keepout.block_radius") {
        // The zones are kept as parsed, so the ring is re-rasterised here
        // (milliseconds) and the mask swapped like a load_keepout's. mutex_
        // is held; the overlay lock after it is the one order ever taken.
        keepout_block_radius_ = v;
        if (map_) {
          keepout_ = o3d::buildKeepoutMask(*map_, links_.get(), zones_, keepout_block_radius_);
          overlay_->setKeepout(keepout_);
          RCLCPP_INFO(
            logger_,
            "[OctomapPlanner3D][%s] keepout.block_radius %.2f m: %zu zones, %zu cells blocked (%zu "
            "core), %zu stair links cut",
            __func__, v, zones_.size(), keepout_ ? keepout_->blocked : size_t{0},
            keepout_ ? keepout_->core : size_t{0}, keepout_ ? keepout_->links_cut : size_t{0});
        }
      } else if (key.rfind("dynamic.", 0) == 0) {
        // The overlay's own values take effect on the next scan / snapshot;
        // the transform tolerance is bound into the message filter.
        const std::string dkey = key.substr(std::string("dynamic.").size());
        o3d::DynamicOverlayParams dyn = dynamic_params_;
        if (dkey == "obstacle_max_range") {
          dyn.obstacle_max_range = v;
        } else if (dkey == "raytrace_max_range") {
          dyn.raytrace_max_range = v;
        } else if (dkey == "self_radius") {
          dyn.self_radius = v;
        } else if (dkey == "min_obstacle_height") {
          dyn.min_obstacle_height = v;
        } else if (dkey == "obstacle_memory") {
          dyn.obstacle_memory = v;
        } else if (dkey == "block_radius") {
          dyn.block_radius = v;
        } else {
          result.successful = false;
          result.reason = key + " cannot be changed at runtime";
        }
        if (result.successful) {
          dynamic_params_ = dyn;
          overlay_->setParams(dynamic_params_);
        }
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
      } else if (key == "snap_single_layer") {
        search_params_.snap_single_layer = p.as_bool();
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
      } else if (
        (key == "octomap_file" || key == "poses_file" || key == "keepout.file") &&
        updating_file_params_) {
        // loadMapCallback() / loadKeepoutCallback() writing back what they
        // loaded; the members are set.
      } else if (key == "octomap_file" || key == "poses_file") {
        result.successful = false;
        result.reason = "call " + name_ + "/load_map with the new octomap.bt to switch maps";
      } else if (key == "keepout.file") {
        result.successful = false;
        result.reason = "call " + name_ + "/load_keepout with the keepout.json to load zones";
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
