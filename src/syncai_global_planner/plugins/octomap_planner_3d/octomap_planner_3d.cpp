#include "syncai_global_planner/plugins/octomap_planner_3d/octomap_planner_3d.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/path_progress.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace syncai_global_planner
{

using syncai_util::declare_parameter_if_not_declared;
namespace o3d = octomap_planner_3d;

namespace
{

// A failure as the planner server reads it (2026-11): the typed
// syncai_nav_core exception it maps to ComputePathToPose's error_code, with
// the sentence the log line has as what(), which becomes error_msg. Until
// then every failure was an empty path and reached the backend as
// PLAN_FAILED, so a goal inside a keepout zone (permanent) and one a person
// stood on (worth a retry) read the same. The ROS-free core keeps its own
// PlanFailure; this is the one mapping. SearchLimit is a TIMEOUT: both are
// the search giving up on a budget, not proof that no path exists.
[[noreturn]] void throwPlanFailure(o3d::PlanFailure f, const std::string & what)
{
  namespace nc = syncai_nav_core;
  switch (f) {
    case o3d::PlanFailure::MapNotLoaded:
      throw nc::MapNotLoaded(what);
    case o3d::PlanFailure::MapLoading:
      throw nc::MapLoading(what);
    case o3d::PlanFailure::StartOutsideMap:
      throw nc::StartOutsideMapBounds(what);
    case o3d::PlanFailure::GoalOutsideMap:
      throw nc::GoalOutsideMapBounds(what);
    case o3d::PlanFailure::StartUnavailable:
      throw nc::StartOccupied(what);
    case o3d::PlanFailure::GoalUnavailable:
      throw nc::GoalOccupied(what);
    case o3d::PlanFailure::StartInKeepout:
      throw nc::StartInKeepout(what);
    case o3d::PlanFailure::GoalInKeepout:
      throw nc::GoalInKeepout(what);
    case o3d::PlanFailure::Disconnected:
      throw nc::GroundDisconnected(what);
    case o3d::PlanFailure::SearchLimit:
    case o3d::PlanFailure::Timeout:
      throw nc::PlannerTimedOut(what);
    case o3d::PlanFailure::NoPath:
      throw nc::NoValidPathCouldBeFound(what);
    default:
      throw nc::PlannerException(what);
  }
}

}  // namespace

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

std::string OctomapPlanner3D::fileBeside(const std::string & octomap_file, const char * name)
{
  const auto slash = octomap_file.find_last_of('/');
  return (slash == std::string::npos ? std::string() : octomap_file.substr(0, slash + 1)) + name;
}

void OctomapPlanner3D::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap::CostmapROS> costmap_ros)
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
  dp("narrow_max_width", rclcpp::ParameterValue(narrow_max_width_));
  dp("narrow_lead", rclcpp::ParameterValue(narrow_lead_));
  // Surface map (build time).
  dp("robot_radius", rclcpp::ParameterValue(map_params_.robot_radius));
  dp("body_clearance", rclcpp::ParameterValue(map_params_.body_clearance));
  dp("max_step_height", rclcpp::ParameterValue(map_params_.max_step_height));
  dp("max_slope_deg", rclcpp::ParameterValue(map_params_.max_slope_deg));
  // Declared since 2026-11: the params file has set it since 2026-10-14 and
  // nothing read it, so the struct's default (the same 0.8) was in force.
  dp("footprint_min_support", rclcpp::ParameterValue(map_params_.footprint_min_support));
  dp(
    "max_unknown_clearance_cells", rclcpp::ParameterValue(map_params_.max_unknown_clearance_cells));
  dp("min_known_free_above", rclcpp::ParameterValue(map_params_.min_known_free_above));
  dp("hole_fill_radius", rclcpp::ParameterValue(map_params_.hole_fill_radius));
  dp("hole_fill_min_support", rclcpp::ParameterValue(map_params_.hole_fill_min_support));
  dp("hole_fill_passes", rclcpp::ParameterValue(map_params_.hole_fill_passes));
  dp("clearance_cost_radius", rclcpp::ParameterValue(map_params_.clearance_cost_radius));
  dp("step_edge_cost_radius", rclcpp::ParameterValue(map_params_.step_edge_cost_radius));
  dp("kerb_min_length", rclcpp::ParameterValue(map_params_.kerb_min_length));
  dp("kerb_min_sure_cells", rclcpp::ParameterValue(map_params_.kerb_min_sure_cells));
  dp("measured_floor", rclcpp::ParameterValue(measured_floor_));
  dp("step_edge_min_rise", rclcpp::ParameterValue(map_params_.step_edge_min_rise));
  dp("floor_same_surface", rclcpp::ParameterValue(map_params_.floor_same_surface));
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
  // Kerb links (build time, stair_links.hpp).
  dp("kerb_links", rclcpp::ParameterValue(stair_params_.kerb_links));
  dp("kerb_lead", rclcpp::ParameterValue(stair_params_.kerb_lead));
  dp("kerb_fit_radius", rclcpp::ParameterValue(stair_params_.kerb_fit_radius));
  // Walked gaps (build time, stair_links.hpp).
  dp("gap_links", rclcpp::ParameterValue(stair_params_.gap_links));
  dp("gap_anchor_min_area", rclcpp::ParameterValue(stair_params_.gap_anchor_min_area));
  dp("gap_max_length", rclcpp::ParameterValue(stair_params_.gap_max_length));
  dp("gap_max_rise", rclcpp::ParameterValue(stair_params_.gap_max_rise));
  dp("stair_min_rise_window", rclcpp::ParameterValue(stair_params_.stair_min_rise_window));
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
  dp("step_edge_cost_weight", rclcpp::ParameterValue(search_params_.step_edge_cost_weight));
  // Declared since 2026-11 (the struct's 1.0 was the only value).
  dp("gap_cost_weight", rclcpp::ParameterValue(search_params_.gap_cost_weight));
  dp("step_edge_cost_scaling", rclcpp::ParameterValue(search_params_.step_edge_cost_scaling));
  dp("step_edge_one_cell", rclcpp::ParameterValue(search_params_.step_edge_one_cell));
  dp("kerb_cost_multiplier", rclcpp::ParameterValue(search_params_.kerb_cost_multiplier));
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
  node->get_parameter(name + ".narrow_max_width", narrow_max_width_);
  node->get_parameter(name + ".narrow_lead", narrow_lead_);
  node->get_parameter(name + ".robot_radius", map_params_.robot_radius);
  node->get_parameter(name + ".body_clearance", map_params_.body_clearance);
  node->get_parameter(name + ".max_step_height", map_params_.max_step_height);
  node->get_parameter(name + ".max_slope_deg", map_params_.max_slope_deg);
  node->get_parameter(name + ".footprint_min_support", map_params_.footprint_min_support);
  node->get_parameter(
    name + ".max_unknown_clearance_cells", map_params_.max_unknown_clearance_cells);
  node->get_parameter(name + ".min_known_free_above", map_params_.min_known_free_above);
  node->get_parameter(name + ".hole_fill_radius", map_params_.hole_fill_radius);
  node->get_parameter(name + ".hole_fill_min_support", map_params_.hole_fill_min_support);
  node->get_parameter(name + ".hole_fill_passes", map_params_.hole_fill_passes);
  node->get_parameter(name + ".clearance_cost_radius", map_params_.clearance_cost_radius);
  node->get_parameter(name + ".step_edge_cost_radius", map_params_.step_edge_cost_radius);
  node->get_parameter(name + ".kerb_min_length", map_params_.kerb_min_length);
  node->get_parameter(name + ".kerb_min_sure_cells", map_params_.kerb_min_sure_cells);
  node->get_parameter(name + ".measured_floor", measured_floor_);
  node->get_parameter(name + ".step_edge_min_rise", map_params_.step_edge_min_rise);
  node->get_parameter(name + ".floor_same_surface", map_params_.floor_same_surface);
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
  node->get_parameter(name + ".kerb_links", stair_params_.kerb_links);
  node->get_parameter(name + ".kerb_lead", stair_params_.kerb_lead);
  node->get_parameter(name + ".kerb_fit_radius", stair_params_.kerb_fit_radius);
  node->get_parameter(name + ".gap_links", stair_params_.gap_links);
  node->get_parameter(name + ".gap_anchor_min_area", stair_params_.gap_anchor_min_area);
  node->get_parameter(name + ".gap_max_length", stair_params_.gap_max_length);
  node->get_parameter(name + ".gap_max_rise", stair_params_.gap_max_rise);
  node->get_parameter(name + ".stair_min_rise_window", stair_params_.stair_min_rise_window);
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
  node->get_parameter(name + ".step_edge_cost_weight", search_params_.step_edge_cost_weight);
  node->get_parameter(name + ".gap_cost_weight", search_params_.gap_cost_weight);
  node->get_parameter(name + ".step_edge_cost_scaling", search_params_.step_edge_cost_scaling);
  node->get_parameter(name + ".step_edge_one_cell", search_params_.step_edge_one_cell);
  node->get_parameter(name + ".kerb_cost_multiplier", search_params_.kerb_cost_multiplier);
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
    // spins for the /tf listener that fills tf_ (costmap_ros.cpp), as the
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
    return;
  }
  // The build takes seconds on a site map (the tree alone is 200-300 MB to
  // load); doing it here would hold up planner_server::configure() and with
  // it the other planner ids' action servers. Plans arriving meanwhile fail
  // with map_loading.
  building_ = true;
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
  std::vector<size_t> hits;
  o3d::KeepoutMaskPtr built = o3d::buildKeepoutMask(map, links, parsed, block_radius, &hits);
  // One line the smoke test greps for; keep its shape.
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] keepout zones from %s: %zu zones, %zu cells blocked (%zu core), %zu "
    "stair links cut (block radius %.2f m)",
    __func__, file.c_str(), parsed.size(), built ? built->blocked : size_t{0},
    built ? built->core : size_t{0}, built ? built->links_cut : size_t{0}, block_radius);
  for (size_t i = 0; i < parsed.size(); ++i) {
    const o3d::KeepoutZone & z = parsed[i];
    if (hits[i] == 0) {
      // WARN, not INFO (2026-11): a zone that covers no floor forbids
      // nothing, and the summary above ("N zones, 0 cells blocked") said so
      // only in a number. The usual cause is a z band on no storey -- the
      // band is floor_z +- 0.5, and a floor_z off by a storey's height lands
      // between two floors.
      RCLCPP_WARN(
        logger_,
        "[OctomapPlanner3D][%s]   zone %s: %zu vertices, z %.2f..%.2f -- covers no floor and no "
        "stair: no surface of the map has its top within the z band inside the polygon (a band "
        "on the wrong storey, or a polygon off the map), so this zone forbids nothing",
        __func__, o3d::zoneLabel(z).c_str(), z.polygon.size(), z.z_min, z.z_max);
    } else {
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s]   zone %s: %zu vertices, z %.2f..%.2f, %zu core cells / "
        "stair links",
        __func__, o3d::zoneLabel(z).c_str(), z.polygon.size(), z.z_min, z.z_max, hits[i]);
    }
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
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const std::string octomap_file = request->map_url;
  RCLCPP_INFO(
    logger_, "[OctomapPlanner3D][%s] load_map request for '%s'", __func__, octomap_file.c_str());

  // An empty map_url unloads (2026-10-19): the backend's switch to a 2D map.
  // Without it the 3D map stayed loaded across that switch and goals kept
  // planning on the previous site's octree -- a route through the wrong
  // building, not a failure. With nothing to navigate a 2D map on since
  // 2026-10-28, an unloaded planner fails every goal with map_not_loaded.
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
    if (auto node = node_.lock()) {
      updating_file_params_ = true;
      node->set_parameters(
        {rclcpp::Parameter(name_ + ".octomap_file", std::string()),
         rclcpp::Parameter(name_ + ".poses_file", std::string()),
         rclcpp::Parameter(name_ + ".keepout.file", std::string())});
      updating_file_params_ = false;
    }
    requestClearPlan();
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

  requestClearPlan();

  if (ok) {
    response->result = LoadMap::Response::RESULT_SUCCESS;
    // The build's own lines above have the surface map, the links and the
    // zones; this one closes the request the backend is waiting on.
    RCLCPP_INFO(
      logger_, "[OctomapPlanner3D][%s] load_map '%s' done in %.2f s", __func__,
      octomap_file.c_str(), std::chrono::duration<double>(Clock::now() - t0).count());
  } else {
    // No fallback to the old map, for the reason above: it is gone already.
    response->result = LoadMap::Response::RESULT_INVALID_MAP_DATA;
    RCLCPP_ERROR(
      logger_, "[OctomapPlanner3D][%s] reload from '%s' failed: %s", __func__, octomap_file.c_str(),
      error.c_str());
  }
}

void OctomapPlanner3D::requestClearPlan()
{
  // planner_server re-sends its last plan at 1 Hz; after a map switch that is
  // a route on the old map. Fire and forget: the server's main executor
  // answers it, and a missing service only leaves a stale drawing -- said,
  // so a route on the old map in the console has an explanation.
  if (clear_plan_client_->service_is_ready()) {
    clear_plan_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  } else {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] clear_plan is not available; `plan` may show the old map's route "
      "until the next plan or plan_idle_clear_time",
      __func__);
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
      // The measured floor reads the map.pcd beside the tree (see
      // surface_map.hpp): the same directory, as poses.txt and keepout.json.
      const std::string pcd = measured_floor_ ? fileBeside(octomap_file, "map.pcd") : "";
      map = o3d::buildSurfaceMap(*tree, map_params_, &stats, pcd);
      if (!stats.floor_error.empty()) {
        RCLCPP_WARN(
          logger_,
          "[OctomapPlanner3D][%s] measured floor: %s; the step edges and kerbs read the voxel "
          "tops, so a kerb under ~0.2 m is not told from a slope's contour line",
          __func__, stats.floor_error.c_str());
      } else if (!pcd.empty()) {
        RCLCPP_INFO(
          logger_,
          "[OctomapPlanner3D][%s] measured floor from %s: %zu points, %zu in traversable cells, "
          "%zu of %zu traversable cells measured, in %.2f s",
          __func__, pcd.c_str(), stats.floor_points, stats.floor_points_kept, stats.floor_cells,
          stats.traversable, stats.t_floor);
      }
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
    return false;
  }
  const auto rej = [&](o3d::Reject r) { return stats.rejected[static_cast<size_t>(r)]; };
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] surface map: %d x %d columns at %.2f m, %zu surface cells (%zu "
    "inferred), "
    "%zu traversable in %zu components (largest %zu); rejected clearance_occupied %zu, "
    "no_free_above %zu, "
    "clearance_unknown %zu, step %zu, footprint_blocked %zu, slope %zu, residual %zu; %zu step "
    "edges (%zu of two cells or more), %zu kerbs of %zu cells; built in %.2f s",
    __func__, map->nx, map->ny, map->res, map->cells.size(), stats.inferred, stats.traversable,
    stats.components, stats.component_sizes.empty() ? size_t{0} : stats.component_sizes.front(),
    rej(o3d::Reject::ClearanceOccupied), rej(o3d::Reject::NoFreeAbove),
    rej(o3d::Reject::ClearanceUnknown), rej(o3d::Reject::Step), rej(o3d::Reject::FootprintBlocked),
    rej(o3d::Reject::Slope), rej(o3d::Reject::Residual), stats.step_edges, stats.sure_step_edges,
    stats.kerb_lines, stats.kerb_cells, stats.t_total);
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
    if (have_track || stair_params_.detect || stair_params_.kerb_links) {
      links = o3d::buildStairLinks(*map, track, stair_params_, tree.get());
      size_t detected = 0, kerb_links = 0, gap_kerbs = 0, gap_ramps = 0;
      for (const auto & l : links->links) {
        if (l.leads && l.evidence == o3d::CrossingEvidence::Track) {
          (l.kind == o3d::CrossingKind::Kerb ? gap_kerbs : gap_ramps) += 1;
        } else if (l.leads) {
          ++kerb_links;
        } else {
          detected += l.detected ? 1 : 0;
        }
      }
      const size_t stairs = links->links.size() - kerb_links - gap_kerbs - gap_ramps;
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] %zu stair links: %zu from %zu keyframes of %s, %zu detected off "
        "the map (min_rise %.2f m); %zu kerb links over %zu of the map's %zu kerbs (lead %.1f m)",
        __func__, stairs, stairs - detected, track.size(), poses_file.c_str(), detected,
        stair_params_.min_rise, kerb_links, links->kerbs, map->kerb_lines.size(),
        stair_params_.kerb_lead);
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] %zu gap links from the track (%zu kerb, %zu ramp: a rise of at "
        "most %.2f m crossed in at most %.1f m of track between surfaces of at least %.1f m2); "
        "%zu walked climbs refused (not onto a floor)",
        __func__, links->gap_links, gap_kerbs, gap_ramps, stair_params_.gap_max_rise,
        stair_params_.gap_max_length, stair_params_.gap_anchor_min_area, links->gap_refused);
      // "kf 2221 -> 2358", "detected at (6.5, -12.6)" (its first point),
      // "kerb at (6.5, -12.6)" (the cell the crossing steps up from), or
      // "gap kf 2992 -> 2994 (ramp) at (11.6, -23.0)" (its midpoint).
      const auto name = [](const o3d::StairLink & l) {
        char buf[96];
        if (l.leads && l.evidence == o3d::CrossingEvidence::Track) {
          std::snprintf(
            buf, sizeof buf, "gap kf %zu -> %zu (%s) at (%.1f, %.1f)", l.kf_a, l.kf_b,
            o3d::crossingKindName(l.kind), (l.pts[1].x + l.pts[2].x) / 2,
            (l.pts[1].y + l.pts[2].y) / 2);
        } else if (l.leads) {
          std::snprintf(buf, sizeof buf, "kerb at (%.1f, %.1f)", l.pts[1].x, l.pts[1].y);
        } else if (l.detected) {
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
        if (l.leads && l.evidence == o3d::CrossingEvidence::Track) {
          RCLCPP_INFO(
            logger_, "[OctomapPlanner3D][%s]   %s: z %.2f -> %.2f, rise %.2f m, %.1f m long",
            __func__, name(l).c_str(), map->zTop(l.a), map->zTop(l.b), l.rise, l.length);
          continue;
        }
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

bool OctomapPlanner3D::startHeightHint(double & z, std::string * error) const
{
  if (!tf_ || body_frame_.empty()) {
    if (error) {
      *error = "no TF buffer or no body_frame set";
    }
    return false;
  }
  try {
    const auto t = tf_->lookupTransform(global_frame_, body_frame_, tf2::TimePointZero);
    z = t.transform.translation.z - lidar_height_;
    return true;
  } catch (const tf2::TransformException & e) {
    if (error) {
      *error = e.what();
    }
    return false;
  }
}

double OctomapPlanner3D::scanAge() const
{
  const double last = last_scan_steady_.load();
  if (last < 0.0) {
    return -1.0;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
           .count() -
         last;
}

void OctomapPlanner3D::waitForScanAfterClear(const char * fn)
{
  // Why (2026-11): clear_overlay empties the overlay, and until the next
  // scan (~0.1 s) a plan sees nothing the lidar can see right now. The trees
  // clear at every new goal and in the planner's recovery, and the recovery
  // retries at once: on robot01 (2026-11-06) the retry of a goal_occupied
  // plan ran 1.5 ms after the clear, read 0 marks and planned straight
  // through the obstacle standing on the goal, so a goal someone stands on
  // went round the same loop (blocked, 10 s, replan, 206, clear, through)
  // instead of failing GOAL_OCCUPIED. One scan is enough: whatever it sees
  // is marked by it, and the marks the clear dropped that it does not see
  // (out of range, behind the robot) stay dropped -- which is what the
  // clear is for.
  //
  // Only while scans are arriving: with none for kStaleScan (the lidar or
  // its TF down, dynamic.enabled false, no 3D map) there is nothing to wait
  // for, and isPathValid() already says so. The bound is the message
  // filter's own wait for a cloud's transform plus two scan periods, so a
  // scan that is coming arrives within it; past it the plan goes ahead on
  // what it has, as before.
  constexpr double kStaleScan = 2.0;    // s, isPathValid()'s
  constexpr double kScanPeriods = 0.2;  // s, body_cloud is ~10 Hz
  const double age = scanAge();
  if (!dynamic_enabled_ || age < 0.0 || age > kStaleScan) {
    return;
  }
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const auto limit = std::chrono::duration<double>(transform_tolerance_ + kScanPeriods);
  std::unique_lock<std::mutex> scan_lock(scan_mutex_);
  if (!scan_pending_after_clear_) {
    return;
  }
  const bool got = scan_cv_.wait_for(
    scan_lock, std::chrono::duration_cast<Clock::duration>(limit),
    [this] { return !scan_pending_after_clear_; });
  const double waited = std::chrono::duration<double>(Clock::now() - t0).count();
  scan_lock.unlock();
  if (got) {
    RCLCPP_INFO(
      logger_, "[OctomapPlanner3D][%s] waited %.3f s for a scan after clear_overlay", fn, waited);
  } else {
    RCLCPP_WARN(
      logger_,
      "[OctomapPlanner3D][%s] no scan within %.2f s of clear_overlay; planning on the cleared "
      "overlay, which does not see what stands in view now",
      fn, waited);
  }
}

nav_msgs::msg::Path OctomapPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return planThrough(start, {goal});
}

nav_msgs::msg::Path OctomapPlanner3D::createPlanThroughPoses(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  if (goals.empty()) {
    // The planner server refuses an empty route before it gets here.
    throw syncai_nav_core::NoViapointsGiven("no waypoints in the request: nothing to plan");
  }
  return planThrough(start, goals);
}

nav_msgs::msg::Path OctomapPlanner3D::planThrough(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const bool through = goals.size() > 1;
  const char * fn = through ? "createPlanThroughPoses" : "createPlan";
  // Before mutex_: the wait must not hold up the parameter callback or a
  // map swap. Counted into the plan's total time in the summary line.
  waitForScanAfterClear(fn);
  std::lock_guard<std::mutex> lock(mutex_);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // A route through several waypoints (compute_path_through_poses, 2026-11)
  // is planned leg by leg here rather than by the GlobalPlanner default,
  // which calls createPlan() per leg: createPlan() takes the start's height
  // from TF (the robot's floor), so a leg starting at a waypoint upstairs
  // would snap its start onto the floor the robot is on; it would also check
  // "the robot stands in a zone" and exempt a disc round a waypoint the robot
  // is not at, and re-snap each joint, where the previous leg's goal snap
  // (round blocked cells) and the next leg's start snap (ignoring them) can
  // pick cells up to snap_xy_radius apart. Here leg 0 starts at the robot
  // exactly as a single plan does, and every later leg starts on the cell
  // the leg before ended on: the right floor, no jump at the joint, one
  // overlay snapshot for the whole route. A single goal is a one-leg route
  // and logs as it always did (`through` and `fn` above).

  // The request as this planner sees it, appended to a failure: the
  // detail of a search failure (no_path, disconnected, timeout) names no
  // coordinates, and the start's z is the TF's, not the request's. On a
  // route, which leg, from which waypoint.
  const auto where = [&](
                       size_t leg, double sx, double sy, double sz,
                       const geometry_msgs::msg::PoseStamped & goal, double goal_z) {
    char buf[224];
    if (through) {
      std::snprintf(
        buf, sizeof buf,
        " [leg %zu of %zu: %s (%.2f, %.2f, z %.2f) -> waypoint %zu (%.2f, %.2f, z %.2f)]", leg + 1,
        goals.size(), leg == 0 ? "start" : "waypoint", sx, sy, sz, leg + 1, goal.pose.position.x,
        goal.pose.position.y, goal_z);
    } else {
      std::snprintf(
        buf, sizeof buf, " [start (%.2f, %.2f, z %.2f) -> goal (%.2f, %.2f, z %.2f)]", sx, sy, sz,
        goal.pose.position.x, goal.pose.position.y, goal_z);
    }
    return std::string(buf);
  };
  std::string request = where(
    0, start.pose.position.x, start.pose.position.y, start.pose.position.z, goals.front(),
    goals.front().pose.position.z);

  // A failure is a typed exception (throwPlanFailure, 2026-11): the planner
  // server answers the goal with its error_code and this sentence as
  // error_msg, which NavigateToGoal carries to the backend. Logged here too,
  // as before, because this line is the planner's own record.
  const auto fail = [&](o3d::PlanFailure f, const std::string & detail) -> nav_msgs::msg::Path {
    const std::string what = std::string(o3d::planFailureName(f)) + ": " + detail + request;
    RCLCPP_ERROR(logger_, "[OctomapPlanner3D][%s] %s", fn, what.c_str());
    throwPlanFailure(f, what);
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
  std::string tf_error;
  if (startHeightHint(tf_z, &tf_error)) {
    start_z = tf_z;
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[OctomapPlanner3D][%s] no TF %s -> %s (%s); using the request's start z %.2f, "
      "which is 0 for the planar robot pose (fine on the ground floor, wrong upstairs)",
      fn, global_frame_.c_str(), body_frame_.c_str(), tf_error.c_str(), start_z);
  }
  request = where(
    0, start.pose.position.x, start.pose.position.y, start_z, goals.front(),
    goal_z_mode_ == "start" ? start_z : goals.front().pose.position.z);

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
  // zone edge, is legal. Only on the first leg: later legs start at a
  // waypoint whose cell the goal snap already chose clear of the dilation.
  const o3d::DynamicMaskPtr mask = overlay_->snapshot(clock_->now().seconds());
  o3d::BlockedView blocked;
  blocked.mask = mask.get();
  blocked.keepout = keepout_.get();
  blocked.exempt = s.cell;
  blocked.exempt_radius =
    std::max(mask ? mask->block_radius : 0.0, keepout_ ? keepout_->block_radius : 0.0);

  std::vector<o3d::Waypoint> wps;
  // The crossings the route takes, as plan_crossings tells the followers:
  // waypoint spans now, pose spans once the poses are made (1:1).
  std::vector<syncai_util::PlanCrossing> crossings;
  o3d::CellId leg_start = s.cell;
  double leg_start_z = start_z;
  double leg_sx = start.pose.position.x, leg_sy = start.pose.position.y;
  size_t vertices = 0, cells = 0, links = 0, iterations = 0;
  double search_s = 0.0, goal_snap_max = 0.0;
  for (size_t leg = 0; leg < goals.size(); ++leg) {
    const geometry_msgs::msg::PoseStamped & goal = goals[leg];
    const double goal_z = goal_z_mode_ == "start" ? leg_start_z : goal.pose.position.z;
    request = where(leg, leg_sx, leg_sy, leg_start_z, goal, goal_z);
    if (leg == 1) {
      blocked.exempt = o3d::kNoCell;
      blocked.exempt_radius = 0.0;
    }

    // A goal point inside a zone: refused as goal_in_keepout (2026-11),
    // before the snap, which keeps off the zones and their ring and would
    // move a goal up to snap_xy_radius - block_radius (0.25 m) inside a
    // zone's edge out of it and plan there, and fail one deeper as
    // goal_unavailable -- which reads as "someone may stand there", a
    // reason worth a retry, where a zone is permanent. The floor is the one
    // an unblocked snap finds under the point, as a zone's band names a
    // floor and a goal's z hint may be well off it; no floor under it at all
    // is left to the snap below to say.
    if (!zones_.empty()) {
      const o3d::SnapResult floor =
        astar_->snap(goal.pose.position.x, goal.pose.position.y, goal_z, search_params_);
      if (floor.cell != o3d::kNoCell) {
        if (
          const o3d::KeepoutZone * z = o3d::zoneAtPoint(
            zones_, goal.pose.position.x, goal.pose.position.y, map_->zTop(floor.cell))) {
          return fail(
            o3d::PlanFailure::GoalInKeepout, std::string(through ? "waypoint" : "the goal") +
                                               " lies inside keepout zone " + o3d::zoneLabel(*z) +
                                               "; move it out of the zone, or edit the zone");
        }
      }
    }
    // The goal the route ends on is never moved off a dynamic obstacle
    // (snapGoal(), 2026-11): one on it fails goal_unavailable -> 206
    // GOAL_OCCUPIED, which the backend retries. A pass-through waypoint of a
    // patrol still snaps round one: the robot only passes within
    // waypoint_passed_radius (1.0 m) of it, so moving it up to snap_xy_radius
    // changes nothing it is for, and failing the whole route because someone
    // stands on a point it drives through would.
    const bool last = leg + 1 == goals.size();
    const auto component = map_->cells[leg_start].component;
    const o3d::SnapResult g = last ? astar_->snapGoal(
                                       goal.pose.position.x, goal.pose.position.y, goal_z,
                                       search_params_, component, blocked)
                                   : astar_->snap(
                                       goal.pose.position.x, goal.pose.position.y, goal_z,
                                       search_params_, component, &blocked);
    if (g.cell == o3d::kNoCell) {
      return fail(o3d::goalSnapFailure(g), g.detail);
    }
    // Said every time: a goal whose z is this far off is one the backend or
    // the console placed without the ground's height, and on a map whose
    // floors overlap elsewhere the same goal would have failed.
    if ((leg == 0 && s.z_fallback) || g.z_fallback) {
      const o3d::SnapResult & f = g.z_fallback ? g : s;
      const double hint = g.z_fallback ? goal_z : start_z;
      RCLCPP_WARN(
        logger_,
        "[OctomapPlanner3D][%s] %s z hint %.2f, ground there at z %.2f (%.2f m off, beyond "
        "snap_z_window %.2f): snapped to it as the only floor within %.2f m "
        "(snap_single_layer)%s",
        fn, g.z_fallback ? (through ? "waypoint" : "goal") : "start", hint, hint + f.dz,
        std::fabs(f.dz), search_params_.snap_z_window, search_params_.snap_xy_radius,
        request.c_str());
    }

    o3d::PlanResult r = astar_->plan(leg_start, g.cell, search_params_, blocked);
    iterations += r.iterations;
    search_s += r.search_s;
    if (r.failure != o3d::PlanFailure::None) {
      char buf[96];
      std::snprintf(buf, sizeof buf, " (%zu iterations, %.3f s)", r.iterations, r.search_s);
      return fail(r.failure, r.detail + buf);
    }

    size_t leg_vertices = 0;
    std::vector<o3d::PathSpan> spans;
    const auto leg_wps = o3d::buildPath(
      *map_, links_.get(), r.cells, r.links, shortcut_, interpolation_resolution_, &leg_vertices,
      &blocked, &spans);
    // Each leg starts on the cell the one before ended on, so its first
    // waypoint repeats the joint: dropped, not driven twice.
    auto first = leg_wps.begin();
    if (
      !wps.empty() && first != leg_wps.end() && first->x == wps.back().x &&
      first->y == wps.back().y && first->z == wps.back().z) {
      ++first;
    }
    // Leg waypoint k is route waypoint base + k (the dropped joint, k = 0,
    // is the route's last one already in).
    const size_t base = wps.size() - static_cast<size_t>(first - leg_wps.begin());
    wps.insert(wps.end(), first, leg_wps.end());
    for (const o3d::PathSpan & sp : spans) {
      const o3d::StairLink & l = links_->links[r.links[sp.use].link];
      syncai_util::PlanCrossing x;
      x.kind = o3d::crossingKindName(l.kind);
      x.stair_gait = l.gait() == o3d::CrossingGait::Stair;  // the route's below
      x.evidence = o3d::crossingEvidenceName(l.evidence);
      x.rise = l.rise;
      x.first = base + sp.first;
      x.last = base + sp.last;
      for (const size_t t : sp.turns) {
        x.turns.push_back(base + t);
      }
      crossings.push_back(std::move(x));
    }
    vertices += leg_vertices;
    cells += r.cells.size();
    links += r.links.size();
    goal_snap_max = std::max(goal_snap_max, g.xy_dist);

    leg_start = g.cell;
    leg_start_z = goal_z + g.dz;
    leg_sx = goal.pose.position.x;
    leg_sy = goal.pose.position.y;
  }

  // Every pose faces the next one; the last takes the last goal's heading.
  // A waypoint's own heading is not used: the route passes through it.
  path.poses.reserve(wps.size());
  for (size_t i = 0; i < wps.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wps[i].x;
    pose.pose.position.y = wps[i].y;
    pose.pose.position.z = wps[i].z + path_z_offset_;
    if (i + 1 < wps.size()) {
      pose.pose.orientation = syncai_util::geometry_utils::orientationAroundZAxis(
        std::atan2(wps[i + 1].y - wps[i].y, wps[i + 1].x - wps[i].x));
    } else {
      pose.pose.orientation = goals.back().pose.orientation;
    }
    path.poses.push_back(pose);
  }
  {
    // The gait is the route's, not each link's: a kerb onto a stair's top
    // step is taken in the stair's gait (o3d::routeStairGaits). The legs
    // were planned in order, so the crossings are in route order.
    std::vector<o3d::RouteCrossing> rc;
    rc.reserve(crossings.size());
    for (const syncai_util::PlanCrossing & x : crossings) {
      rc.push_back({x.first, x.last, x.stair_gait});
    }
    o3d::routeStairGaits(wps, rc, stair_params_.min_rise, stair_params_.stair_min_rise_window);
    for (size_t i = 0; i < crossings.size(); ++i) {
      crossings[i].stair_gait = rc[i].stair;
    }
  }
  {
    // The narrow passages (path_postprocess.hpp), after the gaits: a passage
    // says how to drive a stretch, not which gait, so it takes no part in
    // routeStairGaits. Over the whole route, on the static map only -- a
    // person in a corridor is the dynamic overlay's, not a door. Sorted in
    // with the links' crossings, so the list stays in route order. What a
    // follower reads of it is the `slow` bit (2026-11-07); the kind, width
    // and heading are for the log.
    for (const o3d::NarrowSpan & n :
         o3d::findNarrowPassages(*map_, wps, narrow_max_width_, narrow_lead_)) {
      syncai_util::PlanCrossing x;
      x.kind = "narrow";
      x.stair_gait = false;
      x.slow = true;
      x.evidence = "geometry";
      x.rise = std::fabs(wps[n.last].z - wps[n.first].z);  // a height, as every link's
      x.first = n.first;
      x.last = n.last;
      x.width = n.width;
      x.heading = n.heading;
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] narrow passage %.2f m wide at (%.2f, %.2f) -> (%.2f, %.2f), axis "
        "%.0f deg (narrower than narrow_max_width %.2f; poses %zu..%zu)",
        fn, n.width, wps[n.first].x, wps[n.first].y, wps[n.last].x, wps[n.last].y,
        n.heading * 180.0 / M_PI, narrow_max_width_, n.first, n.last);
      crossings.push_back(std::move(x));
    }
    std::stable_sort(
      crossings.begin(), crossings.end(),
      [](const syncai_util::PlanCrossing & a, const syncai_util::PlanCrossing & b) {
        return a.first < b.first;
      });
  }
  {
    // The poses are the waypoints one for one, so a span carries over; its
    // points are the poses' own positions, path_z_offset and all, which is
    // what a follower holding this plan compares them with.
    syncai_util::PlanCrossings pc;
    pc.known = true;
    pc.stamp_sec = path.header.stamp.sec;
    pc.stamp_nanosec = path.header.stamp.nanosec;
    pc.frame_id = path.header.frame_id;
    pc.poses = path.poses.size();
    for (syncai_util::PlanCrossing & x : crossings) {
      if (x.first >= path.poses.size() || x.last >= path.poses.size()) {
        continue;  // cannot happen (1:1); never sent out of range if it does
      }
      for (size_t i = x.first; i <= x.last; ++i) {
        const auto & p = path.poses[i].pose.position;
        x.pts.push_back({p.x, p.y, p.z});
      }
      pc.crossings.push_back(std::move(x));
    }
    std::lock_guard<std::mutex> lock(crossings_mutex_);
    recent_crossings_.push_back(std::move(pc));
    while (recent_crossings_.size() > kRecentCrossings) {
      recent_crossings_.pop_front();
    }
  }

  double length = 0.0, zmin = wps.empty() ? 0.0 : wps.front().z, zmax = zmin;
  for (size_t i = 1; i < wps.size(); ++i) {
    length += std::hypot(wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y);
    zmin = std::min(zmin, wps[i].z);
    zmax = std::max(zmax, wps[i].z);
  }
  // The scan age says how much the dynamic marks can be trusted: "0 marks"
  // from an overlay that has had no scan for a minute is not a clear road.
  const double age = scanAge();
  char scan[48];
  if (!dynamic_enabled_) {
    std::snprintf(scan, sizeof scan, "overlay off");
  } else if (age < 0.0) {
    std::snprintf(scan, sizeof scan, "no scan yet");
  } else {
    std::snprintf(scan, sizeof scan, "last scan %.1f s ago", age);
  }
  if (!through) {
    request = where(
      0, start.pose.position.x, start.pose.position.y, start_z, goals.front(),
      goal_z_mode_ == "start" ? start_z : goals.front().pose.position.z);
  } else {
    char buf[96];
    std::snprintf(
      buf, sizeof buf, " [start (%.2f, %.2f, z %.2f) through %zu waypoints]", start.pose.position.x,
      start.pose.position.y, start_z, goals.size());
    request = buf;
  }
  RCLCPP_INFO(
    logger_,
    "[OctomapPlanner3D][%s] %zu poses, %.1f m, z %.2f..%.2f (start snapped %.2f m, %s "
    "%.2f m), %zu cells -> %zu vertices, %zu stair links, %zu dynamic marks (%s), %zu keepout "
    "zones, %zu iterations, search %.3f s, total %.3f s%s",
    fn, path.poses.size(), length, zmin, zmax, s.xy_dist,
    through ? "farthest waypoint snap" : "goal", goal_snap_max, cells, vertices, links,
    mask ? mask->marked : size_t{0}, scan, keepout_ ? keepout_->zones : size_t{0}, iterations,
    search_s, std::chrono::duration<double>(Clock::now() - t0).count(), request.c_str());
  return path;
}

std::string OctomapPlanner3D::planAnnotations(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(crossings_mutex_);
  for (const syncai_util::PlanCrossings & c : recent_crossings_) {
    if (c.isFor(path.header.stamp.sec, path.header.stamp.nanosec, path.poses.size())) {
      return syncai_util::toJson(c);
    }
  }
  return {};
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
  using Clock = std::chrono::steady_clock;
  // Every "valid" goes through here, so the end of a blocked spell is said
  // once, with how long it lasted -- whether the blocker left, a replan
  // went round it, or the check stopped judging (the robot on a stair).
  const auto valid = [&](const char * why) {
    if (blocked_since_) {
      RCLCPP_INFO(
        logger_, "[OctomapPlanner3D][isPathValid] Path clear again after %.1f s blocked%s",
        std::chrono::duration<double>(Clock::now() - *blocked_since_).count(), why);
      blocked_since_.reset();
    }
    return true;
  };
  const auto [map, keepout] = overlay_->mapAndKeepout();
  if (!map || path.poses.empty()) {
    return valid(" (no 3D map loaded)");  // nothing to judge against, keep the path
  }
  // An overlay without scans marks nothing, so every path reads valid and
  // move.xml never replans round anything: the robot drives into a person
  // until MPPI stops it. Said while a navigation is checking its path (this
  // service is called only then), at most every 10 s.
  if (dynamic_enabled_) {
    constexpr double kStaleScan = 2.0;  // s; body_cloud is ~10 Hz
    const double age = scanAge();
    if (age < 0.0 || age > kStaleScan) {
      char since[48] = "has never had a scan";
      if (age >= 0.0) {
        std::snprintf(since, sizeof since, "has had no scan for %.0f s", age);
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "[OctomapPlanner3D][isPathValid] dynamic overlay %s (topic %s, held until it transforms "
        "into %s within %.2f s): paths are judged without new marks -- is pointlio publishing, "
        "and the localizer's map frame up?",
        since, dynamic_topic_.c_str(), global_frame_.c_str(), transform_tolerance_);
    }
  }
  const o3d::DynamicMaskPtr mask = overlay_->snapshot(clock_->now().seconds());
  if (!mask && !keepout) {
    return valid("");  // the fast path: not one mark alive and no zone
  }
  double rz = 0.0;
  std::string tf_error;
  if (!startHeightHint(rz, &tf_error)) {
    // The server's rule for "can't tell": valid, so the path is kept.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[OctomapPlanner3D][%s] no TF %s -> %s (%s); cannot place the robot on a floor, keeping "
      "the path unchecked",
      __func__, global_frame_.c_str(), body_frame_.c_str(), tf_error.c_str());
    return valid(" (TF missing: path kept unchecked)");
  }
  const o3d::SurfaceMap & m = *map;
  const auto & poses = path.poses;
  const double rx = robot_pose.pose.position.x, ry = robot_pose.pose.position.y;

  // The pose nearest the robot in 3D (the server's own progress is planar,
  // with a robot z of 0: wrong on a stair whose turns lie over each other),
  // then the robot's own cell. No traversable cell under that pose means the
  // robot is on a stair (a flight's cells are all rejected) or off the map's
  // ground: valid, never a replan from there. A replan whose start snaps
  // mid-flight lands on the wrong floor, and a goal that aborts on a flight
  // stops the robot on the stair.
  // The first local minimum from the path's start, not the whole path's
  // minimum (2026-11, syncai_util/path_progress.hpp): the server hands over
  // the last plan already trimmed to where the robot is on it, and on a
  // patrol route the rest of the path can pass back within a cell of the
  // robot (A -> B -> A), whose obstacles the whole-path minimum would then
  // have judged in place of the leg being driven.
  const auto first_near = syncai_util::firstLocalMinimum(
    poses.begin(), poses.end(),
    [&](const geometry_msgs::msg::PoseStamped & ps) {
      const auto & p = ps.pose.position;
      const double dz = p.z - path_z_offset_ - rz;
      return std::sqrt((p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry) + dz * dz);
    },
    syncai_util::PathProgress::kDefaultHysteresis);
  const size_t i0 = static_cast<size_t>(std::distance(poses.begin(), first_near));
  const auto cellOf = [&](size_t i) {
    const auto & p = poses[i].pose.position;
    return m.traversableAt(p.x, p.y, p.z - path_z_offset_);
  };
  const o3d::CellId c0 = cellOf(i0);
  if (c0 == o3d::kNoCell) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 5000,
      "[OctomapPlanner3D][isPathValid] no traversable cell under path pose %zu (%.2f, %.2f, z "
      "%.2f), the nearest the robot: on a crossing or off the ground, path not judged",
      i0, poses[i0].pose.position.x, poses[i0].pose.position.y, poses[i0].pose.position.z);
    return valid(" (robot on a stair, a gap or off the map's ground: not judged there)");
  }
  // Only the floor stretch the robot is on is judged: the check stops where
  // the path leaves traversable ground (a stair link, or a gap link's
  // crossing -- the cells between its anchors are the ones the rules
  // rejected), and the stretch
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
    // Logged when the path first reads blocked, then every 5 s while it
    // stays so (the BT asks at 1 Hz and replans after blocked_time, 10 s):
    // where, what and for how long, without ten copies of it. A new plan
    // (another stamp) is a new spell.
    const auto now = Clock::now();
    const bool new_path = path.header.stamp != blocked_path_stamp_;
    blocked_path_stamp_ = path.header.stamp;
    const bool first_report = !blocked_since_ || new_path;
    if (first_report) {
      blocked_since_ = now;
    }
    if (first_report || now - blocked_logged_ >= std::chrono::seconds(5)) {
      blocked_logged_ = now;
      const size_t first = static_cast<size_t>(invalid_pose_indices.front());
      double ahead = 0.0;
      for (size_t i = i0; i < first; ++i) {
        ahead += syncai_util::geometry_utils::euclidean_distance(poses[i], poses[i + 1]);
      }
      RCLCPP_INFO(
        logger_,
        "[OctomapPlanner3D][%s] Path blocked for %.1f s: %zu of %zu poses ahead (%zu in keepout "
        "zones); first at index %zu, %.2f m ahead of the robot, (%.2f, %.2f, z %.2f); %zu cells "
        "marked by dynamic obstacles, %zu keepout zones",
        __func__, std::chrono::duration<double>(now - *blocked_since_).count(),
        invalid_pose_indices.size(), poses.size() - i0, in_zones, first, ahead,
        poses[first].pose.position.x, poses[first].pose.position.y, poses[first].pose.position.z,
        mask ? mask->marked : size_t{0}, keepout ? keepout->zones : size_t{0});
    }
    return false;
  }
  return valid("");
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
    // No timeout (2026-11; transform_tolerance_ until then): the message
    // filter released this cloud because the transform at its stamp is in
    // the buffer, and this callback runs on the thread that takes /tf -- a
    // wait here could only block the delivery of the transform it waits
    // for, up to transform_tolerance (0.3 s), stalling every TF consumer of
    // the planner. Missing anyway (a buffer cleared by a time jump between
    // the filter and here), the scan is dropped with the warning below.
    tf = tf_->lookupTransform(
      global_frame_, message->header.frame_id, message->header.stamp, tf2::durationFromSec(0.0));
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
  o3d::ScanStats st;
  {
    // Under scan_mutex_, so a clear cannot land between the add and the
    // flag: a scan added before a clear must not count as the one after it.
    std::lock_guard<std::mutex> scan_lock(scan_mutex_);
    st = overlay_->addScan(points, origin, rclcpp::Time(message->header.stamp).seconds());
    scan_pending_after_clear_ = false;
  }
  scan_cv_.notify_all();
  last_scan_steady_ =
    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
  size_t n = 0;
  {
    // The next plan waits for a scan added after this (waitForScanAfterClear()).
    std::lock_guard<std::mutex> scan_lock(scan_mutex_);
    n = overlay_->clear();
    scan_pending_after_clear_ = true;
  }
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
  // A runtime change is otherwise invisible in the log, and a plan after it
  // is read against the values the log last showed.
  std::string changed;
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
      } else if (key == "step_edge_cost_weight") {
        search_params_.step_edge_cost_weight = v;
      } else if (key == "gap_cost_weight") {
        search_params_.gap_cost_weight = v;
      } else if (key == "step_edge_cost_scaling") {
        search_params_.step_edge_cost_scaling = v;
      } else if (key == "step_edge_one_cell") {
        search_params_.step_edge_one_cell = v;
      } else if (key == "kerb_cost_multiplier") {
        search_params_.kerb_cost_multiplier = v;
      } else if (key == "interpolation_resolution") {
        interpolation_resolution_ = v;
      } else if (key == "path_z_offset") {
        path_z_offset_ = v;
      } else if (key == "narrow_max_width") {
        narrow_max_width_ = v;
      } else if (key == "narrow_lead") {
        narrow_lead_ = v;
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
    if (!result.successful) {
      break;
    }
    if (!updating_file_params_) {
      changed += (changed.empty() ? "" : ", ") + key + " = " + p.value_to_string();
    }
  }
  // rclcpp rejects the whole set on a refusal; the values before the refused
  // one are already in the members, so say which they are.
  if (!result.successful) {
    RCLCPP_WARN(
      logger_, "[OctomapPlanner3D][%s] parameter change refused: %s%s%s", __func__,
      result.reason.c_str(),
      changed.empty() ? "" : "; already applied in this plugin: ", changed.c_str());
  } else if (!changed.empty()) {
    RCLCPP_INFO(logger_, "[OctomapPlanner3D][%s] parameters set: %s", __func__, changed.c_str());
  }
  return result;
}

}  // namespace syncai_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_global_planner::OctomapPlanner3D, syncai_nav_core::GlobalPlanner)
