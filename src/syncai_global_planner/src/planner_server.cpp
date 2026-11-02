#include "syncai_global_planner/planner_server.hpp"

#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/plan_crossings.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace syncai_global_planner
{

namespace
{

// "(1.23, -4.56, z 0.00) in 'map'", for the goal / start lines.
std::string describePose(const geometry_msgs::msg::PoseStamped & p)
{
  char buf[128];
  std::snprintf(
    buf, sizeof(buf), "(%.2f, %.2f, z %.2f) in '%s'", p.pose.position.x, p.pose.position.y,
    p.pose.position.z, p.header.frame_id.c_str());
  return buf;
}

// The result's error_code for a planning failure: the typed exception a
// planner (or this server) threw, mapped to the ComputePathToPose table
// (2026-11). Both planner actions' results carry the same constants, hence
// the template. Anything else -- an untyped PlannerException, an empty path
// (validatePath() throws the base class), a std::exception a plugin was not
// meant to throw -- is PLAN_FAILED, the 2xx base: no finer reason. The most
// derived classes are tested first; all of them derive from
// PlannerException directly, so the order among them does not matter.
template <typename T>
bool isA(const std::exception & e)
{
  return dynamic_cast<const T *>(&e) != nullptr;
}

template <typename ResultT>
uint16_t planErrorCode(const std::exception & e)
{
  namespace nc = syncai_nav_core;
  if (isA<nc::InvalidPlanner>(e)) {
    return ResultT::INVALID_PLANNER;
  }
  if (isA<nc::PlannerTFError>(e)) {
    return ResultT::TF_ERROR;
  }
  if (isA<nc::StartOutsideMapBounds>(e)) {
    return ResultT::START_OUTSIDE_MAP;
  }
  if (isA<nc::GoalOutsideMapBounds>(e)) {
    return ResultT::GOAL_OUTSIDE_MAP;
  }
  if (isA<nc::StartOccupied>(e)) {
    return ResultT::START_OCCUPIED;
  }
  if (isA<nc::GoalOccupied>(e)) {
    return ResultT::GOAL_OCCUPIED;
  }
  if (isA<nc::PlannerTimedOut>(e)) {
    return ResultT::TIMEOUT;
  }
  if (isA<nc::NoValidPathCouldBeFound>(e)) {
    return ResultT::NO_VALID_PATH;
  }
  if (isA<nc::NoViapointsGiven>(e)) {
    return ResultT::NO_VIAPOINTS_GIVEN;
  }
  if (isA<nc::MapNotLoaded>(e)) {
    return ResultT::MAP_NOT_LOADED;
  }
  if (isA<nc::MapLoading>(e)) {
    return ResultT::MAP_LOADING;
  }
  if (isA<nc::StartInKeepout>(e)) {
    return ResultT::START_IN_KEEPOUT;
  }
  if (isA<nc::GoalInKeepout>(e)) {
    return ResultT::GOAL_IN_KEEPOUT;
  }
  if (isA<nc::GroundDisconnected>(e)) {
    return ResultT::DISCONNECTED;
  }
  return ResultT::PLAN_FAILED;
}

}  // namespace

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_server", options),
  gp_loader_("syncai_nav_core", "syncai_nav_core::GlobalPlanner"),
  default_ids_{"Octo3D"},
  default_types_{"syncai_global_planner/OctomapPlanner3D"}
{
  RCLCPP_INFO(this->get_logger(), "[PlannerServer][%s] Creating PlannerServer", __func__);

  // Declare this node's parameters
  this->declare_parameter("planner_plugins", default_ids_);
  this->declare_parameter("expected_planner_frequency", 1.0);
  this->declare_parameter("plan_republish_rate", 1.0);
  this->declare_parameter("plan_idle_clear_time", 30.0);

  this->get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      this->declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap. The CostmapROS constructor only declares
  // parameters; the heavy setup happens in its init() called from configure().
  costmap_ros_ = std::make_shared<syncai_costmap::CostmapROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
  action_server_pose_.reset();
  action_server_poses_.reset();
  action_server_preview_.reset();
  dyn_params_handler_.reset();
  planners_.clear();
  if (costmap_ros_) {
    costmap_ros_->deactivate();
  }
  costmap_thread_.reset();
}

void PlannerServer::configure()
{
  RCLCPP_INFO(this->get_logger(), "[PlannerServer][%s] Configuring PlannerServer", __func__);

  costmap_ros_->init();

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<syncai_util::NodeThread>(costmap_ros_);

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = this->shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = syncai_util::get_plugin_type_param(node, planner_ids_[i]);
      syncai_nav_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        this->get_logger(), "[PlannerServer][%s] Created global planner plugin %s of type %s",
        __func__, planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->initialize(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        this->get_logger(), "[PlannerServer][%s] Failed to create global planner. Exception: %s",
        __func__, ex.what());
      exit(-1);
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    this->get_logger(), "[PlannerServer][%s] Planner Server has %s planners available.", __func__,
    planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "[PlannerServer][%s] expected_planner_frequency is %.4f Hz; it must be greater than 0 for "
      "the slow-plan warning, which is now off",
      __func__, expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 1);
  // Reliable and latched: a controller that starts after the plan was made
  // still gets the one for the plan it is handed (the stamp ties the two).
  crossings_publisher_ = this->create_publisher<std_msgs::msg::String>(
    syncai_util::kPlanCrossingsTopic, rclcpp::QoS(1).reliable().transient_local());

  // Since move.xml plans only when the current path is blocked (2026-10), a
  // plan is published once per route instead of once a second, so a viewer
  // that subscribes mid-drive (rviz opened late, a reconnect) saw nothing until
  // the next replan, which may never come. Re-sending the last plan keeps
  // `plan` showing the route being driven (the part still ahead; see
  // republishPlan()). Nothing here knows when the navigation that asked for it
  // finished, so the task_runner reports a success or a cancel through
  // clear_plan below; after a failure the route stays up for
  // plan_idle_clear_time (see republishPlan()). 0 disables, leaving the
  // plan-time publish only.
  plan_idle_clear_time_.store(get_parameter("plan_idle_clear_time").as_double());
  double plan_republish_rate;
  get_parameter("plan_republish_rate", plan_republish_rate);
  if (plan_republish_rate > 0.0) {
    plan_republish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / plan_republish_rate),
      std::bind(&PlannerServer::republishPlan, this));
  }

  // On the node's main executor (main.cpp), not the action server's thread, so
  // a check never waits behind a plan in progress.
  is_path_valid_service_ = this->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(&PlannerServer::isPathValid, this, std::placeholders::_1, std::placeholders::_2));

  // Also on the main executor, which is the thread the republish timer runs
  // on, so a clear can never interleave with a republish in progress.
  clear_plan_service_ = this->create_service<std_srvs::srv::Empty>(
    "clear_plan",
    std::bind(&PlannerServer::clearPlan, this, std::placeholders::_1, std::placeholders::_2));

  // Create the action server for path planning to a pose. spin_thread=true
  // gives the server its own callback group and executor thread, so the
  // (blocking) execute callback never starves this node's main executor.
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    this->shared_from_this(), "compute_path_to_pose", std::bind(&PlannerServer::computePlan, this),
    nullptr, std::chrono::milliseconds(500), true);

  // The patrol route's planner (navigate_through_poses), on its own thread
  // likewise. Both actions plan under the planner's own lock, so a route and
  // a single plan never search at once.
  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    this->shared_from_this(), "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this), nullptr,
    std::chrono::milliseconds(500), true);

  // The console's route preview, on its own thread too. It plans under the
  // same planner lock, so it waits behind a navigation's search rather than
  // racing it (milliseconds: the lock is held for the search alone, not the
  // TF waits round it), but it never publishes on `plan` or replaces the
  // last plan: a preview
  // there would be drawn on every console as the route being driven, and
  // is_path_valid would judge a running navigation's path against it.
  action_server_preview_ = std::make_unique<ActionServerToPose>(
    this->shared_from_this(), "preview_path_to_pose", std::bind(&PlannerServer::previewPlan, this),
    nullptr, std::chrono::milliseconds(500), true);

  // Activation phase (nav2 on_activate)
  action_server_pose_->activate();
  action_server_poses_->activate();
  action_server_preview_->activate();
  costmap_ros_->activate();

  // Add callback for dynamic parameters
  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));
}

template <typename ServerT>
bool PlannerServer::isServerInactive(std::unique_ptr<ServerT> & server)
{
  if (server == nullptr || !server->is_server_active()) {
    RCLCPP_DEBUG(
      get_logger(), "[PlannerServer][%s] Action server unavailable or inactive. Stopping.",
      __func__);
    return true;
  }

  return false;
}

template <typename ServerT>
bool PlannerServer::isCancelRequested(std::unique_ptr<ServerT> & server)
{
  if (server->is_cancel_requested()) {
    RCLCPP_INFO(
      get_logger(), "[PlannerServer][%s] Goal was canceled. Canceling planning action.", __func__);
    server->terminate_all();
    return true;
  }

  return false;
}

template <typename ServerT, typename GoalT>
bool PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<ServerT> & server, std::shared_ptr<const GoalT> & goal)
{
  if (server->is_preempt_requested()) {
    goal = server->accept_pending_goal();
    return true;
  }
  return false;
}

template <typename GoalT>
void PlannerServer::getStartPose(
  const std::shared_ptr<const GoalT> & goal, geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    // Refused, not honoured (2026-11): Octo3D takes the start's height from
    // TF to the robot's body frame whatever the request says, because the
    // robot pose below is the planar map -> base_link with z = 0. A start
    // somewhere else would be planned at its x / y on the robot's floor --
    // the wrong floor, or start_unavailable, on a two-storey map -- and the
    // start disc's exemption from the dynamic overlay and the "robot stands
    // inside a keepout zone" check both assume the start is the robot. No
    // caller sets it (the BT nodes' `start` port went with this; the
    // backend's preview sends false), so this is a refusal with the reason
    // rather than a quiet plan from the robot. The field stays: it is
    // syncai_common's, nav2's goal field for field.
    throw syncai_nav_core::PlannerException(
      "use_start is not supported: a plan always starts at the robot's pose (Octo3D reads the "
      "start's floor from the robot's TF); send use_start false");
  } else if (!costmap_ros_->getRobotPose(start)) {
    // The TF helper logs the lookup's own error just above; this says what
    // it cost, in the goal's error_msg and in abortPlan()'s line.
    throw syncai_nav_core::PlannerTFError(
      "no robot pose (" + costmap_ros_->getBaseFrameID() + " in " +
      costmap_ros_->getGlobalFrameID() +
      ") to start from -- is the localizer / lio_bridge publishing map -> odom -> base_link?");
  }
}

void PlannerServer::transformPoseToGlobalFrame(
  geometry_msgs::msg::PoseStamped & pose, const std::string & what)
{
  // Which pose, and from which frame: a goal sent in a frame nobody
  // publishes (a typo, a robot_id prefix on `map`) is the usual cause.
  const std::string from = pose.header.frame_id;
  if (!costmap_ros_->transformPoseToGlobalFrame(pose, pose)) {
    throw syncai_nav_core::PlannerTFError(
      "cannot transform the " + what + " pose from '" + from + "' to '" +
      costmap_ros_->getGlobalFrameID() + "'");
  }
}

void PlannerServer::validatePath(
  const std::string & goal_desc, const nav_msgs::msg::Path & path, const std::string & planner_id)
{
  if (path.poses.empty()) {
    // A planner that returns an empty path without throwing gave no reason:
    // PLAN_FAILED, the 2xx base. Octo3D throws a typed exception for every
    // failure (2026-11), so this is a plugin that does not.
    throw syncai_nav_core::PlannerException(
      "planner '" + (planner_id.empty() ? std::string("(none given)") : planner_id) +
      "' returned no path to " + goal_desc + " and gave no reason");
  }

  RCLCPP_DEBUG(
    get_logger(), "[PlannerServer][%s] Found valid path of size %zu to %s", __func__,
    path.poses.size(), goal_desc.c_str());
}

template <typename ServerT, typename ResultT>
void PlannerServer::abortPlan(
  std::unique_ptr<ServerT> & server, const std::shared_ptr<ResultT> & result,
  const std::exception & ex, const std::string & planner_id, const std::string & goal_desc,
  const rclcpp::Time & start_time)
{
  result->path.poses.clear();
  result->error_code = planErrorCode<ResultT>(ex);
  result->error_msg = ex.what();
  const double elapsed = (this->now() - start_time).seconds();
  if (dynamic_cast<const syncai_nav_core::PlannerException *>(&ex) != nullptr) {
    // The expected way to fail: the reason is the planner's (or this
    // server's) own sentence. Octo3D also logged it, with the request's
    // coordinates, just above.
    RCLCPP_WARN(
      get_logger(),
      "[PlannerServer][%s] Plan failed (error_code %u): planner '%s' to %s after %.3f s: %s",
      __func__, static_cast<unsigned>(result->error_code),
      planner_id.empty() ? "(none given)" : planner_id.c_str(), goal_desc.c_str(), elapsed,
      ex.what());
  } else {
    // A plugin is meant to fail with a PlannerException (or, with no reason,
    // an empty path), never to throw anything else: ERROR.
    RCLCPP_ERROR(
      get_logger(),
      "[PlannerServer][%s] Plan aborted (error_code %u): planner '%s' threw planning to %s after "
      "%.3f s: %s",
      __func__, static_cast<unsigned>(result->error_code),
      planner_id.empty() ? "(none given)" : planner_id.c_str(), goal_desc.c_str(), elapsed,
      ex.what());
  }
  server->terminate_current(result);
}

syncai_nav_core::GlobalPlanner * PlannerServer::findPlanner(const std::string & planner_id)
{
  auto it = planners_.find(planner_id);
  if (it != planners_.end()) {
    return it->second.get();
  }
  if (planners_.size() == 1 && planner_id.empty()) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "[PlannerServer][%s] No planners specified in action call. "
      "Server will use only plugin %s in server."
      " This warning will appear once.",
      __func__, planner_ids_concat_.c_str());
    return planners_.begin()->second.get();
  }
  RCLCPP_ERROR(
    get_logger(),
    "[PlannerServer][%s] planner '%s' is not a valid planner. "
    "Planner names are: %s",
    __func__, planner_id.c_str(), planner_ids_concat_.c_str());
  return nullptr;
}

syncai_nav_core::GlobalPlanner * PlannerServer::findPlannerOrThrow(const std::string & planner_id)
{
  syncai_nav_core::GlobalPlanner * planner = findPlanner(planner_id);
  if (planner == nullptr) {
    throw syncai_nav_core::InvalidPlanner(
      "planner '" + planner_id + "' is not loaded (loaded: " + planner_ids_concat_ + ")");
  }
  return planner;
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  // "Can't tell" has two answers here, and which one is right depends on what
  // the caller does with each. The BT (move.xml) replans on invalid and keeps
  // the path on valid. A wrong "invalid" therefore replaces a route nobody
  // has shown to be blocked, which is the oscillation this service exists to
  // stop; a wrong "valid" delays one check by a second. So the configuration
  // faults (empty path, wrong frame) answer invalid, loudly, and a momentary
  // TF miss answers valid: the alternative was a replan whose getStartPose()
  // fails on the same miss.
  response->is_valid = false;

  const auto & poses = request->path.poses;
  if (poses.empty()) {
    // One of the configuration faults above, so loud: silent, the BT replans
    // once a second with nothing in this log to say why.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[PlannerServer][%s] Asked to check an empty path; answering blocked so the BT replans",
      __func__);
    return;
  }
  // A navigation is running: its BT checks the path once a tick (1 Hz), so
  // the route is still the one being driven (republishPlan()).
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    last_plan_used_ = std::chrono::steady_clock::now();
  }
  if (request->path.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    RCLCPP_WARN(
      get_logger(), "[PlannerServer][%s] Path is in frame '%s', the costmap in '%s'", __func__,
      request->path.header.frame_id.c_str(), costmap_ros_->getGlobalFrameID().c_str());
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[PlannerServer][%s] No robot pose; keeping the current path unchecked", __func__);
    response->is_valid = true;
    return;
  }

  // The request carries no planner id, but the path the BT checks is the
  // last plan this server handed out, so the planner that made it is known
  // and judges its own path (2026-10-21; Octo3D on its dynamic overlay). The
  // empty-id case mirrors getPlan(): with one plugin loaded it is that
  // plugin, which also covers a check after clear_plan (Octo3D calls it on a
  // map switch). Until 2026-10-28 a path of a planner that reads the costmap
  // (GridBased) was judged here on the global costmap instead -- centre cell
  // against INSCRIBED, padded footprint perimeter against LETHAL -- and that
  // went with 2D navigation and the costmap's layers.
  std::string planner_id;
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    planner_id = last_plan_planner_id_;
  }
  auto planner_it = planners_.find(planner_id);
  if (planner_it == planners_.end() && planner_id.empty() && planners_.size() == 1) {
    planner_it = planners_.begin();
  }
  if (planner_it == planners_.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[PlannerServer][%s] No planner to judge the path (last plan's planner '%s'; loaded: %s); "
      "answering blocked so the BT replans",
      __func__, planner_id.c_str(), planner_ids_concat_.c_str());
    return;
  }
  // The path the BT checks is the last plan (same stamp and size): the
  // planner is handed only the part from the robot's place on, and the
  // indices it reports are shifted back to the request's. The planner still
  // finds the robot on what it is handed (Octo3D in 3D), but on a patrol
  // route a whole-path search there could match the leg coming back past
  // the robot and judge that one instead (2026-11). A path that is not the
  // last plan (none since a clear_plan) is handed over whole.
  size_t offset = 0;
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    if (
      !last_plan_.poses.empty() && request->path.header.stamp == last_plan_.header.stamp &&
      request->path.poses.size() == last_plan_.poses.size()) {
      offset = last_plan_progress_.update(last_plan_, robot_pose.pose, false);
    }
  }
  // The planner logs a blocked path itself, with what blocks it and for how
  // long (Octo3D: marks / zones, distance ahead); a second line here said
  // less, every second.
  if (offset == 0) {
    response->is_valid =
      planner_it->second->isPathValid(request->path, robot_pose, response->invalid_pose_indices);
    return;
  }
  nav_msgs::msg::Path ahead;
  ahead.header = request->path.header;
  ahead.poses.assign(request->path.poses.begin() + offset, request->path.poses.end());
  response->is_valid =
    planner_it->second->isPathValid(ahead, robot_pose, response->invalid_pose_indices);
  for (auto & i : response->invalid_pose_indices) {
    i += static_cast<int32_t>(offset);
  }
}

void PlannerServer::computePlan()
{
  planToPose(action_server_pose_, true);
}

void PlannerServer::previewPlan()
{
  planToPose(action_server_preview_, false);
}

void PlannerServer::planToPose(std::unique_ptr<ActionServerToPose> & server, bool remember)
{
  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  auto goal = server->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();
  // Which server, in every line below: a preview's failure is not a
  // navigation's, and reads as one in this log otherwise.
  const char * kind = remember ? "Plan" : "Preview";

  try {
    if (isServerInactive(server) || isCancelRequested(server)) {
      return;
    }

    if (getPreemptedGoalIfRequested(server, goal)) {
      RCLCPP_INFO(
        get_logger(), "[PlannerServer][%s] %s preempted by a new goal to %s", __func__, kind,
        describePose(goal->goal).c_str());
    }

    // What was asked, so the planner's success or failure line below can be
    // read against it (a ComputePathToPose result carries no reason, and the
    // BT logs only that the node failed).
    const std::string start_desc =
      goal->use_start ? describePose(goal->start) : std::string("the robot pose");
    RCLCPP_INFO(
      get_logger(), "[PlannerServer][%s] %s request: planner '%s', goal %s, start %s", __func__,
      kind, goal->planner_id.c_str(), describePose(goal->goal).c_str(), start_desc.c_str());

    // No wait for the costmap to be current (upstream's, and GridBased's
    // until 2026-10-28): no loaded planner reads its cells, and it has no
    // layers to be current with.

    // Use start pose if provided otherwise use current robot pose. Every
    // failure from here on is a throw, answered once, with its error_code,
    // by the catch below (2026-11).
    geometry_msgs::msg::PoseStamped start;
    getStartPose(goal, start);

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    transformPoseToGlobalFrame(start, "start");
    transformPoseToGlobalFrame(goal_pose, "goal");

    result->path = getPlan(start, goal_pose, goal->planner_id);
    validatePath(describePose(goal_pose), result->path, goal->planner_id);

    // Publish the plan for visualization purposes, and keep it as the route
    // being driven. A preview is answered to its caller only.
    if (remember) {
      publishPlan(result->path, goal->planner_id);
    }

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    // Said as a duration: plans are made once per goal and on a blocked
    // path, so "missed its desired rate" read as a loop that does not exist.
    const double max_duration = max_planner_duration_.load();
    if (max_duration > 0.0 && cycle_duration.seconds() > max_duration) {
      RCLCPP_WARN(
        get_logger(),
        "[PlannerServer][%s] Plan took %.3f s, over the %.3f s of expected_planner_frequency "
        "(%.1f Hz); a warning only",
        __func__, cycle_duration.seconds(), max_duration, 1 / max_duration);
    }

    server->succeeded_current(result);
  } catch (const std::exception & ex) {
    abortPlan(
      server, result, ex, goal->planner_id,
      (remember ? "" : "preview of ") + describePose(goal->goal), start_time);
  }
}

void PlannerServer::computePlanThroughPoses()
{
  auto start_time = this->now();

  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();

  // "waypoint 3 of 5 (x, y, z) ..." for the log lines below.
  const auto describeRoute = [](const ActionThroughPoses::Goal & g) {
    if (g.goals.empty()) {
      return std::string("no waypoints");
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%zu waypoints, ending at ", g.goals.size());
    return std::string(buf) + describePose(g.goals.back());
  };

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    if (getPreemptedGoalIfRequested(action_server_poses_, goal)) {
      RCLCPP_INFO(
        get_logger(), "[PlannerServer][%s] Preempted by a new route through %s", __func__,
        describeRoute(*goal).c_str());
    }

    const std::string start_desc =
      goal->use_start ? describePose(goal->start) : std::string("the robot pose");
    RCLCPP_INFO(
      get_logger(), "[PlannerServer][%s] Plan request: planner '%s', route through %s, start %s",
      __func__, goal->planner_id.c_str(), describeRoute(*goal).c_str(), start_desc.c_str());

    if (goal->goals.empty()) {
      // What the navigator never leaves in remaining_goals (it keeps the
      // last waypoint), so a client sent an empty route.
      throw syncai_nav_core::NoViapointsGiven("a route with no waypoints");
    }

    geometry_msgs::msg::PoseStamped start;
    getStartPose(goal, start);
    transformPoseToGlobalFrame(start, "start");
    std::vector<geometry_msgs::msg::PoseStamped> goals = goal->goals;
    for (size_t i = 0; i < goals.size(); ++i) {
      transformPoseToGlobalFrame(goals[i], "waypoint " + std::to_string(i + 1));
    }

    result->path = findPlannerOrThrow(goal->planner_id)->createPlanThroughPoses(start, goals);
    validatePath(describeRoute(*goal), result->path, goal->planner_id);

    publishPlan(result->path, goal->planner_id);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;
    const double max_duration = max_planner_duration_.load();
    if (max_duration > 0.0 && cycle_duration.seconds() > max_duration) {
      RCLCPP_WARN(
        get_logger(),
        "[PlannerServer][%s] Plan took %.3f s, over the %.3f s of expected_planner_frequency "
        "(%.1f Hz); a warning only",
        __func__, cycle_duration.seconds(), max_duration, 1 / max_duration);
    }

    action_server_poses_->succeeded_current(result);
  } catch (const std::exception & ex) {
    abortPlan(
      action_server_poses_, result, ex, goal->planner_id, "a route through " + describeRoute(*goal),
      start_time);
  }
}

nav_msgs::msg::Path PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    get_logger(),
    "[PlannerServer][%s] Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).",
    __func__, start.pose.position.x, start.pose.position.y, goal.pose.position.x,
    goal.pose.position.y);

  return findPlannerOrThrow(planner_id)->createPlan(start, goal);
}

void PlannerServer::publishPlan(const nav_msgs::msg::Path & path, const std::string & planner_id)
{
  {
    // Stored even with no subscriber, so one that arrives later gets it from
    // the timer.
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    last_plan_ = path;
    last_plan_planner_id_ = planner_id;
    last_plan_progress_.reset();
    last_plan_used_ = std::chrono::steady_clock::now();
    clear_resends_left_ = 0;
  }
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }

  // Before the action's result goes back, so the controller usually has it
  // when FollowPath brings the path; a late one is matched on arrival (the
  // stamp ties it to the plan). A planner that says nothing is `known:
  // false`, which leaves the followers to their own judgement.
  syncai_nav_core::GlobalPlanner * planner = findPlanner(planner_id);
  std::string json = planner ? planner->planAnnotations(path) : std::string();
  if (json.empty()) {
    json = syncai_util::toJson(syncai_util::PlanCrossings{});
  }
  std_msgs::msg::String crossings;
  crossings.data = std::move(json);
  crossings_publisher_->publish(crossings);
}

void PlannerServer::republishPlan()
{
  // A route no navigation has used for plan_idle_clear_time is dropped
  // (2026-10-25). task_runner clears the route on a success or a cancel, but
  // not on a failure, where the operator wants to see where the robot gave
  // up -- and this timer then re-sent that route for good: on dp the console
  // still drew a failed goal's route 1.5 h after the task had ended, since a
  // re-send every second also kept the backend's 6 s expiry from firing.
  // Nothing here hears that a navigation failed, but every running one asks
  // is_path_valid each BT tick (1 Hz, move*.xml and dock*.xml), so a route
  // neither planned nor checked for that long belongs to no navigation.
  // Checked before the subscriber test, so a viewer that subscribes later
  // does not get a stale route either.
  bool idle = false;
  const double idle_clear_time = plan_idle_clear_time_.load();
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    idle = idle_clear_time > 0.0 && !last_plan_.poses.empty() &&
           std::chrono::steady_clock::now() - last_plan_used_ >
             std::chrono::duration<double>(idle_clear_time);
  }
  if (idle) {
    char reason[96];
    std::snprintf(
      reason, sizeof(reason),
      "no navigation planned or checked it for %.0f s (plan_idle_clear_time)", idle_clear_time);
    dropPlan(reason);
    return;
  }
  if (plan_publisher_->get_subscription_count() == 0) {
    return;
  }
  auto msg = std::make_unique<nav_msgs::msg::Path>();
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    if (last_plan_.poses.empty()) {
      if (clear_resends_left_ > 0) {
        --clear_resends_left_;
        msg->header.frame_id = costmap_ros_->getGlobalFrameID();
        msg->header.stamp = now();
        plan_publisher_->publish(std::move(msg));
      }
      return;
    }
    *msg = last_plan_;
  }
  // Trimmed to the part still ahead, from the robot's place on it: the
  // stored plan starts where the robot was when it was made, which with plans
  // now rare is often tens of metres back, and a viewer showing the driven
  // part reads it as the robot being off its route. last_plan_ itself stays
  // whole; the place is last_plan_progress_, which only moves forward, so a
  // patrol route that comes back past the robot is not trimmed to its later
  // leg (2026-11; until then a stateless whole-path nearest pose). No robot
  // pose (stale TF): the whole plan rather than nothing, since the route is
  // still what the robot is following.
  geometry_msgs::msg::PoseStamped robot_pose;
  if (
    msg->header.frame_id == costmap_ros_->getGlobalFrameID() &&
    costmap_ros_->getRobotPose(robot_pose)) {
    size_t place = 0;
    {
      std::lock_guard<std::mutex> lock(last_plan_mutex_);
      place = last_plan_progress_.update(*msg, robot_pose.pose, false);
    }
    msg->poses.erase(msg->poses.begin(), msg->poses.begin() + place);
  }
  // Restamped: the poses are in the fixed `map` frame, so the content is
  // still true now, and a viewer whose fixed frame is not `map` would
  // otherwise fail to transform a stamp that has aged out of its TF buffer.
  msg->header.stamp = now();
  plan_publisher_->publish(std::move(msg));
}

void PlannerServer::clearPlan(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  // The republish would otherwise keep a finished route on `plan` for good:
  // the backend forwards it to the console, which drew the last route until
  // the next goal replaced it.
  dropPlan("clear_plan called");
}

void PlannerServer::dropPlan(const std::string & reason)
{
  // An empty path is the clear both rviz and the backend's path subscriber
  // already understand. Its frame is the global frame, not empty: the backend
  // drops any `plan` whose frame is not `map` before it looks at the poses,
  // which would leave the old route up.
  constexpr int kClearResends = 3;
  {
    std::lock_guard<std::mutex> lock(last_plan_mutex_);
    last_plan_ = nav_msgs::msg::Path();
    last_plan_planner_id_.clear();
    last_plan_progress_.reset();
    // The immediate publish below is the first of the repeats; the timer sends
    // the rest, one per plan_republish_rate tick (none when it is 0).
    clear_resends_left_ = kClearResends - 1;
  }
  RCLCPP_INFO(get_logger(), "[PlannerServer][%s] Plan cleared: %s", __func__, reason.c_str());

  if (plan_publisher_->get_subscription_count() > 0) {
    auto msg = std::make_unique<nav_msgs::msg::Path>();
    msg->header.frame_id = costmap_ros_->getGlobalFrameID();
    msg->header.stamp = now();
    plan_publisher_->publish(std::move(msg));
  }
  // No plan, no crossings: the terrain layer stops waiving a route's steps
  // that nothing drives any more. Known and empty, for no plan (poses 0).
  syncai_util::PlanCrossings none;
  none.known = true;
  none.frame_id = costmap_ros_->getGlobalFrameID();
  std_msgs::msg::String crossings;
  crossings.data = syncai_util::toJson(none);
  crossings_publisher_->publish(crossings);
}

rcl_interfaces::msg::SetParametersResult PlannerServer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  // Startup only, refused before anything is applied (2026-11; accepted and
  // never used before, so a set read back as live): the republish timer is
  // created once with its period, and the planner plugins are loaded once.
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "plan_republish_rate" || name == "planner_plugins") {
      result.successful = false;
      result.reason = name +
                      " cannot be changed at runtime: it is read once at startup; change it in "
                      "the params file and restart planner_server";
      RCLCPP_WARN(get_logger(), "[PlannerServer][%s] %s", __func__, result.reason.c_str());
      return result;
    }
  }

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "plan_idle_clear_time") {
        // Read by the republish timer every tick, so a set applies at once.
        plan_idle_clear_time_.store(parameter.as_double());
      } else if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
          RCLCPP_INFO(
            get_logger(), "[PlannerServer][%s] expected_planner_frequency set to %.4f Hz", __func__,
            parameter.as_double());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "[PlannerServer][%s] expected_planner_frequency is %.4f Hz; it must be greater than 0 "
            "for the slow-plan warning, which is now off",
            __func__, parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace syncai_global_planner
