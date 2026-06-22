#include "syncai_planner/planner_server.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "syncai_util/node_utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace syncai_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_server", options),
  gp_loader_("syncai_nav_core", "syncai_nav_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"syncai_planner/NavfnPlanner"}
{
  RCLCPP_INFO(get_logger(), "[PlannerServer][%s] Creating PlannerServer", __func__);

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap. The Costmap2DROS constructor only declares
  // parameters; the heavy setup happens in its init() called from configure().
  costmap_ros_ = std::make_shared<syncai_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
  action_server_pose_.reset();
  action_server_poses_.reset();
  dyn_params_handler_.reset();
  planners_.clear();
  if (costmap_ros_) {
    costmap_ros_->deactivate();
  }
  costmap_thread_.reset();
  costmap_ = nullptr;
}

void PlannerServer::configure()
{
  RCLCPP_INFO(get_logger(), "[PlannerServer][%s] Configuring PlannerServer", __func__);

  costmap_ros_->init();
  costmap_ = costmap_ros_->getCostmap();

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<syncai_util::NodeThread>(costmap_ros_);

  RCLCPP_DEBUG(
    get_logger(), "[PlannerServer][%s] Costmap size: %d,%d", __func__, costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = syncai_util::get_plugin_type_param(node, planner_ids_[i]);
      syncai_nav_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "[PlannerServer][%s] Created global planner plugin %s of type %s", __func__,
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->initialize(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "[PlannerServer][%s] Failed to create global planner. Exception: %s",
        __func__, ex.what());
      exit(-1);
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "[PlannerServer][%s] Planner Server has %s planners available.", __func__,
    planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages",
      expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action server for path planning to a pose. spin_thread=true
  // gives the server its own callback group and executor thread, so the
  // (blocking) execute callback never starves this node's main executor.
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(), "compute_path_to_pose", std::bind(&PlannerServer::computePlan, this),
    nullptr, std::chrono::milliseconds(500), true);

  // Create the action server for path planning through an ordered set of poses.
  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(), "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this), nullptr,
    std::chrono::milliseconds(500), true);

  // Activation phase (nav2 on_activate)
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->activate();

  // Add callback for dynamic parameters
  dyn_params_handler_ =
    add_on_set_parameters_callback(std::bind(&PlannerServer::dynamicParametersCallback, this, _1));
}

bool PlannerServer::isServerInactive()
{
  if (action_server_pose_ == nullptr || !action_server_pose_->is_server_active()) {
    RCLCPP_DEBUG(
      get_logger(), "[PlannerServer][%s] Action server unavailable or inactive. Stopping.",
      __func__);
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

bool PlannerServer::isCancelRequested()
{
  if (action_server_pose_->is_cancel_requested()) {
    RCLCPP_INFO(
      get_logger(), "[PlannerServer][%s] Goal was canceled. Canceling planning action.", __func__);
    action_server_pose_->terminate_all();
    return true;
  }

  return false;
}

void PlannerServer::getPreemptedGoalIfRequested(std::shared_ptr<const ActionToPose::Goal> & goal)
{
  if (action_server_pose_->is_preempt_requested()) {
    goal = action_server_pose_->accept_pending_goal();
  }
}

bool PlannerServer::getStartPose(
  std::shared_ptr<const ActionToPose::Goal> goal, geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server_pose_->terminate_current();
    return false;
  }

  return true;
}

bool PlannerServer::transformPosesToGlobalFrame(
  geometry_msgs::msg::PoseStamped & curr_start, geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (
    !costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal)) {
    RCLCPP_WARN(
      get_logger(),
      "[PlannerServer][%s] Could not transform the start or goal pose in the costmap frame",
      __func__);
    action_server_pose_->terminate_current();
    return false;
  }

  return true;
}

bool PlannerServer::validatePath(
  const geometry_msgs::msg::PoseStamped & goal, const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(),
      "[PlannerServer][%s] Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)",
      __func__, planner_id.c_str(), goal.pose.position.x, goal.pose.position.y);
    action_server_pose_->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(), "[PlannerServer][%s] Found valid path of size %zu to (%.2f, %.2f)", __func__,
    path.poses.size(), goal.pose.position.x, goal.pose.position.y);

  return true;
}

void PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive() || isCancelRequested()) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(start, goal_pose)) {
      return;
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath(goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "[PlannerServer][%s] Planner loop missed its desired rate of %.4f Hz. Current loop rate is "
        "%.4f Hz",
        __func__, 1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "[PlannerServer][%s] %s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"", __func__,
      goal->planner_id.c_str(), goal->goal.pose.position.x, goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
}

void PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathThroughPoses goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (action_server_poses_ == nullptr || !action_server_poses_->is_server_active()) {
      RCLCPP_DEBUG(
        get_logger(), "[PlannerServer][%s] Action server unavailable or inactive. Stopping.",
        __func__);
      return;
    }

    if (action_server_poses_->is_cancel_requested()) {
      RCLCPP_INFO(
        get_logger(), "[PlannerServer][%s] Goal was canceled. Canceling planning action.",
        __func__);
      action_server_poses_->terminate_all();
      return;
    }

    waitForCostmap();

    if (action_server_poses_->is_preempt_requested()) {
      goal = action_server_poses_->accept_pending_goal();
    }

    if (goal->goals.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "[PlannerServer][%s] Compute path through poses requested a plan with no viapoint poses,"
        " returning.",
        __func__);
      action_server_poses_->terminate_current();
      return;
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (goal->use_start) {
      start = goal->start;
    } else if (!costmap_ros_->getRobotPose(start)) {
      action_server_poses_->terminate_current();
      return;
    }

    // Plan each segment (start -> goals[0] -> goals[1] -> ...) and concatenate
    for (size_t i = 0; i < goal->goals.size(); ++i) {
      geometry_msgs::msg::PoseStamped curr_start = (i == 0) ? start : goal->goals[i - 1];
      geometry_msgs::msg::PoseStamped curr_goal = goal->goals[i];

      // Transform them into the global frame (inlined rather than reusing the
      // ComputePathToPose helpers, which terminate the wrong action server).
      if (
        !costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
        !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal)) {
        RCLCPP_WARN(
          get_logger(),
          "[PlannerServer][%s] Could not transform the start or goal pose in the costmap frame",
          __func__);
        action_server_poses_->terminate_current();
        return;
      }

      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      if (curr_path.poses.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "[PlannerServer][%s] Planning algorithm %s failed to generate a valid path to"
          " (%.2f, %.2f)",
          __func__, goal->planner_id.c_str(), curr_goal.pose.position.x, curr_goal.pose.position.y);
        action_server_poses_->terminate_current();
        return;
      }

      // Concatenate, dropping the first pose of subsequent segments so the
      // connecting waypoint is not duplicated.
      concat_path.header = curr_path.header;
      if (concat_path.poses.empty()) {
        concat_path.poses.insert(
          concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      } else {
        concat_path.poses.insert(
          concat_path.poses.end(), curr_path.poses.begin() + 1, curr_path.poses.end());
      }
    }

    result->path = concat_path;

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "[PlannerServer][%s] Planner loop missed its desired rate of %.4f Hz. Current loop rate is "
        "%.4f Hz",
        __func__, 1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "[PlannerServer][%s] Failed to compute path through poses: \"%s\"", __func__, ex.what());
    action_server_poses_->terminate_current();
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

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "[PlannerServer][%s] No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.",
        __func__, planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "[PlannerServer][%s] planner %s is not a valid planner. "
        "Planner names are: %s",
        __func__, planner_id.c_str(), planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}

void PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

rcl_interfaces::msg::SetParametersResult PlannerServer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "[PlannerServer][%s] The expected planner frequency parameter is %.4f Hz. The value "
            "should to be greater"
            " than 0.0 to turn on duration overrrun warning messages",
            __func__, parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace syncai_planner
