#include "syncai_controller/controller_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/path_height.hpp"
#include "syncai_util/robot_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
// tf2::getYaw() on a geometry_msgs quaternion needs this in the same TU
// (Release-build symbol lookup error otherwise, see simple_progress_checker.cpp).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using syncai_util::lookupGroundZ;
using syncai_util::pathHasHeight;

namespace syncai_controller
{

namespace
{
// "3.05", for durations in exception messages (std::to_string prints
// "3.050000").
std::string seconds(double s)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", s);
  return buf;
}
}  // namespace

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_server", options),
  progress_checker_loader_("syncai_nav_core", "syncai_nav_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"syncai_controller::SimpleProgressChecker"},
  goal_checker_loader_("syncai_nav_core", "syncai_nav_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"syncai_controller::SimpleGoalChecker"},
  lp_loader_("syncai_nav_core", "syncai_nav_core::Controller"),
  default_ids_{"FollowPath"},
  // MPPI, what controller_server_params.yaml loads as FollowPath; the 2D RPP
  // that was the default went with 2D navigation (2026-10-28).
  default_types_{"syncai_mppi_controller::MPPIController"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("goal_reached_max_remaining_path", rclcpp::ParameterValue(1.0));
  // Height-aware tracking of paths with z (path_height.hpp). Shared with the
  // controller plugins, which read these server-level names.
  declare_parameter("body_frame", rclcpp::ParameterValue(std::string("pointlio_body")));
  declare_parameter("lidar_height", rclcpp::ParameterValue(0.481));
  declare_parameter("goal_z_tolerance", rclcpp::ParameterValue(0.5));
  // Where a path turns back on itself it is driven as legs (path_cusps.hpp).
  // cusp_min_angle <= 0 drives every path whole, as before 2026-11.
  declare_parameter("cusp_min_angle", rclcpp::ParameterValue(2.0));
  declare_parameter("cusp_window", rclcpp::ParameterValue(0.4));

  // TrackPose (track_pose, 2026-11). The ids and the three durations are what
  // an empty / zero goal field resolves to; the rest are the server's guards
  // on the odometry (trackControl() has them). dock_max_start_* are the
  // dock_to_pose start gate's (gone 2026-11-12), kept by name for the
  // approach's first target.
  declare_parameter("dock_max_start_distance", rclcpp::ParameterValue(1.5));
  declare_parameter("dock_max_start_lateral", rclcpp::ParameterValue(0.3));
  declare_parameter("default_track_controller_id", rclcpp::ParameterValue("Charging"));
  declare_parameter("default_track_goal_checker_id", rclcpp::ParameterValue("charge_goal_checker"));
  declare_parameter("track_pose_timeout", rclcpp::ParameterValue(0.5));
  declare_parameter("track_lost_timeout", rclcpp::ParameterValue(5.0));
  declare_parameter("track_max_duration", rclcpp::ParameterValue(90.0));
  declare_parameter("track_target_max_spread_xy", rclcpp::ParameterValue(0.08));
  declare_parameter("track_target_max_spread_yaw", rclcpp::ParameterValue(0.2));
  declare_parameter("track_min_poses", rclcpp::ParameterValue(3));
  declare_parameter("track_transform_wait", rclcpp::ParameterValue(0.3));
  declare_parameter("track_max_odom_speed", rclcpp::ParameterValue(1.0));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<syncai_costmap::CostmapROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  action_server_.reset();
  track_action_server_.reset();
  track_pose_sub_.reset();
  dyn_params_handler_.reset();
  odom_sub_.reset();
  // Plugin maps must be cleared before their class loaders (members declared
  // earlier in the header) are destroyed
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  vel_publisher_.reset();
  if (costmap_ros_) {
    costmap_ros_->deactivate();
  }
  costmap_thread_.reset();
}

void ControllerServer::configure()
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    syncai_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      syncai_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      syncai_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("goal_reached_max_remaining_path", goal_reached_max_remaining_path_);
  get_parameter("dock_max_start_distance", dock_max_start_distance_);
  get_parameter("dock_max_start_lateral", dock_max_start_lateral_);
  get_parameter("default_track_controller_id", default_track_controller_id_);
  get_parameter("default_track_goal_checker_id", default_track_goal_checker_id_);
  get_parameter("track_pose_timeout", track_pose_timeout_);
  get_parameter("track_lost_timeout", track_lost_timeout_);
  get_parameter("track_max_duration", track_max_duration_);
  get_parameter("track_target_max_spread_xy", track_target_max_spread_xy_);
  get_parameter("track_target_max_spread_yaw", track_target_max_spread_yaw_);
  get_parameter("track_min_poses", track_min_poses_);
  get_parameter("track_transform_wait", track_transform_wait_);
  get_parameter("track_max_odom_speed", track_max_odom_speed_);
  get_parameter("body_frame", body_frame_);
  get_parameter("lidar_height", body_height_);
  get_parameter("goal_z_tolerance", goal_z_tolerance_);
  get_parameter("cusp_min_angle", cusp_params_.min_angle);
  get_parameter("cusp_window", cusp_params_.window);

  costmap_ros_->init();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<syncai_util::NodeThread>(costmap_ros_);

  try {
    progress_checker_type_ = syncai_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created progress_checker : %s of type %s", progress_checker_id_.c_str(),
      progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create progress_checker. Exception: %s", ex.what());
    exit(-1);
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = syncai_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      syncai_nav_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s", goal_checker_ids_[i].c_str(),
        goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create goal checker. Exception: %s", ex.what());
      exit(-1);
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "Controller Server has %s goal checkers available.",
    goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = syncai_util::get_plugin_type_param(node, controller_ids_[i]);
      syncai_nav_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s", controller_ids_[i].c_str(),
        controller_types_[i].c_str());
      controller->initialize(node, controller_ids_[i], costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s", ex.what());
      exit(-1);
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "Controller Server has %s controllers available.",
    controller_ids_concat_.c_str());

  // Not fatal: FollowPath must keep working from a params file that does not
  // load the charging plugins. The first track goal then fails with
  // INVALID_PLUGIN and the list of what is loaded.
  if (controllers_.find(default_track_controller_id_) == controllers_.end()) {
    RCLCPP_WARN(
      get_logger(),
      "default_track_controller_id '%s' is not in controller_plugins (%s); track_pose goals "
      "with an empty controller_id will be refused",
      default_track_controller_id_.c_str(), controller_ids_concat_.c_str());
  }
  if (goal_checkers_.find(default_track_goal_checker_id_) == goal_checkers_.end()) {
    RCLCPP_WARN(
      get_logger(),
      "default_track_goal_checker_id '%s' is not in goal_checker_plugins (%s); track_pose goals "
      "with an empty goal_checker_id will be refused",
      default_track_goal_checker_id_.c_str(), goal_checker_ids_concat_.c_str());
  }

  odom_sub_ = std::make_unique<syncai_util::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method.
  // spin_thread=true gives the server its own callback group and executor
  // thread, so the (blocking) execute callback never starves this node's
  // main executor.
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(), "follow_path", std::bind(&ControllerServer::computeControl, this), nullptr,
    std::chrono::milliseconds(500), true);

  // The TrackPose server, built the same way. spin_thread=true gives it its
  // own callback group and executor thread for goal / cancel / accepted
  // handling, and SimpleActionServer runs the execute callback itself on a
  // std::async worker, so the two execute callbacks never share a thread:
  // what serialises them is dynamic_params_lock_ (see the class comment),
  // not the executor.
  track_action_server_ = std::make_unique<TrackActionServer>(
    shared_from_this(), "track_pose", std::bind(&ControllerServer::trackControl, this), nullptr,
    std::chrono::milliseconds(500), true);

  // No speed_limit subscription (2026-10-28): nav2_msgs/SpeedLimit was
  // upstream's speed-filter channel, nothing in the stack or the backend
  // ever published it, and the costmap filters it came with are gone.

  // Activation phase (nav2 on_activate)
  costmap_ros_->activate();
  action_server_->activate();
  track_action_server_->activate();

  // Add callback for dynamic parameters
  dyn_params_handler_ = add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));
}

bool ControllerServer::findControllerId(
  const std::string & c_name, std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.",
        controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Action called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name, std::string & current_goal_checker)
{
  // An empty name resolves to the FIRST entry of goal_checker_plugins,
  // however many are loaded. Upstream (and this server until 2026-10) only
  // fell back when exactly one was loaded, which was fine while the list had
  // one entry; adding a second checker beside goal_checker would have failed
  // every BT-issued FollowPath at once, because move.xml sets
  // controller_id and no goal_checker_id (the BT node defaults it to ""),
  // and nothing in the tree would ever have set it. The asymmetry with
  // findControllerId() -- which still refuses an empty id with several
  // controllers loaded -- is deliberate: MPPI vs RPP is a decision a goal
  // must make, while goal checkers differ only in tolerance and the first
  // one listed is the navigation default by construction. Put the
  // navigation checker first in the yaml.
  if (c_name.empty() && !goal_checker_ids_.empty()) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "No goal checker was specified in the action call; using the first of "
      "goal_checker_plugins, '%s'. This warning will appear once.",
      goal_checker_ids_.front().c_str());
    current_goal_checker = goal_checker_ids_.front();
    return true;
  }

  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    RCLCPP_ERROR(
      get_logger(),
      "Action called with goal_checker name %s, which does not exist. "
      "Available goal checkers are: %s.",
      c_name.c_str(), goal_checker_ids_concat_.c_str());
    return false;
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  // Bounded, not blocking: the other holders are a parameter callback
  // (microseconds) and a track_pose goal (its whole run, seconds). A
  // FollowPath goal arriving during a charge approach must fail, not queue
  // up and start moving the robot the moment the approach ends; 100 ms tells
  // that apart from a goal that merely landed inside a parameter update.
  std::unique_lock<std::timed_mutex> lock(dynamic_params_lock_, std::chrono::milliseconds(100));
  if (!lock.owns_lock()) {
    // No zero twist here: cmd_vel belongs to the loop that holds the lock,
    // and a zero from this thread would stop a charge approach mid-way.
    RCLCPP_ERROR(get_logger(), "follow_path refused: %s is driving the robot", driving_.load());
    action_server_->terminate_current();
    return;
  }
  driving_ = "follow_path";

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current();
      return;
    }

    // Per-goal controller state, cleared exactly once here. Deliberately not in
    // setPlannerPath(): updateGlobalPath() routes the BT's ~0.333 Hz replans
    // through that too, so resetting there re-zeroed RPP's acceleration-clamp
    // baseline mid-drive and chopped cmd_vel into a sawtooth (see setPlan()).
    controllers_[current_controller_]->reset();

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();
    logPath("FollowPath goal", current_path_);

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      if (!waitForCurrentCostmap()) {
        return;  // cancelled while waiting; stopped and terminated there
      }

      if (!updateGlobalPath()) {
        publishZeroVelocity();
        return;
      }

      computeAndPublishVelocity();

      if (isGoalReached()) {
        break;
      }

      if (!loop_rate.sleep()) {
        // Throttled: on a saturated CPU every cycle misses, and 20 lines a
        // second bury the log that says why.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
        loop_rate.reset();
      }
    }
  } catch (syncai_nav_core::PlannerException & e) {
    RCLCPP_ERROR(
      get_logger(), "FollowPath aborted (controller '%s'): %s", current_controller_.c_str(),
      e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "FollowPath aborted by an unexpected exception (controller '%s'): %s",
      current_controller_.c_str(), e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  // Always (2026-11): upstream's publish_zero_velocity, false, left the last
  // command running after success -- driver_manager has no cmd_vel watchdog,
  // so the robot walked on at the final rotation's turn or the walking
  // floor. Its use upstream, goals chained without a stop between them, is
  // not one here: a patrol is one goal, cusp legs hand over inside it, and a
  // new path arrives as a preempt. The parameter went with it.
  publishZeroVelocity();

  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(get_logger(), "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw syncai_nav_core::PlannerException("Invalid path, Path is empty.");
  }

  // Cut where the path turns back on itself (2026-11, path_cusps.hpp has the
  // robot01 patrol that stalled 0.84 m short of its U-turn): the plugin is
  // given one leg at a time, so no controller ever sees a path that comes
  // back on itself, and each leg ends on its cusp facing the next, which the
  // plugin's final rotation turns the robot to. current_path_ stays whole --
  // the feedback and the remaining-length gate measure along all of it. A
  // replan (updateGlobalPath()) starts again from its first leg: it starts
  // at the robot.
  const std::vector<size_t> cusps = syncai_util::findCusps(path, cusp_params_);
  legs_ = syncai_util::splitAtCusps(path, cusps, cusp_params_.window);
  leg_ends_ = cusps;
  leg_ends_.push_back(path.poses.size() - 1);

  current_path_ = path;
  // Its own cusps are the ones cut here, so it passes the robot over them at
  // the same turns (path_progress.hpp).
  path_progress_.setCuspParams(cusp_params_);
  path_progress_.reset();
  startLeg(0);
}

void ControllerServer::startLeg(size_t k)
{
  leg_ = k;
  // With the leg's place in the plan (2026-11-07): the planner's word for
  // it (plan_crossings) indexes the whole plan, and the plugin's stair mode
  // shifts the spans to the leg. Legs share their cusp pose (splitAtCusps),
  // so each starts where the one before ended.
  size_t first = 0;
  for (size_t i = 0; i < k; ++i) {
    first += legs_[i].poses.size() - 1;
  }
  controllers_[current_controller_]->setPlanLeg(legs_[k], first, current_path_.poses.size());

  end_pose_ = legs_[k].poses.back();
  end_pose_.header.frame_id = legs_[k].header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Leg %zu/%zu end point is (%.2f, %.2f)", k + 1, legs_.size(),
    end_pose_.pose.position.x, end_pose_.pose.position.y);
}

void ControllerServer::computeAndPublishVelocity()
{
  VelocityCycle cycle;
  if (!computeVelocityCycle(cycle)) {
    return;
  }
  const geometry_msgs::msg::TwistStamped & cmd_vel_2d = cycle.cmd_vel;

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // From the robot's place as isGoalReached() last updated it (a cycle old).
  // Until 2026-11 a whole-path nearest pose to the cycle's pose, in the local
  // costmap's frame while the path is in map, and which on a patrol route
  // could land on a later leg.
  feedback->distance_to_goal =
    syncai_util::geometry_utils::calculate_path_length(current_path_, path_progress_.index());
  action_server_->publish_feedback(feedback);

  publishVelocity(cmd_vel_2d);
}

bool ControllerServer::computeVelocityCycle(VelocityCycle & cycle)
{
  // The body of computeAndPublishVelocity(), split out so a second loop can
  // run the same pose lookup, patience handling, progress check and
  // controller call with a different feedback. Nothing in here touches an
  // action server. Split once more (2026-11) into its two halves for
  // track_pose, which runs them without the progress check between.
  if (!cycleRobotPose(cycle.pose)) {
    return false;
  }

  if (!progress_checker_->check(cycle.pose)) {
    // The checker logs where the robot stood and for how long.
    throw syncai_nav_core::PlannerException(
      "Failed to make progress (progress checker '" + progress_checker_id_ + "')");
  }

  cycle.cmd_vel = controllerCommand(cycle.pose);
  return true;
}

bool ControllerServer::cycleRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  if (!getRobotPose(pose)) {
    // Under failure_tolerance like an exception out of the controller. Upstream
    // throws here unconditionally, so a single stale odom -> base_link (the LIO
    // bridge's TF, past the costmap's transform_tolerance) failed FollowPath
    // at once, while the same TF miss inside RPP's transformGlobalPlan() got
    // the full grace window -- and the RecoveryNode retry it burned was the
    // one a real blocker later needed. Stopping is safe without a pose; driving
    // is not, so the cycle commands zero and skips the progress checker and
    // the feedback, both of which need the pose.
    //
    // The controller is never called this cycle, so its catch-all cannot zero
    // RPP's acceleration-clamp baseline; reset() does it instead. Without it
    // the first cycle after the TF came back would clamp around the speed held
    // before the outage and step 0 -> ~0.55 m/s in 50 ms, the lurch the
    // catch-all exists to prevent.
    // The TF error itself is logged by getCurrentPose(); this says what it
    // costs the goal.
    const double without_cmd = (now() - last_valid_cmd_time_).seconds();
    const std::string frames =
      costmap_ros_->getBaseFrameID() + " in " + costmap_ros_->getGlobalFrameID();
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot pose (%s) unavailable: commanding zero, %.1f s without a valid command "
        "(failure_tolerance %.1f s)",
        frames.c_str(), without_cmd, failure_tolerance_);
      controllers_[current_controller_]->reset();
      if (without_cmd > failure_tolerance_ && failure_tolerance_ != -1.0) {
        throw syncai_nav_core::PlannerException(
          "Controller patience exceeded: robot pose (" + frames + ") unavailable, " +
          seconds(without_cmd) + " s without a valid command");
      }
      publishZeroVelocity();
      return false;
    }
    throw syncai_nav_core::PlannerException("Failed to obtain robot pose (" + frames + ")");
  }
  return true;
}

geometry_msgs::msg::TwistStamped ControllerServer::controllerCommand(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(
      pose, twist, goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (syncai_nav_core::PlannerException & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      const double without_cmd = (now() - last_valid_cmd_time_).seconds();
      // Throttled: a blocked path fails every cycle (20 Hz) for the whole
      // window. The abort below repeats the last error, so the line that
      // decided the goal is never lost to the throttle.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Controller '%s' failed: %s -- commanding zero, %.1f s without a valid command "
        "(failure_tolerance %.1f s)",
        current_controller_.c_str(), e.what(), without_cmd, failure_tolerance_);
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if (without_cmd > failure_tolerance_ && failure_tolerance_ != -1.0) {
        throw syncai_nav_core::PlannerException(
          "Controller patience exceeded: no valid command for " + seconds(without_cmd) +
          " s; last error: " + e.what());
      }
    } else {
      throw syncai_nav_core::PlannerException(e.what());
    }
  }

  return cmd_vel_2d;
}

bool ControllerServer::updateGlobalPath()
{
  // false = the preempting goal was refused and terminated: computeControl()
  // stops the robot and leaves the loop. Until 2026-11 this returned nothing
  // and the loop went on driving the old path with no goal behind it, and a
  // cancel was then refused, since there was no goal to cancel (upstream's
  // too, Humble and nav2 main alike).
  if (action_server_->is_preempt_requested()) {
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      // A preempt does not reset the controller (computeControl() has why),
      // except when it switches controllers: the
      // newcomer has not run this goal, and without it it would start from
      // whatever its last goal left -- the acceleration clamp's baseline,
      // MPPI's warm start, stair mode. No tree switches mid-goal today.
      if (current_controller != current_controller_) {
        RCLCPP_INFO(
          get_logger(), "Preempt switches controller %s -> %s; resetting %s",
          current_controller_.c_str(), current_controller.c_str(), current_controller.c_str());
        controllers_[current_controller]->reset();
      }
      current_controller_ = current_controller;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Terminating action, invalid controller %s requested; stopping the robot",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return false;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Terminating action, invalid goal checker %s requested; stopping the robot",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return false;
    }
    // Deliberately not reset here: last_valid_cmd_time_ and the progress
    // checker. failure_tolerance is "seconds since the last command the
    // controller could compute", and a replanned path RPP can drive resets
    // that timer itself on its first cycle (computeAndPublishVelocity()), so
    // a reset on preempt would only ever extend the life of a path RPP also
    // refuses. Since the planner's is_path_valid runs RPP's own footprint
    // test (2026-10), a refused path is replanned once a second in a passage
    // too narrow for the footprint, and every replan is a preempt: a reset
    // per preempt would make such a goal immune to failure_tolerance and
    // leave the progress checker's 30 s as the only abort, the robot standing
    // in front of the blocker ten times longer for the same outcome. Waiting
    // for a live blocker to leave is the BT's retry budget (move.xml), not
    // this timer.
    setPlannerPath(goal->path);
    logPath("FollowPath replan", current_path_);
  }
  return true;
}

namespace
{
// The odometry speed guard measures the robot's NET displacement over this
// long, not from one cycle to the next: the pose is the lidar's, carried by a
// body that rocks a few centimetres with every step, and the odometry
// updates at half the control rate, so a cycle-to-cycle speed reads 0.5 m/s
// on a robot walking at 0.1.
constexpr double kTrackSpeedBaseline = 0.4;
// A frozen target is let go only once the robot is this many tolerances off
// it, not at the tolerance itself: this close the tracked thing may be out
// of view, so letting go at the first millimetre outside would leave a robot
// with no fresh pose holding still until LOST, 1 mm from done. Between the
// two the controller corrects against the frozen target.
constexpr double kTrackFreezeRelease = 2.0;

const char * trackStateName(uint8_t state)
{
  using Feedback = syncai_common::action::TrackPose::Feedback;
  switch (state) {
    case Feedback::WAITING_FOR_POSE:
      return "WAITING_FOR_POSE";
    case Feedback::TRACKING:
      return "TRACKING";
    case Feedback::HOLDING:
      return "HOLDING";
  }
  return "not started";
}
}  // namespace

void ControllerServer::trackControl()
{
  // The same bounded lock as the other two loops, and the same rule: nothing
  // is published before it is owned.
  std::unique_lock<std::timed_mutex> lock(dynamic_params_lock_, std::chrono::milliseconds(100));
  TrackRun run;
  if (!lock.owns_lock()) {
    const std::string busy = std::string(driving_.load()) + " is driving the robot";
    RCLCPP_ERROR(get_logger(), "track_pose refused: %s", busy.c_str());
    track_action_server_->terminate_current(
      makeTrackResult(run, TrackAction::Result::CONTROLLER_BUSY, busy));
    return;
  }
  driving_ = "track_pose";

  RCLCPP_INFO(get_logger(), "Received a track goal, begin computing control effort.");

  // The tracked topic is subscribed for one goal only; every way out drops it.
  struct EndTracking
  {
    ControllerServer * server;
    ~EndTracking()
    {
      server->track_pose_sub_.reset();
      std::lock_guard<std::mutex> inbox_lock(server->track_inbox_mutex_);
      server->track_inbox_.clear();
    }
  } end_tracking{this};

  using Feedback = TrackAction::Feedback;
  using Result = TrackAction::Result;

  // Stop, and end the goal with `code`.
  const auto fail = [this, &run](uint16_t code, const std::string & msg) {
    RCLCPP_ERROR(get_logger(), "track_pose aborted: %s", msg.c_str());
    publishZeroVelocity();
    track_action_server_->terminate_current(makeTrackResult(run, code, msg));
  };

  try {
    uint16_t error_code = Result::NONE;
    std::string error_msg;
    if (!startTrackGoal(*track_action_server_->get_current_goal(), run, error_code, error_msg)) {
      RCLCPP_ERROR(get_logger(), "track_pose rejected: %s", error_msg.c_str());
      publishZeroVelocity();
      track_action_server_->terminate_current(makeTrackResult(run, error_code, error_msg));
      return;
    }

    last_valid_cmd_time_ = now();
    bool was_tracking = false;
    char buf[400];
    rclcpp::WallRate loop_rate(controller_frequency_);
    const auto pace = [this, &loop_rate]() {
      if (!loop_rate.sleep()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Track control loop missed its desired rate of %.4fHz",
          controller_frequency_);
        loop_rate.reset();
      }
    };

    while (rclcpp::ok()) {
      if (track_action_server_ == nullptr || !track_action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Track action server unavailable or inactive. Stopping.");
        return;
      }

      if (track_action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Track goal was canceled. Stopping the robot.");
        track_action_server_->terminate_all(makeTrackResult(run, Result::NONE, ""));
        publishZeroVelocity();
        return;
      }

      if (track_action_server_->is_preempt_requested()) {
        // A new goal is a new thing to track: nothing of the old one carries
        // over (unlike a new POSE, which is the same goal seen again), so the
        // robot stops until the new goal has a target of its own.
        RCLCPP_INFO(get_logger(), "A new track goal replaces the running one.");
        publishZeroVelocity();
        auto goal = track_action_server_->accept_pending_goal();
        if (!startTrackGoal(*goal, run, error_code, error_msg)) {
          RCLCPP_ERROR(get_logger(), "track_pose preempt rejected: %s", error_msg.c_str());
          track_action_server_->terminate_current(makeTrackResult(run, error_code, error_msg));
          return;
        }
        was_tracking = false;
        last_valid_cmd_time_ = now();
      }

      const rclcpp::Time t = now();
      if ((t - run.start).seconds() > run.max_duration) {
        std::snprintf(
          buf, sizeof(buf),
          "max_duration %.1f s elapsed in state %s before the goal checker '%s' accepted the pose "
          "(%u poses received, %u dropped)",
          run.max_duration, trackStateName(run.state), current_goal_checker_.c_str(), run.received,
          run.rejected);
        fail(Result::TIMEOUT, buf);
        return;
      }

      // The robot pose, with FollowPath's patience for an outage (zero is
      // published and the controller reset inside).
      geometry_msgs::msg::PoseStamped pose;
      if (!cycleRobotPose(pose)) {
        was_tracking = false;
        pace();
        continue;
      }
      const Pose2 robot{
        pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)};

      // Guard 1: an odometry that moves the robot faster than anything this
      // action commands is not an odometry to drive blind on. Point-LIO
      // diverging reads exactly so (2026-11-09: 2 -> 20 m/s within seconds),
      // and every target taken meanwhile is in a frame that is running away.
      {
        const double stamp = t.seconds();
        run.pose_history.push_back({stamp, robot.x, robot.y});
        while (run.pose_history.size() > 2 &&
               run.pose_history[1][0] <= stamp - kTrackSpeedBaseline) {
          run.pose_history.pop_front();
        }
        const auto & old = run.pose_history.front();
        const double dt = stamp - old[0];
        const double speed =
          dt >= kTrackSpeedBaseline ? std::hypot(robot.x - old[1], robot.y - old[2]) / dt : 0.0;
        if (speed > track_max_odom_speed_) {
          if (!run.odom_fast) {
            run.odom_fast = true;
            run.odom_fast_since = t;
            RCLCPP_ERROR(
              get_logger(),
              "track_pose: the odometry moved the robot %.2f m in %.2f s (%.2f m/s, limit "
              "track_max_odom_speed %.2f; nothing here commands that): stopping, and dropping "
              "the %zu targets held -- they were placed in a frame that is not holding still",
              speed * dt, dt, speed, track_max_odom_speed_, run.filter.size());
            controllers_[current_controller_]->reset();
            goal_checkers_[current_goal_checker_]->reset();
            run.filter.clear();
          }
          publishZeroVelocity();
          was_tracking = false;
          const double fast_for = (t - run.odom_fast_since).seconds();
          if (fast_for > std::max(failure_tolerance_, 1.0)) {
            std::snprintf(
              buf, sizeof(buf),
              "the odometry has moved the robot faster than track_max_odom_speed (%.2f m/s) for "
              "%.1f s, last %.2f m/s: it is not to be trusted for a blind approach (Point-LIO "
              "diverging?)",
              track_max_odom_speed_, fast_for, speed);
            fail(Result::FAILED_TO_CONTROL, buf);
            return;
          }
          pace();
          continue;
        }
        if (run.odom_fast) {
          run.odom_fast = false;
          RCLCPP_WARN(
            get_logger(),
            "track_pose: the odometry is back under track_max_odom_speed after %.1f s; waiting "
            "for new targets",
            (t - run.odom_fast_since).seconds());
          // What arrived meanwhile would be placed with the poses of an
          // odometry that was running away.
          std::lock_guard<std::mutex> inbox_lock(track_inbox_mutex_);
          track_inbox_.clear();
        }
      }

      intakeTrackedPoses(run);

      // The goal checker's tolerances, in the target's frame, for the freeze.
      // A checker that reports none (lowest()) never freezes the target.
      geometry_msgs::msg::Pose pose_tolerance;
      geometry_msgs::msg::Twist vel_tolerance;
      const bool have_tolerance =
        goal_checkers_[current_goal_checker_]->getTolerances(pose_tolerance, vel_tolerance) &&
        pose_tolerance.position.x > 0.0 && pose_tolerance.position.y > 0.0;
      const double tol_along = pose_tolerance.position.x;
      const double tol_lateral = pose_tolerance.position.y;
      const double tol_yaw = have_tolerance ? tf2::getYaw(pose_tolerance.orientation) : 0.0;
      const auto within = [&](const TargetErrors & e, double ratio) {
        return have_tolerance && std::fabs(e.along) <= ratio * tol_along &&
               std::fabs(e.lateral) <= ratio * tol_lateral && std::fabs(e.yaw) <= ratio * tol_yaw;
      };

      // A frozen target is let go once the robot has left it (see
      // kTrackFreezeRelease); what follows then judges the live window.
      if (run.filter.frozen()) {
        const TargetErrors e = targetErrors(robot, run.filter.target());
        if (!within(e, kTrackFreezeRelease)) {
          run.filter.unfreeze();
          RCLCPP_INFO(
            get_logger(),
            "track_pose: target released -- the robot left it: along %+.3f m (tolerance %.3f), "
            "lateral %+.3f m (%.3f), yaw %+.3f rad (%.3f), released beyond %.0fx; following "
            "the tracked poses again (%zu in the window)",
            e.along, tol_along, e.lateral, tol_lateral, e.yaw, tol_yaw, kTrackFreezeRelease,
            run.filter.size());
        }
      }

      const bool has_target = run.filter.hasTarget();
      const double newest = run.filter.newestStamp();
      const double pose_age = newest >= 0.0 ? t.seconds() - newest : -1.0;
      const bool fresh = pose_age >= 0.0 && pose_age <= run.pose_timeout;
      double spread_xy = 0.0;
      double spread_yaw = 0.0;
      run.filter.spread(spread_xy, spread_yaw);
      const bool stable = run.filter.stable();

      Pose2 target;
      TargetErrors errors;
      if (has_target) {
        target = run.filter.target();
        errors = targetErrors(robot, target);
        run.errors = errors;
        run.have_errors = true;
      }

      // Inside the goal checker's tolerances on a target worth keeping: hold
      // it. The checker's settling is then timed against ONE target, and
      // pose_timeout no longer applies -- this close a charger's tag may have
      // left the image.
      if (!run.filter.frozen() && has_target && fresh && stable && within(errors, 1.0)) {
        run.filter.freeze();
        RCLCPP_INFO(
          get_logger(),
          "track_pose: target frozen at (%.3f, %.3f, yaw %+.3f) -- the robot is inside the goal "
          "checker's tolerances (along %+.3f m, lateral %+.3f m, yaw %+.3f rad); settling "
          "against it, tracked poses no longer move it",
          target.x, target.y, target.yaw, errors.along, errors.lateral, errors.yaw);
      }

      uint8_t state = Feedback::TRACKING;
      if (!run.filter.frozen()) {
        if (!has_target) {
          state = Feedback::WAITING_FOR_POSE;
        } else if (!fresh || !stable) {
          state = Feedback::HOLDING;
        }
      }

      if (state != run.state) {
        // The numbers that decided it.
        RCLCPP_INFO(
          get_logger(),
          "track_pose: %s -> %s: %zu targets in the window (need %zu), newest %.2f s old "
          "(pose_timeout %.2f), spread %.3f m / %.3f rad (limits %.3f / %.3f)%s; %u poses "
          "received, %u dropped",
          trackStateName(run.state), trackStateName(state), run.filter.size(),
          run.filter.params().window > 0.0 ? run.filter.params().min_poses : std::size_t{1},
          pose_age, run.pose_timeout, spread_xy, spread_yaw, run.filter.params().max_spread_xy,
          run.filter.params().max_spread_yaw, run.filter.frozen() ? ", target frozen" : "",
          run.received, run.rejected);
        run.state = state;
      }

      bool reached = false;
      if (state == Feedback::TRACKING) {
        run.last_usable = t;
        run.received_at_usable = run.received;
        run.accepted_at_usable = run.accepted;

        if (!run.checked_start) {
          // Blind, so it starts only from in front of its target: a
          // preempt that moved the goal after the staging leg would otherwise
          // hand a far target straight to this controller, round whatever
          // stood between (the dock_to_pose gate's reason, gone 2026-11-12,
          // whose parameters these are). Checked on the first target,
          // because a track goal has none when it is accepted.
          run.checked_start = true;
          const double distance = std::hypot(errors.along, errors.lateral);
          if (
            distance > dock_max_start_distance_ ||
            std::fabs(errors.lateral) > dock_max_start_lateral_) {
            std::snprintf(
              buf, sizeof(buf),
              "the robot is %.2f m from the first target and %.2f m off its axis (limits "
              "dock_max_start_distance %.2f m, dock_max_start_lateral %.2f m): the approach is "
              "blind, so it starts only from in front of the target",
              distance, std::fabs(errors.lateral), dock_max_start_distance_,
              dock_max_start_lateral_);
            fail(Result::FAILED_TO_CONTROL, buf);
            return;
          }
          RCLCPP_INFO(
            get_logger(),
            "track_pose: first target (%.3f, %.3f, yaw %+.3f) in '%s', %.3f m ahead, %+.3f m "
            "across, %+.3f rad",
            target.x, target.y, target.yaw, pose.header.frame_id.c_str(), errors.along,
            errors.lateral, errors.yaw);
        }

        // The plugin reads the last pose of a path; the robot's pose leads it
        // as in a dock's. Straight to setPlan(), not setPlannerPath(): that
        // one resets the goal checker, whose settling must survive a target
        // that is the same goal seen again.
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = pose.header.frame_id;
        target_pose.header.stamp = t;
        target_pose.pose.position.x = target.x;
        target_pose.pose.position.y = target.y;
        target_pose.pose.orientation =
          syncai_util::geometry_utils::orientationAroundZAxis(target.yaw);
        nav_msgs::msg::Path path;
        path.header = target_pose.header;
        path.poses = {pose, target_pose};
        path.poses.front().header = path.header;
        controllers_[current_controller_]->setPlan(path);
        end_pose_ = target_pose;

        if (!was_tracking) {
          // Standing still for a pose was not a failure to command.
          last_valid_cmd_time_ = t;
        }
        const geometry_msgs::msg::TwistStamped cmd_vel = controllerCommand(pose);
        publishVelocity(cmd_vel);
        was_tracking = true;

        reached = goal_checkers_[current_goal_checker_]->isGoalReached(
          pose.pose, target_pose.pose, getThresholdedTwist(odom_sub_->getTwist()));
      } else {
        // Zero, every cycle: driver_manager repeats the last command it got.
        if (was_tracking) {
          // The plugin's band state and ramp baseline, and the checker's
          // settling, belong to a robot that was moving.
          controllers_[current_controller_]->reset();
          goal_checkers_[current_goal_checker_]->reset();
          was_tracking = false;
        }
        publishZeroVelocity();
        last_valid_cmd_time_ = t;

        const double unusable_for = (t - run.last_usable).seconds();
        if (unusable_for > run.lost_timeout) {
          const uint32_t received = run.received - run.received_at_usable;
          const uint32_t accepted = run.accepted - run.accepted_at_usable;
          uint16_t code = Result::LOST;
          if (received == 0) {
            std::snprintf(
              buf, sizeof(buf),
              "no tracked pose for %.1f s (lost_timeout %.1f); %u received since the goal began",
              unusable_for, run.lost_timeout, run.received);
          } else if (accepted == 0) {
            code = Result::TRANSFORM_FAILED;
            std::snprintf(
              buf, sizeof(buf),
              "%u poses arrived in %.1f s and none could be transformed into '%s' at its stamp "
              "(each waited track_transform_wait %.2f s): an unknown frame, or the odometry lags "
              "the poses by more than that",
              received, unusable_for, costmap_ros_->getGlobalFrameID().c_str(),
              track_transform_wait_);
          } else if (has_target && fresh && !stable) {
            code = Result::TARGET_UNSTABLE;
            std::snprintf(
              buf, sizeof(buf),
              "for %.1f s the targets in the %.1f s window have not agreed on one: spread %.3f m "
              "/ %.3f rad over %zu targets (limits %.3f / %.3f). The tracked thing moves, the "
              "odometry is drifting fast or was reset, or the publisher alternates between two "
              "poses",
              unusable_for, run.filter.params().window, spread_xy, spread_yaw, run.filter.size(),
              run.filter.params().max_spread_xy, run.filter.params().max_spread_yaw);
          } else {
            std::snprintf(
              buf, sizeof(buf),
              "no usable target for %.1f s (lost_timeout %.1f): %u poses arrived, %zu in the "
              "window (need %zu), the newest %.2f s old (pose_timeout %.2f)",
              unusable_for, run.lost_timeout, received, run.filter.size(),
              run.filter.params().min_poses, pose_age, run.pose_timeout);
          }
          fail(code, buf);
          return;
        }
      }

      auto feedback = std::make_shared<Feedback>();
      feedback->state = state;
      if (has_target) {
        feedback->distance_to_target = static_cast<float>(std::hypot(errors.along, errors.lateral));
        feedback->lateral_error = static_cast<float>(errors.lateral);
        feedback->yaw_error = static_cast<float>(errors.yaw);
      }
      feedback->pose_age = static_cast<float>(pose_age);
      feedback->poses_received = run.received;
      feedback->poses_rejected = run.rejected;
      feedback->tracking_time = t - run.start;
      track_action_server_->publish_feedback(feedback);

      // Once a second, for the same reason as the dock loop's line: the
      // feedback reaches only the BT node.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Tracking %.1f s, %s%s: robot (%.3f, %.3f, yaw %+.3f) in '%s'; to the target along %+.3f "
        "m, lateral %+.3f m, yaw %+.3f rad; newest pose %.2f s old, %zu in the window, spread "
        "%.3f m / %.3f rad; %u received, %u dropped",
        (t - run.start).seconds(), trackStateName(state),
        run.filter.frozen() ? " (target frozen)" : "", robot.x, robot.y, robot.yaw,
        pose.header.frame_id.c_str(), errors.along, errors.lateral, errors.yaw, pose_age,
        run.filter.size(), spread_xy, spread_yaw, run.received, run.rejected);

      if (reached) {
        RCLCPP_INFO(
          get_logger(),
          "Reached the tracked target after %.1f s: along %+.3f m, lateral %+.3f m, yaw %+.3f "
          "rad (goal checker '%s'); %u poses received, %u dropped",
          (t - run.start).seconds(), errors.along, errors.lateral, errors.yaw,
          current_goal_checker_.c_str(), run.received, run.rejected);
        break;
      }

      pace();
    }
  } catch (syncai_nav_core::PlannerException & e) {
    RCLCPP_ERROR(
      get_logger(), "track_pose aborted (controller '%s'): %s", current_controller_.c_str(),
      e.what());
    publishZeroVelocity();
    track_action_server_->terminate_current(
      makeTrackResult(run, Result::FAILED_TO_CONTROL, e.what()));
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "track_pose aborted by an unexpected exception (controller '%s'): %s",
      current_controller_.c_str(), e.what());
    publishZeroVelocity();
    track_action_server_->terminate_current(
      makeTrackResult(run, Result::FAILED_TO_CONTROL, std::string("unexpected: ") + e.what()));
    return;
  }

  // Always: driver_manager has no cmd_vel watchdog.
  publishZeroVelocity();
  track_action_server_->succeeded_current(makeTrackResult(run, TrackAction::Result::NONE, ""));
}

bool ControllerServer::startTrackGoal(
  const TrackAction::Goal & goal, TrackRun & run, uint16_t & error_code, std::string & error_msg)
{
  run = TrackRun();
  track_pose_sub_.reset();
  {
    std::lock_guard<std::mutex> inbox_lock(track_inbox_mutex_);
    track_inbox_.clear();
  }

  if (goal.pose_topic.empty() == goal.tracked_frame.empty()) {
    error_code = TrackAction::Result::INVALID_GOAL;
    error_msg = goal.pose_topic.empty()
                  ? "neither pose_topic nor tracked_frame is set; exactly one is needed"
                  : "both pose_topic ('" + goal.pose_topic + "') and tracked_frame ('" +
                      goal.tracked_frame + "') are set; exactly one is needed";
    return false;
  }
  const auto & q = goal.target_offset.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(q_norm > 1e-6)) {
    error_code = TrackAction::Result::INVALID_GOAL;
    error_msg =
      "target_offset.orientation is a zero quaternion, not a rotation (no rotation is "
      "w: 1.0)";
    return false;
  }
  if (!(goal.target_filter_window >= 0.0f)) {
    error_code = TrackAction::Result::INVALID_GOAL;
    error_msg = "target_filter_window must be >= 0 (0 = no filter)";
    return false;
  }

  // Empty ids resolve to the track defaults here, as a dock goal's do.
  const std::string controller_id =
    goal.controller_id.empty() ? default_track_controller_id_ : goal.controller_id;
  std::string controller;
  if (!findControllerId(controller_id, controller)) {
    error_code = TrackAction::Result::INVALID_PLUGIN;
    error_msg =
      "controller '" + controller_id + "' is not loaded; available: " + controller_ids_concat_;
    return false;
  }
  const std::string goal_checker_id =
    goal.goal_checker_id.empty() ? default_track_goal_checker_id_ : goal.goal_checker_id;
  std::string goal_checker;
  if (!findGoalCheckerId(goal_checker_id, goal_checker)) {
    error_code = TrackAction::Result::INVALID_PLUGIN;
    error_msg = "goal checker '" + goal_checker_id +
                "' is not loaded; available: " + goal_checker_ids_concat_;
    return false;
  }

  // No point starting a loop that fails on its first cycle.
  geometry_msgs::msg::PoseStamped start;
  if (!getRobotPose(start)) {
    error_code = TrackAction::Result::FAILED_TO_CONTROL;
    error_msg = "robot pose unavailable in '" + costmap_ros_->getGlobalFrameID() + "'";
    return false;
  }

  run.pose_topic = goal.pose_topic;
  run.tracked_frame = goal.tracked_frame;
  run.offset = tf2::Transform(
    tf2::Quaternion(q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm),
    tf2::Vector3(
      goal.target_offset.position.x, goal.target_offset.position.y, goal.target_offset.position.z));
  const auto or_default = [](const builtin_interfaces::msg::Duration & d, double fallback) {
    const double s = rclcpp::Duration(d).seconds();
    return s > 0.0 ? s : fallback;
  };
  run.pose_timeout = or_default(goal.pose_timeout, track_pose_timeout_);
  run.lost_timeout = or_default(goal.lost_timeout, track_lost_timeout_);
  run.max_duration = or_default(goal.max_duration, track_max_duration_);

  TrackTargetFilterParams filter;
  filter.window = goal.target_filter_window;
  filter.min_poses = static_cast<std::size_t>(std::max(track_min_poses_, 1));
  filter.max_spread_xy = track_target_max_spread_xy_;
  filter.max_spread_yaw = track_target_max_spread_yaw_;
  run.filter.configure(filter);

  current_controller_ = controller;
  current_goal_checker_ = goal_checker;
  controllers_[current_controller_]->reset();
  goal_checkers_[current_goal_checker_]->reset();

  if (!run.pose_topic.empty()) {
    // Best effort, so a reliable and a best-effort publisher both match; the
    // callback only queues -- the transform needs the stamp's TF, which the
    // loop waits for.
    track_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      run.pose_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> inbox_lock(track_inbox_mutex_);
        track_inbox_.push_back(TrackedPose{*msg, now(), false});
        // Bounded: the loop empties it every cycle while a goal runs.
        while (track_inbox_.size() > 50) {
          track_inbox_.pop_front();
        }
      });
  }

  run.start = now();
  run.last_usable = run.start;
  run.started = true;

  const double offset_yaw = tf2::getYaw(run.offset.getRotation());
  const std::string filtering = filter.window > 0.0
                                  ? "target = median of " + seconds(filter.window) +
                                      " s of poses, at least " + std::to_string(filter.min_poses)
                                  : "no filter: each pose is the target";
  RCLCPP_INFO(
    get_logger(),
    "Track goal: %s '%s', offset (%.3f, %.3f, yaw %+.3f) in its frame; controller '%s', goal "
    "checker '%s'; pose_timeout %.2f s, lost_timeout %.1f s, max_duration %.1f s; %s",
    run.pose_topic.empty() ? "frame" : "topic",
    run.pose_topic.empty() ? run.tracked_frame.c_str() : track_pose_sub_->get_topic_name(),
    run.offset.getOrigin().x(), run.offset.getOrigin().y(), offset_yaw, controller.c_str(),
    goal_checker.c_str(), run.pose_timeout, run.lost_timeout, run.max_duration, filtering.c_str());
  return true;
}

void ControllerServer::intakeTrackedPoses(TrackRun & run)
{
  const std::string & global_frame = costmap_ros_->getGlobalFrameID();
  const auto tf = costmap_ros_->getTfBuffer();
  const rclcpp::Time t = now();

  // A tracked pose in odom -> the target: the offset in the pose's own frame,
  // then the floor (x, y, yaw).
  const auto take = [&run](const tf2::Transform & tracked_in_odom, double stamp) {
    const tf2::Transform target = tracked_in_odom * run.offset;
    run.filter.add(
      stamp,
      Pose2{target.getOrigin().x(), target.getOrigin().y(), tf2::getYaw(target.getRotation())});
    ++run.accepted;
  };

  if (!run.tracked_frame.empty()) {
    try {
      const geometry_msgs::msg::TransformStamped transform =
        tf->lookupTransform(global_frame, run.tracked_frame, tf2::TimePointZero);
      const rclcpp::Time stamp(transform.header.stamp, RCL_ROS_TIME);
      // A static transform carries no stamp: it is true now.
      const bool is_static = stamp.nanoseconds() == 0;
      if (is_static || stamp > run.last_frame_stamp) {
        run.last_frame_stamp = stamp;
        ++run.received;
        tf2::Transform tracked_in_odom;
        tf2::fromMsg(transform.transform, tracked_in_odom);
        take(tracked_in_odom, is_static ? t.seconds() : stamp.seconds());
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "track_pose: no transform from '%s' to '%s': %s",
        run.tracked_frame.c_str(), global_frame.c_str(), e.what());
    }
    return;
  }

  std::deque<TrackedPose> arrived;
  {
    std::lock_guard<std::mutex> inbox_lock(track_inbox_mutex_);
    arrived.swap(track_inbox_);
  }
  std::deque<TrackedPose> waiting;
  for (auto & item : arrived) {
    if (!item.counted) {
      item.counted = true;
      ++run.received;
    }
    const auto & q = item.pose.pose.orientation;
    if (!(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6)) {
      ++run.rejected;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "track_pose: dropped a pose whose orientation is a zero quaternion (%u of %u dropped)",
        run.rejected, run.received);
      continue;
    }
    // At the pose's OWN stamp: it is where the tracked thing was when it was
    // seen, and the robot has moved since. An unstamped pose (a hand-typed
    // test) is taken as true when it arrived.
    const rclcpp::Time stamp(item.pose.header.stamp, RCL_ROS_TIME);
    const bool unstamped = stamp.nanoseconds() == 0;
    try {
      const geometry_msgs::msg::TransformStamped transform = tf->lookupTransform(
        global_frame, item.pose.header.frame_id,
        unstamped ? tf2::TimePointZero : tf2_ros::fromMsg(item.pose.header.stamp));
      tf2::Transform frame_in_odom;
      tf2::Transform pose_in_frame;
      tf2::fromMsg(transform.transform, frame_in_odom);
      tf2::fromMsg(item.pose.pose, pose_in_frame);
      take(frame_in_odom * pose_in_frame, unstamped ? item.arrival.seconds() : stamp.seconds());
    } catch (const tf2::TransformException & e) {
      // Guard 2: the odometry behind the camera. Point-LIO publishes a scan's
      // pose after it has processed the scan, and on a saturated CPU that is
      // later than the image the pose was seen in -- the transform at the
      // image's stamp does not exist YET. Waited for, then dropped and
      // counted; never the end of the goal by itself (TrackPose.action).
      const double waited = (t - item.arrival).seconds();
      if (waited < track_transform_wait_) {
        waiting.push_back(item);
      } else {
        ++run.rejected;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "track_pose: dropped a pose stamped %.2f s ago in '%s' after waiting %.2f s for its "
          "transform into '%s' (track_transform_wait %.2f): %s -- %u of %u poses dropped so far",
          unstamped ? waited : (t - stamp).seconds(), item.pose.header.frame_id.c_str(), waited,
          global_frame.c_str(), track_transform_wait_, e.what(), run.rejected, run.received);
      }
    }
  }
  if (!waiting.empty()) {
    std::lock_guard<std::mutex> inbox_lock(track_inbox_mutex_);
    track_inbox_.insert(track_inbox_.begin(), waiting.begin(), waiting.end());
  }
}

std::shared_ptr<ControllerServer::TrackAction::Result> ControllerServer::makeTrackResult(
  const TrackRun & run, uint16_t error_code, const std::string & error_msg)
{
  auto result = std::make_shared<TrackAction::Result>();
  result->error_code = error_code;
  result->error_msg = error_msg;
  if (run.started) {
    result->total_elapsed_time = now() - run.start;
  }
  if (run.have_errors) {
    result->final_distance_error =
      static_cast<float>(std::hypot(run.errors.along, run.errors.lateral));
    result->final_lateral_error = static_cast<float>(run.errors.lateral);
    result->final_yaw_error = static_cast<float>(run.errors.yaw);
  } else {
    // Documented sentinel: no cycle had both a robot pose and a target.
    result->final_distance_error = -1.0f;
  }
  return result;
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

bool ControllerServer::waitForCurrentCostmap()
{
  if (costmap_ros_->isCurrent()) {
    return true;
  }
  // No command is computed until the local costmap is current again: just
  // after a clear (one update, <= 0.2 s), or while the terrain layer has no
  // fresh cloud (pointlio/body_cloud stopped, or its TF did, which drops the
  // cloud at the layer's message filter; terrain_layer.cpp, cloud_timeout).
  //
  // Until 2026-11 this waited with nothing published, cancel unread and no
  // bound. syncai_driver_manager has no cmd_vel watchdog -- it sends the
  // last command until a new one comes -- so the gait kept walking at the
  // last speed for as long as the wait lasted, and a cancel from the BT was
  // not seen until it ended. That stayed latent only because the terrain
  // layer never reported a stale cloud; now that it does, the wait must
  // stop the robot:
  //  - zero at once, and the controller reset() as on a missing robot pose
  //    (computeVelocity()), so the first command after the wait ramps up
  //    from a stand instead of clamping round the speed held before it;
  //  - a cancel is honoured inside the wait;
  //  - it is bounded by failure_tolerance on the same clock as a refused
  //    command (last_valid_cmd_time_): no fresh terrain is no valid command.
  //    Never shorter than costmap_wait_min_s, so the one-update gap after a
  //    costmap clear (the BT's recovery clears before every FollowPath
  //    retry) cannot abort a goal even at failure_tolerance 0.
  constexpr double costmap_wait_min_s = 1.0;
  publishZeroVelocity();
  controllers_[current_controller_]->reset();
  const rclcpp::Time wait_start = now();
  rclcpp::Rate r(100);
  while (rclcpp::ok() && !costmap_ros_->isCurrent()) {
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(
        get_logger(), "Goal was canceled while waiting for the local costmap. Stopping the robot.");
      action_server_->terminate_all();
      publishZeroVelocity();
      return false;
    }
    const double waited = (now() - wait_start).seconds();
    const double without_cmd = (now() - last_valid_cmd_time_).seconds();
    if (
      failure_tolerance_ != -1.0 &&
      without_cmd > std::max(failure_tolerance_, costmap_wait_min_s)) {
      throw syncai_nav_core::PlannerException(
        "Controller patience exceeded: the local costmap has not been current for " +
        seconds(waited) + " s (" + costmap_ros_->getLayeredCostmap()->staleLayers() + "), " +
        seconds(without_cmd) + " s without a valid command");
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Local costmap not current for %.1f s (%s): commanding zero, %.1f s without a valid "
      "command (failure_tolerance %.1f s)",
      waited, costmap_ros_->getLayeredCostmap()->staleLayers().c_str(), without_cmd,
      failure_tolerance_);
    // Re-sent while waiting: the driver repeats whatever it last received.
    publishZeroVelocity();
    r.sleep();
  }
  return true;
}

void ControllerServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  geometry_msgs::msg::Twist velocity = getThresholdedTwist(odom_sub_->getTwist());

  // Use the latest available map->global_frame transform (Time(0)) instead of the
  // path's original (and now stale) stamp; otherwise long paths hit an
  // "extrapolation into the past" TF error and isGoalReached never returns true.
  geometry_msgs::msg::PoseStamped end_pose_latest = end_pose_;
  end_pose_latest.header.stamp = rclcpp::Time(0);

  // Checked: unchecked, a failed transform left transformed_end_pose at the
  // origin of the costmap's frame, and the goal was judged against that.
  geometry_msgs::msg::PoseStamped transformed_end_pose;
  if (!syncai_util::transformPoseInTargetFrame(
        end_pose_latest, transformed_end_pose, *costmap_ros_->getTfBuffer(),
        costmap_ros_->getGlobalFrameID(), costmap_ros_->getTransformTolerance())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Goal not checked: cannot transform the end pose from %s to %s",
      end_pose_.header.frame_id.c_str(), costmap_ros_->getGlobalFrameID().c_str());
    return false;
  }

  // A path with heights (OctomapPlanner3D) can end right above a stretch of
  // itself: a 2F goal over the 1F corridor that leads to the stairs, or any
  // point of a spiral stair over the turn below it. The goal checkers compare
  // x / y only, so the robot is first required to be on the end pose's level.
  // The robot's height is looked up, not read from `pose`: base_link is planar
  // (path_height.hpp). No height, no goal -- a "reached" one floor short of it
  // is the failure this exists for, and the next cycle tries again.
  const bool path_has_height = pathHasHeight(current_path_);
  double ground_z = 0.0;
  if (path_has_height) {
    std::string error;
    if (!lookupGroundZ(
          *costmap_ros_->getTfBuffer(), current_path_.header.frame_id, body_frame_, body_height_,
          ground_z, &error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Path has heights but the robot's height is unknown (%s); goal not checked: %s",
        body_frame_.c_str(), error.c_str());
      return false;
    }
  }

  // The robot's place on the path, every cycle and before any check below
  // can return: a check that skipped the update (the level check does, for
  // the whole of a 1F -> 2F route's ground floor) would leave the place
  // behind the robot, further than one update reads. Also what the feedback
  // reports distance_to_goal from.
  if (!current_path_.poses.empty()) {
    geometry_msgs::msg::PoseStamped robot_pose_latest = pose;
    robot_pose_latest.header.stamp = rclcpp::Time(0);
    geometry_msgs::msg::PoseStamped robot_in_path_frame;
    if (!syncai_util::transformPoseInTargetFrame(
          robot_pose_latest, robot_in_path_frame, *costmap_ros_->getTfBuffer(),
          current_path_.header.frame_id, costmap_ros_->getTransformTolerance())) {
      return false;
    }
    // Nearest in 3D on a path with heights: on a spiral stair the turn above
    // the robot is as near in x / y as the one under it.
    if (path_has_height) {
      robot_in_path_frame.pose.position.z = ground_z;
    }
    path_progress_.update(current_path_, robot_in_path_frame.pose, path_has_height);
  }

  if (path_has_height) {
    if (std::abs(ground_z - end_pose_.pose.position.z) > goal_z_tolerance_) {
      // Said only near the end pose in x / y, where it is the reason the
      // goal is not reached: a robot standing on its goal with the ground
      // estimate off (lidar_height, the LIO's z drift) otherwise waits
      // silently for the progress checker.
      const double dxy = std::hypot(
        pose.pose.position.x - transformed_end_pose.pose.position.x,
        pose.pose.position.y - transformed_end_pose.pose.position.y);
      if (dxy < 1.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%.2f m from the end pose but on another level: ground z %.2f vs end pose z %.2f "
          "(goal_z_tolerance %.2f); goal not reached",
          dxy, ground_z, end_pose_.pose.position.z, goal_z_tolerance_);
      }
      return false;
    }
  }

  // Proximity to the end pose alone cannot decide completion: on a path that
  // loops back onto its own start (patrol), the robot is within goal tolerance
  // before it has moved. Only consult the goal checker once the remaining
  // length from the robot's place is short. The place is path_progress_'s,
  // which only moves forward (2026-11): this was a whole-path nearest pose,
  // strict '<' so a tie went to the start, which on a loop meant a robot back
  // at A could still read as at the start and never finish.
  // Measured to the end of the leg the controller is driving: on a path cut
  // at its cusps (setPlannerPath()) end_pose_ is that leg's end, and the rest
  // of the path is no reason to keep the checker from it.
  if (goal_reached_max_remaining_path_ > 0.0 && !current_path_.poses.empty()) {
    const size_t leg_end = std::min(leg_ends_[leg_], current_path_.poses.size() - 1);
    double remaining_path_length = 0.0;
    for (size_t i = path_progress_.index(); i < leg_end; ++i) {
      remaining_path_length += syncai_util::geometry_utils::euclidean_distance(
        current_path_.poses[i].pose, current_path_.poses[i + 1].pose);
    }
    if (remaining_path_length > goal_reached_max_remaining_path_) {
      return false;
    }
  }

  if (!goal_checkers_[current_goal_checker_]->isGoalReached(
        pose.pose, transformed_end_pose.pose, velocity)) {
    return false;
  }
  if (leg_ + 1 < legs_.size()) {
    // On a cusp and turned to the next leg: hand it over. Not the goal, and
    // not a controller reset either (see computeControl()) -- the plugin's
    // setPlan() drops its own per-path latches.
    RCLCPP_INFO(
      get_logger(),
      "Reached cusp %zu/%zu at (%.2f, %.2f): %.3f m and %+.3f rad from it; handing over leg "
      "%zu/%zu, %.2f m",
      leg_ + 1, legs_.size() - 1, end_pose_.pose.position.x, end_pose_.pose.position.y,
      std::hypot(
        pose.pose.position.x - transformed_end_pose.pose.position.x,
        pose.pose.position.y - transformed_end_pose.pose.position.y),
      angles::shortest_angular_distance(
        tf2::getYaw(pose.pose.orientation), tf2::getYaw(transformed_end_pose.pose.orientation)),
      leg_ + 2, legs_.size(),
      syncai_util::geometry_utils::calculate_path_length(legs_[leg_ + 1], 0));
    // The goal checker has the robot on the cusp, whatever the turn in place
    // drifted: the place moves past it here, not only once PathProgress sees
    // the robot within its cusp_pass_radius (path_progress.hpp).
    path_progress_.passTo(leg_ends_[leg_]);
    startLeg(leg_ + 1);
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "Reached the goal: %.3f m and %+.3f rad from the end pose (goal checker '%s')",
    std::hypot(
      pose.pose.position.x - transformed_end_pose.pose.position.x,
      pose.pose.position.y - transformed_end_pose.pose.position.y),
    angles::shortest_angular_distance(
      tf2::getYaw(pose.pose.orientation), tf2::getYaw(transformed_end_pose.pose.orientation)),
    current_goal_checker_.c_str());
  return true;
}

void ControllerServer::logPath(const char * what, const nav_msgs::msg::Path & path)
{
  // What the controller was handed, so a log line after it can be read
  // against it: which plugins, how long a path, in which frame, whether it
  // is tracked in 3D, and where it ends.
  const auto & end = path.poses.back().pose;
  RCLCPP_INFO(
    get_logger(),
    "%s: controller '%s', goal checker '%s'; %zu poses, %.2f m in '%s'%s, ending at (%.2f, %.2f, "
    "%.2f) yaw %+.3f",
    what, current_controller_.c_str(), current_goal_checker_.c_str(), path.poses.size(),
    syncai_util::geometry_utils::calculate_path_length(path, 0), path.header.frame_id.c_str(),
    pathHasHeight(path) ? " with heights" : "", end.position.x, end.position.y, end.position.z,
    tf2::getYaw(end.orientation));
  // Said up front, because the stop it causes is otherwise unexplained: the
  // robot halts on each cusp and turns in place before going on.
  if (legs_.size() > 1) {
    std::string where;
    for (size_t k = 0; k + 1 < legs_.size(); ++k) {
      const auto & p = legs_[k].poses.back().pose.position;
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%s(%.2f, %.2f)", k == 0 ? "" : ", ", p.x, p.y);
      where += buf;
    }
    RCLCPP_INFO(
      get_logger(),
      "%s: the path turns back on itself at %zu cusp(s) %s; driven as %zu legs, stopping on each "
      "cusp to turn in place",
      what, legs_.size() - 1, where.c_str(), legs_.size());
  }
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

rcl_interfaces::msg::SetParametersResult ControllerServer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    // Refused at runtime (2026-11). Every controller reads it once, at
    // startup: MPPI pairs it with its model_dt (setOffset(): the warm-start
    // shift is on only when 1 / controller_frequency == model_dt, and a
    // longer period throws), and MPPI and RPP 3D take 1 / f as the
    // dt of their acceleration clamps. A runtime set changed the loop's rate
    // under all of them, so every clamp and the warm start ran on a dt that
    // was no longer true. The backend's settings page does not offer it.
    // Also startup only (2026-11; accepted and never used before): the robot
    // height's frame and offset, which MPPI's path handler, RPP 3D and
    // StairMode copy at startup too, and the track defaults, checked against
    // controller_plugins / goal_checker_plugins once in the constructor.
    if (
      name == "body_frame" || name == "lidar_height" || name == "default_track_controller_id" ||
      name == "default_track_goal_checker_id") {
      result.successful = false;
      result.reason = name +
                      " cannot be changed at runtime: it is read once at startup (by the plugins "
                      "too); change it in the params file and restart controller_server";
      RCLCPP_WARN(get_logger(), "%s", result.reason.c_str());
      return result;
    }

    if (name == "controller_frequency") {
      result.successful = false;
      result.reason =
        "controller_frequency cannot be changed at runtime: the controllers read it at startup "
        "(MPPI's model_dt must equal 1 / controller_frequency); change it in the params file, "
        "with FollowPath.model_dt, and restart controller_server";
      RCLCPP_WARN(get_logger(), "%s", result.reason.c_str());
      return result;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      } else if (name == "goal_reached_max_remaining_path") {
        goal_reached_max_remaining_path_ = parameter.as_double();
      } else if (name == "goal_z_tolerance") {
        goal_z_tolerance_ = parameter.as_double();
      } else if (name == "cusp_min_angle") {
        cusp_params_.min_angle = parameter.as_double();
      } else if (name == "cusp_window") {
        cusp_params_.window = parameter.as_double();
      } else if (name == "dock_max_start_distance") {
        dock_max_start_distance_ = parameter.as_double();
      } else if (name == "dock_max_start_lateral") {
        dock_max_start_lateral_ = parameter.as_double();
      } else if (name == "track_pose_timeout") {
        track_pose_timeout_ = parameter.as_double();
      } else if (name == "track_lost_timeout") {
        track_lost_timeout_ = parameter.as_double();
      } else if (name == "track_max_duration") {
        track_max_duration_ = parameter.as_double();
      } else if (name == "track_target_max_spread_xy") {
        track_target_max_spread_xy_ = parameter.as_double();
      } else if (name == "track_target_max_spread_yaw") {
        track_target_max_spread_yaw_ = parameter.as_double();
      } else if (name == "track_transform_wait") {
        track_transform_wait_ = parameter.as_double();
      } else if (name == "track_max_odom_speed") {
        track_max_odom_speed_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "track_min_poses") {
        track_min_poses_ = static_cast<int>(parameter.as_int());
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace syncai_controller
