#include "syncai_controller/controller_server.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "syncai_nav_core/exceptions.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/robot_utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace syncai_controller
{

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
  default_types_{"syncai_controller::RegulatedPurePursuitController"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));
  declare_parameter("goal_reached_max_remaining_path", rclcpp::ParameterValue(1.0));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<syncai_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  action_server_.reset();
  dyn_params_handler_.reset();
  speed_limit_sub_.reset();
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

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
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

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("publish_zero_velocity", publish_zero_velocity_);
  get_parameter("goal_reached_max_remaining_path", goal_reached_max_remaining_path_);

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

  odom_sub_ = std::make_unique<syncai_util::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method.
  // spin_thread=true gives the server its own callback group and executor
  // thread, so the (blocking) execute callback never starves this node's
  // main executor.
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(), "follow_path", std::bind(&ControllerServer::computeControl, this), nullptr,
    std::chrono::milliseconds(500), true);

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  // Activation phase (nav2 on_activate)
  costmap_ros_->activate();
  action_server_->activate();

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
        "FollowPath called with controller name %s, "
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
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.",
        goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

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

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz", controller_frequency_);
        loop_rate.reset();
      }
    }
  } catch (syncai_nav_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  if (publish_zero_velocity_) {
    publishZeroVelocity();
  }

  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(get_logger(), "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw syncai_nav_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)", end_pose_.pose.position.x,
    end_pose_.pose.position.y);

  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw syncai_nav_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    throw syncai_nav_core::PlannerException("Failed to make progress");
  }

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(
      pose, twist, goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (syncai_nav_core::PlannerException & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if (
        (now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0) {
        throw syncai_nav_core::PlannerException("Controller patience exceeded");
      }
    } else {
      throw syncai_nav_core::PlannerException(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx = [&pose, &current_path]() {
    size_t closest_pose_idx = 0;
    double curr_min_dist = std::numeric_limits<double>::max();
    for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
      double curr_dist =
        syncai_util::geometry_utils::euclidean_distance(pose, current_path.poses[curr_idx]);
      if (curr_dist < curr_min_dist) {
        curr_min_dist = curr_dist;
        closest_pose_idx = curr_idx;
      }
    }
    return closest_pose_idx;
  };

  feedback->distance_to_goal =
    syncai_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
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

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  syncai_util::transformPoseInTargetFrame(
    end_pose_latest, transformed_end_pose, *costmap_ros_->getTfBuffer(),
    costmap_ros_->getGlobalFrameID(), costmap_ros_->getTransformTolerance());

  // Proximity to the end pose alone cannot decide completion: on a path that
  // loops back onto its own start (patrol), the robot is within goal tolerance
  // before it has moved. Only consult the goal checker once the remaining
  // length from the path pose closest to the robot is short.
  if (goal_reached_max_remaining_path_ > 0.0 && !current_path_.poses.empty()) {
    geometry_msgs::msg::PoseStamped robot_pose_latest = pose;
    robot_pose_latest.header.stamp = rclcpp::Time(0);

    geometry_msgs::msg::PoseStamped robot_in_path_frame;
    if (!syncai_util::transformPoseInTargetFrame(
          robot_pose_latest, robot_in_path_frame, *costmap_ros_->getTfBuffer(),
          current_path_.header.frame_id, costmap_ros_->getTransformTolerance())) {
      return false;
    }

    // Strict '<' keeps the earliest pose on ties, so on a closed loop the
    // robot standing on the shared start/end point resolves to the start.
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
      double dist = syncai_util::geometry_utils::euclidean_distance(
        robot_in_path_frame, current_path_.poses[i]);
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    double remaining_path_length =
      syncai_util::geometry_utils::calculate_path_length(current_path_, closest_idx);
    if (remaining_path_length > goal_reached_max_remaining_path_) {
      return false;
    }
  }

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose, velocity);
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

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
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
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      } else if (name == "goal_reached_max_remaining_path") {
        goal_reached_max_remaining_path_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace syncai_controller
