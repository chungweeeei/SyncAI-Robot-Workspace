#include "syncai_task_runner/syncai_task_runner.hpp"

#include <memory>
#include <string>
#include <vector>

#include "syncai_util/node_utils.hpp"

namespace syncai_task_runner
{
TaskRunner::TaskRunner(rclcpp::NodeOptions options)
: rclcpp::Node(
    "task_runner", "",
    options.automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(this->get_logger(), "[TaskRunner][%s] Creating", __func__);

  // BT node plugins ported so far. Extend this list as more nodes
  // (recovery, condition, control, decorator) are added.
  const std::vector<std::string> plugin_libs = {
    "syncai_compute_path_to_pose_action_bt_node",
    "syncai_compute_path_through_poses_action_bt_node",
    "syncai_follow_path_action_bt_node",
    "syncai_remove_passed_goals_action_bt_node",
    "syncai_pipeline_sequence_bt_node",
    "syncai_rate_controller_bt_node",
  };

  syncai_util::declare_parameter_if_not_declared(
    this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
  syncai_util::declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));
  syncai_util::declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  syncai_util::declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  syncai_util::declare_parameter_if_not_declared(
    this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
}

TaskRunner::~TaskRunner()
{
}

bool TaskRunner::initialize()
{
  RCLCPP_INFO(this->get_logger(), "[TaskRunner][%s] Initializing TaskRunner node...", __func__);

  // Initialize transform buffer && transform listener
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = this->get_parameter("global_frame").as_string();
  RCLCPP_INFO(
    this->get_logger(), "[TaskRunner][%s] Global frame: %s", __func__, global_frame_.c_str());

  robot_frame_ = this->get_parameter("robot_base_frame").as_string();
  RCLCPP_INFO(
    this->get_logger(), "[TaskRunner][%s] Robot base frame: %s", __func__, robot_frame_.c_str());

  transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  RCLCPP_INFO(
    this->get_logger(), "[TaskRunner][%s] Odometry topic: %s", __func__, odom_topic_.c_str());

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = this->get_parameter("plugin_lib_names").as_string_array();

  pose_navigator_ = std::make_unique<syncai_task_runner::NavigateToPoseNavigator>();
  poses_navigator_ = std::make_unique<syncai_task_runner::NavigateThroughPosesNavigator>();

  syncai_task_runner::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // Odometry smoother for getting current speed (used to compute feedback).
  odom_smoother_ =
    std::make_shared<syncai_util::OdomSmoother>(this->shared_from_this(), 0.3, odom_topic_);

  if (!pose_navigator_->on_initialize(
        this->shared_from_this(), plugin_lib_names, feedback_utils, &plugin_mutex_,
        odom_smoother_)) {
    RCLCPP_ERROR(
      this->get_logger(), "[TaskRunner][%s] Failed to initialize navigate_to_pose navigator",
      __func__);
    return false;
  }

  if (!poses_navigator_->on_initialize(
        this->shared_from_this(), plugin_lib_names, feedback_utils, &plugin_mutex_,
        odom_smoother_)) {
    RCLCPP_ERROR(
      this->get_logger(), "[TaskRunner][%s] Failed to initialize navigate_through_poses navigator",
      __func__);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[TaskRunner][%s] Initialized", __func__);
  return true;
}

bool TaskRunner::cleanup()
{
  RCLCPP_INFO(this->get_logger(), "[TaskRunner][%s] Cleaning up", __func__);

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  bool ok = true;
  if (pose_navigator_ && !pose_navigator_->on_cleanup()) {
    ok = false;
  }
  if (poses_navigator_ && !poses_navigator_->on_cleanup()) {
    ok = false;
  }

  pose_navigator_.reset();
  poses_navigator_.reset();
  odom_smoother_.reset();

  RCLCPP_INFO(this->get_logger(), "[TaskRunner][%s] Completed cleaning up", __func__);
  return ok;
}

}  // namespace syncai_task_runner
