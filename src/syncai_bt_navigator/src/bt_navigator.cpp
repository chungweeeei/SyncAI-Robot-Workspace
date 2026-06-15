#include "syncai_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <vector>

#include "syncai_util/node_utils.hpp"

namespace syncai_bt_navigator
{

BtNavigator::BtNavigator(rclcpp::NodeOptions options)
: rclcpp::Node(
    "bt_navigator", "",
    options.automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Only the behavior tree action node plugins ported so far. Extend this list
  // as more nodes (recovery, condition, control) are added.
  const std::vector<std::string> plugin_libs = {
    "syncai_compute_path_to_pose_action_bt_node",
    "syncai_follow_path_action_bt_node",
    "syncai_navigate_to_pose_action_bt_node",
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

BtNavigator::~BtNavigator()
{
}

bool
BtNavigator::configure()
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  pose_navigator_ = std::make_unique<syncai_bt_navigator::NavigateToPoseNavigator>();

  syncai_bt_navigator::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_shared<syncai_util::OdomSmoother>(
    shared_from_this(), 0.3, odom_topic_);

  if (!pose_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure navigate_to_pose navigator");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Configured");
  return true;
}

bool
BtNavigator::cleanup()
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  bool ok = true;
  if (pose_navigator_ && !pose_navigator_->on_cleanup()) {
    ok = false;
  }

  pose_navigator_.reset();
  odom_smoother_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return ok;
}

}  // namespace syncai_bt_navigator
