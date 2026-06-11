#include "syncai_navigator/navigator.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace syncai_navigator
{

Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp::Node("navigator", options)
{
  RCLCPP_INFO(get_logger(), "Creating navigator");

  // Relative names: both resolve under this node's namespace, matching the
  // planner_server / controller_server launched in the same namespace.
  compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this, "compute_path_to_pose");
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    this, "follow_path");

  // RViz's "2D Goal Pose" tool publishes /goal_pose by default; point it to
  // /<namespace>/goal_pose (Tool Properties) or remap this subscription.
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(1),
    std::bind(&Navigator::onGoalPose, this, _1));

  RCLCPP_INFO(get_logger(), "Navigator ready, waiting for goal poses");
}

void Navigator::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  RCLCPP_INFO(
    get_logger(), "Received goal pose (%.2f, %.2f) in frame %s",
    goal->pose.position.x, goal->pose.position.y, goal->header.frame_id.c_str());

  if (!compute_path_client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(get_logger(), "compute_path_to_pose action server not available");
    return;
  }

  ComputePathToPose::Goal path_goal;
  path_goal.goal = *goal;
  path_goal.use_start = false;  // plan from the robot's current pose

  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&Navigator::onComputePathResult, this, _1);

  compute_path_client_->async_send_goal(path_goal, send_goal_options);
}

void Navigator::onComputePathResult(const ComputePathGoalHandle::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(get_logger(), "Planning failed (result code %d)", static_cast<int>(result.code));
    return;
  }

  const auto & path = result.result->path;
  if (path.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "Planner returned an empty path");
    return;
  }

  RCLCPP_INFO(get_logger(), "Received path with %zu poses, forwarding to controller", path.poses.size());

  if (!follow_path_client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(get_logger(), "follow_path action server not available");
    return;
  }

  FollowPath::Goal follow_goal;
  follow_goal.path = path;

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&Navigator::onFollowPathResult, this, _1);

  follow_path_client_->async_send_goal(follow_goal, send_goal_options);
}

void Navigator::onFollowPathResult(const FollowPathGoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded: goal reached");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Navigation aborted by the controller");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Navigation canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Navigation finished with unknown result code");
      break;
  }
}

}  // namespace syncai_navigator
