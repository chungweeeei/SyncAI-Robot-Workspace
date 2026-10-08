#include "syncai_behavior_tree/plugins/condition/is_path_valid_condition.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace syncai_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // Own callback group + executor, like BtServiceNode: spinning the shared
  // task_runner node from inside a tick would run unrelated callbacks here.
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  server_timeout_ = config().blackboard->get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  std::string service_name = "is_path_valid";
  getInput("service_name", service_name);
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>(
    service_name, rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

  // Same contract as the other BT clients: fail tree construction (and so the
  // goal) loudly if the planner is not up, rather than answer FAILURE forever
  // and silently replan every tick.
  auto wait_for_service_timeout =
    config().blackboard->get<std::chrono::milliseconds>("wait_for_service_timeout");
  if (!client_->wait_for_service(wait_for_service_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[IsPathValidCondition][%s] \"%s\" service not available after %.2fs",
      __func__, service_name.c_str(), wait_for_service_timeout.count() / 1000.0);
    throw std::runtime_error("Service server " + service_name + " not available");
  }
}

BT::NodeStatus IsPathValidCondition::tick()
{
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  getInput("path", request->path);
  // No path yet (first tick of a goal, or ComputePathToPose just failed and
  // wrote an empty one): nothing to keep, so skip the round trip.
  if (request->path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  auto future = client_->async_send_request(request);
  if (
    callback_group_executor_.spin_until_future_complete(future, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    // Drop the abandoned request, or its late reply has nowhere to go and the
    // client's pending map grows by one per timeout.
    client_->remove_pending_request(future);
    // A timeout keeps the current path (SUCCESS); it does not replan. The two
    // mistakes are not symmetric. A false "invalid" replaces a route nobody
    // has shown to be blocked -- a fresh plan always wins, and the 2D-raytraced
    // global costmap forgets a blocker as soon as the robot can see past it,
    // so the replacement can be the route that ran into it; stopping that is
    // the reason this branch of the tree exists. A false "valid" delays a
    // replan by one RateController tick (1 s), and in that second RPP's own
    // collision check holds the robot (zero cmd_vel under failure_tolerance)
    // if the blocker is already within its projection. The timeout itself is
    // the global costmap's 1 Hz update holding the costmap mutex (layers plus
    // the keepout filter) past server_timeout (100 ms in move.xml) while the
    // service waits for it. If this WARN shows up every tick, the two 1 Hz
    // loops have phase-locked and the path is going unchecked: raise
    // server_timeout in move.xml, do not flip this back to FAILURE -- that
    // was a 1 Hz replan loop in disguise (until 2026-10). An empty path stays
    // FAILURE above: there is nothing to keep.
    RCLCPP_WARN(
      node_->get_logger(),
      "[IsPathValidCondition][%s] is_path_valid timed out after %ld ms; keeping the current path "
      "unchecked",
      __func__, static_cast<long>(server_timeout_.count()));
    return BT::NodeStatus::SUCCESS;
  }

  return future.get()->is_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::IsPathValidCondition>("IsPathValid");
}
