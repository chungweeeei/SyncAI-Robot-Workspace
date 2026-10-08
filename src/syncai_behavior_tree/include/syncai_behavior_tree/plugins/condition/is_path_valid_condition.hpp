#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief SUCCESS while the part of {path} ahead of the robot is still free in the
 * planner's global costmap (syncai_planner's is_path_valid service), FAILURE otherwise.
 * Port of nav2's IsPathValidCondition.
 *
 * Synchronous on purpose, as upstream: a ConditionNode that never returns RUNNING
 * keeps the ReactiveSequence / Fallback around it trivial to reason about, and the
 * call is one costmap scan on the planner's otherwise idle main executor. A timeout
 * answers SUCCESS (keep the path, re-check next tick) and only an empty path or a
 * `false` from the service answers FAILURE: a replan replaces the route, so it has
 * to be earned by an answer, not by the lack of one (see tick()).
 */
class IsPathValidCondition : public BT::ConditionNode
{
public:
  IsPathValidCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPathValidCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
      BT::InputPort<std::string>("service_name", "is_path_valid", "Planner's validity service"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  std::chrono::milliseconds server_timeout_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
