#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief SUCCESS while the part of {path} ahead of the robot is still free in the
 * planner's global costmap (syncai_global_planner's is_path_valid service), FAILURE otherwise.
 * Port of nav2's IsPathValidCondition.
 *
 * Synchronous on purpose, as upstream: a ConditionNode that never returns RUNNING
 * keeps the ReactiveSequence / Fallback around it trivial to reason about, and the
 * call is one costmap scan on the planner's otherwise idle main executor. A timeout
 * answers SUCCESS (keep the path, re-check next tick) and only an empty path or a
 * `false` from the service answers FAILURE: a replan replaces the route, so it has
 * to be earned by an answer, not by the lack of one (see tick()).
 *
 * Not upstream: a `false` answers FAILURE only once the answers have said so
 * about the same spot (within `blocked_match_distance`) of the same path, without
 * a `true` in between, for `blocked_time` seconds; the ones before keep the path.
 * A person crossing or stopping on the route is given that long to leave, since
 * the detour a single answer would have bought sticks for the rest of the goal
 * (2026-10). blocked_time="0" is upstream's behaviour.
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
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::InputPort<double>(
        "blocked_time", 0.0, "Seconds one spot must stay blocked before FAILURE (replan)"),
      BT::InputPort<double>(
        "blocked_match_distance", 0.5,
        "Metres between two answers' blocked poses that still count as the same blocker")};
  }

private:
  // Forget the blocked answers counted so far (see tick()).
  void resetBlocked();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  std::chrono::milliseconds server_timeout_;

  double blocked_time_{0.0};
  double blocked_match_distance_{0.5};
  // The status sentence while a blocked path is kept; names blocked_time_.
  std::string waiting_sentence_;
  // Since when one spot of the path identified by the stamp and size below has
  // been answered blocked without a break (empty: it is not), and the last
  // answer's blocked poses (nearest the robot first).
  std::optional<std::chrono::steady_clock::time_point> blocked_since_;
  builtin_interfaces::msg::Time counted_path_stamp_;
  size_t counted_path_size_{0};
  std::vector<geometry_msgs::msg::Point> last_blocked_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
