#include "syncai_behavior_tree/plugins/condition/is_path_valid_condition.hpp"

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "syncai_behavior_tree/blackboard_keys.hpp"

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
  getInput("blocked_time", blocked_time_);
  getInput("blocked_match_distance", blocked_match_distance_);
  if (blocked_time_ < 0.0) {
    throw std::runtime_error("IsPathValid: blocked_time must be >= 0");
  }
  // While a blocked path is being kept (see tick()). Says what happens next
  // either way, since the robot is likely standing in front of the blocker.
  char sentence[96];
  std::snprintf(
    sentence, sizeof(sentence), "Path blocked; rerouting if it is still blocked after %.0f s",
    blocked_time_);
  waiting_sentence_ = sentence;

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

void IsPathValidCondition::resetBlocked()
{
  blocked_since_.reset();
  last_blocked_.clear();
}

BT::NodeStatus IsPathValidCondition::tick()
{
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  getInput("path", request->path);
  // No path yet (first tick of a goal, or ComputePathToPose just failed and
  // wrote an empty one): nothing to keep, so skip the round trip.
  if (request->path.poses.empty()) {
    resetBlocked();
    return BT::NodeStatus::FAILURE;
  }
  // A count belongs to one path. Any new plan -- the replan this node asked
  // for, a changed goal, a new goal (the navigator empties {path} first) --
  // starts from zero; Smac stamps every plan with the time it was made.
  if (
    request->path.header.stamp != counted_path_stamp_ ||
    request->path.poses.size() != counted_path_size_) {
    resetBlocked();
    counted_path_stamp_ = request->path.header.stamp;
    counted_path_size_ = request->path.poses.size();
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
    // FAILURE above: there is nothing to keep. The blocked timer is left
    // running: no answer is no evidence either way.
    RCLCPP_WARN(
      node_->get_logger(),
      "[IsPathValidCondition][%s] is_path_valid timed out after %ld ms; keeping the current path "
      "unchecked",
      __func__, static_cast<long>(server_timeout_.count()));
    return BT::NodeStatus::SUCCESS;
  }

  const auto response = future.get();
  const auto now = std::chrono::steady_clock::now();
  if (response->is_valid) {
    if (blocked_since_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[IsPathValidCondition][%s] Path clear again after %.1f s blocked; keeping it, no replan",
        __func__, std::chrono::duration<double>(now - *blocked_since_).count());
      // Take back this node's own sentence only, as FollowPath does with its
      // retry one: anything else was written later and is the newer news.
      std::string status;
      config().blackboard->get<std::string>(blackboard_keys::kNavStatusMsg, status);  // NOLINT
      if (status == waiting_sentence_) {
        config().blackboard->set<std::string>(  // NOLINT
          blackboard_keys::kNavStatusMsg, blackboard_keys::kStatusFollowing);
      }
      resetBlocked();
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Blocked. Replan only once the same blocker has been reported blocked for
  // blocked_time without a break (2026-10). Until then a single answer was a
  // replan, and the global costmap marks whatever crosses the route within
  // obstacle_max_range (2.5 m), so a person walking across it bought a detour
  // -- on robot01 one ~35 m round a ~11 m leg -- that then stuck for the rest
  // of the goal, because move.xml does not switch back to a route that frees
  // up. A crossing person clears the path in about a second, and the global
  // costmap (1 Hz) forgets them on the update after their cells are seen
  // through; one who stops in the way is given blocked_time to move on.
  // Meanwhile the path is kept and MPPI, on the 5 Hz local costmap, bends
  // round the blocker or stops in front of it. The time runs from the first
  // blocked answer, so with checks ~1 s apart the replan lands on the first
  // check at or past blocked_time. move.xml sets 10 s, close to the ~12 s
  // FollowPath used to survive a blocker in front of it before failing the
  // goal: its retries are sized to outlast blocked_time, and must stay so
  // (see move.xml).
  //
  // "Same blocker": some pose blocked now lies within blocked_match_distance
  // of one blocked last time. The path does not change while waiting, so a
  // standing blocker (and the cells marked round it as more of it comes into
  // view) keeps matching, while different people crossing at different
  // points of the route restart the timer instead of adding up to a replan.
  std::vector<geometry_msgs::msg::Point> blocked;
  blocked.reserve(response->invalid_pose_indices.size());
  for (const auto idx : response->invalid_pose_indices) {
    if (idx >= 0 && static_cast<size_t>(idx) < request->path.poses.size()) {
      blocked.push_back(request->path.poses[static_cast<size_t>(idx)].pose.position);
    }
  }
  // An answer with no indices (the planner's wrong-frame refusal) has no spot
  // to compare, so it matches: the count must still end in a replan.
  bool same_blocker = blocked.empty() || last_blocked_.empty();
  const double match_sq = blocked_match_distance_ * blocked_match_distance_;
  for (const auto & cur : blocked) {
    for (const auto & prev : last_blocked_) {
      const double dx = cur.x - prev.x;
      const double dy = cur.y - prev.y;
      if (dx * dx + dy * dy <= match_sq) {
        same_blocker = true;
        break;
      }
    }
    if (same_blocker) {
      break;
    }
  }
  if (blocked_since_ && !same_blocker) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[IsPathValidCondition][%s] Path blocked at a different spot (more than %.2f m from the "
      "last); restarting the %.0f s wait",
      __func__, blocked_match_distance_, blocked_time_);
  }
  if (!blocked_since_ || !same_blocker) {
    blocked_since_ = now;
  }
  const double blocked_for = std::chrono::duration<double>(now - *blocked_since_).count();
  last_blocked_ = std::move(blocked);

  // The planner logs the full detail (which test, how far ahead); this line is
  // what the task_runner log says about the decision. Nearest blocked pose.
  const double bx = last_blocked_.empty() ? 0.0 : last_blocked_.front().x;
  const double by = last_blocked_.empty() ? 0.0 : last_blocked_.front().y;
  if (blocked_for < blocked_time_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[IsPathValidCondition][%s] Path blocked at (%.2f, %.2f) for %.1f of %.1f s; keeping it",
      __func__, bx, by, blocked_for, blocked_time_);
    // Only a service answer says "blocked": the empty-path and timeout returns
    // above are not evidence of a blocker and leave the sentence alone.
    config().blackboard->set<std::string>(  // NOLINT
      blackboard_keys::kNavStatusMsg, waiting_sentence_);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[IsPathValidCondition][%s] Path blocked at (%.2f, %.2f) for %.1f s; replanning", __func__, bx,
    by, blocked_for);
  resetBlocked();
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg, "Path blocked; replanning");
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::IsPathValidCondition>("IsPathValid");
}
