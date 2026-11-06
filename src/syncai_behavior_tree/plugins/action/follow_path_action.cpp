#include "syncai_behavior_tree/plugins/action/follow_path_action.hpp"

#include <memory>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

// Humble's FollowPath result carries no reason, so the sentence names the
// three causes the controller aborts on (see on_aborted).
static constexpr char kControllerGaveUp[] =
  "Controller gave up (blocked, no progress, or pose lost); clearing the local costmap and "
  "retrying";

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

BT::NodeStatus FollowPathAction::tick()
{
  // Why (2026-11): {path} is empty while the planner branch replans after a
  // failure (ComputePathToPose::on_aborted clears it) and after a preempt
  // (the navigator clears it). A controller abort in that window -- a
  // blocker, the progress checker -- failed this node, the RecoveryNode
  // cleared the costmap and ticked it again, it failed at once on the empty
  // path (on_tick() below; tick() fails when no goal is sent), and so on:
  // move.xml's four retries went in as many ticks, and the goal ended
  // FOLLOW_PATH_FAILED while the planner was still running and could well
  // have found a way. Holding RUNNING without a goal until a path lands
  // spends one retry per controller abort, as meant. It cannot hang:
  // PipelineSequence keeps ticking the planner branch, IsPathValid fails on
  // an empty path so a plan starts within a second, and a planner failure
  // fails the tree and halts this node.
  if (status() == BT::NodeStatus::IDLE || waiting_for_path_) {
    nav_msgs::msg::Path path;
    getInput("path", path);
    if (path.poses.empty()) {
      if (!waiting_for_path_) {
        waiting_for_path_ = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "[FollowPathAction][%s] No path to follow yet (the planner is replanning); holding "
          "until one lands",
          __func__);
      }
      return BT::NodeStatus::RUNNING;
    }
    if (waiting_for_path_) {
      waiting_for_path_ = false;
      // BtActionNode::tick() starts a goal only from IDLE.
      setStatus(BT::NodeStatus::IDLE);
      RCLCPP_INFO(
        node_->get_logger(), "[FollowPathAction][%s] A path landed; following it", __func__);
    }
  }
  return BtActionNode<nav2_msgs::action::FollowPath>::tick();
}

void FollowPathAction::halt()
{
  waiting_for_path_ = false;
  BtActionNode<nav2_msgs::action::FollowPath>::halt();
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);

  // tick() holds until {path} has poses, so this is only a guard: sent on,
  // an empty path throws "Invalid path, Path is empty." in the controller at
  // goal start, which reads as a controller fault.
  if (goal_.path.poses.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[FollowPathAction][%s] No path to follow (empty path); not sending the goal", __func__);
    should_send_goal_ = false;
    return;
  }
  sent_generation_ = goalGeneration();
  waiting_logged_ = false;

  // A goal is going out: the first drive (ComputePathToPose already said
  // "Following the path" in this tick) or a retry after the controller gave
  // up. Only the retry needs saying, and only over this node's own sentence:
  // overwriting unconditionally would hide a replan the planner branch
  // started in the same tick (it is ticked first) for its whole duration.
  std::string status;
  config().blackboard->get<std::string>(blackboard_keys::kNavStatusMsg, status);  // NOLINT
  if (status == kControllerGaveUp) {
    config().blackboard->set<std::string>(  // NOLINT
      blackboard_keys::kNavStatusMsg, blackboard_keys::kStatusFollowing);
  }
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> /*feedback*/)
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Re-send when the path changed, unless there is nothing to follow. Upstream
  // screens only the all-default message (`new_path != nav_msgs::msg::Path()`),
  // which is what ComputePathToPose writes on abort / cancel / halt and what
  // the navigator writes at every new goal, so the two tests coincide today
  // -- but the property that matters is "no poses", and a stamped-but-empty
  // path would have been sent on and thrown in the controller mid-drive,
  // taking the retry with it.
  if (goal_.path != new_path && !new_path.poses.empty()) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
    sent_generation_ = goalGeneration();
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

BT::NodeStatus FollowPathAction::on_aborted()
{
  // Humble's FollowPath result carries no reason either, so "patience
  // exceeded", "failed to make progress" and a lost robot pose all arrive
  // here as one ABORTED; the controller log has which. The navigator turns
  // this into FOLLOW_PATH_FAILED if the tree ends on it.
  report_failure("controller " + action_name_ + " aborted the goal");
  // A retry, as in ComputePathToPose::on_aborted: move.xml clears the local
  // costmap and sends the path again, up to three times, and the last
  // failure ends the goal with error_msg instead.
  config().blackboard->set<std::string>(
    blackboard_keys::kNavStatusMsg, kControllerGaveUp);  // NOLINT
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPathAction::on_success()
{
  // Why (2026-11): a preempt empties {path} until the new goal's plan lands,
  // and on_wait_for_result() keeps the old path on the controller meanwhile
  // (an empty one is never sent). If the robot reached the old goal in that
  // window -- a retarget sent while it was finishing its final rotation --
  // the controller succeeded, PipelineSequence took that as the tree's
  // success while the planner was still RUNNING, and the navigator reported
  // the NEW goal reached, standing at the old one; the task went on to its
  // next step in the wrong place.
  const int generation = goalGeneration();
  if (generation == sent_generation_) {
    return BT::NodeStatus::SUCCESS;
  }
  // The old path is done and the controller has stopped. Stay RUNNING, which
  // brings the tick back here (BtActionNode::tick() keeps the result), until
  // the planner branch lands the new goal's path; a planner failure fails
  // the tree through PipelineSequence and halts this node as usual.
  nav_msgs::msg::Path path;
  getInput("path", path);
  if (path.poses.empty()) {
    if (!waiting_logged_) {
      waiting_logged_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "[FollowPathAction][%s] Reached the end of the preempted goal's path; holding until the "
        "new goal's path is planned",
        __func__);
    }
    return BT::NodeStatus::RUNNING;
  }
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  goal_.path = path;
  sent_generation_ = generation;
  waiting_logged_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "[FollowPathAction][%s] The new goal's path landed after the preempted one finished; "
    "following it",
    __func__);
  send_new_goal();
  return BT::NodeStatus::RUNNING;
}

int FollowPathAction::goalGeneration()
{
  int generation = 0;
  config().blackboard->get<int>(blackboard_keys::kGoalGeneration, generation);  // NOLINT
  return generation;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::FollowPathAction>(name, "follow_path", config);
  };

  factory.registerBuilder<syncai_behavior_tree::FollowPathAction>("FollowPath", builder);
}
