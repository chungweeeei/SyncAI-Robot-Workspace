#include "syncai_task_runner/navigators/pose_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "syncai_behavior_tree/blackboard_keys.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/path_height.hpp"
#include "syncai_util/robot_utils.hpp"

namespace syncai_task_runner
{

namespace
{

namespace keys = syncai_behavior_tree::blackboard_keys;
using GoalResult = syncai_common::action::NavigateToGoal::Result;

struct FailureReason
{
  uint16_t code;
  std::string msg;
};

// failed_node -> (error_code, operator sentence). The number lives here and
// not in the BT package because it belongs to the action this navigator
// serves (blackboard_keys.hpp has the argument); the BT nodes only say which
// of them failed. A node absent from this table maps to UNKNOWN with its own
// message, which is safe but uninformative -- add a row when a new node
// starts calling report_failure(). The sentences are what the console shows
// the operator; the node's detail follows in parentheses.
const std::map<std::string, std::pair<uint16_t, const char *>> kStageByFailedNode = {
  {"ComputePathToPose", {GoalResult::PLAN_FAILED, "Planning failed: no path to the goal"}},
  // patrol.xml (2026-11). Not reported by navigate_through_poses' nav2 result,
  // which has no fields; logged, and ready for its syncai_common twin.
  {"ComputePathThroughPoses",
   {GoalResult::PLAN_FAILED, "Planning failed: no route through the waypoints"}},
  {"FollowPath",
   {GoalResult::FOLLOW_PATH_FAILED,
    "Path following failed: the robot could not reach the goal along its path"}},
  // The gait nodes, written for the docking branch of dock.xml / undock.xml
  // (2026-10), which went on 2026-11-12 with DockToPose, ComputeStagingPose
  // and ComputeBackOutPose and their rows. 300 is the docking range's base,
  // the same contract as 100 / 200; the gait switch stays mapped there
  // because a dock-like last metre is what it was written for -- a tree that
  // switches the gait for anything else needs a tree-aware answer. No tree
  // uses either node today. Wait never fails on a configured tree and has no
  // row. A refused key is a gait switch (MPC or RL).
  {"SetMotionKey",
   {GoalResult::DOCK_FAILED, "Gait switch failed: the gait controller refused a motion command"}},
  // 2026-10-14: the check after a gait switch; the detail names the state
  // waited for and the last one the controller reported.
  {"WaitForMotionState",
   {GoalResult::DOCK_FAILED,
    "Gait switch failed: the gait controller did not reach the requested motion state"}},
};

// The planner's own reasons (2026-11): ComputePathToPose's error_code, which
// the planner nodes pass on in failure_code and which replaces their row's
// PLAN_FAILED, with a sentence that says it. The numbers are the interface's
// (one table in four files, syncai_common); a code not listed here is still
// copied through, under the planner node's own sentence, so a code the
// interface adds first is never lost to a stale table. Phrased for a single
// goal: on a patrol route the detail in parentheses names the waypoint.
const std::map<uint16_t, const char *> kPlanningSentenceByCode = {
  {GoalResult::INVALID_PLANNER, "Planning failed: the requested planner is not loaded"},
  {GoalResult::TF_ERROR, "Planning failed: no robot pose or transform to plan with"},
  {GoalResult::START_OUTSIDE_MAP, "Planning failed: the robot is outside the map"},
  {GoalResult::GOAL_OUTSIDE_MAP, "Planning failed: the goal is outside the map"},
  {GoalResult::START_OCCUPIED, "Planning failed: no free ground where the robot stands"},
  {GoalResult::GOAL_OCCUPIED,
   "Planning failed: no free ground at the goal (a wall, a ledge, another floor, or an "
   "obstacle on it)"},
  {GoalResult::TIMEOUT, "Planning failed: the search for a path ran out of time"},
  {GoalResult::NO_VALID_PATH, "Planning failed: the way to the goal is closed"},
  {GoalResult::NO_VIAPOINTS_GIVEN, "Planning failed: the route has no waypoints"},
  {GoalResult::MAP_NOT_LOADED, "Planning failed: no map is loaded"},
  {GoalResult::MAP_LOADING, "Planning failed: the map is still loading"},
  {GoalResult::START_IN_KEEPOUT, "Planning failed: the robot stands inside a keepout zone"},
  {GoalResult::GOAL_IN_KEEPOUT, "Planning failed: the goal is inside a keepout zone"},
  {GoalResult::DISCONNECTED,
   "Planning failed: the goal is on ground no stair connects to the robot's"},
};

// NavigateToGoal.behavior_tree, as the client sent it, to the file to load.
// A bare file name ("patrol.xml") resolves against this package's own
// behavior_trees/ directory, so the backend does not have to embed the
// install prefix -- a workspace-layout detail nothing else here exposes.
// Anything with a '/' is a path and goes through unchanged (loadBehaviorTree
// is a plain std::ifstream open: no search of its own). Also applied to a
// preempting goal's name before the same-tree comparison, or "patrol.xml"
// against a running absolute-path patrol would read as a different tree.
std::string resolveBtFilename(const std::string & requested)
{
  if (requested.empty() || requested.find('/') != std::string::npos) {
    return requested;
  }
  return ament_index_cpp::get_package_share_directory("syncai_task_runner") + "/behavior_trees/" +
         requested;
}

FailureReason readFailureReason(const BT::Blackboard::Ptr & blackboard)
{
  std::string failed_node;
  std::string failure_msg;
  blackboard->get<std::string>(keys::kFailedNode, failed_node);
  blackboard->get<std::string>(keys::kFailureMsg, failure_msg);

  const auto stage = kStageByFailedNode.find(failed_node);
  if (stage != kStageByFailedNode.end()) {
    // A planner node's own finer code (failure_code, 2026-11): only inside
    // the planning range and above its base, so a node that wrote none (0)
    // or a stray value keeps its row's code. Read for the planner nodes only
    // -- a node that writes failed_node by hand (Wait) does not write it.
    uint16_t code = 0;
    blackboard->get<uint16_t>(keys::kFailureCode, code);
    if (
      stage->second.first == GoalResult::PLAN_FAILED && code > GoalResult::PLAN_FAILED &&
      code < GoalResult::DOCK_FAILED) {
      const auto sentence = kPlanningSentenceByCode.find(code);
      const char * text =
        sentence != kPlanningSentenceByCode.end() ? sentence->second : stage->second.second;
      return {code, std::string(text) + " (" + failure_msg + ")"};
    }
    return {stage->second.first, std::string(stage->second.second) + " (" + failure_msg + ")"};
  }
  if (failed_node.empty() && !failure_msg.empty()) {
    // Unattributed by the node itself (a server never acknowledged the goal,
    // or rejected it): the message is the whole story, and blaming planning
    // or driving would be a guess.
    return {GoalResult::UNKNOWN, failure_msg};
  }
  if (failed_node.empty()) {
    // Nothing wrote: an exception inside the tree, which BehaviorTreeEngine
    // logs and swallows, or a node that fails without reporting.
    return {
      GoalResult::UNKNOWN,
      "Navigation failed without a node reporting a reason (see the task_runner log)"};
  }
  return {GoalResult::UNKNOWN, failed_node + ": " + failure_msg};
}

}  // namespace

template <class ActionT>
bool PoseNavigator<ActionT>::configure(
  rclcpp::Node::WeakPtr parent_node, std::shared_ptr<syncai_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  // Node-level parameters, shared by both instantiations: the second one to
  // configure finds them declared and reads the same values, which is the
  // point -- same tree, same blackboard keys.
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  odom_smoother_ = odom_smoother;

  if constexpr (Traits::kThroughPoses) {
    // Generous: a waypoint is passed through, not stopped on, so the robot
    // only comes within the path's tracking error of it (more on an MPPI
    // detour round a person), and a waypoint missed by this test is not
    // driven back to unless the path is replanned before the next one.
    if (!node->has_parameter("waypoint_passed_radius")) {
      node->declare_parameter("waypoint_passed_radius", 1.0);
    }
    waypoint_passed_radius_ = node->get_parameter("waypoint_passed_radius").as_double();
    // The four navigators share one node, so each declares only once.
    if (!node->has_parameter("waypoint_passed_z_tolerance")) {
      node->declare_parameter("waypoint_passed_z_tolerance", 1.0);
    }
    if (!node->has_parameter("body_frame")) {
      node->declare_parameter("body_frame", std::string("pointlio_body"));
    }
    if (!node->has_parameter("lidar_height")) {
      node->declare_parameter("lidar_height", 0.481);
    }
    waypoint_passed_z_tolerance_ = node->get_parameter("waypoint_passed_z_tolerance").as_double();
    body_frame_ = node->get_parameter("body_frame").as_string();
    lidar_height_ = node->get_parameter("lidar_height").as_double();
  }

  // On the node's default callback group, which main()'s spin services, so
  // the reply is consumed there and goalCompleted() never waits on it.
  clear_plan_client_ = node->create_client<std_srvs::srv::Empty>("clear_plan");

  if constexpr (Traits::kDrivesRvizGoalPose) {
    // register action client
    self_client_ = rclcpp_action::create_client<ActionT>(node, this->getName());

    // register subscription  to receiving goal pose from rviz
    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  }

  return true;
}

template <class ActionT>
std::string PoseNavigator<ActionT>::getDefaultBTFilepath(rclcpp::Node::WeakPtr parent_node)
{
  auto node = parent_node.lock();

  std::string default_bt_xml_filename;
  if (!node->has_parameter(Traits::kDefaultBtParam)) {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("syncai_task_runner");
    node->declare_parameter<std::string>(
      Traits::kDefaultBtParam, pkg_share_dir + "/behavior_trees/" + Traits::kDefaultBtFile);
  }

  node->get_parameter(Traits::kDefaultBtParam, default_bt_xml_filename);

  return default_bt_xml_filename;
}

template <class ActionT>
bool PoseNavigator<ActionT>::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  clear_plan_client_.reset();
  return true;
}

template <class ActionT>
std::string PoseNavigator<ActionT>::selectBtFilename(const std::string & requested) const
{
  std::string filename = resolveBtFilename(requested);
  if (filename.empty()) {
    filename = this->bt_action_server_->getDefaultBTFilename();
  }
  return filename;
}

template <class ActionT>
bool PoseNavigator<ActionT>::goalReceived(typename ActionT::Goal::ConstSharedPtr goal)
{
  if constexpr (Traits::kThroughPoses) {
    if (goal->poses.empty()) {
      RCLCPP_ERROR(
        this->logger_, "[PoseNavigator][%s] %s: a route with no poses; navigation canceled",
        __func__, this->getName().c_str());
      return false;
    }
  }
  const std::string bt_xml_filename = selectBtFilename(goal->behavior_tree);
  RCLCPP_INFO(
    this->logger_, "[PoseNavigator][%s] %s: running %s", __func__, this->getName().c_str(),
    bt_xml_filename.c_str());

  if (!this->bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    // Two causes, one return value: the file is missing, or a node in it
    // threw while being constructed -- a BtActionNode / BtServiceNode whose
    // server is not up within wait_for_service_timeout (a server only some
    // tree uses, set_motion_key for one, is first waited for here, at that
    // tree's first goal, not at startup). BtActionServer logged which just
    // above this line.
    RCLCPP_ERROR(
      this->logger_,
      "[PoseNavigator][%s] %s: could not load BT %s (file missing, or a node's server is down "
      "-- see the lines above). Navigation canceled.",
      __func__, this->getName().c_str(), bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

template <class ActionT>
void PoseNavigator<ActionT>::goalCompleted(
  typename ActionT::Result::SharedPtr result, const syncai_behavior_tree::BtStatus final_bt_status)
{
  // SUCCEEDED and CANCELED carry NONE: a default-constructed result already
  // says so. A superseded goal never reaches here at all (the action server
  // aborts it with an empty result from accept_pending_goal()), nor does one
  // rejected in goalReceived(); both read as ABORTED + NONE on the client,
  // which the .action file documents as "superseded or not started".
  if (
    final_bt_status == syncai_behavior_tree::BtStatus::SUCCEEDED ||
    final_bt_status == syncai_behavior_tree::BtStatus::CANCELED) {
    clearPlannerPlan();
    return;
  }

  const FailureReason reason = readFailureReason(this->bt_action_server_->getBlackboard());

  // Logged for both actions: navigate_to_pose's clients get the reason in the
  // task_runner log before they migrate, and the log is the only place the
  // nav2 result can carry it.
  RCLCPP_ERROR(
    this->logger_, "[PoseNavigator][%s] %s: navigation failed (error_code %u): %s", __func__,
    this->getName().c_str(), static_cast<unsigned>(reason.code), reason.msg.c_str());

  if constexpr (Traits::kReportsFailure) {
    result->error_code = reason.code;
    result->error_msg = reason.msg;
  } else {
    (void)result;
  }
}

template <class ActionT>
void PoseNavigator<ActionT>::clearPlannerPlan()
{
  // The planner re-sends its last plan on `plan` once a second and cannot
  // tell when the navigation that asked for it ended, so the console went on
  // drawing a finished route until the next goal replaced it. Cleared on a
  // cancel (2026-10-14) and on a success (2026-10-22). Success was left out at
  // first on the reasoning that the re-send trims a reached route to a stub
  // no longer than xy_goal_tolerance, but the trim starts at the pose nearest
  // the robot anywhere on the whole plan: the stub stayed drawn, and grew
  // back along the route as soon as the robot moved off the goal (a joystick
  // drive, a task's next step that does not plan). Not on a failure: there
  // the route is what the operator wants to see to understand it, and the
  // planner drops it itself once no navigation has used it for
  // plan_idle_clear_time (30 s, 2026-10-25). A preempt
  // never comes here -- the tree keeps running and the next plan replaces the
  // route anyway.
  //
  // Fire and forget: goalCompleted() runs on the action server's thread
  // before the result is sent, and an unreachable planner must not delay
  // that. The next goal's first plan cannot race this request in practice:
  // the request is out before the client hears that this goal ended, and a
  // task's next step reaches the planner only after a round trip through the
  // backend and its workflow, then the new tree's clear_overlay. A clear
  // that did land after the next plan would empty that plan's route on the
  // console until the next replan (is_path_valid still finds Octo3D: with
  // one planner loaded an empty planner id falls back to it). It also sends
  // an empty plan_crossings; the controllers' stair mode keeps the last few
  // words matched on the plan's stamp (2026-11-09), so the held plan keeps
  // its gait and speed spans, while the terrain layer, which holds the
  // empty plan by then, opens no corridor until the next replan.
  if (!clear_plan_client_->service_is_ready()) {
    RCLCPP_WARN(
      this->logger_,
      "[PoseNavigator][%s] %s: planner's clear_plan is not available; `plan` keeps showing the "
      "finished route",
      __func__, this->getName().c_str());
    return;
  }
  clear_plan_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
}

template <class ActionT>
void PoseNavigator<ActionT>::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<typename ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  if (!syncai_util::getCurrentPose(
        current_pose, *this->feedback_utils_.tf, this->feedback_utils_.global_frame,
        this->feedback_utils_.robot_frame, this->feedback_utils_.transform_tolerance)) {
    // No feedback and no waypoint tracking this tick (2026-11). A failed
    // lookup leaves current_pose at the identity in the robot's own frame, and
    // everything below read it as the robot standing on the map origin: every
    // waypoint within waypoint_passed_radius of (0, 0) -- often where the map
    // was started, so where routes begin and end -- was dropped from
    // kRemainingGoals as passed and the next replan skipped it, and the
    // feedback showed the robot at the origin with the distance from there.
    // The tree itself is not stopped: the controller and planner judge the
    // pose themselves, and a TF gap of a few ticks should not end a patrol.
    RCLCPP_WARN_THROTTLE(
      this->logger_, *this->clock_, 2000,
      "[PoseNavigator][%s] %s: no robot pose (%s -> %s within %.2f s); waypoints and feedback "
      "not updated until it is back",
      __func__, this->getName().c_str(), this->feedback_utils_.global_frame.c_str(),
      this->feedback_utils_.robot_frame.c_str(), this->feedback_utils_.transform_tolerance);
    return;
  }

  // Named type, not auto: initialised from a dependent expression the variable
  // would be dependent too, and every get<T>() below would need `template`.
  BT::Blackboard::Ptr blackboard = this->bt_action_server_->getBlackboard();
  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // From the robot's place on the path, forward only (feedback_progress_):
    // planar, like the pose it is measured from.
    double distance_remaining = syncai_util::geometry_utils::calculate_path_length(
      current_path, feedback_progress_.update(current_path, current_pose.pose, false));

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
  }

  if constexpr (Traits::kThroughPoses) {
    // Drop the waypoints the robot has passed from the front of what is
    // still ahead (blackboard_keys::kRemainingGoals). Front only, in order:
    // a route that comes back past an earlier waypoint must not skip the
    // ones in between. The last one stays: it is the goal.
    //
    // Passed = within waypoint_passed_radius on x / y (current_pose is the
    // planar map -> base_link) AND on the robot's floor: within
    // waypoint_passed_z_tolerance of its ground height (2026-11). A waypoint's
    // z names its floor (NavigateThroughGoals); on x / y alone, a robot on 1F
    // dropped the 2F waypoint above it, and the replan after skipped it. With
    // no ground height (body_frame's TF missing) nothing is dropped this tick:
    // keeping a waypoint one tick longer is harmless, dropping one is not.
    std::vector<geometry_msgs::msg::PoseStamped> remaining;
    double ground_z = 0.0;
    std::string z_error;
    const bool have_z = syncai_util::lookupGroundZ(
      *this->feedback_utils_.tf, this->feedback_utils_.global_frame, body_frame_, lidar_height_,
      ground_z, &z_error);
    if (!have_z) {
      RCLCPP_WARN_THROTTLE(
        this->logger_, *this->clock_, 2000,
        "[PoseNavigator][%s] %s: no ground height (%s -> %s: %s); no waypoint is marked passed "
        "until it is back",
        __func__, this->getName().c_str(), this->feedback_utils_.global_frame.c_str(),
        body_frame_.c_str(), z_error.c_str());
    }
    if (blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>(
          keys::kRemainingGoals, remaining)) {
      size_t passed = 0;
      while (have_z && remaining.size() - passed > 1 &&
             syncai_util::geometry_utils::euclidean_distance(
               remaining[passed].pose, current_pose.pose) <= waypoint_passed_radius_ &&
             std::abs(remaining[passed].pose.position.z - ground_z) <=
               waypoint_passed_z_tolerance_) {
        ++passed;
      }
      if (passed > 0) {
        for (size_t i = 0; i < passed; ++i) {
          RCLCPP_INFO(
            this->logger_,
            "[PoseNavigator][%s] %s: passed waypoint (%.2f, %.2f, z %.2f) within %.2f m, robot's "
            "ground z %.2f; %zu still ahead",
            __func__, this->getName().c_str(), remaining[i].pose.position.x,
            remaining[i].pose.position.y, remaining[i].pose.position.z, waypoint_passed_radius_,
            ground_z, remaining.size() - i - 1);
        }
        remaining.erase(remaining.begin(), remaining.begin() + static_cast<std::ptrdiff_t>(passed));
        blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(
          keys::kRemainingGoals, remaining);
      }
      feedback_msg->number_of_poses_remaining = static_cast<int16_t>(remaining.size());
    }
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = this->clock_->now() - start_time_;

  if constexpr (Traits::kReportsFailure) {
    // Plans beyond the first = detours, since move.xml plans only when the
    // path is blocked or the goal changed (a goal change resets the count).
    int plans = 0;
    blackboard->get<int>(keys::kNumberPlans, plans);
    feedback_msg->number_of_replans = static_cast<uint16_t>(std::max(plans - 1, 0));

    blackboard->get<std::string>(keys::kNavStatusMsg, feedback_msg->status_msg);
  }

  this->bt_action_server_->publishFeedback(feedback_msg);
}

template <class ActionT>
void PoseNavigator<ActionT>::onPreempt(typename ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    this->logger_, "[PoseNavigator][%s] %s: received goal preemption request", __func__,
    this->getName().c_str());

  if constexpr (Traits::kThroughPoses) {
    // goalReceived()'s check, which a preempt does not pass through.
    if (goal->poses.empty()) {
      RCLCPP_WARN(
        this->logger_,
        "[PoseNavigator][%s] %s: preempting route has no poses; rejected, continuing the current "
        "one",
        __func__, this->getName().c_str());
      this->bt_action_server_->terminatePendingGoal();
      return;
    }
  }

  // The tree the pending goal would run as a new goal (an empty name is the
  // default), so the comparison is like for like.
  const std::string requested = selectBtFilename(goal->behavior_tree);
  if (requested == this->bt_action_server_->getCurrentBTFilename()) {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    initializeGoalPose(this->bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      this->logger_,
      "[PoseNavigator][%s] Preemption request was rejected since the requested BT XML "
      "file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.",
      __func__);
    this->bt_action_server_->terminatePendingGoal();
  }
}

template <class ActionT>
void PoseNavigator<ActionT>::initializeGoalPose(typename ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  syncai_util::getCurrentPose(
    current_pose, *this->feedback_utils_.tf, this->feedback_utils_.global_frame,
    this->feedback_utils_.robot_frame, this->feedback_utils_.transform_tolerance);

  if constexpr (Traits::kThroughPoses) {
    RCLCPP_INFO(
      this->logger_,
      "Begin navigating from current location (%.2f, %.2f) through %zu poses to (%.2f, %.2f)",
      current_pose.pose.position.x, current_pose.pose.position.y, goal->poses.size(),
      goal->poses.back().pose.position.x, goal->poses.back().pose.position.y);
  } else {
    RCLCPP_INFO(
      this->logger_, "Begin navigating from current location (%.2f, %.2f) to (%.2f, %.2f)",
      current_pose.pose.position.x, current_pose.pose.position.y, goal->pose.pose.position.x,
      goal->pose.pose.position.y);
  }

  // Reset state for new action feedback. A preempt comes through here too, so
  // the superseding goal starts with a clean attribution, its first plan is
  // not counted as a replan, and its status says it is planning.
  start_time_ = this->clock_->now();
  BT::Blackboard::Ptr blackboard = this->bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT
  blackboard->set<std::string>(keys::kFailedNode, "");
  blackboard->set<std::string>(keys::kFailureMsg, "");
  blackboard->set<uint16_t>(keys::kFailureCode, 0);
  blackboard->set<int>(keys::kNumberPlans, 0);
  // A new goal, so FollowPath no longer counts the controller's success on a
  // path sent before this as the goal reached (blackboard_keys.hpp).
  blackboard->set<int>(keys::kGoalGeneration, ++goal_generation_);
  // Seeded rather than cleared: the tree's first write is the first path
  // landing, and until then planning is what the robot is doing.
  blackboard->set<std::string>(keys::kNavStatusMsg, keys::kStatusPlanning);

  // Update the goal pose on the blackboard: the route's waypoints for
  // navigate_through_poses, both the client's list and the copy the
  // navigator drops passed waypoints from (blackboard_keys.hpp).
  if constexpr (Traits::kThroughPoses) {
    blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(keys::kGoals, goal->poses);
    blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(
      keys::kRemainingGoals, goal->poses);
  } else {
    blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
  }
  feedback_progress_.reset();

  // Drop the previous goal's path. move.xml keeps {path} for as long as
  // IsPathValid accepts it (the 2D tree) or for the whole goal (the Octo3D
  // tree, which plans once per goal), and nothing else clears it between goals
  // (haltAllActions() only halts nodes still RUNNING, so ComputePathToPose's
  // halt() rarely runs). A stale path that is still free would otherwise be
  // followed from wherever it starts when the same goal is sent again. On a
  // preempt FollowPath ignores the empty path and keeps driving the one it
  // has until the replan lands; should that one finish first, the goal
  // generation above keeps it from succeeding the new goal.
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
}

template <class ActionT>
void PoseNavigator<ActionT>::onGoalPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // Instantiated for every action, subscribed for one (Traits::kDrivesRvizGoalPose).
  if constexpr (Traits::kDrivesRvizGoalPose) {
    if (!self_client_) {
      return;
    }

    typename ActionT::Goal goal;
    goal.pose = *pose;

    // subscribe the goal pose topic and send the goal to the action server
    self_client_->async_send_goal(goal);
  } else {
    (void)pose;
  }
}

template class PoseNavigator<nav2_msgs::action::NavigateToPose>;
template class PoseNavigator<syncai_common::action::NavigateToGoal>;
template class PoseNavigator<nav2_msgs::action::NavigateThroughPoses>;
template class PoseNavigator<syncai_common::action::NavigateThroughGoals>;

}  // namespace syncai_task_runner
