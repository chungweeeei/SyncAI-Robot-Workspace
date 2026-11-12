#ifndef SYNCAI_BEHAVIOR_TREE__BLACKBOARD_KEYS_HPP_
#define SYNCAI_BEHAVIOR_TREE__BLACKBOARD_KEYS_HPP_

namespace syncai_behavior_tree
{
namespace blackboard_keys
{

// Keys shared between the BT nodes that write them and the navigator (in
// syncai_task_runner) that reads them. Both sides include this header so a
// typo is a compile error, which the string literal `number_recoveries`
// (four sites, predating this header) does not give. The blackboard is the
// only channel from a BT node to the navigator: no logger is wired into the
// engine, and an output port would need every tree to wire it in XML.

// Failure attribution, written by BtActionNode::report_failure() (and, since
// 2026-10, BtServiceNode's copy of it, plus Wait by hand) and reset by the
// navigator before every goal. Last writer wins, and that names the branch
// that failed the tree because PipelineSequence returns at the first FAILURE
// it meets (move.xml's header has the case analysis; in a tree that restores
// the gait after a failed step, a refused restore is the last writer on
// purpose). kFailedNode is the node's registration name
// ("ComputePathToPose", "FollowPath", "SetMotionKey", "WaitForMotionState"),
// or "" when the failure cannot be attributed to a
// stage (the server never acknowledged the goal, the service never
// answered); kFailureMsg is the detail in either case.
inline constexpr char kFailedNode[] = "failed_node";
inline constexpr char kFailureMsg[] = "failure_msg";
// The failed action's own error_code (uint16_t), when its result carries
// one: since 2026-11 the planner's (syncai_common/ComputePathToPose, 201..),
// which the navigator copies into NavigateToGoal / NavigateThroughGoals in
// place of its node's 200. 0 = none, the node's base applies. Written by
// every report_failure() call -- 0 when it has no code -- so it always
// belongs to the same failure as kFailedNode; a node that writes those two
// by hand does not write it, and the navigator reads it only for the
// planner nodes.
inline constexpr char kFailureCode[] = "failure_code";

// Plans completed for the current goal: incremented by ComputePathToPose on
// success, reset by the navigator per goal, reported as plans - 1 (the
// replans, i.e. detours, since the tree plans only when the path is blocked).
inline constexpr char kNumberPlans[] = "number_plans";

// What the navigator is doing right now, as one English sentence for the
// operator console (NavigateToGoal feedback `status_msg`). Written at each
// turn of move.xml -- IsPathValid / GlobalUpdatedGoal when a replan starts,
// ComputePathToPose when a path lands or the planner gives up, FollowPath when
// the controller gives up or a retry resumes -- and set to the planning
// sentence by the navigator at every goal. A tree that switches the gait adds
// its own turns with SetBlackboard ("Switching the gait to MPC" -- written by
// the tree, since SetMotionKey is a generic node that does not know why it
// is sending a key; dock.xml / undock.xml did, until 2026-11-12). Last writer
// wins, and that is the right answer for the same reason as kFailedNode: the
// node that wrote last is the one whose outcome the robot is now acting on.
// The ClearEntireCostmap nodes stay silent: they only run because an action
// node just failed, and that node's sentence already says the costmap is
// being cleared. Free text on the wire; nothing may branch on it.
inline constexpr char kNavStatusMsg[] = "nav_status_msg";

// The sentences more than one site needs: the navigator seeds the first, and
// FollowPath writes the second on a resume only after its own gave-up
// sentence, so it can compare (see FollowPathAction::on_tick).
inline constexpr char kStatusPlanning[] = "Planning a path to the goal";
inline constexpr char kStatusFollowing[] = "Following the path";

// Which goal the tree is running (int), bumped by the navigator at every goal
// and every accepted preempt (2026-11). FollowPath remembers the generation
// of the path it last sent and does not take the controller's success for a
// path sent under an older one: after a preempt {path} is empty until the
// new goal's plan lands, FollowPath keeps driving the old path meanwhile, and
// reaching the old goal in that window succeeded the tree -- reported as the
// new goal reached, at the old goal's place (FollowPathAction::on_success).
inline constexpr char kGoalGeneration[] = "goal_generation";

// The patrol route's waypoints (navigate_through_poses, patrol.xml, 2026-11).
// kGoals is the client's list as sent, rewritten only by a new goal or a
// preempt, so GlobalUpdatedGoal sees a change of route and nothing else.
// kRemainingGoals is what is still ahead, the list ComputePathThroughPoses
// plans through: the navigator drops a waypoint from its front once the robot
// has come within waypoint_passed_radius of it (never the last), so a replan
// mid-route starts from the next waypoint instead of driving back to one
// already passed. Dropping it here and not in the tree (nav2's
// RemovePassedGoals) checks it at the navigator's loop rate rather than the
// planner branch's 1 Hz, and does not need a TF buffer on the blackboard.
inline constexpr char kGoals[] = "goals";
inline constexpr char kRemainingGoals[] = "remaining_goals";

}  // namespace blackboard_keys
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BLACKBOARD_KEYS_HPP_
