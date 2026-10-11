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

// Failure attribution, written by BtActionNode::report_failure() and reset by
// the navigator before every goal. Last writer wins, and that names the
// branch that failed the tree because PipelineSequence returns at the first
// FAILURE it meets (move.xml's header has the case analysis). kFailedNode is
// the node's registration name ("ComputePathToPose", "FollowPath"), or ""
// when the failure cannot be attributed to a stage (the server never
// acknowledged the goal); kFailureMsg is the detail in either case.
inline constexpr char kFailedNode[] = "failed_node";
inline constexpr char kFailureMsg[] = "failure_msg";

// Plans completed for the current goal: incremented by ComputePathToPose on
// success, reset by the navigator per goal, reported as plans - 1 (the
// replans, i.e. detours, since the tree plans only when the path is blocked).
inline constexpr char kNumberPlans[] = "number_plans";

// What the navigator is doing right now, as one English sentence for the
// operator console (NavigateToGoal feedback `status_msg`). Written at each
// turn of move.xml -- IsPathValid / GlobalUpdatedGoal when a replan starts,
// ComputePathToPose when a path lands or the planner gives up, FollowPath when
// the controller gives up or a retry resumes -- and set to the planning
// sentence by the navigator at every goal. Last writer wins, and that is the
// right answer for the same reason as kFailedNode: the node that wrote last is
// the one whose outcome the robot is now acting on. The ClearEntireCostmap
// nodes stay silent: they only run because an action node just failed, and
// that node's sentence already says the costmap is being cleared. Free text
// on the wire; nothing may branch on it.
inline constexpr char kNavStatusMsg[] = "nav_status_msg";

// The sentences more than one site needs: the navigator seeds the first, and
// FollowPath writes the second on a resume only after its own gave-up
// sentence, so it can compare (see FollowPathAction::on_tick).
inline constexpr char kStatusPlanning[] = "Planning a path to the goal";
inline constexpr char kStatusFollowing[] = "Following the path";

}  // namespace blackboard_keys
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BLACKBOARD_KEYS_HPP_
