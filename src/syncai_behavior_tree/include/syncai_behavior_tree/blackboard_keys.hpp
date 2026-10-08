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

}  // namespace blackboard_keys
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BLACKBOARD_KEYS_HPP_
